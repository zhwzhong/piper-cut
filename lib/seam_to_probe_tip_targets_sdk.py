#!/usr/bin/env python3
"""Convert an RGB-D box seam into PIPER probe-tip targets (no motion).

The script bridges four independently calibrated quantities:

1. seam pixel endpoints from ``detect_center_seam.py``;
2. registered depth and the RGB camera factory intrinsics;
3. eye-to-hand ``T_base_camera``;
4. the translation from PIPER link6/flange to the physical probe tip.

The seam points transformed into ``base_link`` are already the physical
probe-tip contact targets.  If a fixed flange orientation is supplied, this
script additionally computes the flange origins required to place the probe
tip at those targets:

    p_base_tip = p_base_flange + R_base_flange @ t_flange_tip

No PIPER SDK object is constructed and no CAN or motion command is sent.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


CAMERA_FRAME_DEFAULT = "gemini336l_color_optical_frame"
BASE_FRAME_DEFAULT = "base_link"


def load_mapping(path: Path) -> dict[str, Any]:
    with path.expanduser().open("r", encoding="utf-8") as stream:
        if path.suffix.lower() == ".json":
            value = json.load(stream)
        else:
            value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a YAML/JSON mapping")
    return value


def finite_vector(value: Any, size: int, label: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64).reshape(-1)
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain {size} finite values")
    return array


def extract_seam_pixels(document: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    status = document.get("status")
    if status not in (None, "ok", "prepared_no_motion"):
        raise ValueError(f"seam detector status is {status!r}: {document.get('error')}")

    result = document.get("result")
    if not isinstance(result, dict):
        result = document
    result_status = result.get("status")
    if result_status not in (None, "ok"):
        raise ValueError(f"seam result status is {result_status!r}")

    line = result.get("rgb_line")
    if not isinstance(line, dict):
        detection = document.get("seam_detection")
        line = detection.get("rgb_line_pixel") if isinstance(detection, dict) else None
    if not isinstance(line, dict):
        raise ValueError("seam YAML has no rgb_line start_px/end_px")

    start = finite_vector(line.get("start_px"), 2, "seam start_px")
    end = finite_vector(line.get("end_px"), 2, "seam end_px")
    if float(np.linalg.norm(end - start)) < 5.0:
        raise ValueError("detected seam is shorter than 5 pixels")
    quality = result.get("quality") if isinstance(result.get("quality"), dict) else {}
    return start, end, quality


def extract_intrinsics(document: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, str, dict[str, Any]]:
    candidates: list[dict[str, Any]] = [document]
    for key in ("color_camera_info", "intrinsics"):
        value = document.get(key)
        if isinstance(value, dict):
            candidates.insert(0, value)
    input_block = document.get("input")
    if isinstance(input_block, dict) and isinstance(input_block.get("intrinsics"), dict):
        candidates.insert(0, input_block["intrinsics"])

    selected = None
    for candidate in candidates:
        if candidate.get("K") is not None or candidate.get("k") is not None:
            selected = candidate
            break
    if selected is None:
        raise ValueError("camera-info YAML has no K/k matrix")

    camera_matrix = finite_vector(selected.get("K", selected.get("k")), 9, "camera K").reshape(3, 3)
    if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    distortion_value = selected.get("D", selected.get("d", []))
    distortion = np.asarray(distortion_value, dtype=np.float64).reshape(-1)
    if distortion.size and not np.all(np.isfinite(distortion)):
        raise ValueError("camera D contains non-finite values")
    frame_id = str(
        selected.get("frame_id")
        or document.get("color_frame_id")
        or document.get("frame_id")
        or CAMERA_FRAME_DEFAULT
    )
    metadata = {
        "width": int(selected.get("width", document.get("width", 0)) or 0),
        "height": int(selected.get("height", document.get("height", 0)) or 0),
        "serial_number": str(
            document.get("device", {}).get("serial_number", "")
            if isinstance(document.get("device"), dict)
            else ""
        ),
    }
    return camera_matrix, distortion, frame_id, metadata


def load_transform(document: dict[str, Any]) -> tuple[np.ndarray, str, str]:
    block = document.get("T_base_camera")
    if not isinstance(block, dict):
        raise ValueError("extrinsics YAML has no T_base_camera mapping")
    transform = np.asarray(block.get("matrix_4x4"), dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("T_base_camera.matrix_4x4 must be a finite 4x4 matrix")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-9):
        raise ValueError("T_base_camera bottom row must be [0,0,0,1]")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-6):
        raise ValueError("T_base_camera rotation is not orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-6):
        raise ValueError("T_base_camera rotation determinant is not +1")

    frames = document.get("frames") if isinstance(document.get("frames"), dict) else {}
    camera_frame = str(frames.get("camera") or CAMERA_FRAME_DEFAULT)
    base_frame = str(frames.get("robot_base") or BASE_FRAME_DEFAULT)
    return transform, camera_frame, base_frame


def load_tcp(document: dict[str, Any]) -> tuple[np.ndarray, str]:
    candidates = [document]
    for key in ("result", "recommended_candidate"):
        value = document.get(key)
        if isinstance(value, dict):
            candidates.insert(0, value)
    block = next(
        (candidate for candidate in candidates if candidate.get("tcp_offset_m_rad") is not None),
        None,
    )
    if block is None:
        raise ValueError("TCP calibration has no tcp_offset_m_rad")
    offset = finite_vector(block["tcp_offset_m_rad"], 6, "tcp_offset_m_rad")
    if not np.allclose(offset[3:], 0.0, atol=1.0e-12):
        raise ValueError("only the calibrated translation-only PIPER TCP convention is supported")
    status = str(block.get("status") or document.get("status") or "unknown")
    return offset, status


def validate_calibration_bundle(
    document: dict[str, Any] | None,
    *,
    intrinsics_frame: str,
    base_frame: str,
    camera_serial: str,
) -> dict[str, Any]:
    if document is None:
        return {
            "supplied": False,
            "accepted": None,
            "warning": "calibration bundle was not supplied; individual files were validated structurally only",
        }
    status = str(document.get("status", ""))
    if status != "calibrated_and_validated":
        raise ValueError(
            f"calibration bundle status must be 'calibrated_and_validated', got {status!r}"
        )
    runtime = document.get("runtime_geometry")
    if not isinstance(runtime, dict):
        raise ValueError("calibration bundle has no runtime_geometry")
    source_frame = str(runtime.get("source_frame", ""))
    target_frame = str(runtime.get("target_frame", ""))
    if source_frame != intrinsics_frame or target_frame != base_frame:
        raise ValueError(
            "calibration bundle frame mismatch: "
            f"{source_frame!r}->{target_frame!r}, expected "
            f"{intrinsics_frame!r}->{base_frame!r}"
        )
    hardware = document.get("hardware") if isinstance(document.get("hardware"), dict) else {}
    bundle_serial = str(hardware.get("camera_serial", ""))
    if camera_serial and bundle_serial and camera_serial != bundle_serial:
        raise ValueError(
            f"camera serial mismatch: frame={camera_serial!r}, bundle={bundle_serial!r}"
        )
    acceptance = document.get("acceptance")
    if not isinstance(acceptance, dict):
        raise ValueError("calibration bundle has no acceptance mapping")
    tcp_check_names = (
        "tcp_calibration_32_points",
        "tcp_selected_12_points",
    )
    selected_tcp_check = next(
        (
            name
            for name in tcp_check_names
            if isinstance(acceptance.get(name), dict)
            and acceptance[name].get("passed") is True
        ),
        None,
    )
    if selected_tcp_check is None:
        raise ValueError(
            "calibration bundle has no passed TCP calibration check; expected one of "
            f"{tcp_check_names}"
        )
    required_checks = (
        "eye_to_hand_fit_12_points",
        "independent_probe_validation_6_points",
        "live_rgbd_vs_probe_validation_6_points",
    )
    passed: dict[str, bool] = {selected_tcp_check: True}
    for name in required_checks:
        block = acceptance.get(name)
        if not isinstance(block, dict) or block.get("passed") is not True:
            raise ValueError(f"calibration bundle check {name!r} is not passed")
        passed[name] = True
    return {
        "supplied": True,
        "accepted": True,
        "bundle_status": status,
        "camera_serial": bundle_serial or camera_serial,
        "source_frame": source_frame,
        "target_frame": target_frame,
        "passed_checks": passed,
        "warning": None,
    }


def robust_patch_depth_mm(depth: np.ndarray, u: float, v: float, radius: int) -> float | None:
    center_u = int(round(float(u)))
    center_v = int(round(float(v)))
    y0, y1 = max(0, center_v - radius), min(depth.shape[0], center_v + radius + 1)
    x0, x1 = max(0, center_u - radius), min(depth.shape[1], center_u + radius + 1)
    values = depth[y0:y1, x0:x1].astype(np.float64).reshape(-1)
    values = values[(values > 100.0) & (values < 2000.0)]
    if values.size < max(5, (2 * radius + 1) ** 2 // 4):
        return None
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    threshold = max(2.0, 3.0 * 1.4826 * mad)
    inliers = values[np.abs(values - median) <= threshold]
    if inliers.size < 5:
        return None
    return float(np.median(inliers))


def fit_seam_depth(
    depth: np.ndarray,
    start_px: np.ndarray,
    end_px: np.ndarray,
    *,
    patch_radius: int,
    sample_count: int,
    inset_ratio: float,
) -> dict[str, Any]:
    parameters = np.linspace(inset_ratio, 1.0 - inset_ratio, sample_count)
    valid_t: list[float] = []
    valid_z: list[float] = []
    for parameter in parameters:
        pixel = start_px * (1.0 - parameter) + end_px * parameter
        value = robust_patch_depth_mm(depth, pixel[0], pixel[1], patch_radius)
        if value is not None:
            valid_t.append(float(parameter))
            valid_z.append(value)
    if len(valid_t) < max(12, sample_count // 2):
        raise ValueError(
            f"only {len(valid_t)}/{sample_count} valid internal seam depth samples"
        )

    t = np.asarray(valid_t, dtype=np.float64)
    z = np.asarray(valid_z, dtype=np.float64)
    keep = np.ones(len(t), dtype=bool)
    coefficients = np.zeros(2, dtype=np.float64)
    residuals = np.zeros(len(t), dtype=np.float64)
    for _ in range(8):
        coefficients = np.linalg.lstsq(
            np.column_stack([t[keep], np.ones(np.count_nonzero(keep))]),
            z[keep],
            rcond=None,
        )[0]
        residuals = z - (coefficients[0] * t + coefficients[1])
        center = float(np.median(residuals[keep]))
        mad = float(np.median(np.abs(residuals[keep] - center)))
        threshold = max(2.0, 3.0 * 1.4826 * mad)
        updated = np.abs(residuals - center) <= threshold
        if np.array_equal(updated, keep):
            break
        if np.count_nonzero(updated) < 12:
            break
        keep = updated

    start_depth = float(coefficients[1])
    end_depth = float(coefficients[0] + coefficients[1])
    inlier_residuals = residuals[keep]
    return {
        "start_raw_mm": start_depth,
        "end_raw_mm": end_depth,
        "sample_count": int(sample_count),
        "valid_sample_count": int(len(t)),
        "inlier_sample_count": int(np.count_nonzero(keep)),
        "fit_slope_mm_per_line": float(coefficients[0]),
        "fit_rms_mm": float(np.sqrt(np.mean(np.square(inlier_residuals)))),
        "fit_median_abs_mm": float(np.median(np.abs(inlier_residuals))),
        "inset_ratio": float(inset_ratio),
        "patch_radius_px": int(patch_radius),
    }


def depth_correction_decision(
    document: dict[str, Any] | None,
    raw_depths_mm: np.ndarray,
    mode: str,
) -> tuple[float, dict[str, Any]]:
    if document is None or mode == "off":
        return 0.0, {
            "mode": mode,
            "applied": False,
            "add_to_raw_depth_mm": 0.0,
            "reason": "disabled_or_not_supplied",
        }
    correction = document.get("correction")
    if not isinstance(correction, dict):
        correction = document
    add_mm = float(correction.get("add_to_raw_depth_mm", 0.0))
    if not math.isfinite(add_mm):
        raise ValueError("depth correction is not finite")
    valid_range = correction.get("validated_expected_depth_range_mm")
    parsed_range = None
    if isinstance(valid_range, (list, tuple)) and len(valid_range) == 2:
        parsed_range = [float(valid_range[0]), float(valid_range[1])]
    inside = bool(
        parsed_range is not None
        and np.all(raw_depths_mm >= parsed_range[0])
        and np.all(raw_depths_mm <= parsed_range[1])
    )
    apply = mode == "force" or (mode == "auto" and inside)
    reason = "forced" if mode == "force" else (
        "inside_validated_range" if inside else "outside_validated_range"
    )
    return (add_mm if apply else 0.0), {
        "mode": mode,
        "applied": bool(apply),
        "add_to_raw_depth_mm": float(add_mm if apply else 0.0),
        "available_add_to_raw_depth_mm": add_mm,
        "validated_expected_depth_range_mm": parsed_range,
        "raw_endpoint_depths_mm": [float(value) for value in raw_depths_mm],
        "reason": reason,
        "warning": (
            None
            if inside or mode == "off"
            else "correction was calibrated at another depth; auto mode did not extrapolate"
        ),
    }


def deproject_pixel(
    pixel: np.ndarray,
    depth_mm: float,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    normalized = cv2.undistortPoints(
        np.asarray(pixel, dtype=np.float64).reshape(1, 1, 2),
        camera_matrix,
        distortion if distortion.size else None,
    )[0, 0]
    z_m = float(depth_mm) * 0.001
    return np.asarray([normalized[0] * z_m, normalized[1] * z_m, z_m])


def transform_point(transform: np.ndarray, point: np.ndarray) -> np.ndarray:
    homogeneous = np.append(np.asarray(point, dtype=np.float64), 1.0)
    return (transform @ homogeneous)[:3]


def line_geometry(start: np.ndarray, end: np.ndarray, frame_id: str) -> dict[str, Any]:
    vector = np.asarray(end) - np.asarray(start)
    length = float(np.linalg.norm(vector))
    if length <= 1.0e-9:
        raise ValueError("3D seam has zero length")
    midpoint = (np.asarray(start) + np.asarray(end)) * 0.5
    return {
        "frame_id": frame_id,
        "unit": "meter",
        "start_m": [float(value) for value in start],
        "end_m": [float(value) for value in end],
        "mid_m": [float(value) for value in midpoint],
        "direction": [float(value) for value in vector / length],
        "length_m": length,
        "delta_z_m": float(vector[2]),
        "start_mm": [float(value * 1000.0) for value in start],
        "end_mm": [float(value * 1000.0) for value in end],
    }


def apply_tip_z_min(
    start: np.ndarray,
    end: np.ndarray,
    z_min_mm: float | None,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    if z_min_mm is None:
        return start, end, {"enabled": False}
    if not math.isfinite(float(z_min_mm)):
        raise ValueError("--target-z-min-mm fixed Z value must be finite")
    z_min_m = float(z_min_mm) * 0.001
    adjusted_start = np.asarray(start, dtype=np.float64).copy()
    adjusted_end = np.asarray(end, dtype=np.float64).copy()
    original_z_mm = [float(adjusted_start[2] * 1000.0), float(adjusted_end[2] * 1000.0)]
    adjusted_start[2] = z_min_m
    adjusted_end[2] = z_min_m
    adjusted_z_mm = [float(adjusted_start[2] * 1000.0), float(adjusted_end[2] * 1000.0)]
    return adjusted_start, adjusted_end, {
        "enabled": True,
        "mode": "fixed_z",
        "fixed_z_mm": float(z_min_mm),
        "z_min_mm": float(z_min_mm),
        "original_start_end_z_mm": original_z_mm,
        "adjusted_start_end_z_mm": adjusted_z_mm,
        "applied": any(abs(a - b) > 1.0e-9 for a, b in zip(original_z_mm, adjusted_z_mm)),
    }


def rotation_from_piper_rpy_deg(values: np.ndarray) -> np.ndarray:
    rx, ry, rz = np.deg2rad(values)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.asarray(
        [
            [cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
            [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
            [-sy, sx * cy, cx * cy],
        ]
    )


def rotation_from_quaternion_xyzw(values: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(values))
    if norm < 1.0e-12:
        raise ValueError("flange quaternion has zero length")
    x, y, z, w = values / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def piper_rpy_deg_from_rotation(rotation: np.ndarray) -> list[float]:
    sy = -float(rotation[2, 0])
    sy = max(-1.0, min(1.0, sy))
    ry = math.asin(sy)
    if abs(math.cos(ry)) > 1.0e-8:
        rx = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        rz = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        rx = 0.0
        rz = math.atan2(-float(rotation[0, 1]), float(rotation[1, 1]))
    return [math.degrees(rx), math.degrees(ry), math.degrees(rz)]


def quaternion_xyzw_from_rotation(rotation: np.ndarray) -> list[float]:
    # OpenCV returns [x, y, z] Rodrigues vectors, not quaternions, so keep the
    # conversion explicit and independent of optional scipy.
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        values = [
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ]
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            values = [0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale,
                      (rotation[0, 2] + rotation[2, 0]) / scale,
                      (rotation[2, 1] - rotation[1, 2]) / scale]
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            values = [(rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                      (rotation[1, 2] + rotation[2, 1]) / scale,
                      (rotation[0, 2] - rotation[2, 0]) / scale]
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            values = [(rotation[0, 2] + rotation[2, 0]) / scale,
                      (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale,
                      (rotation[1, 0] - rotation[0, 1]) / scale]
    quaternion = np.asarray(values, dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return [float(value) for value in quaternion]


def flange_targets(
    tip_start: np.ndarray,
    tip_end: np.ndarray,
    tcp_offset_m_rad: np.ndarray,
    rotation_base_flange: np.ndarray,
    base_frame: str,
) -> dict[str, Any]:
    offset_base = rotation_base_flange @ tcp_offset_m_rad[:3]
    flange_start = tip_start - offset_base
    flange_end = tip_end - offset_base
    roundtrip_start = flange_start + offset_base
    roundtrip_end = flange_end + offset_base
    roundtrip_error = max(
        float(np.linalg.norm(roundtrip_start - tip_start)),
        float(np.linalg.norm(roundtrip_end - tip_end)),
    )
    if roundtrip_error > 1.0e-12:
        raise RuntimeError(
            f"TCP flange/tip round-trip check failed: {roundtrip_error:.3e} m"
        )
    line = line_geometry(flange_start, flange_end, base_frame)
    line["orientation_quaternion_xyzw"] = quaternion_xyzw_from_rotation(rotation_base_flange)
    line["orientation_piper_rpy_deg"] = piper_rpy_deg_from_rotation(rotation_base_flange)
    line["tcp_offset_rotated_into_base_m"] = [float(value) for value in offset_base]
    line["tcp_roundtrip_max_error_m"] = roundtrip_error
    return line


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seam-yaml", type=Path, required=True)
    parser.add_argument("--depth", type=Path, required=True, help="Registered uint16 depth PNG in mm")
    parser.add_argument("--camera-info", type=Path, required=True)
    parser.add_argument("--extrinsics", type=Path, required=True)
    parser.add_argument("--tcp-calibration", type=Path, required=True)
    parser.add_argument(
        "--calibration-bundle",
        type=Path,
        help="Recommended manifest proving that the selected TCP and eye-to-hand result passed validation",
    )
    parser.add_argument("--depth-correction", type=Path)
    parser.add_argument(
        "--depth-correction-mode",
        choices=("off", "auto", "force"),
        default="auto",
        help="auto applies the correction only inside its validated depth range",
    )
    parser.add_argument("--patch-radius", type=int, default=4)
    parser.add_argument("--depth-samples", type=int, default=81)
    parser.add_argument("--depth-inset-ratio", type=float, default=0.08)
    parser.add_argument(
        "--target-z-min-mm",
        type=float,
        help="Fixed Z for generated probe-tip start/end targets in base_link, in mm",
    )
    orientation = parser.add_mutually_exclusive_group()
    orientation.add_argument(
        "--flange-rpy-deg",
        nargs=3,
        type=float,
        metavar=("RX", "RY", "RZ"),
        help="Fixed PIPER flange orientation using Rz@Ry@Rx, in degrees",
    )
    orientation.add_argument(
        "--flange-quaternion-xyzw",
        nargs=4,
        type=float,
        metavar=("QX", "QY", "QZ", "QW"),
    )
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.patch_radius < 1 or args.patch_radius > 20:
        raise ValueError("--patch-radius must be in [1,20]")
    if args.depth_samples < 21:
        raise ValueError("--depth-samples must be at least 21")
    if not 0.0 <= args.depth_inset_ratio < 0.45:
        raise ValueError("--depth-inset-ratio must be in [0,0.45)")

    seam_document = load_mapping(args.seam_yaml)
    camera_document = load_mapping(args.camera_info)
    extrinsics_document = load_mapping(args.extrinsics)
    tcp_document = load_mapping(args.tcp_calibration)
    bundle_document = load_mapping(args.calibration_bundle) if args.calibration_bundle else None
    correction_document = load_mapping(args.depth_correction) if args.depth_correction else None

    start_px, end_px, detector_quality = extract_seam_pixels(seam_document)
    camera_matrix, distortion, intrinsics_frame, camera_metadata = extract_intrinsics(camera_document)
    transform_base_camera, extrinsics_camera_frame, base_frame = load_transform(extrinsics_document)
    tcp_offset, tcp_status = load_tcp(tcp_document)
    if intrinsics_frame != extrinsics_camera_frame:
        raise ValueError(
            f"camera frame mismatch: intrinsics={intrinsics_frame!r}, "
            f"extrinsics={extrinsics_camera_frame!r}"
        )
    calibration_acceptance = validate_calibration_bundle(
        bundle_document,
        intrinsics_frame=intrinsics_frame,
        base_frame=base_frame,
        camera_serial=camera_metadata["serial_number"],
    )

    depth = cv2.imread(str(args.depth.expanduser()), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise FileNotFoundError(args.depth)
    if depth.ndim != 2 or depth.dtype != np.uint16:
        raise ValueError(f"registered depth must be uint16 single-channel PNG, got {depth.dtype} {depth.shape}")
    height, width = depth.shape
    for label, pixel in (("start", start_px), ("end", end_px)):
        if not (0.0 <= pixel[0] < width and 0.0 <= pixel[1] < height):
            raise ValueError(f"seam {label} pixel {pixel.tolist()} is outside {width}x{height}")
    if camera_metadata["width"] not in (0, width) or camera_metadata["height"] not in (0, height):
        raise ValueError(
            f"depth image {width}x{height} does not match camera info "
            f"{camera_metadata['width']}x{camera_metadata['height']}"
        )

    depth_fit = fit_seam_depth(
        depth,
        start_px,
        end_px,
        patch_radius=args.patch_radius,
        sample_count=args.depth_samples,
        inset_ratio=args.depth_inset_ratio,
    )
    raw_depths = np.asarray([depth_fit["start_raw_mm"], depth_fit["end_raw_mm"]])
    add_depth_mm, correction_record = depth_correction_decision(
        correction_document, raw_depths, args.depth_correction_mode
    )
    corrected_depths = raw_depths + add_depth_mm
    depth_fit["start_used_mm"] = float(corrected_depths[0])
    depth_fit["end_used_mm"] = float(corrected_depths[1])

    start_camera = deproject_pixel(start_px, corrected_depths[0], camera_matrix, distortion)
    end_camera = deproject_pixel(end_px, corrected_depths[1], camera_matrix, distortion)
    raw_start_tip_base = transform_point(transform_base_camera, start_camera)
    raw_end_tip_base = transform_point(transform_base_camera, end_camera)
    start_tip_base, end_tip_base, target_z_min = apply_tip_z_min(
        raw_start_tip_base,
        raw_end_tip_base,
        args.target_z_min_mm,
    )
    camera_line = line_geometry(start_camera, end_camera, intrinsics_frame)
    raw_tip_line = line_geometry(raw_start_tip_base, raw_end_tip_base, base_frame)
    tip_line = line_geometry(start_tip_base, end_tip_base, base_frame)
    rigid_length_error_m = abs(camera_line["length_m"] - raw_tip_line["length_m"])
    if rigid_length_error_m > 1.0e-9:
        raise RuntimeError(
            f"rigid transform changed seam length by {rigid_length_error_m:.3e} m"
        )

    orientation_source = None
    rotation = None
    if args.flange_rpy_deg is not None:
        orientation_source = "flange_rpy_deg"
        rotation = rotation_from_piper_rpy_deg(finite_vector(args.flange_rpy_deg, 3, "flange RPY"))
    elif args.flange_quaternion_xyzw is not None:
        orientation_source = "flange_quaternion_xyzw"
        rotation = rotation_from_quaternion_xyzw(
            finite_vector(args.flange_quaternion_xyzw, 4, "flange quaternion")
        )
    flange_line = (
        flange_targets(start_tip_base, end_tip_base, tcp_offset, rotation, base_frame)
        if rotation is not None
        else None
    )

    detector_confidence = detector_quality.get("line_refine_confidence")
    output_document: dict[str, Any] = {
        "schema_version": 1,
        "status": "prepared_no_motion",
        "motion_command_sent": False,
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "inputs": {
            "seam_yaml": str(args.seam_yaml.expanduser().resolve()),
            "registered_depth_png": str(args.depth.expanduser().resolve()),
            "camera_info": str(args.camera_info.expanduser().resolve()),
            "eye_to_hand_extrinsics": str(args.extrinsics.expanduser().resolve()),
            "probe_tcp_calibration": str(args.tcp_calibration.expanduser().resolve()),
            "calibration_bundle": (
                str(args.calibration_bundle.expanduser().resolve())
                if args.calibration_bundle
                else None
            ),
            "depth_correction": (
                str(args.depth_correction.expanduser().resolve()) if args.depth_correction else None
            ),
        },
        "frames": {
            "camera": intrinsics_frame,
            "robot_base": base_frame,
            "transform_formula": "p_base = T_base_camera @ p_camera",
        },
        "seam_pixels": {
            "frame_id": "rgb_image",
            "unit": "pixel",
            "start_uv": [float(value) for value in start_px],
            "end_uv": [float(value) for value in end_px],
            "mid_uv": [float(value) for value in (start_px + end_px) * 0.5],
            "detector_line_refine_confidence": (
                float(detector_confidence) if detector_confidence is not None else None
            ),
        },
        "depth_estimation": depth_fit,
        "depth_correction": correction_record,
        "calibration_acceptance": calibration_acceptance,
        "quality_checks": {
            "rigid_transform_length_error_m": rigid_length_error_m,
            "depth_fit_rms_mm": depth_fit["fit_rms_mm"],
            "depth_inlier_fraction": (
                depth_fit["inlier_sample_count"] / depth_fit["valid_sample_count"]
            ),
            "calibration_bundle_accepted": calibration_acceptance.get("accepted"),
        },
        "camera_seam_line": camera_line,
        "raw_probe_tip_contact_targets_base": raw_tip_line,
        "target_z_min": target_z_min,
        "target_fixed_z": target_z_min,
        "probe_tip_contact_targets_base": tip_line,
        "probe_tcp": {
            "convention": "p_base_tip = p_base_flange + R_base_flange @ t_flange_tip",
            "tcp_offset_m_rad": [float(value) for value in tcp_offset],
            "translation_flange_to_tip_m": [float(value) for value in tcp_offset[:3]],
            "source_status": tcp_status,
            "accepted_by_calibration_bundle": calibration_acceptance.get("accepted"),
        },
        "flange_pose_targets_base": flange_line,
        "orientation_source": orientation_source,
        "integration_notes": [
            "probe_tip_contact_targets_base are the physical seam endpoints for a TCP-aware IK solver",
            "do not subtract tcp_offset again when the IK solver already targets the configured probe TCP",
            "if the IK solver targets link6/flange, supply a flange orientation and use flange_pose_targets_base",
            "this output contains geometry only and does not authorize robot motion",
        ],
    }
    if flange_line is None:
        output_document["integration_notes"].append(
            "flange targets are omitted because no flange orientation was supplied"
        )
    if correction_record.get("warning"):
        output_document["integration_notes"].append(str(correction_record["warning"]))
    if calibration_acceptance.get("warning"):
        output_document["integration_notes"].append(
            str(calibration_acceptance["warning"])
        )

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        json.dump(output_document, stream, ensure_ascii=False, indent=2)
        stream.write("\n")

    print("=== Seam -> PIPER probe-tip targets (no motion) ===")
    print(f"pixel start/end: {start_px.tolist()} -> {end_px.tolist()}")
    print(
        "depth start/end used [mm]: "
        f"{corrected_depths[0]:.3f} -> {corrected_depths[1]:.3f}; "
        f"fit RMS={depth_fit['fit_rms_mm']:.3f}"
    )
    print(f"tip start base [m]: {[round(float(v), 9) for v in start_tip_base]}")
    print(f"tip end   base [m]: {[round(float(v), 9) for v in end_tip_base]}")
    print(f"seam length [mm]: {tip_line['length_m'] * 1000.0:.3f}")
    if flange_line is None:
        print("flange targets: omitted (supply a fixed flange orientation for IK)")
    else:
        print(f"flange start base [m]: {[round(v, 9) for v in flange_line['start_m']]}")
        print(f"flange end   base [m]: {[round(v, 9) for v in flange_line['end_m']]}")
    print(f"Saved: {output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}")
        raise SystemExit(2)
