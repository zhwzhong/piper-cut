#!/usr/bin/env python3
"""Browser control panel for the SDK-only PIPER box cutting pipeline.

This server is intentionally a thin wrapper around the existing command-line
scripts. It does not modify the detection or robot-control implementation.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import mimetypes
import os
import re
import select
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse


ROOT = Path(__file__).resolve().parents[1]
LIB = ROOT / "lib"
STATIC = Path(__file__).resolve().parent / "static"
OUTPUTS = ROOT / "outputs"
WEB_OUTPUTS = OUTPUTS / "web_panel"
CAPTURES = WEB_OUTPUTS / "captures"
STREAM_CAPTURES = WEB_OUTPUTS / "stream_captures"
ARTIFACTS = WEB_OUTPUTS / "artifacts"
VALIDATION_DIR = WEB_OUTPUTS / "calibration_validation"
if str(LIB) not in sys.path:
    sys.path.insert(0, str(LIB))
DEFAULT_ROI = [400, 150, 400, 400]
DEFAULT_TARGET_Z_MIN_MM = 99.0
JPEG_QUALITY = 58
CONFIRM_EXECUTE = "EXECUTE"
CONFIRM_RESTORE = "RESTORE_CONTROL_MODE"
STREAM_BOUNDARY = "piper_frame"
CAMERA_LOCK = threading.Lock()
STREAM_LOCK = threading.Lock()
STREAM_STATE_LOCK = threading.Lock()
STREAM_PROCESS: Any = None
STREAM_EPOCH = 0
JOG_LOCK = threading.Lock()
JOG_CONTROL: dict[str, Any] = {
    "stop": None,
    "thread": None,
    "heartbeat_deadline": 0.0,
    "state": {"running": False},
}
FAR_POSE_JSON = ROOT / "config" / "far_pose.json"
SAVED_POSES_DIR = ROOT / "config" / "saved_poses"
ORIGIN_POSE_NAME = "origin"
SAFE_ORIGIN_Z_MM = 220.0

STATE: dict[str, Any] = {
    "roi": DEFAULT_ROI[:],
    "target_z_min_mm": DEFAULT_TARGET_Z_MIN_MM,
    "last_snapshot": None,
    "last_detection_dir": None,
    "last_target_json": None,
    "last_pose_json": None,
    "far_pose_json": str(FAR_POSE_JSON),
    "saved_poses_dir": str(SAVED_POSES_DIR),
    "last_image_url": None,
    "last_seam_pixels": None,
    "last_box_pixels": None,
    "last_validation_target": None,
    "last_validation_target_json": None,
    "last_log": "",
    "image_format": f"jpeg_quality_{JPEG_QUALITY}",
    "stream": "mjpeg",
}


def claim_stream_request() -> tuple[int, bool]:
    global STREAM_EPOCH
    with STREAM_STATE_LOCK:
        STREAM_EPOCH += 1
        process = STREAM_PROCESS
        replaced = process is not None and process.poll() is None
        if replaced:
            process.terminate()
        return STREAM_EPOCH, replaced


def register_stream_process(epoch: int, process: Any) -> bool:
    global STREAM_PROCESS
    with STREAM_STATE_LOCK:
        if epoch != STREAM_EPOCH:
            return False
        STREAM_PROCESS = process
        return True


def stream_was_replaced(epoch: int) -> bool:
    with STREAM_STATE_LOCK:
        return epoch != STREAM_EPOCH


def clear_stream_process(process: Any) -> None:
    global STREAM_PROCESS
    with STREAM_STATE_LOCK:
        if STREAM_PROCESS is process:
            STREAM_PROCESS = None


def stop_stream_for_camera(timeout_s: float = 12.0) -> None:
    claim_stream_request()
    acquired = STREAM_LOCK.acquire(timeout=timeout_s)
    if not acquired:
        raise TimeoutError("video stream did not release camera in time")
    try:
        time.sleep(1.0)
    finally:
        STREAM_LOCK.release()


def read_stream_chunk(process: Any, timeout_s: float) -> bytes:
    if process.stdout is None:
        return b""
    fd = process.stdout.fileno()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if process.poll() is not None:
            try:
                return os.read(fd, 65536)
            except OSError:
                return b""
        readable, _, _ = select.select([fd], [], [], min(0.5, max(0.0, deadline - time.time())))
        if readable:
            return os.read(fd, 65536)
    return b""


def now_id() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def safe_artifact_suffix(value: Any) -> str:
    suffix = re.sub(r"[^0-9A-Za-z_.\-\u4e00-\u9fff]+", "_", str(value or "").strip())
    suffix = suffix.strip("._-")
    return suffix[:40]


def ensure_dirs() -> None:
    CAPTURES.mkdir(parents=True, exist_ok=True)
    STREAM_CAPTURES.mkdir(parents=True, exist_ok=True)
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    VALIDATION_DIR.mkdir(parents=True, exist_ok=True)


def parse_roi(value: Any) -> list[int]:
    if value is None:
        return STATE["roi"][:]
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",")]
    else:
        parts = list(value)
    if len(parts) != 4:
        raise ValueError("roi must be [x, y, width, height]")
    roi = [int(float(part)) for part in parts]
    x, y, w, h = roi
    if x < 0 or y < 0 or w <= 0 or h <= 0:
        raise ValueError("roi values must be positive")
    if x + w > 1280 or y + h > 720:
        raise ValueError("roi is outside 1280x720 image")
    return roi


def parse_motion_mode(value: Any) -> str:
    mode = str(value or "moveL")
    if mode not in {"moveL", "moveP"}:
        raise ValueError("motion_mode must be moveL or moveP")
    return mode


def parse_target_z_min_mm(value: Any) -> float:
    if value is None or value == "":
        return float(STATE.get("target_z_min_mm", DEFAULT_TARGET_Z_MIN_MM))
    fixed_z = float(value)
    if not math.isfinite(fixed_z):
        raise ValueError("fixed target Z must be finite")
    if fixed_z < -150.0 or fixed_z > 500.0:
        raise ValueError("fixed target Z must be within [-150, 500] mm")
    STATE["target_z_min_mm"] = fixed_z
    return fixed_z


def clamp_xyz_z_min_mm(xyz_mm: list[float], target_z_min_mm: float | None = None) -> tuple[list[float], bool]:
    fixed_z = parse_target_z_min_mm(target_z_min_mm)
    adjusted = [float(value) for value in xyz_mm]
    if len(adjusted) != 3:
        raise ValueError("xyz_mm must contain x/y/z")
    if not all(math.isfinite(value) for value in adjusted):
        raise ValueError("x/y/z must be finite numbers")
    z_adjusted = abs(adjusted[2] - fixed_z) > 1.0e-9
    adjusted[2] = fixed_z
    return adjusted, z_adjusted


def parse_segment_mm(value: Any) -> float:
    segment = float(value if value not in (None, "") else 30.0)
    if not math.isfinite(segment):
        raise ValueError("segment_mm must be finite")
    if segment < 1.0 or segment > 300.0:
        raise ValueError("segment_mm must be within [1, 300] mm")
    return segment


def parse_side_cut_px(value: Any) -> float:
    length = float(value if value not in (None, "") else 90.0)
    if not math.isfinite(length):
        raise ValueError("side_cut_px must be finite")
    if length < 5.0 or length > 400.0:
        raise ValueError("side_cut_px must be within [5, 400] px")
    return length


def command_result(command: list[str], *, timeout: float | None = None) -> dict[str, Any]:
    started = time.time()
    completed = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )
    output = completed.stdout or ""
    STATE["last_log"] = output[-12000:]
    return {
        "command": command,
        "returncode": completed.returncode,
        "duration_s": round(time.time() - started, 3),
        "output": output,
    }


def transient_camera_error(output: str) -> bool:
    markers = (
        "uvc_open failed",
        "openUsbDevice failed",
        "was not found",
        "timed out waiting for Orbbec RGB-D frames",
    )
    return any(marker in output for marker in markers)


def camera_command_result(
    command: list[str],
    *,
    timeout: float | None = None,
    attempts: int = 3,
    retry_delay_s: float = 5.0,
) -> dict[str, Any]:
    history = []
    for attempt in range(1, max(1, attempts) + 1):
        result = command_result(command, timeout=timeout)
        result["attempt"] = attempt
        history.append(
            {
                "attempt": attempt,
                "returncode": result["returncode"],
                "duration_s": result["duration_s"],
                "output_tail": result["output"][-2000:],
            }
        )
        if result["returncode"] == 0:
            result["attempts"] = history
            return result
        if attempt >= attempts or not transient_camera_error(result["output"]):
            result["attempts"] = history
            return result
        time.sleep(retry_delay_s)
    return result


def newest_dir(root: Path, prefix: str, before: set[Path] | None = None) -> Path:
    before = before or set()
    candidates = [
        path
        for path in root.glob(f"{prefix}*")
        if path.is_dir() and path.resolve() not in before
    ]
    if not candidates:
        raise RuntimeError(f"no {prefix} directory found under {root}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def write_jpeg(image: Any, output: Path, quality: int = JPEG_QUALITY) -> Path:
    import cv2

    output.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(output), image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError(f"failed to write compressed image: {output}")
    return output


def compress_image(image_path: Path, stem: str, quality: int = JPEG_QUALITY) -> Path:
    import cv2

    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f"failed to read image: {image_path}")
    output = ARTIFACTS / f"{stem}_{now_id()}.jpg"
    return write_jpeg(image, output, quality)


def label_text(
    image: Any,
    text: str,
    pos: tuple[int, int],
    color: tuple[int, int, int],
    scale: float = 0.75,
) -> None:
    import cv2

    black = (0, 0, 0)
    px, py = pos
    cv2.putText(image, text, (px + 2, py + 2), cv2.FONT_HERSHEY_SIMPLEX, scale, black, 4, cv2.LINE_AA)
    cv2.putText(image, text, (px, py), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def draw_roi_overlay(image: Any, roi: list[int]) -> None:
    import cv2

    x, y, w, h = roi
    blue = (255, 120, 40)
    yellow = (40, 230, 255)
    red = (40, 40, 255)
    green = (70, 210, 30)
    white = (255, 255, 255)

    cv2.rectangle(image, (x, y), (x + w, y + h), blue, 4)
    cv2.circle(image, (x, y), 7, yellow, -1)
    cv2.circle(image, (24, 24), 7, yellow, -1)
    cv2.arrowedLine(image, (24, 24), (230, 24), red, 5, tipLength=0.08)
    cv2.arrowedLine(image, (24, 24), (24, 230), green, 5, tipLength=0.08)
    label_text(image, "O (0,0)", (42, 46), yellow)
    label_text(image, "x", (240, 32), red, 0.9)
    label_text(image, "y", (36, 246), green, 0.9)
    label_text(image, "pixel: x -> right, y -> down", (70, 88), white, 0.7)
    label_text(image, f"ROI ({x},{y},{w},{h})", (x + 10, max(32, y - 16)), blue)


def draw_seam_overlay(image: Any, seam_pixels: dict[str, Any] | None) -> None:
    if not seam_pixels:
        return
    import cv2

    lines = seam_pixels.get("lines")
    if isinstance(lines, list):
        colors = {
            "left_end": (0, 255, 255),
            "center": (0, 0, 255),
            "right_end": (255, 0, 255),
        }
        for line in lines:
            if not isinstance(line, dict):
                continue
            try:
                start = line["start_px"]
                end = line["end_px"]
                a = (int(round(float(start[0]))), int(round(float(start[1]))))
                b = (int(round(float(end[0]))), int(round(float(end[1]))))
            except (KeyError, TypeError, ValueError, IndexError):
                continue
            name = str(line.get("name", "cut_line"))
            color = colors.get(name, (0, 0, 255))
            cv2.line(image, a, b, color, 4, cv2.LINE_AA)
            cv2.circle(image, a, 7, (0, 255, 0), -1, cv2.LINE_AA)
            cv2.circle(image, b, 7, (255, 0, 0), -1, cv2.LINE_AA)
            start_label = str(line.get("start_label") or "")
            end_label = str(line.get("end_label") or "")
            label_text(image, start_label or name, (a[0] + 10, a[1] - 10), (0, 255, 0), 0.85)
            label_text(image, end_label or name, (b[0] + 10, b[1] - 10), (255, 0, 0), 0.85)
        return

    try:
        start = seam_pixels["start_px"]
        end = seam_pixels["end_px"]
        a = (int(round(float(start[0]))), int(round(float(start[1]))))
        b = (int(round(float(end[0]))), int(round(float(end[1]))))
    except (KeyError, TypeError, ValueError, IndexError):
        return
    cv2.line(image, a, b, (0, 0, 255), 4, cv2.LINE_AA)
    cv2.circle(image, a, 7, (0, 255, 0), -1, cv2.LINE_AA)
    cv2.circle(image, b, 7, (255, 0, 0), -1, cv2.LINE_AA)
    label_text(image, "seam start", (a[0] + 10, a[1] - 10), (0, 255, 0), 0.65)
    label_text(image, "seam end", (b[0] + 10, b[1] - 10), (255, 0, 0), 0.65)


def draw_box_overlay(image: Any, box_pixels: dict[str, Any] | None) -> None:
    if not box_pixels:
        return
    import cv2
    import numpy as np

    try:
        points = np.asarray(box_pixels["box_px"], dtype=np.float32).reshape(4, 2)
    except (KeyError, TypeError, ValueError):
        return
    pts = np.round(points).astype(np.int32)
    cv2.polylines(image, [pts], True, (0, 200, 255), 4, cv2.LINE_AA)
    center = np.mean(points, axis=0)
    label_text(
        image,
        "box",
        (int(round(float(center[0]))) + 8, int(round(float(center[1]))) - 8),
        (0, 200, 255),
        0.7,
    )


def annotate_roi(color_path: Path, roi: list[int], include_detection_overlay: bool = True) -> Path:
    import cv2

    image = cv2.imread(str(color_path))
    if image is None:
        raise RuntimeError(f"failed to read image: {color_path}")

    draw_roi_overlay(image, roi)
    if include_detection_overlay:
        draw_box_overlay(image, STATE.get("last_box_pixels"))
        draw_seam_overlay(image, STATE.get("last_seam_pixels"))

    x, y, w, h = roi
    output = ARTIFACTS / f"roi_{x}_{y}_{w}_{h}_{now_id()}.jpg"
    return write_jpeg(image, output)


def capture_snapshot(roi: list[int], include_detection_overlay: bool = True) -> dict[str, Any]:
    ensure_dirs()
    before = {path.resolve() for path in CAPTURES.glob("snapshot_*")}
    stop_stream_for_camera()
    with CAMERA_LOCK:
        result = camera_command_result(
            [
                sys.executable,
                str(ROOT / "lib" / "capture_orbbec_sdk_snapshot.py"),
                "--config",
                str(ROOT / "config" / "runtime_config.yaml"),
                "--output-root",
                str(CAPTURES),
            ],
            timeout=30,
            attempts=3,
            retry_delay_s=5.0,
        )
    if result["returncode"] != 0:
        return {"ok": False, "result": result}
    snapshot = newest_dir(CAPTURES, "snapshot_", before)
    annotated = annotate_roi(snapshot / "color.png", roi, include_detection_overlay)
    image_url = file_url(annotated)
    STATE["last_snapshot"] = str(snapshot)
    STATE["last_image_url"] = image_url
    STATE["roi"] = roi[:]
    return {
        "ok": True,
        "snapshot": str(snapshot),
        "roi": roi,
        "image_url": image_url,
        "result": result,
    }


def capture_stream_frame_bgr(roi: list[int], show_roi: bool, show_box: bool, show_seam: bool) -> Any:
    import cv2

    before = {path.resolve() for path in STREAM_CAPTURES.glob("snapshot_*")}
    if not CAMERA_LOCK.acquire(timeout=0.2):
        raise RuntimeError("camera is busy")
    try:
        result = command_result(
            [
                sys.executable,
                str(ROOT / "lib" / "capture_orbbec_sdk_snapshot.py"),
                "--config",
                str(ROOT / "config" / "runtime_config.yaml"),
                "--output-root",
                str(STREAM_CAPTURES),
                "--warmup-frames",
                "5",
                "--frame-timeout-ms",
                "5000",
            ],
            timeout=35,
        )
    finally:
        CAMERA_LOCK.release()
    if result["returncode"] != 0:
        raise RuntimeError(result["output"] or f"stream capture failed with returncode {result['returncode']}")
    snapshot = newest_dir(STREAM_CAPTURES, "snapshot_", before)
    image = cv2.imread(str(snapshot / "color.png"))
    if image is None:
        raise RuntimeError(f"failed to read stream color image: {snapshot / 'color.png'}")
    if show_roi:
        draw_roi_overlay(image, roi)
    if show_box:
        draw_box_overlay(image, STATE.get("last_box_pixels"))
    if show_seam:
        draw_seam_overlay(image, STATE.get("last_seam_pixels"))
    STATE["last_snapshot"] = str(snapshot)
    STATE["last_image_url"] = file_url(snapshot / "color.png")
    return image


def detect_seam(
    roi: list[int],
    mask_mode: str,
    use_last_snapshot: bool,
    target_z_min_mm: float,
) -> dict[str, Any]:
    if mask_mode not in {"rgbd", "cardboard", "depth"}:
        raise ValueError("mask_mode must be rgbd/cardboard/depth")
    ensure_dirs()
    before = {path.resolve() for path in OUTPUTS.glob("seam_run_*")}
    if not use_last_snapshot:
        stop_stream_for_camera()
    command = [
        sys.executable,
        str(ROOT / "01_detect_seam_start_to_base.py"),
        "--roi",
        ",".join(map(str, roi)),
        "--mask-mode",
        mask_mode,
        "--target-z-min-mm",
        f"{float(target_z_min_mm):.6f}",
    ]
    if use_last_snapshot and STATE.get("last_snapshot"):
        command.extend(["--snapshot-dir", str(STATE["last_snapshot"])])
    with CAMERA_LOCK:
        result = camera_command_result(command, timeout=90, attempts=2, retry_delay_s=5.0)
    if result["returncode"] != 0:
        return {"ok": False, "result": result}
    detection_dir = newest_dir(OUTPUTS, "seam_run_", before)
    overlay = max(detection_dir.glob("center_seam_overlay_*.png"), key=lambda p: p.stat().st_mtime)
    overlay_preview = compress_image(overlay, "seam_overlay")
    overlay_url = file_url(overlay_preview)
    target_json = detection_dir / "probe_tip_targets_base.json"
    target = json.loads(target_json.read_text(encoding="utf-8"))
    STATE["last_detection_dir"] = str(detection_dir)
    STATE["last_target_json"] = str(target_json)
    STATE["last_image_url"] = overlay_url
    STATE["last_seam_pixels"] = read_seam_pixels(detection_dir)
    STATE["last_box_pixels"] = read_box_pixels(detection_dir)
    STATE["roi"] = roi[:]
    STATE["target_z_min_mm"] = float(target_z_min_mm)
    return {
        "ok": True,
        "roi": roi,
        "target_z_min_mm": float(target_z_min_mm),
        "detection_dir": str(detection_dir),
        "overlay_url": overlay_url,
        "overlay_png": str(overlay),
        "target_json": str(target_json),
        "target": target,
        "seam_pixels": STATE["last_seam_pixels"],
        "box_pixels": STATE["last_box_pixels"],
        "result": result,
    }


def latest_seam_yaml(detection_dir: Path) -> Path:
    yaml_files = sorted(
        detection_dir.glob("center_seam_result_*.yaml"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not yaml_files:
        yaml_files = sorted(
            detection_dir.glob("center_seam_result.yaml"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
    if not yaml_files:
        raise FileNotFoundError(f"no center_seam_result YAML under {detection_dir}")
    return yaml_files[0]


def read_seam_pixels(detection_dir: Path) -> dict[str, list[float]] | None:
    import yaml

    try:
        yaml_path = latest_seam_yaml(detection_dir)
    except FileNotFoundError:
        return None
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    line = data.get("rgb_line", {}) if isinstance(data, dict) else {}
    start = line.get("start_px")
    end = line.get("end_px")
    if not is_xy(start) or not is_xy(end):
        return None
    return {
        "start_px": [float(start[0]), float(start[1])],
        "end_px": [float(end[0]), float(end[1])],
        "source_yaml": str(yaml_path),
    }


def read_box_pixels(detection_dir: Path) -> dict[str, Any] | None:
    import cv2
    import numpy as np

    mask_files = sorted(
        detection_dir.glob("box_top_mask_*.png"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not mask_files:
        mask_files = sorted(
            detection_dir.glob("box_top_mask.png"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
    if not mask_files:
        return None
    mask = cv2.imread(str(mask_files[0]), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        return None
    points = cv2.findNonZero((mask > 0).astype(np.uint8))
    if points is None or len(points) < 100:
        return None
    rect = cv2.minAreaRect(points)
    box = cv2.boxPoints(rect).astype(np.float32)
    return {
        "box_px": [[float(x), float(y)] for x, y in box],
        "center_px": [float(rect[0][0]), float(rect[0][1])],
        "size_px": [float(rect[1][0]), float(rect[1][1])],
        "angle_deg": float(rect[2]),
        "source_mask": str(mask_files[0]),
    }


def is_xy(value: Any) -> bool:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return False
    try:
        float(value[0])
        float(value[1])
    except (TypeError, ValueError):
        return False
    return True


def target_inputs() -> dict[str, Path]:
    target_json = STATE.get("last_target_json")
    if not target_json:
        raise ValueError("run seam detection before manual calibration")
    document = json.loads(Path(str(target_json)).read_text(encoding="utf-8"))
    inputs = document.get("inputs")
    if not isinstance(inputs, dict):
        raise ValueError("last target JSON has no inputs block")
    depth = Path(str(inputs["registered_depth_png"]))
    camera_info = Path(str(inputs["camera_info"]))
    snapshot = depth.parent
    return {
        "depth": depth,
        "camera_info": camera_info,
        "color": snapshot / "color.png",
    }


def convert_seam_yaml_to_target(seam_yaml: Path, output: Path, target_z_min_mm: float) -> dict[str, Any]:
    paths = target_inputs()
    command = [
        sys.executable,
        str(ROOT / "lib" / "seam_to_probe_tip_targets_sdk.py"),
        "--seam-yaml",
        str(seam_yaml),
        "--depth",
        str(paths["depth"]),
        "--camera-info",
        str(paths["camera_info"]),
        "--extrinsics",
        str(ROOT / "config" / "eye_to_hand_extrinsics.yaml"),
        "--tcp-calibration",
        str(ROOT / "config" / "tcp_offset_m_rad.yaml"),
        "--calibration-bundle",
        str(ROOT / "config" / "calibration_bundle.yaml"),
        "--depth-correction",
        str(ROOT / "config" / "depth_correction.yaml"),
        "--depth-correction-mode",
        "auto",
        "--target-z-min-mm",
        f"{float(target_z_min_mm):.6f}",
        "--output",
        str(output),
    ]
    return command_result(command, timeout=45)


def manual_seam_from_points(
    new_start: list[float],
    new_end: list[float],
    roi: list[int],
    target_z_min_mm: float,
    manual_adjustment: dict[str, Any],
) -> dict[str, Any]:
    import cv2
    import yaml

    detection_dir_value = STATE.get("last_detection_dir")
    if not detection_dir_value:
        raise ValueError("run seam detection before manual calibration")
    detection_dir = Path(str(detection_dir_value))
    if not is_xy(new_start) or not is_xy(new_end):
        raise ValueError("manual seam line requires start_px/end_px")
    new_start = [float(new_start[0]), float(new_start[1])]
    new_end = [float(new_end[0]), float(new_end[1])]
    for label, point in (("start", new_start), ("end", new_end)):
        if not (0.0 <= point[0] < 1280.0 and 0.0 <= point[1] < 720.0):
            raise ValueError(f"manual seam {label} pixel is outside 1280x720: {point}")
    if math.hypot(new_end[0] - new_start[0], new_end[1] - new_start[1]) < 5.0:
        raise ValueError("manual seam start/end are too close")

    source_yaml = latest_seam_yaml(detection_dir)
    document = yaml.safe_load(source_yaml.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(document.get("rgb_line"), dict):
        raise ValueError("source seam YAML has no rgb_line block")
    document["rgb_line"]["start_px"] = new_start
    document["rgb_line"]["end_px"] = new_end
    manual_adjustment = dict(manual_adjustment)
    manual_adjustment.update(
        {
            "new_start_px": new_start,
            "new_end_px": new_end,
            "source_yaml": str(source_yaml),
        }
    )
    document["rgb_line"]["manual_adjustment"] = manual_adjustment
    suffix = safe_artifact_suffix(manual_adjustment.get("line_name") or manual_adjustment.get("anchor") or manual_adjustment.get("mode"))
    manual_id = f"{now_id()}_{suffix}" if suffix else now_id()
    manual_yaml = detection_dir / f"center_seam_result_manual_{manual_id}.yaml"
    manual_yaml.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    target_json = detection_dir / f"probe_tip_targets_base_manual_{manual_id}.json"
    result = convert_seam_yaml_to_target(manual_yaml, target_json, target_z_min_mm)
    if result["returncode"] != 0:
        return {"ok": False, "manual_yaml": str(manual_yaml), "result": result}

    target = json.loads(target_json.read_text(encoding="utf-8"))
    paths = target_inputs()
    image = cv2.imread(str(paths["color"]))
    if image is None:
        raise RuntimeError(f"failed to read image: {paths['color']}")
    manual_pixels = {
        "start_px": new_start,
        "end_px": new_end,
        "source_yaml": str(manual_yaml),
        "manual": True,
    }
    draw_roi_overlay(image, roi)
    draw_box_overlay(image, STATE.get("last_box_pixels"))
    draw_seam_overlay(image, manual_pixels)
    overlay = ARTIFACTS / f"manual_seam_overlay_{manual_id}.jpg"
    write_jpeg(image, overlay)

    STATE["last_target_json"] = str(target_json)
    STATE["last_seam_pixels"] = manual_pixels
    STATE["last_image_url"] = file_url(overlay)
    STATE["roi"] = roi[:]
    STATE["target_z_min_mm"] = float(target_z_min_mm)
    return {
        "ok": True,
        "roi": roi,
        "target_z_min_mm": float(target_z_min_mm),
        "manual_yaml": str(manual_yaml),
        "target_json": str(target_json),
        "overlay_url": STATE["last_image_url"],
        "seam_pixels": manual_pixels,
        "box_pixels": STATE.get("last_box_pixels"),
        "target": target,
        "result": result,
    }


def manual_seam_anchor(
    anchor: str,
    point_px: list[float],
    roi: list[int],
    target_z_min_mm: float,
) -> dict[str, Any]:
    if anchor not in {"start", "end"}:
        raise ValueError("anchor must be start/end")
    if not is_xy(point_px):
        raise ValueError("point_px must be [x, y]")
    detection_dir_value = STATE.get("last_detection_dir")
    if not detection_dir_value:
        raise ValueError("run seam detection before manual calibration")
    detection_dir = Path(str(detection_dir_value))
    seam_pixels = STATE.get("last_seam_pixels") or read_seam_pixels(detection_dir)
    if not seam_pixels:
        raise ValueError("no existing seam line is available to shift")

    old_start = [float(seam_pixels["start_px"][0]), float(seam_pixels["start_px"][1])]
    old_end = [float(seam_pixels["end_px"][0]), float(seam_pixels["end_px"][1])]
    vector = [old_end[0] - old_start[0], old_end[1] - old_start[1]]
    if anchor == "start":
        new_start = [float(point_px[0]), float(point_px[1])]
        new_end = [new_start[0] + vector[0], new_start[1] + vector[1]]
    else:
        new_end = [float(point_px[0]), float(point_px[1])]
        new_start = [new_end[0] - vector[0], new_end[1] - vector[1]]
    return manual_seam_from_points(
        new_start,
        new_end,
        roi,
        target_z_min_mm,
        {
            "mode": f"shift_by_new_{anchor}_px",
            "anchor": anchor,
            "old_start_px": old_start,
            "old_end_px": old_end,
            "delta_px": [new_start[0] - old_start[0], new_start[1] - old_start[1]],
        },
    )


def manual_seam_start(start_px: list[float], roi: list[int], target_z_min_mm: float) -> dict[str, Any]:
    return manual_seam_anchor("start", start_px, roi, target_z_min_mm)


def manual_seam_end(end_px: list[float], roi: list[int], target_z_min_mm: float) -> dict[str, Any]:
    return manual_seam_anchor("end", end_px, roi, target_z_min_mm)


def manual_seam_line(
    start_px: list[float],
    end_px: list[float],
    roi: list[int],
    target_z_min_mm: float,
) -> dict[str, Any]:
    seam_pixels = None
    detection_dir_value = STATE.get("last_detection_dir")
    if detection_dir_value:
        seam_pixels = STATE.get("last_seam_pixels") or read_seam_pixels(Path(str(detection_dir_value)))
    adjustment: dict[str, Any] = {"mode": "select_start_end_px"}
    if seam_pixels:
        adjustment["old_start_px"] = seam_pixels.get("start_px")
        adjustment["old_end_px"] = seam_pixels.get("end_px")
    return manual_seam_from_points(start_px, end_px, roi, target_z_min_mm, adjustment)


def clamp_pixel(point: list[float]) -> list[float]:
    return [
        min(1279.0, max(0.0, float(point[0]))),
        min(719.0, max(0.0, float(point[1]))),
    ]


def three_cut_line_pixels(side_cut_px: float) -> list[dict[str, Any]]:
    seam_pixels = STATE.get("last_seam_pixels")
    if not isinstance(seam_pixels, dict) or not is_xy(seam_pixels.get("start_px")) or not is_xy(seam_pixels.get("end_px")):
        raise ValueError("run seam detection before building three cut lines")
    start = [float(seam_pixels["start_px"][0]), float(seam_pixels["start_px"][1])]
    end = [float(seam_pixels["end_px"][0]), float(seam_pixels["end_px"][1])]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 5.0:
        raise ValueError("center seam is too short")
    perp = [-dy / length, dx / length]
    half = float(side_cut_px) * 0.5
    left_a = clamp_pixel([start[0] + perp[0] * half, start[1] + perp[1] * half])
    left_b = clamp_pixel([start[0] - perp[0] * half, start[1] - perp[1] * half])
    right_a = clamp_pixel([end[0] - perp[0] * half, end[1] - perp[1] * half])
    right_b = clamp_pixel([end[0] + perp[0] * half, end[1] + perp[1] * half])
    return [
        {"name": "left_end", "label": "左端短线", "start_label": "A", "end_label": "B", "start_px": left_a, "end_px": left_b},
        {"name": "center", "label": "中间长线", "start_label": "C", "end_label": "D", "start_px": start, "end_px": end},
        {"name": "right_end", "label": "右端短线", "start_label": "E", "end_label": "F", "start_px": right_a, "end_px": right_b},
    ]


def build_three_cut_lines(roi: list[int], side_cut_px: float, target_z_min_mm: float) -> dict[str, Any]:
    import cv2

    lines = three_cut_line_pixels(side_cut_px)
    target_lines: list[dict[str, Any]] = []
    cut_points: list[dict[str, Any]] = []
    for line in lines:
        result = manual_seam_from_points(
            line["start_px"],
            line["end_px"],
            roi,
            target_z_min_mm,
            {
                "mode": "three_cut_line",
                "line_name": line["name"],
                "side_cut_px": float(side_cut_px),
            },
        )
        if not result.get("ok"):
            return {
                "ok": False,
                "cut_mode": "three_line",
                "failed_line": line["name"],
                "cut_lines": target_lines,
                "result": result.get("result"),
            }
        target_block = (result.get("target") or {}).get("probe_tip_contact_targets_base", {})
        start_m = target_block.get("start_m")
        end_m = target_block.get("end_m")
        start_mm = [float(value) * 1000.0 for value in start_m] if isinstance(start_m, list) and len(start_m) == 3 else None
        end_mm = [float(value) * 1000.0 for value in end_m] if isinstance(end_m, list) and len(end_m) == 3 else None
        target_lines.append(
            {
                **line,
                "target_json": result["target_json"],
                "target": result.get("target"),
            }
        )
        cut_points.extend(
            [
                {
                    "label": line["start_label"],
                    "line_name": line["name"],
                    "line_label": line["label"],
                    "point_name": "start",
                    "pixel_xy": line["start_px"],
                    "xyz_m": start_m,
                    "xyz_mm": start_mm,
                    "target_json": result["target_json"],
                },
                {
                    "label": line["end_label"],
                    "line_name": line["name"],
                    "line_label": line["label"],
                    "point_name": "end",
                    "pixel_xy": line["end_px"],
                    "xyz_m": end_m,
                    "xyz_mm": end_mm,
                    "target_json": result["target_json"],
                },
            ]
        )

    paths = target_inputs()
    image = cv2.imread(str(paths["color"]))
    if image is None:
        raise RuntimeError(f"failed to read image: {paths['color']}")
    seam_pixels = {
        "mode": "three_cut",
        "start_px": lines[1]["start_px"],
        "end_px": lines[1]["end_px"],
        "lines": lines,
    }
    draw_roi_overlay(image, roi)
    draw_box_overlay(image, STATE.get("last_box_pixels"))
    draw_seam_overlay(image, seam_pixels)
    overlay = ARTIFACTS / f"three_cut_overlay_{now_id()}.jpg"
    write_jpeg(image, overlay)

    center_line = target_lines[1]
    STATE["last_cut_lines"] = target_lines
    STATE["last_cut_points"] = cut_points
    STATE["last_target_json"] = str(center_line["target_json"])
    STATE["last_seam_pixels"] = seam_pixels
    STATE["last_image_url"] = file_url(overlay)
    STATE["roi"] = roi[:]
    STATE["target_z_min_mm"] = float(target_z_min_mm)
    return {
        "ok": True,
        "cut_mode": "three_line",
        "roi": roi,
        "target_z_min_mm": float(target_z_min_mm),
        "side_cut_px": float(side_cut_px),
        "overlay_url": STATE["last_image_url"],
        "seam_pixels": seam_pixels,
        "box_pixels": STATE.get("last_box_pixels"),
        "cut_lines": target_lines,
        "cut_points": cut_points,
        "target_json": str(center_line["target_json"]),
        "target": center_line.get("target"),
    }



def snapshot_paths() -> dict[str, Path]:
    snapshot_value = STATE.get("last_snapshot")
    if not snapshot_value:
        raise ValueError("capture a calibration-board snapshot first")
    snapshot = Path(str(snapshot_value))
    paths = {
        "snapshot": snapshot,
        "color": snapshot / "color.png",
        "depth": snapshot / "depth_mm.png",
        "camera_info": snapshot / "camera_info.yaml",
    }
    for label, path in paths.items():
        if label != "snapshot" and not path.is_file():
            raise FileNotFoundError(path)
    return paths


def draw_validation_overlay(
    color_path: Path,
    roi: list[int],
    point_px: list[float],
    target_xyz_mm: list[float],
) -> Path:
    import cv2

    image = cv2.imread(str(color_path))
    if image is None:
        raise RuntimeError(f"failed to read image: {color_path}")
    draw_roi_overlay(image, roi)
    x = int(round(float(point_px[0])))
    y = int(round(float(point_px[1])))
    cv2.drawMarker(image, (x, y), (0, 255, 255), cv2.MARKER_CROSS, 32, 4, cv2.LINE_AA)
    cv2.circle(image, (x, y), 9, (0, 255, 255), 2, cv2.LINE_AA)
    label_text(image, "validation target", (x + 12, y - 12), (0, 255, 255), 0.65)
    label_text(
        image,
        f"base mm: {target_xyz_mm[0]:.1f}, {target_xyz_mm[1]:.1f}, {target_xyz_mm[2]:.1f}",
        (x + 12, y + 18),
        (255, 255, 255),
        0.55,
    )
    output = ARTIFACTS / f"calibration_target_overlay_{now_id()}.jpg"
    return write_jpeg(image, output)


def calibration_target_from_pixel(point_px: list[float], roi: list[int]) -> dict[str, Any]:
    import cv2
    import numpy as np
    import yaml
    from seam_to_probe_tip_targets_sdk import (
        deproject_pixel,
        depth_correction_decision,
        extract_intrinsics,
        load_transform,
        robust_patch_depth_mm,
    )

    if not is_xy(point_px):
        raise ValueError("point_px must be [x, y]")
    paths = snapshot_paths()
    depth = cv2.imread(str(paths["depth"]), cv2.IMREAD_UNCHANGED)
    if depth is None or depth.ndim != 2:
        raise RuntimeError(f"failed to read depth image: {paths['depth']}")
    u, v = float(point_px[0]), float(point_px[1])
    if not (0.0 <= u < depth.shape[1] and 0.0 <= v < depth.shape[0]):
        raise ValueError(f"validation pixel is outside depth image: {point_px}")
    raw_depth_mm = robust_patch_depth_mm(depth, u, v, radius=5)
    if raw_depth_mm is None:
        raise ValueError(f"no stable depth around validation pixel {point_px}")

    camera_info = yaml.safe_load(paths["camera_info"].read_text(encoding="utf-8"))
    extrinsics = yaml.safe_load((ROOT / "config" / "eye_to_hand_extrinsics.yaml").read_text(encoding="utf-8"))
    correction = yaml.safe_load((ROOT / "config" / "depth_correction.yaml").read_text(encoding="utf-8"))
    camera_matrix, distortion, camera_frame, intrinsics_meta = extract_intrinsics(camera_info)
    transform, extrinsic_camera_frame, base_frame = load_transform(extrinsics)
    add_depth_mm, correction_record = depth_correction_decision(
        correction,
        np.asarray([float(raw_depth_mm)], dtype=np.float64),
        "auto",
    )
    used_depth_mm = float(raw_depth_mm + add_depth_mm)
    camera_xyz_m = deproject_pixel(
        np.asarray([u, v], dtype=np.float64),
        used_depth_mm,
        camera_matrix,
        distortion,
    )
    base_xyz_m = (transform @ np.asarray([*camera_xyz_m.tolist(), 1.0], dtype=np.float64))[:3]
    target = {
        "created_at": now_id(),
        "frame_id": base_frame,
        "camera_frame": camera_frame,
        "extrinsic_camera_frame": extrinsic_camera_frame,
        "pixel_xy": [u, v],
        "depth": {
            "raw_mm": float(raw_depth_mm),
            "used_mm": used_depth_mm,
            "correction": correction_record,
        },
        "camera_xyz_m": camera_xyz_m.tolist(),
        "base_xyz_m": base_xyz_m.tolist(),
        "base_xyz_mm": [float(value * 1000.0) for value in base_xyz_m],
        "inputs": {
            "snapshot": str(paths["snapshot"]),
            "color": str(paths["color"]),
            "depth": str(paths["depth"]),
            "camera_info": str(paths["camera_info"]),
            "extrinsics": str(ROOT / "config" / "eye_to_hand_extrinsics.yaml"),
            "depth_correction": str(ROOT / "config" / "depth_correction.yaml"),
            "camera_intrinsics": intrinsics_meta,
        },
    }
    target_id = now_id()
    target_json = VALIDATION_DIR / f"calibration_target_{target_id}.json"
    target_json.write_text(json.dumps(target, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    overlay = draw_validation_overlay(paths["color"], roi, [u, v], target["base_xyz_mm"])
    STATE["last_validation_target"] = target
    STATE["last_validation_target_json"] = str(target_json)
    STATE["last_image_url"] = file_url(overlay)
    STATE["roi"] = roi[:]
    return {
        "ok": True,
        "roi": roi,
        "validation_target": target,
        "validation_target_json": str(target_json),
        "overlay_url": STATE["last_image_url"],
    }


def calibration_validation_error() -> dict[str, Any]:
    target = STATE.get("last_validation_target")
    if not isinstance(target, dict):
        raise ValueError("select a calibration validation target first")
    pose_report = read_pose()
    if not pose_report.get("ok"):
        return pose_report
    actual = pose_report["pose"]["corrected_probe_tip"]
    actual_xyz_mm = [float(value) for value in actual["xyz_mm"]]
    target_xyz_mm = [float(value) for value in target["base_xyz_mm"]]
    diff = [actual_xyz_mm[index] - target_xyz_mm[index] for index in range(3)]
    norm = math.sqrt(sum(value * value for value in diff))
    xy_norm = math.sqrt(diff[0] * diff[0] + diff[1] * diff[1])
    error = {
        "unit": "mm",
        "target_xyz_mm": target_xyz_mm,
        "actual_tcp_xyz_mm": actual_xyz_mm,
        "diff_actual_minus_target_mm": diff,
        "norm_3d_mm": norm,
        "norm_xy_mm": xy_norm,
        "z_error_mm": diff[2],
        "target_pixel_xy": target.get("pixel_xy"),
        "target_json": STATE.get("last_validation_target_json"),
    }
    return {
        "ok": True,
        "validation_target": target,
        "validation_error": error,
        "pose": pose_report["pose"],
        "pose_json": pose_report["pose_json"],
        "result": pose_report["result"],
    }


def read_pose() -> dict[str, Any]:
    before = {path.resolve() for path in OUTPUTS.glob("probe_tip_pose_*.json")}
    result = command_result([sys.executable, str(ROOT / "02_read_probe_tip_pose.py")], timeout=30)
    if result["returncode"] != 0:
        return {"ok": False, "result": result}
    pose_path = newest_file(OUTPUTS, "probe_tip_pose_*.json", before)
    pose = json.loads(pose_path.read_text(encoding="utf-8"))
    STATE["last_pose_json"] = str(pose_path)
    return {"ok": True, "pose_json": str(pose_path), "pose": pose, "result": result}


def newest_file(root: Path, pattern: str, before: set[Path] | None = None) -> Path:
    before = before or set()
    candidates = [
        path
        for path in root.glob(pattern)
        if path.is_file() and path.resolve() not in before
    ]
    if not candidates:
        raise RuntimeError(f"no file matched {pattern} under {root}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def restore_control(execute: bool, confirm: str) -> dict[str, Any]:
    command = [sys.executable, str(ROOT / "05_restore_sdk_control_mode.py")]
    if execute:
        if confirm != CONFIRM_RESTORE:
            raise ValueError(f"restore requires confirm={CONFIRM_RESTORE}")
        command.extend(["--execute", "--confirm", CONFIRM_RESTORE])
    result = command_result(command, timeout=30)
    return {"ok": result["returncode"] == 0, "result": result}


def pose_slug(name: str) -> str:
    cleaned = re.sub(r"[^0-9A-Za-z_.\-\u4e00-\u9fff]+", "_", name.strip())
    cleaned = cleaned.strip("._-")
    if not cleaned:
        raise ValueError("pose name is empty")
    if len(cleaned) > 64:
        cleaned = cleaned[:64]
    return cleaned


def saved_pose_path(name: str) -> Path:
    slug = pose_slug(name)
    return SAVED_POSES_DIR / f"{slug}.json"


def pose_document(name: str, pose_report: dict[str, Any]) -> dict[str, Any]:
    pose = pose_report["pose"]
    corrected = pose.get("corrected_probe_tip")
    if not isinstance(corrected, dict):
        raise ValueError("pose output has no corrected_probe_tip block")
    return {
        "name": name,
        "frame_id": "base_link",
        "unit_note": "TCP probe-tip pose; position in meter/mm, orientation in degree",
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "source_pose_json": pose_report["pose_json"],
        "corrected_probe_tip": {
            "xyz_m": corrected.get("xyz_m"),
            "xyz_mm": corrected.get("xyz_mm"),
            "rpy_deg": corrected.get("rpy_deg"),
            "one_line_xyz_m_rpy_deg": corrected.get("one_line_xyz_m_rpy_deg"),
        },
        "raw_code_output_flange": pose.get("raw_code_output_flange"),
        "joint_feedback": pose.get("joint_feedback"),
    }


def save_named_pose(name: str) -> dict[str, Any]:
    pose_report = read_pose()
    if not pose_report["ok"]:
        return pose_report
    SAVED_POSES_DIR.mkdir(parents=True, exist_ok=True)
    slug = pose_slug(name)
    path = saved_pose_path(slug)
    document = pose_document(slug, pose_report)
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if slug in {"far", ORIGIN_POSE_NAME}:
        FAR_POSE_JSON.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    return {
        "ok": True,
        "pose_name": slug,
        "pose_json": str(path),
        "saved_pose": document,
        "saved_poses": list_saved_poses()["saved_poses"],
        "pose_read": pose_report,
    }


def save_far_pose() -> dict[str, Any]:
    report = save_named_pose(ORIGIN_POSE_NAME)
    if not report["ok"]:
        return report
    STATE["far_pose_json"] = str(FAR_POSE_JSON)
    return {
        "ok": True,
        "far_pose_json": str(FAR_POSE_JSON),
        "far_pose": report["saved_pose"],
        "origin_pose_json": report["pose_json"],
        "origin_pose": report["saved_pose"],
        "saved_poses": report["saved_poses"],
        "pose_read": report["pose_read"],
    }


def list_saved_poses() -> dict[str, Any]:
    SAVED_POSES_DIR.mkdir(parents=True, exist_ok=True)
    poses: list[dict[str, Any]] = []
    for path in sorted(SAVED_POSES_DIR.glob("*.json")):
        try:
            document = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            continue
        name = str(document.get("name") or path.stem)
        corrected = document.get("corrected_probe_tip", {})
        joint = document.get("joint_feedback", {})
        poses.append(
            {
                "name": name,
                "pose_json": str(path),
                "created_at": document.get("created_at"),
                "xyz_mm": corrected.get("xyz_mm") if isinstance(corrected, dict) else None,
                "rpy_deg": corrected.get("rpy_deg") if isinstance(corrected, dict) else None,
                "joint_deg": joint.get("deg") if isinstance(joint, dict) else None,
            }
        )
    STATE["saved_poses"] = poses
    return {"ok": True, "saved_poses": poses}


def delete_saved_pose(name: str) -> dict[str, Any]:
    slug = pose_slug(name)
    path = saved_pose_path(slug)
    deleted: list[str] = []
    if path.is_file():
        path.unlink()
        deleted.append(str(path))
    if slug in {"far", ORIGIN_POSE_NAME} and FAR_POSE_JSON.is_file():
        FAR_POSE_JSON.unlink()
        deleted.append(str(FAR_POSE_JSON))
    if not deleted:
        raise ValueError(f"saved pose does not exist: {slug}")
    return {
        "ok": True,
        "deleted_pose_name": slug,
        "deleted_files": deleted,
        "saved_poses": list_saved_poses()["saved_poses"],
    }


def move_pose_json(
    pose_json: Path,
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
    target_z_min_mm: float | None = None,
) -> dict[str, Any]:
    if not pose_json.is_file():
        raise ValueError(f"pose json does not exist: {pose_json}")
    pose = json.loads(pose_json.read_text(encoding="utf-8"))
    corrected = pose.get("corrected_probe_tip")
    if not isinstance(corrected, dict):
        raise ValueError("pose json has no corrected_probe_tip block")
    xyz_mm = corrected.get("xyz_mm")
    rpy_deg = corrected.get("rpy_deg")
    if not isinstance(xyz_mm, list) or len(xyz_mm) != 3:
        raise ValueError("pose json corrected_probe_tip.xyz_mm is invalid")
    if not isinstance(rpy_deg, list) or len(rpy_deg) != 3:
        raise ValueError("pose json corrected_probe_tip.rpy_deg is invalid")
    response = move_xyz_rpy_mm(
        [float(value) for value in xyz_mm],
        [float(value) for value in rpy_deg],
        execute,
        confirm,
        speed,
        motion_mode,
        target_z_min_mm=target_z_min_mm,
        send_duration=5.0,
        wait_after_send=1.0,
    )
    response.update(
        {
            "pose_json": str(pose_json),
            "saved_pose": pose,
        }
    )
    return response


def move_xyz_rpy_mm(
    xyz_mm: list[float],
    rpy_deg: list[float],
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
    *,
    target_z_min_mm: float | None = None,
    send_duration: float = 8.0,
    wait_after_send: float = 1.0,
) -> dict[str, Any]:
    requested_xyz_mm = [float(value) for value in xyz_mm]
    target_xyz_mm, z_clamped = clamp_xyz_z_min_mm(requested_xyz_mm, target_z_min_mm)
    z_min = parse_target_z_min_mm(target_z_min_mm)
    command = [
        sys.executable,
        str(ROOT / "04_move_to_probe_tip_pose.py"),
        "--x",
        f"{float(target_xyz_mm[0]):.6f}",
        "--y",
        f"{float(target_xyz_mm[1]):.6f}",
        "--z",
        f"{float(target_xyz_mm[2]):.6f}",
        "--unit",
        "mm",
        "--rx",
        f"{float(rpy_deg[0]):.6f}",
        "--ry",
        f"{float(rpy_deg[1]):.6f}",
        "--rz",
        f"{float(rpy_deg[2]):.6f}",
        "--speed-percent",
        str(int(speed)),
        "--motion-mode",
        motion_mode,
        "--send-duration",
        f"{float(send_duration):.3f}",
        "--wait-after-send",
        f"{float(wait_after_send):.3f}",
    ]
    if execute:
        if confirm != CONFIRM_EXECUTE:
            raise ValueError(f"motion requires confirm={CONFIRM_EXECUTE}")
        command.extend(["--execute", "--confirm", CONFIRM_EXECUTE])
    result = command_result(command, timeout=max(20.0, send_duration + wait_after_send + 10.0))
    return {
        "ok": result["returncode"] == 0,
        "requested_xyz_mm": requested_xyz_mm,
        "target_xyz_mm": target_xyz_mm,
        "target_z_min_mm": z_min,
        "z_clamped": z_clamped,
        "target_rpy_deg": rpy_deg,
        "motion_mode": motion_mode,
        "result": result,
    }


def move_named_pose(
    name: str,
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
    target_z_min_mm: float | None = None,
) -> dict[str, Any]:
    path = saved_pose_path(name)
    if not path.is_file() and pose_slug(name) == "far" and FAR_POSE_JSON.is_file():
        path = FAR_POSE_JSON
    response = move_pose_json(path, execute, confirm, speed, motion_mode, target_z_min_mm)
    response["pose_name"] = pose_slug(name)
    return response


def move_manual_xyz(
    xyz_mm: list[float],
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
    target_z_min_mm: float | None = None,
) -> dict[str, Any]:
    if len(xyz_mm) != 3:
        raise ValueError("xyz_mm must contain x/y/z")
    xyz = [float(value) for value in xyz_mm]
    if not all(math.isfinite(value) for value in xyz):
        raise ValueError("x/y/z must be finite numbers")
    pose_report = read_pose()
    if not pose_report["ok"]:
        return pose_report
    current = pose_report["pose"]["corrected_probe_tip"]
    rpy_deg = [float(value) for value in current["rpy_deg"]]
    response = move_xyz_rpy_mm(
        xyz,
        rpy_deg,
        execute,
        confirm,
        speed,
        motion_mode,
        target_z_min_mm=target_z_min_mm,
        send_duration=5.0,
        wait_after_send=1.0,
    )
    response["pose_before"] = pose_report["pose"]
    response["target_xyz_mm"] = xyz
    response["target_rpy_deg"] = rpy_deg
    return response


def move_far_pose(
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
) -> dict[str, Any]:
    origin_json = saved_pose_path(ORIGIN_POSE_NAME)
    if not origin_json.is_file():
        raise ValueError("origin pose is not saved yet; click 保存原点 first")
    if execute and confirm != CONFIRM_EXECUTE:
        raise ValueError(f"motion requires confirm={CONFIRM_EXECUTE}")

    origin_pose = json.loads(origin_json.read_text(encoding="utf-8"))
    origin_corrected = origin_pose.get("corrected_probe_tip")
    if not isinstance(origin_corrected, dict):
        raise ValueError("origin pose has no corrected_probe_tip block")
    origin_xyz_mm = [float(value) for value in origin_corrected["xyz_mm"]]
    origin_rpy_deg = [float(value) for value in origin_corrected["rpy_deg"]]

    steps: list[dict[str, Any]] = []

    def add_step(name: str, response: dict[str, Any]) -> bool:
        steps.append({"name": name, **response})
        return bool(response.get("ok"))

    pose_report = read_pose()
    if not add_step("read_current_pose", pose_report):
        return {
            "ok": False,
            "far_pose_json": str(origin_json),
            "far_pose": origin_pose,
            "origin_pose_json": str(origin_json),
            "origin_pose": origin_pose,
            "stopped_at": "read_current_pose",
            "steps": steps,
        }

    current = pose_report["pose"]["corrected_probe_tip"]
    current_xyz_mm = [float(value) for value in current["xyz_mm"]]
    current_rpy_deg = [float(value) for value in current["rpy_deg"]]
    safe_z_mm = max(SAFE_ORIGIN_Z_MM, current_xyz_mm[2], origin_xyz_mm[2])
    lift_xyz_mm = [current_xyz_mm[0], current_xyz_mm[1], safe_z_mm]

    if not add_step(
        "move_origin_lift",
        move_xyz_rpy_mm(
            lift_xyz_mm,
            current_rpy_deg,
            execute,
            confirm,
            speed,
            motion_mode,
            send_duration=8.0,
            wait_after_send=1.0,
        ),
    ):
        return {
            "ok": False,
            "far_pose_json": str(origin_json),
            "far_pose": origin_pose,
            "origin_pose_json": str(origin_json),
            "origin_pose": origin_pose,
            "safe_z_mm": safe_z_mm,
            "stopped_at": "move_origin_lift",
            "steps": steps,
        }

    return {
        "ok": True,
        "far_pose_json": str(origin_json),
        "far_pose": origin_pose,
        "origin_pose_json": str(origin_json),
        "origin_pose": origin_pose,
        "safe_z_mm": safe_z_mm,
        "message": "stopped at safe height above current/origin; final descent to origin is disabled",
        "steps": steps,
    }


def move_point(
    point_name: str,
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
    target_z_min_mm: float | None = None,
) -> dict[str, Any]:
    if point_name not in {"start", "end", "mid"}:
        raise ValueError("point_name must be start/end/mid")
    target_json = STATE.get("last_target_json")
    if not target_json:
        raise ValueError("run seam detection before moving to a seam point")
    return move_point_from_target_json(
        Path(str(target_json)),
        point_name,
        execute,
        confirm,
        speed,
        motion_mode,
        target_z_min_mm,
    )


def move_point_from_target_json(
    target_json: Path,
    point_name: str,
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
    target_z_min_mm: float | None = None,
) -> dict[str, Any]:
    if point_name not in {"start", "end", "mid"}:
        raise ValueError("point_name must be start/end/mid")
    target_mm = current_target_point_mm_from_path(target_json, point_name)
    pose_report = read_pose()
    if not pose_report["ok"]:
        return pose_report
    rpy_deg = [float(value) for value in pose_report["pose"]["corrected_probe_tip"]["rpy_deg"]]
    response = move_xyz_rpy_mm(
        target_mm,
        rpy_deg,
        execute,
        confirm,
        speed,
        motion_mode,
        target_z_min_mm=target_z_min_mm,
        send_duration=3.0,
        wait_after_send=0.5,
    )
    response.update(
        {
            "point_json": str(target_json),
            "point_name": point_name,
            "pose_before": pose_report["pose"],
        }
    )
    return response


def current_target_point_mm_from_path(path: Path, point_name: str) -> list[float]:
    document = json.loads(path.read_text(encoding="utf-8"))
    key = f"{point_name}_m"
    for block_name in ("probe_tip_contact_targets_base", "base_link_line"):
        block = document.get(block_name)
        if isinstance(block, dict) and isinstance(block.get(key), list) and len(block[key]) == 3:
            return [float(value) * 1000.0 for value in block[key]]
    if point_name == "mid":
        start_mm, end_mm, _ = current_target_line_mm_from_path(path)
        return [(start_mm[index] + end_mm[index]) * 0.5 for index in range(3)]
    if document.get("xyz_m") is not None and point_name == "start":
        xyz_m = document["xyz_m"]
        if isinstance(xyz_m, list) and len(xyz_m) == 3:
            return [float(value) * 1000.0 for value in xyz_m]
    if document.get("target_base_m") is not None and point_name == "start":
        xyz_m = document["target_base_m"]
        if isinstance(xyz_m, list) and len(xyz_m) == 3:
            return [float(value) * 1000.0 for value in xyz_m]
    raise ValueError(f"{path} has no {point_name} target point")


def current_target_line_mm_from_path(path: Path) -> tuple[list[float], list[float], Path]:
    document = json.loads(path.read_text(encoding="utf-8"))
    block = document.get("probe_tip_contact_targets_base")
    if not isinstance(block, dict):
        raise ValueError("last target JSON has no probe_tip_contact_targets_base block")
    start_m = block.get("start_m")
    end_m = block.get("end_m")
    if not isinstance(start_m, list) or not isinstance(end_m, list) or len(start_m) != 3 or len(end_m) != 3:
        raise ValueError("last target JSON start_m/end_m are invalid")
    start_mm = [float(value) * 1000.0 for value in start_m]
    end_mm = [float(value) * 1000.0 for value in end_m]
    return start_mm, end_mm, path


def current_target_line_mm() -> tuple[list[float], list[float], Path]:
    target_json = STATE.get("last_target_json")
    if not target_json:
        raise ValueError("run seam detection before moving along the seam")
    return current_target_line_mm_from_path(Path(str(target_json)))


def interpolated_line_points_mm(
    start_mm: list[float],
    end_mm: list[float],
    segment_mm: float,
) -> tuple[list[list[float]], float, int]:
    delta = [end_mm[index] - start_mm[index] for index in range(3)]
    length = math.sqrt(sum(value * value for value in delta))
    if length <= 1.0e-9:
        raise ValueError("start/end seam targets are identical")
    count = max(1, int(math.ceil(length / float(segment_mm))))
    points = []
    for index in range(1, count + 1):
        ratio = index / count
        points.append([start_mm[axis] + delta[axis] * ratio for axis in range(3)])
    return points, length, count


def move_line_segments(
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
    segment_mm: float,
    target_z_min_mm: float | None = None,
) -> dict[str, Any]:
    if execute and confirm != CONFIRM_EXECUTE:
        raise ValueError(f"segmented motion requires confirm={CONFIRM_EXECUTE}")
    start_mm, end_mm, path = current_target_line_mm()
    return move_line_segments_for_path(path, start_mm, end_mm, execute, confirm, speed, motion_mode, segment_mm, target_z_min_mm)


def move_line_segments_for_path(
    path: Path,
    start_mm: list[float],
    end_mm: list[float],
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
    segment_mm: float,
    target_z_min_mm: float | None = None,
) -> dict[str, Any]:
    points, length_mm, count = interpolated_line_points_mm(start_mm, end_mm, segment_mm)
    pose_report = read_pose()
    if not pose_report["ok"]:
        return pose_report
    rpy_deg = [float(value) for value in pose_report["pose"]["corrected_probe_tip"]["rpy_deg"]]
    steps: list[dict[str, Any]] = []
    for index, point in enumerate(points, start=1):
        response = move_xyz_rpy_mm(
            point,
            rpy_deg,
            execute,
            confirm,
            speed,
            motion_mode,
            target_z_min_mm=target_z_min_mm,
            send_duration=3.0,
            wait_after_send=0.5,
        )
        step = {
            "name": f"line_segment_{index:03d}_of_{count:03d}",
            "segment_index": index,
            "segment_count": count,
            "target_xyz_mm": point,
            **response,
        }
        steps.append(step)
        if not response.get("ok"):
            return {
                "ok": False,
                "stopped_at": step["name"],
                "segmented_line": {
                    "target_json": str(path),
                    "start_xyz_mm": start_mm,
                    "end_xyz_mm": end_mm,
                    "target_z_min_mm": parse_target_z_min_mm(target_z_min_mm),
                    "line_length_mm": length_mm,
                    "requested_segment_mm": segment_mm,
                    "segment_count": count,
                    "completed_segments": index - 1,
                },
                "steps": steps,
            }
    return {
        "ok": True,
        "execute": execute,
        "motion_mode": motion_mode,
        "segmented_line": {
            "target_json": str(path),
            "start_xyz_mm": start_mm,
            "end_xyz_mm": end_mm,
            "target_z_min_mm": parse_target_z_min_mm(target_z_min_mm),
            "line_length_mm": length_mm,
            "requested_segment_mm": segment_mm,
            "segment_count": count,
            "completed_segments": count,
        },
        "steps": steps,
    }


def move_three_cut_lines(
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
    segment_mm: float,
    use_segments: bool,
    target_z_min_mm: float | None = None,
) -> dict[str, Any]:
    if execute and confirm != CONFIRM_EXECUTE:
        raise ValueError(f"three-line motion requires confirm={CONFIRM_EXECUTE}")
    cut_points = STATE.get("last_cut_points")
    if not isinstance(cut_points, list) or not cut_points:
        raise ValueError("build three cut lines before moving")
    point_by_label = {
        str(point.get("label", "")).upper(): point
        for point in cut_points
        if isinstance(point, dict)
    }
    required_labels = ["A", "B", "C", "D", "E", "F"]
    missing = [label for label in required_labels if label not in point_by_label]
    if missing:
        raise ValueError(f"three-cut points missing: {missing}")
    ordered_labels = ["E", "F", "D", "C", "B", "A"]
    ordered_points = [point_by_label[label] for label in ordered_labels]
    xyz_points: list[list[float]] = []
    for label, point in zip(ordered_labels, ordered_points):
        xyz = point.get("xyz_mm")
        if not isinstance(xyz, list) or len(xyz) != 3:
            raise ValueError(f"three-cut point {label} has no xyz_mm")
        xyz_points.append([float(value) for value in xyz])

    pose_report = read_pose()
    if not pose_report["ok"]:
        return pose_report
    rpy_deg = [float(value) for value in pose_report["pose"]["corrected_probe_tip"]["rpy_deg"]]

    steps: list[dict[str, Any]] = []
    first_response = move_xyz_rpy_mm(
        xyz_points[0],
        rpy_deg,
        execute,
        confirm,
        speed,
        motion_mode,
        target_z_min_mm=target_z_min_mm,
        send_duration=3.0,
        wait_after_send=0.5,
    )
    first_step = {
        "name": "move_to_E",
        "point_label": "E",
        "target_xyz_mm": xyz_points[0],
        **first_response,
    }
    steps.append(first_step)
    if not first_response.get("ok"):
        return {
            "ok": False,
            "cut_mode": "three_line",
            "path_mode": "continuous_E_to_A",
            "stopped_at": first_step["name"],
            "cut_lines": STATE.get("last_cut_lines"),
            "cut_points": cut_points,
            "steps": steps,
        }

    for index in range(1, len(xyz_points)):
        start_label = ordered_labels[index - 1]
        end_label = ordered_labels[index]
        start_mm = xyz_points[index - 1]
        end_mm = xyz_points[index]
        if use_segments:
            points, length_mm, count = interpolated_line_points_mm(start_mm, end_mm, segment_mm)
            for segment_index, point in enumerate(points, start=1):
                response = move_xyz_rpy_mm(
                    point,
                    rpy_deg,
                    execute,
                    confirm,
                    speed,
                    motion_mode,
                    target_z_min_mm=target_z_min_mm,
                    send_duration=3.0,
                    wait_after_send=0.5,
                )
                step = {
                    "name": f"{start_label}_to_{end_label}_segment_{segment_index:03d}_of_{count:03d}",
                    "from_label": start_label,
                    "to_label": end_label,
                    "segment_index": segment_index,
                    "segment_count": count,
                    "line_length_mm": length_mm,
                    "target_xyz_mm": point,
                    **response,
                }
                steps.append(step)
                if not response.get("ok"):
                    return {
                        "ok": False,
                        "cut_mode": "three_line",
                        "path_mode": "continuous_E_to_A",
                        "stopped_at": step["name"],
                        "cut_lines": STATE.get("last_cut_lines"),
                        "cut_points": cut_points,
                        "steps": steps,
                    }
        else:
            response = move_xyz_rpy_mm(
                end_mm,
                rpy_deg,
                execute,
                confirm,
                speed,
                motion_mode,
                target_z_min_mm=target_z_min_mm,
                send_duration=3.0,
                wait_after_send=0.5,
            )
            step = {
                "name": f"{start_label}_to_{end_label}",
                "from_label": start_label,
                "to_label": end_label,
                "target_xyz_mm": end_mm,
                **response,
            }
            steps.append(step)
            if not response.get("ok"):
                return {
                    "ok": False,
                    "cut_mode": "three_line",
                    "path_mode": "continuous_E_to_A",
                    "stopped_at": step["name"],
                    "cut_lines": STATE.get("last_cut_lines"),
                    "cut_points": cut_points,
                    "steps": steps,
                }
    return {
        "ok": True,
        "cut_mode": "three_line",
        "path_mode": "continuous_E_to_A",
        "path_labels": ordered_labels,
        "execute": execute,
        "motion_mode": motion_mode,
        "use_segments": use_segments,
        "target_z_min_mm": parse_target_z_min_mm(target_z_min_mm),
        "cut_lines": STATE.get("last_cut_lines"),
        "cut_points": cut_points,
        "steps": steps,
    }


def move_three_cut_point(
    point_label: str,
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
    target_z_min_mm: float | None = None,
) -> dict[str, Any]:
    label = str(point_label).strip().upper()
    if label not in {"A", "B", "C", "D", "E", "F"}:
        raise ValueError("point_label must be one of A/B/C/D/E/F")
    cut_points = STATE.get("last_cut_points")
    if not isinstance(cut_points, list) or not cut_points:
        raise ValueError("build three cut lines before moving to A-F")
    point = next((item for item in cut_points if isinstance(item, dict) and item.get("label") == label), None)
    if not isinstance(point, dict):
        raise ValueError(f"three-cut point {label} is not available")
    response = move_point_from_target_json(
        Path(str(point["target_json"])),
        str(point["point_name"]),
        execute,
        confirm,
        speed,
        motion_mode,
        target_z_min_mm,
    )
    response.update(
        {
            "cut_mode": "three_line",
            "point_label": label,
            "cut_point": point,
            "cut_points": cut_points,
        }
    )
    return response


def run_cut_sequence(
    roi: list[int],
    mask_mode: str,
    segment_mm: float,
    use_segments: bool,
    target_z_min_mm: float,
    execute: bool,
    confirm: str,
    speed: int,
    motion_mode: str,
) -> dict[str, Any]:
    if execute and confirm != CONFIRM_EXECUTE:
        raise ValueError(f"one-click execution requires confirm={CONFIRM_EXECUTE}")

    steps: list[dict[str, Any]] = []

    def add_step(name: str, response: dict[str, Any], required: bool = True) -> bool:
        steps.append({"name": name, **response})
        return bool(response.get("ok")) or not required

    if not add_step("detect_seam", detect_seam(roi, mask_mode, False, target_z_min_mm)):
        return {"ok": False, "stopped_at": "detect_seam", "steps": steps}

    restore_confirm = CONFIRM_RESTORE if execute else ""
    if not add_step("restore_sdk_control_mode", restore_control(execute, restore_confirm)):
        return {"ok": False, "stopped_at": "restore_sdk_control_mode", "steps": steps}

    if not add_step("move_point_2_pose", move_named_pose("point_2", execute, confirm, speed, motion_mode, target_z_min_mm)):
        return {"ok": False, "stopped_at": "move_point_2_pose", "steps": steps}

    if not add_step("move_seam_start", move_point("start", execute, confirm, speed, motion_mode, target_z_min_mm)):
        return {"ok": False, "stopped_at": "move_seam_start", "steps": steps}
    if use_segments:
        if not add_step(
            "move_seam_end_segments",
            move_line_segments(execute, confirm, speed, motion_mode, segment_mm, target_z_min_mm),
        ):
            return {"ok": False, "stopped_at": "move_seam_end_segments", "steps": steps}
    else:
        if not add_step("move_seam_end", move_point("end", execute, confirm, speed, motion_mode, target_z_min_mm)):
            return {"ok": False, "stopped_at": "move_seam_end", "steps": steps}

    return {
        "ok": True,
        "execute": execute,
        "motion_mode": motion_mode,
        "use_segments": use_segments,
        "target_z_min_mm": target_z_min_mm,
        "steps": steps,
    }


def jog(
    axis: str,
    direction: int,
    step_mm: float,
    execute: bool,
    confirm: str,
    speed: int,
    target_z_min_mm: float | None = None,
    send_duration: float = 0.35,
    send_rate_hz: float = 80.0,
) -> dict[str, Any]:
    if axis not in {"x", "y", "z"}:
        raise ValueError("axis must be x/y/z")
    if direction not in {-1, 1}:
        raise ValueError("direction must be -1 or 1")
    if step_mm <= 0 or step_mm > 50:
        raise ValueError("step_mm must be in (0, 50]")
    if send_duration <= 0.05 or send_duration > 2.0:
        raise ValueError("send_duration must be in (0.05, 2.0]")
    if send_rate_hz < 10 or send_rate_hz > 120:
        raise ValueError("send_rate_hz must be in [10, 120]")
    pose_report = read_pose()
    if not pose_report["ok"]:
        return pose_report
    pose = pose_report["pose"]["corrected_probe_tip"]
    xyz_mm = list(pose["xyz_mm"])
    rpy = list(pose["rpy_deg"])
    index = {"x": 0, "y": 1, "z": 2}[axis]
    xyz_mm[index] += direction * float(step_mm)
    requested_xyz_mm = [float(value) for value in xyz_mm]
    target_xyz_mm, z_clamped = clamp_xyz_z_min_mm(requested_xyz_mm, target_z_min_mm)
    z_min = parse_target_z_min_mm(target_z_min_mm)
    command = [
        sys.executable,
        str(ROOT / "04_move_to_probe_tip_pose.py"),
        "--x",
        f"{target_xyz_mm[0]:.6f}",
        "--y",
        f"{target_xyz_mm[1]:.6f}",
        "--z",
        f"{target_xyz_mm[2]:.6f}",
        "--unit",
        "mm",
        "--rx",
        f"{rpy[0]:.6f}",
        "--ry",
        f"{rpy[1]:.6f}",
        "--rz",
        f"{rpy[2]:.6f}",
        "--speed-percent",
        str(int(speed)),
        "--send-duration",
        f"{float(send_duration):.3f}",
        "--send-rate-hz",
        f"{float(send_rate_hz):.3f}",
        "--motion-mode",
        "moveL",
        "--wait-after-send",
        "0.150",
    ]
    if execute:
        if confirm != CONFIRM_EXECUTE:
            raise ValueError(f"jog requires confirm={CONFIRM_EXECUTE}")
        command.extend(["--execute", "--confirm", CONFIRM_EXECUTE])
    result = command_result(command, timeout=max(5.0, float(send_duration) + 4.0))
    return {
        "ok": result["returncode"] == 0,
        "motion_mode": "moveL",
        "requested_xyz_mm": requested_xyz_mm,
        "target_xyz_mm": target_xyz_mm,
        "target_z_min_mm": z_min,
        "z_clamped": z_clamped,
        "target_rpy_deg": rpy,
        "pose_read": pose_report,
        "result": result,
    }


def stop_continuous_jog() -> dict[str, Any]:
    with JOG_LOCK:
        stop_event = JOG_CONTROL.get("stop")
        thread = JOG_CONTROL.get("thread")
        state = dict(JOG_CONTROL.get("state") or {})
    if stop_event is not None:
        stop_event.set()
    if thread is not None and thread.is_alive():
        thread.join(timeout=1.0)
    with JOG_LOCK:
        latest = dict(JOG_CONTROL.get("state") or state)
        latest["running"] = False
        JOG_CONTROL["state"] = latest
        JOG_CONTROL["stop"] = None
        JOG_CONTROL["thread"] = None
        JOG_CONTROL["heartbeat_deadline"] = 0.0
    return {"ok": True, "jog": latest}


def keepalive_continuous_jog() -> dict[str, Any]:
    with JOG_LOCK:
        state = dict(JOG_CONTROL.get("state") or {})
        if not state.get("running"):
            return {"ok": False, "error": "continuous jog is not running", "jog": state}
        JOG_CONTROL["heartbeat_deadline"] = time.time() + 0.60
        state["heartbeat_deadline"] = JOG_CONTROL["heartbeat_deadline"]
        JOG_CONTROL["state"] = state
    return {"ok": True, "jog": state}


def continuous_jog_worker(
    stop_event: threading.Event,
    axis: str,
    direction: int,
    speed_mm_s: float,
    speed_percent: int,
    target_z_min_mm: float,
    can_name: str,
) -> None:
    try:
        from piper_sdk_control_utils import (
            connect_piper,
            end_pose_cmd_from_flange_pose,
            flange_from_tip_target_m,
            load_tcp_offset_m_rad,
            move_mode_code,
            read_probe_tip_pose,
            require_command_ready,
            validate_xyz_workspace,
        )

        tcp_offset, _ = load_tcp_offset_m_rad(ROOT / "config" / "tcp_offset_m_rad.yaml")
        piper = connect_piper(can_name, enable=True, enable_timeout=5.0)
        time.sleep(0.2)
        require_command_ready(piper)
        index = {"x": 0, "y": 1, "z": 2}[axis]
        fixed_z_m = float(target_z_min_mm) / 1000.0
        dt = 0.01
        move_mode = move_mode_code("moveL")
        sent = 0
        with JOG_LOCK:
            JOG_CONTROL["state"] = {
                "running": True,
                "axis": axis,
                "direction": direction,
                "speed_mm_s": speed_mm_s,
                "target_z_min_mm": target_z_min_mm,
                "sent": sent,
            }
        while not stop_event.is_set():
            with JOG_LOCK:
                heartbeat_deadline = float(JOG_CONTROL.get("heartbeat_deadline") or 0.0)
            if time.time() > heartbeat_deadline:
                raise TimeoutError("continuous jog heartbeat timeout")

            feedback = read_probe_tip_pose(piper, tcp_offset)["corrected_probe_tip"]
            xyz_m = [float(value) for value in feedback["xyz_m"]]
            rpy_deg = [float(value) for value in feedback["rpy_deg"]]
            target_xyz_m = xyz_m[:]
            target_xyz_m[index] += direction * speed_mm_s * dt / 1000.0
            z_clamped = abs(target_xyz_m[2] - fixed_z_m) > 1.0e-9
            target_xyz_m[2] = fixed_z_m
            flange_target = flange_from_tip_target_m(target_xyz_m, rpy_deg, tcp_offset)
            validate_xyz_workspace(
                "continuous_jog_tip",
                target_xyz_m,
                x_min=0.0,
                x_max=0.8,
                y_min=-0.45,
                y_max=0.45,
                z_min=-0.15,
                z_max=0.5,
            )
            validate_xyz_workspace(
                "continuous_jog_flange",
                flange_target,
                x_min=0.0,
                x_max=0.8,
                y_min=-0.45,
                y_max=0.45,
                z_min=-0.15,
                z_max=0.5,
            )
            cmd = end_pose_cmd_from_flange_pose(flange_target, rpy_deg)
            piper.MotionCtrl_2(0x01, move_mode, int(speed_percent), 0x00)
            piper.EndPoseCtrl(*cmd)
            sent += 1
            with JOG_LOCK:
                JOG_CONTROL["state"] = {
                    "running": True,
                    "axis": axis,
                    "direction": direction,
                    "speed_mm_s": speed_mm_s,
                    "sent": sent,
                    "feedback_xyz_mm": [value * 1000.0 for value in xyz_m],
                    "target_xyz_mm": [value * 1000.0 for value in target_xyz_m],
                    "target_z_min_mm": target_z_min_mm,
                    "z_clamped": z_clamped,
                    "rpy_deg": rpy_deg,
                }
            time.sleep(dt)
    except Exception as error:
        with JOG_LOCK:
            state = dict(JOG_CONTROL.get("state") or {})
            state.update({"running": False, "error": str(error)})
            JOG_CONTROL["state"] = state
    finally:
        with JOG_LOCK:
            state = dict(JOG_CONTROL.get("state") or {})
            state["running"] = False
            JOG_CONTROL["state"] = state


def start_continuous_jog(
    axis: str,
    direction: int,
    speed_mm_s: float,
    execute: bool,
    confirm: str,
    speed_percent: int,
    target_z_min_mm: float | None = None,
) -> dict[str, Any]:
    if axis not in {"x", "y", "z"}:
        raise ValueError("axis must be x/y/z")
    if direction not in {-1, 1}:
        raise ValueError("direction must be -1 or 1")
    if speed_mm_s <= 0 or speed_mm_s > 80:
        raise ValueError("speed_mm_s must be in (0, 80]")
    if execute and confirm != CONFIRM_EXECUTE:
        raise ValueError(f"continuous jog requires confirm={CONFIRM_EXECUTE}")
    z_min = parse_target_z_min_mm(target_z_min_mm)
    stop_continuous_jog()
    with JOG_LOCK:
        old_thread = JOG_CONTROL.get("thread")
        if old_thread is not None and old_thread.is_alive():
            return {"ok": False, "error": "previous jog thread is still stopping"}
    if not execute:
        return {
            "ok": True,
            "jog": {
                "running": False,
                "dry_run": True,
                "axis": axis,
                "direction": direction,
                "speed_mm_s": speed_mm_s,
                "target_z_min_mm": z_min,
            },
        }
    from piper_sdk import C_PiperInterface_V2
    from piper_sdk_control_utils import arm_status_summary, command_ready_problems

    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort(piper_init=False)
    status = arm_status_summary(piper)
    if "time stamp:0" in str(status.get("text", "")):
        time.sleep(0.5)
        status = arm_status_summary(piper)
    if "time stamp:0" in str(status.get("text", "")):
        return {
            "ok": False,
            "error": "jog refused: no fresh arm status feedback",
            "status": status,
        }
    problems = command_ready_problems(status)
    if problems:
        return {
            "ok": False,
            "error": "jog refused: robot is not ready for SDK motion",
            "problems": problems,
            "status": status,
        }
    stop_event = threading.Event()
    thread = threading.Thread(
        target=continuous_jog_worker,
        args=(stop_event, axis, direction, float(speed_mm_s), int(speed_percent), z_min, "can0"),
        daemon=True,
    )
    with JOG_LOCK:
        JOG_CONTROL["stop"] = stop_event
        JOG_CONTROL["thread"] = thread
        JOG_CONTROL["heartbeat_deadline"] = time.time() + 0.60
        JOG_CONTROL["state"] = {
            "running": True,
            "axis": axis,
            "direction": direction,
            "speed_mm_s": speed_mm_s,
            "target_z_min_mm": z_min,
            "sent": 0,
        }
    thread.start()
    return {"ok": True, "jog": dict(JOG_CONTROL["state"]), "status": status}


def runtime_camera_kwargs() -> dict[str, Any]:
    import yaml

    config_path = ROOT / "config" / "runtime_config.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    return {
        "serial_number": str(config.get("camera_serial")) if config.get("camera_serial") else None,
        "color_width": int(config.get("color_width", 1280)),
        "color_height": int(config.get("color_height", 720)),
        "fps": int(config.get("camera_fps", 30)),
        "warmup_frames": 5,
    }


def encode_jpeg_bytes(image: Any, quality: int = JPEG_QUALITY) -> bytes:
    import cv2

    ok, encoded = cv2.imencode(
        ".jpg",
        image,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)],
    )
    if not ok:
        raise RuntimeError("failed to encode stream frame")
    return encoded.tobytes()


def bool_query(query: dict[str, list[str]], key: str, default: bool) -> bool:
    values = query.get(key)
    if not values:
        return default
    return values[0].lower() in {"1", "true", "yes", "on"}


def file_url(path: Path) -> str:
    return "/files/" + path.resolve().relative_to(ROOT).as_posix()


def path_under(root: Path, rel: str) -> Path:
    candidate = (root / rel).resolve()
    root_resolved = root.resolve()
    if not candidate.is_relative_to(root_resolved):
        raise ValueError("path escapes root")
    return candidate


class Handler(BaseHTTPRequestHandler):
    server_version = "PiperWebPanel/0.1"

    def do_GET(self) -> None:
        try:
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/":
                self.send_static("index.html")
            elif path.startswith("/static/"):
                self.send_file(path_under(STATIC, unquote(path[len("/static/") :])))
            elif path.startswith("/files/"):
                self.send_file(path_under(ROOT, unquote(path[len("/files/") :])))
            elif path == "/api/status":
                list_saved_poses()
                self.send_json({"ok": True, "state": STATE, "root": str(ROOT)})
            elif path == "/api/saved_poses":
                self.send_json(list_saved_poses())
            elif path == "/stream.mjpg":
                self.send_stream(parsed)
            else:
                self.send_error(404)
        except Exception as error:
            self.send_json({"ok": False, "error": str(error)}, status=500)

    def do_POST(self) -> None:
        try:
            parsed = urlparse(self.path)
            body = self.read_json()
            if parsed.path == "/api/capture":
                response = capture_snapshot(
                    parse_roi(body.get("roi")),
                    bool(body.get("include_detection_overlay", True)),
                )
            elif parsed.path == "/api/detect":
                response = detect_seam(
                    parse_roi(body.get("roi")),
                    str(body.get("mask_mode", "rgbd")),
                    bool(body.get("use_last_snapshot", True)),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/manual_seam_start":
                response = manual_seam_start(
                    list(body.get("start_px", [])),
                    parse_roi(body.get("roi")),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/manual_seam_end":
                response = manual_seam_end(
                    list(body.get("end_px", [])),
                    parse_roi(body.get("roi")),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/manual_seam_line":
                response = manual_seam_line(
                    list(body.get("start_px", [])),
                    list(body.get("end_px", [])),
                    parse_roi(body.get("roi")),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/build_three_cut_lines":
                response = build_three_cut_lines(
                    parse_roi(body.get("roi")),
                    parse_side_cut_px(body.get("side_cut_px")),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/calibration_target_from_pixel":
                response = calibration_target_from_pixel(
                    list(body.get("point_px", [])),
                    parse_roi(body.get("roi")),
                )
            elif parsed.path == "/api/calibration_validation_error":
                response = calibration_validation_error()
            elif parsed.path == "/api/read_pose":
                response = read_pose()
            elif parsed.path == "/api/restore":
                response = restore_control(bool(body.get("execute")), str(body.get("confirm", "")))
            elif parsed.path == "/api/save_far_pose":
                response = save_far_pose()
            elif parsed.path == "/api/move_far_pose":
                response = move_far_pose(
                    bool(body.get("execute")),
                    str(body.get("confirm", "")),
                    int(body.get("speed_percent", 5)),
                    parse_motion_mode(body.get("motion_mode")),
                )
            elif parsed.path == "/api/save_named_pose":
                response = save_named_pose(str(body.get("pose_name", "")))
            elif parsed.path == "/api/delete_saved_pose":
                response = delete_saved_pose(str(body.get("pose_name", "")))
            elif parsed.path == "/api/move_named_pose":
                response = move_named_pose(
                    str(body.get("pose_name", "")),
                    bool(body.get("execute")),
                    str(body.get("confirm", "")),
                    int(body.get("speed_percent", 5)),
                    parse_motion_mode(body.get("motion_mode")),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/move_xyz":
                response = move_manual_xyz(
                    [
                        float(body.get("x_mm")),
                        float(body.get("y_mm")),
                        float(body.get("z_mm")),
                    ],
                    bool(body.get("execute")),
                    str(body.get("confirm", "")),
                    int(body.get("speed_percent", 5)),
                    parse_motion_mode(body.get("motion_mode")),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/move_point":
                response = move_point(
                    str(body.get("point_name", "start")),
                    bool(body.get("execute")),
                    str(body.get("confirm", "")),
                    int(body.get("speed_percent", 5)),
                    parse_motion_mode(body.get("motion_mode")),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/move_line_segments":
                response = move_line_segments(
                    bool(body.get("execute")),
                    str(body.get("confirm", "")),
                    int(body.get("speed_percent", 5)),
                    parse_motion_mode(body.get("motion_mode")),
                    parse_segment_mm(body.get("segment_mm")),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/move_three_cut_lines":
                response = move_three_cut_lines(
                    bool(body.get("execute")),
                    str(body.get("confirm", "")),
                    int(body.get("speed_percent", 5)),
                    parse_motion_mode(body.get("motion_mode")),
                    parse_segment_mm(body.get("segment_mm")),
                    bool(body.get("use_segments", True)),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/move_three_cut_point":
                response = move_three_cut_point(
                    str(body.get("point_label", "")),
                    bool(body.get("execute")),
                    str(body.get("confirm", "")),
                    int(body.get("speed_percent", 5)),
                    parse_motion_mode(body.get("motion_mode")),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/run_cut_sequence":
                response = run_cut_sequence(
                    parse_roi(body.get("roi")),
                    str(body.get("mask_mode", "rgbd")),
                    parse_segment_mm(body.get("segment_mm")),
                    bool(body.get("use_segments", True)),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                    bool(body.get("execute")),
                    str(body.get("confirm", "")),
                    int(body.get("speed_percent", 5)),
                    parse_motion_mode(body.get("motion_mode")),
                )
            elif parsed.path == "/api/jog":
                response = jog(
                    str(body.get("axis")),
                    int(body.get("direction")),
                    float(body.get("step_mm", 5)),
                    bool(body.get("execute")),
                    str(body.get("confirm", "")),
                    int(body.get("speed_percent", 5)),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                    float(body.get("send_duration", 0.35)),
                    float(body.get("send_rate_hz", 80.0)),
                )
            elif parsed.path == "/api/jog_start":
                response = start_continuous_jog(
                    str(body.get("axis")),
                    int(body.get("direction")),
                    float(body.get("speed_mm_s", 30)),
                    bool(body.get("execute")),
                    str(body.get("confirm", "")),
                    int(body.get("speed_percent", 8)),
                    parse_target_z_min_mm(body.get("target_z_min_mm")),
                )
            elif parsed.path == "/api/jog_keepalive":
                response = keepalive_continuous_jog()
            elif parsed.path == "/api/jog_stop":
                response = stop_continuous_jog()
            else:
                self.send_error(404)
                return
            self.send_json(response, status=200 if response.get("ok") else 400)
        except Exception as error:
            self.send_json({"ok": False, "error": str(error)}, status=400)

    def read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length == 0:
            return {}
        payload = self.rfile.read(length).decode("utf-8")
        return json.loads(payload)

    def send_static(self, name: str) -> None:
        self.send_file(path_under(STATIC, name))

    def send_file(self, path: Path) -> None:
        if not path.is_file():
            self.send_error(404)
            return
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, data: dict[str, Any], status: int = 200) -> None:
        payload = json.dumps(data, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def send_stream(self, parsed: Any) -> None:
        query = parse_qs(parsed.query)
        roi = parse_roi(query.get("roi", [",".join(map(str, STATE["roi"]))])[0])
        show_roi = bool_query(query, "show_roi", True)
        show_box = bool_query(query, "show_box", True)
        show_seam = bool_query(query, "show_seam", False)
        quality = int(query.get("quality", [str(JPEG_QUALITY)])[0])
        quality = min(90, max(25, quality))
        target_fps = float(query.get("fps", ["10"])[0])
        target_fps = min(15.0, max(1.0, target_fps))

        stream_epoch, replaced_stream = claim_stream_request()
        if not STREAM_LOCK.acquire(timeout=8.0):
            self.send_json(
                {"ok": False, "error": "video stream is busy; previous stream did not stop in time"},
                status=503,
            )
            return
        if replaced_stream:
            time.sleep(1.5)

        command = [
            sys.executable,
            str(ROOT / "lib" / "stream_orbbec_sdk_mjpeg.py"),
            "--config",
            str(ROOT / "config" / "runtime_config.yaml"),
            "--roi",
            ",".join(map(str, roi)),
            "--quality",
            str(quality),
            "--fps",
            str(target_fps),
            "--warmup-frames",
            "5",
            "--frame-timeout-ms",
            "5000",
            "--overlay-json",
            json.dumps(
                {
                    "box_pixels": STATE.get("last_box_pixels"),
                    "seam_pixels": STATE.get("last_seam_pixels"),
                },
                ensure_ascii=False,
            ),
        ]
        if show_roi:
            command.append("--show-roi")
        if show_box:
            command.append("--show-box")
        if show_seam:
            command.append("--show-seam")
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if not register_stream_process(stream_epoch, process):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            STREAM_LOCK.release()
            self.send_json({"ok": False, "error": "video stream was replaced by a newer request"}, status=409)
            return

        first_chunk = read_stream_chunk(process, timeout_s=32.0)
        if not first_chunk:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            stderr = ""
            if process.stderr is not None:
                stderr = process.stderr.read().decode("utf-8", errors="replace").strip()
            if stderr:
                print(f"[web] stream process stderr:\n{stderr[-4000:]}", flush=True)
            clear_stream_process(process)
            STREAM_LOCK.release()
            self.send_json({"ok": False, "error": "camera stream did not produce a frame"}, status=503)
            return

        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={STREAM_BOUNDARY}")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        try:
            try:
                self.wfile.write(first_chunk)
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return
            while True:
                if stream_was_replaced(stream_epoch):
                    break
                if process.stdout is None:
                    break
                chunk = read_stream_chunk(process, timeout_s=2.0)
                if not chunk:
                    break
                try:
                    self.wfile.write(chunk)
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    break
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            stderr = ""
            if process.stderr is not None:
                stderr = process.stderr.read().decode("utf-8", errors="replace").strip()
            if stderr:
                print(f"[web] stream process stderr:\n{stderr[-4000:]}", flush=True)
            clear_stream_process(process)
            STREAM_LOCK.release()

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[web] {self.address_string()} {fmt % args}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    ensure_dirs()
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Serving web control panel at http://{args.host}:{args.port}", flush=True)
    httpd.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
