#!/usr/bin/env python3
"""Scan a TCP probe-tip workspace for coarse PiPER reachability.

This script does not move the robot. It checks whether sampled TCP targets can
be converted into SDK flange targets and pass the configured safe workspace
limits. With ``--connect`` it also reads the current TCP pose/status from the
robot and can use the current RPY as the scan orientation.

Important: this is a conservative workspace filter, not the controller's
internal IK solver. A point marked reachable here still needs a real IK solver
or slow physical validation before cutting.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parent
DEFAULT_TCP_CALIBRATION = ROOT / "config" / "tcp_offset_m_rad.yaml"
DEFAULT_OUTPUT_DIR = ROOT / "outputs" / "reachability"


def now_id() -> str:
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")


def parse_range_mm(text: str, name: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in str(text).split(",")]
    if len(parts) != 3:
        raise ValueError(f"{name} must be min,max,step in mm")
    start, stop, step = [float(part) for part in parts]
    if not all(math.isfinite(value) for value in (start, stop, step)):
        raise ValueError(f"{name} contains NaN or infinity")
    if step <= 0:
        raise ValueError(f"{name} step must be positive")
    if stop < start:
        raise ValueError(f"{name} max must be >= min")
    return start, stop, step


def values_mm(spec: tuple[float, float, float]) -> list[float]:
    start, stop, step = spec
    values: list[float] = []
    current = start
    while current <= stop + step * 1.0e-9:
        values.append(round(current, 6))
        current += step
    return values


def xyz_mm_to_m(xyz_mm: list[float]) -> list[float]:
    return [value / 1000.0 for value in xyz_mm]


def finite_vector(values: Any, expected_len: int, name: str) -> list[float]:
    if len(values) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} values")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} contains NaN or infinity")
    return result


def load_tcp_offset_m_rad(path: str | Path = DEFAULT_TCP_CALIBRATION) -> tuple[list[float], str]:
    text = Path(path).expanduser().read_text(encoding="utf-8").splitlines()
    for index, line in enumerate(text):
        if line.strip() != "tcp_offset_m_rad:":
            continue
        values: list[float] = []
        for row in text[index + 1 : index + 7]:
            stripped = row.strip()
            if not stripped.startswith("-"):
                break
            values.append(float(stripped[1:].strip()))
        if len(values) == 6:
            return values, "loaded_from_yaml"
    raise ValueError(f"{path} has no tcp_offset_m_rad list")


def euler_xyz_deg_to_matrix(values: list[float]) -> list[list[float]]:
    rx, ry, rz = [math.radians(value) for value in finite_vector(values, 3, "rpy_deg")]
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [
        [cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
        [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
        [-sy, sx * cy, cx * cy],
    ]


def mat_vec_mul(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(row[index] * vector[index] for index in range(3)) for row in matrix]


def flange_from_tip_target_m(
    tip_xyz_m: list[float],
    rpy_deg: list[float],
    tcp_offset_m_rad: list[float],
) -> list[float]:
    tip = finite_vector(tip_xyz_m, 3, "tip_xyz_m")
    tcp = finite_vector(tcp_offset_m_rad, 6, "tcp_offset_m_rad")
    rotated_tcp = mat_vec_mul(euler_xyz_deg_to_matrix(rpy_deg), tcp[:3])
    return [tip[index] - rotated_tcp[index] for index in range(3)]


def m_to_piper_pos(value_m: float) -> int:
    return int(round(float(value_m) * 1_000_000.0))


def deg_to_piper_angle(value_deg: float) -> int:
    return int(round(float(value_deg) * 1000.0))


def end_pose_cmd_from_flange_pose(
    flange_xyz_m: list[float],
    rpy_deg: list[float],
) -> tuple[int, int, int, int, int, int]:
    xyz = finite_vector(flange_xyz_m, 3, "flange_xyz_m")
    rpy = finite_vector(rpy_deg, 3, "rpy_deg")
    return (
        m_to_piper_pos(xyz[0]),
        m_to_piper_pos(xyz[1]),
        m_to_piper_pos(xyz[2]),
        deg_to_piper_angle(rpy[0]),
        deg_to_piper_angle(rpy[1]),
        deg_to_piper_angle(rpy[2]),
    )


def validate_xyz_workspace(
    label: str,
    xyz_m: list[float],
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> None:
    x, y, z = finite_vector(xyz_m, 3, label)
    for axis, value, low, high in (
        ("x", x, x_min, x_max),
        ("y", y, y_min, y_max),
        ("z", z, z_min, z_max),
    ):
        if value < low or value > high:
            raise RuntimeError(
                f"{label} {axis}={value:.6f} m is outside [{low:.6f}, {high:.6f}] m"
            )


def radius_mm(xyz_m: list[float]) -> float:
    return math.sqrt(sum(value * value for value in xyz_m)) * 1000.0


def workspace_ok(label: str, xyz_m: list[float], args: argparse.Namespace) -> tuple[bool, str | None]:
    try:
        validate_xyz_workspace(
            label,
            xyz_m,
            x_min=args.x_min_mm / 1000.0,
            x_max=args.x_max_mm / 1000.0,
            y_min=args.y_min_mm / 1000.0,
            y_max=args.y_max_mm / 1000.0,
            z_min=args.z_min_mm / 1000.0,
            z_max=args.z_max_mm / 1000.0,
        )
        return True, None
    except RuntimeError as error:
        return False, str(error)


def classify_target(
    xyz_mm: list[float],
    rpy_deg: list[float],
    tcp_offset: Any,
    args: argparse.Namespace,
) -> dict[str, Any]:
    tip_m = xyz_mm_to_m(xyz_mm)
    tip_ok, tip_error = workspace_ok("target_tip", tip_m, args)
    try:
        flange_m = flange_from_tip_target_m(tip_m, rpy_deg, tcp_offset)
        flange_ok, flange_error = workspace_ok("command_flange", flange_m, args)
        cmd = end_pose_cmd_from_flange_pose(flange_m, rpy_deg)
        command_ok = all(-(2**31) <= int(value) <= 2**31 - 1 for value in cmd)
        command_error = None if command_ok else "EndPoseCtrl command exceeds int32 range"
    except (RuntimeError, ValueError) as error:
        flange_m = None
        flange_ok = False
        flange_error = str(error)
        cmd = None
        command_ok = False
        command_error = str(error)

    r_mm = radius_mm(tip_m)
    radius_ok = args.min_radius_mm <= r_mm <= args.max_radius_mm
    radius_error = None if radius_ok else (
        f"tip radius {r_mm:.3f} mm outside [{args.min_radius_mm:.3f}, {args.max_radius_mm:.3f}] mm"
    )
    reachable = bool(tip_ok and flange_ok and command_ok and radius_ok)
    errors = [item for item in (tip_error, flange_error, command_error, radius_error) if item]
    return {
        "xyz_mm": xyz_mm,
        "rpy_deg": rpy_deg,
        "reachable": reachable,
        "checks": {
            "tip_workspace_ok": tip_ok,
            "flange_workspace_ok": flange_ok,
            "radius_ok": radius_ok,
            "command_ok": command_ok,
        },
        "tip_radius_mm": r_mm,
        "flange_xyz_mm": [value * 1000.0 for value in flange_m] if flange_m is not None else None,
        "end_pose_ctrl": list(cmd) if cmd is not None else None,
        "errors": errors,
    }


def load_orientation(args: argparse.Namespace) -> tuple[list[float], dict[str, Any] | None]:
    if args.orientation_source == "fixed":
        return finite_vector([args.rx, args.ry, args.rz], 3, "rpy_deg"), None
    if args.orientation_source == "current" and not args.connect:
        raise ValueError("--orientation-source current requires --connect")

    sys.path.insert(0, str(ROOT / "lib"))
    from piper_sdk_control_utils import (
        arm_status_summary,
        command_ready_problems,
        connect_piper,
        load_tcp_offset_m_rad as load_tcp_offset_with_yaml,
        read_probe_tip_pose,
    )

    tcp_offset, _ = load_tcp_offset_with_yaml(args.tcp_calibration)
    piper = connect_piper(args.can_name, enable=False, piper_init=False)
    time.sleep(args.feedback_wait)
    pose = read_probe_tip_pose(piper, tcp_offset)
    status = arm_status_summary(piper)
    report = {
        "pose": pose,
        "status": status,
        "command_ready_problems": command_ready_problems(status),
    }
    return [float(value) for value in pose["corrected_probe_tip"]["rpy_deg"]], report


def scan(args: argparse.Namespace) -> dict[str, Any]:
    tcp_offset, tcp_status = load_tcp_offset_m_rad(args.tcp_calibration)
    rpy_deg, robot_report = load_orientation(args)
    xs = values_mm(parse_range_mm(args.x_range_mm, "x-range-mm"))
    ys = values_mm(parse_range_mm(args.y_range_mm, "y-range-mm"))
    zs = values_mm(parse_range_mm(args.z_range_mm, "z-range-mm"))

    points: list[dict[str, Any]] = []
    reachable_count = 0
    per_z: dict[str, dict[str, int]] = {}
    for z in zs:
        key = f"{z:.3f}"
        per_z[key] = {"total": 0, "reachable": 0}
        for y in ys:
            for x in xs:
                item = classify_target([x, y, z], rpy_deg, tcp_offset, args)
                points.append(item)
                per_z[key]["total"] += 1
                if item["reachable"]:
                    reachable_count += 1
                    per_z[key]["reachable"] += 1

    total = len(points)
    reachable_points = [point for point in points if point["reachable"]]
    if reachable_points:
        bounds = {
            axis: {
                "min_mm": min(point["xyz_mm"][index] for point in reachable_points),
                "max_mm": max(point["xyz_mm"][index] for point in reachable_points),
            }
            for index, axis in enumerate(("x", "y", "z"))
        }
    else:
        bounds = None

    return {
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "note": "coarse workspace filter; not controller internal IK",
        "frame_id": "base_link",
        "unit": "mm_deg",
        "tcp_calibration": str(args.tcp_calibration),
        "tcp_status": tcp_status,
        "orientation_source": args.orientation_source,
        "rpy_deg": rpy_deg,
        "scan_ranges_mm": {
            "x": args.x_range_mm,
            "y": args.y_range_mm,
            "z": args.z_range_mm,
        },
        "safe_workspace_limits_mm": {
            "x": [args.x_min_mm, args.x_max_mm],
            "y": [args.y_min_mm, args.y_max_mm],
            "z": [args.z_min_mm, args.z_max_mm],
        },
        "radius_filter_mm": [args.min_radius_mm, args.max_radius_mm],
        "summary": {
            "total": total,
            "reachable": reachable_count,
            "unreachable": total - reachable_count,
            "reachable_ratio": reachable_count / total if total else 0.0,
            "reachable_bounds_mm": bounds,
            "per_z": per_z,
        },
        "robot_report": robot_report,
        "points": points,
    }


def write_csv(path: Path, report: dict[str, Any]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "x_mm",
                "y_mm",
                "z_mm",
                "rx_deg",
                "ry_deg",
                "rz_deg",
                "reachable",
                "tip_radius_mm",
                "flange_x_mm",
                "flange_y_mm",
                "flange_z_mm",
                "errors",
            ]
        )
        for point in report["points"]:
            flange = point["flange_xyz_mm"] or ["", "", ""]
            writer.writerow(
                [
                    *point["xyz_mm"],
                    *point["rpy_deg"],
                    int(bool(point["reachable"])),
                    point["tip_radius_mm"],
                    *flange,
                    "; ".join(point["errors"]),
                ]
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x-range-mm", default="150,650,25", help="TCP X scan range: min,max,step")
    parser.add_argument("--y-range-mm", default="-300,300,25", help="TCP Y scan range: min,max,step")
    parser.add_argument("--z-range-mm", default="80,220,20", help="TCP Z scan range: min,max,step")
    parser.add_argument("--orientation-source", choices=("fixed", "current"), default="fixed")
    parser.add_argument("--rx", type=float, default=173.0, help="Fixed TCP RX in degree")
    parser.add_argument("--ry", type=float, default=-5.0, help="Fixed TCP RY in degree")
    parser.add_argument("--rz", type=float, default=163.0, help="Fixed TCP RZ in degree")
    parser.add_argument("--tcp-calibration", type=Path, default=DEFAULT_TCP_CALIBRATION)
    parser.add_argument("--can-name", default="can0")
    parser.add_argument("--connect", action="store_true", help="Read current robot pose/status; no motion")
    parser.add_argument("--feedback-wait", type=float, default=0.5)
    parser.add_argument("--x-min-mm", type=float, default=0.0)
    parser.add_argument("--x-max-mm", type=float, default=800.0)
    parser.add_argument("--y-min-mm", type=float, default=-450.0)
    parser.add_argument("--y-max-mm", type=float, default=450.0)
    parser.add_argument("--z-min-mm", type=float, default=-150.0)
    parser.add_argument("--z-max-mm", type=float, default=500.0)
    parser.add_argument("--min-radius-mm", type=float, default=80.0)
    parser.add_argument("--max-radius-mm", type=float, default=626.0)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--output-stem", default="")
    return parser.parse_args()


def main() -> int:
    try:
        args = parse_args()
        if args.connect:
            args.orientation_source = "current"
        report = scan(args)
        args.output_dir.mkdir(parents=True, exist_ok=True)
        stem = args.output_stem or f"reachability_{now_id()}"
        json_path = args.output_dir / f"{stem}.json"
        csv_path = args.output_dir / f"{stem}.csv"
        json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        write_csv(csv_path, report)
        summary = report["summary"]
        print("=== PiPER TCP reachability scan ===")
        print("mode: coarse workspace filter; no motion")
        print(f"frame: {report['frame_id']}, unit: {report['unit']}")
        print(f"rpy_deg: {report['rpy_deg']}")
        print(f"total={summary['total']} reachable={summary['reachable']} ratio={summary['reachable_ratio']:.3f}")
        print(f"reachable_bounds_mm={summary['reachable_bounds_mm']}")
        if report.get("robot_report"):
            problems = report["robot_report"].get("command_ready_problems") or []
            print(f"robot_command_ready_problems={problems}")
        print(f"Saved JSON: {json_path}")
        print(f"Saved CSV:  {csv_path}")
        return 0
    except Exception as error:
        print(f"ERROR: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
