#!/usr/bin/env python3
"""Dry-run or move the calibrated probe tip to one base_link point.

Default mode is dry-run. Real motion requires:

    --execute --confirm EXECUTE
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any

from lib.piper_sdk_control_utils import (
    DEFAULT_TCP_CALIBRATION,
    connect_piper,
    end_pose_cmd_from_flange_pose,
    finite_vector,
    flange_from_tip_target_m,
    load_pose_json,
    load_tcp_offset_m_rad,
    read_probe_tip_pose,
    require_command_ready,
    send_end_pose_repeated,
    validate_xyz_workspace,
)


CONFIRMATION_TOKEN = "EXECUTE"


def load_point_json(path: str | Path, point_name: str | None) -> list[float]:
    data = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if isinstance(data.get("probe_tip_contact_targets_base"), dict):
        block = data["probe_tip_contact_targets_base"]
        key = f"{point_name}_m" if point_name in ("start", "end", "mid") else "start_m"
        if key in block:
            return finite_vector(block[key], 3, f"probe_tip_contact_targets_base.{key}")
    if isinstance(data.get("base_link_line"), dict):
        block = data["base_link_line"]
        key = f"{point_name}_m" if point_name in ("start", "end", "mid") else "start_m"
        if key in block:
            return finite_vector(block[key], 3, f"base_link_line.{key}")
    if data.get("xyz_m") is not None:
        return finite_vector(data["xyz_m"], 3, "xyz_m")
    if data.get("target_base_m") is not None:
        return finite_vector(data["target_base_m"], 3, "target_base_m")
    raise ValueError(
        f"{path} does not contain probe_tip_contact_targets_base/base_link_line/xyz_m"
    )


def target_from_args(args: argparse.Namespace) -> list[float]:
    if args.point_json:
        return load_point_json(args.point_json, args.point_name)
    missing = [name for name in ("x", "y", "z") if getattr(args, name) is None]
    if missing:
        raise ValueError("Provide either --point-json or all of --x --y --z")
    scale = 0.001 if args.unit == "mm" else 1.0
    return [float(args.x) * scale, float(args.y) * scale, float(args.z) * scale]


def rpy_from_args(args: argparse.Namespace, piper: Any | None, tcp_offset: Any) -> list[float]:
    if args.orientation_source == "fixed":
        return [float(args.rx), float(args.ry), float(args.rz)]
    if args.orientation_source == "json":
        if not args.orientation_json:
            raise ValueError("--orientation-source json requires --orientation-json")
        return load_pose_json(args.orientation_json).rpy_deg
    if args.orientation_source == "current":
        if piper is None:
            raise RuntimeError("current orientation requires a PiPER connection")
        return read_probe_tip_pose(piper, tcp_offset)["corrected_probe_tip"]["rpy_deg"]
    raise ValueError(f"unknown orientation source: {args.orientation_source}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--point-json", type=Path, help="JSON with probe_tip_contact_targets_base or xyz_m")
    parser.add_argument("--point-name", choices=("start", "end", "mid"), default="start")
    parser.add_argument("--x", type=float, help="TCP tip target x")
    parser.add_argument("--y", type=float, help="TCP tip target y")
    parser.add_argument("--z", type=float, help="TCP tip target z")
    parser.add_argument("--unit", choices=("m", "mm"), default="m", help="Unit for --x/--y/--z")
    parser.add_argument("--z-offset-m", type=float, default=0.0)
    parser.add_argument("--can-name", default="can0")
    parser.add_argument("--tcp-calibration", type=Path, default=DEFAULT_TCP_CALIBRATION)
    parser.add_argument("--orientation-source", choices=("current", "fixed", "json"), default="current")
    parser.add_argument("--orientation-json", type=Path)
    parser.add_argument("--rx", type=float, default=177.0, help="Fixed RX in degree")
    parser.add_argument("--ry", type=float, default=0.0, help="Fixed RY in degree")
    parser.add_argument("--rz", type=float, default=145.0, help="Fixed RZ in degree")
    parser.add_argument("--motion-mode", choices=("moveP", "moveL"), default="moveL")
    parser.add_argument("--speed-percent", type=int, default=5)
    parser.add_argument("--send-duration", type=float, default=3.0)
    parser.add_argument("--send-rate-hz", type=float, default=50.0)
    parser.add_argument("--feedback-wait", type=float, default=0.3)
    parser.add_argument("--enable-timeout", type=float, default=5.0)
    parser.add_argument("--wait-after-send", type=float, default=1.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--allow-not-normal-status", action="store_true")
    parser.add_argument("--x-min", type=float, default=0.0)
    parser.add_argument("--x-max", type=float, default=0.8)
    parser.add_argument("--y-min", type=float, default=-0.45)
    parser.add_argument("--y-max", type=float, default=0.45)
    parser.add_argument("--z-min", type=float, default=-0.15)
    parser.add_argument("--z-max", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if not (1 <= args.speed_percent <= 100):
            raise ValueError("--speed-percent must be in range 1..100")
        if args.execute and args.confirm != CONFIRMATION_TOKEN:
            raise ValueError(f"Real motion requires --confirm {CONFIRMATION_TOKEN}")

        tcp_offset, tcp_status = load_tcp_offset_m_rad(args.tcp_calibration)
        target_tip = target_from_args(args)
        target_tip[2] += float(args.z_offset_m)

        needs_connection = args.execute or args.orientation_source == "current"
        piper = connect_piper(
            args.can_name,
            enable=args.execute,
            enable_timeout=args.enable_timeout,
        ) if needs_connection else None
        if piper is not None:
            time.sleep(args.feedback_wait)
            current = read_probe_tip_pose(piper, tcp_offset)
            print("current_probe_tip:", current["corrected_probe_tip"]["one_line_xyz_m_rpy_deg"])

        rpy_deg = rpy_from_args(args, piper, tcp_offset)
        flange_target = flange_from_tip_target_m(target_tip, rpy_deg, tcp_offset)
        validate_xyz_workspace(
            "target_tip",
            target_tip,
            x_min=args.x_min,
            x_max=args.x_max,
            y_min=args.y_min,
            y_max=args.y_max,
            z_min=args.z_min,
            z_max=args.z_max,
        )
        validate_xyz_workspace(
            "command_flange",
            flange_target,
            x_min=args.x_min,
            x_max=args.x_max,
            y_min=args.y_min,
            y_max=args.y_max,
            z_min=args.z_min,
            z_max=args.z_max,
        )
        cmd = end_pose_cmd_from_flange_pose(flange_target, rpy_deg)

        print(f"tcp_calibration={args.tcp_calibration} status={tcp_status}")
        print(f"execute={args.execute}")
        print(f"target_probe_tip_m={target_tip}")
        print(f"target_rpy_deg={rpy_deg}")
        print(f"command_flange_m={flange_target}")
        print(f"EndPoseCtrl cmd={cmd}")

        if not args.execute:
            print("dry-run only. Add --execute --confirm EXECUTE to move the robot.")
            return 0

        require_command_ready(piper, allow_not_normal=args.allow_not_normal_status)
        sent = send_end_pose_repeated(
            piper,
            cmd,
            motion_mode=args.motion_mode,
            speed_percent=args.speed_percent,
            send_duration=args.send_duration,
            send_rate_hz=args.send_rate_hz,
        )
        print(f"sent frames={sent}")
        time.sleep(args.wait_after_send)
        after = read_probe_tip_pose(piper, tcp_offset)
        print("after_probe_tip:", after["corrected_probe_tip"]["one_line_xyz_m_rpy_deg"])
        return 0
    except (OSError, RuntimeError, ValueError, TimeoutError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
