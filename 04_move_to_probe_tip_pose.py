#!/usr/bin/env python3
"""Dry-run or move the calibrated probe tip to one base_link pose.

Default mode is dry-run. Real motion requires:

    --execute --confirm EXECUTE
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from lib.piper_sdk_control_utils import (
    DEFAULT_TCP_CALIBRATION,
    ProbePose,
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


def pose_from_args(args: argparse.Namespace) -> ProbePose:
    if args.pose_json:
        return load_pose_json(args.pose_json)
    missing = [name for name in ("x", "y", "z", "rx", "ry", "rz") if getattr(args, name) is None]
    if missing:
        raise ValueError("Provide either --pose-json or all of --x --y --z --rx --ry --rz")
    scale = 0.001 if args.unit == "mm" else 1.0
    xyz = [float(args.x) * scale, float(args.y) * scale, float(args.z) * scale]
    rpy = finite_vector([args.rx, args.ry, args.rz], 3, "rpy_deg")
    return ProbePose(xyz, rpy)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-json", type=Path, help="JSON containing corrected_probe_tip or one_line_xyz_m_rpy_deg")
    parser.add_argument("--x", type=float, help="TCP tip pose x")
    parser.add_argument("--y", type=float, help="TCP tip pose y")
    parser.add_argument("--z", type=float, help="TCP tip pose z")
    parser.add_argument("--unit", choices=("m", "mm"), default="m", help="Unit for --x/--y/--z")
    parser.add_argument("--rx", type=float, help="TCP pose RX, degree")
    parser.add_argument("--ry", type=float, help="TCP pose RY, degree")
    parser.add_argument("--rz", type=float, help="TCP pose RZ, degree")
    parser.add_argument("--z-offset-m", type=float, default=0.0)
    parser.add_argument("--can-name", default="can0")
    parser.add_argument("--tcp-calibration", type=Path, default=DEFAULT_TCP_CALIBRATION)
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
        target = pose_from_args(args)
        target_xyz = [target.xyz_m[0], target.xyz_m[1], target.xyz_m[2] + float(args.z_offset_m)]
        flange_target = flange_from_tip_target_m(target_xyz, target.rpy_deg, tcp_offset)
        validate_xyz_workspace(
            "target_tip",
            target_xyz,
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
        cmd = end_pose_cmd_from_flange_pose(flange_target, target.rpy_deg)

        print(f"tcp_calibration={args.tcp_calibration} status={tcp_status}")
        print(f"execute={args.execute}")
        print(f"target_probe_tip_pose={[*target_xyz, *target.rpy_deg]}")
        print(f"command_flange_m={flange_target}")
        print(f"EndPoseCtrl cmd={cmd}")

        if not args.execute:
            print("dry-run only. Add --execute --confirm EXECUTE to move the robot.")
            return 0

        piper = connect_piper(args.can_name, enable=True, enable_timeout=args.enable_timeout)
        time.sleep(args.feedback_wait)
        before = read_probe_tip_pose(piper, tcp_offset)
        print("before_probe_tip:", before["corrected_probe_tip"]["one_line_xyz_m_rpy_deg"])
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
