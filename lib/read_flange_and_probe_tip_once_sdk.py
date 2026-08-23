#!/usr/bin/env python3
"""Print raw PIPER flange feedback and the calibrated probe-tip position."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time

import numpy as np
import yaml


def euler_xyz_deg_to_matrix(values: np.ndarray) -> np.ndarray:
    """PIPER fixed-axis XYZ convention: R = Rz @ Ry @ Rx."""

    rx, ry, rz = np.deg2rad(values)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.asarray(
        [
            [cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
            [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
            [-sy, sx * cy, cx * cy],
        ],
        dtype=np.float64,
    )


def load_tcp(path: str) -> np.ndarray:
    with open(path, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    values = np.asarray(document["result"]["tcp_offset_m_rad"], dtype=np.float64)
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ValueError("invalid tcp_offset_m_rad")
    if not np.allclose(values[3:], 0.0, atol=1.0e-12):
        raise ValueError("only translation-only TCP calibration is supported")
    return values


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tcp-calibration", required=True)
    parser.add_argument("--can-name", default="can0")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    from piper_sdk import C_PiperInterface_V2

    tcp = load_tcp(args.tcp_calibration)
    piper = C_PiperInterface_V2(args.can_name)
    # Feedback-only connection: the returned pose fields are the same fields
    # used by the operator's snippet, without sending initialization queries.
    piper.ConnectPort(piper_init=False)
    try:
        time.sleep(0.5)
        wrapper = piper.GetArmEndPoseMsgs()
        pose = wrapper.end_pose
        joint_wrapper = piper.GetArmJointMsgs()
        joint_state = joint_wrapper.joint_state
        joint_raw = [
            int(joint_state.joint_1),
            int(joint_state.joint_2),
            int(joint_state.joint_3),
            int(joint_state.joint_4),
            int(joint_state.joint_5),
            int(joint_state.joint_6),
        ]
        joint_deg = [value / 1000.0 for value in joint_raw]
        flange_position = np.asarray(
            [pose.X_axis, pose.Y_axis, pose.Z_axis], dtype=np.float64
        ) / 1_000_000.0
        flange_rpy_deg = np.asarray(
            [pose.RX_axis, pose.RY_axis, pose.RZ_axis], dtype=np.float64
        ) / 1000.0
        rotation = euler_xyz_deg_to_matrix(flange_rpy_deg)
        rotated_tcp = rotation @ tcp[:3]
        probe_tip = flange_position + rotated_tcp
        report = {
            "status": "feedback_only_no_motion",
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "frame_id": "base_link",
            "raw_code_output_flange": {
                "xyz_m": flange_position.tolist(),
                "rpy_deg": flange_rpy_deg.tolist(),
                "one_line": [
                    *flange_position.tolist(),
                    *flange_rpy_deg.tolist(),
                ],
            },
            "tcp_correction": {
                "formula": "p_base_tip = p_base_flange + Rz(rz)@Ry(ry)@Rx(rx)@t_flange_tip",
                "tcp_offset_flange_m": tcp[:3].tolist(),
                "tcp_offset_rotated_base_m": rotated_tcp.tolist(),
            },
            "corrected_probe_tip": {
                "xyz_m": probe_tip.tolist(),
                "xyz_mm": (probe_tip * 1000.0).tolist(),
                "rpy_deg": flange_rpy_deg.tolist(),
                "one_line_xyz_m_rpy_deg": [
                    *probe_tip.tolist(),
                    *flange_rpy_deg.tolist(),
                ],
            },
            "joint_feedback": {
                "unit_note": "joint raw unit is 0.001 degree",
                "joint_names": ["J1", "J2", "J3", "J4", "J5", "J6"],
                "raw_0p001deg": joint_raw,
                "deg": joint_deg,
                "one_line_deg": joint_deg,
                "feedback_timestamp": float(joint_wrapper.time_stamp),
                "hz": float(joint_wrapper.Hz),
                "valid_feedback": bool(joint_wrapper.time_stamp),
            },
            "feedback_timestamp": float(wrapper.time_stamp),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if args.output:
            output = args.output.expanduser().resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"Saved: {output}")
    finally:
        piper.DisconnectPort(thread_timeout=0.5)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
