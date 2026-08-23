#!/usr/bin/env python3
"""Shared PiPER SDK helpers for probe-tip control.

Public coordinates in this package use:
  - frame: base_link
  - position unit: meter
  - orientation unit: degree
  - end effector: calibrated physical probe tip

The PiPER SDK feedback and EndPoseCtrl command are treated as raw J6/flange
pose, so motion commands convert probe-tip targets back to flange targets.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import time
from typing import Any, Sequence

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TCP_CALIBRATION = ROOT / "config/tcp_offset_m_rad.yaml"


@dataclass(frozen=True)
class ProbePose:
    xyz_m: list[float]
    rpy_deg: list[float]

    def one_line(self) -> list[float]:
        return [*self.xyz_m, *self.rpy_deg]


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def finite_vector(values: Sequence[Any], expected_len: int, name: str) -> list[float]:
    if len(values) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} values")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} contains NaN or infinity")
    return result


def load_tcp_offset_m_rad(path: str | Path = DEFAULT_TCP_CALIBRATION) -> tuple[np.ndarray, str]:
    with Path(path).expanduser().open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
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
        raise ValueError(f"{path} has no tcp_offset_m_rad")
    values = np.asarray(finite_vector(block["tcp_offset_m_rad"], 6, "tcp_offset_m_rad"))
    if not np.allclose(values[3:], 0.0, atol=1.0e-12):
        raise ValueError("only translation-only TCP calibration is supported")
    status = str(block.get("status") or document.get("status") or "unknown")
    return values, status


def euler_xyz_deg_to_matrix(values: Sequence[float]) -> np.ndarray:
    """PIPER fixed-axis XYZ convention: R = Rz @ Ry @ Rx."""

    rx, ry, rz = np.deg2rad(finite_vector(values, 3, "rpy_deg"))
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


def flange_from_tip_target_m(
    tip_xyz_m: Sequence[float],
    rpy_deg: Sequence[float],
    tcp_offset_m_rad: Sequence[float],
) -> list[float]:
    tip = np.asarray(finite_vector(tip_xyz_m, 3, "tip_xyz_m"), dtype=np.float64)
    tcp = np.asarray(finite_vector(tcp_offset_m_rad, 6, "tcp_offset_m_rad"), dtype=np.float64)
    rotated_tcp = euler_xyz_deg_to_matrix(rpy_deg) @ tcp[:3]
    return (tip - rotated_tcp).tolist()


def tip_from_flange_pose_m(
    flange_xyz_m: Sequence[float],
    rpy_deg: Sequence[float],
    tcp_offset_m_rad: Sequence[float],
) -> tuple[list[float], list[float]]:
    flange = np.asarray(finite_vector(flange_xyz_m, 3, "flange_xyz_m"), dtype=np.float64)
    tcp = np.asarray(finite_vector(tcp_offset_m_rad, 6, "tcp_offset_m_rad"), dtype=np.float64)
    rotated_tcp = euler_xyz_deg_to_matrix(rpy_deg) @ tcp[:3]
    return (flange + rotated_tcp).tolist(), rotated_tcp.tolist()


def m_to_piper_pos(value_m: float) -> int:
    """Convert meter to PiPER EndPoseCtrl position unit, 0.001 mm."""

    return int(round(float(value_m) * 1_000_000.0))


def deg_to_piper_angle(value_deg: float) -> int:
    return int(round(float(value_deg) * 1000.0))


def rpy_raw_to_deg(raw: Sequence[int]) -> list[float]:
    if len(raw) != 3:
        raise ValueError("raw RPY must contain 3 values")
    return [int(value) / 1000.0 for value in raw]


def end_pose_cmd_from_flange_pose(
    flange_xyz_m: Sequence[float],
    rpy_deg: Sequence[float],
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


def connect_piper(
    can_name: str,
    *,
    enable: bool,
    enable_timeout: float = 5.0,
    piper_init: bool = True,
) -> Any:
    from piper_sdk import C_PiperInterface_V2

    piper = C_PiperInterface_V2(can_name)
    if piper_init:
        piper.ConnectPort()
    else:
        piper.ConnectPort(piper_init=False)
    if enable:
        deadline = time.time() + enable_timeout
        while not piper.EnablePiper():
            if time.time() > deadline:
                raise TimeoutError("Timed out enabling PiPER.")
            time.sleep(0.02)
    return piper


def read_flange_feedback(piper: Any) -> tuple[list[float], list[float], float | None]:
    wrapper = piper.GetArmEndPoseMsgs()
    pose = wrapper.end_pose
    flange_xyz_m = [
        int(pose.X_axis) / 1_000_000.0,
        int(pose.Y_axis) / 1_000_000.0,
        int(pose.Z_axis) / 1_000_000.0,
    ]
    rpy_deg = [
        int(pose.RX_axis) / 1000.0,
        int(pose.RY_axis) / 1000.0,
        int(pose.RZ_axis) / 1000.0,
    ]
    return flange_xyz_m, rpy_deg, getattr(wrapper, "time_stamp", None)


def read_joint_feedback(piper: Any) -> dict[str, Any]:
    wrapper = piper.GetArmJointMsgs()
    joint = wrapper.joint_state
    raw = [
        int(joint.joint_1),
        int(joint.joint_2),
        int(joint.joint_3),
        int(joint.joint_4),
        int(joint.joint_5),
        int(joint.joint_6),
    ]
    deg = [value / 1000.0 for value in raw]
    timestamp = getattr(wrapper, "time_stamp", None)
    hz = getattr(wrapper, "Hz", None)
    return {
        "unit_note": "joint raw unit is 0.001 degree",
        "joint_names": ["J1", "J2", "J3", "J4", "J5", "J6"],
        "raw_0p001deg": raw,
        "deg": deg,
        "one_line_deg": deg,
        "feedback_timestamp": float(timestamp) if timestamp is not None else None,
        "hz": float(hz) if hz is not None else None,
        "valid_feedback": bool(timestamp),
    }


def read_probe_tip_pose(
    piper: Any,
    tcp_offset_m_rad: Sequence[float],
) -> dict[str, Any]:
    flange_xyz_m, rpy_deg, timestamp = read_flange_feedback(piper)
    probe_xyz_m, rotated_tcp = tip_from_flange_pose_m(flange_xyz_m, rpy_deg, tcp_offset_m_rad)
    joint_feedback = read_joint_feedback(piper)
    return {
        "status": "feedback_only_no_motion",
        "created_at": now_iso(),
        "frame_id": "base_link",
        "raw_code_output_flange": {
            "xyz_m": flange_xyz_m,
            "xyz_mm": [value * 1000.0 for value in flange_xyz_m],
            "rpy_deg": rpy_deg,
            "one_line_xyz_m_rpy_deg": [*flange_xyz_m, *rpy_deg],
        },
        "tcp_correction": {
            "formula": "p_base_tip = p_base_flange + Rz(rz)@Ry(ry)@Rx(rx)@t_flange_tip",
            "tcp_offset_flange_m": list(tcp_offset_m_rad[:3]),
            "tcp_offset_rotated_base_m": rotated_tcp,
        },
        "corrected_probe_tip": {
            "xyz_m": probe_xyz_m,
            "xyz_mm": [value * 1000.0 for value in probe_xyz_m],
            "rpy_deg": rpy_deg,
            "one_line_xyz_m_rpy_deg": [*probe_xyz_m, *rpy_deg],
        },
        "joint_feedback": joint_feedback,
        "feedback_timestamp": float(timestamp) if timestamp is not None else None,
    }


def arm_status_summary(piper: Any) -> dict[str, Any]:
    status_msg = piper.GetArmStatus()
    status = status_msg.arm_status
    return {
        "ctrl_mode": str(status.ctrl_mode),
        "arm_status": str(status.arm_status),
        "mode_feed": str(status.mode_feed),
        "teach_status": str(status.teach_status),
        "motion_status": str(status.motion_status),
        "err_code": int(status.err_code),
        "text": str(status_msg),
    }


def print_status(label: str, status: dict[str, Any]) -> None:
    print(f"{label}:")
    print(f"  ctrl_mode     = {status['ctrl_mode']}")
    print(f"  arm_status    = {status['arm_status']}")
    print(f"  mode_feed     = {status['mode_feed']}")
    print(f"  teach_status  = {status['teach_status']}")
    print(f"  motion_status = {status['motion_status']}")
    print(f"  err_code      = {status['err_code']}")


def command_ready_problems(status: dict[str, Any]) -> list[str]:
    problems: list[str] = []
    if "NORMAL" not in str(status["arm_status"]):
        problems.append(f"arm_status is not NORMAL: {status['arm_status']}")
    if "TEACHING_MODE" in str(status["ctrl_mode"]):
        problems.append(f"still in TEACHING_MODE: {status['ctrl_mode']}")
    if "START_RECORDING" in str(status["teach_status"]):
        problems.append(f"teach recording is active: {status['teach_status']}")
    if int(status["err_code"]) != 0:
        problems.append(f"err_code is not 0: {status['err_code']}")
    return problems


def require_command_ready(piper: Any, *, allow_not_normal: bool = False) -> None:
    status = arm_status_summary(piper)
    print_status("arm status", status)
    problems = command_ready_problems(status)
    if problems and not allow_not_normal:
        raise RuntimeError("Refusing motion: " + "; ".join(problems))


def move_mode_code(name: str) -> int:
    return {"moveP": 0x00, "moveJ": 0x01, "moveL": 0x02, "p": 0x00, "j": 0x01, "l": 0x02}[name]


def validate_xyz_workspace(
    label: str,
    xyz_m: Sequence[float],
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> None:
    x, y, z = finite_vector(xyz_m, 3, label)
    limits = {
        "x": (x, x_min, x_max),
        "y": (y, y_min, y_max),
        "z": (z, z_min, z_max),
    }
    for axis, (value, low, high) in limits.items():
        if value < low or value > high:
            raise RuntimeError(
                f"{label} {axis}={value:.6f} m is outside [{low:.6f}, {high:.6f}] m"
            )


def send_end_pose_repeated(
    piper: Any,
    cmd: tuple[int, int, int, int, int, int],
    *,
    motion_mode: str,
    speed_percent: int,
    send_duration: float,
    send_rate_hz: float,
) -> int:
    mode = move_mode_code(motion_mode)
    deadline = time.time() + float(send_duration)
    sent = 0
    while time.time() < deadline:
        piper.MotionCtrl_2(0x01, mode, int(speed_percent), 0x00)
        piper.EndPoseCtrl(*cmd)
        sent += 1
        time.sleep(max(0.005, 1.0 / float(send_rate_hz)))
    return sent


def load_pose_json(path: str | Path) -> ProbePose:
    data = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if isinstance(data.get("corrected_probe_tip"), dict):
        block = data["corrected_probe_tip"]
        values = block.get("one_line_xyz_m_rpy_deg")
        if values is not None:
            parsed = finite_vector(values, 6, "corrected_probe_tip.one_line_xyz_m_rpy_deg")
            return ProbePose(parsed[:3], parsed[3:])
        if block.get("xyz_m") is not None and block.get("rpy_deg") is not None:
            return ProbePose(
                finite_vector(block["xyz_m"], 3, "corrected_probe_tip.xyz_m"),
                finite_vector(block["rpy_deg"], 3, "corrected_probe_tip.rpy_deg"),
            )
    values = data.get("one_line_xyz_m_rpy_deg")
    if values is not None:
        parsed = finite_vector(values, 6, "one_line_xyz_m_rpy_deg")
        return ProbePose(parsed[:3], parsed[3:])
    if data.get("xyz_m") is not None and data.get("rpy_deg") is not None:
        return ProbePose(
            finite_vector(data["xyz_m"], 3, "xyz_m"),
            finite_vector(data["rpy_deg"], 3, "rpy_deg"),
        )
    raise ValueError(f"{path} does not contain a supported probe-tip pose")
