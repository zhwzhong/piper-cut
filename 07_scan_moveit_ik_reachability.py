#!/usr/bin/env python3
"""Scan PiPER TCP reachability with MoveIt2 IK services.

This script does not execute robot motion. It samples TCP poses, calls
``/compute_ik`` for each pose, and optionally calls ``/plan_kinematic_path`` to
check whether MoveIt can plan from the current state to the IK solution.

Run it on a machine where ROS2/MoveIt2 and the PiPER MoveIt stack are sourced.
"""

from __future__ import annotations

import argparse
import csv
from copy import deepcopy
from datetime import datetime
import json
import math
from pathlib import Path
import time
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState


ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = ROOT / "outputs" / "reachability"
ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


def now_id() -> str:
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")


def parse_range_mm(text: str, name: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in str(text).split(",")]
    if len(parts) != 3:
        raise ValueError(f"{name} must be min,max,step in mm")
    start, stop, step = [float(part) for part in parts]
    if step <= 0 or stop < start:
        raise ValueError(f"{name} must satisfy min <= max and step > 0")
    return start, stop, step


def values_mm(spec: tuple[float, float, float]) -> list[float]:
    start, stop, step = spec
    values = []
    current = start
    while current <= stop + step * 1.0e-9:
        values.append(round(current, 6))
        current += step
    return values


def quat_from_euler_xyz_deg(rx_deg: float, ry_deg: float, rz_deg: float) -> tuple[float, float, float, float]:
    roll = math.radians(rx_deg)
    pitch = math.radians(ry_deg)
    yaw = math.radians(rz_deg)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        float(sr * cp * cy - cr * sp * sy),
        float(cr * sp * cy + sr * cp * sy),
        float(cr * cp * sy - sr * sp * cy),
        float(cr * cp * cy + sr * sp * sy),
    )


def quat_from_matrix(matrix: list[list[float]]) -> tuple[float, float, float, float]:
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def matrix_to_euler_xyz_deg(matrix: list[list[float]]) -> tuple[float, float, float]:
    m20 = max(-1.0, min(1.0, matrix[2][0]))
    if abs(m20) < 0.999999:
        pitch = math.asin(-m20)
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        pitch = math.asin(-m20)
        roll = 0.0
        yaw = math.atan2(-matrix[0][1], matrix[1][1])
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(dot(v, v))
    if length <= 1.0e-12:
        raise ValueError("Cannot normalize zero-length vector")
    return (v[0] / length, v[1] / length, v[2] / length)


def axis_vector(axis: str, sign: int) -> tuple[float, float, float]:
    base = {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 1.0, 0.0),
        "z": (0.0, 0.0, 1.0),
    }[axis]
    return (base[0] * sign, base[1] * sign, base[2] * sign)


def orthonormal_basis_from_axis(axis: tuple[float, float, float]) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    w = normalize(axis)
    reference = (1.0, 0.0, 0.0) if abs(w[0]) < 0.9 else (0.0, 1.0, 0.0)
    projected = (
        reference[0] - dot(reference, w) * w[0],
        reference[1] - dot(reference, w) * w[1],
        reference[2] - dot(reference, w) * w[2],
    )
    u = normalize(projected)
    v = cross(w, u)
    return u, v, w


def matmul_abt(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[j][k] for k in range(3)) for j in range(3)] for i in range(3)]


def columns_to_matrix(columns: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]) -> list[list[float]]:
    return [[columns[col][row] for col in range(3)] for row in range(3)]


def direction_near_down(tilt_deg: float, azimuth_deg: float) -> tuple[float, float, float]:
    tilt = math.radians(tilt_deg)
    azimuth = math.radians(azimuth_deg)
    return normalize(
        (
            math.sin(tilt) * math.cos(azimuth),
            math.sin(tilt) * math.sin(azimuth),
            -math.cos(tilt),
        )
    )


def orientation_for_tool_axis(
    tool_axis: tuple[float, float, float],
    world_direction: tuple[float, float, float],
    roll_about_tool_deg: float,
) -> tuple[tuple[float, float, float, float], tuple[float, float, float], list[list[float]]]:
    local_u, local_v, local_w = orthonormal_basis_from_axis(tool_axis)
    local_basis = columns_to_matrix((local_u, local_v, local_w))
    world_w = normalize(world_direction)
    world_u0, world_v0, _ = orthonormal_basis_from_axis(world_w)
    roll = math.radians(roll_about_tool_deg)
    world_u = (
        math.cos(roll) * world_u0[0] + math.sin(roll) * world_v0[0],
        math.cos(roll) * world_u0[1] + math.sin(roll) * world_v0[1],
        math.cos(roll) * world_u0[2] + math.sin(roll) * world_v0[2],
    )
    world_v = cross(world_w, world_u)
    world_basis = columns_to_matrix((world_u, world_v, world_w))
    matrix = matmul_abt(world_basis, local_basis)
    return quat_from_matrix(matrix), matrix_to_euler_xyz_deg(matrix), matrix


def tip_down_candidates(args: argparse.Namespace) -> list[dict[str, Any]]:
    tool_axis = axis_vector(args.tool_axis, args.tool_axis_sign)
    candidates: list[dict[str, Any]] = []
    tilt = 0.0
    while tilt <= args.max_tilt_deg + 1.0e-9:
        azimuths = [0.0] if abs(tilt) <= 1.0e-9 else values_mm((0.0, 360.0 - args.azimuth_step_deg, args.azimuth_step_deg))
        for azimuth in azimuths:
            direction = direction_near_down(tilt, azimuth)
            roll_values = values_mm((0.0, 360.0 - args.roll_step_deg, args.roll_step_deg))
            for roll in roll_values:
                quat, rpy, _matrix = orientation_for_tool_axis(tool_axis, direction, roll)
                candidates.append(
                    {
                        "tilt_deg": round(tilt, 6),
                        "azimuth_deg": round(azimuth, 6),
                        "roll_about_tool_deg": round(roll, 6),
                        "quat_xyzw": quat,
                        "rpy_deg": [round(rpy[0], 6), round(rpy[1], 6), round(rpy[2], 6)],
                    }
                )
        tilt += args.tilt_step_deg
    return candidates


def duration_msg(seconds: float) -> Duration:
    sec = int(math.floor(seconds))
    nanosec = int((seconds - sec) * 1_000_000_000)
    return Duration(sec=sec, nanosec=nanosec)


def is_success(error_code: Any) -> bool:
    return int(getattr(error_code, "val", 99999)) == MoveItErrorCodes.SUCCESS


def error_code_value(error_code: Any) -> int:
    return int(getattr(error_code, "val", 99999))


class MoveItReachabilityScanner(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("piper_moveit_ik_reachability_scan")
        self.args = args
        self.joint_states_by_topic: dict[str, JointState] = {}
        for topic in ("/feedback/joint_states", "/joint_states", "/control/joint_states"):
            self.create_subscription(JointState, topic, self._make_joint_cb(topic), 10)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")

    def _make_joint_cb(self, topic: str):
        def _joint_cb(msg: JointState) -> None:
            if all(joint in msg.name for joint in ARM_JOINTS):
                self.joint_states_by_topic[topic] = msg

        return _joint_cb

    def selected_joint_state(self) -> JointState | None:
        if self.args.state_topic != "auto":
            return self.joint_states_by_topic.get(self.args.state_topic)
        for topic in ("/feedback/joint_states", "/joint_states", "/control/joint_states"):
            if topic in self.joint_states_by_topic:
                return self.joint_states_by_topic[topic]
        return None

    def wait_for_startup(self) -> None:
        if not self.ik_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise TimeoutError("Timed out waiting for /compute_ik")
        if self.args.check_plan and not self.plan_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise TimeoutError("Timed out waiting for /plan_kinematic_path")
        deadline = time.time() + self.args.state_timeout
        while rclpy.ok() and self.selected_joint_state() is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.selected_joint_state() is None:
            raise TimeoutError(f"No current JointState received for state_topic={self.args.state_topic}")

    def current_robot_state(self) -> RobotState:
        state = RobotState()
        state.joint_state = deepcopy(self.selected_joint_state())
        return state

    def target_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        quat_xyzw: tuple[float, float, float, float] | None = None,
    ) -> PoseStamped:
        target = PoseStamped()
        target.header.frame_id = self.args.frame
        target.pose.position.x = float(x_mm) / 1000.0
        target.pose.position.y = float(y_mm) / 1000.0
        target.pose.position.z = float(z_mm) / 1000.0
        qx, qy, qz, qw = quat_xyzw or quat_from_euler_xyz_deg(self.args.rx, self.args.ry, self.args.rz)
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw
        return target

    def solve_ik(self, target: PoseStamped) -> tuple[bool, int, RobotState | None]:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.args.group
        request.ik_request.ik_link_name = self.args.pose_link
        request.ik_request.pose_stamped = target
        request.ik_request.robot_state = self.current_robot_state()
        request.ik_request.timeout = duration_msg(self.args.ik_timeout)
        request.ik_request.avoid_collisions = self.args.avoid_collisions
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.service_timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("IK service call failed")
        response = future.result()
        return is_success(response.error_code), error_code_value(response.error_code), response.solution

    def plan_to_state(self, goal_state: RobotState) -> tuple[bool, int]:
        request = GetMotionPlan.Request()
        mpr = request.motion_plan_request
        mpr.group_name = self.args.group
        mpr.num_planning_attempts = self.args.planning_attempts
        mpr.allowed_planning_time = self.args.planning_time
        mpr.max_velocity_scaling_factor = self.args.velocity_scale
        mpr.max_acceleration_scaling_factor = self.args.acceleration_scale
        mpr.start_state = self.current_robot_state()
        mpr.goal_constraints = [self.joint_goal_constraints(goal_state)]
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.planning_time + 5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("Planning service call failed")
        response = future.result().motion_plan_response
        return is_success(response.error_code), error_code_value(response.error_code)

    def joint_goal_constraints(self, state: RobotState) -> Constraints:
        positions = dict(zip(state.joint_state.name, state.joint_state.position))
        constraints = Constraints()
        constraints.name = "ik_joint_solution"
        for joint in ARM_JOINTS:
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = float(positions[joint])
            jc.tolerance_above = self.args.joint_goal_tolerance
            jc.tolerance_below = self.args.joint_goal_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def scan(self) -> dict[str, Any]:
        xs = values_mm(parse_range_mm(self.args.x_range_mm, "x-range-mm"))
        ys = values_mm(parse_range_mm(self.args.y_range_mm, "y-range-mm"))
        zs = values_mm(parse_range_mm(self.args.z_range_mm, "z-range-mm"))
        orientation_candidates = tip_down_candidates(self.args) if self.args.prefer_tip_down else []
        points: list[dict[str, Any]] = []
        ik_count = 0
        plan_count = 0
        per_z: dict[str, dict[str, int]] = {}
        for z in zs:
            z_key = f"{z:.3f}"
            per_z[z_key] = {"total": 0, "ik": 0, "plan": 0}
            for y in ys:
                for x in xs:
                    tried_orientations = 1
                    selected_orientation: dict[str, Any] | None = None
                    if self.args.prefer_tip_down:
                        ik_ok = False
                        ik_code = 99999
                        solution = None
                        tried_orientations = 0
                        for candidate in orientation_candidates:
                            tried_orientations += 1
                            target = self.target_pose(x, y, z, candidate["quat_xyzw"])
                            ik_ok, ik_code, solution = self.solve_ik(target)
                            if ik_ok:
                                selected_orientation = candidate
                                break
                    else:
                        target = self.target_pose(x, y, z)
                        ik_ok, ik_code, solution = self.solve_ik(target)
                    plan_ok = None
                    plan_code = None
                    if ik_ok:
                        ik_count += 1
                        per_z[z_key]["ik"] += 1
                        if self.args.check_plan and solution is not None:
                            plan_ok, plan_code = self.plan_to_state(solution)
                            if plan_ok:
                                plan_count += 1
                                per_z[z_key]["plan"] += 1
                    points.append(
                        {
                            "xyz_mm": [x, y, z],
                            "rpy_deg": selected_orientation["rpy_deg"] if selected_orientation else [self.args.rx, self.args.ry, self.args.rz],
                            "tip_down": selected_orientation,
                            "tried_orientations": tried_orientations,
                            "ik_ok": ik_ok,
                            "ik_error_code": ik_code,
                            "plan_ok": plan_ok,
                            "plan_error_code": plan_code,
                        }
                    )
                    per_z[z_key]["total"] += 1
        total = len(points)
        return {
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "mode": "moveit_compute_ik",
            "frame_id": self.args.frame,
            "group": self.args.group,
            "pose_link": self.args.pose_link,
            "avoid_collisions": self.args.avoid_collisions,
            "check_plan": self.args.check_plan,
            "prefer_tip_down": self.args.prefer_tip_down,
            "tool_axis": self.args.tool_axis,
            "tool_axis_sign": self.args.tool_axis_sign,
            "max_tilt_deg": self.args.max_tilt_deg if self.args.prefer_tip_down else None,
            "tilt_step_deg": self.args.tilt_step_deg if self.args.prefer_tip_down else None,
            "azimuth_step_deg": self.args.azimuth_step_deg if self.args.prefer_tip_down else None,
            "roll_step_deg": self.args.roll_step_deg if self.args.prefer_tip_down else None,
            "scan_ranges_mm": {
                "x": self.args.x_range_mm,
                "y": self.args.y_range_mm,
                "z": self.args.z_range_mm,
            },
            "rpy_deg": [self.args.rx, self.args.ry, self.args.rz],
            "summary": {
                "total": total,
                "ik_ok": ik_count,
                "ik_ratio": ik_count / total if total else 0.0,
                "plan_ok": plan_count if self.args.check_plan else None,
                "plan_ratio": plan_count / total if total and self.args.check_plan else None,
                "per_z": per_z,
            },
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
                "tip_tilt_deg",
                "tip_azimuth_deg",
                "roll_about_tool_deg",
                "tried_orientations",
                "ik_ok",
                "ik_error_code",
                "plan_ok",
                "plan_error_code",
            ]
        )
        for point in report["points"]:
            tip_down = point.get("tip_down") or {}
            writer.writerow(
                [
                    *point["xyz_mm"],
                    *point["rpy_deg"],
                    tip_down.get("tilt_deg"),
                    tip_down.get("azimuth_deg"),
                    tip_down.get("roll_about_tool_deg"),
                    point.get("tried_orientations", 1),
                    int(point["ik_ok"]),
                    point["ik_error_code"],
                    point["plan_ok"],
                    point["plan_error_code"],
                ]
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x-range-mm", default="250,550,20")
    parser.add_argument("--y-range-mm", default="-180,180,20")
    parser.add_argument("--z-range-mm", default="99,180,20")
    parser.add_argument("--rx", type=float, default=173.0)
    parser.add_argument("--ry", type=float, default=-5.0)
    parser.add_argument("--rz", type=float, default=163.0)
    parser.add_argument("--prefer-tip-down", action="store_true", help="sample orientations and prefer the TCP tool axis closest to base_link -Z")
    parser.add_argument("--tool-axis", choices=("x", "y", "z"), default="z", help="TCP local axis that points along the probe direction")
    parser.add_argument("--tool-axis-sign", type=int, choices=(-1, 1), default=1, help="sign of the TCP probe axis; default +Z")
    parser.add_argument("--max-tilt-deg", type=float, default=30.0, help="maximum allowed angle between probe axis and base_link -Z")
    parser.add_argument("--tilt-step-deg", type=float, default=10.0)
    parser.add_argument("--azimuth-step-deg", type=float, default=30.0)
    parser.add_argument("--roll-step-deg", type=float, default=45.0)
    parser.add_argument("--group", default="arm")
    parser.add_argument("--pose-link", default="tcp_link")
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--state-topic", default="auto")
    parser.add_argument("--avoid-collisions", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--check-plan", action="store_true")
    parser.add_argument("--ik-timeout", type=float, default=0.25)
    parser.add_argument("--planning-time", type=float, default=1.0)
    parser.add_argument("--planning-attempts", type=int, default=2)
    parser.add_argument("--joint-goal-tolerance", type=float, default=0.003)
    parser.add_argument("--velocity-scale", type=float, default=0.05)
    parser.add_argument("--acceleration-scale", type=float, default=0.05)
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument("--state-timeout", type=float, default=10.0)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--output-stem", default="")
    args = parser.parse_args()
    for name in ("tilt_step_deg", "azimuth_step_deg", "roll_step_deg"):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be > 0")
    if args.max_tilt_deg < 0:
        parser.error("--max-tilt-deg must be >= 0")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = MoveItReachabilityScanner(args)
    try:
        node.wait_for_startup()
        report = node.scan()
        args.output_dir.mkdir(parents=True, exist_ok=True)
        stem = args.output_stem or f"moveit_ik_reachability_{now_id()}"
        json_path = args.output_dir / f"{stem}.json"
        csv_path = args.output_dir / f"{stem}.csv"
        json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        write_csv(csv_path, report)
        summary = report["summary"]
        print("=== MoveIt2 IK reachability scan ===")
        print("mode: /compute_ik only; no execution")
        print(f"frame={args.frame} group={args.group} pose_link={args.pose_link}")
        if args.prefer_tip_down:
            print(
                "tip_down="
                f"axis={args.tool_axis} sign={args.tool_axis_sign} "
                f"max_tilt={args.max_tilt_deg} step={args.tilt_step_deg} "
                f"azimuth_step={args.azimuth_step_deg} roll_step={args.roll_step_deg}"
            )
        else:
            print(f"rpy_deg={[args.rx, args.ry, args.rz]}")
        print(f"total={summary['total']} ik_ok={summary['ik_ok']} ik_ratio={summary['ik_ratio']:.3f}")
        if args.check_plan:
            print(f"plan_ok={summary['plan_ok']} plan_ratio={summary['plan_ratio']:.3f}")
        print(f"Saved JSON: {json_path}")
        print(f"Saved CSV:  {csv_path}")
        return 0
    except Exception as error:
        print(f"ERROR: {error}")
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
