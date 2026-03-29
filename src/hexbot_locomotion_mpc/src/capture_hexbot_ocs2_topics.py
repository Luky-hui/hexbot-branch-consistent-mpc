#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import ModeSchedule, MpcTargetTrajectories
from rclpy.node import Node
from rclpy.parameter import Parameter


def diagnostic_level_to_int(level: Any) -> int:
    if isinstance(level, (bytes, bytearray)):
        return int(level[0]) if level else 0
    return int(level)


def to_float_list(values: Any) -> list[float]:
    return [float(value) for value in values]


def unwrap_delta(delta: float) -> float:
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def extract_pose(values: list[float], base_pose_offset: int) -> dict[str, float] | None:
    if len(values) < base_pose_offset + 6:
        return None
    return {
        "x": float(values[base_pose_offset + 0]),
        "y": float(values[base_pose_offset + 1]),
        "z": float(values[base_pose_offset + 2]),
        "yaw": float(values[base_pose_offset + 3]),
        "pitch": float(values[base_pose_offset + 4]),
        "roll": float(values[base_pose_offset + 5]),
    }


def extract_target_snapshot(
    message: MpcTargetTrajectories | None, base_pose_offset: int
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "available": False,
        "base_pose_offset": int(base_pose_offset),
    }
    if message is None:
        return payload

    time_trajectory = to_float_list(message.time_trajectory)
    state_trajectory = [to_float_list(state.value) for state in message.state_trajectory]
    input_trajectory = [to_float_list(command.value) for command in message.input_trajectory]

    payload.update(
        {
            "available": True,
            "time_trajectory": time_trajectory,
            "state_count": len(state_trajectory),
            "input_count": len(input_trajectory),
            "state_dims": [len(values) for values in state_trajectory[:4]],
            "input_dims": [len(values) for values in input_trajectory[:4]],
        }
    )

    if len(time_trajectory) >= 2:
        payload["horizon_sec"] = float(time_trajectory[-1] - time_trajectory[0])

    if len(state_trajectory) < 2:
        return payload

    start_pose = extract_pose(state_trajectory[0], base_pose_offset)
    knot1_pose = extract_pose(state_trajectory[1], base_pose_offset)
    last_pose = extract_pose(state_trajectory[-1], base_pose_offset)
    if start_pose is None or knot1_pose is None or last_pose is None:
        return payload

    def build_pose_delta(goal_pose: dict[str, float]) -> dict[str, Any]:
        delta_x = goal_pose["x"] - start_pose["x"]
        delta_y = goal_pose["y"] - start_pose["y"]
        delta_yaw = unwrap_delta(goal_pose["yaw"] - start_pose["yaw"])
        cos_yaw = math.cos(start_pose["yaw"])
        sin_yaw = math.sin(start_pose["yaw"])
        delta_forward = cos_yaw * delta_x + sin_yaw * delta_y
        delta_lateral = -sin_yaw * delta_x + cos_yaw * delta_y
        return {
            "delta_world": {
                "x": delta_x,
                "y": delta_y,
                "yaw": delta_yaw,
            },
            "delta_body": {
                "forward_m": delta_forward,
                "lateral_m": delta_lateral,
            },
        }

    knot1_delta = build_pose_delta(knot1_pose)
    last_delta = build_pose_delta(last_pose)
    horizon_sec = payload.get("horizon_sec")
    implied_velocity = None
    if isinstance(horizon_sec, (int, float)) and abs(float(horizon_sec)) > 1.0e-9:
        implied_velocity = {
            "forward_mps": float(last_delta["delta_body"]["forward_m"]) / float(horizon_sec),
            "lateral_mps": float(last_delta["delta_body"]["lateral_m"]) / float(horizon_sec),
            "yaw_radps": float(last_delta["delta_world"]["yaw"]) / float(horizon_sec),
        }

    payload.update(
        {
            "start_pose": start_pose,
            "goal_pose": knot1_pose,
            "goal_pose_knot1": knot1_pose,
            "goal_pose_last": last_pose,
            "delta_world": knot1_delta["delta_world"],
            "delta_body": knot1_delta["delta_body"],
            "delta_world_knot1": knot1_delta["delta_world"],
            "delta_body_knot1": knot1_delta["delta_body"],
            "delta_world_last": last_delta["delta_world"],
            "delta_body_last": last_delta["delta_body"],
            "implied_velocity_last": implied_velocity,
        }
    )
    return payload


def extract_mode_schedule(message: ModeSchedule | None) -> dict[str, Any]:
    payload: dict[str, Any] = {"available": False}
    if message is None:
        return payload

    mode_sequence = [int(value) for value in message.mode_sequence]
    unique_modes = sorted(set(mode_sequence))
    payload.update(
        {
            "available": True,
            "event_times": to_float_list(message.event_times),
            "mode_sequence": mode_sequence,
            "sequence_length": len(mode_sequence),
            "unique_modes": unique_modes,
            "transition_count": max(0, len(mode_sequence) - 1),
            "moving_cycle_detected": len(unique_modes) > 1 or len(mode_sequence) > 1,
        }
    )
    return payload


def extract_solver_snapshot(message: DiagnosticArray | None) -> dict[str, Any]:
    payload: dict[str, Any] = {"available": False, "status": []}
    if message is None:
        return payload

    max_level = 0
    status_payload = []
    for status in message.status:
        level = diagnostic_level_to_int(status.level)
        max_level = max(max_level, level)
        values: dict[str, str] = {}
        for pair in status.values:
            key = str(pair.key)
            if key:
                values[key] = str(pair.value)
        status_payload.append(
            {
                "name": str(status.name),
                "level": level,
                "message": str(status.message),
                "hardware_id": str(status.hardware_id),
                "values": values,
            }
        )

    payload.update(
        {
            "available": True,
            "max_level": max_level,
            "status": status_payload,
        }
    )
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture Hexbot OCS2 target, gait schedule, and solver topics into JSON."
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--spin-timeout", type=float, default=0.1)
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--target-topic", default="/hexbot_mpc_target")
    parser.add_argument("--mode-topic", default="/hexbot_mpc_mode_schedule")
    parser.add_argument("--solver-topic", default="/hexbot_mpc_solver_diagnostics")
    parser.add_argument("--cmd-topic", default="/hexbot_mpc/input/cmd_vel")
    parser.add_argument("--base-pose-offset", type=int, default=6)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


class Ocs2TopicCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("hexbot_ocs2_topic_capture")
        self.args = args
        if args.use_sim_time:
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.start_wall_sec = time.monotonic()
        self.target_messages = 0
        self.mode_messages = 0
        self.solver_messages = 0
        self.cmd_messages = 0
        self.target_first_elapsed_sec: float | None = None
        self.mode_first_elapsed_sec: float | None = None
        self.solver_first_elapsed_sec: float | None = None
        self.cmd_first_elapsed_sec: float | None = None
        self.target_last_elapsed_sec: float | None = None
        self.mode_last_elapsed_sec: float | None = None
        self.solver_last_elapsed_sec: float | None = None
        self.cmd_last_elapsed_sec: float | None = None
        self.last_target: MpcTargetTrajectories | None = None
        self.last_mode: ModeSchedule | None = None
        self.last_solver: DiagnosticArray | None = None
        self.last_cmd: Twist | None = None
        self.max_abs_cmd_linear_x = 0.0
        self.max_abs_cmd_linear_y = 0.0
        self.max_abs_cmd_angular_z = 0.0

        self.create_subscription(
            MpcTargetTrajectories, args.target_topic, self._on_target, 20
        )
        self.create_subscription(ModeSchedule, args.mode_topic, self._on_mode, 20)
        self.create_subscription(
            DiagnosticArray, args.solver_topic, self._on_solver, 20
        )
        self.create_subscription(Twist, args.cmd_topic, self._on_cmd, 20)

    def _elapsed_sec(self) -> float:
        return time.monotonic() - self.start_wall_sec

    def _on_target(self, message: MpcTargetTrajectories) -> None:
        self.target_messages += 1
        elapsed_sec = self._elapsed_sec()
        if self.target_first_elapsed_sec is None:
            self.target_first_elapsed_sec = elapsed_sec
        self.target_last_elapsed_sec = elapsed_sec
        self.last_target = message

    def _on_mode(self, message: ModeSchedule) -> None:
        self.mode_messages += 1
        elapsed_sec = self._elapsed_sec()
        if self.mode_first_elapsed_sec is None:
            self.mode_first_elapsed_sec = elapsed_sec
        self.mode_last_elapsed_sec = elapsed_sec
        self.last_mode = message

    def _on_solver(self, message: DiagnosticArray) -> None:
        self.solver_messages += 1
        elapsed_sec = self._elapsed_sec()
        if self.solver_first_elapsed_sec is None:
            self.solver_first_elapsed_sec = elapsed_sec
        self.solver_last_elapsed_sec = elapsed_sec
        self.last_solver = message

    def _on_cmd(self, message: Twist) -> None:
        self.cmd_messages += 1
        elapsed_sec = self._elapsed_sec()
        if self.cmd_first_elapsed_sec is None:
            self.cmd_first_elapsed_sec = elapsed_sec
        self.cmd_last_elapsed_sec = elapsed_sec
        self.last_cmd = message
        self.max_abs_cmd_linear_x = max(
            self.max_abs_cmd_linear_x, abs(float(message.linear.x))
        )
        self.max_abs_cmd_linear_y = max(
            self.max_abs_cmd_linear_y, abs(float(message.linear.y))
        )
        self.max_abs_cmd_angular_z = max(
            self.max_abs_cmd_angular_z, abs(float(message.angular.z))
        )

    def finalize(self) -> dict[str, Any]:
        cmd_snapshot = {
            "available": self.last_cmd is not None,
            "max_abs_linear_x": self.max_abs_cmd_linear_x,
            "max_abs_linear_y": self.max_abs_cmd_linear_y,
            "max_abs_angular_z": self.max_abs_cmd_angular_z,
        }
        if self.last_cmd is not None:
            cmd_snapshot["last"] = {
                "linear_x": float(self.last_cmd.linear.x),
                "linear_y": float(self.last_cmd.linear.y),
                "angular_z": float(self.last_cmd.angular.z),
            }
        return {
            "ready": self.target_messages > 0 and self.mode_messages > 0,
            "topics": {
                "target": self.args.target_topic,
                "mode_schedule": self.args.mode_topic,
                "solver_diagnostics": self.args.solver_topic,
                "bridge_cmd_vel": self.args.cmd_topic,
            },
            "publishers": {
                self.args.target_topic: self.count_publishers(self.args.target_topic),
                self.args.mode_topic: self.count_publishers(self.args.mode_topic),
                self.args.solver_topic: self.count_publishers(self.args.solver_topic),
                self.args.cmd_topic: self.count_publishers(self.args.cmd_topic),
            },
            "message_counts": {
                "target": self.target_messages,
                "mode_schedule": self.mode_messages,
                "solver_diagnostics": self.solver_messages,
                "bridge_cmd_vel": self.cmd_messages,
            },
            "first_message_after_start_sec": {
                "target": self.target_first_elapsed_sec,
                "mode_schedule": self.mode_first_elapsed_sec,
                "solver_diagnostics": self.solver_first_elapsed_sec,
                "bridge_cmd_vel": self.cmd_first_elapsed_sec,
            },
            "last_message_after_start_sec": {
                "target": self.target_last_elapsed_sec,
                "mode_schedule": self.mode_last_elapsed_sec,
                "solver_diagnostics": self.solver_last_elapsed_sec,
                "bridge_cmd_vel": self.cmd_last_elapsed_sec,
            },
            "bridge_cmd_vel_snapshot": cmd_snapshot,
            "target_snapshot": extract_target_snapshot(
                self.last_target, self.args.base_pose_offset
            ),
            "mode_schedule_snapshot": extract_mode_schedule(self.last_mode),
            "solver_capture_snapshot": extract_solver_snapshot(self.last_solver),
        }


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Ocs2TopicCapture(args)

    try:
        deadline = time.monotonic() + max(2.0, float(args.duration))
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=float(args.spin_timeout))

        payload = node.finalize()
        text = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
        print(text)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
