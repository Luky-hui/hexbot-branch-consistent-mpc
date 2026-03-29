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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from ensure_isaac_clean_start import DEFAULT_REFERENCE_FILE, load_expected_joint_positions


def quaternion_to_yaw(quaternion) -> float:
    x_value = float(quaternion.x)
    y_value = float(quaternion.y)
    z_value = float(quaternion.z)
    w_value = float(quaternion.w)
    norm = math.sqrt(
        x_value * x_value
        + y_value * y_value
        + z_value * z_value
        + w_value * w_value
    )
    if norm <= 1.0e-9:
        return 0.0
    x_value /= norm
    y_value /= norm
    z_value /= norm
    w_value /= norm
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap_angle(previous_angle: float | None, current_angle: float) -> float:
    if previous_angle is None:
        return current_angle
    delta = current_angle - previous_angle
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return previous_angle + delta


def joint_state_to_map(message: JointState | None) -> dict[str, float] | None:
    if message is None:
        return None
    result: dict[str, float] = {}
    for index, joint_name in enumerate(message.name):
        if index >= len(message.position):
            break
        result[str(joint_name)] = float(message.position[index])
    return result


def all_finite(values: list[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def odom_is_finite(message: Odometry) -> bool:
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    linear = message.twist.twist.linear
    angular = message.twist.twist.angular
    return all_finite(
        [
            float(position.x),
            float(position.y),
            float(position.z),
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
            float(linear.x),
            float(linear.y),
            float(linear.z),
            float(angular.x),
            float(angular.y),
            float(angular.z),
        ]
    )


def joint_state_is_finite(message: JointState | None) -> bool | None:
    if message is None:
        return None
    payload = list(message.position) + list(message.velocity) + list(message.effort)
    return all_finite([float(value) for value in payload])


def diagnostic_level_to_int(level: Any) -> int:
    if isinstance(level, (bytes, bytearray)):
        return int(level[0]) if level else 0
    return int(level)


def compare_joint_maps(
    left_name: str,
    left_map: dict[str, float] | None,
    right_name: str,
    right_map: dict[str, float] | None,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "available": False,
        "left": left_name,
        "right": right_name,
        "max_abs_diff_rad": None,
        "missing_from_left": [],
        "missing_from_right": [],
        "top_diffs": [],
    }
    if left_map is None or right_map is None:
        return payload

    shared = sorted(set(left_map.keys()) & set(right_map.keys()))
    payload["missing_from_left"] = sorted(set(right_map.keys()) - set(left_map.keys()))
    payload["missing_from_right"] = sorted(set(left_map.keys()) - set(right_map.keys()))
    if not shared:
        return payload

    diffs = []
    max_abs_diff = 0.0
    for joint_name in shared:
        abs_diff = abs(left_map[joint_name] - right_map[joint_name])
        max_abs_diff = max(max_abs_diff, abs_diff)
        diffs.append(
            {
                "joint": joint_name,
                left_name: left_map[joint_name],
                right_name: right_map[joint_name],
                "abs_diff_rad": abs_diff,
            }
        )

    diffs.sort(key=lambda item: float(item["abs_diff_rad"]), reverse=True)
    payload["available"] = True
    payload["max_abs_diff_rad"] = max_abs_diff
    payload["top_diffs"] = diffs[:6]
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Publish a fixed /cmd_vel command and verify the closed-loop response "
            "through /hexbot_mpc/output/joint_command, /joint_command, and /odom."
        )
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--spin-timeout", type=float, default=0.1)
    parser.add_argument("--cmd-publish-rate", type=float, default=10.0)
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--clock-topic", default="/clock")
    parser.add_argument(
        "--solver-diagnostics-topic", default="/hexbot_mpc_solver_diagnostics"
    )
    parser.add_argument("--runtime-joint-command-topic", default="/joint_command")
    parser.add_argument(
        "--backend-joint-command-topic", default="/hexbot_mpc/output/joint_command"
    )
    parser.add_argument("--linear-x", type=float, default=0.0)
    parser.add_argument("--linear-y", type=float, default=0.0)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--expected-joint-count", type=int, default=18)
    parser.add_argument("--min-joint-messages", type=int, default=1)
    parser.add_argument("--min-odom-messages", type=int, default=1)
    parser.add_argument(
        "--require-backend",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="When enabled, /hexbot_mpc/output/joint_command must be present and active.",
    )
    parser.add_argument(
        "--require-clock",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="When enabled, /clock must be present and active in sim-time runs.",
    )
    parser.add_argument(
        "--reference-file",
        type=Path,
        default=DEFAULT_REFERENCE_FILE,
        help="Reference stance used to compare expected joint positions against commands and measured state.",
    )
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


class ClosedLoopProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("hexbot_closed_loop_probe")
        self.args = args
        if args.use_sim_time:
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.wall_start_sec = time.monotonic()
        self.clock_start_sec: float | None = None
        self.clock_now_sec: float | None = None
        self.command_started = False
        self.command_finished = False
        self.command_start_wall_sec: float | None = None

        self.cmd_message = Twist()
        self.cmd_message.linear.x = float(args.linear_x)
        self.cmd_message.linear.y = float(args.linear_y)
        self.cmd_message.angular.z = float(args.angular_z)

        self.clock_messages = 0
        self.odom_messages = 0
        self.odom_messages_after_start = 0
        self.joint_state_messages = 0
        self.joint_state_messages_after_start = 0
        self.backend_messages = 0
        self.backend_messages_after_start = 0
        self.runtime_messages = 0
        self.runtime_messages_after_start = 0
        self.solver_diagnostics_messages = 0

        self.last_actual_joint_state: JointState | None = None
        self.last_runtime_joint_command: JointState | None = None
        self.last_backend_joint_command: JointState | None = None
        self.last_solver_diagnostics: DiagnosticArray | None = None
        self.last_actual_joint_count = 0
        self.last_runtime_joint_count = 0
        self.last_backend_joint_count = 0
        self.last_runtime_wall_age_sec: float | None = None
        self.last_backend_wall_age_sec: float | None = None
        self.runtime_first_message_after_start_sec: float | None = None
        self.backend_first_message_after_start_sec: float | None = None

        self.start_pose: dict[str, float] | None = None
        self.end_pose: dict[str, float] | None = None
        self.previous_odom_time_sec: float | None = None
        self.previous_odom_yaw_unwrapped: float | None = None
        self.current_still_sec = 0.0
        self.max_still_sec = 0.0
        self.odom_nonfinite_messages = 0
        self.actual_joint_state_nonfinite_messages = 0
        self.runtime_joint_command_nonfinite_messages = 0
        self.backend_joint_command_nonfinite_messages = 0
        self.max_solver_diagnostics_level = 0
        self.last_command_vs_state_max_abs_diff_rad: float | None = None
        self.max_command_vs_state_max_abs_diff_rad = 0.0
        self.command_vs_state_max_abs_diff_sum_rad = 0.0
        self.command_vs_state_samples = 0

        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.expected_joint_positions = load_expected_joint_positions(
            args.reference_file.resolve()
        )
        self.create_subscription(Clock, args.clock_topic, self._on_clock, 20)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, 20)
        self.create_subscription(
            JointState,
            args.joint_state_topic,
            self._on_joint_state,
            20,
        )
        self.create_subscription(
            DiagnosticArray,
            args.solver_diagnostics_topic,
            self._on_solver_diagnostics,
            20,
        )
        self.create_subscription(
            JointState,
            args.runtime_joint_command_topic,
            self._on_runtime_joint_command,
            20,
        )
        if args.backend_joint_command_topic:
            self.create_subscription(
                JointState,
                args.backend_joint_command_topic,
                self._on_backend_joint_command,
                20,
            )
        self.cmd_timer = self.create_timer(
            1.0 / max(1.0, float(args.cmd_publish_rate)), self._on_cmd_timer
        )

    def _elapsed_sec(self) -> float | None:
        if not self.args.use_sim_time:
            return time.monotonic() - self.wall_start_sec
        if self.clock_start_sec is None or self.clock_now_sec is None:
            return None
        return self.clock_now_sec - self.clock_start_sec

    def _mark_command_started_if_ready(self) -> float | None:
        elapsed_sec = self._elapsed_sec()
        if elapsed_sec is None:
            return None
        if not self.command_started:
            self.command_started = True
            self.command_start_wall_sec = time.monotonic()
        return elapsed_sec

    def _on_clock(self, message: Clock) -> None:
        self.clock_messages += 1
        clock_sec = float(message.clock.sec) + float(message.clock.nanosec) * 1.0e-9
        if self.clock_start_sec is None:
            self.clock_start_sec = clock_sec
        self.clock_now_sec = clock_sec

    def _on_odom(self, message: Odometry) -> None:
        self.odom_messages += 1
        if not odom_is_finite(message):
            self.odom_nonfinite_messages += 1
        elapsed_sec = self._elapsed_sec()
        if elapsed_sec is None or not self.command_started:
            return

        self.odom_messages_after_start += 1
        position_x = float(message.pose.pose.position.x)
        position_y = float(message.pose.pose.position.y)
        yaw_unwrapped = unwrap_angle(
            self.previous_odom_yaw_unwrapped,
            quaternion_to_yaw(message.pose.pose.orientation),
        )

        if self.start_pose is None:
            self.start_pose = {
                "x": position_x,
                "y": position_y,
                "yaw": yaw_unwrapped,
            }

        self.end_pose = {
            "x": position_x,
            "y": position_y,
            "yaw": yaw_unwrapped,
        }

        if self.previous_odom_time_sec is not None:
            delta_time_sec = max(0.0, elapsed_sec - self.previous_odom_time_sec)
            linear_speed = math.hypot(
                float(message.twist.twist.linear.x),
                float(message.twist.twist.linear.y),
            )
            angular_speed = abs(float(message.twist.twist.angular.z))
            moving = linear_speed > 0.005 or angular_speed > 0.02
            command_active = any(
                abs(value) > 1.0e-9
                for value in (
                    self.args.linear_x,
                    self.args.linear_y,
                    self.args.angular_z,
                )
            )
            if command_active and not moving:
                self.current_still_sec += delta_time_sec
                self.max_still_sec = max(self.max_still_sec, self.current_still_sec)
            else:
                self.current_still_sec = 0.0

        self.previous_odom_time_sec = elapsed_sec
        self.previous_odom_yaw_unwrapped = yaw_unwrapped

    def _on_joint_state(self, message: JointState) -> None:
        self.joint_state_messages += 1
        if joint_state_is_finite(message) is False:
            self.actual_joint_state_nonfinite_messages += 1
        self.last_actual_joint_state = message
        self.last_actual_joint_count = len(message.name)
        if self.command_started:
            self.joint_state_messages_after_start += 1
            self._update_command_vs_state_tracking()

    def _on_runtime_joint_command(self, message: JointState) -> None:
        self.runtime_messages += 1
        if joint_state_is_finite(message) is False:
            self.runtime_joint_command_nonfinite_messages += 1
        self.last_runtime_joint_command = message
        self.last_runtime_joint_count = len(message.name)
        if self.command_started:
            self.runtime_messages_after_start += 1
            if self.runtime_first_message_after_start_sec is None:
                self.runtime_first_message_after_start_sec = (
                    time.monotonic() - (self.command_start_wall_sec or time.monotonic())
                )
            self._update_command_vs_state_tracking()
        self.last_runtime_wall_age_sec = 0.0

    def _on_backend_joint_command(self, message: JointState) -> None:
        self.backend_messages += 1
        if joint_state_is_finite(message) is False:
            self.backend_joint_command_nonfinite_messages += 1
        self.last_backend_joint_command = message
        self.last_backend_joint_count = len(message.name)
        if self.command_started:
            self.backend_messages_after_start += 1
            if self.backend_first_message_after_start_sec is None:
                self.backend_first_message_after_start_sec = (
                    time.monotonic() - (self.command_start_wall_sec or time.monotonic())
                )
        self.last_backend_wall_age_sec = 0.0

    def _on_solver_diagnostics(self, message: DiagnosticArray) -> None:
        self.solver_diagnostics_messages += 1
        self.last_solver_diagnostics = message
        for status in message.status:
            self.max_solver_diagnostics_level = max(
                self.max_solver_diagnostics_level, diagnostic_level_to_int(status.level)
            )

    def _update_command_vs_state_tracking(self) -> None:
        command_map = joint_state_to_map(self.last_runtime_joint_command)
        state_map = joint_state_to_map(self.last_actual_joint_state)
        comparison = compare_joint_maps("command", command_map, "state", state_map)
        if not comparison["available"]:
            return
        max_abs_diff_rad = float(comparison["max_abs_diff_rad"] or 0.0)
        self.last_command_vs_state_max_abs_diff_rad = max_abs_diff_rad
        self.max_command_vs_state_max_abs_diff_rad = max(
            self.max_command_vs_state_max_abs_diff_rad, max_abs_diff_rad
        )
        self.command_vs_state_max_abs_diff_sum_rad += max_abs_diff_rad
        self.command_vs_state_samples += 1

    def _on_cmd_timer(self) -> None:
        elapsed_sec = self._mark_command_started_if_ready()
        if elapsed_sec is None:
            return
        if self.command_finished:
            self.cmd_pub.publish(Twist())
            return
        if elapsed_sec < float(self.args.duration):
            self.cmd_pub.publish(self.cmd_message)
            return
        self.command_finished = True
        self.cmd_pub.publish(Twist())

    def stop_command(self, repeat_count: int = 3) -> None:
        stop_message = Twist()
        for _ in range(max(1, int(repeat_count))):
            self.cmd_pub.publish(stop_message)
            time.sleep(0.05)

    def finalize(self) -> dict[str, Any]:
        now_monotonic = time.monotonic()
        if self.command_start_wall_sec is not None:
            if self.runtime_first_message_after_start_sec is None and self.runtime_messages > 0:
                self.runtime_first_message_after_start_sec = max(
                    0.0, now_monotonic - self.command_start_wall_sec
                )
            if self.backend_first_message_after_start_sec is None and self.backend_messages > 0:
                self.backend_first_message_after_start_sec = max(
                    0.0, now_monotonic - self.command_start_wall_sec
                )

        runtime_map = joint_state_to_map(self.last_runtime_joint_command)
        backend_map = joint_state_to_map(self.last_backend_joint_command)
        actual_state_map = joint_state_to_map(self.last_actual_joint_state)
        expected_map = dict(self.expected_joint_positions)
        joint_compare = compare_joint_maps("runtime", runtime_map, "backend", backend_map)
        command_vs_state_compare = compare_joint_maps(
            "command", runtime_map, "state", actual_state_map
        )
        expected_vs_runtime_compare = compare_joint_maps(
            "expected", expected_map, "runtime", runtime_map
        )
        expected_vs_backend_compare = compare_joint_maps(
            "expected", expected_map, "backend", backend_map
        )
        expected_vs_state_compare = compare_joint_maps(
            "expected", expected_map, "state", actual_state_map
        )

        if self.start_pose is None or self.end_pose is None:
            delta_x = None
            delta_y = None
            delta_yaw = None
            delta_forward = None
            delta_lateral = None
            arc_expected_lateral = None
            arc_lateral_excess = None
            arc_radius = None
        else:
            delta_x = self.end_pose["x"] - self.start_pose["x"]
            delta_y = self.end_pose["y"] - self.start_pose["y"]
            delta_yaw = self.end_pose["yaw"] - self.start_pose["yaw"]
            start_yaw = self.start_pose["yaw"]
            cos_yaw = math.cos(start_yaw)
            sin_yaw = math.sin(start_yaw)
            delta_forward = cos_yaw * delta_x + sin_yaw * delta_y
            delta_lateral = -sin_yaw * delta_x + cos_yaw * delta_y
            if abs(delta_yaw) > 1.0e-6:
                arc_expected_lateral = delta_forward * math.tan(0.5 * delta_yaw)
                arc_lateral_excess = delta_lateral - arc_expected_lateral
                sin_yaw_delta = math.sin(delta_yaw)
                arc_radius = (
                    None
                    if abs(sin_yaw_delta) <= 1.0e-6
                    else delta_forward / sin_yaw_delta
                )
            else:
                arc_expected_lateral = 0.0
                arc_lateral_excess = delta_lateral
                arc_radius = None

        runtime_publishers = self.count_publishers(self.args.runtime_joint_command_topic)
        backend_publishers = (
            self.count_publishers(self.args.backend_joint_command_topic)
            if self.args.backend_joint_command_topic
            else 0
        )
        odom_publishers = self.count_publishers(self.args.odom_topic)
        clock_publishers = self.count_publishers(self.args.clock_topic)

        failures: list[str] = []
        ready = True

        if self.args.use_sim_time and self.args.require_clock:
            if clock_publishers < 1 and self.clock_messages < 1:
                ready = False
                failures.append(f"{self.args.clock_topic} has no discovered publishers")
            elif self.clock_messages < 1:
                ready = False
                failures.append(f"{self.args.clock_topic} received no messages")

        if odom_publishers < 1 and self.odom_messages_after_start < int(self.args.min_odom_messages):
            ready = False
            failures.append(f"{self.args.odom_topic} has no discovered publishers")
        elif self.odom_messages_after_start < int(self.args.min_odom_messages):
            ready = False
            failures.append(
                f"{self.args.odom_topic} received {self.odom_messages_after_start} odom samples after command start"
            )

        if runtime_publishers < 1 and self.runtime_messages_after_start < int(self.args.min_joint_messages):
            ready = False
            failures.append(
                f"{self.args.runtime_joint_command_topic} has no discovered publishers"
            )
        elif self.runtime_messages_after_start < int(self.args.min_joint_messages):
            ready = False
            failures.append(
                f"{self.args.runtime_joint_command_topic} received {self.runtime_messages_after_start} joint command samples after command start"
            )
        elif self.last_runtime_joint_count != int(self.args.expected_joint_count):
            ready = False
            failures.append(
                f"{self.args.runtime_joint_command_topic} last_joint_count={self.last_runtime_joint_count} (expected {self.args.expected_joint_count})"
            )

        if self.args.require_backend:
            if backend_publishers < 1 and self.backend_messages_after_start < int(self.args.min_joint_messages):
                ready = False
                failures.append(
                    f"{self.args.backend_joint_command_topic} has no discovered publishers"
                )
            elif self.backend_messages_after_start < int(self.args.min_joint_messages):
                ready = False
                failures.append(
                    f"{self.args.backend_joint_command_topic} received {self.backend_messages_after_start} joint command samples after command start"
                )
            elif self.last_backend_joint_count != int(self.args.expected_joint_count):
                ready = False
                failures.append(
                    f"{self.args.backend_joint_command_topic} last_joint_count={self.last_backend_joint_count} (expected {self.args.expected_joint_count})"
                )

        if self.odom_nonfinite_messages > 0:
            ready = False
            failures.append(
                f"{self.args.odom_topic} observed {self.odom_nonfinite_messages} non-finite messages"
            )
        if self.actual_joint_state_nonfinite_messages > 0:
            ready = False
            failures.append(
                f"{self.args.joint_state_topic} observed {self.actual_joint_state_nonfinite_messages} non-finite messages"
            )
        if self.runtime_joint_command_nonfinite_messages > 0:
            ready = False
            failures.append(
                f"{self.args.runtime_joint_command_topic} observed {self.runtime_joint_command_nonfinite_messages} non-finite messages"
            )
        if (
            self.args.require_backend
            and self.backend_joint_command_nonfinite_messages > 0
        ):
            ready = False
            failures.append(
                f"{self.args.backend_joint_command_topic} observed {self.backend_joint_command_nonfinite_messages} non-finite messages"
            )

        solver_status = []
        if self.last_solver_diagnostics is not None:
            for status in self.last_solver_diagnostics.status:
                values = {}
                for key_value in status.values:
                    values[str(key_value.key)] = str(key_value.value)
                solver_status.append(
                    {
                        "name": status.name,
                        "level": diagnostic_level_to_int(status.level),
                        "message": status.message,
                        "hardware_id": status.hardware_id,
                        "values": values,
                    }
                )

        return {
            "ready": ready,
            "failures": failures,
            "probe": {
                "duration_sec": float(self.args.duration),
                "use_sim_time": bool(self.args.use_sim_time),
                "cmd_topic": self.args.cmd_topic,
                "odom_topic": self.args.odom_topic,
                "joint_state_topic": self.args.joint_state_topic,
                "clock_topic": self.args.clock_topic,
                "solver_diagnostics_topic": self.args.solver_diagnostics_topic,
                "runtime_joint_command_topic": self.args.runtime_joint_command_topic,
                "backend_joint_command_topic": self.args.backend_joint_command_topic,
                "require_backend": bool(self.args.require_backend),
                "cmd_linear_x": float(self.args.linear_x),
                "cmd_linear_y": float(self.args.linear_y),
                "cmd_angular_z": float(self.args.angular_z),
                "reference_file": str(self.args.reference_file.resolve()),
            },
            "publishers": {
                self.args.clock_topic: clock_publishers,
                self.args.odom_topic: odom_publishers,
                self.args.runtime_joint_command_topic: runtime_publishers,
                self.args.backend_joint_command_topic: backend_publishers,
            },
            "message_counts": {
                "clock": self.clock_messages,
                "odom_total": self.odom_messages,
                "odom_after_command_start": self.odom_messages_after_start,
                "joint_state_total": self.joint_state_messages,
                "joint_state_after_command_start": self.joint_state_messages_after_start,
                "runtime_joint_command_total": self.runtime_messages,
                "runtime_joint_command_after_command_start": self.runtime_messages_after_start,
                "backend_joint_command_total": self.backend_messages,
                "backend_joint_command_after_command_start": self.backend_messages_after_start,
                "solver_diagnostics_total": self.solver_diagnostics_messages,
            },
            "latency": {
                "runtime_first_message_after_command_start_sec": self.runtime_first_message_after_start_sec,
                "backend_first_message_after_command_start_sec": self.backend_first_message_after_start_sec,
            },
            "joint_counts": {
                "actual_last_joint_count": self.last_actual_joint_count,
                "runtime_last_joint_count": self.last_runtime_joint_count,
                "backend_last_joint_count": self.last_backend_joint_count,
                "expected_joint_count": int(self.args.expected_joint_count),
            },
            "backend_vs_runtime_joint_command": joint_compare,
            "expected_vs_runtime_joint_command": expected_vs_runtime_compare,
            "expected_vs_backend_joint_command": expected_vs_backend_compare,
            "expected_vs_actual_joint_state": expected_vs_state_compare,
            "command_vs_actual_joint_state": command_vs_state_compare,
            "command_state_tracking": {
                "samples_after_command_start": self.command_vs_state_samples,
                "last_max_abs_diff_rad": self.last_command_vs_state_max_abs_diff_rad,
                "max_of_max_abs_diff_rad": (
                    self.max_command_vs_state_max_abs_diff_rad
                    if self.command_vs_state_samples > 0
                    else None
                ),
                "avg_of_max_abs_diff_rad": (
                    self.command_vs_state_max_abs_diff_sum_rad
                    / float(self.command_vs_state_samples)
                )
                if self.command_vs_state_samples > 0
                else None,
            },
            "finiteness": {
                "odom_nonfinite_messages": self.odom_nonfinite_messages,
                "joint_state_nonfinite_messages": self.actual_joint_state_nonfinite_messages,
                "runtime_joint_command_nonfinite_messages": self.runtime_joint_command_nonfinite_messages,
                "backend_joint_command_nonfinite_messages": self.backend_joint_command_nonfinite_messages,
            },
            "solver_diagnostics": {
                "max_level": self.max_solver_diagnostics_level,
                "last_status": solver_status,
            },
            "motion": {
                "delta_x_m": delta_x,
                "delta_y_m": delta_y,
                "delta_yaw_rad": delta_yaw,
                "delta_forward_m": delta_forward,
                "delta_lateral_m": delta_lateral,
                "arc_expected_lateral_m": arc_expected_lateral,
                "arc_lateral_excess_m": arc_lateral_excess,
                "arc_radius_m": arc_radius,
                "max_still_sec": self.max_still_sec,
            },
        }


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ClosedLoopProbe(args)

    try:
        deadline = time.monotonic() + max(2.0, float(args.duration) + 5.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=float(args.spin_timeout))
            if node.command_finished:
                break

        node.stop_command()
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=float(args.spin_timeout))

        result = node.finalize()
        payload = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
        print(payload)
        if args.output is not None:
            args.output.write_text(payload + "\n", encoding="utf-8")
        if not result["ready"]:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
