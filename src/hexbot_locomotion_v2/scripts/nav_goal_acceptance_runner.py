#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener


PRESETS: dict[str, dict[str, float | str]] = {
    "nav_fwd_0p5": {
        "label": "forward 0.5 m",
        "forward_m": 0.5,
        "relative_yaw_rad": 0.0,
    },
    "nav_fwd_1p0": {
        "label": "forward 1.0 m",
        "forward_m": 1.0,
        "relative_yaw_rad": 0.0,
    },
    "nav_turn15_fwd_0p5": {
        "label": "turn 15 deg then forward 0.5 m",
        "forward_m": 0.5,
        "relative_yaw_rad": math.radians(15.0),
    },
}


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


def yaw_to_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * float(yaw_rad)
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def wrap_to_pi(angle_rad: float) -> float:
    wrapped = float(angle_rad)
    while wrapped > math.pi:
        wrapped -= 2.0 * math.pi
    while wrapped < -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


class NavGoalAcceptanceRunner(Node):
    def __init__(
        self,
        preset: str,
        use_sim_time: bool,
        goal_frame: str,
        robot_frame: str,
        odom_topic: str,
        action_name: str,
        timeout_sec: float,
        linear_motion_threshold_mps: float,
        angular_motion_threshold_rad_s: float,
    ) -> None:
        super().__init__("hexbot_nav_goal_acceptance_runner")
        if use_sim_time:
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.preset = preset
        self.goal_frame = str(goal_frame)
        self.robot_frame = str(robot_frame)
        self.odom_topic = str(odom_topic)
        self.action_name = str(action_name)
        self.timeout_sec = float(timeout_sec)
        self.linear_motion_threshold_mps = float(linear_motion_threshold_mps)
        self.angular_motion_threshold_rad_s = float(angular_motion_threshold_rad_s)

        self.odom_messages = 0
        self.latest_odom_stamp_sec: float | None = None
        self.previous_motion_stamp_sec: float | None = None
        self.current_still_sec = 0.0
        self.max_still_sec = 0.0
        self.measure_active = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.navigate_client = ActionClient(self, NavigateToPose, self.action_name)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 20)

    def _odom_callback(self, message: Odometry) -> None:
        self.odom_messages += 1
        stamp_sec = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9
        self.latest_odom_stamp_sec = stamp_sec
        if not self.measure_active:
            return

        if self.previous_motion_stamp_sec is None:
            self.previous_motion_stamp_sec = stamp_sec
            return

        delta_sec = max(0.0, stamp_sec - self.previous_motion_stamp_sec)
        self.previous_motion_stamp_sec = stamp_sec

        linear_speed = math.hypot(
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        )
        angular_speed = abs(float(message.twist.twist.angular.z))
        moving = (
            linear_speed > self.linear_motion_threshold_mps
            or angular_speed > self.angular_motion_threshold_rad_s
        )
        if moving:
            self.current_still_sec = 0.0
        else:
            self.current_still_sec += delta_sec
            self.max_still_sec = max(self.max_still_sec, self.current_still_sec)

    def _lookup_robot_pose(self) -> dict[str, float]:
        transform = self.tf_buffer.lookup_transform(
            self.goal_frame,
            self.robot_frame,
            rclpy.time.Time(),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            "x": float(translation.x),
            "y": float(translation.y),
            "yaw": quaternion_to_yaw(rotation),
        }

    def wait_for_ready(self, timeout_sec: float) -> dict[str, object]:
        start_sec = time.monotonic()
        while time.monotonic() - start_sec < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            odom_ok = self.odom_messages > 0
            action_ok = self.navigate_client.wait_for_server(timeout_sec=0.1)
            tf_ok = False
            try:
                tf_ok = self.tf_buffer.can_transform(
                    self.goal_frame, self.robot_frame, rclpy.time.Time()
                )
            except Exception:
                tf_ok = False
            if odom_ok and action_ok and tf_ok:
                return {
                    "ready": True,
                    "odom_ok": odom_ok,
                    "action_ok": action_ok,
                    "tf_ok": tf_ok,
                }
        return {
            "ready": False,
            "odom_ok": self.odom_messages > 0,
            "action_ok": self.navigate_client.server_is_ready(),
            "tf_ok": False,
        }

    def build_goal_pose(self) -> tuple[dict[str, float], dict[str, float], float]:
        preset = PRESETS[self.preset]
        start_pose = self._lookup_robot_pose()
        relative_yaw_rad = float(preset["relative_yaw_rad"])
        forward_m = float(preset["forward_m"])
        goal_yaw = wrap_to_pi(start_pose["yaw"] + relative_yaw_rad)
        goal_x = start_pose["x"] + math.cos(goal_yaw) * forward_m
        goal_y = start_pose["y"] + math.sin(goal_yaw) * forward_m
        return start_pose, {"x": goal_x, "y": goal_y, "yaw": goal_yaw}, goal_yaw

    def run_goal(self) -> dict[str, object]:
        preset = PRESETS[self.preset]
        start_pose, goal_pose, path_heading_rad = self.build_goal_pose()

        goal_message = NavigateToPose.Goal()
        goal_message.pose.header.frame_id = self.goal_frame
        goal_message.pose.header.stamp = self.get_clock().now().to_msg()
        goal_message.pose.pose.position.x = goal_pose["x"]
        goal_message.pose.pose.position.y = goal_pose["y"]
        goal_message.pose.pose.position.z = 0.0
        quat_x, quat_y, quat_z, quat_w = yaw_to_quaternion(goal_pose["yaw"])
        goal_message.pose.pose.orientation.x = quat_x
        goal_message.pose.pose.orientation.y = quat_y
        goal_message.pose.pose.orientation.z = quat_z
        goal_message.pose.pose.orientation.w = quat_w

        send_future = self.navigate_client.send_goal_async(goal_message)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {
                "preset": self.preset,
                "label": str(preset["label"]),
                "success": False,
                "goal_accepted": False,
                "nav_status": "rejected",
                "start_pose": start_pose,
                "goal_pose": goal_pose,
                "final_pose": None,
                "max_still_sec": 0.0,
            }

        result_future = goal_handle.get_result_async()
        self.measure_active = True
        self.previous_motion_stamp_sec = None
        start_wait_sec = time.monotonic()
        timed_out = False
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start_wait_sec >= self.timeout_sec:
                timed_out = True
                cancel_future = goal_handle.cancel_goal_async()
                while rclpy.ok() and not cancel_future.done():
                    rclpy.spin_once(self, timeout_sec=0.1)
                break
        self.measure_active = False

        final_pose = None
        try:
            final_pose = self._lookup_robot_pose()
        except Exception:
            final_pose = None

        if timed_out:
            return {
                "preset": self.preset,
                "label": str(preset["label"]),
                "success": False,
                "goal_accepted": True,
                "nav_status": "timeout",
                "start_pose": start_pose,
                "goal_pose": goal_pose,
                "final_pose": final_pose,
                "max_still_sec": self.max_still_sec,
            }

        result = result_future.result()
        status = int(result.status)
        success = status == GoalStatus.STATUS_SUCCEEDED

        goal_error_x = None
        goal_error_y = None
        goal_distance_error_m = None
        lateral_error_m = None
        longitudinal_error_m = None
        terminal_yaw_error_rad = None
        if final_pose is not None:
            goal_error_x = final_pose["x"] - goal_pose["x"]
            goal_error_y = final_pose["y"] - goal_pose["y"]
            goal_distance_error_m = math.hypot(goal_error_x, goal_error_y)
            longitudinal_error_m = (
                math.cos(path_heading_rad) * goal_error_x
                + math.sin(path_heading_rad) * goal_error_y
            )
            lateral_error_m = (
                -math.sin(path_heading_rad) * goal_error_x
                + math.cos(path_heading_rad) * goal_error_y
            )
            terminal_yaw_error_rad = wrap_to_pi(final_pose["yaw"] - goal_pose["yaw"])

        return {
            "preset": self.preset,
            "label": str(preset["label"]),
            "success": success,
            "goal_accepted": True,
            "nav_status": status,
            "goal_timeout_sec": self.timeout_sec,
            "goal_frame": self.goal_frame,
            "start_pose": start_pose,
            "goal_pose": goal_pose,
            "final_pose": final_pose,
            "goal_error_x_m": goal_error_x,
            "goal_error_y_m": goal_error_y,
            "goal_distance_error_m": goal_distance_error_m,
            "lateral_error_m": lateral_error_m,
            "longitudinal_error_m": longitudinal_error_m,
            "terminal_yaw_error_rad": terminal_yaw_error_rad,
            "max_still_sec": self.max_still_sec,
            "pause_gt_2s_measured": self.max_still_sec > 2.0,
            "odom_messages": self.odom_messages,
        }


def prompt_yes_no(prompt_text: str, default_no: bool = True) -> bool:
    raw = input(prompt_text).strip().lower()
    if raw == "":
        return not default_no
    return raw in {"y", "yes"}


def write_markdown(result_path: Path, payload: dict[str, object]) -> None:
    lines = [
        f"# Navigation Acceptance Result: {payload['preset']}",
        "",
        f"- success: {'yes' if payload['success'] else 'no'}",
        f"- trip_drag: {'yes' if payload['trip_drag'] else 'no'}",
        f"- pause_gt_2s: {'yes' if payload['pause_gt_2s'] else 'no'}",
        f"- pause_gt_2s_measured: {'yes' if payload['pause_gt_2s_measured'] else 'no'}",
        f"- lateral_error_m: {payload['lateral_error_m']}",
        f"- terminal_yaw_error_rad: {payload['terminal_yaw_error_rad']}",
        f"- goal_distance_error_m: {payload['goal_distance_error_m']}",
        f"- max_still_sec: {payload['max_still_sec']}",
    ]
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Send one NavigateToPose goal relative to the current robot pose and "
            "record fresh-start navigation acceptance metrics."
        )
    )
    parser.add_argument("--preset", required=True, choices=sorted(PRESETS.keys()))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--result-md", type=Path, required=True)
    parser.add_argument("--geometry-config-path", type=Path, required=True)
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--goal-frame", default="map")
    parser.add_argument("--robot-frame", default="base_link")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--action-name", default="navigate_to_pose")
    parser.add_argument("--ready-timeout-sec", type=float, default=20.0)
    parser.add_argument("--goal-timeout-sec", type=float, default=120.0)
    parser.add_argument("--linear-motion-threshold", type=float, default=0.005)
    parser.add_argument("--angular-motion-threshold", type=float, default=0.02)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = NavGoalAcceptanceRunner(
        preset=args.preset,
        use_sim_time=args.use_sim_time,
        goal_frame=args.goal_frame,
        robot_frame=args.robot_frame,
        odom_topic=args.odom_topic,
        action_name=args.action_name,
        timeout_sec=args.goal_timeout_sec,
        linear_motion_threshold_mps=args.linear_motion_threshold,
        angular_motion_threshold_rad_s=args.angular_motion_threshold,
    )
    try:
        ready_payload = node.wait_for_ready(args.ready_timeout_sec)
        if not ready_payload["ready"]:
            payload = {
                "preset": args.preset,
                "geometry_config_path": str(args.geometry_config_path),
                "ready": False,
                **ready_payload,
            }
            serialized = json.dumps(payload, indent=2, sort_keys=True)
            print(serialized)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(serialized + "\n", encoding="utf-8")
            return 1

        payload = node.run_goal()
        payload["ready"] = True
        payload["geometry_config_path"] = str(args.geometry_config_path)
        payload["trip_drag"] = prompt_yes_no(
            f"[{args.preset}] Obvious trip/drag? [y/N]: ", default_no=True
        )
        measured_pause = bool(payload["pause_gt_2s_measured"])
        pause_prompt = (
            f"[{args.preset}] Pause >2s? "
            f"[Enter=use measured {'yes' if measured_pause else 'no'}, y/N]: "
        )
        pause_override = input(pause_prompt).strip().lower()
        if pause_override == "":
            payload["pause_gt_2s"] = measured_pause
        else:
            payload["pause_gt_2s"] = pause_override in {"y", "yes"}

        serialized = json.dumps(payload, indent=2, sort_keys=True)
        print(serialized)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized + "\n", encoding="utf-8")
        write_markdown(args.result_md, payload)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
