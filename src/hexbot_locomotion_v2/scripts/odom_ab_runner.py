#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock


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


class OdomABRunner(Node):
    def __init__(
        self,
        duration_sec: float,
        use_sim_time: bool,
        cmd_topic: str,
        odom_topic: str,
        cmd_linear_x: float,
        cmd_linear_y: float,
        cmd_angular_z: float,
        cmd_publish_rate_hz: float,
        linear_motion_threshold_mps: float,
        angular_motion_threshold_rad_s: float,
    ) -> None:
        super().__init__("hexbot_odom_ab_runner")
        self.use_sim_time = bool(use_sim_time)
        if self.use_sim_time:
            self.set_parameters(
                [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
            )

        self.duration_sec = max(0.1, float(duration_sec))
        self.cmd_topic = str(cmd_topic)
        self.odom_topic = str(odom_topic)
        self.cmd_linear_x = float(cmd_linear_x)
        self.cmd_linear_y = float(cmd_linear_y)
        self.cmd_angular_z = float(cmd_angular_z)
        self.cmd_publish_rate_hz = max(1.0, float(cmd_publish_rate_hz))
        self.linear_motion_threshold_mps = max(0.0, float(linear_motion_threshold_mps))
        self.angular_motion_threshold_rad_s = max(
            0.0, float(angular_motion_threshold_rad_s)
        )

        self.wall_start_sec = time.monotonic()
        self.clock_start_sec: float | None = None
        self.clock_now_sec: float | None = None

        self.odom_messages = 0
        self.started = False
        self.finished = False

        self.start_pose: dict[str, float] | None = None
        self.end_pose: dict[str, float] | None = None
        self.previous_odom_time_sec: float | None = None
        self.previous_odom_yaw_unwrapped: float | None = None
        self.current_still_sec = 0.0
        self.max_still_sec = 0.0

        self.cmd_message = Twist()
        self.cmd_message.linear.x = self.cmd_linear_x
        self.cmd_message.linear.y = self.cmd_linear_y
        self.cmd_message.angular.z = self.cmd_angular_z

        self.create_subscription(Clock, "/clock", self._clock_callback, 20)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 20)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.cmd_timer = self.create_timer(
            1.0 / self.cmd_publish_rate_hz, self._cmd_timer_callback
        )

    def _clock_callback(self, message: Clock) -> None:
        clock_sec = float(message.clock.sec) + float(message.clock.nanosec) * 1.0e-9
        if self.clock_start_sec is None:
            self.clock_start_sec = clock_sec
        self.clock_now_sec = clock_sec

    def _elapsed_sec(self) -> float | None:
        if not self.use_sim_time:
            return time.monotonic() - self.wall_start_sec
        if self.clock_start_sec is None or self.clock_now_sec is None:
            return None
        return self.clock_now_sec - self.clock_start_sec

    def _odom_callback(self, message: Odometry) -> None:
        elapsed_sec = self._elapsed_sec()
        self.odom_messages += 1
        if elapsed_sec is None:
            return

        position_x = float(message.pose.pose.position.x)
        position_y = float(message.pose.pose.position.y)
        yaw_unwrapped = unwrap_angle(
            self.previous_odom_yaw_unwrapped,
            quaternion_to_yaw(message.pose.pose.orientation),
        )

        if not self.started:
            self.started = True
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
            moving = (
                linear_speed > self.linear_motion_threshold_mps
                or angular_speed > self.angular_motion_threshold_rad_s
            )
            command_active = any(
                abs(value) > 1.0e-9
                for value in (self.cmd_linear_x, self.cmd_linear_y, self.cmd_angular_z)
            )
            if command_active and not moving:
                self.current_still_sec += delta_time_sec
                self.max_still_sec = max(self.max_still_sec, self.current_still_sec)
            else:
                self.current_still_sec = 0.0

        self.previous_odom_time_sec = elapsed_sec
        self.previous_odom_yaw_unwrapped = yaw_unwrapped

    def _cmd_timer_callback(self) -> None:
        elapsed_sec = self._elapsed_sec()
        if elapsed_sec is None:
            return
        if self.finished:
            self.cmd_pub.publish(Twist())
            return
        if elapsed_sec < self.duration_sec:
            self.cmd_pub.publish(self.cmd_message)
            return
        self.finished = True
        self.cmd_pub.publish(Twist())

    def timed_out_waiting_for_clock(self) -> bool:
        return (
            self.use_sim_time
            and self.clock_now_sec is None
            and (time.monotonic() - self.wall_start_sec) > 5.0
        )

    def timed_out_waiting_for_odom(self) -> bool:
        return self.odom_messages == 0 and (time.monotonic() - self.wall_start_sec) > 5.0

    def stop_command(self, repeat_count: int = 3) -> None:
        stop_message = Twist()
        for _ in range(max(1, int(repeat_count))):
            self.cmd_pub.publish(stop_message)
            time.sleep(0.05)

    def result(self) -> dict[str, float | int | bool | None]:
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
                if abs(sin_yaw_delta) > 1.0e-6:
                    arc_radius = delta_forward / sin_yaw_delta
                else:
                    arc_radius = None
            else:
                arc_expected_lateral = 0.0
                arc_lateral_excess = delta_lateral
                arc_radius = None

        return {
            "duration_sec": self.duration_sec,
            "use_sim_time": self.use_sim_time,
            "cmd_topic": self.cmd_topic,
            "odom_topic": self.odom_topic,
            "cmd_linear_x": self.cmd_linear_x,
            "cmd_linear_y": self.cmd_linear_y,
            "cmd_angular_z": self.cmd_angular_z,
            "cmd_publish_rate_hz": self.cmd_publish_rate_hz,
            "odom_messages": self.odom_messages,
            "delta_x_m": delta_x,
            "delta_y_m": delta_y,
            "delta_yaw_rad": delta_yaw,
            "delta_forward_m": delta_forward,
            "delta_lateral_m": delta_lateral,
            "arc_expected_lateral_m": arc_expected_lateral,
            "arc_lateral_excess_m": arc_lateral_excess,
            "arc_radius_m": arc_radius,
            "max_still_sec": self.max_still_sec,
        }


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Publish a fixed /cmd_vel command for a fixed interval and measure "
            "odom delta pose plus the longest still segment."
        )
    )
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--cmd-topic", type=str, default="/cmd_vel")
    parser.add_argument("--odom-topic", type=str, default="/odom")
    parser.add_argument("--linear-x", type=float, default=0.0)
    parser.add_argument("--linear-y", type=float, default=0.0)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--cmd-publish-rate", type=float, default=10.0)
    parser.add_argument("--linear-motion-threshold", type=float, default=0.005)
    parser.add_argument("--angular-motion-threshold", type=float, default=0.02)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    rclpy.init()
    node = OdomABRunner(
        duration_sec=args.duration,
        use_sim_time=args.use_sim_time,
        cmd_topic=args.cmd_topic,
        odom_topic=args.odom_topic,
        cmd_linear_x=args.linear_x,
        cmd_linear_y=args.linear_y,
        cmd_angular_z=args.angular_z,
        cmd_publish_rate_hz=args.cmd_publish_rate,
        linear_motion_threshold_mps=args.linear_motion_threshold,
        angular_motion_threshold_rad_s=args.angular_motion_threshold,
    )
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.timed_out_waiting_for_clock():
                raise RuntimeError(
                    "Timed out waiting for /clock while use_sim_time is enabled."
                )
            if node.timed_out_waiting_for_odom():
                raise RuntimeError("Timed out waiting for /odom.")
        result = node.result()
    finally:
        node.stop_command()
        node.destroy_node()
        rclpy.shutdown()

    payload = json.dumps(result, indent=2, sort_keys=True)
    print(payload)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
