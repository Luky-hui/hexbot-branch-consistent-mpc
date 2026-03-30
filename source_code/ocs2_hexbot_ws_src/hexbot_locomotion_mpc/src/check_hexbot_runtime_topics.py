#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import time
from typing import Any, Dict

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Probe the live Hexbot runtime topics on the host. "
            "Use this after Isaac Sim loads the scene and the ROS bridge is active."
        )
    )
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--spin-timeout", type=float, default=0.2)
    parser.add_argument("--min-messages", type=int, default=1)
    parser.add_argument("--expected-joint-count", type=int, default=18)
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--clock-topic", default="/clock")
    parser.add_argument("--imu-topic", default="/imu")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument(
        "--require-clock",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="When enabled, /clock must be visible and receive messages.",
    )
    parser.add_argument(
        "--require-imu",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="When enabled, /imu must be visible and receive messages.",
    )
    parser.add_argument(
        "--require-cmd-vel",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="When enabled, /cmd_vel must be visible and receive messages.",
    )
    return parser.parse_args()


def _stamp_to_sec(sec: int, nanosec: int) -> float:
    return float(sec) + float(nanosec) * 1.0e-9


class RuntimeTopicProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("hexbot_runtime_topic_probe")
        self.args = args
        self._subscription_handles = []
        self.topic_stats: Dict[str, Dict[str, Any]] = {
            args.joint_states_topic: {
                "required": True,
                "publishers": 0,
                "received": 0,
                "last_header_stamp_sec": None,
                "last_wall_time_sec": None,
                "last_joint_name_count": 0,
                "last_position_count": 0,
            },
            args.odom_topic: {
                "required": True,
                "publishers": 0,
                "received": 0,
                "last_header_stamp_sec": None,
                "last_wall_time_sec": None,
                "last_frame_id": "",
                "last_child_frame_id": "",
            },
            args.clock_topic: {
                "required": bool(args.require_clock),
                "publishers": 0,
                "received": 0,
                "last_sim_time_sec": None,
                "last_wall_time_sec": None,
            },
            args.imu_topic: {
                "required": bool(args.require_imu),
                "publishers": 0,
                "received": 0,
                "last_header_stamp_sec": None,
                "last_wall_time_sec": None,
                "last_frame_id": "",
            },
            args.cmd_vel_topic: {
                "required": bool(args.require_cmd_vel),
                "publishers": 0,
                "received": 0,
                "last_wall_time_sec": None,
                "last_linear_x": None,
                "last_linear_y": None,
                "last_angular_z": None,
            },
        }

        self._subscription_handles.append(
            self.create_subscription(
                JointState, args.joint_states_topic, self._joint_state_callback, 20
            )
        )
        self._subscription_handles.append(
            self.create_subscription(Odometry, args.odom_topic, self._odom_callback, 20)
        )
        self._subscription_handles.append(
            self.create_subscription(Clock, args.clock_topic, self._clock_callback, 20)
        )
        self._subscription_handles.append(
            self.create_subscription(Imu, args.imu_topic, self._imu_callback, 20)
        )
        self._subscription_handles.append(
            self.create_subscription(Twist, args.cmd_vel_topic, self._cmd_vel_callback, 20)
        )

    def _mark_received(self, topic: str) -> Dict[str, Any]:
        stats = self.topic_stats[topic]
        stats["received"] += 1
        stats["last_wall_time_sec"] = time.time()
        return stats

    def _joint_state_callback(self, message: JointState) -> None:
        stats = self._mark_received(self.args.joint_states_topic)
        stats["last_header_stamp_sec"] = _stamp_to_sec(
            message.header.stamp.sec, message.header.stamp.nanosec
        )
        stats["last_joint_name_count"] = len(message.name)
        stats["last_position_count"] = len(message.position)

    def _odom_callback(self, message: Odometry) -> None:
        stats = self._mark_received(self.args.odom_topic)
        stats["last_header_stamp_sec"] = _stamp_to_sec(
            message.header.stamp.sec, message.header.stamp.nanosec
        )
        stats["last_frame_id"] = message.header.frame_id
        stats["last_child_frame_id"] = message.child_frame_id

    def _clock_callback(self, message: Clock) -> None:
        stats = self._mark_received(self.args.clock_topic)
        stats["last_sim_time_sec"] = _stamp_to_sec(
            message.clock.sec, message.clock.nanosec
        )

    def _imu_callback(self, message: Imu) -> None:
        stats = self._mark_received(self.args.imu_topic)
        stats["last_header_stamp_sec"] = _stamp_to_sec(
            message.header.stamp.sec, message.header.stamp.nanosec
        )
        stats["last_frame_id"] = message.header.frame_id

    def _cmd_vel_callback(self, message: Twist) -> None:
        stats = self._mark_received(self.args.cmd_vel_topic)
        stats["last_linear_x"] = float(message.linear.x)
        stats["last_linear_y"] = float(message.linear.y)
        stats["last_angular_z"] = float(message.angular.z)

    def finalize(self) -> Dict[str, Any]:
        now_sec = time.time()
        ready = True
        failures = []

        for topic, stats in self.topic_stats.items():
            stats["publishers"] = self.count_publishers(topic)
            last_wall_time_sec = stats.get("last_wall_time_sec")
            stats["last_wall_age_sec"] = (
                None if last_wall_time_sec is None else now_sec - last_wall_time_sec
            )

            if not stats["required"]:
                continue

            if stats["publishers"] < 1:
                ready = False
                failures.append(f"{topic} has no discovered publishers")
                continue

            if int(stats["received"]) < int(self.args.min_messages):
                ready = False
                failures.append(
                    f"{topic} received {stats['received']} messages "
                    f"(expected at least {self.args.min_messages})"
                )
                continue

            if topic == self.args.joint_states_topic:
                last_joint_name_count = int(stats.get("last_joint_name_count", 0))
                if last_joint_name_count != int(self.args.expected_joint_count):
                    ready = False
                    failures.append(
                        f"{topic} last_joint_name_count={last_joint_name_count} "
                        f"(expected {self.args.expected_joint_count})"
                    )

        return {
            "ready": ready,
            "probe_duration_sec": float(self.args.duration),
            "min_messages": int(self.args.min_messages),
            "expected_joint_count": int(self.args.expected_joint_count),
            "topics": self.topic_stats,
            "failures": failures,
        }


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = RuntimeTopicProbe(args)

    try:
        deadline = time.time() + float(args.duration)
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=float(args.spin_timeout))
        result = node.finalize()
        print(json.dumps(result, indent=2, sort_keys=True))
        if not result["ready"]:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
