#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from ensure_isaac_clean_start import DEFAULT_REFERENCE_FILE, load_expected_joint_positions


class ReferenceJointCommandPrimer(Node):
    def __init__(
        self,
        joint_state_topic: str,
        joint_command_topic: str,
        expected_positions: dict[str, float],
        use_sim_time: bool,
    ) -> None:
        super().__init__("hexbot_reference_joint_command_primer")
        if use_sim_time:
            self.set_parameters(
                [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
            )
        self.expected_positions = dict(expected_positions)
        self.latest_joint_state: JointState | None = None
        self.joint_state_messages = 0
        self.command_messages = 0
        self.command_message: JointState | None = None
        self.joint_command_pub = self.create_publisher(JointState, joint_command_topic, 10)
        self.create_subscription(
            JointState, joint_state_topic, self._on_joint_state, 20
        )

    def _on_joint_state(self, message: JointState) -> None:
        self.latest_joint_state = message
        self.joint_state_messages += 1

    def prepare_command(self) -> tuple[bool, str]:
        if self.latest_joint_state is None:
            return False, "No /joint_states sample received yet."

        runtime_names = list(self.latest_joint_state.name)
        if not runtime_names:
            return False, "Latest /joint_states sample had an empty name array."

        missing = [
            joint_name
            for joint_name in runtime_names
            if joint_name not in self.expected_positions
        ]
        if missing:
            return (
                False,
                "Runtime joint order contains joints that are absent from the reference file: "
                + ", ".join(missing),
            )

        command = JointState()
        command.name = runtime_names
        command.position = [self.expected_positions[name] for name in runtime_names]
        self.command_message = command
        return True, ""

    def publish_once(self) -> None:
        if self.command_message is None:
            raise RuntimeError("Command message has not been prepared.")
        self.command_message.header.stamp = self.get_clock().now().to_msg()
        self.joint_command_pub.publish(self.command_message)
        self.command_messages += 1

    def build_summary(self) -> dict[str, Any]:
        summary: dict[str, Any] = {
            "joint_state_messages": self.joint_state_messages,
            "command_messages": self.command_messages,
            "command_ready": self.command_message is not None,
        }

        if self.command_message is not None:
            summary["command_joint_names"] = list(self.command_message.name)
            summary["command_positions_rad"] = list(self.command_message.position)

        if self.latest_joint_state is None:
            summary["joint_state_received"] = False
            return summary

        actual_by_name = {
            name: float(position)
            for name, position in zip(
                self.latest_joint_state.name, self.latest_joint_state.position
            )
        }
        diffs = []
        for joint_name, expected in self.expected_positions.items():
            if joint_name not in actual_by_name:
                continue
            diffs.append(
                {
                    "joint": joint_name,
                    "expected_rad": expected,
                    "actual_rad": actual_by_name[joint_name],
                    "abs_error_rad": abs(actual_by_name[joint_name] - expected),
                }
            )
        diffs.sort(key=lambda item: item["abs_error_rad"], reverse=True)
        max_abs_velocity = max(
            (abs(float(value)) for value in self.latest_joint_state.velocity), default=0.0
        )
        summary["joint_state_received"] = True
        summary["joint_state_joint_count"] = len(self.latest_joint_state.name)
        summary["max_abs_joint_error_rad"] = (
            diffs[0]["abs_error_rad"] if diffs else None
        )
        summary["max_abs_joint_velocity_rad_s"] = max_abs_velocity
        summary["top_joint_diffs"] = diffs[:6]
        return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Publish the reference joint pose on /joint_command for a short burst so "
            "Isaac's ROS subscriber caches a known-good neutral pose before OCS2 bring-up."
        )
    )
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--publish-rate", type=float, default=20.0)
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--joint-command-topic", default="/joint_command")
    parser.add_argument(
        "--reference-file",
        type=Path,
        default=DEFAULT_REFERENCE_FILE,
        help="Reference joint pose source.",
    )
    parser.add_argument(
        "--sample-timeout",
        type=float,
        default=3.0,
        help="How long to wait for the first /joint_states sample.",
    )
    parser.add_argument(
        "--max-final-error-rad",
        type=float,
        default=None,
        help="Optional threshold for the final max absolute joint error.",
    )
    parser.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use sim time for header stamps while keeping the publish duration on wall time.",
    )
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    expected_positions = load_expected_joint_positions(args.reference_file.resolve())
    publish_rate = max(args.publish_rate, 1.0)
    publish_period = 1.0 / publish_rate

    rclpy.init()
    node = ReferenceJointCommandPrimer(
        joint_state_topic=args.joint_state_topic,
        joint_command_topic=args.joint_command_topic,
        expected_positions=expected_positions,
        use_sim_time=bool(args.use_sim_time),
    )

    deadline = time.monotonic() + max(args.sample_timeout, 0.1)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        ready, _ = node.prepare_command()
        if ready:
            break

    ready, reason = node.prepare_command()
    if not ready:
        payload = {
            "ok": False,
            "reason": reason,
            "reference_file": str(args.reference_file.resolve()),
            "duration": args.duration,
            "publish_rate": publish_rate,
            "summary": node.build_summary(),
        }
        if args.output is not None:
            args.output.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        raise SystemExit(1)

    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < max(args.duration, 0.1):
        node.publish_once()
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(publish_period)

    settle_deadline = time.monotonic() + 0.5
    while rclpy.ok() and time.monotonic() < settle_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    summary = node.build_summary()
    failures: list[str] = []
    max_final_error = summary.get("max_abs_joint_error_rad")
    if args.max_final_error_rad is not None:
        if max_final_error is None or not math.isfinite(float(max_final_error)):
            failures.append(
                "Final joint error was unavailable after the priming burst."
            )
        elif float(max_final_error) > float(args.max_final_error_rad):
            failures.append(
                "Final joint error remained above the requested threshold: "
                f"{float(max_final_error):.4f} rad > {float(args.max_final_error_rad):.4f} rad."
            )

    payload = {
        "ok": len(failures) == 0,
        "failures": failures,
        "reference_file": str(args.reference_file.resolve()),
        "duration": args.duration,
        "publish_rate": publish_rate,
        "summary": summary,
    }

    if args.output is not None:
        args.output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    raise SystemExit(0 if payload["ok"] else 1)


if __name__ == "__main__":
    main()
