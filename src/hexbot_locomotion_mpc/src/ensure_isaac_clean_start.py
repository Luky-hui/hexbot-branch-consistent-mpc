#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


THIS_FILE = Path(__file__).resolve()
WORKSPACE_ROOT = THIS_FILE.parents[3]
DEFAULT_REFERENCE_FILE = (
    WORKSPACE_ROOT
    / "src"
    / "ocs2_hexbot_legged_robot"
    / "config"
    / "command"
    / "reference.info"
)
DEFAULT_EXPECTED_JOINT_COUNT = 18
REFERENCE_LINE_PATTERN = re.compile(
    r"\(\s*\d+\s*,\s*0\s*\)\s*([-+0-9.eE]+)\s*;\s*([A-Za-z0-9_]+)"
)


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class IsaacStateProbe(Node):
    def __init__(self, odom_topic: str, joint_state_topic: str) -> None:
        super().__init__("hexbot_isaac_clean_start_probe")
        self.latest_odom: Odometry | None = None
        self.latest_joint_state: JointState | None = None
        self.odom_count = 0
        self.joint_state_count = 0
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 10)

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.odom_count += 1

    def _on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg
        self.joint_state_count += 1


def load_expected_joint_positions(reference_file: Path) -> dict[str, float]:
    text = reference_file.read_text(encoding="utf-8")
    expected: dict[str, float] = {}
    inside_default_joint_state = False

    for line in text.splitlines():
        stripped = line.strip()
        if not inside_default_joint_state:
            if stripped.startswith("defaultJointState"):
                inside_default_joint_state = True
            continue

        if stripped == "}":
            break

        match = REFERENCE_LINE_PATTERN.search(line)
        if match is None:
            continue
        expected[match.group(2)] = float(match.group(1))

    if len(expected) != DEFAULT_EXPECTED_JOINT_COUNT:
        raise ValueError(
            f"Expected {DEFAULT_EXPECTED_JOINT_COUNT} joints in {reference_file}, "
            f"parsed {len(expected)}."
        )
    return expected


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check whether Isaac Sim is in a clean, upright start state before running control experiments."
    )
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument(
        "--reference-file",
        type=Path,
        default=DEFAULT_REFERENCE_FILE,
        help="Reference stance used to validate the idle joint posture.",
    )
    parser.add_argument(
        "--max-joint-position-error-rad",
        type=float,
        default=0.25,
        help=(
            "Maximum allowed absolute joint position error versus the reference stance. "
            "Keep this loose enough for passive Isaac settle, and tighten it only when "
            "the asset is intentionally held at a stand pose."
        ),
    )
    parser.add_argument(
        "--min-base-z",
        type=float,
        default=None,
        help=(
            "Optional absolute base z threshold in the odom/world frame. "
            "Disabled by default because many Isaac scenes do not place the ground at z=0."
        ),
    )
    parser.add_argument("--max-abs-roll-rad", type=float, default=0.35)
    parser.add_argument("--max-abs-pitch-rad", type=float, default=0.35)
    parser.add_argument("--max-joint-speed-rad-s", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    expected_joint_positions = load_expected_joint_positions(args.reference_file.resolve())
    rclpy.init()
    node = IsaacStateProbe(args.odom_topic, args.joint_state_topic)

    end_time_ns = node.get_clock().now().nanoseconds + int(max(args.duration, 0.1) * 1.0e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end_time_ns:
        rclpy.spin_once(node, timeout_sec=0.1)

    failures: list[str] = []
    odom_summary: dict[str, Any] = {"available": False, "received": node.odom_count}
    joint_summary: dict[str, Any] = {"available": False, "received": node.joint_state_count}

    if node.latest_odom is None:
        failures.append("No /odom sample received.")
    else:
        odom = node.latest_odom
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_rpy(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        linear = odom.twist.twist.linear
        angular = odom.twist.twist.angular
        odom_summary = {
            "available": True,
            "received": node.odom_count,
            "position_xyz_m": [float(position.x), float(position.y), float(position.z)],
            "rpy_rad": [roll, pitch, yaw],
            "linear_xyz_mps": [float(linear.x), float(linear.y), float(linear.z)],
            "angular_xyz_radps": [float(angular.x), float(angular.y), float(angular.z)],
        }
        if args.min_base_z is not None and float(position.z) < args.min_base_z:
            failures.append(
                f"Base height too low for a clean start: z={float(position.z):.4f} m < {args.min_base_z:.4f} m."
            )
        if abs(roll) > args.max_abs_roll_rad:
            failures.append(
                f"Roll tilt too large for a clean start: |roll|={abs(roll):.4f} rad > {args.max_abs_roll_rad:.4f} rad."
            )
        if abs(pitch) > args.max_abs_pitch_rad:
            failures.append(
                f"Pitch tilt too large for a clean start: |pitch|={abs(pitch):.4f} rad > {args.max_abs_pitch_rad:.4f} rad."
            )

    if node.latest_joint_state is None:
        failures.append("No /joint_states sample received.")
    else:
        msg = node.latest_joint_state
        max_abs_velocity = max((abs(float(v)) for v in msg.velocity), default=0.0)
        state_by_name = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
        }
        missing_expected_joints = [
            joint_name
            for joint_name in expected_joint_positions
            if joint_name not in state_by_name
        ]
        joint_position_diffs = [
            {
                "joint": joint_name,
                "expected_rad": expected_joint_positions[joint_name],
                "actual_rad": state_by_name[joint_name],
                "abs_error_rad": abs(
                    state_by_name[joint_name] - expected_joint_positions[joint_name]
                ),
            }
            for joint_name in expected_joint_positions
            if joint_name in state_by_name
        ]
        joint_position_diffs.sort(
            key=lambda item: item["abs_error_rad"], reverse=True
        )
        max_abs_position_error = (
            joint_position_diffs[0]["abs_error_rad"] if joint_position_diffs else None
        )
        joint_summary = {
            "available": True,
            "received": node.joint_state_count,
            "joint_count": len(msg.name),
            "max_abs_velocity_rad_s": max_abs_velocity,
            "reference_file": str(args.reference_file.resolve()),
            "max_abs_position_error_rad": max_abs_position_error,
            "missing_expected_joints": missing_expected_joints,
            "top_position_diffs": joint_position_diffs[:6],
        }
        if max_abs_velocity > args.max_joint_speed_rad_s:
            failures.append(
                f"Joint velocity too large for a clean start: {max_abs_velocity:.4f} rad/s > {args.max_joint_speed_rad_s:.4f} rad/s."
            )
        if missing_expected_joints:
            failures.append(
                "Joint state is missing expected joints: "
                + ", ".join(missing_expected_joints)
            )
        if (
            max_abs_position_error is not None
            and max_abs_position_error > args.max_joint_position_error_rad
        ):
            failures.append(
                "Joint position error too large for a clean start: "
                f"{max_abs_position_error:.4f} rad > {args.max_joint_position_error_rad:.4f} rad."
            )

    payload = {
        "clean": len(failures) == 0,
        "failures": failures,
        "odom": odom_summary,
        "joint_states": joint_summary,
        "thresholds": {
            "min_base_z": args.min_base_z,
            "max_abs_roll_rad": args.max_abs_roll_rad,
            "max_abs_pitch_rad": args.max_abs_pitch_rad,
            "max_joint_speed_rad_s": args.max_joint_speed_rad_s,
            "max_joint_position_error_rad": args.max_joint_position_error_rad,
        },
    }

    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    raise SystemExit(0 if payload["clean"] else 1)


if __name__ == "__main__":
    main()
