#!/usr/bin/env python3

from __future__ import annotations

import copy
import pathlib
from typing import Any, Dict

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


def _read_yaml(path: pathlib.Path) -> Dict[str, Any]:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        return {}
    return data


class HexbotLocomotionMpcBridge(Node):
    def __init__(self) -> None:
        super().__init__("hexbot_locomotion_mpc_bridge")

        self.declare_parameter("geometry_config_path", "")
        self.declare_parameter("forward_runtime_velocity", False)
        self.declare_parameter("forward_runtime_effort", False)
        config_path = pathlib.Path(
            str(self.get_parameter("geometry_config_path").value)
        ).expanduser()
        config = _read_yaml(config_path)

        controller_cfg = config.get("controller", {})
        ocs2_cfg = config.get("ocs2", {})
        input_topics = controller_cfg.get("input_topics", {})
        bridge_topics = controller_cfg.get("bridge_topics", {})

        self.backend = str(controller_cfg.get("backend", "ocs2_ros2"))
        self.mode = str(controller_cfg.get("mode", "mpc_wbc"))
        self.forward_runtime_velocity = bool(
            self.get_parameter("forward_runtime_velocity").value
        )
        self.forward_runtime_effort = bool(
            self.get_parameter("forward_runtime_effort").value
        )

        self.cmd_vel_input_topic = str(input_topics.get("cmd_vel", "/cmd_vel"))
        self.joint_states_input_topic = str(
            input_topics.get("joint_states", "/joint_states")
        )
        self.imu_input_topic = str(input_topics.get("imu", "/imu"))
        self.odom_input_topic = str(input_topics.get("odom", "/odom"))

        self.cmd_vel_bridge_topic = str(
            bridge_topics.get("cmd_vel", "/hexbot_mpc/input/cmd_vel")
        )
        self.joint_states_bridge_topic = str(
            bridge_topics.get("joint_states", "/hexbot_mpc/input/joint_states")
        )
        self.imu_bridge_topic = str(bridge_topics.get("imu", "/hexbot_mpc/input/imu"))
        self.odom_bridge_topic = str(
            bridge_topics.get("odom", "/hexbot_mpc/input/odom")
        )
        self.backend_joint_command_topic = str(
            bridge_topics.get(
                "joint_command_backend", "/hexbot_mpc/output/joint_command"
            )
        )
        self.runtime_joint_command_topic = str(
            bridge_topics.get("joint_command_runtime", "/joint_command")
        )

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_bridge_topic, 20)
        self.joint_states_pub = self.create_publisher(
            JointState, self.joint_states_bridge_topic, 20
        )
        self.imu_pub = self.create_publisher(Imu, self.imu_bridge_topic, 20)
        self.odom_pub = self.create_publisher(Odometry, self.odom_bridge_topic, 20)
        self.joint_command_pub = self.create_publisher(
            JointState, self.runtime_joint_command_topic, 20
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_input_topic, self._forward_cmd_vel, 20
        )
        self.joint_states_sub = self.create_subscription(
            JointState,
            self.joint_states_input_topic,
            self._forward_joint_states,
            20,
        )
        self.imu_sub = self.create_subscription(
            Imu, self.imu_input_topic, self._forward_imu, 20
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_input_topic, self._forward_odom, 20
        )
        self.backend_joint_command_sub = self.create_subscription(
            JointState,
            self.backend_joint_command_topic,
            self._forward_joint_command,
            20,
        )
        self.runtime_joint_name_order: list[str] = []
        self._reported_runtime_joint_order = False
        self._reported_reorder_applied = False
        self._last_reorder_skip_reason = ""

        self._warn_missing_paths(ocs2_cfg)
        self.get_logger().info(
            "Hexbot MPC/WBC bridge ready. "
            f"backend={self.backend} mode={self.mode} config={config_path}"
        )
        self.get_logger().info(
            "Bridge topics: "
            f"{self.cmd_vel_input_topic} -> {self.cmd_vel_bridge_topic} | "
            f"{self.joint_states_input_topic} -> {self.joint_states_bridge_topic} | "
            f"{self.imu_input_topic} -> {self.imu_bridge_topic} | "
            f"{self.odom_input_topic} -> {self.odom_bridge_topic} | "
            f"{self.backend_joint_command_topic} -> {self.runtime_joint_command_topic}"
        )
        self.get_logger().info(
            "Runtime joint command passthrough: "
            f"velocity={'on' if self.forward_runtime_velocity else 'off'} "
            f"effort={'on' if self.forward_runtime_effort else 'off'}"
        )
        self.get_logger().warn(
            "This package currently provides the ROS 2 integration boundary and "
            "readiness checks. It does not ship an OCS2 solver or WBC backend itself."
        )

    def _warn_missing_paths(self, ocs2_cfg: Dict[str, Any]) -> None:
        for key in ("task_file", "reference_file", "gait_file", "urdf_path", "seed_dir"):
            raw_value = ocs2_cfg.get(key)
            if not raw_value:
                continue
            path = pathlib.Path(str(raw_value)).expanduser()
            if not path.exists():
                self.get_logger().warn(
                    f"Configured OCS2 path does not exist yet: {key}={path}"
                )

    def _forward_cmd_vel(self, message: Twist) -> None:
        self.cmd_vel_pub.publish(copy.deepcopy(message))

    def _forward_joint_states(self, message: JointState) -> None:
        if message.name:
            self.runtime_joint_name_order = list(message.name)
            if not self._reported_runtime_joint_order:
                self.get_logger().info(
                    "Captured runtime joint order from Isaac: "
                    + ", ".join(self.runtime_joint_name_order)
                )
                self._reported_runtime_joint_order = True
        self.joint_states_pub.publish(copy.deepcopy(message))

    def _forward_imu(self, message: Imu) -> None:
        self.imu_pub.publish(copy.deepcopy(message))

    def _forward_odom(self, message: Odometry) -> None:
        self.odom_pub.publish(copy.deepcopy(message))

    def _reorder_joint_command_for_runtime(
        self, message: JointState
    ) -> JointState:
        if not self.runtime_joint_name_order or not message.name:
            return copy.deepcopy(message)

        source_names = list(message.name)
        source_index = {name: index for index, name in enumerate(source_names)}
        if len(source_index) != len(source_names):
            reason = "backend joint command contains duplicate joint names"
            if reason != self._last_reorder_skip_reason:
                self.get_logger().warn(f"Skipping runtime joint reorder: {reason}")
                self._last_reorder_skip_reason = reason
            return copy.deepcopy(message)

        runtime_names = list(self.runtime_joint_name_order)
        if set(source_names) != set(runtime_names):
            reason = (
                "backend/runtime joint name sets differ: "
                f"backend={source_names} runtime={runtime_names}"
            )
            if reason != self._last_reorder_skip_reason:
                self.get_logger().warn(f"Skipping runtime joint reorder: {reason}")
                self._last_reorder_skip_reason = reason
            return copy.deepcopy(message)

        reordered = JointState()
        reordered.header = copy.deepcopy(message.header)
        reordered.name = runtime_names

        def reorder_values(values: list[float]) -> list[float]:
            return [values[source_index[name]] for name in runtime_names]

        reordered.position = (
            reorder_values(list(message.position))
            if len(message.position) == len(source_names)
            else list(message.position)
        )
        if self.forward_runtime_velocity:
            reordered.velocity = (
                reorder_values(list(message.velocity))
                if len(message.velocity) == len(source_names)
                else list(message.velocity)
            )
        if self.forward_runtime_effort:
            reordered.effort = (
                reorder_values(list(message.effort))
                if len(message.effort) == len(source_names)
                else list(message.effort)
            )

        if (
            not self._reported_reorder_applied
            and source_names != runtime_names
        ):
            self.get_logger().warn(
                "Reordering backend joint command into Isaac runtime joint order "
                f"before publishing {self.runtime_joint_command_topic}. "
                f"backend={source_names} runtime={runtime_names}"
            )
            self._reported_reorder_applied = True
        self._last_reorder_skip_reason = ""
        return reordered

    def _forward_joint_command(self, message: JointState) -> None:
        self.joint_command_pub.publish(
            self._reorder_joint_command_for_runtime(message)
        )


def main() -> None:
    rclpy.init()
    node = HexbotLocomotionMpcBridge()
    spin_exception: Exception | None = None
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        spin_exception = exc
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    if spin_exception is not None:
        raise spin_exception


if __name__ == "__main__":
    main()
