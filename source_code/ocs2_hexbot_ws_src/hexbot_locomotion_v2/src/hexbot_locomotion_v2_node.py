#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    from cmd_filter import CommandFilter
    from foot_trajectory import FootTrajectoryGenerator
    from hexapod_geometry import HexapodGeometry, load_yaml
    from imu_state_estimator import ImuStateEstimator
    from leg_ik import LegIKSolver
    from locomotion_state_machine import (
        FAULT,
        READY,
        RECOVER_TO_STAND,
        STOPPING_TO_STAND,
        WAIT_FOR_VALID_SEED,
        WALKING,
        LocomotionStateMachine,
    )
    from pose_stabilizer import PoseStabilizer
    from tripod_scheduler import TripodScheduler
except ImportError:
    from .cmd_filter import CommandFilter
    from .foot_trajectory import FootTrajectoryGenerator
    from .hexapod_geometry import HexapodGeometry, load_yaml
    from .imu_state_estimator import ImuStateEstimator
    from .leg_ik import LegIKSolver
    from .locomotion_state_machine import (
        FAULT,
        READY,
        RECOVER_TO_STAND,
        STOPPING_TO_STAND,
        WAIT_FOR_VALID_SEED,
        WALKING,
        LocomotionStateMachine,
    )
    from .pose_stabilizer import PoseStabilizer
    from .tripod_scheduler import TripodScheduler


TRIPOD_MODES = {"cmd_vel_tripod", "tripod"}


def clamp(value: float, lower: float, upper: float) -> float:
    if lower > upper:
        lower, upper = upper, lower
    return max(lower, min(upper, value))


def copy_vector_map(source: dict[str, list[float]]) -> dict[str, list[float]]:
    return {name: list(values) for name, values in source.items()}


def add_vector_map_offsets(
    source: dict[str, list[float]], offsets: dict[str, list[float]]
) -> dict[str, list[float]]:
    result: dict[str, list[float]] = {}
    for name, values in source.items():
        offset = offsets.get(name, [0.0, 0.0, 0.0])
        result[name] = [
            float(values[index]) + float(offset[index]) for index in range(3)
        ]
    return result


def default_tripod_turn_leg_biases(leg_order: list[str]) -> dict[str, float]:
    biases = {leg_name: 1.0 for leg_name in leg_order}
    for leg_name in ("RM", "LM"):
        if leg_name in biases:
            biases[leg_name] = 1.42
    return biases


class HexbotLocomotionV2Node(Node):
    def __init__(self) -> None:
        super().__init__("hexbot_locomotion_v2")

        self.declare_parameter("joint_config_path", "")
        self.declare_parameter("geometry_config_path", "")

        joint_config_path = self._resolve_config_path(
            self.get_parameter("joint_config_path").value,
            "config/joint_name_list.yaml",
        )
        geometry_config_path = self._resolve_config_path(
            self.get_parameter("geometry_config_path").value,
            "config/hexbot_v2_stage2_explicit_current_candidate.yaml",
        )

        raw_config = load_yaml(geometry_config_path)
        controller_cfg = dict(raw_config["controller"])
        imu_cfg = dict(raw_config["imu_estimator"])
        stabilizer_cfg = dict(raw_config["stabilizer"])
        gait_cfg = dict(raw_config["gait"])
        raw_stance_cfg = raw_config.get("stance", {})
        if raw_stance_cfg is None:
            stance_cfg: dict[str, object] = {}
        elif isinstance(raw_stance_cfg, dict):
            stance_cfg = dict(raw_stance_cfg)
        else:
            raise ValueError("stance must be a mapping when provided.")

        self.geometry = HexapodGeometry.from_config_files(
            joint_config_path, geometry_config_path
        )
        self.ik_solver = LegIKSolver.from_config(self.geometry.ik_config)
        self.tripod_turn_leg_biases = self._load_tripod_turn_leg_biases(gait_cfg)
        self.tripod_leg_home_offsets = self._load_tripod_leg_home_offsets(gait_cfg)

        self.declare_parameter("mode", controller_cfg["default_mode"])
        self.declare_parameter("publish_rate_hz", controller_cfg["publish_rate_hz"])
        self.declare_parameter("joint_state_topic", controller_cfg["joint_state_topic"])
        self.declare_parameter(
            "joint_command_topic", controller_cfg["joint_command_topic"]
        )
        self.declare_parameter("cmd_vel_topic", controller_cfg["cmd_vel_topic"])
        self.declare_parameter("imu_topic", controller_cfg["imu_topic"])
        self.declare_parameter("odom_frame", controller_cfg["odom_frame"])
        self.declare_parameter("base_frame", controller_cfg["base_frame"])
        self.declare_parameter("imu_frame", controller_cfg["imu_frame"])
        self.declare_parameter(
            "enable_debug_topics", controller_cfg["enable_debug_topics"]
        )
        self.declare_parameter(
            "cmd_vel_timeout_sec", controller_cfg["cmd_vel_timeout_sec"]
        )
        self.declare_parameter(
            "cmd_vel_linear_deadband", controller_cfg["cmd_vel_linear_deadband"]
        )
        self.declare_parameter(
            "cmd_vel_angular_deadband", controller_cfg["cmd_vel_angular_deadband"]
        )
        self.declare_parameter("cmd_filter_tau_sec", controller_cfg["cmd_filter_tau_sec"])
        self.declare_parameter("cmd_accel_limit_x", controller_cfg["cmd_accel_limit_x"])
        self.declare_parameter("cmd_accel_limit_y", controller_cfg["cmd_accel_limit_y"])
        self.declare_parameter(
            "cmd_accel_limit_yaw", controller_cfg["cmd_accel_limit_yaw"]
        )
        self.declare_parameter("seed_timeout_sec", controller_cfg["seed_timeout_sec"])
        self.declare_parameter(
            "reject_seed_if_near_zero", controller_cfg["reject_seed_if_near_zero"]
        )
        self.declare_parameter(
            "zero_seed_threshold_rad", controller_cfg["zero_seed_threshold_rad"]
        )
        self.declare_parameter(
            "locomotion_startup_mode", controller_cfg["locomotion_startup_mode"]
        )
        self.declare_parameter(
            "startup_joint_tolerance_rad",
            controller_cfg["startup_joint_tolerance_rad"],
        )
        self.declare_parameter(
            "recovery_complete_joint_tolerance_rad",
            controller_cfg.get(
                "recovery_complete_joint_tolerance_rad",
                controller_cfg["startup_joint_tolerance_rad"],
            ),
        )
        self.declare_parameter(
            "stand_recovery_timeout_sec",
            controller_cfg["stand_recovery_timeout_sec"],
        )
        self.declare_parameter(
            "tripod_max_joint_delta_rad",
            controller_cfg["tripod_max_joint_delta_rad"],
        )
        self.declare_parameter(
            "tripod_ik_reject_error_norm",
            controller_cfg["tripod_ik_reject_error_norm"],
        )
        self.declare_parameter(
            "seed_from_joint_states", controller_cfg["seed_from_joint_states"]
        )
        self.declare_parameter(
            "reset_to_stand_on_shutdown",
            controller_cfg["reset_to_stand_on_shutdown"],
        )
        self.declare_parameter("imu_timeout_sec", imu_cfg["imu_timeout_sec"])
        self.declare_parameter(
            "imu_orientation_alpha", imu_cfg["imu_orientation_alpha"]
        )
        self.declare_parameter(
            "imu_angular_velocity_alpha", imu_cfg["imu_angular_velocity_alpha"]
        )
        self.declare_parameter("imu_roll_offset_rad", imu_cfg["imu_roll_offset_rad"])
        self.declare_parameter("imu_pitch_offset_rad", imu_cfg["imu_pitch_offset_rad"])
        self.declare_parameter(
            "imu_gravity_min_mps2", imu_cfg["imu_gravity_min_mps2"]
        )
        self.declare_parameter(
            "imu_gravity_max_mps2", imu_cfg["imu_gravity_max_mps2"]
        )
        self.declare_parameter("imu_fail_open", imu_cfg["imu_fail_open"])
        self.declare_parameter(
            "enable_imu_stabilizer", stabilizer_cfg["enable_imu_stabilizer"]
        )
        self.declare_parameter(
            "stabilizer_deadband_rad", stabilizer_cfg["stabilizer_deadband_rad"]
        )
        self.declare_parameter(
            "stabilizer_activation_rad",
            stabilizer_cfg["stabilizer_activation_rad"],
        )
        self.declare_parameter(
            "stabilizer_roll_kp", stabilizer_cfg["stabilizer_roll_kp"]
        )
        self.declare_parameter(
            "stabilizer_roll_kd", stabilizer_cfg["stabilizer_roll_kd"]
        )
        self.declare_parameter(
            "stabilizer_pitch_kp", stabilizer_cfg["stabilizer_pitch_kp"]
        )
        self.declare_parameter(
            "stabilizer_pitch_kd", stabilizer_cfg["stabilizer_pitch_kd"]
        )
        self.declare_parameter(
            "stabilizer_stand_scale", stabilizer_cfg["stabilizer_stand_scale"]
        )
        self.declare_parameter(
            "stabilizer_walk_scale", stabilizer_cfg["stabilizer_walk_scale"]
        )
        self.declare_parameter(
            "stabilizer_max_roll_correction_rad",
            stabilizer_cfg["stabilizer_max_roll_correction_rad"],
        )
        self.declare_parameter(
            "stabilizer_max_pitch_correction_rad",
            stabilizer_cfg["stabilizer_max_pitch_correction_rad"],
        )
        self.declare_parameter(
            "stabilizer_max_correction_rate_rad_s",
            stabilizer_cfg["stabilizer_max_correction_rate_rad_s"],
        )
        self.declare_parameter("leg_lift_height", gait_cfg["leg_lift_height"])
        self.declare_parameter(
            "tripod_base_cadence_hz", gait_cfg["tripod_base_cadence_hz"]
        )
        self.declare_parameter(
            "tripod_max_cadence_hz", gait_cfg["tripod_max_cadence_hz"]
        )
        self.declare_parameter(
            "tripod_max_linear_x", gait_cfg["tripod_max_linear_x"]
        )
        self.declare_parameter(
            "tripod_max_linear_y", gait_cfg["tripod_max_linear_y"]
        )
        self.declare_parameter(
            "tripod_max_angular_z", gait_cfg["tripod_max_angular_z"]
        )
        self.declare_parameter(
            "tripod_forward_stride_m", gait_cfg["tripod_forward_stride_m"]
        )
        self.declare_parameter(
            "tripod_lateral_stride_m", gait_cfg["tripod_lateral_stride_m"]
        )
        self.declare_parameter(
            "tripod_turn_stride_m", gait_cfg["tripod_turn_stride_m"]
        )
        self.declare_parameter("tripod_turn_gain", gait_cfg["tripod_turn_gain"])
        self.declare_parameter(
            "tripod_turn_input_exponent",
            gait_cfg.get("tripod_turn_input_exponent", 1.0),
        )
        self.declare_parameter(
            "tripod_turn_cadence_boost_hz",
            gait_cfg["tripod_turn_cadence_boost_hz"],
        )
        self.declare_parameter(
            "tripod_lateral_gain", gait_cfg["tripod_lateral_gain"]
        )
        self.declare_parameter(
            "tripod_stop_cadence_hz", gait_cfg["tripod_stop_cadence_hz"]
        )

        self.mode = str(self.get_parameter("mode").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.joint_command_topic = str(
            self.get_parameter("joint_command_topic").value
        )
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.enable_debug_topics = bool(
            self.get_parameter("enable_debug_topics").value
        )
        self.cmd_vel_timeout_sec = float(
            self.get_parameter("cmd_vel_timeout_sec").value
        )
        self.cmd_vel_linear_deadband = float(
            self.get_parameter("cmd_vel_linear_deadband").value
        )
        self.cmd_vel_angular_deadband = float(
            self.get_parameter("cmd_vel_angular_deadband").value
        )
        self.seed_timeout_sec = float(self.get_parameter("seed_timeout_sec").value)
        self.reject_seed_if_near_zero = bool(
            self.get_parameter("reject_seed_if_near_zero").value
        )
        self.zero_seed_threshold_rad = float(
            self.get_parameter("zero_seed_threshold_rad").value
        )
        self.locomotion_startup_mode = str(
            self.get_parameter("locomotion_startup_mode").value
        ).lower()
        self.startup_joint_tolerance_rad = float(
            self.get_parameter("startup_joint_tolerance_rad").value
        )
        self.recovery_complete_joint_tolerance_rad = float(
            self.get_parameter("recovery_complete_joint_tolerance_rad").value
        )
        self.stand_recovery_timeout_sec = float(
            self.get_parameter("stand_recovery_timeout_sec").value
        )
        self.tripod_max_joint_delta_rad = float(
            self.get_parameter("tripod_max_joint_delta_rad").value
        )
        self.tripod_ik_reject_error_norm = float(
            self.get_parameter("tripod_ik_reject_error_norm").value
        )
        self.seed_from_joint_states = bool(
            self.get_parameter("seed_from_joint_states").value
        )
        self.reset_to_stand_on_shutdown = bool(
            self.get_parameter("reset_to_stand_on_shutdown").value
        )
        self.imu_fail_open = bool(self.get_parameter("imu_fail_open").value)
        self.stabilizer_activation_rad = float(
            self.get_parameter("stabilizer_activation_rad").value
        )
        self.stabilizer_stand_scale = clamp(
            float(self.get_parameter("stabilizer_stand_scale").value), 0.0, 1.0
        )
        self.stabilizer_walk_scale = clamp(
            float(self.get_parameter("stabilizer_walk_scale").value), 0.0, 1.0
        )
        self.leg_lift_height = float(self.get_parameter("leg_lift_height").value)
        self.tripod_base_cadence_hz = float(
            self.get_parameter("tripod_base_cadence_hz").value
        )
        self.tripod_max_cadence_hz = float(
            self.get_parameter("tripod_max_cadence_hz").value
        )
        self.tripod_max_linear_x = float(
            self.get_parameter("tripod_max_linear_x").value
        )
        self.tripod_max_linear_y = float(
            self.get_parameter("tripod_max_linear_y").value
        )
        self.tripod_max_angular_z = float(
            self.get_parameter("tripod_max_angular_z").value
        )
        self.tripod_forward_stride_m = float(
            self.get_parameter("tripod_forward_stride_m").value
        )
        self.tripod_lateral_stride_m = float(
            self.get_parameter("tripod_lateral_stride_m").value
        )
        self.tripod_turn_stride_m = float(
            self.get_parameter("tripod_turn_stride_m").value
        )
        self.tripod_turn_gain = float(
            self.get_parameter("tripod_turn_gain").value
        )
        self.tripod_turn_input_exponent = max(
            0.1, float(self.get_parameter("tripod_turn_input_exponent").value)
        )
        self.tripod_turn_cadence_boost_hz = float(
            self.get_parameter("tripod_turn_cadence_boost_hz").value
        )
        self.tripod_lateral_gain = float(
            self.get_parameter("tripod_lateral_gain").value
        )
        self.tripod_stop_cadence_hz = float(
            self.get_parameter("tripod_stop_cadence_hz").value
        )

        self.tripod_scheduler = TripodScheduler(
            self.geometry.gait_config["tripod_groups"],
            self.geometry.gait_config["cycle_length"],
        )
        self.foot_trajectory = FootTrajectoryGenerator(self.leg_lift_height)
        self.command_filter = CommandFilter(
            tau_sec=float(self.get_parameter("cmd_filter_tau_sec").value),
            accel_limit_x=float(self.get_parameter("cmd_accel_limit_x").value),
            accel_limit_y=float(self.get_parameter("cmd_accel_limit_y").value),
            accel_limit_yaw=float(self.get_parameter("cmd_accel_limit_yaw").value),
        )
        self.imu_state_estimator = ImuStateEstimator(
            imu_timeout_sec=float(self.get_parameter("imu_timeout_sec").value),
            orientation_alpha=float(
                self.get_parameter("imu_orientation_alpha").value
            ),
            angular_velocity_alpha=float(
                self.get_parameter("imu_angular_velocity_alpha").value
            ),
            roll_offset_rad=float(self.get_parameter("imu_roll_offset_rad").value),
            pitch_offset_rad=float(self.get_parameter("imu_pitch_offset_rad").value),
            gravity_min_mps2=float(
                self.get_parameter("imu_gravity_min_mps2").value
            ),
            gravity_max_mps2=float(
                self.get_parameter("imu_gravity_max_mps2").value
            ),
            fail_open=self.imu_fail_open,
        )
        self.pose_stabilizer = PoseStabilizer(
            enabled=bool(self.get_parameter("enable_imu_stabilizer").value),
            deadband_rad=float(
                self.get_parameter("stabilizer_deadband_rad").value
            ),
            roll_kp=float(self.get_parameter("stabilizer_roll_kp").value),
            roll_kd=float(self.get_parameter("stabilizer_roll_kd").value),
            pitch_kp=float(self.get_parameter("stabilizer_pitch_kp").value),
            pitch_kd=float(self.get_parameter("stabilizer_pitch_kd").value),
            max_roll_correction_rad=float(
                self.get_parameter("stabilizer_max_roll_correction_rad").value
            ),
            max_pitch_correction_rad=float(
                self.get_parameter("stabilizer_max_pitch_correction_rad").value
            ),
            max_correction_rate_rad_s=float(
                self.get_parameter("stabilizer_max_correction_rate_rad_s").value
            ),
        )

        self.joint_command_pub = self.create_publisher(
            JointState, self.joint_command_topic, 10
        )
        self.create_subscription(
            JointState, self.joint_state_topic, self._joint_state_callback, 20
        )
        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 20)
        self.create_subscription(Imu, self.imu_topic, self._imu_callback, 20)

        self.debug_filtered_cmd_pub = None
        self.debug_body_tilt_pub = None
        self.debug_stabilizer_pub = None
        self.debug_state_pub = None
        if self.enable_debug_topics:
            self.debug_filtered_cmd_pub = self.create_publisher(
                TwistStamped,
                "/hexbot_locomotion_v2/debug/filtered_cmd_vel",
                10,
            )
            self.debug_body_tilt_pub = self.create_publisher(
                Vector3Stamped, "/hexbot_locomotion_v2/debug/body_tilt", 10
            )
            self.debug_stabilizer_pub = self.create_publisher(
                Vector3Stamped,
                "/hexbot_locomotion_v2/debug/stabilizer_correction",
                10,
            )
            self.debug_state_pub = self.create_publisher(
                String, "/hexbot_locomotion_v2/debug/state", 10
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tf_contract_validated = False
        self.last_tf_check_sec = -1.0
        self.last_published_state: str | None = None
        self.implicit_stand_foot_targets = self.geometry.neutral_foot_positions()
        self.implicit_stand_leg_angles = {
            leg_name: list(self.geometry.neutral_joint_positions[leg_name])
            for leg_name in self.geometry.leg_order
        }

        (
            self.stance_source,
            self.stand_foot_targets,
            self.stance_metadata,
        ) = self._resolve_stand_foot_targets(stance_cfg)
        if self.stance_source == "implicit_neutral_fk":
            self.configured_stand_leg_angles = {
                leg_name: list(self.geometry.neutral_joint_positions[leg_name])
                for leg_name in self.geometry.leg_order
            }
        else:
            self.configured_stand_leg_angles = self._solve_configured_stand_leg_angles(
                self.stand_foot_targets
            )
        self.tripod_home_targets = copy_vector_map(self.stand_foot_targets)
        self._refresh_tripod_gait_home_targets()
        self.current_leg_angles = copy_vector_map(self.configured_stand_leg_angles)
        self.live_leg_angles = copy_vector_map(self.configured_stand_leg_angles)
        self.configured_stand_joint_positions = (
            self.geometry.joint_positions_from_leg_map(
                self.configured_stand_leg_angles
            )
        )

        self.received_joint_state = False
        self.invalid_zero_seed_logged = False
        self.seed_applied = not self.seed_from_joint_states
        initial_state = READY if self.seed_applied else WAIT_FOR_VALID_SEED
        self.state_machine = LocomotionStateMachine(initial_state=initial_state)
        self.latest_cmd_vel = Twist()
        self.latest_cmd_vel_time = None
        self.last_active_tripod_cmd = (0.0, 0.0, 0.0)
        self.last_filtered_tripod_cmd = (0.0, 0.0, 0.0)
        self.last_stabilizer_correction = self.pose_stabilizer.update(
            0.0, 0.0, 0.0, 0.0, False, 0.0
        )
        self.start_time = self.get_clock().now()
        self.last_update_time = self.start_time
        self.seed_wall_start_sec = None
        self.recovery_start_time = None
        self.recovery_wall_start_sec = None
        self.last_joint_command_publisher_warning_sec = -1.0
        self.last_ready_seed_source = "not_ready"
        self.last_gait_command_active = False
        self.lock_current_to_configured_until_walk = False
        self.gait_activation_count = 0

        if self._is_tripod_mode():
            if self.seed_from_joint_states:
                self.state_machine.set_state(WAIT_FOR_VALID_SEED)
            else:
                self._set_tripod_ready(
                    "fallback_stand", self.configured_stand_leg_angles
                )
        else:
            self.state_machine.set_state(READY)
            self.seed_applied = True

        self.timer = self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self._update)

        nominal_summary = {
            leg_name: [round(value, 4) for value in self.stand_foot_targets[leg_name]]
            for leg_name in self.geometry.leg_order
        }
        self.get_logger().info(
            f"Loaded geometry for {self.geometry.robot_name}. "
            f"Mode={self.mode}, stand_source={self.stance_source}, stand_feet={nominal_summary}"
        )
        if self.stance_metadata:
            self.get_logger().info(
                f"Explicit stance parameters: {self.stance_metadata}"
            )
        correction_summary = {
            leg_name: {
                "turn_bias": round(self.tripod_turn_leg_biases[leg_name], 3),
                "home_offset": [
                    round(value, 4) for value in self.tripod_leg_home_offsets[leg_name]
                ],
            }
            for leg_name in self.geometry.leg_order
            if abs(self.tripod_turn_leg_biases[leg_name] - 1.0) > 1.0e-6
            or any(
                abs(value) > 1.0e-6
                for value in self.tripod_leg_home_offsets[leg_name]
            )
        }
        if correction_summary:
            self.get_logger().info(
                f"Tripod per-leg corrections active: {correction_summary}"
            )
        if self.stance_source != "implicit_neutral_fk" or correction_summary:
            gait_home_summary = {
                leg_name: [
                    round(value, 4) for value in self.tripod_gait_home_targets[leg_name]
                ]
                for leg_name in self.geometry.leg_order
            }
            self.get_logger().info(
                f"Resolved tripod gait home targets: {gait_home_summary}"
            )
        self._log_stand_diagnostics()
        self.get_logger().info(
            "Phase-1 v2 interface ready: "
            f"use_sim_time={bool(self.get_parameter('use_sim_time').value)}, "
            f"joint_state_topic={self.joint_state_topic}, "
            f"joint_command_topic={self.joint_command_topic}, "
            f"cmd_vel_topic={self.cmd_vel_topic}, "
            f"imu_topic={self.imu_topic}, "
            f"frames={self.odom_frame}->{self.base_frame}->{self.imu_frame}"
        )

    def _resolve_config_path(self, configured_path: str, fallback_relative_path: str) -> Path:
        if configured_path:
            return Path(configured_path).expanduser().resolve()

        package_share = Path(get_package_share_directory("hexbot_locomotion_v2"))
        return (package_share / fallback_relative_path).resolve()

    def _is_tripod_mode(self) -> bool:
        return self.mode in TRIPOD_MODES

    def _resolve_stand_foot_targets(
        self, stance_cfg: dict[str, object]
    ) -> tuple[str, dict[str, list[float]], dict[str, float]]:
        implicit_targets = self.geometry.neutral_foot_positions()
        if not stance_cfg:
            return "implicit_neutral_fk", implicit_targets, {}

        targets = copy_vector_map(implicit_targets)
        metadata: dict[str, float] = {}

        standing_body_height = stance_cfg.get("standing_body_height_m")
        if standing_body_height is None:
            standing_body_height = stance_cfg.get("standing_body_height")
        if standing_body_height is None:
            standing_body_height = stance_cfg.get("body_clearance_m")
        if standing_body_height is None:
            standing_body_height = stance_cfg.get("body_clearance")
        stance_foot_z_offset = stance_cfg.get("stance_foot_z_offset_m")
        if stance_foot_z_offset is None:
            stance_foot_z_offset = stance_cfg.get("stance_foot_z_offset")
        if stance_foot_z_offset is None:
            stance_foot_z_offset = stance_cfg.get("foot_z_offset_m")
        per_leg_positions = stance_cfg.get("per_leg_stance_positions")
        if per_leg_positions is None:
            per_leg_positions = stance_cfg.get("stance_positions")

        if standing_body_height is not None:
            stand_z = -float(standing_body_height)
            metadata["standing_body_height_m"] = float(standing_body_height)
            if stance_foot_z_offset is not None:
                stand_z -= float(stance_foot_z_offset)
                metadata["stance_foot_z_offset_m"] = float(stance_foot_z_offset)
            metadata["resolved_stand_z_m"] = stand_z
            for leg_name in self.geometry.leg_order:
                targets[leg_name][2] = stand_z
        elif stance_foot_z_offset is not None:
            raise ValueError(
                "stance.stance_foot_z_offset_m requires stance.standing_body_height_m."
            )

        if per_leg_positions is None:
            if standing_body_height is None:
                return "implicit_neutral_fk", implicit_targets, {}
            return "explicit_stance", targets, metadata
        if not isinstance(per_leg_positions, dict):
            raise ValueError("stance.per_leg_stance_positions must be a leg->position map.")

        missing_legs = [
            leg_name
            for leg_name in self.geometry.leg_order
            if leg_name not in per_leg_positions
        ]
        extra_legs = sorted(
            leg_name
            for leg_name in per_leg_positions.keys()
            if leg_name not in self.geometry.leg_order
        )
        if missing_legs or extra_legs:
            raise ValueError(
                "stance.per_leg_stance_positions must define exactly the configured leg set. "
                f"missing={missing_legs}, extra={extra_legs}"
            )

        for leg_name in self.geometry.leg_order:
            x_pos, y_pos, z_override = self._parse_stance_position_entry(
                leg_name, per_leg_positions[leg_name]
            )
            targets[leg_name][0] = x_pos
            targets[leg_name][1] = y_pos
            if z_override is not None:
                targets[leg_name][2] = z_override

        return "explicit_stance", targets, metadata

    def _parse_stance_position_entry(
        self, leg_name: str, raw_value: object
    ) -> tuple[float, float, float | None]:
        if isinstance(raw_value, dict):
            if "x" not in raw_value or "y" not in raw_value:
                raise ValueError(
                    "Each stance.per_leg_stance_positions entry must provide x and y."
                )
            z_value = raw_value.get("z")
            return (
                float(raw_value["x"]),
                float(raw_value["y"]),
                None if z_value is None else float(z_value),
            )

        if not isinstance(raw_value, (list, tuple)) or len(raw_value) not in (2, 3):
            raise ValueError(
                "Each stance.per_leg_stance_positions entry must contain [x, y] "
                "or [x, y, z]."
            )

        z_override = None if len(raw_value) == 2 else float(raw_value[2])
        return float(raw_value[0]), float(raw_value[1]), z_override

    def _solve_configured_stand_leg_angles(
        self, foot_targets: dict[str, list[float]]
    ) -> dict[str, list[float]]:
        solved_leg_angles: dict[str, list[float]] = {}
        max_error_norm = max(self.ik_solver.tolerance * 10.0, 1.0e-3)
        for leg_name in self.geometry.leg_order:
            solution = self.ik_solver.solve(
                self.geometry,
                leg_name,
                foot_targets[leg_name],
                initial_guess=self.geometry.neutral_joint_positions[leg_name],
            )
            if not solution.success and solution.error_norm > max_error_norm:
                raise ValueError(
                    "Configured stand target is unreachable for "
                    f"{leg_name}: residual={solution.error_norm:.6f}, "
                    f"target={foot_targets[leg_name]}"
                )
            solved_leg_angles[leg_name] = [
                float(value) for value in solution.joint_angles
            ]
        return solved_leg_angles

    def _load_tripod_turn_leg_biases(
        self, gait_cfg: dict[str, object]
    ) -> dict[str, float]:
        biases = default_tripod_turn_leg_biases(self.geometry.leg_order)
        raw_biases = gait_cfg.get("tripod_turn_leg_biases", {})
        if raw_biases is None:
            return biases
        if not isinstance(raw_biases, dict):
            raise ValueError("gait.tripod_turn_leg_biases must be a leg->float map.")

        for leg_name, raw_value in raw_biases.items():
            if leg_name not in biases:
                raise ValueError(
                    f"Unknown leg '{leg_name}' in gait.tripod_turn_leg_biases."
                )
            biases[leg_name] = float(raw_value)
        return biases

    def _load_tripod_leg_home_offsets(
        self, gait_cfg: dict[str, object]
    ) -> dict[str, list[float]]:
        offsets = {
            leg_name: [0.0, 0.0, 0.0] for leg_name in self.geometry.leg_order
        }
        raw_offsets = gait_cfg.get("tripod_leg_home_offsets", {})
        if raw_offsets is None:
            return offsets
        if not isinstance(raw_offsets, dict):
            raise ValueError("gait.tripod_leg_home_offsets must be a leg->[x,y,z] map.")

        for leg_name, raw_value in raw_offsets.items():
            if leg_name not in offsets:
                raise ValueError(
                    f"Unknown leg '{leg_name}' in gait.tripod_leg_home_offsets."
                )
            if not isinstance(raw_value, (list, tuple)) or len(raw_value) != 3:
                raise ValueError(
                    "Each gait.tripod_leg_home_offsets entry must contain 3 values."
                )
            offsets[leg_name] = [float(raw_value[index]) for index in range(3)]
        return offsets

    def _refresh_tripod_gait_home_targets(self) -> None:
        self.tripod_gait_home_targets = add_vector_map_offsets(
            self.tripod_home_targets, self.tripod_leg_home_offsets
        )

    def _joint_map_max_abs_difference(
        self, left: dict[str, list[float]], right: dict[str, list[float]]
    ) -> float:
        max_abs_difference = 0.0
        for leg_name in self.geometry.leg_order:
            for joint_index in range(3):
                max_abs_difference = max(
                    max_abs_difference,
                    abs(left[leg_name][joint_index] - right[leg_name][joint_index]),
                )
        return max_abs_difference

    def _vector_map_max_abs_difference(
        self, left: dict[str, list[float]], right: dict[str, list[float]]
    ) -> float:
        max_abs_difference = 0.0
        for leg_name in self.geometry.leg_order:
            for axis in range(3):
                max_abs_difference = max(
                    max_abs_difference,
                    abs(left[leg_name][axis] - right[leg_name][axis]),
                )
        return max_abs_difference

    def _joint_map_leg_delta_summary(
        self,
        left: dict[str, list[float]],
        right: dict[str, list[float]],
        threshold: float = 1.0e-6,
    ) -> dict[str, float]:
        summary: dict[str, float] = {}
        for leg_name in self.geometry.leg_order:
            leg_delta = max(
                abs(left[leg_name][joint_index] - right[leg_name][joint_index])
                for joint_index in range(3)
            )
            if leg_delta > threshold:
                summary[leg_name] = round(leg_delta, 6)
        return summary

    def _vector_map_leg_delta_summary(
        self,
        left: dict[str, list[float]],
        right: dict[str, list[float]],
        threshold: float = 1.0e-6,
    ) -> dict[str, float]:
        summary: dict[str, float] = {}
        for leg_name in self.geometry.leg_order:
            leg_delta = max(
                abs(left[leg_name][axis] - right[leg_name][axis]) for axis in range(3)
            )
            if leg_delta > threshold:
                summary[leg_name] = round(leg_delta, 6)
        return summary

    def _log_stand_diagnostics(self) -> None:
        implicit_vs_configured_joint_max = self._joint_map_max_abs_difference(
            self.configured_stand_leg_angles, self.implicit_stand_leg_angles
        )
        implicit_vs_configured_foot_max = self._vector_map_max_abs_difference(
            self.stand_foot_targets, self.implicit_stand_foot_targets
        )
        self.get_logger().info(
            "Stand diagnostics: "
            f"configured_vs_implicit_joint_max_rad={implicit_vs_configured_joint_max:.6f}, "
            f"configured_vs_implicit_foot_max_m={implicit_vs_configured_foot_max:.6f}, "
            f"joint_delta_by_leg={self._joint_map_leg_delta_summary(self.configured_stand_leg_angles, self.implicit_stand_leg_angles)}, "
            f"foot_delta_by_leg={self._vector_map_leg_delta_summary(self.stand_foot_targets, self.implicit_stand_foot_targets)}"
        )

    def _log_gait_entry_diagnostics(
        self, planar_cmd: tuple[float, float, float]
    ) -> None:
        self.gait_activation_count += 1
        self.get_logger().info(
            "Tripod gait entry diagnostics: "
            f"entry_index={self.gait_activation_count}, "
            f"seed_source={self.last_ready_seed_source}, "
            f"phase={self.tripod_scheduler.phase:.4f}, "
            f"cmd=({planar_cmd[0]:.3f}, {planar_cmd[1]:.3f}, {planar_cmd[2]:.3f}), "
            f"current_vs_configured_max_rad={self._joint_map_max_abs_difference(self.current_leg_angles, self.configured_stand_leg_angles):.6f}, "
            f"live_vs_configured_max_rad={self._joint_map_max_abs_difference(self.live_leg_angles, self.configured_stand_leg_angles):.6f}, "
            f"current_vs_live_max_rad={self._joint_map_max_abs_difference(self.current_leg_angles, self.live_leg_angles):.6f}, "
            f"configured_vs_implicit_max_rad={self._joint_map_max_abs_difference(self.configured_stand_leg_angles, self.implicit_stand_leg_angles):.6f}"
        )

    def _seed_looks_near_zero(self, leg_angle_map: dict[str, list[float]]) -> bool:
        max_abs_angle = 0.0
        for leg_name in self.geometry.leg_order:
            for joint_angle in leg_angle_map[leg_name]:
                max_abs_angle = max(max_abs_angle, abs(float(joint_angle)))
        return max_abs_angle < self.zero_seed_threshold_rad

    def _publish_joint_positions_once(self, positions: list[float], stamp=None) -> None:
        joint_state = JointState()
        joint_state.name = list(self.geometry.joint_names)
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        joint_state.header.stamp = stamp
        joint_state.position = [float(value) for value in positions]
        self.joint_command_pub.publish(joint_state)

    def _publish_leg_joint_map(self, leg_joint_map: dict[str, list[float]], stamp) -> None:
        self._publish_joint_positions_once(
            self.geometry.joint_positions_from_leg_map(leg_joint_map), stamp
        )

    def _cmd_vel_callback(self, message: Twist) -> None:
        self.latest_cmd_vel = message
        self.latest_cmd_vel_time = self.get_clock().now()

    def _imu_callback(self, message: Imu) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        self.imu_state_estimator.update_from_message(message, now_sec)

    def _joint_state_callback(self, message: JointState) -> None:
        if not message.position:
            return

        updated_leg_angles = copy_vector_map(self.live_leg_angles)
        updated_count = 0
        for index, joint_name in enumerate(message.name):
            if index >= len(message.position):
                break
            if joint_name not in self.geometry.joint_to_leg_index:
                continue
            leg_name, joint_index = self.geometry.joint_to_leg_index[joint_name]
            updated_leg_angles[leg_name][joint_index] = float(message.position[index])
            updated_count += 1

        if updated_count == 0:
            return

        if (
            not self.seed_applied
            and self.reject_seed_if_near_zero
            and self._seed_looks_near_zero(updated_leg_angles)
        ):
            if not self.invalid_zero_seed_logged:
                self.invalid_zero_seed_logged = True
                self.get_logger().warn(
                    "Ignoring near-zero joint-state seed and waiting for a valid standing pose."
                )
            return

        self.live_leg_angles = updated_leg_angles
        self.received_joint_state = True
        if self.seed_wall_start_sec is None:
            self.seed_wall_start_sec = time.monotonic()

        # During active gait execution, use the last commanded joint map as the
        # IK initial guess. Feeding back the live articulation state here can
        # make a single lagging leg jump to an alternative IK branch and then
        # ratchet away over multiple steps. We still keep the live angles for
        # startup/recovery decisions and resynchronize whenever the gait is not
        # actively stepping.
        should_sync_current_to_live = (
            not self.seed_applied
            or not self._is_tripod_mode()
            or self.state_machine.state not in {WALKING, STOPPING_TO_STAND}
        )
        if (
            should_sync_current_to_live
            and self.lock_current_to_configured_until_walk
            and self._is_tripod_mode()
            and self.state_machine.state == READY
        ):
            should_sync_current_to_live = False
        if should_sync_current_to_live:
            self.current_leg_angles = copy_vector_map(updated_leg_angles)

        if self._is_tripod_mode():
            self._handle_tripod_seed_candidate(updated_leg_angles)
        elif not self.seed_applied:
            self.seed_applied = True
            self.state_machine.set_state(READY)

    def _set_tripod_ready(
        self, seed_source: str, leg_angles: dict[str, list[float]]
    ) -> None:
        self.current_leg_angles = copy_vector_map(leg_angles)
        self.seed_applied = True
        self.tripod_scheduler.phase = 0.0
        self.last_active_tripod_cmd = (0.0, 0.0, 0.0)
        self.last_filtered_tripod_cmd = (0.0, 0.0, 0.0)
        self.recovery_start_time = None
        self.recovery_wall_start_sec = None
        self.command_filter.reset()
        self.pose_stabilizer.reset()
        self.last_ready_seed_source = seed_source
        self.last_gait_command_active = False
        self.lock_current_to_configured_until_walk = seed_source in {
            "fixed_stand",
            "fallback_stand",
        }
        self.state_machine.set_ready(seed_source)
        self.get_logger().info(
            "Tripod gait startup ready: "
            f"seed_source={seed_source}, gait_home=configured_stand, state={self.state_machine.state}, "
            f"seed_vs_configured_max_rad={self._joint_map_max_abs_difference(leg_angles, self.configured_stand_leg_angles):.6f}, "
            f"seed_vs_implicit_max_rad={self._joint_map_max_abs_difference(leg_angles, self.implicit_stand_leg_angles):.6f}, "
            f"configured_vs_implicit_max_rad={self._joint_map_max_abs_difference(self.configured_stand_leg_angles, self.implicit_stand_leg_angles):.6f}"
        )

    def _begin_recovery_to_stand(self, max_joint_diff: float) -> None:
        if self.state_machine.state == RECOVER_TO_STAND:
            return
        self.state_machine.begin_recovery()
        self.recovery_start_time = self.get_clock().now()
        self.recovery_wall_start_sec = time.monotonic()
        self.get_logger().warn(
            "Tripod gait startup rejected the live pose "
            f"(max_joint_diff_to_stand={max_joint_diff:.4f} rad). "
            "Recovering to the configured stand pose before walking. "
            f"startup_tolerance_rad={self.startup_joint_tolerance_rad:.4f}, "
            f"recovery_complete_tolerance_rad={self.recovery_complete_joint_tolerance_rad:.4f}"
        )

    def _handle_tripod_seed_candidate(self, leg_angles: dict[str, list[float]]) -> None:
        if self.seed_applied:
            return

        max_joint_diff = self._joint_map_max_abs_difference(
            leg_angles, self.configured_stand_leg_angles
        )
        if self.state_machine.state == RECOVER_TO_STAND:
            if max_joint_diff <= self.startup_joint_tolerance_rad:
                if self.locomotion_startup_mode == "fixed_stand":
                    self._set_tripod_ready(
                        "fixed_stand", self.configured_stand_leg_angles
                    )
                else:
                    self._set_tripod_ready("recovered_stand", leg_angles)
            return

        if self.locomotion_startup_mode == "live":
            self._set_tripod_ready("live", leg_angles)
            return

        if self.locomotion_startup_mode == "fixed_stand":
            self._begin_recovery_to_stand(max_joint_diff)
            return

        if max_joint_diff <= self.startup_joint_tolerance_rad:
            self._set_tripod_ready("live", leg_angles)
            return

        self._begin_recovery_to_stand(max_joint_diff)

    def _handle_tripod_startup(self, now) -> str:
        if self.seed_applied:
            return READY

        if self.state_machine.state == RECOVER_TO_STAND:
            if self.received_joint_state:
                max_joint_diff = self._joint_map_max_abs_difference(
                    self.live_leg_angles, self.configured_stand_leg_angles
                )
                if max_joint_diff <= self.recovery_complete_joint_tolerance_rad:
                    if self.locomotion_startup_mode == "fixed_stand":
                        self._set_tripod_ready(
                            "fixed_stand", self.configured_stand_leg_angles
                        )
                    else:
                        self._set_tripod_ready("recovered_stand", self.live_leg_angles)
                    return READY

            if self.recovery_start_time is None:
                self.recovery_start_time = now
            if self.recovery_wall_start_sec is None:
                self.recovery_wall_start_sec = time.monotonic()
            recovery_elapsed = time.monotonic() - self.recovery_wall_start_sec
            if recovery_elapsed >= self.stand_recovery_timeout_sec:
                self._set_tripod_ready(
                    "fallback_stand", self.configured_stand_leg_angles
                )
                return READY
            return RECOVER_TO_STAND

        if self.seed_wall_start_sec is None:
            return WAIT_FOR_VALID_SEED

        waited_sec = time.monotonic() - self.seed_wall_start_sec
        if waited_sec >= self.seed_timeout_sec:
            self._set_tripod_ready("fallback_stand", self.configured_stand_leg_angles)
            return READY

        return WAIT_FOR_VALID_SEED

    def _current_planar_cmd(self, now) -> tuple[float, float, float]:
        if self.latest_cmd_vel_time is None:
            return (0.0, 0.0, 0.0)

        age_sec = (now - self.latest_cmd_vel_time).nanoseconds * 1.0e-9
        if age_sec > self.cmd_vel_timeout_sec:
            return (0.0, 0.0, 0.0)

        return (
            clamp(
                float(self.latest_cmd_vel.linear.x),
                -self.tripod_max_linear_x,
                self.tripod_max_linear_x,
            ),
            clamp(
                float(self.latest_cmd_vel.linear.y),
                -self.tripod_max_linear_y,
                self.tripod_max_linear_y,
            ),
            clamp(
                float(self.latest_cmd_vel.angular.z),
                -self.tripod_max_angular_z,
                self.tripod_max_angular_z,
            ),
        )

    def _planar_cmd_is_active(self, planar_cmd: tuple[float, float, float]) -> bool:
        return (
            abs(planar_cmd[0]) > self.cmd_vel_linear_deadband
            or abs(planar_cmd[1]) > self.cmd_vel_linear_deadband
            or abs(planar_cmd[2]) > self.cmd_vel_angular_deadband
        )

    def _tripod_normalized_cmd(
        self, planar_cmd: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        angular_ratio = abs(planar_cmd[2]) / max(self.tripod_max_angular_z, 1.0e-6)
        angular_ratio = clamp(angular_ratio, 0.0, 1.0)
        # Exponent < 1.0 boosts low/mid turn commands without increasing the
        # full-scale saturation limit used by the tripod turn envelope.
        shaped_angular_ratio = math.pow(
            angular_ratio, self.tripod_turn_input_exponent
        )
        return (
            clamp(
                planar_cmd[0] / max(self.tripod_max_linear_x, 1.0e-6),
                -1.0,
                1.0,
            ),
            clamp(
                self.tripod_lateral_gain
                * planar_cmd[1]
                / max(self.tripod_max_linear_y, 1.0e-6),
                -1.0,
                1.0,
            ),
            clamp(
                math.copysign(
                    self.tripod_turn_gain * shaped_angular_ratio, planar_cmd[2]
                ),
                -1.0,
                1.0,
            ),
        )

    def _tripod_motion_scale(self, planar_cmd: tuple[float, float, float]) -> float:
        sx, sy, sz = self._tripod_normalized_cmd(planar_cmd)
        return max(abs(sx), abs(sy), abs(sz))

    def _tripod_cadence_hz(self, planar_cmd: tuple[float, float, float]) -> float:
        sx, sy, sz = self._tripod_normalized_cmd(planar_cmd)
        motion_scale = max(abs(sx), abs(sy), abs(sz))
        cadence_floor = max(0.1, self.tripod_base_cadence_hz)
        cadence_ceiling = max(cadence_floor, self.tripod_max_cadence_hz)
        cadence = cadence_floor + (cadence_ceiling - cadence_floor) * motion_scale
        cadence += self.tripod_turn_cadence_boost_hz * abs(sz)
        return max(0.1, cadence)

    def _tripod_leg_turn_bias(self, leg_name: str) -> float:
        return self.tripod_turn_leg_biases.get(leg_name, 1.0)

    def _tripod_step_delta(
        self,
        leg_name: str,
        home: list[float],
        planar_cmd: tuple[float, float, float],
    ) -> list[float]:
        sx, sy, sz = self._tripod_normalized_cmd(planar_cmd)
        if max(abs(sx), abs(sy), abs(sz)) <= 1.0e-6:
            return [0.0, 0.0, 0.0]

        translation_dx = self.tripod_forward_stride_m * sx
        translation_dy = self.tripod_lateral_stride_m * sy

        tangent_x = -home[1]
        tangent_y = home[0]
        tangent_norm = math.sqrt(tangent_x * tangent_x + tangent_y * tangent_y)
        if tangent_norm <= 1.0e-6:
            tangent_x = 1.0
            tangent_y = 0.0
        else:
            tangent_x /= tangent_norm
            tangent_y /= tangent_norm

        rotation_stride = (
            self.tripod_turn_stride_m * sz * self._tripod_leg_turn_bias(leg_name)
        )
        rotation_dx = tangent_x * rotation_stride
        rotation_dy = tangent_y * rotation_stride

        return [translation_dx + rotation_dx, translation_dy + rotation_dy, 0.0]

    def _tripod_targets(
        self, planar_cmd: tuple[float, float, float]
    ) -> dict[str, list[float]]:
        foot_targets: dict[str, list[float]] = {}
        for leg_name in self.geometry.leg_order:
            home = self.tripod_gait_home_targets[leg_name]
            is_swing, phase = self.tripod_scheduler.leg_state(leg_name)
            step_delta = self._tripod_step_delta(leg_name, home, planar_cmd)
            foot_targets[leg_name] = self.foot_trajectory.tripod_step(
                home, step_delta, phase, is_swing
            )
        return foot_targets

    def _stabilize_tripod_joint_solution(
        self,
        leg_name: str,
        previous_angles: list[float],
        solution,
    ) -> list[float]:
        if (
            not solution.success
            and solution.error_norm > self.tripod_ik_reject_error_norm
        ):
            self.get_logger().warn(
                f"Rejected unstable IK solution for {leg_name} in tripod mode; "
                f"residual={solution.error_norm:.6f}. Keeping previous joint angles.",
                throttle_duration_sec=2.0,
            )
            return list(previous_angles)

        max_joint_delta = max(0.0, self.tripod_max_joint_delta_rad)
        if max_joint_delta <= 0.0:
            return [float(value) for value in solution.joint_angles]

        return [
            clamp(
                float(solution.joint_angles[idx]),
                float(previous_angles[idx]) - max_joint_delta,
                float(previous_angles[idx]) + max_joint_delta,
            )
            for idx in range(3)
        ]

    def _tripod_stop_boundary_crossed(
        self, previous_phase: float, current_phase: float
    ) -> bool:
        if previous_phase < 0.5 <= current_phase:
            return True
        if current_phase < previous_phase:
            return True
        return False

    def _stabilizer_control_scale(self, planar_cmd, imu_estimate) -> float:
        if not self.pose_stabilizer.enabled or not imu_estimate.healthy:
            return 0.0

        if self.state_machine.state in {WALKING, STOPPING_TO_STAND}:
            # Phase-1 motion stabilizer is not yet reliable enough to inject
            # foot-target corrections during gait. Keep walking open-loop until
            # a support-aware stabilizer replaces this heuristic.
            return 0.0

        if self._planar_cmd_is_active(planar_cmd):
            return 0.0

        tilt_mag = max(abs(float(imu_estimate.roll)), abs(float(imu_estimate.pitch)))
        if tilt_mag >= self.stabilizer_activation_rad:
            return self.stabilizer_stand_scale

        return 0.0

    def _apply_stabilizer_to_targets(
        self,
        foot_targets: dict[str, list[float]],
        correction,
    ) -> dict[str, list[float]]:
        corrected_targets: dict[str, list[float]] = {}
        for leg_name in self.geometry.leg_order:
            weight = 1.0
            if self.state_machine.state in {WALKING, STOPPING_TO_STAND}:
                is_swing, _ = self.tripod_scheduler.leg_state(leg_name)
                if is_swing:
                    # Keep swing-leg foot placement purely gait-driven so IMU
                    # compensation cannot push a lifting leg into an IK dead zone.
                    weight = 0.0
            corrected_targets[leg_name] = self.pose_stabilizer.apply_to_target(
                foot_targets[leg_name], correction, weight=weight
            )
        return corrected_targets

    def _publish_debug_topics(self, stamp, filtered_cmd, imu_estimate, stabilizer_correction) -> None:
        if not self.enable_debug_topics:
            return

        if self.debug_filtered_cmd_pub is not None:
            message = TwistStamped()
            message.header.stamp = stamp
            message.twist.linear.x = float(filtered_cmd[0])
            message.twist.linear.y = float(filtered_cmd[1])
            message.twist.angular.z = float(filtered_cmd[2])
            self.debug_filtered_cmd_pub.publish(message)

        if self.debug_body_tilt_pub is not None:
            message = Vector3Stamped()
            message.header.stamp = stamp
            message.vector.x = float(imu_estimate.roll)
            message.vector.y = float(imu_estimate.pitch)
            message.vector.z = 1.0 if imu_estimate.healthy else 0.0
            self.debug_body_tilt_pub.publish(message)

        if self.debug_stabilizer_pub is not None:
            message = Vector3Stamped()
            message.header.stamp = stamp
            message.vector.x = float(stabilizer_correction.roll_correction)
            message.vector.y = float(stabilizer_correction.pitch_correction)
            message.vector.z = 1.0 if stabilizer_correction.active else 0.0
            self.debug_stabilizer_pub.publish(message)

        if self.debug_state_pub is not None and self.last_published_state != self.state_machine.state:
            message = String()
            message.data = self.state_machine.state
            self.debug_state_pub.publish(message)
            self.last_published_state = self.state_machine.state

    def _validate_tf_contract(self, now_sec: float) -> None:
        if self.tf_contract_validated:
            return
        if self.last_tf_check_sec >= 0.0 and now_sec - self.last_tf_check_sec < 1.0:
            return
        self.last_tf_check_sec = now_sec

        try:
            odom_ok = self.tf_buffer.can_transform(
                self.odom_frame, self.base_frame, Time()
            )
            imu_ok = self.tf_buffer.can_transform(
                self.base_frame, self.imu_frame, Time()
            )
        except Exception:
            odom_ok = False
            imu_ok = False

        if odom_ok and imu_ok:
            self.tf_contract_validated = True
            if self.seed_wall_start_sec is None:
                self.seed_wall_start_sec = time.monotonic()
            self.get_logger().info(
                "Validated TF contract for phase-1: "
                f"{self.odom_frame}->{self.base_frame} and "
                f"{self.base_frame}->{self.imu_frame}"
            )
            return

        self.get_logger().warn(
            "Waiting for the required TF contract: "
            f"{self.odom_frame}->{self.base_frame}, "
            f"{self.base_frame}->{self.imu_frame}",
            throttle_duration_sec=5.0,
        )

    def _warn_if_joint_command_conflicted(self, now_sec: float) -> None:
        publisher_count = int(self.count_publishers(self.joint_command_topic))
        if publisher_count <= 1:
            return
        if (
            self.last_joint_command_publisher_warning_sec >= 0.0
            and now_sec - self.last_joint_command_publisher_warning_sec < 5.0
        ):
            return
        self.last_joint_command_publisher_warning_sec = now_sec
        self.get_logger().warn(
            "Detected multiple active publishers on "
            f"{self.joint_command_topic}. Stop the other controller before testing."
        )

    def _update(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1.0e-9
        dt = (now - self.last_update_time).nanoseconds * 1.0e-9
        dt = max(dt, 0.0)
        self.last_update_time = now

        self._warn_if_joint_command_conflicted(now_sec)
        self._validate_tf_contract(now_sec)
        imu_estimate = self.imu_state_estimator.current(now_sec)

        if self._is_tripod_mode():
            startup_state = self._handle_tripod_startup(now)
            if startup_state == WAIT_FOR_VALID_SEED:
                self._publish_debug_topics(
                    now.to_msg(),
                    self.last_filtered_tripod_cmd,
                    imu_estimate,
                    self.last_stabilizer_correction,
                )
                return
            if startup_state == RECOVER_TO_STAND:
                self._publish_joint_positions_once(
                    self.configured_stand_joint_positions, now.to_msg()
                )
                self._publish_debug_topics(
                    now.to_msg(),
                    self.last_filtered_tripod_cmd,
                    imu_estimate,
                    self.last_stabilizer_correction,
                )
                return

        if not self.seed_applied:
            return

        raw_planar_cmd = self._current_planar_cmd(now)
        filtered_cmd = self.command_filter.step(raw_planar_cmd, dt).as_tuple()
        self.last_filtered_tripod_cmd = filtered_cmd
        # Phase-1 acceptance compares against the v1 tripod gait. Keep the gait
        # command path identical to v1 and reserve the filtered command for
        # observability only; otherwise the continuously moving stride endpoint
        # during ramp-up can push a swing leg into an unreachable IK target.
        gait_planar_cmd = raw_planar_cmd
        command_active = self._planar_cmd_is_active(gait_planar_cmd)

        if not self.imu_fail_open and not imu_estimate.healthy:
            self.state_machine.enter_fault(imu_estimate.status)

        if self.state_machine.is_fault():
            self._publish_joint_positions_once(
                self.configured_stand_joint_positions, now.to_msg()
            )
            self._publish_debug_topics(
                now.to_msg(), filtered_cmd, imu_estimate, self.last_stabilizer_correction
            )
            return

        if self.mode == "stand":
            self.state_machine.set_state(READY)
            foot_targets = copy_vector_map(self.tripod_home_targets)
        elif self._is_tripod_mode():
            if command_active:
                if self.lock_current_to_configured_until_walk:
                    self.lock_current_to_configured_until_walk = False
                if not self.last_gait_command_active:
                    self._log_gait_entry_diagnostics(gait_planar_cmd)
                self.state_machine.begin_walking()
                self.last_active_tripod_cmd = gait_planar_cmd
                cadence_hz = self._tripod_cadence_hz(gait_planar_cmd)
                self.tripod_scheduler.advance(dt, cadence_hz=cadence_hz)
                foot_targets = self._tripod_targets(gait_planar_cmd)
            elif self.state_machine.state in (WALKING, STOPPING_TO_STAND):
                self.state_machine.begin_stopping()
                cadence_hz = max(0.1, self.tripod_stop_cadence_hz)
                previous_phase = self.tripod_scheduler.phase
                self.tripod_scheduler.advance(dt, cadence_hz=cadence_hz)
                foot_targets = self._tripod_targets(self.last_active_tripod_cmd)
                if self._tripod_stop_boundary_crossed(
                    previous_phase, self.tripod_scheduler.phase
                ):
                    self.state_machine.set_state(READY)
                    self.last_active_tripod_cmd = (0.0, 0.0, 0.0)
                    self.tripod_scheduler.phase = 0.0
                    if self.locomotion_startup_mode == "fixed_stand":
                        self.lock_current_to_configured_until_walk = True
                    foot_targets = copy_vector_map(self.tripod_home_targets)
            else:
                self.state_machine.set_state(READY)
                self.last_active_tripod_cmd = (0.0, 0.0, 0.0)
                self.tripod_scheduler.phase = 0.0
                if self.locomotion_startup_mode == "fixed_stand":
                    self.lock_current_to_configured_until_walk = True
                foot_targets = copy_vector_map(self.tripod_home_targets)
        else:
            self.get_logger().warn(
                f"Unknown mode '{self.mode}', falling back to stand.",
                throttle_duration_sec=2.0,
            )
            self.mode = "stand"
            self.state_machine.set_state(READY)
            foot_targets = copy_vector_map(self.tripod_home_targets)

        self.last_gait_command_active = command_active and self._is_tripod_mode()

        if (
            self._is_tripod_mode()
            and self.state_machine.state == READY
            and self.lock_current_to_configured_until_walk
        ):
            self.current_leg_angles = copy_vector_map(self.configured_stand_leg_angles)
            self._publish_joint_positions_once(
                self.configured_stand_joint_positions, now.to_msg()
            )
            self._publish_debug_topics(
                now.to_msg(),
                filtered_cmd,
                imu_estimate,
                self.last_stabilizer_correction,
            )
            return

        stabilizer_control_scale = self._stabilizer_control_scale(
            gait_planar_cmd, imu_estimate
        )
        self.last_stabilizer_correction = self.pose_stabilizer.update(
            imu_estimate.roll,
            imu_estimate.pitch,
            imu_estimate.roll_rate,
            imu_estimate.pitch_rate,
            imu_estimate.healthy,
            dt,
            control_scale=stabilizer_control_scale,
        )
        corrected_targets = self._apply_stabilizer_to_targets(
            foot_targets, self.last_stabilizer_correction
        )

        leg_joint_map: dict[str, list[float]] = {}
        for leg_name in self.geometry.leg_order:
            previous_angles = list(self.current_leg_angles[leg_name])
            solution = self.ik_solver.solve(
                self.geometry,
                leg_name,
                corrected_targets[leg_name],
                initial_guess=previous_angles,
            )
            joint_angles = self._stabilize_tripod_joint_solution(
                leg_name, previous_angles, solution
            )
            self.current_leg_angles[leg_name] = joint_angles
            leg_joint_map[leg_name] = joint_angles

        self._publish_leg_joint_map(leg_joint_map, now.to_msg())
        self._publish_debug_topics(
            now.to_msg(),
            filtered_cmd,
            imu_estimate,
            self.last_stabilizer_correction,
        )

    def publish_configured_stand_pose(
        self, repeat_count: int = 20, sleep_sec: float = 0.02
    ) -> bool:
        if not rclpy.ok():
            return False

        for _ in range(max(1, repeat_count)):
            if not rclpy.ok():
                return False
            try:
                self._publish_joint_positions_once(self.configured_stand_joint_positions)
            except Exception:
                return False
            time.sleep(max(0.0, sleep_sec))
        return True


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HexbotLocomotionV2Node()
    spin_exception: Exception | None = None
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        spin_exception = exc
    finally:
        if node.reset_to_stand_on_shutdown:
            try:
                if rclpy.ok():
                    node.get_logger().info(
                        "Publishing configured stand pose before shutdown."
                    )
                    published = node.publish_configured_stand_pose()
                    if not published and rclpy.ok():
                        node.get_logger().warn(
                            "Skipped stand-pose publish during shutdown because the ROS context was already closing."
                        )
            except Exception:
                pass
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
        message = str(spin_exception)
        if "Unable to convert call argument to Python object" not in message:
            raise spin_exception


if __name__ == "__main__":
    main()
