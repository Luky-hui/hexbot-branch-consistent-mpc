import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_optional_float(context, name: str):
    raw_value = LaunchConfiguration(name).perform(context).strip()
    return None if raw_value == "" else float(raw_value)


def _resolve_optional_string(context, name: str):
    raw_value = LaunchConfiguration(name).perform(context).strip()
    return None if raw_value == "" else raw_value


def _scaled_value(explicit_value, base_value: float, scale: float) -> float:
    if explicit_value is not None:
        return explicit_value
    return base_value * scale


def _create_locomotion_node(context):
    package_share = get_package_share_directory("hexbot_locomotion_v2")
    joint_config_path = os.path.join(package_share, "config", "joint_name_list.yaml")
    geometry_config_path = LaunchConfiguration("geometry_config_path").perform(context)

    with open(geometry_config_path, encoding="utf-8") as stream:
        raw_config = yaml.safe_load(stream)

    controller_cfg = raw_config["controller"]
    gait_cfg = raw_config["gait"]
    speed_scale = float(LaunchConfiguration("speed_scale").perform(context))

    tripod_max_linear_x = _scaled_value(
        _resolve_optional_float(context, "tripod_max_linear_x"),
        float(gait_cfg["tripod_max_linear_x"]),
        speed_scale,
    )
    tripod_max_linear_y = _scaled_value(
        _resolve_optional_float(context, "tripod_max_linear_y"),
        float(gait_cfg["tripod_max_linear_y"]),
        speed_scale,
    )
    tripod_max_angular_z = _scaled_value(
        _resolve_optional_float(context, "tripod_max_angular_z"),
        float(gait_cfg["tripod_max_angular_z"]),
        speed_scale,
    )
    tripod_forward_stride_m = _scaled_value(
        _resolve_optional_float(context, "tripod_forward_stride_m"),
        float(gait_cfg["tripod_forward_stride_m"]),
        speed_scale,
    )
    tripod_lateral_stride_m = _scaled_value(
        _resolve_optional_float(context, "tripod_lateral_stride_m"),
        float(gait_cfg["tripod_lateral_stride_m"]),
        speed_scale,
    )
    tripod_turn_stride_m = _scaled_value(
        _resolve_optional_float(context, "tripod_turn_stride_m"),
        float(gait_cfg["tripod_turn_stride_m"]),
        speed_scale,
    )
    locomotion_startup_mode = _resolve_optional_string(
        context, "locomotion_startup_mode"
    )

    parameters = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "mode": LaunchConfiguration("mode"),
        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
        "joint_state_topic": LaunchConfiguration("joint_state_topic"),
        "joint_command_topic": LaunchConfiguration("joint_command_topic"),
        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
        "imu_topic": LaunchConfiguration("imu_topic"),
        "odom_frame": LaunchConfiguration("odom_frame"),
        "base_frame": LaunchConfiguration("base_frame"),
        "imu_frame": LaunchConfiguration("imu_frame"),
        "enable_debug_topics": LaunchConfiguration("enable_debug_topics"),
        "joint_config_path": joint_config_path,
        "geometry_config_path": geometry_config_path,
        "cmd_accel_limit_x": float(controller_cfg["cmd_accel_limit_x"]) * speed_scale,
        "cmd_accel_limit_y": float(controller_cfg["cmd_accel_limit_y"]) * speed_scale,
        "cmd_accel_limit_yaw": float(controller_cfg["cmd_accel_limit_yaw"]) * speed_scale,
        "tripod_max_linear_x": tripod_max_linear_x,
        "tripod_max_linear_y": tripod_max_linear_y,
        "tripod_max_angular_z": tripod_max_angular_z,
        "tripod_forward_stride_m": tripod_forward_stride_m,
        "tripod_lateral_stride_m": tripod_lateral_stride_m,
        "tripod_turn_stride_m": tripod_turn_stride_m,
    }
    if locomotion_startup_mode is not None:
        parameters["locomotion_startup_mode"] = locomotion_startup_mode

    return [
        Node(
            package="hexbot_locomotion_v2",
            executable="hexbot_locomotion_v2_node.py",
            name="hexbot_locomotion_v2",
            output="screen",
            parameters=[parameters],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("hexbot_locomotion_v2")
    default_geometry_config_path = os.path.join(
        package_share, "config", "hexbot_v2_stage2_explicit_current_candidate.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("mode", default_value="tripod"),
            DeclareLaunchArgument(
                "geometry_config_path",
                default_value=default_geometry_config_path,
                description="Path to the locomotion geometry and gait config YAML.",
            ),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            DeclareLaunchArgument("joint_command_topic", default_value="/joint_command"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("imu_topic", default_value="/imu"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("imu_frame", default_value="imu_link"),
            DeclareLaunchArgument("enable_debug_topics", default_value="true"),
            DeclareLaunchArgument(
                "locomotion_startup_mode",
                default_value="",
                description="Optional override for startup seeding mode (live, hybrid, fixed_stand).",
            ),
            DeclareLaunchArgument(
                "speed_scale",
                default_value="1.0",
                description="Scales tripod speed envelope and stride lengths.",
            ),
            DeclareLaunchArgument(
                "tripod_max_linear_x",
                default_value="",
                description="Optional override for tripod max forward speed.",
            ),
            DeclareLaunchArgument(
                "tripod_max_linear_y",
                default_value="",
                description="Optional override for tripod max lateral speed.",
            ),
            DeclareLaunchArgument(
                "tripod_max_angular_z",
                default_value="",
                description="Optional override for tripod max yaw speed.",
            ),
            DeclareLaunchArgument(
                "tripod_forward_stride_m",
                default_value="",
                description="Optional override for tripod forward stride length.",
            ),
            DeclareLaunchArgument(
                "tripod_lateral_stride_m",
                default_value="",
                description="Optional override for tripod lateral stride length.",
            ),
            DeclareLaunchArgument(
                "tripod_turn_stride_m",
                default_value="",
                description="Optional override for tripod turn stride length.",
            ),
            OpaqueFunction(function=_create_locomotion_node),
        ]
    )
