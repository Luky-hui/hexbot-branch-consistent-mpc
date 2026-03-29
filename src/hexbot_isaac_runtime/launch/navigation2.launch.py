import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _resolve_optional_float(context, name: str):
    raw_value = LaunchConfiguration(name).perform(context).strip()
    return None if raw_value == "" else float(raw_value)


def _scaled_value(explicit_value, base_value: float, scale: float) -> float:
    if explicit_value is not None:
        return explicit_value
    return base_value * scale


def _build_navigation_launch(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    speed_scale = float(LaunchConfiguration("speed_scale").perform(context))

    with open(params_file, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)

    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = params["velocity_smoother"]["ros__parameters"]
    behavior = params["behavior_server"]["ros__parameters"]

    max_vel_x = _scaled_value(
        _resolve_optional_float(context, "nav_max_vel_x"),
        float(follow_path["max_vel_x"]),
        speed_scale,
    )
    max_vel_theta = _scaled_value(
        _resolve_optional_float(context, "nav_max_vel_theta"),
        float(follow_path["max_vel_theta"]),
        speed_scale,
    )
    acc_lim_x = _scaled_value(
        _resolve_optional_float(context, "nav_acc_lim_x"),
        float(follow_path["acc_lim_x"]),
        speed_scale,
    )
    acc_lim_theta = _scaled_value(
        _resolve_optional_float(context, "nav_acc_lim_theta"),
        float(follow_path["acc_lim_theta"]),
        speed_scale,
    )
    behavior_max_rotational_vel = _scaled_value(
        _resolve_optional_float(context, "behavior_max_rotational_vel"),
        float(behavior["max_rotational_vel"]),
        speed_scale,
    )
    behavior_rotational_acc_lim = _scaled_value(
        _resolve_optional_float(context, "behavior_rotational_acc_lim"),
        float(behavior["rotational_acc_lim"]),
        speed_scale,
    )

    reverse_linear_ratio = abs(float(smoother["min_velocity"][0])) / max(
        float(smoother["max_velocity"][0]), 1.0e-6
    )
    reverse_angular_ratio = abs(float(smoother["min_velocity"][2])) / max(
        float(smoother["max_velocity"][2]), 1.0e-6
    )

    follow_path["max_vel_x"] = max_vel_x
    follow_path["max_speed_xy"] = max_vel_x
    follow_path["max_vel_theta"] = max_vel_theta
    follow_path["acc_lim_x"] = acc_lim_x
    follow_path["decel_lim_x"] = -acc_lim_x
    follow_path["acc_lim_theta"] = acc_lim_theta
    follow_path["decel_lim_theta"] = -acc_lim_theta

    smoother["max_velocity"][0] = max_vel_x
    smoother["max_velocity"][2] = max_vel_theta
    smoother["min_velocity"][0] = -reverse_linear_ratio * max_vel_x
    smoother["min_velocity"][2] = -reverse_angular_ratio * max_vel_theta
    smoother["max_accel"][0] = acc_lim_x
    smoother["max_accel"][2] = acc_lim_theta
    smoother["max_decel"][0] = -acc_lim_x
    smoother["max_decel"][2] = -acc_lim_theta

    behavior["max_rotational_vel"] = behavior_max_rotational_vel
    behavior["rotational_acc_lim"] = behavior_rotational_acc_lim

    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="hexbot_nav2_",
        suffix=".yaml",
        delete=False,
    ) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        rewritten_params_file = stream.name

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    navigation_launch_file = os.path.join(
        nav2_bringup_dir, "launch", "navigation_launch.py"
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_file),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
                "params_file": rewritten_params_file,
            }.items(),
        )
    ]


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true",
    )

    pkg_share = get_package_share_directory("hexbot_isaac_runtime")
    default_params = os.path.join(pkg_share, "config", "hexbot_nav2_params.yaml")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to the ROS 2 parameters file for Nav2",
    )

    speed_scale_arg = DeclareLaunchArgument(
        "speed_scale",
        default_value="1.0",
        description="Scales Nav2 speed and acceleration limits.",
    )
    nav_max_vel_x_arg = DeclareLaunchArgument(
        "nav_max_vel_x",
        default_value="",
        description="Optional override for Nav2 max forward speed.",
    )
    nav_max_vel_theta_arg = DeclareLaunchArgument(
        "nav_max_vel_theta",
        default_value="",
        description="Optional override for Nav2 max yaw speed.",
    )
    nav_acc_lim_x_arg = DeclareLaunchArgument(
        "nav_acc_lim_x",
        default_value="",
        description="Optional override for Nav2 forward acceleration limit.",
    )
    nav_acc_lim_theta_arg = DeclareLaunchArgument(
        "nav_acc_lim_theta",
        default_value="",
        description="Optional override for Nav2 yaw acceleration limit.",
    )
    behavior_max_rotational_vel_arg = DeclareLaunchArgument(
        "behavior_max_rotational_vel",
        default_value="",
        description="Optional override for behavior-server max rotation speed.",
    )
    behavior_rotational_acc_lim_arg = DeclareLaunchArgument(
        "behavior_rotational_acc_lim",
        default_value="",
        description="Optional override for behavior-server rotation acceleration.",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            params_file_arg,
            speed_scale_arg,
            nav_max_vel_x_arg,
            nav_max_vel_theta_arg,
            nav_acc_lim_x_arg,
            nav_acc_lim_theta_arg,
            behavior_max_rotational_vel_arg,
            behavior_rotational_acc_lim_arg,
            OpaqueFunction(function=_build_navigation_launch),
        ]
    )
