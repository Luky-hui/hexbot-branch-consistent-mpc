import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("hexbot_isaac_runtime")
    left_config = os.path.join(pkg_share, "config", "tags_left.yaml")
    right_config = os.path.join(pkg_share, "config", "tags_right.yaml")

    arg_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    apriltag_left = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_left",
        namespace="camera/left",
        output="screen",
        remappings=[
            ("image_rect", "/camera/left/image_raw"),
            ("camera_info", "/camera/left/camera_info"),
        ],
        parameters=[left_config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    apriltag_right = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_right",
        namespace="camera/right",
        output="screen",
        remappings=[
            ("image_rect", "/camera/right/image_raw"),
            ("camera_info", "/camera/right/camera_info"),
        ],
        parameters=[right_config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            arg_use_sim_time,
            apriltag_left,
            apriltag_right,
        ]
    )
