import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    declare_frame_id_cmd = DeclareLaunchArgument(
        "frame_id",
        default_value="base_link",
        description="Robot base frame_id",
    )
    declare_odom_topic_cmd = DeclareLaunchArgument(
        "odom_topic",
        default_value="odom",
        description="Odometry topic name",
    )
    declare_scan_topic_cmd = DeclareLaunchArgument(
        "scan_topic",
        default_value="scan",
        description="Laser scan topic name",
    )
    declare_database_path_cmd = DeclareLaunchArgument(
        "database_path",
        default_value=os.path.join(os.path.expanduser("~"), ".ros", "hexbot_rtabmap.db"),
        description="Path to RTAB-Map database",
    )
    declare_delete_db_cmd = DeclareLaunchArgument(
        "delete_db_on_start",
        default_value="true",
        description="Whether to delete the RTAB-Map database on start",
    )
    declare_viz_cmd = DeclareLaunchArgument(
        "launch_viz",
        default_value="true",
        description="Whether to launch rtabmap_viz",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    rtabmap_arguments = [
        PythonExpression(
            [
                "'--delete_db_on_start' if '",
                LaunchConfiguration("delete_db_on_start"),
                "' == 'true' else ''",
            ]
        )
    ]

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            {
                "frame_id": LaunchConfiguration("frame_id"),
                "subscribe_depth": False,
                "subscribe_rgb": False,
                "subscribe_stereo": False,
                "subscribe_rgbd": False,
                "subscribe_scan": True,
                "subscribe_scan_cloud": False,
                "approx_sync": True,
                "queue_size": 30,
                "qos_scan": 1,
                "qos_odom": 1,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "database_path": LaunchConfiguration("database_path"),
                "RGBD/NeighborLinkRefining": "true",
                "RGBD/ProximityBySpace": "true",
                "RGBD/AngularUpdate": "0.01",
                "RGBD/LinearUpdate": "0.01",
                "RGBD/OptimizeFromGraphEnd": "false",
                "Grid/FromDepth": "false",
                "Grid/Sensor": "0",
                "Reg/Force3DoF": "true",
                "Reg/Strategy": "1",
                "Icp/VoxelSize": "0.05",
                "Icp/MaxCorrespondenceDistance": "0.1",
                "Vis/MinInliers": "10",
            }
        ],
        remappings=[
            ("odom", LaunchConfiguration("odom_topic")),
            ("scan", LaunchConfiguration("scan_topic")),
        ],
        arguments=rtabmap_arguments,
    )

    rtabmap_viz_node = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[
            {
                "frame_id": LaunchConfiguration("frame_id"),
                "subscribe_odom_info": False,
                "subscribe_rgb": False,
                "subscribe_stereo": False,
                "subscribe_rgbd": False,
                "subscribe_scan": True,
                "subscribe_scan_cloud": False,
                "queue_size": 10,
                "qos_scan": 1,
                "qos_odom": 1,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        remappings=[
            ("odom", LaunchConfiguration("odom_topic")),
            ("scan", LaunchConfiguration("scan_topic")),
        ],
        condition=IfCondition(LaunchConfiguration("launch_viz")),
    )

    return LaunchDescription(
        [
            declare_frame_id_cmd,
            declare_odom_topic_cmd,
            declare_scan_topic_cmd,
            declare_database_path_cmd,
            declare_delete_db_cmd,
            declare_viz_cmd,
            declare_use_sim_time_cmd,
            rtabmap_node,
            rtabmap_viz_node,
        ]
    )
