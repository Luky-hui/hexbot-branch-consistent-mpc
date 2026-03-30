import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_file = (
        get_package_share_directory("ocs2_hexbot_legged_robot_ros")
        + "/rviz/legged_robot.rviz"
    )
    description_share = get_package_share_directory("hexbot_description_ros2")
    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time", default_value="true"
            ),
            launch.actions.DeclareLaunchArgument(
                name="use_dummy", default_value="false"
            ),
            launch.actions.DeclareLaunchArgument(name="rviz", default_value="false"),
            launch.actions.DeclareLaunchArgument(
                name="multiplot", default_value="false"
            ),
            launch.actions.DeclareLaunchArgument(
                name="terminal_prefix", default_value=""
            ),
            launch.actions.DeclareLaunchArgument(
                name="taskFile",
                default_value=get_package_share_directory(
                    "ocs2_hexbot_legged_robot"
                )
                + "/config/mpc/task.info",
            ),
            launch.actions.DeclareLaunchArgument(
                name="referenceFile",
                default_value=get_package_share_directory(
                    "ocs2_hexbot_legged_robot"
                )
                + "/config/command/reference.info",
            ),
            launch.actions.DeclareLaunchArgument(
                name="urdfFile",
                default_value=os.path.join(
                    description_share, "urdf", "hexbot_isaac_rooted.urdf"
                ),
            ),
            launch.actions.DeclareLaunchArgument(
                name="gaitCommandFile",
                default_value=get_package_share_directory(
                    "ocs2_hexbot_legged_robot"
                )
                + "/config/command/gait.info",
            ),
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[launch.substitutions.LaunchConfiguration("urdfFile")],
                parameters=[
                    {
                        "use_sim_time": launch.substitutions.LaunchConfiguration(
                            "use_sim_time"
                        )
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
                parameters=[
                    {
                        "use_sim_time": launch.substitutions.LaunchConfiguration(
                            "use_sim_time"
                        )
                    }
                ],
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration("rviz")
                ),
            ),
            launch_ros.actions.Node(
                package="ocs2_hexbot_legged_robot_ros",
                executable="legged_robot_ddp_mpc",
                name="hexbot_ddp_mpc",
                output="screen",
                prefix="",
                parameters=[
                    {"taskFile": launch.substitutions.LaunchConfiguration("taskFile")},
                    {
                        "referenceFile": launch.substitutions.LaunchConfiguration(
                            "referenceFile"
                        )
                    },
                    {"urdfFile": launch.substitutions.LaunchConfiguration("urdfFile")},
                    {
                        "use_sim_time": launch.substitutions.LaunchConfiguration(
                            "use_sim_time"
                        )
                    },
                ],
            ),
            launch_ros.actions.Node(
                package="ocs2_hexbot_legged_robot_ros",
                executable="legged_robot_dummy",
                name="hexbot_mrt",
                output="screen",
                prefix=launch.substitutions.LaunchConfiguration("terminal_prefix"),
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration("use_dummy")
                ),
                parameters=[
                    {"taskFile": launch.substitutions.LaunchConfiguration("taskFile")},
                    {
                        "referenceFile": launch.substitutions.LaunchConfiguration(
                            "referenceFile"
                        )
                    },
                    {"urdfFile": launch.substitutions.LaunchConfiguration("urdfFile")},
                    {
                        "backendJointCommandTopic": "/hexbot_mpc/output/joint_command"
                    },
                    {
                        "use_sim_time": launch.substitutions.LaunchConfiguration(
                            "use_sim_time"
                        )
                    },
                ],
            ),
            launch_ros.actions.Node(
                package="ocs2_hexbot_legged_robot_ros",
                executable="hexbot_runtime_mrt",
                name="hexbot_runtime_mrt",
                output="screen",
                prefix=launch.substitutions.LaunchConfiguration("terminal_prefix"),
                condition=launch.conditions.UnlessCondition(
                    launch.substitutions.LaunchConfiguration("use_dummy")
                ),
                parameters=[
                    {"taskFile": launch.substitutions.LaunchConfiguration("taskFile")},
                    {
                        "referenceFile": launch.substitutions.LaunchConfiguration(
                            "referenceFile"
                        )
                    },
                    {"urdfFile": launch.substitutions.LaunchConfiguration("urdfFile")},
                    {"jointStatesTopic": "/hexbot_mpc/input/joint_states"},
                    {"odomTopic": "/hexbot_mpc/input/odom"},
                    {
                        "backendJointCommandTopic": "/hexbot_mpc/output/joint_command"
                    },
                    {
                        "use_sim_time": launch.substitutions.LaunchConfiguration(
                            "use_sim_time"
                        )
                    },
                ],
            ),
            launch_ros.actions.Node(
                package="ocs2_hexbot_legged_robot_ros",
                executable="hexbot_cmd_vel_target",
                name="hexbot_target",
                output="screen",
                prefix=launch.substitutions.LaunchConfiguration("terminal_prefix"),
                parameters=[
                    {
                        "referenceFile": launch.substitutions.LaunchConfiguration(
                            "referenceFile"
                        )
                    },
                    {"cmdVelTopic": "/hexbot_mpc/input/cmd_vel"},
                    {
                        "use_sim_time": launch.substitutions.LaunchConfiguration(
                            "use_sim_time"
                        )
                    },
                ],
            ),
            launch_ros.actions.Node(
                package="ocs2_hexbot_legged_robot_ros",
                executable="hexbot_auto_gait_command",
                name="hexbot_gait_command",
                output="screen",
                prefix=launch.substitutions.LaunchConfiguration("terminal_prefix"),
                parameters=[
                    {
                        "gaitCommandFile": launch.substitutions.LaunchConfiguration(
                            "gaitCommandFile"
                        )
                    },
                    {"cmdVelTopic": "/hexbot_mpc/input/cmd_vel"},
                    {
                        "use_sim_time": launch.substitutions.LaunchConfiguration(
                            "use_sim_time"
                        )
                    },
                ],
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
