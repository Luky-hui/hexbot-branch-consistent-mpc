import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory("ocs2_hexbot_legged_robot_ros")
    core_share = get_package_share_directory("ocs2_hexbot_legged_robot")
    description_share = get_package_share_directory("hexbot_description_ros2")
    rviz_config_file = os.path.join(package_share, "rviz", "legged_robot.rviz")

    return launch.LaunchDescription(
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
                name="useHardFrictionConeConstraint", default_value="true"
            ),
            launch.actions.DeclareLaunchArgument(
                name="terminal_prefix", default_value=""
            ),
            launch.actions.DeclareLaunchArgument(
                name="taskFile",
                default_value=os.path.join(core_share, "config", "mpc", "task.info"),
            ),
            launch.actions.DeclareLaunchArgument(
                name="referenceFile",
                default_value=os.path.join(
                    core_share, "config", "command", "reference.info"
                ),
            ),
            launch.actions.DeclareLaunchArgument(
                name="urdfFile",
                default_value=os.path.join(
                    description_share, "urdf", "hexbot_isaac_rooted.urdf"
                ),
            ),
            launch.actions.DeclareLaunchArgument(
                name="gaitCommandFile",
                default_value=os.path.join(core_share, "config", "command", "gait.info"),
            ),
            launch.actions.DeclareLaunchArgument(
                name="cmdVelTimeout",
                default_value="2.0",
            ),
            launch.actions.DeclareLaunchArgument(
                name="lockNominalHeightToObservation",
                default_value="true",
            ),
            launch.actions.DeclareLaunchArgument(
                name="lockResetHeightToObservation",
                default_value="true",
            ),
            launch.actions.DeclareLaunchArgument(
                name="translationLookaheadTime",
                default_value="0.35",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rotationLookaheadTime",
                default_value="0.50",
            ),
            launch.actions.DeclareLaunchArgument(
                name="commandTrackingResetThreshold",
                default_value="2.0",
            ),
            launch.actions.DeclareLaunchArgument(
                name="equivalentAngleWrapResetThreshold",
                default_value="4.5",
            ),
            launch.actions.DeclareLaunchArgument(
                name="enableJointBranchConsistencyGuard",
                default_value="true",
            ),
            launch.actions.DeclareLaunchArgument(
                name="jointBranchGuardCommandDeviationThreshold",
                default_value="1.5",
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
                executable="legged_robot_sqp_mpc",
                name="hexbot_sqp_mpc",
                output="screen",
                parameters=[
                    {"multiplot": launch.substitutions.LaunchConfiguration("multiplot")},
                    {
                        "useHardFrictionConeConstraint": launch.substitutions.LaunchConfiguration(
                            "useHardFrictionConeConstraint"
                        )
                    },
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
                        "lockResetHeightToObservation": launch.substitutions.LaunchConfiguration(
                            "lockResetHeightToObservation"
                        )
                    },
                    {
                        "commandTrackingResetThreshold": launch.substitutions.LaunchConfiguration(
                            "commandTrackingResetThreshold"
                        )
                    },
                    {
                        "equivalentAngleWrapResetThreshold": launch.substitutions.LaunchConfiguration(
                            "equivalentAngleWrapResetThreshold"
                        )
                    },
                    {
                        "enableJointBranchConsistencyGuard": launch.substitutions.LaunchConfiguration(
                            "enableJointBranchConsistencyGuard"
                        )
                    },
                    {
                        "jointBranchGuardCommandDeviationThreshold": launch.substitutions.LaunchConfiguration(
                            "jointBranchGuardCommandDeviationThreshold"
                        )
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
                    {"taskFile": launch.substitutions.LaunchConfiguration("taskFile")},
                    {"urdfFile": launch.substitutions.LaunchConfiguration("urdfFile")},
                    {
                        "referenceFile": launch.substitutions.LaunchConfiguration(
                            "referenceFile"
                        )
                    },
                    {"cmdVelTopic": "/hexbot_mpc/input/cmd_vel"},
                    {
                        "cmdVelTimeout": launch.substitutions.LaunchConfiguration(
                            "cmdVelTimeout"
                        )
                    },
                    {
                        "lockNominalHeightToObservation": launch.substitutions.LaunchConfiguration(
                            "lockNominalHeightToObservation"
                        )
                    },
                    {
                        "translationLookaheadTime": launch.substitutions.LaunchConfiguration(
                            "translationLookaheadTime"
                        )
                    },
                    {
                        "rotationLookaheadTime": launch.substitutions.LaunchConfiguration(
                            "rotationLookaheadTime"
                        )
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
                        "cmdVelTimeout": launch.substitutions.LaunchConfiguration(
                            "cmdVelTimeout"
                        )
                    },
                    {
                        "use_sim_time": launch.substitutions.LaunchConfiguration(
                            "use_sim_time"
                        )
                    },
                ],
            ),
        ]
    )
