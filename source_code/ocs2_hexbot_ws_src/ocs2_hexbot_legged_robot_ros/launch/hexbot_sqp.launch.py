import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    package_share = get_package_share_directory("ocs2_hexbot_legged_robot_ros")
    core_share = get_package_share_directory("ocs2_hexbot_legged_robot")
    description_share = get_package_share_directory("hexbot_description_ros2")
    rviz_config_file = os.path.join(package_share, "rviz", "legged_robot.rviz")
    solver = LaunchConfiguration("solver")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_dummy = LaunchConfiguration("use_dummy")
    rviz = LaunchConfiguration("rviz")
    multiplot = LaunchConfiguration("multiplot")
    use_hard_friction = LaunchConfiguration("useHardFrictionConeConstraint")
    terminal_prefix = LaunchConfiguration("terminal_prefix")
    task_file = LaunchConfiguration("taskFile")
    reference_file = LaunchConfiguration("referenceFile")
    urdf_file = LaunchConfiguration("urdfFile")
    gait_command_file = LaunchConfiguration("gaitCommandFile")
    cmd_vel_timeout = LaunchConfiguration("cmdVelTimeout")
    lock_nominal_height = LaunchConfiguration("lockNominalHeightToObservation")
    lock_reset_height = LaunchConfiguration("lockResetHeightToObservation")
    translation_lookahead = LaunchConfiguration("translationLookaheadTime")
    rotation_lookahead = LaunchConfiguration("rotationLookaheadTime")
    moving_goal_reference_file = LaunchConfiguration("movingGoalReferenceFile")
    auto_load_moving_goal_reference_file = LaunchConfiguration(
        "autoLoadMovingGoalReferenceFile"
    )
    use_observed_joint_target_while_moving = LaunchConfiguration(
        "useObservedJointTargetWhileMoving"
    )
    command_tracking_reset_threshold = LaunchConfiguration(
        "commandTrackingResetThreshold"
    )
    equivalent_angle_wrap_reset_threshold = LaunchConfiguration(
        "equivalentAngleWrapResetThreshold"
    )
    enable_joint_branch_consistency_guard = LaunchConfiguration(
        "enableJointBranchConsistencyGuard"
    )
    joint_branch_guard_command_deviation_threshold = LaunchConfiguration(
        "jointBranchGuardCommandDeviationThreshold"
    )

    solver_is_sqp = IfCondition(
        PythonExpression(["'", solver, "' == 'sqp'"])
    )
    solver_is_ipm = IfCondition(
        PythonExpression(["'", solver, "' == 'ipm'"])
    )

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
                name="solver", default_value="ipm"
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
                default_value="1.2",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rotationLookaheadTime",
                default_value="1.2",
            ),
            launch.actions.DeclareLaunchArgument(
                name="movingGoalReferenceFile",
                default_value="",
            ),
            launch.actions.DeclareLaunchArgument(
                name="autoLoadMovingGoalReferenceFile",
                default_value="true",
            ),
            launch.actions.DeclareLaunchArgument(
                name="useObservedJointTargetWhileMoving",
                default_value="false",
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
                arguments=[urdf_file],
                parameters=[
                    {"use_sim_time": use_sim_time}
                ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(rviz),
            ),
            launch_ros.actions.Node(
                package="ocs2_hexbot_legged_robot_ros",
                executable="legged_robot_sqp_mpc",
                name="hexbot_sqp_mpc",
                output="screen",
                condition=solver_is_sqp,
                parameters=[
                    {"multiplot": multiplot},
                    {"useHardFrictionConeConstraint": use_hard_friction},
                    {"taskFile": task_file},
                    {"referenceFile": reference_file},
                    {"urdfFile": urdf_file},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            launch_ros.actions.Node(
                package="ocs2_hexbot_legged_robot_ros",
                executable="legged_robot_ipm_mpc",
                name="hexbot_ipm_mpc",
                output="screen",
                condition=solver_is_ipm,
                parameters=[
                    {"multiplot": multiplot},
                    {"taskFile": task_file},
                    {"referenceFile": reference_file},
                    {"urdfFile": urdf_file},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            launch_ros.actions.Node(
                package="ocs2_hexbot_legged_robot_ros",
                executable="legged_robot_dummy",
                name="hexbot_mrt",
                output="screen",
                prefix=terminal_prefix,
                condition=IfCondition(use_dummy),
                parameters=[
                    {"taskFile": task_file},
                    {"referenceFile": reference_file},
                    {"urdfFile": urdf_file},
                    {"backendJointCommandTopic": "/hexbot_mpc/output/joint_command"},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            launch_ros.actions.Node(
                package="ocs2_hexbot_legged_robot_ros",
                executable="hexbot_runtime_mrt",
                name="hexbot_runtime_mrt",
                output="screen",
                prefix=terminal_prefix,
                condition=launch.conditions.UnlessCondition(use_dummy),
                parameters=[
                    {"taskFile": task_file},
                    {"referenceFile": reference_file},
                    {"urdfFile": urdf_file},
                    {"jointStatesTopic": "/hexbot_mpc/input/joint_states"},
                    {"odomTopic": "/hexbot_mpc/input/odom"},
                    {"backendJointCommandTopic": "/hexbot_mpc/output/joint_command"},
                    {"lockResetHeightToObservation": lock_reset_height},
                    {
                        "commandTrackingResetThreshold": command_tracking_reset_threshold
                    },
                    {
                        "equivalentAngleWrapResetThreshold": equivalent_angle_wrap_reset_threshold
                    },
                    {
                        "enableJointBranchConsistencyGuard": enable_joint_branch_consistency_guard
                    },
                    {
                        "jointBranchGuardCommandDeviationThreshold": joint_branch_guard_command_deviation_threshold
                    },
                    {"use_sim_time": use_sim_time},
                ],
            ),
            launch_ros.actions.Node(
                package="ocs2_hexbot_legged_robot_ros",
                executable="hexbot_cmd_vel_target",
                name="hexbot_target",
                output="screen",
                prefix=terminal_prefix,
                parameters=[
                    {"taskFile": task_file},
                    {"urdfFile": urdf_file},
                    {"referenceFile": reference_file},
                    {"cmdVelTopic": "/hexbot_mpc/input/cmd_vel"},
                    {"cmdVelTimeout": cmd_vel_timeout},
                    {"lockNominalHeightToObservation": lock_nominal_height},
                    {"translationLookaheadTime": translation_lookahead},
                    {"rotationLookaheadTime": rotation_lookahead},
                    {"movingGoalReferenceFile": moving_goal_reference_file},
                    {
                        "autoLoadMovingGoalReferenceFile": auto_load_moving_goal_reference_file
                    },
                    {
                        "useObservedJointTargetWhileMoving": use_observed_joint_target_while_moving
                    },
                    {"use_sim_time": use_sim_time},
                ],
            ),
            launch_ros.actions.Node(
                package="ocs2_hexbot_legged_robot_ros",
                executable="hexbot_auto_gait_command",
                name="hexbot_gait_command",
                output="screen",
                prefix=terminal_prefix,
                parameters=[
                    {"gaitCommandFile": gait_command_file},
                    {"cmdVelTopic": "/hexbot_mpc/input/cmd_vel"},
                    {"cmdVelTimeout": cmd_vel_timeout},
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
