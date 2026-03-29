import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory("hexbot_locomotion_mpc")
    default_config = os.path.join(package_share, "config", "hexbot_mpc_default.yaml")

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time", default_value="true"
            ),
            launch.actions.DeclareLaunchArgument(
                name="geometry_config_path", default_value=default_config
            ),
            launch_ros.actions.Node(
                package="hexbot_locomotion_mpc",
                executable="hexbot_locomotion_mpc_bridge.py",
                name="hexbot_locomotion_mpc_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": launch.substitutions.LaunchConfiguration(
                            "use_sim_time"
                        )
                    },
                    {
                        "geometry_config_path": launch.substitutions.LaunchConfiguration(
                            "geometry_config_path"
                        )
                    },
                ],
            ),
        ]
    )
