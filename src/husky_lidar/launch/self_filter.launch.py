"""The self filter: the lidar's points without the robot's own body.

The one place that starts it.  lidar.launch.py includes it behind the driver, the container's mock includes it on the
plant's clock, and ``up.sh --lidar on`` launches it on a running MuJoCo stack.  The node's names are relative; every
one is mapped here onto husky_lidar.wiring.

    ros2 launch husky_lidar self_filter.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from husky_lidar import RESPAWN_DELAY_S, wiring


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="the plant's clock, which stamps the points"
            ),
            Node(
                package="husky_lidar",
                executable="self_filter",
                namespace=wiring.NAMESPACE,
                parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}],
                remappings=[
                    ("points", wiring.points_topic()),
                    ("points_filtered", wiring.filtered_points_topic()),
                    ("robot_description", wiring.description_topic()),
                    *wiring.TF_REMAPS,
                ],
                respawn=True,
                respawn_delay=RESPAWN_DELAY_S,
                output="screen",
            ),
        ]
    )
