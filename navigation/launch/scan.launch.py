"""The RS16's 2D scan, on top of its sensor path: onboard/rs16's lidar.launch.py plus pointcloud_to_laserscan.

READ-ONLY, like the launch it includes: nothing here commands the base or the arm, so it may run on the graph of the
real robot -- what is barred is navigation.launch.py, because its controller_server writes to cmd_vel.  AMCL,
slam_toolbox and the costmaps read the scan; the robot's lidar service starts this launch, so the scan is there before
Nav2 is.

Invocation inside the container:
    ros2 launch /opt/clair/navigation/launch/scan.launch.py pcap:=/data/recordings/rs16_labor.pcap   # mock
    ros2 launch /opt/clair/navigation/launch/scan.launch.py                                          # device

Every name comes from clair.navigation.wiring (tests/test_launch_files_do_not_restate_the_wiring.py).
"""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from clair.navigation import wiring
from clair.rs16 import rslidar_config
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pcap", default_value="", description="the path of an RS16 recording. Empty = the real device."
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rslidar_config.LAUNCH_PATH)),
                launch_arguments={"pcap": LaunchConfiguration("pcap")}.items(),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                namespace=wiring.NAMESPACE,
                parameters=[wiring.pointcloud_to_laserscan_params()],
                remappings=[
                    ("cloud_in", wiring.filtered_points_topic()),
                    ("scan", wiring.scan_topic()),
                    *wiring.TF_REMAPS,
                ],
                output="screen",
            ),
        ]
    )
