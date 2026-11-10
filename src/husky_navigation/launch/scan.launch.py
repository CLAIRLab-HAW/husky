"""The lidar's 2D scan, on top of its sensor path: husky_lidar's lidar.launch.py plus pointcloud_to_laserscan.

READ-ONLY, like the launch it includes: nothing here commands the base or the arm, so it may run on the graph of the
real robot -- what is barred is navigation.launch.py, because its controller_server writes to cmd_vel.  AMCL,
slam_toolbox and the costmaps read the scan.  On the robot Clearpath's clearpath-platform-extras service starts this
launch through husky_bringup, with robot.yaml's ``lidar_model``, so the scan is there before Nav2 is; the
nodes respawn, because that service's unit carries no Restart=.

Invocation inside the container:
    ros2 launch husky_navigation scan.launch.py pcap:=/data/recordings/rs16_labor.pcap   # mock
    ros2 launch husky_navigation scan.launch.py                                          # device

Every name comes from husky_navigation.wiring (tests/test_launch_files_do_not_restate_the_wiring.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from husky_lidar import LAUNCH_PATH, RESPAWN_DELAY_S, models
from husky_navigation import wiring


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_model",
                default_value=models.DEFAULT_MODEL,
                description="the lidar model (husky_lidar.models), robot.yaml's lidar_model on the robot",
            ),
            DeclareLaunchArgument(
                "pcap", default_value="", description="the path of a recording of that model. Empty = the device."
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(LAUNCH_PATH)),
                launch_arguments={
                    "model": LaunchConfiguration("lidar_model"),
                    "pcap": LaunchConfiguration("pcap"),
                }.items(),
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
                respawn=True,
                respawn_delay=RESPAWN_DELAY_S,
                output="screen",
            ),
        ]
    )
