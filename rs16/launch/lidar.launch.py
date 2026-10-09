"""Sensor path of the RS16: the driver and the self filter -- the points, raw and without the robot's body.

READ-ONLY.  This launch commands nothing and may therefore also run on the graph of the real robot.  It is all a
consumer of the points needs, perception as much as navigation; navigation's ``launch/scan.launch.py`` includes it
and derives its 2D scan on top.

Invocation inside the container:
    ros2 launch /opt/clair/rs16/launch/lidar.launch.py \
        pcap:=/data/recordings/rs16_labor.pcap     # mock
    ros2 launch /opt/clair/rs16/launch/lidar.launch.py
                                                   # device (no pcap arg)

Every name comes from clair.rs16.wiring -- none of them is stated a second time here
(tests/test_lidar_launch_restates_no_wiring.py).
"""

from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from clair.rs16 import rslidar_config, wiring
from launch import LaunchDescription


def _setup(context, *args, **kwargs):
    pcap = LaunchConfiguration("pcap").perform(context)
    # rslidar_sdk knows no way to override individual keys -- it reads exactly one file.  The resolved version is
    # therefore produced here, from the SAME template the robot uses as well.
    resolved = rslidar_config.write_resolved("/tmp/rslidar_rs16_resolved.yaml", pcap_path=pcap or None)

    return [
        Node(
            package="rslidar_sdk",
            executable="rslidar_sdk_node",
            name="rslidar_sdk_node",
            namespace=wiring.NAMESPACE,
            parameters=[{"config_path": str(resolved)}],
            remappings=wiring.TF_REMAPS,
            output="screen",
        ),
        # The points without the robot's own body (the arch beside the sensor, the arm reaching out): what navigation's
        # scan and costmaps read.  A python module, not an ament package -- hence ExecuteProcess.
        ExecuteProcess(cmd=["python3", "-m", "clair.rs16.self_filter"], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pcap", default_value="", description="the path of an RS16 recording. Empty = the real device."
            ),
            OpaqueFunction(function=_setup),
        ]
    )
