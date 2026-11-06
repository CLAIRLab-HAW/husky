"""Sensor path of the lidar: the model's driver and the self filter -- the points, raw and without the robot's body.

READ-ONLY.  This launch commands nothing and may therefore also run on the graph of the real robot.  It is all a
consumer of the points needs, perception as much as navigation; navigation's ``launch/scan.launch.py`` includes it
and derives its 2D scan on top.

Invocation inside the container:
    ros2 launch /opt/clair/lidar/launch/lidar.launch.py \
        pcap:=/data/recordings/rs16_labor.pcap     # mock
    ros2 launch /opt/clair/lidar/launch/lidar.launch.py
                                                   # device (no pcap arg)

``model:=`` picks the driver (clair.lidar.models); without it, the a200-0553's.  On the robot Clearpath's
clearpath-platform-extras service starts it, through husky_extras_bringup, with robot.yaml's ``lidar_model``.  Both
nodes respawn: that service's unit carries no Restart=.

Every name comes from clair.lidar.wiring -- none of them is stated a second time here
(tests/test_lidar_launch_restates_no_wiring.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from clair.lidar import RESPAWN_DELAY_S, models, rslidar, wiring


def _driver(model: models.LidarModel, pcap: str | None) -> Node:
    # rslidar_sdk knows no way to override individual keys -- it reads exactly one file.  The resolved version is
    # therefore produced here, from the SAME template the robot uses as well.
    resolved = rslidar.write_resolved(f"/tmp/rslidar_{model.name}_resolved.yaml", model, pcap_path=pcap)
    return Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="rslidar_sdk_node",
        namespace=wiring.NAMESPACE,
        parameters=[{"config_path": str(resolved)}],
        remappings=wiring.TF_REMAPS,
        respawn=True,
        respawn_delay=RESPAWN_DELAY_S,
        output="screen",
    )


def _setup(context, *args, **kwargs):
    model = models.model(LaunchConfiguration("model").perform(context))
    pcap = LaunchConfiguration("pcap").perform(context) or None
    return [
        _driver(model, pcap),
        # The points without the robot's own body (the arch beside the sensor, the arm reaching out): what navigation's
        # scan and costmaps read.  A python module, not an ament package -- hence ExecuteProcess.
        ExecuteProcess(
            cmd=["python3", "-m", "clair.lidar.self_filter"],
            respawn=True,
            respawn_delay=RESPAWN_DELAY_S,
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=models.DEFAULT_MODEL,
                choices=sorted(models.MODELS),
                description="the lidar model (clair.lidar.models), robot.yaml's lidar_model on the robot",
            ),
            DeclareLaunchArgument(
                "pcap", default_value="", description="the path of a recording of that model. Empty = the device."
            ),
            OpaqueFunction(function=_setup),
        ]
    )
