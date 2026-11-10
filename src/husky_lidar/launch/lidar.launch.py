"""Sensor path of the lidar: the model's driver and the self filter -- the points, raw and without the robot's body.

READ-ONLY.  This launch commands nothing and may therefore also run on the graph of the real robot.  It is all a
consumer of the points needs, perception as much as navigation; navigation's ``launch/scan.launch.py`` includes it
and derives its 2D scan on top.

Invocation inside the container:
    ros2 launch husky_lidar lidar.launch.py \
        pcap:=/data/recordings/rs16_labor.pcap     # mock
    ros2 launch husky_lidar lidar.launch.py
                                                   # device (no pcap arg)

``model:=`` picks the driver (husky_lidar.models): rslidar_sdk for the RS16, Blickfeld's driver for the Qb2; without
it, the RS16, whose recording the container replays.  ``pcap:=`` is rslidar_sdk's alone -- the Qb2's driver replays
nothing.  On the robot Clearpath's clearpath-platform-extras service starts this, through husky_bringup, with
robot.yaml's ``lidar_model``.  Driver and filter respawn: that service's unit carries no Restart=.

Every name comes from husky_lidar.wiring -- none of them is stated a second time here
(tests/test_lidar_launch_restates_no_wiring.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from husky_lidar import RESPAWN_DELAY_S, blickfeld_qb2, models, rslidar, wiring


def _rslidar(model: models.LidarModel, pcap: str | None) -> Node:
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


def _blickfeld_qb2(model: models.LidarModel, pcap: str | None) -> Node:
    if pcap:
        raise ValueError(f"{model.name}'s driver replays no recording ({pcap}); play a rosbag of its topic instead")
    # A component and nothing else: the driver ships no executable of its own, its launch loads it into a container.
    return ComposableNodeContainer(
        name="blickfeld_qb2_container",
        namespace=wiring.NAMESPACE,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package=blickfeld_qb2.PACKAGE,
                plugin=blickfeld_qb2.PLUGIN,
                name="blickfeld_qb2_driver",
                namespace=wiring.NAMESPACE,
                parameters=[blickfeld_qb2.parameters(model)],
            )
        ],
        respawn=True,
        respawn_delay=RESPAWN_DELAY_S,
        output="screen",
    )


#: The launch's half of husky_lidar.models.DRIVERS: how each driver is started.
DRIVERS = {"rslidar": _rslidar, "blickfeld_qb2": _blickfeld_qb2}


def _setup(context, *args, **kwargs):
    model = models.model(LaunchConfiguration("model").perform(context))
    pcap = LaunchConfiguration("pcap").perform(context) or None
    return [
        DRIVERS[model.driver](model, pcap),
        # The points without the robot's own body (the arch beside the sensor, the arm reaching out): what navigation's
        # scan and costmaps read.  A python module, not an ament package -- hence ExecuteProcess.
        ExecuteProcess(
            cmd=["python3", "-m", "husky_lidar.self_filter"],
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
                description="the lidar model (husky_lidar.models), robot.yaml's lidar_model on the robot",
            ),
            DeclareLaunchArgument(
                "pcap", default_value="", description="an rslidar_sdk recording of that model. Empty = the device."
            ),
            OpaqueFunction(function=_setup),
        ]
    )
