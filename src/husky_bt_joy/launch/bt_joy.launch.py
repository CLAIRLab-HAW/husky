"""The Bluetooth joystick pairing, as robot.yaml starts it: the BlueZ agent and ``bt_joy/open_pairing``.

robot.yaml's ``platform.extras.launch`` names this file with the robot's ``namespace``; Clearpath's
clearpath-platform-extras service runs it.  The node respawns: that service's unit carries no Restart=, and an agent
started before bluetoothd is up exits and comes back.

The installer's D-Bus policy is the opt-in: without it the robot user may not talk to ``org.bluez``, the agent would
die at registration and respawn every few seconds, so where the installer's pairing step was skipped this launch
starts nothing and says so.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: The policy the installer writes (husky/setup/install-clearpath-custom-setup.sh, BT_JOY_DBUS_POLICY).
DBUS_POLICY = Path("/etc/dbus-1/system.d/clearpath-custom-bt-joy.conf")
#: How long a crashed agent waits before it comes back [s] -- the 5 s its former systemd unit gave it.
RESPAWN_DELAY_S = 5.0


def _setup(context, *args, **kwargs):
    if not DBUS_POLICY.is_file():
        return [LogInfo(msg=f"{DBUS_POLICY} is missing (the installer's pairing step was skipped) -- not started")]
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    return [
        Node(
            package="husky_bt_joy",
            executable="bt_joy_pairing",
            namespace=f"/{namespace}",
            respawn=True,
            respawn_delay=RESPAWN_DELAY_S,
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", description="the robot's namespace, robot.yaml's system.ros2.namespace"),
            OpaqueFunction(function=_setup),
        ]
    )
