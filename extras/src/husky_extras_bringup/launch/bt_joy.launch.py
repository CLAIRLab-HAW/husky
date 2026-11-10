"""The Bluetooth joystick pairing, as robot.yaml starts it: the BlueZ agent and ``bt_joy/open_pairing``.

robot.yaml's ``platform.extras.launch`` names this file with the robot's ``namespace``; Clearpath's
clearpath-platform-extras service runs it.  The program is the installer's root-owned copy of
husky/setup's ``scripts/bt_joy_pairing.py``, ``bt-joy-pairing`` on the PATH -- the installer smoke-tests it and
``--verify`` measures it against the checkout.  It respawns: that service's unit carries no Restart=, and an agent
started before bluetoothd is up exits and comes back.

Where the installer left the pairing out, the program is missing and this launch starts nothing and says so: an
executable launch cannot find raises in it, and an exception ends the WHOLE extras launch, the lidar with it.
"""

import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

#: The program, as the installer puts it on the PATH (husky/setup/install-clearpath-custom-setup.sh, BT_JOY_BIN).
PROGRAM = "bt-joy-pairing"
#: How long a crashed agent waits before it comes back [s] -- the 5 s its own systemd unit gave it.
RESPAWN_DELAY_S = 5.0


def _setup(context, *args, **kwargs):
    if shutil.which(PROGRAM) is None:
        return [LogInfo(msg=f"{PROGRAM} is not installed (the installer's pairing step was skipped) -- not started")]
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    return [
        ExecuteProcess(
            cmd=[PROGRAM, "--ros-args", "-r", f"__ns:=/{namespace}"],
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
