"""The a200-0553's lidar and its scan, as robot.yaml starts them.

robot.yaml's ``platform.extras.launch`` names this file with ``lidar_model``; Clearpath's clearpath-platform-extras
service runs it.  It includes onboard/navigation's ``scan.launch.py``, which includes onboard/lidar's sensor path --
READ-ONLY all of it: nothing commands the base or the arm.  ``lidar_model`` has no default here, so robot.yaml has to
name the model and is the only place that does.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from clair.navigation import SCAN_LAUNCH_PATH


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("lidar_model", description="the lidar model (clair.lidar.models), robot.yaml's"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(SCAN_LAUNCH_PATH)),
                launch_arguments={"lidar_model": LaunchConfiguration("lidar_model")}.items(),
            ),
        ]
    )
