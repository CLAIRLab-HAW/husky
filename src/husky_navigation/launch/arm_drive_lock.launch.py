"""Autonomous driving locked while the arm is out of a drive pose (husky_navigation.arm_drive_lock) -- started from
robot.yaml's platform.extras.launch, on the robot."""

from launch import LaunchDescription
from launch_ros.actions import Node

from husky_navigation import wiring


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="husky_navigation",
                executable="arm_drive_lock",
                name="arm_drive_lock",
                namespace=wiring.NAMESPACE,
                respawn=True,
                respawn_delay=2.0,
                output="screen",
            )
        ]
    )
