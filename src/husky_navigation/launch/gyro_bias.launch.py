"""The UM7's gyro bias off before the EKF (husky_navigation.gyro_bias) -- started from robot.yaml's
platform.extras.launch, on the robot."""

from launch import LaunchDescription
from launch_ros.actions import Node

from husky_navigation import wiring


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="husky_navigation",
                executable="gyro_bias",
                name="gyro_bias",
                namespace=wiring.NAMESPACE,
                respawn=True,
                respawn_delay=2.0,
                output="screen",
            )
        ]
    )
