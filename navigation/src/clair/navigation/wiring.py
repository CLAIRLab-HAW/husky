"""Topic and frame names of the navigation layer -- the single source.

ROS-free.  The launch files in ../../launch/ import from here; a test holds them to not stating the values a second
time.  The lidar's own names -- its points, its frame, the namespace -- are ``clair.lidar.wiring``'s and are only
re-exported here, so that a launch file needs one import.
"""

from __future__ import annotations

from clair.lidar.wiring import LIDAR_FRAME, NAMESPACE, TF_REMAPS, filtered_points_topic, points_topic

__all__ = [
    "BASE_FRAME",
    "LIDAR_FRAME",
    "NAMESPACE",
    "TF_REMAPS",
    "cmd_vel_topic",
    "filtered_points_topic",
    "pointcloud_to_laserscan_params",
    "points_topic",
    "scan_topic",
]

#: Root link of the URDF (robot-contract profile: frames.base_link).
BASE_FRAME = "base_link"


def scan_topic() -> str:
    """2D scan derived from the cloud (sensor_msgs/LaserScan).

    AMCL and slam_toolbox read LaserScan EXCLUSIVELY -- neither of them can process a PointCloud2.  This node is
    therefore not a convenience but a precondition.
    """
    return f"/{NAMESPACE}/sensors/lidar3d_0/scan"


def cmd_vel_topic() -> str:
    """Nav2's drive command.

    This is the twist_mux input ``external`` (priority 1, the lowest) -- joystick, RC and interactive marker therefore
    override Nav2 at any time.  The type is geometry_msgs/TwistStamped, not Twist.
    """
    return f"/{NAMESPACE}/cmd_vel"


def pointcloud_to_laserscan_params() -> dict:
    """Parameters of the pointcloud_to_laserscan node.

    The height band is RELATIVE TO base_link (target_frame) and has to contain the driving plane -- a band above or
    below it delivers nothing but ``inf`` and looks like a broken driver.
    """
    return {
        "target_frame": BASE_FRAME,
        "transform_tolerance": 0.05,
        # Band around the driving plane: everything between 7 cm below and 50 cm above base_link.  base_link sits
        # 10.1 cm above the ground (base_footprint at z=-0.101, the loaded tires measured on the robot, R58), so the
        # band starts 3 cm above the floor.
        "min_height": -0.07,
        "max_height": 0.50,
        "angle_min": -3.141592653589793,
        "angle_max": 3.141592653589793,
        "angle_increment": 0.0087,  # 0,5 degrees ─▶ 720 rays
        "scan_time": 0.1,  # the RS16 spins at 10 Hz
        # Below 20 cm the sensor sees its own housing; in the costmap those points would become a ring of obstacles
        # around the robot.
        "range_min": 0.2,
        "range_max": 100.0,
        "use_inf": True,
        "inf_epsilon": 1.0,
    }
