"""Topic and frame names of the RS16 -- the single source.

ROS-free.  ``launch/lidar.launch.py`` and ``config/rslidar_rs16.yaml`` take them from here, a test holds both to not
stating them a second time, and ``clair.navigation.wiring`` builds its scan on them.
"""

from __future__ import annotations

#: Namespace of the a200-0553.  The whole graph hangs off it.
NAMESPACE = "a200_0553"

#: The frame rs16_description's macro puts at the optical center (``lidar3d_0_laser`` on the a200-0553) and that the
#: driver writes into every PointCloud2.  If the two diverge, tf2 does not find the cloud and does not report it.
LIDAR_FRAME = "lidar3d_0_laser"

#: tf2 broadcasts on the ABSOLUTE names; the node namespace does not take
#: effect there.  Without these remaps a node publishes globally while the
#: rest of the graph listens on /a200_0553/tf -- an empty TF chain without an
#: error.
TF_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]


def points_topic() -> str:
    """Raw point cloud of the RS16 (sensor_msgs/PointCloud2)."""
    return f"/{NAMESPACE}/sensors/lidar3d_0/points"


def filtered_points_topic() -> str:
    """The RS16's points without the robot's own body (:mod:`clair.rs16.self_filter`): what the costmaps and the scan
    read, so neither takes the sensor arch or an arm reaching out for an obstacle."""
    return f"/{NAMESPACE}/sensors/lidar3d_0/points_filtered"


def description_topic() -> str:
    """The robot's whole description (std_msgs/String, transient local), as robot_state_publisher publishes it."""
    return f"/{NAMESPACE}/robot_description"
