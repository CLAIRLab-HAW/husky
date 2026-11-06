"""blickfeld_qb2_ros2_driver, Blickfeld's driver for the Qb2: its parameters, from one file and the wiring.

The driver is a composable node (``blickfeld::ros_interop::Qb2Driver``) that reads the device's point-cloud stream
over gRPC and publishes each frame as one PointCloud2 in the device's own coordinates (x right, y ahead, z up).  It
replays nothing: there is no recording to hand it, only a live device (or a rosbag of its topic).

ROS-free and therefore testable on the Mac.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from clair.lidar import wiring
from clair.lidar.models import LidarModel

TEMPLATE_PATH = Path(__file__).resolve().parents[3] / "config" / "blickfeld_qb2.yaml"

#: The driver's package and component, as its own launch file loads it (v2.1.0).
PACKAGE = "blickfeld_qb2_ros2_driver"
PLUGIN = "blickfeld::ros_interop::Qb2Driver"


def parameters(model: LidarModel) -> dict:
    """The driver's parameters for ``model``: the template's, plus the frame and topic the wiring names."""
    if model.driver != "blickfeld_qb2":
        raise ValueError(f"{model.name} is driven by {model.driver}, not by {PACKAGE}")
    template = yaml.safe_load(TEMPLATE_PATH.read_text(encoding="utf-8"))
    if not template.get("fqdn"):
        raise ValueError(f"{TEMPLATE_PATH.name} names no fqdn -- the driver would wait for a device it cannot reach")
    return {**template, "frame_id": wiring.LIDAR_FRAME, "point_cloud_topic": wiring.points_topic()}
