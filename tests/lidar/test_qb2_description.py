"""The Qb2's macro: the frame its driver stamps, turned so the points land where the sensor looks.

Reads the xacro as XML -- the Mac has no xacro and needs none for this.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from husky_lidar import wiring

PACKAGE = Path(__file__).resolve().parents[2] / "src" / "qb2_description"
MACRO = ET.parse(PACKAGE / "urdf" / "qb2.urdf.xacro").getroot()[0]


def _joint(name: str) -> ET.Element:
    return next(j for j in MACRO.findall("joint") if j.get("name") == name)


def test_the_macro_builds_the_rs16s_names():
    """The Qb2 stands in the RS16's slot: same link names, so TF, wiring and the self filter need no second set."""
    names = [link.get("name").replace("${name}", "lidar3d_0") for link in MACRO.findall("link")]
    assert names == ["lidar3d_0_link", wiring.LIDAR_FRAME]


def test_the_laser_frame_is_the_devices_own():
    """The driver hands on x right, y ahead: -90° about z puts its +y on the link's +x, the boresight."""
    origin = _joint("${name}_laser_joint").find("origin")
    assert origin.get("rpy") == "0 0 ${-pi / 2}"
    yaw = -math.pi / 2
    assert (round(-math.sin(yaw)), round(math.cos(yaw))) == (1, 0), "the laser's +y is the link's +x"


def test_visual_and_collision_are_the_same_box():
    link = MACRO.find("link")
    assert link.find("visual/geometry/box").get("size") == link.find("collision/geometry/box").get("size")


def test_the_package_installs_the_macro():
    assert "DIRECTORY urdf" in (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
