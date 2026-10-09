"""The macro states the manual's figures, and the frame the driver writes into is the one it builds.

Reads the xacro as XML -- the Mac has no xacro and needs none for this.  That the macro EXPANDS, inside the robot's
whole description, is the container's to show (the generator dies on a broken include).
"""

import xml.etree.ElementTree as ET
from pathlib import Path

from clair.rs16 import wiring

PACKAGE = Path(__file__).resolve().parents[1] / "src" / "rs16_description"
MACRO = ET.parse(PACKAGE / "urdf" / "rs16.urdf.xacro").getroot()[0]


def _joint(name: str) -> ET.Element:
    return next(j for j in MACRO.findall("joint") if j.get("name") == name)


def test_the_laser_frame_is_the_driver_frame():
    """``rs16(name="lidar3d_0")`` builds ``lidar3d_0_laser``, the frame rslidar_sdk stamps every cloud with."""
    names = [link.get("name").replace("${name}", "lidar3d_0") for link in MACRO.findall("link")]
    assert names == ["lidar3d_0_link", wiring.LIDAR_FRAME]


def test_the_optical_center_sits_39_mm_above_the_mounting_face():
    """RS-LiDAR-16 user manual §8.1, note 2."""
    assert _joint("${name}_laser_joint").find("origin").get("xyz") == "0 0 0.039"


def test_the_visual_mesh_is_in_the_package():
    uri = MACRO.find("link/visual/geometry/mesh").get("filename")
    assert uri == "package://rs16_description/meshes/rs16.dae"
    assert (PACKAGE / "meshes" / "rs16.dae").is_file()


def test_the_visual_carries_no_material():
    """A ``<material>`` would paint over the four colors the DAE carries."""
    assert MACRO.find("link/visual/material") is None


def test_the_collision_is_the_housing():
    """Ø109 x 80.7 mm (§3), standing on the mounting face."""
    collision = MACRO.find("link/collision")
    cylinder = collision.find("geometry/cylinder")
    assert (cylinder.get("radius"), cylinder.get("length")) == ("0.0545", "0.0807")
    assert collision.find("origin").get("xyz") == "0 0 0.04035"


def test_the_mass_is_the_datasheets():
    """0.87 kg without cable (§3)."""
    assert MACRO.find("link/inertial/mass").get("value") == "0.87"
