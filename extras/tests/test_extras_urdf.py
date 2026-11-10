"""The a200-0553's URDF extras: does the file still say what the rest of the workspace assumes it says?

There is nothing to execute here -- the package is data -- so what the tests guard is the seam, and a seam is exactly
what a move breaks:

* an XML comment cannot contain ``--``, and a file that is not well-formed takes the WHOLE stack down with it
  (``mock`` dies in ``_process_urdf``, ``move_group`` never comes up).  That has happened once already, in
  ``onrobot-rg6`` on 2026-08-30, and it stayed invisible until an image rebuild;
* every ``package://`` URI has to name the package that actually ships the file, otherwise RViz and the
  foxglove_bridge draw a robot with holes in it and nothing errors;
* ``robot.yaml`` (``setup/``) addresses this file by ABSOLUTE path and lists the workspaces it is installed from.
  If either stops matching, the next boot generates a robot without extras.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
PACKAGE = "husky_extras_description"
EXTRAS = REPO / "src" / PACKAGE / "urdf" / "clearpath_extras.urdf.xacro"
#: The end effectors ``end_effector:=<name>`` selects, one file each.
TOOLS = sorted((EXTRAS.parent / "tools").glob("*.urdf.xacro"))
#: The robot contract's gripper models, one file each -- in the neighboring repo, robot/.
GRIPPER_MODELS = REPO.parents[1] / "robot/contract/src/clair/robot/contract/profiles/grippers"
#: The lidars ``lidar_model:=<name>`` selects, one file each -- the robot carries one of them.
LIDARS = sorted((EXTRAS.parent / "lidars").glob("*.urdf.xacro"))
#: Every xacro file of every package here -- tools, the macros they call, the other robots' descriptions.
XACROS = sorted((REPO / "src").rglob("*.xacro"))

#: The absolute path robot.yaml addresses this file by, and the workspaces it sources: this one and the tools
#: repository's, whose macros the tool files include.  The robot's own layout (/home/robot/husky/<dir>,
#: /home/robot/tools/<dir>), which the offboard container reproduces with a symlink.
ROBOT_EXTRAS_PATH = f"/home/robot/husky/extras/src/{PACKAGE}/urdf/clearpath_extras.urdf.xacro"
ROBOT_WORKSPACES = ("/home/robot/husky/extras/install/setup.bash", "/home/robot/tools/description/install/setup.bash")
#: robot.yaml, in this repository's setup/.
ROBOT_YAML = REPO.parent / "setup" / "config" / "robot.yaml"


@pytest.fixture(scope="session")
def extras_text() -> str:
    return EXTRAS.read_text(encoding="utf-8")


def _tool(name: str) -> str:
    return (EXTRAS.parent / "tools" / f"{name}.urdf.xacro").read_text(encoding="utf-8")


@pytest.mark.parametrize("path", XACROS, ids=lambda p: p.name)
def test_every_file_is_well_formed_xml(path):
    """The `--` trap included: ElementTree rejects it exactly as expat does inside xacro."""
    ET.fromstring(path.read_text(encoding="utf-8"))


@pytest.mark.parametrize("path", XACROS, ids=lambda p: p.name)
def test_every_mesh_uri_names_a_file_this_package_ships(path):
    """A package:// URI that points at a package which does not carry the file fails silently, in the viewer."""
    own = path.relative_to(REPO / "src").parts[0]
    for package, relpath in re.findall(r'filename="package://([^/]+)/([^"]+)"', path.read_text(encoding="utf-8")):
        assert package == own, f"package://{package} is not {own}; who installs {relpath}?"
        assert (REPO / "src" / package / relpath).is_file(), f"package://{package}/{relpath} does not exist here"


def test_the_end_effector_is_an_argument_whose_default_is_the_rg6(extras_text):
    """The robot's own hand unless the offboard mock asks for another; every name has its file."""
    assert '<xacro:arg name="end_effector" default="rg6"/>' in extras_text
    assert '<xacro:include filename="tools/$(arg end_effector).urdf.xacro"/>' in extras_text
    assert {p.name for p in TOOLS} >= {"rg6.urdf.xacro", "key_holder.urdf.xacro"}


@pytest.mark.parametrize("tool", ["rg6", "key_holder"])
def test_the_tool_macros_are_included_from_the_package_that_owns_them(tool):
    """The tools repository's macros, and it must run this way round: assembly includes component."""
    assert f"$(find tool_description)/urdf/{tool}.macro.xacro" in _tool(tool)
    manifest = (REPO / "src" / PACKAGE / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>tool_description</exec_depend>" in manifest, (
        "the include is there but the dependency is not declared -- colcon and rosdep cannot see it"
    )


def test_the_platform_keeps_the_links_the_stack_addresses_by_name(extras_text):
    assert '<link name="husky_top_assembly"' in extras_text


def test_the_platform_carries_no_aruco_marker(extras_text):
    """The marker was never surveyed (its frame and offset were placed); it left the model rather than mislead."""
    assert "aruco" not in extras_text.lower()


@pytest.mark.parametrize("path", TOOLS, ids=lambda p: p.name)
def test_every_tool_carries_the_link_robot_yaml_hangs_the_camera_on(path):
    """robot.yaml names one camera parent for every tool; a tool without it leaves the camera unattached."""
    assert '<link name="rg6_onrobot_rg6_base_link"' in path.read_text(encoding="utf-8")


def test_the_arch_carries_collision_geometry_and_a_mass(extras_text):
    """R15 and R47 in one link: without the boxes move_group plans through the arch, without the mass it weighs 0.1
    kg."""
    arch = extras_text.split('<link name="husky_top_assembly">', 1)[1].split("</link>", 1)[0]
    assert arch.count("<collision>") == 5, "the five boxes along the real structure (the RS16 is rs16_description's)"
    assert "<inertial>" in arch and 'value="6.21"' in arch


def test_the_charging_connector_hangs_on_the_top_chassis_as_a_visual_alone(extras_text):
    """A collision box there would touch the chassis' and invalidate every MoveIt state unless the SRDF disabled it."""
    link = extras_text.split('<link name="charging_connector">', 1)[1].split("</link>", 1)[0]
    assert "<visual>" in link and "<collision>" not in link
    joint = extras_text.split('<joint name="charging_connector_joint" type="fixed">', 1)[1].split("</joint>", 1)[0]
    assert '<parent link="top_chassis_link"/>' in joint


def test_the_rg6_cable_hangs_on_the_flange_as_a_visual_alone():
    """Its mesh is written in arm_0_tool0; a collision body along the wrist would need SRDF disables for every pair."""
    tool = _tool("rg6")
    link = tool.split('<link name="rg6_cable">', 1)[1].split("</link>", 1)[0]
    assert "<visual>" in link and "<collision>" not in link
    assert "package://husky_extras_description/meshes/rg6_cable.dae" in link
    joint = tool.split('<joint name="rg6_cable_joint" type="fixed">', 1)[1].split("</joint>", 1)[0]
    assert '<parent link="arm_0_tool0"/>' in joint
    assert '<origin xyz="0 0 0" rpy="0 0 0"/>' in joint


def test_the_package_installs_the_two_directories_the_uris_resolve_through():
    cmake = (REPO / "src" / PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "DIRECTORY urdf meshes" in cmake
    assert "DESTINATION share/${PROJECT_NAME}" in cmake


def test_robot_yaml_addresses_this_file_and_sources_this_workspace():
    """The SSOT in setup/, and the seam this move actually turns on."""
    import yaml

    robot_yaml = yaml.safe_load(ROBOT_YAML.read_text(encoding="utf-8"))
    assert robot_yaml["platform"]["extras"]["urdf"]["path"] == ROBOT_EXTRAS_PATH
    workspaces = robot_yaml["system"]["ros2"]["workspaces"]
    missing = [w for w in ROBOT_WORKSPACES if w not in workspaces]
    assert not missing, (
        f"{missing} not in system.ros2.workspaces -- the generator would find the file and then fail "
        f"on $(find tool_description) resp. leave package://{PACKAGE} unresolvable"
    )


def test_the_lidar_is_an_argument_whose_default_is_robot_yamls(extras_text):
    """Clearpath's generator passes this file no arguments, so on the robot the default IS the lidar: it has to be the
    model robot.yaml starts the driver for, or the robot is drawn with one sensor while it reads the other."""
    import re

    import yaml

    default = re.search(r'<xacro:arg name="lidar_model" default="([^"]+)"/>', extras_text)
    assert default, "lidar_model is no argument with a default"
    assert '<xacro:include filename="lidars/$(arg lidar_model).urdf.xacro"/>' in extras_text
    robot_yaml = yaml.safe_load(ROBOT_YAML.read_text(encoding="utf-8"))
    named = [
        launch["args"]["lidar_model"]
        for launch in robot_yaml["platform"]["extras"]["launch"]
        if "lidar_model" in (launch.get("args") or {})
    ]
    assert named == [default.group(1)], f"robot.yaml starts {named}, the description draws {default.group(1)}"


def test_every_lidar_model_has_a_file_and_every_file_a_model():
    """One file per model of clair.lidar.models, no more: an unknown lidar_model fails on its missing file, by name."""
    models = pytest.importorskip("clair.lidar.models")
    assert {p.name.removesuffix(".urdf.xacro") for p in LIDARS} == set(models.MODELS)


@pytest.mark.parametrize("path", LIDARS, ids=lambda p: p.name)
def test_every_lidar_hangs_on_the_one_slot(path):
    text = path.read_text(encoding="utf-8")
    assert 'name="lidar3d_0" parent_link="lidar3d_0_mount_link"' in text


BRINGUP = REPO / "src" / "husky_extras_bringup"


def test_the_bringup_installs_its_launch_directory():
    """The generated extras launch finds the file through ``FindPackageShare``, so it has to be in the share."""
    cmake = (BRINGUP / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "DIRECTORY launch" in cmake
    assert "DESTINATION share/${PROJECT_NAME}" in cmake


def test_robot_yaml_starts_a_launch_file_the_bringup_ships():
    """Every ``platform.extras.launch`` entry names a package -- clearpath_generator_robot 2.9.3 dies on
    ``'pkg_' + None`` without one -- and the bringup's entry a file it installs: a wrong path fails at the robot's boot,
    in clearpath-platform-extras, and nowhere before."""
    import yaml

    robot_yaml = yaml.safe_load(ROBOT_YAML.read_text(encoding="utf-8"))
    launches = robot_yaml["platform"]["extras"]["launch"]
    assert all(launch.get("package") for launch in launches), launches
    ours = [launch for launch in launches if launch["package"] == BRINGUP.name]
    assert ours, f"robot.yaml starts nothing of {BRINGUP.name}"
    for launch in ours:
        assert (BRINGUP / launch["path"]).is_file(), launch["path"]


def test_robot_yaml_hands_its_own_namespace_to_the_bringup():
    """The generator passes an extras launch no namespace; robot.yaml states it as an argument, which has to be the
    one the rest of the robot runs under, or bt_joy/open_pairing appears where Echo does not look."""
    import yaml

    robot_yaml = yaml.safe_load(ROBOT_YAML.read_text(encoding="utf-8"))
    stated = [
        launch["args"]["namespace"]
        for launch in robot_yaml["platform"]["extras"]["launch"]
        if "namespace" in (launch.get("args") or {})
    ]
    assert stated, "no extras launch takes the namespace -- bt_joy.launch.py needs it"
    assert set(stated) == {robot_yaml["system"]["ros2"]["namespace"]}


@pytest.mark.skipif(not GRIPPER_MODELS.is_dir(), reason="robot/contract is not checked out beside husky/")
def test_every_gripper_model_has_the_tool_file_of_its_name():
    """A tool named after a model is that hand: the mock, the SRDF and the stack's logic find the model by the tool's
    name, so a model without its tool file could never be put on the flange."""
    models = {p.stem for p in GRIPPER_MODELS.glob("*.yaml")}
    assert models and models <= {p.name.removesuffix(".urdf.xacro") for p in TOOLS}
