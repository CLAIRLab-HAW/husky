"""The patcher for the a200's look: right wheel visuals turned by pi about z, the chassis on the recolored mesh.

The wheel fixture is a COPY of ``urdf/a200/drivetrain/wheels/indoor.urdf.xacro`` as ``clearpath_platform_description``
2.9.15 ships it, read from the plant-mock container's apt copy on 2026-10-09.  ``outdoor.urdf.xacro`` is the same file
with the variant name swapped (and one blank line moved, compared the same day), so it is derived rather than copied
a second time.  The chassis fixture is ``urdf/a200/a200.urdf.xacro`` from the same copy's ``.bak`` (the pristine one;
``ride_height_patch`` edits the live file), verbatim down to the chassis joint, with the macro and the robot closed.
Both are embedded rather than read from the workspace bundle under ``urdf/``: that bundle is re-synced from a
container in which this very patcher has already run, and a fixture that is already patched tests nothing.

What is worth testing is not that a string replace replaces a string.  It is that the VISUAL origin moves and the
collision and inertial origins next to it do not, that only the chassis link's mesh changes and the base link's next to
it does not, that a second run and the ``.bak`` behave, that all three files are covered, and -- the one that matters
most -- that an upstream that renamed a file or rewrote the line is reported with a nonzero exit instead of crashing
or passing silently.
"""

from __future__ import annotations

import difflib
import shutil
import xml.etree.ElementTree as ET

import a200_look_patch as patch
import pytest

#: Verbatim, long inertia lines included -- hence the E501 waiver on the closing quotes.
INDOOR = """<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="a200_indoor_wheel">

  <xacro:property name="a200_indoor_wheel_radius" value="0.1651" />
  <xacro:property name="a200_indoor_wheel_mass" value="2.637" />
  <xacro:property name="a200_indoor_wheel_width" value="0.1143" />

  <xacro:macro name="a200_indoor_wheel" params="prefix *joint_pose">
    <link name="${prefix}_wheel_link">
      <inertial>
        <mass value="${a200_indoor_wheel_mass}" />
        <origin xyz="0 0 0" />
        <inertia
          ixx="${ a200_indoor_wheel_mass/12 * (3*a200_indoor_wheel_radius**2 + a200_indoor_wheel_width**2) }" ixy="0" ixz="0"
          iyy="${ a200_indoor_wheel_mass/2 * a200_indoor_wheel_radius**2 }" iyz="0"
          izz="${ a200_indoor_wheel_mass/12 * (3*a200_indoor_wheel_radius**2 + a200_indoor_wheel_width**2) }" />
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <mesh filename="package://clearpath_platform_description/meshes/a200/wheels/indoor.dae" />
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0" />
        <geometry>
          <cylinder length="${a200_indoor_wheel_width}" radius="${a200_indoor_wheel_radius}" />
        </geometry>
      </collision>
    </link>

    <gazebo>
      <plugin filename="libgz-sim-wheel-slip-system.so" name="gz::sim::systems::WheelSlip">
        <wheel link_name="${prefix}_wheel_link">
          <slip_compliance_lateral>1.0</slip_compliance_lateral>
          <slip_compliance_longitudinal>1.0</slip_compliance_longitudinal>
          <wheel_normal_force>113</wheel_normal_force>
          <wheel_radius>${a200_indoor_wheel_radius}</wheel_radius>
        </wheel>
      </plugin>
    </gazebo>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel_link"/>
      <xacro:insert_block name="joint_pose"/>
      <axis xyz="0 1 0" rpy="0 0 0" />
    </joint>

  </xacro:macro>
</robot>
"""  # noqa: E501

STOCK = {"indoor": INDOOR, "outdoor": INDOOR.replace("indoor", "outdoor")}

#: ``a200.urdf.xacro`` verbatim down to the chassis joint, then the macro and the robot closed.
CHASSIS = """<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:arg name="namespace" default="" />
  <xacro:arg name="is_sim" default="false" />
  <xacro:arg name="serial_port" default="/dev/clearpath/prolific" />
  <xacro:arg name="gazebo_controllers" default="$(find clearpath_control)/config/a200/control.yaml"/>
  <xacro:arg name="use_platform_controllers" default="true"/>

  <xacro:property name="M_PI" value="3.14159"/>

  <xacro:macro name="a200" params="control:='diff_4wd' front_wheels:='outdoor' rear_wheels:='outdoor'">

    <!-- Base Size -->
    <xacro:property name="base_x_size" value="0.98740000" />
    <xacro:property name="base_y_size" value="0.57090000" />
    <xacro:property name="base_z_size" value="0.24750000" />

    <!-- Wheel Mounting Positions -->
    <xacro:property name="wheel_base" value="0.5120" />
    <xacro:property name="wheel_track" value="0.555" />
    <xacro:property name="wheel_vertical_offset" value="0.03282" />

    <!-- Base link is the center of the robot's bottom plate -->
    <link name="base_link">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <mesh filename="package://clearpath_platform_description/meshes/a200/base_link.dae" />
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 ${base_z_size/4}" rpy="0 0 0" />
        <geometry>
          <box size="${base_x_size} ${base_y_size} ${base_z_size/2}"/>
        </geometry>
      </collision>
      <collision>
        <origin xyz="0 0 ${base_z_size*3/4-0.01}" rpy="0 0 0" />
        <geometry>
          <box size="${base_x_size*4/5} ${base_y_size} ${base_z_size/2-0.02}"/>
        </geometry>
      </collision>
    </link>

    <!-- Spawn A200 chassis -->
    <link name="top_chassis_link">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <mesh filename="package://clearpath_platform_description/meshes/a200/top_chassis.dae" />
        </geometry>
      </visual>
    </link>

    <!-- Attach chassis to the robot -->
    <joint name="top_chassis_joint" type="fixed">
      <parent link="base_link" />
      <child link="top_chassis_link" />
    </joint>
  </xacro:macro>
</robot>
"""


@pytest.fixture()
def share(tmp_path):
    """A fake ``/opt/ros/<distro>/share`` with copies of both wheel macros and the a200 macro; yields the glob and the
    package directory."""
    package = tmp_path / "jazzy" / "share" / patch.PACKAGE
    for variant in patch.VARIANTS:
        macro = package / patch.XACRO_REL.format(variant=variant)
        macro.parent.mkdir(parents=True, exist_ok=True)
        macro.write_text(STOCK[variant])
    chassis = package / patch.CHASSIS_REL
    chassis.parent.mkdir(parents=True, exist_ok=True)
    chassis.write_text(CHASSIS)
    return str(tmp_path / "*" / "share"), package


def _files(package):
    """Every file the patcher targets in ``package``, mapped to its stock text."""
    stock = {package / patch.XACRO_REL.format(variant=v): STOCK[v] for v in patch.VARIANTS}
    return {**stock, package / patch.CHASSIS_REL: CHASSIS}


def _changed_lines(before: str, after: str) -> list[str]:
    """The ``-``/``+`` lines of a zero-context diff, prefix kept."""
    return [
        line
        for line in difflib.unified_diff(before.splitlines(), after.splitlines(), n=0, lineterm="")
        if line.startswith(("+", "-")) and not line.startswith(("+++", "---"))
    ]


def _visual_origin(text: str) -> ET.Element:
    return ET.fromstring(text).find(".//link/visual/origin")


@pytest.mark.parametrize("variant", patch.VARIANTS)
def test_the_visual_origin_turns_in_both_variants(share, variant):
    glob_, package = share
    assert patch.main([f"--ros-share={glob_}"]) == 0
    text = (package / patch.XACRO_REL.format(variant=variant)).read_text()
    assert _visual_origin(text).get("rpy") == "0 0 ${pi if 'right' in prefix else 0}"
    assert patch.MARKER in text


@pytest.mark.parametrize("variant", patch.VARIANTS)
def test_only_the_visual_origin_changes(variant):
    """Joint, axis, collision and inertial stay as upstream ships them: two lines in (marker and origin), one out."""
    stock = STOCK[variant]
    out, status = patch.patch_wheel_text(stock)
    assert status == "changed"
    changed = _changed_lines(stock, out)
    assert [line[1:].strip() for line in changed] == [patch.ORIGIN_OLD, patch.MARKER, patch.ORIGIN_NEW], changed
    assert '<origin xyz="0 0 0" rpy="${M_PI/2} 0 0" />' in out


def test_the_chassis_visual_takes_the_recolored_mesh(share):
    glob_, package = share
    assert patch.main([f"--ros-share={glob_}"]) == 0
    text = (package / patch.CHASSIS_REL).read_text()
    link = ET.fromstring(text).find(".//link[@name='top_chassis_link']")
    assert link.find("visual/geometry/mesh").get("filename") == (
        "package://husky_extras_description/meshes/a200_top_chassis.dae"
    )
    assert patch.MARKER in text


def test_only_the_chassis_mesh_changes():
    """The base link's mesh, the origins and the joint stay: the marker and the mesh line in, one mesh line out."""
    out, status = patch.patch_chassis_text(CHASSIS)
    assert status == "changed"
    changed = _changed_lines(CHASSIS, out)
    assert [line[1:].strip() for line in changed] == [
        patch.CHASSIS_MESH_OLD,
        patch.MARKER,
        patch.CHASSIS_MESH_NEW,
    ], changed
    assert "package://clearpath_platform_description/meshes/a200/base_link.dae" in out


def test_a_second_run_changes_nothing(share):
    glob_, package = share
    patch.main([f"--ros-share={glob_}"])
    once = {path: path.read_text() for path in _files(package)}
    assert patch.main([f"--ros-share={glob_}"]) == 0
    assert {path: path.read_text() for path in _files(package)} == once


def test_the_backup_keeps_the_pristine_apt_copy(share):
    """A second run must not overwrite the backup with an already-patched file."""
    glob_, package = share
    patch.main([f"--ros-share={glob_}"])
    patch.main([f"--ros-share={glob_}"])
    for path, stock in _files(package).items():
        assert path.with_name(path.name + ".bak").read_text() == stock


def test_a_rewritten_chassis_mesh_is_reported_not_crashed(share, capsys):
    """Upstream pointing the chassis at another mesh: the anchor no longer matches, and the tool says so."""
    glob_, package = share
    target = package / patch.CHASSIS_REL
    rewritten = CHASSIS.replace("meshes/a200/top_chassis.dae", "meshes/a200/top_chassis_v2.dae")
    target.write_text(rewritten)
    assert patch.main([f"--ros-share={glob_}"]) == 1
    assert "0 chassis visual mesh(es)" in capsys.readouterr().err
    assert target.read_text() == rewritten
    assert not target.with_name(target.name + ".bak").exists()
    assert patch.MARKER in (package / patch.XACRO_REL.format(variant="indoor")).read_text()


def test_the_chassis_mesh_of_another_link_is_left_alone():
    """The link name is part of the anchor: the same mesh under another link is not the chassis visual."""
    moved = CHASSIS.replace('<link name="top_chassis_link">', '<link name="chassis_shell_link">')
    _, status = patch.patch_chassis_text(moved)
    assert status.startswith("0 chassis visual mesh(es)")


def test_a_renamed_macro_is_reported_not_crashed(share, capsys):
    """Upstream renaming one variant must not stop the others from being patched, and must not pass silently."""
    glob_, package = share
    macro = package / patch.XACRO_REL.format(variant="indoor")
    shutil.move(macro, macro.with_name("indoor_v2.urdf.xacro"))
    assert patch.main([f"--ros-share={glob_}"]) == 1
    assert "no indoor.urdf.xacro" in capsys.readouterr().err
    assert patch.MARKER in (package / patch.XACRO_REL.format(variant="outdoor")).read_text()
    assert patch.CHASSIS_MESH_NEW in (package / patch.CHASSIS_REL).read_text()


def test_a_rewritten_origin_line_is_reported_not_crashed(share, capsys):
    """Also the shape an upstream fix would take: the anchor no longer matches, and the tool says so."""
    glob_, package = share
    target = package / patch.XACRO_REL.format(variant="indoor")
    rewritten = target.read_text().replace(patch.ORIGIN_OLD, '<origin xyz="0 0 0" rpy="0 0 ${reflect}" />', 1)
    target.write_text(rewritten)
    assert patch.main([f"--ros-share={glob_}"]) == 1
    assert "0 visual origin(s)" in capsys.readouterr().err
    assert target.read_text() == rewritten
    assert not target.with_suffix(".xacro.bak").exists()


def test_a_missing_package_fails_loudly(tmp_path):
    assert patch.main([f"--ros-share={tmp_path}/*/share"]) == 1


def test_the_dry_run_writes_nothing(share):
    glob_, package = share
    assert patch.main([f"--ros-share={glob_}", "--dry-run"]) == 0
    for path, stock in _files(package).items():
        assert path.read_text() == stock
        assert not path.with_name(path.name + ".bak").exists()
