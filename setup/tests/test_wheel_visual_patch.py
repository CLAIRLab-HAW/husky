"""The patcher that turns the a200's right wheel visuals by pi about z.

The fixture is a COPY of ``urdf/a200/drivetrain/wheels/indoor.urdf.xacro`` as ``clearpath_platform_description``
2.9.15 ships it, read from the plant-mock container's apt copy on 2026-10-09.  ``outdoor.urdf.xacro`` is the same file
with the variant name swapped (and one blank line moved, compared the same day), so it is derived rather than copied
a second time.  Embedded rather than read from the workspace bundle under ``urdf/``: that bundle is re-synced from a
container in which this very patcher has already run, and a fixture that is already patched tests nothing.

What is worth testing is not that a string replace replaces a string.  It is that the VISUAL origin moves and the
collision and inertial origins next to it do not, that a second run and the ``.bak`` behave, that both variants are
covered, and -- the one that matters most -- that an upstream that renamed a macro or rewrote the line is reported
with a nonzero exit instead of crashing or passing silently.
"""

from __future__ import annotations

import difflib
import shutil
import xml.etree.ElementTree as ET

import pytest
import wheel_visual_patch as patch

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


@pytest.fixture()
def share(tmp_path):
    """A fake ``/opt/ros/<distro>/share`` holding copies of both real wheel macros; yields the glob and the dir."""
    wheels = tmp_path / "jazzy" / "share" / patch.PACKAGE / "urdf" / "a200" / "drivetrain" / "wheels"
    wheels.mkdir(parents=True)
    for variant in patch.VARIANTS:
        (wheels / f"{variant}.urdf.xacro").write_text(STOCK[variant])
    return str(tmp_path / "*" / "share"), wheels


def _visual_origin(text: str) -> ET.Element:
    return ET.fromstring(text).find(".//link/visual/origin")


@pytest.mark.parametrize("variant", patch.VARIANTS)
def test_the_visual_origin_turns_in_both_variants(share, variant):
    glob_, wheels = share
    assert patch.main([f"--ros-share={glob_}"]) == 0
    text = (wheels / f"{variant}.urdf.xacro").read_text()
    assert _visual_origin(text).get("rpy") == "0 0 ${pi if 'right' in prefix else 0}"
    assert patch.MARKER in text


@pytest.mark.parametrize("variant", patch.VARIANTS)
def test_only_the_visual_origin_changes(variant):
    """Joint, axis, collision and inertial stay as upstream ships them: two lines in (marker and origin), one out."""
    stock = STOCK[variant]
    out, status = patch.patch_text(stock)
    assert status == "changed"
    changed = [
        line
        for line in difflib.unified_diff(stock.splitlines(), out.splitlines(), n=0, lineterm="")
        if line.startswith(("+", "-")) and not line.startswith(("+++", "---"))
    ]
    assert len(changed) == 3, f"the edit is not confined to the visual origin: {changed}"
    assert [line[1:].strip() for line in changed] == [patch.ORIGIN_OLD, patch.MARKER, patch.ORIGIN_NEW]
    assert '<origin xyz="0 0 0" rpy="${M_PI/2} 0 0" />' in out


def test_a_second_run_changes_nothing(share):
    glob_, wheels = share
    patch.main([f"--ros-share={glob_}"])
    once = {p.name: p.read_text() for p in wheels.glob("*.urdf.xacro")}
    assert patch.main([f"--ros-share={glob_}"]) == 0
    assert {p.name: p.read_text() for p in wheels.glob("*.urdf.xacro")} == once


def test_the_backup_keeps_the_pristine_apt_copy(share):
    """A second run must not overwrite the backup with an already-patched file."""
    glob_, wheels = share
    patch.main([f"--ros-share={glob_}"])
    patch.main([f"--ros-share={glob_}"])
    for variant in patch.VARIANTS:
        assert (wheels / f"{variant}.urdf.xacro.bak").read_text() == STOCK[variant]


def test_a_renamed_macro_is_reported_not_crashed(share, capsys):
    """Upstream renaming one variant must not stop the other from being patched, and must not pass silently."""
    glob_, wheels = share
    shutil.move(wheels / "indoor.urdf.xacro", wheels / "indoor_v2.urdf.xacro")
    assert patch.main([f"--ros-share={glob_}"]) == 1
    assert "no indoor wheel macro" in capsys.readouterr().err
    assert patch.MARKER in (wheels / "outdoor.urdf.xacro").read_text()


def test_a_rewritten_origin_line_is_reported_not_crashed(share, capsys):
    """Also the shape an upstream fix would take: the anchor no longer matches, and the tool says so."""
    glob_, wheels = share
    target = wheels / "indoor.urdf.xacro"
    rewritten = target.read_text().replace(patch.ORIGIN_OLD, '<origin xyz="0 0 0" rpy="0 0 ${reflect}" />', 1)
    target.write_text(rewritten)
    assert patch.main([f"--ros-share={glob_}"]) == 1
    assert "0 visual origin(s)" in capsys.readouterr().err
    assert target.read_text() == rewritten
    assert not target.with_suffix(".xacro.bak").exists()


def test_a_missing_package_fails_loudly(tmp_path):
    assert patch.main([f"--ros-share={tmp_path}/*/share"]) == 1


def test_the_dry_run_writes_nothing(share):
    glob_, wheels = share
    assert patch.main([f"--ros-share={glob_}", "--dry-run"]) == 0
    for variant in patch.VARIANTS:
        assert (wheels / f"{variant}.urdf.xacro").read_text() == STOCK[variant]
