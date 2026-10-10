"""The patcher for the a200's look: right wheel visuals turned, wheels, chassis and D435 on this robot's meshes, and
the D435's collision box moved back onto its visual body.

The wheel fixture is a COPY of ``urdf/a200/drivetrain/wheels/indoor.urdf.xacro`` as ``clearpath_platform_description``
2.9.15 ships it, read from the plant-mock container's apt copy on 2026-10-09.  ``outdoor.urdf.xacro`` is the same file
with the variant name swapped (and one blank line moved, compared the same day), so it is derived rather than copied
a second time.  The chassis fixture is ``urdf/a200/a200.urdf.xacro`` from the same copy's ``.bak`` (the pristine one;
``ride_height_patch`` edits the live file), verbatim down to the chassis joint, with the macro and the robot closed.
Both are embedded rather than read from the workspace bundle under ``urdf/``: that bundle is re-synced from a
container in which this very patcher has already run, and a fixture that is already patched tests nothing.

What is worth testing is not that a string replace replaces a string.  It is that the VISUAL origin moves and the
collision and inertial origins next to it do not, that only the indoor wheel's visual mesh changes and the outdoor
one's does not, that only the chassis link's mesh changes and the base link's next to it does not, that the D435 box
moves onto the measured visual body and the same origin line in the box visual next to it does not, that the D435
macro ends the same with ``sensor_mesh_uri_patch`` run before this patcher or after it, that a second run and the
``.bak`` behave, that all four files are covered, and -- the one that matters most -- that an upstream that renamed a
file or rewrote the line is reported with a nonzero exit instead of crashing or passing silently.
"""

from __future__ import annotations

import difflib
import re
import shutil
import xml.etree.ElementTree as ET

import a200_look_patch as patch
import pytest
import sensor_mesh_uri_patch as sensor

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


#: ``clearpath_sensors_description``'s ``urdf/intel/d435.urdf.xacro`` verbatim down to the camera link's end, then the
#: macro and the robot closed -- from the plant-mock container's ``.bak``, the pristine apt copy with ``file://``
#: (``sensor_mesh_uri_patch`` rewrites the live file at build time), 2.9.15, read on 2026-10-10.
D435 = """<?xml version="1.0"?>
<robot name="sensor_d435" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- Includes -->
  <xacro:include filename="$(find realsense2_description)/urdf/_usb_plug.urdf.xacro" />

  <xacro:macro name="sensor_d435" params="parent *origin name:=camera use_nominal_extrinsics:=false add_plug:=false use_mesh:=true">
    <xacro:property name="M_PI" value="3.1415926535897931" />

    <!-- The following values are approximate, and the camera node
     publishing TF values with actual calibrated camera extrinsic values -->
    <xacro:property name="d435_cam_depth_to_infra1_offset" value="0.0"/>
    <xacro:property name="d435_cam_depth_to_infra2_offset" value="-0.050"/>
    <xacro:property name="d435_cam_depth_to_color_offset" value="0.015"/>

    <!-- The following values model the aluminum peripherial case for the
  	D435 camera, with the camera joint represented by the actual
  	peripherial camera tripod mount -->
    <xacro:property name="d435_cam_width" value="0.090"/>
    <xacro:property name="d435_cam_height" value="0.025"/>
    <xacro:property name="d435_cam_depth" value="0.02505"/>
    <xacro:property name="d435_cam_mount_from_center_offset" value="0.0149"/>
    <!-- glass cover is 0.1 mm inwards from front aluminium plate -->
    <xacro:property name="d435_glass_to_front" value="0.1e-3"/>
    <!-- see datasheet Revision 007, Fig. 4-4 page 65 -->
    <xacro:property name="d435_zero_depth_to_glass" value="4.2e-3"/>
    <!-- convenience precomputation to avoid clutter-->
    <xacro:property name="d435_mesh_x_offset" value="${d435_cam_mount_from_center_offset-d435_glass_to_front-d435_zero_depth_to_glass}"/>

    <!-- The following offset is relative the the physical D435 camera peripherial
  	camera tripod mount -->
    <xacro:property name="d435_cam_depth_px" value="${d435_cam_mount_from_center_offset}"/>
    <xacro:property name="d435_cam_depth_py" value="0.0175"/>
    <xacro:property name="d435_cam_depth_pz" value="${d435_cam_height/2}"/>

    <!-- camera body, with origin at bottom screw mount -->
    <joint name="${name}_joint" type="fixed">
      <xacro:insert_block name="origin" />
      <parent link="${parent}"/>
      <child link="${name}_bottom_screw_frame" />
    </joint>
    <link name="${name}_bottom_screw_frame"/>

    <joint name="${name}_link_joint" type="fixed">
      <origin xyz="${d435_mesh_x_offset} ${d435_cam_depth_py} ${d435_cam_depth_pz}" rpy="0 0 0"/>
      <parent link="${name}_bottom_screw_frame"/>
      <child link="${name}_link" />
    </joint>

    <link name="${name}_link">
      <visual>
        <xacro:if value="${use_mesh}">
          <!-- the mesh origin is at front plate in between the two infrared camera axes -->
          <origin xyz="${d435_zero_depth_to_glass + d435_glass_to_front} ${-d435_cam_depth_py} 0" rpy="${M_PI/2} 0 ${M_PI/2}"/>
          <geometry>
            <mesh filename="file://$(find realsense2_description)/meshes/d435.dae" />
          </geometry>
        </xacro:if>
        <xacro:unless value="${use_mesh}">
          <origin xyz="0 ${-d435_cam_depth_py} 0" rpy="0 0 0"/>
          <geometry>
            <box size="${d435_cam_depth} ${d435_cam_width} ${d435_cam_height}"/>
          </geometry>
          <material name="clearpath_aluminum"/>
        </xacro:unless>
      </visual>
      <collision>
        <origin xyz="0 ${-d435_cam_depth_py} 0" rpy="0 0 0"/>
        <geometry>
          <box size="${d435_cam_depth} ${d435_cam_width} ${d435_cam_height}"/>
        </geometry>
      </collision>
      <inertial>
        <!-- The following are not reliable values, and should not be used for modeling -->
        <mass value="0.072" />
        <origin xyz="0 0 0" />
        <inertia ixx="0.003881243" ixy="0.0" ixz="0.0" iyy="0.000498940" iyz="0.0" izz="0.003879257" />
      </inertial>
    </link>
  </xacro:macro>
</robot>
"""  # noqa: E501


@pytest.fixture()
def share(tmp_path):
    """A fake ``/opt/ros/<distro>/share`` with copies of both wheel macros, the a200 macro and the D435 macro; yields
    the glob and the platform package's directory."""
    camera = tmp_path / "jazzy" / "share" / patch.SENSORS_PACKAGE / patch.D435_REL
    camera.parent.mkdir(parents=True)
    camera.write_text(D435)
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
    """Every file the patcher targets beside and in ``package``, mapped to its stock text."""
    stock = {package / patch.XACRO_REL.format(variant=v): STOCK[v] for v in patch.VARIANTS}
    return {**stock, package / patch.CHASSIS_REL: CHASSIS, _camera(package): D435}


def _camera(package):
    """The D435 macro in the share ``package`` lies in."""
    return package.parent / patch.SENSORS_PACKAGE / patch.D435_REL


def _camera_mesh(text: str) -> str:
    return ET.fromstring(text).find(".//link/visual/{*}if/geometry/mesh").get("filename")


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


def test_the_indoor_wheel_takes_the_real_rims(share):
    """The indoor macro's visual points at husky_extras_description's wheel; the outdoor one keeps Clearpath's."""
    glob_, package = share
    assert patch.main([f"--ros-share={glob_}"]) == 0
    meshes = {
        variant: ET.fromstring((package / patch.XACRO_REL.format(variant=variant)).read_text())
        .find(".//link/visual/geometry/mesh")
        .get("filename")
        for variant in patch.VARIANTS
    }
    assert meshes == {
        "indoor": "package://husky_extras_description/meshes/a200_indoor_wheel.dae",
        "outdoor": "package://clearpath_platform_description/meshes/a200/wheels/outdoor.dae",
    }


def test_only_the_wheel_mesh_changes():
    """The collision cylinder, the inertial and the visual origin stay: the marker and the mesh line in, one out."""
    out, status = patch.patch_wheel_mesh_text(INDOOR)
    assert status == "changed"
    changed = _changed_lines(INDOOR, out)
    assert [line[1:].strip() for line in changed] == [
        patch.WHEEL_MESH_OLD,
        patch.MARKER,
        patch.WHEEL_MESH_NEW,
    ], changed
    assert '<cylinder length="${a200_indoor_wheel_width}" radius="${a200_indoor_wheel_radius}" />' in out


def test_the_two_wheel_edits_do_not_depend_on_their_order():
    """The mesh swap matches with the origin turned or not, and the origin turn with the mesh swapped or not."""
    origin_first = patch.patch_wheel_mesh_text(patch.patch_wheel_text(INDOOR)[0])
    mesh_first = patch.patch_wheel_text(patch.patch_wheel_mesh_text(INDOOR)[0])
    assert origin_first == mesh_first
    assert origin_first[1] == "changed"


def test_a_rewritten_wheel_mesh_is_reported_and_the_origin_still_turns(share, capsys):
    """Upstream pointing the indoor wheel at another mesh: that edit fails loudly, the other one in the file holds."""
    glob_, package = share
    target = package / patch.XACRO_REL.format(variant="indoor")
    target.write_text(INDOOR.replace("wheels/indoor.dae", "wheels/indoor_v2.dae"))
    assert patch.main([f"--ros-share={glob_}"]) == 1
    assert "0 wheel visual mesh(es)" in capsys.readouterr().err
    text = target.read_text()
    assert "wheels/indoor_v2.dae" in text
    assert patch.ORIGIN_NEW in text


def test_the_d435_visual_takes_the_colored_mesh(share):
    glob_, package = share
    assert patch.main([f"--ros-share={glob_}"]) == 0
    text = _camera(package).read_text()
    assert _camera_mesh(text) == "package://husky_extras_description/meshes/d435.dae"
    assert patch.MARKER in text


def test_only_the_d435_mesh_changes():
    """The visual origin, the box of the ``use_mesh:=false`` branch, the collision and the inertial stay: the marker
    and the mesh line in, one mesh line out."""
    out, status = patch.patch_d435_text(D435)
    assert status == "changed"
    changed = _changed_lines(D435, out)
    assert [line[1:].strip() for line in changed] == [
        '<mesh filename="file://$(find realsense2_description)/meshes/d435.dae" />',
        patch.MARKER,
        patch.D435_MESH_NEW,
    ], changed
    assert 'rpy="${M_PI/2} 0 ${M_PI/2}"/>' in out


def test_the_d435_swap_takes_the_line_with_sensor_mesh_uri_patch_applied_or_not():
    """``sensor_mesh_uri_patch`` makes the same line ``package://``: either form ends in the same text."""
    rewritten = D435.replace(sensor.MESH_URI_OLD, sensor.MESH_URI_NEW)
    assert patch.patch_d435_text(rewritten) == patch.patch_d435_text(D435)


@pytest.mark.parametrize("first", ["sensor_mesh_uri_patch", "a200_look_patch"])
def test_the_two_patchers_compose_in_either_order(share, first):
    """Both rewrite the D435 macro: in either order the result is the same, the shared ``.bak`` is the pristine apt
    copy, and a second run of either changes nothing."""
    glob_, package = share
    camera = _camera(package)
    runs = [lambda: sensor.main([f"--ros-share={glob_}"]), lambda: patch.main([f"--ros-share={glob_}"])]
    for run in runs if first == "sensor_mesh_uri_patch" else runs[::-1]:
        assert run() == 0
    once = camera.read_text()
    assert once == patch.patch_d435_collision_text(patch.patch_d435_text(D435)[0])[0]
    assert camera.with_name(camera.name + ".bak").read_text() == D435
    for run in runs:
        assert run() == 0
    assert camera.read_text() == once


def test_a_rewritten_d435_mesh_is_reported_and_the_rest_still_patched(share, capsys):
    """Upstream pointing the camera at another mesh: that edit fails loudly, the a200's edits and the box hold."""
    glob_, package = share
    camera = _camera(package)
    rewritten = D435.replace("meshes/d435.dae", "meshes/d435_v2.dae")
    camera.write_text(rewritten)
    assert patch.main([f"--ros-share={glob_}"]) == 1
    assert "0 D435 visual mesh(es)" in capsys.readouterr().err
    assert camera.read_text() == patch.patch_d435_collision_text(rewritten)[0]
    assert patch.CHASSIS_MESH_NEW in (package / patch.CHASSIS_REL).read_text()


#: The D435's visual body in ``camera_0_link`` [m], measured 2026-10-10 on both meshes its visual takes (realsense's
#: and husky_extras_description's ``d435.dae`` through the visual origin): front glass and back face along x.
D435_VISUAL_X_M = (-0.02075, 0.0043)


def _d435_properties(text: str) -> dict[str, float]:
    """The macro's numeric ``xacro:property`` values, as xacro would substitute them."""
    root = ET.fromstring(text)
    values = {}
    for prop in root.iter("{http://ros.org/wiki/xacro}property"):
        try:
            values[prop.get("name")] = float(prop.get("value"))
        except ValueError:
            pass
    return values


def _d435_collision_x_m(text: str) -> tuple[float, float]:
    """The collision box's extent along x in the camera link, its xacro expressions evaluated."""
    link = ET.fromstring(text).find(".//link[@name='${name}_link']")
    first = re.compile(r"\$\{[^}]*\}|\S+")  # an expression may hold blanks: ${a + b}
    x_expr = first.match(link.find("collision/origin").get("xyz")).group()
    depth_expr = first.match(link.find("collision/geometry/box").get("size")).group()
    env = _d435_properties(text)
    x, depth = (eval(e[2:-1] if e.startswith("${") else e, {}, env) for e in (x_expr, depth_expr))  # noqa: S307
    return x - depth / 2, x + depth / 2


def test_the_d435_collision_box_encloses_the_measured_visual_body(share):
    """Upstream's box is centered on the link, 8.2 mm ahead of the camera; patched, it spans the visual's x."""
    glob_, package = share
    assert _d435_collision_x_m(D435) == pytest.approx((-0.012525, 0.012525))
    assert patch.main([f"--ros-share={glob_}"]) == 0
    assert _d435_collision_x_m(_camera(package).read_text()) == pytest.approx(D435_VISUAL_X_M, abs=1e-5)


def test_only_the_d435_collision_origin_changes():
    """The box visual of the ``use_mesh:=false`` branch carries the same origin line and keeps it: the marker and the
    collision origin in, one origin out."""
    out, status = patch.patch_d435_collision_text(D435)
    assert status == "changed"
    changed = _changed_lines(D435, out)
    assert [line[1:].strip() for line in changed] == [
        patch.D435_COLLISION_ORIGIN_OLD,
        patch.MARKER,
        patch.D435_COLLISION_ORIGIN_NEW,
    ], changed
    assert out.count(patch.D435_COLLISION_ORIGIN_OLD) == 1
    unless = ET.fromstring(out).find(".//link/visual/{http://ros.org/wiki/xacro}unless/origin")
    assert unless.get("xyz") == "0 ${-d435_cam_depth_py} 0"


def test_the_two_d435_edits_do_not_depend_on_their_order():
    """The box moves with the mesh swapped or not, and the mesh swaps with the box moved or not."""
    mesh_first = patch.patch_d435_collision_text(patch.patch_d435_text(D435)[0])
    box_first = patch.patch_d435_text(patch.patch_d435_collision_text(D435)[0])
    assert mesh_first == box_first
    assert mesh_first[1] == "changed"


def test_a_rewritten_d435_collision_origin_is_reported_and_the_mesh_still_swapped(share, capsys):
    """Upstream moving the box itself: that edit fails loudly, the mesh swap in the same file holds."""
    glob_, package = share
    camera = _camera(package)
    moved = '<collision>\n        <origin xyz="-0.008 ${-d435_cam_depth_py} 0" rpy="0 0 0"/>'
    camera.write_text(D435.replace('<collision>\n        <origin xyz="0 ${-d435_cam_depth_py} 0" rpy="0 0 0"/>', moved))
    assert patch.main([f"--ros-share={glob_}"]) == 1
    assert "0 D435 collision origin(s)" in capsys.readouterr().err
    text = camera.read_text()
    assert moved in text
    assert _camera_mesh(text) == patch.D435_MESH_NEW.split('"')[1]


def test_a_missing_sensors_package_is_reported(share, capsys):
    glob_, package = share
    shutil.rmtree(package.parent / patch.SENSORS_PACKAGE)
    assert patch.main([f"--ros-share={glob_}"]) == 1
    assert "no d435.urdf.xacro" in capsys.readouterr().err
    assert patch.CHASSIS_MESH_NEW in (package / patch.CHASSIS_REL).read_text()


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
    """Also the shape an upstream fix would take: the anchor no longer matches, and the tool says so.  On the outdoor
    macro, whose origin is its only edit: the file stays as upstream wrote it, without a ``.bak``."""
    glob_, package = share
    target = package / patch.XACRO_REL.format(variant="outdoor")
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
