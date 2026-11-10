#!/usr/bin/env python3
"""a200_look_patch: makes the a200 LOOK like the real one -- rims outward and real, chassis and camera in real colors.

Three upstream mismatches of ``clearpath_platform_description`` and two of ``clearpath_sensors_description``: four
that only a viewer sees, and the wrist D435's collision box, which sits off the camera it stands for.  All are fixed in
the package files the generator and every viewer read, after apt and before the generator, idempotently, with a
backup -- the same shape as the other patchers here.  None changes a joint or an inertial; the D435 box is the one
collision this tool edits, and it only moves that box back onto the camera's own visual body (THE WRIST CAMERA'S BOX
below).

THE WHEELS: the right wheel visuals are turned by pi about z, so the rim faces outward on both sides.  The package
places all four wheels with ``rpy="0 0 0"`` (``urdf/a200/drivetrain/wheels.urdf.xacro``), and both wheel macros
(``urdf/a200/drivetrain/wheels/indoor.urdf.xacro`` and ``outdoor.urdf.xacro``) give the visual
``<origin xyz="0 0 0" rpy="0 0 0"/>`` with no left/right handling.  That is only harmless for a mesh that is
symmetric along the axle, and the one this vehicle's robot.yaml selects is not.

MEASURED 2026-10-09 on ``meshes/a200/wheels/indoor.dae`` (``drivetrain.wheels.front/rear: indoor``): the mesh is
one-sided along the axle.  Its hub and rim face (submesh ``Shape_IndexedFaceSet_311``, y +0.032..+0.051 m; the hub
ring at r ~0.03 m sits at y ~+0.045) are on +y, the -y side is recessed (~-0.03 m).  So the left wheels (y > 0) show
the rim outward and the right wheels (y < 0) show it facing the chassis.  ``outdoor.dae`` is nearly symmetric along
the axle, so the same turn costs it nothing visible.

WHY Rz(pi) AND NOT A MIRROR: the real Husky mounts the same wheel on both sides, the right one turned 180 deg about
the vertical axis -- which is exactly Rz(pi), tread direction included.  A mirror (y -> -y) would put the rim
outward too, but with the tread running the wrong way.  The expression ``${pi if 'right' in prefix else 0}`` uses
xacro's builtin ``pi`` and the macro's own ``prefix`` (``front_right``, ``rear_right``, ...); checked 2026-10-09 with
xacro 2.1.1 in the plant-mock container, it expands to ``0 0 3.141592653589793`` for the right prefixes and ``0 0 0``
for the left ones.  Both wheel macros are patched, as ``urdf_physics_patch`` does for the same reason: which one is
fitted is robot.yaml's choice and this tool does not read it.  Patching the variant that is not built costs nothing;
missing the one that is would be silent.

THE CHASSIS: the ``top_chassis_link`` visual is pointed at ``husky_extras_description``'s
``meshes/a200_top_chassis.dae`` instead of the package's ``meshes/a200/top_chassis.dae``.  Clearpath's mesh is ONE
geometry with ONE material, a greenish yellow (diffuse ``0.8 0.8 0 1``) -- the emergency stop, the power button and the
key lock at the rear included.  The real a200-0553's chassis is #E29000, its emergency stop red, its power button and
key lock metallic silver (the owner, 2026-10-09).  One material cannot carry that, so husky's
``scripts/a200_chassis_mesh.py`` splits Clearpath's mesh into its parts and writes one material per look; this tool only
swaps the URI.  WHY THE URI AND NOT A URDF MATERIAL: the visual sets no ``<material>``, so the mesh's colors are what
every viewer shows (Foxglove and RViz via the DAE, the MuJoCo twin via ``clair.twin.collada``), and a URDF material is
one color for the whole link.  ``husky_extras_description`` is installed wherever the generator runs: on the robot the
installer builds ``husky_extras_description`` and robot.yaml lists its workspace, in the offboard images it is built
into ``/opt/husky``.  ``ride_height_patch`` edits the same ``a200.urdf.xacro``; whichever of the two runs first
takes the one ``.bak``, the pristine apt copy.

THE INDOOR WHEELS: the visual mesh of ``urdf/a200/drivetrain/wheels/indoor.urdf.xacro`` is pointed at
``husky_extras_description``'s ``meshes/a200_indoor_wheel.dae`` instead of the package's
``meshes/a200/wheels/indoor.dae``.  Clearpath's mesh draws another rim than the a200-0553's, and a tire of r 0.1625 m
where the real one rolls on 0.1477 m (R58).  husky's ``scripts/a200_wheel_mesh.py`` keeps Clearpath's tire,
shrinks it radially to that tread radius, and seats the real rim in it, taken from Clearpath's own CAD; its outer face
lies on +y like Clearpath's, so the turn of the right wheels above holds for it unchanged.  Only the indoor macro:
robot.yaml fits indoor wheels on both axles (``drivetrain.wheels.front/rear: indoor``), and the outdoor wheel is
another tire this mesh does not show -- that variant keeps Clearpath's mesh.  The collision cylinder (r 0.1651, the
macro's nominal radius) and the inertia stay: what the wheel touches the ground with is a separate decision, as the
ground reference comes from ``ride_height_patch``'s LOADED radius and MoveIt and the twin collide with the cylinder.

THE WRIST CAMERA: the visual of ``clearpath_sensors_description``'s D435 macro (``urdf/intel/d435.urdf.xacro``, which
``d435i.urdf.xacro`` includes as well) is pointed at ``husky_extras_description``'s ``meshes/d435.dae`` instead of
``realsense2_description``'s ``meshes/d435.dae``.  realsense's mesh is one gray (diffuse 0.749) for the whole camera;
the a200-0553's D435 has a black front, dark lenses in polished rims and an aluminum casing (the owner, 2026-10-10).
husky's ``scripts/d435_mesh.py`` writes MuJoCo Menagerie's colored D435 in exactly the frame of realsense's
mesh, so the macro's visual origin stays as it is and only the URI changes.  ``sensor_mesh_uri_patch`` rewrites the
same line's ``file://$(find realsense2_description)`` to ``package://realsense2_description``: the anchor here takes
the line in either form, so the two compose in any order.  After this one, the line names no
``realsense2_description`` and the other finds nothing to do in it; before it, this one finds the ``package://``
form.  The robot runs ``sensor_mesh_uri_patch`` first (step 2 of the boot service, this tool step 7), the offboard
image at build time and this tool at container start, and whichever runs first takes the shared ``.bak``, the
pristine apt copy.

THE WRIST CAMERA'S BOX: the D435 macro's collision box (``<box size="${d435_cam_depth} ${d435_cam_width}
${d435_cam_height}"/>``, 25.05 x 90 x 25 mm) is centered on ``${name}_link`` along x, its origin ``0`` -- but the
camera's visual body is not.  MEASURED 2026-10-10 on both meshes the visual takes (realsense's ``d435.dae`` and
``husky_extras_description``'s, through the visual origin ``xyz 0.0043 -0.0175 0``, ``rpy pi/2 0 pi/2``), in
``camera_0_link``: x -20.75..+4.30 mm, y -62.42..+27.52, z -12.51..+12.50.  The front glass lies 4.3 mm AHEAD of the
link (``d435_zero_depth_to_glass + d435_glass_to_front``, the very offset the visual origin carries) and the body
reaches 25.05 mm back from it, so the box stood 8.2 mm too far forward: on the a200-0553's wrist, in the RG6 body's
frame, the box spanned z 71.7..97.9 mm and the visual 63.7..89.5 -- 8 mm toward the fingers, into the space a grasped
object or the fingers' approach takes.  The fix moves the box's x origin to ``${d435_zero_depth_to_glass +
d435_glass_to_front - d435_cam_depth/2}`` (-8.225 mm): its front face on the glass, its back 25.05 mm behind.  The
size stays, as the measured extent matches it to 0.06 mm (y 89.94 against 90, z 25.01 against 25), and so do y and z
(the visual's center lies at y -17.45, z 0.00 against the box's -17.5 and 0).  The expression rather than a literal:
it is upstream's own offset, so the box stays on the glass if upstream ever changes the depth or the glass offset
and the anchor still matches.  Only the ``<collision>``: the ``use_mesh:=false`` branch of the visual draws the same
box with the same origin line, and is left as upstream ships it -- this robot draws the mesh.  It is the only
collision this tool edits; MoveIt checks it against the arm, the RG6 and the world, and the SRDF the container
generates is what decides which of those pairs it ignores.

NO LEVER IN robot.yaml for any of them: ``clearpath_config`` picks the wheel variant per axle (``indoor``/``outdoor``)
and nothing else -- the visual origin and the wheel mesh are literals inside the wheel macro, the chassis mesh a
literal inside the a200 macro, the camera mesh and its collision origin literals inside the D435 macro.

WHY ITS OWN TOOL: ``urdf_physics_patch`` is about physical properties a physics engine needs (joint dynamics,
inertials), and this changes none; ``ride_height_patch`` moves the ground reference by a measured property of THIS
vehicle (its loaded tire radius), and nothing here moves a frame -- the indoor wheel mesh draws this vehicle's rims
and rolling radius, but only draws them.  The D435 box is the exception that stays here: it is the same macro and
the same camera as the mesh swap, the correction is the visual body's own extent, and a tool of its own for one
origin would be a second patcher with the same anchor, marker and ``.bak`` on the same file.  The first two looks
are defects every a200 with the indoor wheel, and every a200, shows.  That also means both belong upstream: an issue
against ``clearpath_common`` for the wheels is drafted (the owner files it; no number yet), and the chassis colors
could be raised there too -- the owner decides.  Until then: both are unchanged in 2.9.15 (installed) and on the
``jazzy`` branch (checked 2026-10-09; its ``top_chassis.dae`` is byte-identical to the installed one), so an upgrade
will not make this unnecessary by itself -- when upstream changes any of them, its anchor below stops matching and
the tool says so instead of patching twice.

Invocations:
  Robot:    from clearpath_custom_setup.py (per boot, before clearpath-robot-generate)
  Offboard: offboard entrypoint.sh, before the generate_* runs
  Manual:   a200-look-patch --dry-run

Self-contained: needs only python3 (no ROS environment, no PyYAML) -- the robot runs it as a root-owned copy in
/usr/local/bin (the installer places it, like every other script here), and the offboard container copies the same
one file out of this repo at build time.  One file is the whole point: both deployments are a copy, not a build.
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

TAG = "a200_look_patch"

#: Written above what this tool rewrites, so that a reader of an apt-owned file can see at a glance where the line
#: came from.  No double dash inside: XML forbids it in a comment.
MARKER = "<!-- husky-custom-setup:a200_look_patch -->"

#: The visual origin exactly as both wheel macros ship it (2.9.15), and what it has to become.  The same literal
#: without the ``<visual>`` in front of it is not unique -- the collision origin differs only in its roll -- so the
#: anchor below is the visual element's opening tag plus this line.
ORIGIN_OLD = '<origin xyz="0 0 0" rpy="0 0 0" />'
ORIGIN_NEW = '<origin xyz="0 0 0" rpy="0 0 ${pi if \'right\' in prefix else 0}" />'

#: ``<visual>`` and the origin on the line directly below it; group 2 keeps that line's indentation.
VISUAL_ORIGIN_RX = re.compile(r"(<visual>[ \t]*\n)([ \t]*)" + re.escape(ORIGIN_OLD))

#: The chassis visual's mesh exactly as ``a200.urdf.xacro`` ships it (2.9.15), and the recolored one in
#: ``husky_extras_description``.
CHASSIS_MESH_OLD = '<mesh filename="package://clearpath_platform_description/meshes/a200/top_chassis.dae" />'
CHASSIS_MESH_NEW = '<mesh filename="package://husky_extras_description/meshes/a200_top_chassis.dae" />'

#: The chassis link's visual from its opening tag to the mesh line; group 2 keeps the mesh line's indentation.  The
#: link name is part of the anchor, so a second link that might take the same mesh is never touched by accident.
CHASSIS_VISUAL_RX = re.compile(
    r'(<link name="top_chassis_link">\s*<visual>(?:(?!</visual>).)*?\n)([ \t]*)' + re.escape(CHASSIS_MESH_OLD), re.S
)

#: The indoor wheel visual's mesh exactly as ``indoor.urdf.xacro`` ships it (2.9.15), and the a200-0553's wheel in
#: ``husky_extras_description``: Clearpath's tire at the measured tread radius on the real rims.
WHEEL_MESH_OLD = '<mesh filename="package://clearpath_platform_description/meshes/a200/wheels/indoor.dae" />'
WHEEL_MESH_NEW = '<mesh filename="package://husky_extras_description/meshes/a200_indoor_wheel.dae" />'

#: The wheel link's visual from its opening tag to the mesh line; group 2 keeps the mesh line's indentation.  Inside
#: one ``<visual>``, so the marker and the turned origin :func:`patch_wheel_text` puts there do not stop the match.
WHEEL_VISUAL_RX = re.compile(r"(<visual>(?:(?!</visual>).)*?\n)([ \t]*)" + re.escape(WHEEL_MESH_OLD), re.S)

#: The D435 visual's mesh as ``clearpath_sensors_description`` 2.9.15 ships it -- ``file://$(find ...)`` from apt,
#: ``package://`` once ``sensor_mesh_uri_patch`` has run -- and the a200-0553's colored D435 in
#: ``husky_extras_description``, in the same frame.
D435_MESH_OLD = re.compile(
    r'<mesh filename="(?:file://\$\(find realsense2_description\)|package://realsense2_description)'
    r'/meshes/d435\.dae" />'
)
D435_MESH_NEW = '<mesh filename="package://husky_extras_description/meshes/d435.dae" />'

#: The camera link's visual from its opening tag to the mesh line; group 2 keeps the mesh line's indentation.  The
#: link name is part of the anchor, as for the chassis: the mesh inside the ``use_mesh`` branch of
#: ``${name}_link``'s visual and no other.
D435_VISUAL_RX = re.compile(
    r'(<link name="\$\{name\}_link">\s*<visual>(?:(?!</visual>).)*?\n)([ \t]*)' + D435_MESH_OLD.pattern, re.S
)

#: The D435 collision box's origin as ``clearpath_sensors_description`` 2.9.15 ships it -- centered on the link along
#: x -- and the one that puts its front face on the camera's front glass, where the visual body starts (measured
#: 2026-10-10, see the docstring): -8.225 mm along x, y and z unchanged.
D435_COLLISION_ORIGIN_OLD = '<origin xyz="0 ${-d435_cam_depth_py} 0" rpy="0 0 0"/>'
D435_COLLISION_ORIGIN_NEW = (
    '<origin xyz="${d435_zero_depth_to_glass + d435_glass_to_front - d435_cam_depth/2} ${-d435_cam_depth_py} 0" '
    'rpy="0 0 0"/>'
)

#: The camera link up to its ``<collision>`` and the origin on the line directly below it; group 2 keeps that line's
#: indentation.  The same origin line stands in the visual's ``use_mesh:=false`` branch, so the anchor is the
#: collision element's opening tag plus this line, inside ``${name}_link``.
D435_COLLISION_RX = re.compile(
    r'(<link name="\$\{name\}_link">(?:(?!</link>).)*?<collision>[ \t]*\n)([ \t]*)'
    + re.escape(D435_COLLISION_ORIGIN_OLD),
    re.S,
)

#: Where the ROS packages live.  A glob rather than a fixed distro, for the same reason as in
#: ``urdf_physics_patch``: the same tool runs on the robot (jazzy from apt) and in the container (jazzy from the
#: image), and a hard-coded ``jazzy`` would fail silently on the day one of them moves.
ROS_SHARE_GLOB = "/opt/ros/*/share"

PACKAGE = "clearpath_platform_description"
SENSORS_PACKAGE = "clearpath_sensors_description"

#: The two wheel macros.  Named, not globbed: ``.urdf.xacro`` keeps a ``.bak`` from an earlier run out of the
#: targets, and a variant upstream renames has to be REPORTED, which a glob would simply stop finding.
VARIANTS = ("indoor", "outdoor")
XACRO_REL = "urdf/a200/drivetrain/wheels/{variant}.urdf.xacro"
CHASSIS_REL = "urdf/a200/a200.urdf.xacro"
D435_REL = "urdf/intel/d435.urdf.xacro"


def log(msg, err=False):
    """Log line (stdout/stderr); captured by journald via SyslogIdentifier on the robot."""
    print(f"{TAG}: {msg}", file=(sys.stderr if err else sys.stdout), flush=True)


def atomic_write(path, content):
    """Write ``content`` over ``path``, keeping one backup and never leaving a half-written file.

    The same shape as ``ride_height_patch.atomic_write``: the backup is taken ONCE, so a second run cannot overwrite
    the pristine apt copy with an already-patched one.  The diff against that copy is the only way to see what a
    boot changed.  A backup that cannot be written does not stop the rewrite -- the fix outranks the undo record.
    """
    backup = path + ".bak"
    if not os.path.exists(backup):
        try:
            shutil.copy2(path, backup)
            log(f"backup written: {backup}")
        except OSError as e:
            # The write below goes ahead regardless, but without the .bak it cannot be undone -- hence stderr.
            log(f"no backup of {path}: {e}", err=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path) or ".", suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(content)
        shutil.copymode(path, tmp)
        os.replace(tmp, path)
    except Exception:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise


def _well_formed(content, patched):
    """``(patched, "changed")`` if ``patched`` parses as XML, else the untouched ``content`` and the reason."""
    try:
        ET.fromstring(patched)
    except ET.ParseError as e:
        return content, f"the patched text is not well-formed ({e}) -- nothing written"
    return patched, "changed"


def patch_wheel_text(content):
    """Turn the visual origin of one wheel macro.

    String surgery rather than an XML round trip, for the reason ``urdf_physics_patch.element_spans`` gives: this is
    a file the workspace does not own, and ElementTree would reformat every untouched line and drop the upstream
    comments.  Well-formedness is checked on the result instead.

    :returns: ``(content, status)`` -- status is ``"changed"``, ``"clean"`` (already turned) or a reason string
        for a failure (anchor missing or ambiguous, result not well-formed); ``content`` is unchanged unless the
        status is ``"changed"``.
    """
    if MARKER in content and ORIGIN_NEW in content:
        return content, "clean"
    hits = len(VISUAL_ORIGIN_RX.findall(content))
    if hits != 1:
        # Zero is an upstream rewrite of the line (or upstream fixed it itself); more than one is a macro shape this
        # tool was not written against.  Either way nothing is guessed.
        return content, f"{hits} visual origin(s) of the shipped form found, expected exactly 1 -- upstream changed?"
    patched = VISUAL_ORIGIN_RX.sub(lambda m: f"{m.group(1)}{m.group(2)}{MARKER}\n{m.group(2)}{ORIGIN_NEW}", content)
    return _well_formed(content, patched)


def patch_chassis_text(content):
    """Point the ``top_chassis_link`` visual of ``a200.urdf.xacro`` at the recolored chassis mesh.

    String surgery for the same reason as :func:`patch_wheel_text`, with the same marker above the rewritten line.

    :returns: ``(content, status)`` as :func:`patch_wheel_text` returns it.
    """
    if MARKER in content and CHASSIS_MESH_NEW in content:
        return content, "clean"
    hits = len(CHASSIS_VISUAL_RX.findall(content))
    if hits != 1:
        # Zero is an upstream rewrite of the chassis visual or its mesh; nothing is guessed.
        return (
            content,
            f"{hits} chassis visual mesh(es) of the shipped form found, expected exactly 1 -- upstream changed?",
        )
    patched = CHASSIS_VISUAL_RX.sub(
        lambda m: f"{m.group(1)}{m.group(2)}{MARKER}\n{m.group(2)}{CHASSIS_MESH_NEW}", content
    )
    return _well_formed(content, patched)


def patch_wheel_mesh_text(content):
    """Point the visual of the indoor wheel macro at the a200-0553's wheel mesh.

    String surgery for the same reason as :func:`patch_wheel_text`, with the same marker above the rewritten line;
    it matches with the visual origin turned or not, so the order of the two does not matter.

    :returns: ``(content, status)`` as :func:`patch_wheel_text` returns it.
    """
    if MARKER in content and WHEEL_MESH_NEW in content:
        return content, "clean"
    hits = len(WHEEL_VISUAL_RX.findall(content))
    if hits != 1:
        # Zero is an upstream rewrite of the wheel visual or its mesh; nothing is guessed.
        return (
            content,
            f"{hits} wheel visual mesh(es) of the shipped form found, expected exactly 1 -- upstream changed?",
        )
    patched = WHEEL_VISUAL_RX.sub(lambda m: f"{m.group(1)}{m.group(2)}{MARKER}\n{m.group(2)}{WHEEL_MESH_NEW}", content)
    return _well_formed(content, patched)


def patch_d435_text(content):
    """Point the camera link's visual of the D435 macro at the a200-0553's colored D435 mesh.

    String surgery for the same reason as :func:`patch_wheel_text`, with the same marker above the rewritten line; it
    matches the mesh line with ``sensor_mesh_uri_patch`` applied or not, so the order of the two does not matter.

    :returns: ``(content, status)`` as :func:`patch_wheel_text` returns it.
    """
    if MARKER in content and D435_MESH_NEW in content:
        return content, "clean"
    hits = len(D435_VISUAL_RX.findall(content))
    if hits != 1:
        # Zero is an upstream rewrite of the camera visual or its mesh; nothing is guessed.
        return (
            content,
            f"{hits} D435 visual mesh(es) of the shipped form found, expected exactly 1 -- upstream changed?",
        )
    patched = D435_VISUAL_RX.sub(lambda m: f"{m.group(1)}{m.group(2)}{MARKER}\n{m.group(2)}{D435_MESH_NEW}", content)
    return _well_formed(content, patched)


def patch_d435_collision_text(content):
    """Move the D435 macro's collision box back by 8.225 mm along x, onto the camera's visual body.

    String surgery for the same reason as :func:`patch_wheel_text`, with the same marker above the rewritten line; it
    matches with the visual's mesh swapped by :func:`patch_d435_text` or not, so the order of the two does not matter.

    :returns: ``(content, status)`` as :func:`patch_wheel_text` returns it.
    """
    if MARKER in content and D435_COLLISION_ORIGIN_NEW in content:
        return content, "clean"
    hits = len(D435_COLLISION_RX.findall(content))
    if hits != 1:
        # Zero is an upstream rewrite of the collision origin (or upstream moved the box itself); nothing is guessed.
        return (
            content,
            f"{hits} D435 collision origin(s) of the shipped form found, expected exactly 1 -- upstream changed?",
        )
    patched = D435_COLLISION_RX.sub(
        lambda m: f"{m.group(1)}{m.group(2)}{MARKER}\n{m.group(2)}{D435_COLLISION_ORIGIN_NEW}", content
    )
    return _well_formed(content, patched)


def patch_file(path, patcher, dry_run):
    """Apply ``patcher`` (:func:`patch_wheel_text`, :func:`patch_wheel_mesh_text`, :func:`patch_chassis_text`,
    :func:`patch_d435_text` or :func:`patch_d435_collision_text`) to one file.

    :returns: ``"changed"``, ``"clean"`` or ``"failed"`` -- the last one when the file cannot be read or written or
        the anchor does not match, each logged on stderr with the reason.
    """
    try:
        with open(path, encoding="utf-8") as f:
            content = f.read()
    except OSError as e:
        log(f"cannot read {path}, skipping: {e}", err=True)
        return "failed"
    patched, status = patcher(content)
    if status == "clean":
        return "clean"
    if status != "changed":
        log(f"{path}: {status}", err=True)
        return "failed"
    if dry_run:
        log(f"dry run: {path} would change.")
        return "changed"
    try:
        atomic_write(path, patched)
    except OSError as e:
        log(f"cannot write {path}: {e}", err=True)
        return "failed"
    return "changed"


def targets(share):
    """``(path, patcher, what it fixes)`` for every edit this tool makes in one share directory -- the indoor wheel
    macro and the D435 macro twice each, each edit on its own, so an upstream rewrite of one does not hold back the
    other."""
    package = os.path.join(share, PACKAGE)
    wheels = "the right wheels' visual turned by pi about z"
    macros = [(os.path.join(package, XACRO_REL.format(variant=v)), patch_wheel_text, wheels) for v in VARIANTS]
    indoor = os.path.join(package, XACRO_REL.format(variant="indoor"))
    camera = os.path.join(share, SENSORS_PACKAGE, D435_REL)
    return [
        *macros,
        (indoor, patch_wheel_mesh_text, "the indoor wheel mesh swapped for the a200-0553's rims and tread radius"),
        (os.path.join(package, CHASSIS_REL), patch_chassis_text, "the chassis mesh swapped for the recolored one"),
        (camera, patch_d435_text, "the D435 mesh swapped for the colored one"),
        (camera, patch_d435_collision_text, "the D435 collision box moved back onto its visual body"),
    ]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--ros-share",
        default=ROS_SHARE_GLOB,
        help=f"glob of the ROS share directories to patch in (default: {ROS_SHARE_GLOB})",
    )
    parser.add_argument("--dry-run", action="store_true", help="report what would change, write nothing")
    args = parser.parse_args(argv)

    shares = sorted(share for share in glob.glob(args.ros_share) if os.path.isdir(os.path.join(share, PACKAGE)))
    # Finding nothing is an error, not a steady state: with the package gone or moved the right wheels silently show
    # the rim to the chassis again and the chassis its greenish yellow.  The nonzero exit is what the offboard
    # entrypoint and the boot service (run_root_tool) log; neither stops over it.  A share with the platform but
    # without the sensors package reports the D435 macro missing below, the same way.
    if not shares:
        log(f"WARN: no {PACKAGE} under {args.ros_share} -- package missing or moved?", err=True)
        return 1

    results = []
    for share in shares:
        for path, patcher, look in targets(share):
            if not os.path.isfile(path):
                log(f"WARN: no {os.path.basename(path)} at {path} -- upstream renamed or moved it?", err=True)
                results.append((path, look, "failed"))
                continue
            results.append((path, look, patch_file(path, patcher, args.dry_run)))

    changed = [f"{os.path.basename(p)} ({look})" for p, look, result in results if result == "changed"]
    failed = [f"{os.path.basename(p)} ({look})" for p, look, result in results if result == "failed"]
    if changed:
        did = "would patch" if args.dry_run else "patched"
        log(f"{did}: {', '.join(changed)}.")
    elif not failed:
        log(f"a200 look already patched -- no change ({len(results)} edit(s) checked).")
    if failed:
        log(f"failed to patch: {', '.join(failed)} -- that part of the a200 keeps the upstream look.", err=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
