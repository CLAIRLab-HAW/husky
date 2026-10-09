#!/usr/bin/env python3
"""wheel_visual_patch: turns the a200's RIGHT wheel visuals by pi about z, so the rim faces outward on both sides.

``clearpath_platform_description`` places all four wheels with ``rpy="0 0 0"``
(``urdf/a200/drivetrain/wheels.urdf.xacro``), and both wheel macros (``urdf/a200/drivetrain/wheels/indoor.urdf.xacro``
and ``outdoor.urdf.xacro``) give the visual ``<origin xyz="0 0 0" rpy="0 0 0"/>`` with no left/right handling.  That
is only harmless for a mesh that is symmetric along the axle, and the one this vehicle's robot.yaml selects is not.

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
for the left ones.

VISUAL ONLY: the joint, its axis, the collision cylinder (rotationally symmetric about the axle anyway) and the
inertial stay exactly as upstream ships them.  Odometry, planning and the physics engines see no difference.

NO LEVER IN robot.yaml: ``clearpath_config`` picks the wheel variant per axle (``indoor``/``outdoor``) and nothing
else -- the visual origin is a literal inside the package macro.  Same situation, same answer as the other patchers
here: patch the package after apt, before the generator, idempotently, with a backup.

WHY ITS OWN TOOL: ``urdf_physics_patch`` is about physical properties a physics engine needs (joint dynamics,
inertials), and this changes none; ``ride_height_patch`` carries a measured property of THIS vehicle (its loaded
tire radius), and this is not a property of our build at all -- every a200 with the indoor wheel shows it.  That
also means it belongs upstream: an issue against ``clearpath_common`` is being prepared (the owner files it; no
number yet).  Until then: upstream is unchanged in 2.9.15 (installed) and on the ``jazzy`` branch (checked
2026-10-09), so an upgrade will not make this unnecessary by itself -- when upstream fixes it, the anchor below
stops matching and the tool says so instead of patching twice.

Both wheel macros are patched, as ``urdf_physics_patch`` does for the same reason: which one is fitted is
robot.yaml's choice and this tool does not read it.  Patching the variant that is not built costs nothing; missing
the one that is would be silent.

Invocations:
  Robot:    from clearpath_custom_setup.py (per boot, before clearpath-robot-generate)
  Offboard: offboard entrypoint.sh, before the generate_* runs
  Manual:   wheel-visual-patch --dry-run

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

TAG = "wheel_visual_patch"

#: Written above the origin this tool rewrites, so that a reader of an apt-owned file can see at a glance where the
#: line came from.  No double dash inside: XML forbids it in a comment.
MARKER = "<!-- husky-custom-setup:wheel_visual_patch -->"

#: The visual origin exactly as both wheel macros ship it (2.9.15), and what it has to become.  The same literal
#: without the ``<visual>`` in front of it is not unique -- the collision origin differs only in its roll -- so the
#: anchor below is the visual element's opening tag plus this line.
ORIGIN_OLD = '<origin xyz="0 0 0" rpy="0 0 0" />'
ORIGIN_NEW = '<origin xyz="0 0 0" rpy="0 0 ${pi if \'right\' in prefix else 0}" />'

#: ``<visual>`` and the origin on the line directly below it; group 2 keeps that line's indentation.
VISUAL_ORIGIN_RX = re.compile(r"(<visual>[ \t]*\n)([ \t]*)" + re.escape(ORIGIN_OLD))

#: Where the ROS packages live.  A glob rather than a fixed distro, for the same reason as in
#: ``urdf_physics_patch``: the same tool runs on the robot (jazzy from apt) and in the container (jazzy from the
#: image), and a hard-coded ``jazzy`` would fail silently on the day one of them moves.
ROS_SHARE_GLOB = "/opt/ros/*/share"

PACKAGE = "clearpath_platform_description"

#: The two wheel macros.  Named, not globbed: ``.urdf.xacro`` keeps a ``.bak`` from an earlier run out of the
#: targets, and a variant upstream renames has to be REPORTED, which a glob would simply stop finding.
VARIANTS = ("indoor", "outdoor")
XACRO_REL = "urdf/a200/drivetrain/wheels/{variant}.urdf.xacro"


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


def patch_text(content):
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
    try:
        ET.fromstring(patched)
    except ET.ParseError as e:
        return content, f"the patched text is not well-formed ({e}) -- nothing written"
    return patched, "changed"


def patch_file(path, dry_run):
    """Apply :func:`patch_text` to one file.

    :returns: ``"changed"``, ``"clean"`` or ``"failed"`` -- the last one when the file cannot be read or written or
        the anchor does not match, each logged on stderr with the reason.
    """
    try:
        with open(path) as f:
            content = f.read()
    except OSError as e:
        log(f"cannot read {path}, skipping: {e}", err=True)
        return "failed"
    patched, status = patch_text(content)
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


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--ros-share",
        default=ROS_SHARE_GLOB,
        help=f"glob of the ROS share directories to patch in (default: {ROS_SHARE_GLOB})",
    )
    parser.add_argument("--dry-run", action="store_true", help="report what would change, write nothing")
    args = parser.parse_args(argv)

    packages = sorted(
        os.path.join(share, PACKAGE)
        for share in glob.glob(args.ros_share)
        if os.path.isdir(os.path.join(share, PACKAGE))
    )
    # Finding nothing is an error, not a steady state: with the package gone or moved the right wheels silently show
    # the rim to the chassis again.  The nonzero exit is what the offboard entrypoint and the boot service
    # (run_root_tool) log; neither stops over it.
    if not packages:
        log(f"WARN: no {PACKAGE} under {args.ros_share} -- package missing or moved?", err=True)
        return 1

    results = []
    for package in packages:
        for variant in VARIANTS:
            path = os.path.join(package, XACRO_REL.format(variant=variant))
            if not os.path.isfile(path):
                log(f"WARN: no {variant} wheel macro at {path} -- upstream renamed or moved it?", err=True)
                results.append((path, "failed"))
                continue
            results.append((path, patch_file(path, args.dry_run)))

    changed = [os.path.basename(p) for p, result in results if result == "changed"]
    failed = [os.path.basename(p) for p, result in results if result == "failed"]
    if changed:
        did = "would turn" if args.dry_run else "turned"
        log(f"{did} the right wheels' visual by pi about z in: {', '.join(changed)}.")
    elif not failed:
        log(f"right wheel visuals already turned -- no change ({len(results)} macro(s) checked).")
    if failed:
        log(f"failed to patch: {', '.join(failed)} -- the right wheels there show the rim to the chassis.", err=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
