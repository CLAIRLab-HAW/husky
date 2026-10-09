# husky-extras

The URDF extras of the **a200-0553**: everything this particular robot has that
neither Clearpath's own description nor a component package knows about — the
sensor arch on the top plate, the RS16 on the arch, and the end effector on the
UR5 flange: the OnRobot RG6, or in the offboard mock another tool.

One file is the entry,
`src/husky_extras_description/urdf/clearpath_extras.urdf.xacro`, and the
Clearpath generator pulls it in through `robot.yaml`:

```yaml
platform:
  extras:
    urdf:
      path: /home/robot/onboard/extras/src/husky_extras_description/urdf/clearpath_extras.urdf.xacro
```

**Why this is a repo of its own.** Every link here hangs on a frame that only
exists once the generator has built an a200 with a UR5 on it — `arm_0_tool0`,
`top_plate_rear_mount`, `top_plate_front_mount`. A component package must not
know those: `rg6_description` describes an RG6, which is the same hand on any
arm, and it stayed unusable elsewhere for as long as it carried this robot's
assembly. The dependency runs one way only — this package includes the gripper
macro, the gripper package includes nothing from here.

## Features

- **The sensor arch as a body, not a picture** — a DAE `<visual>` in its own color plus five
  collision boxes along the real structure (2.3 L of material in a 91.3 L hull;
  a single bounding box would have walled up the whole rear) and an `<inertial>`
  of 6.21 kg. Until 2026-08-20 the link had no collision at all and MoveIt
  planned straight through it (R15); until 2026-08-31 no mass, so both
  simulators carried a half-metre portal frame as 0.1 kg (R47).
- **The RS16 where robot.yaml measured it.** `robot.yaml` places the frame `lidar3d_0_mount_link`; this file hangs
  [rs16_description](../rs16/src/rs16_description/README.md)'s macro on it, which brings the lidar's mesh,
  collision cylinder, mass and `lidar3d_0_laser`.
- **The a200's top chassis in its real colors**, `meshes/a200_top_chassis.dae`: Clearpath's mesh split into its
  parts by `tools/a200_chassis_mesh.py`: the chassis #E29000, the emergency stop red, the power button and the key
  lock metallic silver, the status LEDs black (Clearpath's mesh has one greenish-yellow material for all of it).
  `a200_look_patch` in [husky-custom-setup](../setup/README.md) points the `top_chassis_link` visual here.
- **The a200's indoor wheel with its real rims**, `meshes/a200_indoor_wheel.dae`, written by
  `tools/a200_wheel_mesh.py`: Clearpath's indoor tire, shrunk radially to the measured rolling radius 0.1477 m (R58;
  Clearpath's reaches 0.1625 m) with its bead and width kept, on the a200-0553's rim from Clearpath's CAD (two
  pressed halves, the keyed hub and the hub plate), dark gray with a metallic sheen. The outer face lies on +y like
  Clearpath's mesh. `a200_look_patch` points the indoor wheel macro's visual here; the collision cylinder stays at
  the nominal 0.1651 m. The CAD export the rim comes from is not in the repo (see License).
- **A swappable end effector.** The xacro argument `end_effector` (default
  `rg6`) includes `urdf/tools/<name>.urdf.xacro`; `robot.yaml` passes none, so
  the robot always gets its RG6. The offboard mock takes `end_effector:=<name>`.
  - `tools/rg6.urdf.xacro`: the RG6 at `arm_0_tool0`, without a mounting offset
    (the bracket screws onto the flange) and rotated by π, measured on the
    device rather than guessed.
  - `tools/key_holder.urdf.xacro`: a printed holder with a Rittal double-bit
    key for control cabinets (meshes from mujoco-mia, primitives collide), TCP
    `key_tip` 107.8 mm out, and the wrist camera clamped to the holder instead
    of 136 mm out on the RG6. Placed, not measured (R61).
- **Two frames the rest of the stack addresses by name**:
  `rg6_onrobot_rg6_base_link`, which `robot.yaml` hangs the camera on and which
  every tool carries (the key holder as an alias where its camera sits), and
  `rg6_hand_tcp`, the RG6's TCP that every calibrated quantity is expressed
  against.
- **The tools as macros**, `urdf/rg6.macro.xacro` and
  `urdf/key_holder.macro.xacro`: each hangs on a given `parent`. The a200's tool
  files call them on `arm_0_tool0`; MARWIN 5 calls them behind its tool changer.
- **MARWIN 5's tooled arm**, a second package, `marwin_extras_description`:
  `urdf/marwin5_tooled.urdf.xacro` is marwin_control's controlled description
  plus the SmartShift changer on `ur5e_tool0` (`urdf/smartshift.macro.xacro`,
  vendor figures: 65 mm coupled stack, Ø 63 mm, 180 g + 310 g electric; the
  split into 35 mm master and 30 mm holder is assumed) and the
  tool on it: `tool:=rg6` (default, TCP `rg6_hand_tcp` 0.275 m out),
  `tool:=key_holder` (TCP `key_tip` 0.1728 m out) or `tool:=none`. An unknown
  name is refused. The changer is mechanical, so which tool is on is a
  start-time choice; the holder's clocking against the master is not measured.

## Tech Stack

- **ROS 2 Jazzy**, `ament_cmake`, xacro — the package builds nothing, it
  installs `urdf/` and `meshes/` so `package://husky_extras_description/…`
  resolves through the ament index.
- **pytest** for the seam checks, ROS-free and robot-free.

## Installation

The robot builds it in its onboard clone like the gripper workspace; the
[installer](../setup/README.md) does that, and `robot.yaml` lists the result
under `system.ros2.workspaces`.

```bash
cd ~/onboard/extras && colcon build --packages-select husky_extras_description
```

In the offboard container the package comes out of the build context and is
built into `/opt/husky-extras` (the `extras-build` stage), so the image tag is
the state of the checkout it was built from; the entrypoint symlinks
`/home/robot/onboard/extras` onto it, so the absolute paths from `robot.yaml`
resolve there as well.

## Usage

Nothing here is started. The file is read by

- `clearpath_generator_common generate_description` (per boot, on the robot and
  in the `plant-mock` container),
- RViz and the `foxglove_bridge`, which resolve the arch mesh through the
  ament index of their own container,
- and `tools/derive_link_inertia.py` in `onrobot-rg6`, whose box mode recomputes
  the arch inertia from its collision boxes:

```bash
python3 ../onrobot-rg6/tools/derive_link_inertia.py --box-link \
    src/husky_extras_description/urdf/clearpath_extras.urdf.xacro husky_top_assembly 6.21
```

## Running Tests

```bash
uv run pytest onboard/extras/tests
```

Sixteen checks, from the workspace root and without ROS: that every file is
well-formed XML (an XML comment cannot contain `--`, and a malformed extras file
takes `move_group` down with it), that every `package://` URI names a package
that really ships the file, that `end_effector` defaults to the RG6, that the
addressed link names survived (the camera's parent in every tool), and that
`robot.yaml` in `husky-custom-setup` still points at this path and lists this
workspace.

`.github/workflows/ci.yml` runs the same suite on every push and pull request.
It checks `husky-custom-setup` out next to this repo and lays a
`workspace.repos` above both, because the `robot.yaml` check skips itself while
there is no workspace root — a run without that marker would be green having
compared nothing.

## Related

- [onrobot-rg6](../rg6/README.md) — the RG6 model whose macro this file
  instantiates, and `rg6_moveit_patch`, which puts the gripper into the
  generated SRDF
- [husky-custom-setup](../setup/README.md) — `robot.yaml` (SSOT),
  the boot patcher and the installer that rolls this workspace out
- [offboard](../../deploy/offboard/README.md) — the container that
  reconstructs the same setup without a robot

## Versioning

[Semantic Versioning](https://semver.org/); what changed when is in
[CHANGELOG.md](CHANGELOG.md).

## License

MIT. The RG6 model this file instantiates is vendored in `onrobot-rg6`; its
origin and the changes made to it are documented there
(`LICENSE-THIRD-PARTY.md`). `meshes/a200_top_chassis.dae` is derived from
`clearpath_platform_description`'s `top_chassis.dae` and stays under its BSD-3-Clause license, Copyright (c) 2023
clearpathrobotics; the notice is carried in full in the file's `<asset><contributor><copyright>`. The tire of
`meshes/a200_indoor_wheel.dae` is derived from `clearpath_platform_description`'s `wheels/indoor.dae` under the same
license and notice. Its rim comes from Clearpath Robotics' CAD "034991-MOBILITY_Simplified mCAD, STEP-Husky A200,
Skid Steer", which comes without a BSD license: the owner's glTF export of it (`onboard/a200_rims.gltf`, untracked) is
not redistributed, only the derived mesh, as with RoboSense's STEP behind `rs16.dae`.
