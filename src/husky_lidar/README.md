# lidar

The lidar of the Husky **a200-0553** — ONE, in the slot `lidar3d_0`: a RoboSense **RS-LiDAR-16**, or the lab's
Blickfeld **Qb2** in its place. Their descriptions, their driver configuration and the point cloud, raw and without the
robot's own body. Runs in the offboard container (the RS16 against a recording, either one cast by a sim plant) **and**
onboard on the real robot — same configuration, same driver. A model is an entry in `husky_lidar.models`, its driver
a module beside it, its macro a description package under `src/`.

This is the sensor and nothing that consumes it. Navigation derives its 2D scan from these points
([husky_navigation](../../src/husky_navigation/README.md), `launch/scan.launch.py`), perception reads them as they are; both import the
names from here, and neither from the other.

## Features

- **Models and their drivers** (`husky_lidar.models`): a `LidarModel` names its driver, the device type the driver
  knows it by, and the sweep a simulator casts in its stead (scan lines, azimuths, rate, blind zone).
  `launch/lidar.launch.py` starts the driver of `model:=<name>` (without it the RS16, whose recording the container
  replays):

  | Model | Driver module | Driver | Sweep a sim casts |
  |---|---|---|---|
  | `rs16` | `husky_lidar.rslidar` | `rslidar_sdk`, apt | 16 rings over ±15°, the full turn, 10 Hz |
  | `qb2` | `husky_lidar.blickfeld_qb2` | `blickfeld_qb2_ros2_driver` 2.1.0, source | 80 lines over ±25°, 90° about +y |

- **Not every lidar belongs here.** clearpath_config brings Velodyne, Ouster and Seyond up under `sensors.lidar3d`
  in `robot.yaml` itself, driver and mesh included, and publishes on the same `sensors/lidar3d_<n>/…` names. A lidar
  of those makes goes there; this package carries what clearpath_config has no model for (RoboSense, Blickfeld;
  2.9.x).
- **The robot names its ONE model in `robot.yaml`.** `platform.extras.launch` starts
  [husky_bringup](../../src/husky_extras_description/README.md)'s `lidar.launch.py` with `lidar_model: rs16` (or `qb2`), which includes
  the scan and with it this launch — run by Clearpath's own `clearpath-platform-extras` service.
  `models.robot_model()` reads that name and refuses none or two; the container's sim plants cast the model it
  returns, and the extras file draws it (its `lidar_model` default, which its tests hold to `robot.yaml`).
- **The RS16 as a URDF macro** (`src/rs16_description`): `rs16(name, parent_link)` puts `${name}_link` on the
  mounting face — RoboSense's CAD model in its own colors, a collision cylinder Ø109 × 80.7 mm, 0.87 kg — and
  `${name}_laser` 39 mm above it, at the optical center. Where the sensor is mounted is the robot's: `robot.yaml`
  places the frame `lidar3d_0_mount_link`, [husky_extras_description](../../src/husky_extras_description/README.md) hangs the macro on it.
- **The Qb2 as a URDF macro** (`src/qb2_description`): `qb2(name, parent_link)` puts `${name}_link` on the back
  face, where the M4 threads are, a box 53 × 111 × 75 mm of 0.40 kg (the ST housing), and `${name}_laser` about
  20 mm ahead of it in the device's own frame — x right, y ahead, z up, as the driver hands the points on.
- **One driver configuration for the real sensor and the recording.** `rslidar_sdk` (apt, `ros-jazzy-rslidar-sdk`)
  reads the device on the robot and `data/recordings/rs16_labor.pcap` in the container; the difference is
  `common.msg_source`, rendered from the one template `config/rslidar.yaml`, which the model's `lidar_type` completes.
  The Qb2's driver takes `config/blickfeld_qb2.yaml` (the device's address) plus the wiring's frame and topic; it
  replays nothing, so the Qb2 has only a live device or a sim plant. A sim plant (ManiSkill, MuJoCo) casts the
  model's sweep in its world and publishes on the same topic.
- **The robot's own body out of the points.** `self_filter`, a C++ node (`ros2 launch husky_lidar
  self_filter.launch.py`), drops the points inside the robot's collision bodies — the sensor arch beside it, an arm
  reaching out — and publishes the rest on `…/lidar3d_0/points_filtered`. The bodies are `geometric_shapes`', built
  from the live `robot_description`: primitives as they are, meshes as their convex hulls, grown by 5 cm. Until the
  description has come, the points pass through unfiltered.
- **Names by slot, not by model** (`husky_lidar.wiring`): `lidar3d_0`, Clearpath's convention.
- **ROS-free core.** Names, models and the driver configuration are testable without ROS; the launch file imports
  them.

## Tech Stack

ROS 2 Jazzy · `rslidar_sdk` 1.5.16 · `blickfeld_qb2_ros2_driver` 2.1.0 (Qb2 only) · `geometric_shapes` 2.3 ·
xacro · C++17 · Python 3.12+

## Installation

An ament_cmake package with its Python modules beside the C++ node (`ament_cmake_python`) in husky's colcon workspace:
the installer builds it in `~/husky` with the rest (`robot.yaml` lists the install space), and the offboard image
builds it into `/opt/husky` from the build context `husky-src`. The self filter builds against
`ros-jazzy-geometric-shapes`. Where `robot.yaml` names the `qb2`, the installer
also clones `blickfeld_qb2_ros2_driver` at `v2.1.0` into husky's `src/` (untracked) and builds it there: Blickfeld ships no apt package and
names Foxy and Humble, not Jazzy, so that build is the first test of it (R88).

## Usage

```bash
ros2 launch husky_lidar lidar.launch.py pcap:=/data/recordings/rs16_labor.pcap   # container
ros2 launch husky_lidar lidar.launch.py model:=rs16                                # robot, by hand
ros2 launch husky_lidar lidar.launch.py model:=qb2                                 # the Qb2 instead
```

To put the Qb2 in the RS16's place: `lidar_model: qb2` in `robot.yaml` and in the extras file's `lidar_model` default
(one test fails until both say it), the mount pose `lidar3d_0_mount` measured again for it, the installer rerun, then
`sudo systemctl restart clearpath-robot` (R88).

The points alone. On the robot Clearpath's `clearpath-platform-extras` service starts navigation's `scan.launch.py`,
which includes this one, through `husky_bringup`.

Another RoboSense model is an entry in `husky_lidar.models` and a description package beside `rs16_description`,
plus its file under the extras' `lidars/`. A model of another make adds a driver module beside `husky_lidar.rslidar`,
its name to `models.DRIVERS` and its start to the launch's `DRIVERS`, as the Qb2 did.

To rewrite the RS16 mesh from RoboSense's CAD model (`RS16.stp`, robosense.ai → Downloads → "RS-LiDAR-16 3D Model",
not in the repo, and neither is the script: the owner keeps it in `husky/.local/`, its last versioned state is
husky `1453d06`):

```bash
uv run husky/.local/rs16_mesh.py RS16.stp husky/src/rs16_description/meshes/rs16.dae
```

## Running Tests

```bash
uv run pytest tests/lidar                 # from the husky root: no ROS, no container
colcon test --packages-select husky_lidar # the self filter's bodies (gtest), in a ROS workspace
```

The driver replaying the RS16 recording in the offboard container is clearpath's
`deploy/offboard/tests/test_lidar_driver_e2e.py`.

## Related

- [rs16_description](src/rs16_description/README.md) — the RS16's macro
- [qb2_description](src/qb2_description/README.md) — the Qb2's macro
- [husky_navigation](../../src/husky_navigation/README.md) — the scan and Nav2 on these points
- [husky_extras_description](../../src/husky_extras_description/README.md) — `husky_bringup`, what robot.yaml starts
- [robot-contract profile](../../robot/contract/src/clair/robot/contract/profiles/bindings/a200_0553.ros2.yaml) —
  the topics as clients see them

## Versioning

[SemVer](https://semver.org/), history in the [CHANGELOG](CHANGELOG.md).

## License

See the workspace. The RS16 mesh is derived from RoboSense's CAD model of the RS-LiDAR-16. `blickfeld_qb2_ros2_driver`,
which the installer clones, is Blickfeld's, under BSD-3-Clause.
