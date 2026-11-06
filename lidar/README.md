# lidar

The lidar of the Husky **a200-0553**, a RoboSense **RS-LiDAR-16**: its description, its driver configuration and its
point cloud, raw and without the robot's own body. Runs in the offboard container against a recording **and** onboard
on the real robot — same configuration, same driver. Built to take another model: a model is an entry in
`clair.lidar.models`, its driver a module beside it, its macro a description package under `src/`.

This is the sensor and nothing that consumes it. Navigation derives its 2D scan from these points
([navigation](../navigation/README.md), `launch/scan.launch.py`), perception reads them as they are; both import the
names from here, and neither from the other.

## Features

- **Models and their drivers** (`clair.lidar.models`): a `LidarModel` names its driver, the device type the driver
  knows it by, and the sweep a simulator casts in its stead (rings, rate, blind zone). `launch/lidar.launch.py` starts
  the driver of `model:=<name>`, by default the a200-0553's `rs16`. Today there is one driver, `rslidar`
  (`clair.lidar.rslidar`): RoboSense's `rslidar_sdk`, one binary for the whole RoboSense line.
- **Not every lidar belongs here.** clearpath_config brings Velodyne, Ouster and Seyond up under `sensors.lidar3d`
  in `robot.yaml` itself, driver and mesh included, and publishes on the same `sensors/lidar3d_<n>/…` names. A lidar
  of those makes goes there; this package carries what clearpath_config has no model for (RoboSense, 2.9.x).
- **The robot names its model in `robot.yaml`.** `platform.extras.launch` starts
  [husky_extras_bringup](../extras/README.md)'s `lidar.launch.py` with `lidar_model: rs16`, which includes the scan
  and with it this launch — run by Clearpath's own `clearpath-platform-extras` service. A test holds that name to
  the model the container's sim plants cast.
- **The RS16 as a URDF macro** (`src/rs16_description`): `rs16(name, parent_link)` puts `${name}_link` on the
  mounting face — RoboSense's CAD model in its own colors, a collision cylinder Ø109 × 80.7 mm, 0.87 kg — and
  `${name}_laser` 39 mm above it, at the optical center. Where the sensor is mounted is the robot's: `robot.yaml`
  places the frame `lidar3d_0_mount_link`, [onboard/extras](../extras/README.md) hangs the macro on it.
- **One driver configuration for the real sensor and the recording.** `rslidar_sdk` (apt, `ros-jazzy-rslidar-sdk`)
  reads the device on the robot and `data/recordings/rs16_labor.pcap` in the container; the difference is
  `common.msg_source`, rendered from the one template `config/rslidar.yaml`, which the model's `lidar_type` completes.
  A sim plant (ManiSkill, MuJoCo) casts the model's sweep in its world instead and publishes on the same topic.
- **The robot's own body out of the points.** `self_filter` (`python3 -m clair.lidar.self_filter`) drops the points
  inside the robot's collision hulls — the sensor arch beside it, an arm reaching out — and publishes the rest on
  `…/lidar3d_0/points_filtered`. It needs `clair-twin[body]`; without it, the points pass through unfiltered.
- **Names by slot, not by model** (`clair.lidar.wiring`): `lidar3d_0`, Clearpath's convention.
- **ROS-free core.** Names, models and the driver configuration are testable without ROS; the launch file imports
  them.

## Tech Stack

ROS 2 Jazzy · `rslidar_sdk` 1.5.16 · xacro · Python 3.12+

## Installation

Part of the clearpath uv workspace (`uv sync` at the root). The offboard image takes the directory as the build
context `onboard-lidar`: `rs16_description` is built with colcon, the Python source and `config/`, `launch/` go to
`/opt/clair/lidar`. On the robot the installer builds `~/onboard/lidar` with colcon (`robot.yaml` lists the workspace,
so the generator finds the macro) and puts `~/onboard/lidar/src` on the system Python's path with a `.pth`, as the
image does.

## Usage

```bash
ros2 launch /opt/clair/lidar/launch/lidar.launch.py pcap:=/data/recordings/rs16_labor.pcap   # container
ros2 launch ~/onboard/lidar/launch/lidar.launch.py model:=rs16                                # robot, by hand
```

The points alone. On the robot Clearpath's `clearpath-platform-extras` service starts navigation's `scan.launch.py`,
which includes this one, through `husky_extras_bringup`.

Another RoboSense model is an entry in `clair.lidar.models` (its `lidar_type`, rings, rate, blind zone) and a
description package beside `rs16_description`. A model of another make adds a driver module beside
`clair.lidar.rslidar` and its name to `models.DRIVERS`; `launch/lidar.launch.py` picks it by `model.driver`.

To rewrite the RS16 mesh from RoboSense's CAD model (`RS16.stp`, robosense.ai → Downloads → "RS-LiDAR-16 3D Model",
not in the repo):

```bash
uv run onboard/lidar/tools/rs16_mesh.py RS16.stp onboard/lidar/src/rs16_description/meshes/rs16.dae
```

## Running Tests

```bash
uv run pytest -c pyproject.toml onboard/lidar                       # root addopts: no ROS, no container
uv run pytest -c pyproject.toml -m e2e_container_nav onboard/lidar  # the driver replaying the recording
```

## Related

- [rs16_description](src/rs16_description/README.md) — the RS16's macro
- [navigation](../navigation/README.md) — the scan and Nav2 on these points
- [extras](../extras/README.md) — `husky_extras_bringup`, what robot.yaml starts
- [robot-contract profile](../../robot/contract/src/clair/robot/contract/profiles/a200_0553.yaml) — the topics as
  clients see them

## Versioning

[SemVer](https://semver.org/), history in the [CHANGELOG](CHANGELOG.md).

## License

See the workspace. The mesh is derived from RoboSense's CAD model of the RS-LiDAR-16.
