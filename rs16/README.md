# rs16

The RoboSense **RS-LiDAR-16** of the Husky **a200-0553**: its description, its driver configuration and its point
cloud, raw and without the robot's own body. Runs in the offboard container against a recording **and** onboard on
the real robot — same configuration, same driver.

This is the sensor and nothing that consumes it. Navigation derives its 2D scan from these points
([navigation](../navigation/README.md), `launch/scan.launch.py`), perception reads them as they are; both import the
names from here, and neither from the other.

## Features

- **The sensor as a URDF macro** (`src/rs16_description`): `rs16(name, parent_link)` puts `${name}_link` on the
  mounting face — RoboSense's CAD model in its own colors, a collision cylinder Ø109 × 80.7 mm, 0.87 kg — and
  `${name}_laser` 39 mm above it, at the optical center. Where the sensor is mounted is the robot's: `robot.yaml`
  places the frame `lidar3d_0_mount_link`, [husky-extras](../extras/README.md) hangs the macro on it.
- **One driver for the real sensor.** `rslidar_sdk` (apt, `ros-jazzy-rslidar-sdk`) reads the device on the robot and
  the recording `data/recordings/rs16_labor.pcap` in the container; the difference is `common.msg_source`, rendered
  from the one template `config/rslidar_rs16.yaml` by `clair.rs16.rslidar_config`. A sim plant (ManiSkill, MuJoCo)
  casts the RS16 in its world instead, with the sweep `rslidar_config` states, and publishes on the same topic.
- **The robot's own body out of the points.** `self_filter` (`python3 -m clair.rs16.self_filter`) drops the points
  inside the robot's collision hulls — the sensor arch beside it, an arm reaching out — and publishes the rest on
  `…/lidar3d_0/points_filtered`. It needs `clair-twin[body]`; without it, the points pass through unfiltered.
- **ROS-free core.** The names (`clair.rs16.wiring`) and the driver configuration are testable without ROS; the
  launch file imports them.

## Tech Stack

ROS 2 Jazzy · `rslidar_sdk` 1.5.16 · xacro · Python 3.12+

## Installation

Part of the clearpath uv workspace (`uv sync` at the root). The offboard image takes the directory as the build
context `rs16`: `rs16_description` is built with colcon, the Python source and `config/`, `launch/` go to
`/opt/clair/rs16`. On the robot the installer builds `~/onboard/rs16` with colcon, and `robot.yaml` lists the
workspace, so the generator finds the macro.

## Usage

```bash
ros2 launch /opt/clair/rs16/launch/lidar.launch.py pcap:=/data/recordings/rs16_labor.pcap   # container
ros2 launch ~/onboard/rs16/launch/lidar.launch.py                                          # robot, the device
```

The points alone; the robot's lidar service starts navigation's `scan.launch.py`, which includes this one.

To rewrite the mesh from RoboSense's CAD model (`RS16.stp`, robosense.ai → Downloads → "RS-LiDAR-16 3D Model", not in
the repo):

```bash
uv run onboard/rs16/tools/rs16_mesh.py RS16.stp onboard/rs16/src/rs16_description/meshes/rs16.dae
```

## Running Tests

```bash
uv run pytest -c pyproject.toml onboard/rs16                       # root addopts: no ROS, no container
uv run pytest -c pyproject.toml -m e2e_container_nav onboard/rs16  # the driver replaying the recording
```

## Related

- [rs16_description](src/rs16_description/README.md) — the macro
- [navigation](../navigation/README.md) — the scan and Nav2 on these points
- [robot-contract profile](../../robot/contract/src/clair/robot/contract/profiles/a200_0553.yaml) — the topics as
  clients see them

## Versioning

[SemVer](https://semver.org/), history in the [CHANGELOG](CHANGELOG.md).

## License

See the workspace. The mesh is derived from RoboSense's CAD model of the RS-LiDAR-16.
