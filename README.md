# husky — what runs on the a200-0553

What the a200-0553 itself runs, whether or not a workstation is switched on, and what describes this robot and no
other. What MARWIN 5 carries as well — the grippers and the tool macros — is the
[tools](https://github.com/CLAIRLab-HAW/tools) repository's; MARWIN 5's own description is
[marwin](https://github.com/CLAIRLab-HAW/marwin)'s.

One colcon workspace under `src/`, and beside it what colcon does not build:

| Directory | What it is |
|---|---|
| [`src/husky_extras_description`](src/husky_extras_description/README.md) | This robot's URDF extras: sensor arch, the tool on the UR5 flange (tool_description's macro), the lidar on the arch. One xacro, pulled in by the Clearpath generator through `robot.yaml`. |
| `src/husky_bringup` | The launches `robot.yaml` starts under `platform.extras.launch`: the lidar and its scan, the Bluetooth joystick pairing. |
| [`src/husky_lidar`](src/husky_lidar/README.md) | The robot's one lidar, the RS-LiDAR-16 or the Blickfeld Qb2 in its place: models and drivers, the sensor's names and the self filter on the robot's own hulls. |
| [`src/rs16_description`](src/rs16_description/README.md), [`src/qb2_description`](src/qb2_description/README.md) | The two lidars' URDF macros and meshes. |
| [`src/husky_navigation`](src/husky_navigation/README.md) | The 2D scan on the lidar's points and Nav2 — the same configuration in the container mock and on the robot. |
| [`src/husky_ur_state`](src/husky_ur_state/README.md) | Arm state (`prepare`, `recover`, `power_off`) and the controller-mode manager; its node is `ur_state_manager`. |
| [`setup/`](setup/README.md) | The installer, the per-boot patcher, the boot services, the UR5 calibration — and **`robot.yaml`, the single source of truth** every other layer reads. |
| [`cockpit-diagnostics/`](cockpit-diagnostics/README.md) | A Cockpit plugin: the robot's ROS 2 diagnostics as a web panel. |
| [`cockpit-tools/`](cockpit-tools/README.md) | A Cockpit page for the recurring manual tasks — today one card: start and stop the offboard `lite` container, with its VNC address. |
| `scripts/` | The mesh generators behind `husky_extras_description`'s and `rs16_description`'s meshes (not installed). |
| `tests/` | The packages' suites, without ROS (`setup/tests` beside the installer). |

**On the robot this repository is the clone `/home/robot/husky`, beside `/home/robot/tools`.** The
[installer](setup/README.md) clones or pulls both, builds each as one colcon workspace and points
`/etc/clearpath/robot.yaml` at `setup/config/robot.yaml`, which lists the two install spaces.

## Running tests

```bash
uv sync
uv run ruff check . && uv run ruff format --check --extend-exclude cockpit-diagnostics .
uv run pytest                             # every package's suite, without ROS and without a container
```

This repository needs no workspace around it: `pyproject.toml` here is a uv workspace over `lidar` and `navigation`,
with its own `uv.lock` and `ruff.toml`. Tests that compare a file here against one in a neighboring clearpath
repository skip with a named cause when there is none. Tests that import clearpath packages, and the end-to-end tests
against the offboard container, live in the clearpath workspace (`deploy/offboard/tests`, `robot/contract/tests`,
`twin/twinlink/tests`).

## Versioning

Each package keeps its own `CHANGELOG.md` and its own version; this directory's
[CHANGELOG.md](CHANGELOG.md) records what happens to the repository as a whole.
