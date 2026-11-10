# husky — what runs on the a200-0553

What the a200-0553 itself runs, whether or not a workstation is switched on, and what describes this robot and no
other. What MARWIN 5 carries as well — the grippers and the tool macros — is the
[tools](https://github.com/CLAIRLab-HAW/tools) repository's; MARWIN 5's own description is
[marwin](https://github.com/CLAIRLab-HAW/marwin)'s.

| Directory | Was | What it is |
|---|---|---|
| [`setup/`](setup/README.md) | `husky-custom-setup` | The installer, the per-boot patcher, the boot services, the UR5 calibration — and **`robot.yaml`, the single source of truth** every other layer reads. |
| [`extras/`](extras/README.md) | `husky-extras` | This robot's URDF extras: sensor arch, RG6 on the UR5 flange, RS16 on the arch. One xacro, pulled in by the Clearpath generator through `robot.yaml`, and `husky_extras_bringup`, the lidar launch `robot.yaml` starts. |
| [`ur-state/`](ur-state/README.md) | `ur-state-manager` | Arm state (`prepare`, `recover`, `power_off`) and the controller-mode manager. |
| [`lidar/`](lidar/README.md) | `rs16` | The robot's one lidar, the RS-LiDAR-16 or the Blickfeld Qb2 in its place: models and drivers, their URDF macros (`rs16_description`, `qb2_description`) and the self filter — the points, raw and without the robot's body. |
| [`navigation/`](navigation/README.md) | `husky-navigation` | The 2D scan on the lidar's points and Nav2 — the same configuration in the container mock and on the robot. |
| [`cockpit-diagnostics/`](cockpit-diagnostics/README.md) | `cockpit-ros2-diagnostics` | A Cockpit plugin: the robot's ROS 2 diagnostics as a web panel. |
| [`cockpit-tools/`](cockpit-tools/README.md) | `cockpit-robot-tools` | A Cockpit page for the recurring manual tasks — today one card: start and stop the offboard `lite` container, with its VNC address. |

**On the robot this repository is the clone `/home/robot/husky`, beside `/home/robot/tools`.** The
[installer](setup/README.md) clones or pulls both, builds `tools/gripper/`, `tools/description/`, `lidar/`, `extras/`
and `ur-state/` in place and points `/etc/clearpath/robot.yaml` at `setup/config/robot.yaml`, whose absolute paths
name exactly those directories.

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
