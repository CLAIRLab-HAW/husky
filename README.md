# husky

ROS 2 Jazzy packages, configuration and installer for the Clearpath Husky A200 `a200-0553` with a UR5 (CB3) arm.

## Layout

| Path | Type | Contents |
|---|---|---|
| `src/husky_extras_description` | ament_cmake | URDF extras: sensor arch, end effector, lidar mount, charging port |
| `src/husky_bringup` | ament_cmake | Launch file for `platform.extras.launch`: the lidar chain |
| `src/husky_bt_joy` | ament_python | The phone as Bluetooth joystick: BlueZ pairing agent, its teleop chain |
| `src/husky_manipulator_diagnostics` | ament_python | UR5 + RG6 as `diagnostic_msgs` for Cockpit/`diagnostics_agg` |
| `src/husky_lidar` | ament_python | Lidar models (RS16, Qb2), driver configuration, topic names, self filter |
| `src/husky_navigation` | ament_python | 2D scan from the lidar points, Nav2 and slam_toolbox configuration, maps |
| `src/husky_moveit_config` | ament_cmake | The MoveIt viewer's RViz config and marker kinematics; the rest is generated |
| `src/husky_ur_state` | ament_python | Arm state services (`prepare`, `recover`, `power_off`), controller modes |
| `src/rs16_description` | ament_cmake | RS-LiDAR-16 URDF macro and mesh |
| `src/qb2_description` | ament_cmake | Blickfeld Qb2 URDF macro |
| `setup/` | — | `config/robot.yaml`, UR5 calibration, installer, per-boot patchers, systemd units, operator tools |
| `cockpit-diagnostics/` | npm | Cockpit plugin: ROS 2 diagnostics |
| `cockpit-tools/` | npm | Cockpit page: offboard container control |
| `tests/` | — | pytest suites of the packages (`setup/tests` for the installer and patchers) |

## Configuration

`setup/config/robot.yaml` is the robot's Clearpath configuration; `/etc/clearpath/robot.yaml` is a symlink to it.

| Key | Value |
|---|---|
| `system.ros2.workspaces` | `/home/robot/tools/install/setup.bash`, `/home/robot/husky/install/setup.bash` |
| `platform.extras.urdf.path` | `/home/robot/husky/src/husky_extras_description/urdf/clearpath_extras.urdf.xacro` |
| `platform.extras.launch` | `husky_bringup`'s `lidar.launch.py` (`lidar_model`), `husky_bt_joy`'s `bt_joy.launch.py` |
| `system.ros2.middleware` | `rmw_zenoh_cpp` |

## Dependencies

Run time, not in this repository: `rg6_description`, `rg6_control`, `gripper_control`, `tool_description`
(workspace `/home/robot/tools`); `rslidar_sdk`, `pointcloud_to_laserscan`, `nav2_bringup`, `slam_toolbox`,
`ur_robot_driver`, `geometric_shapes` (apt).

## Installation

On the robot, as user `robot`:

```bash
git clone https://github.com/CLAIRLab-HAW/husky.git ~/husky
sudo bash ~/husky/setup/install-clearpath-custom-setup.sh            # interactive; -y answers yes
sudo bash ~/husky/setup/install-clearpath-custom-setup.sh --verify   # deployed copies against the checkout
```

The installer clones or pulls `~/husky` and `~/tools`, builds both with colcon, links `/etc/clearpath/robot.yaml`,
and installs the systemd units and the root-owned patcher copies. Details: [setup/README.md](setup/README.md).

Manual build:

```bash
cd ~/husky && colcon build --packages-select husky_extras_description husky_bringup husky_bt_joy \
  rs16_description qb2_description husky_lidar husky_navigation husky_ur_state husky_manipulator_diagnostics \
  husky_moveit_config
```

## Tests

```bash
uv sync
uv run ruff check --extend-exclude cockpit-diagnostics .
uv run ruff format --check --extend-exclude cockpit-diagnostics .
uv run pytest
```

No ROS required. Tests that compare against files outside this repository skip when those files are absent.

## Versioning

Each package keeps its own `CHANGELOG.md`; [CHANGELOG.md](CHANGELOG.md) covers the repository.

## License

MIT, unless a package states otherwise.
