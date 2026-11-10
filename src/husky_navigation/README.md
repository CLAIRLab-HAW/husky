# husky_navigation

Provides the mobile-base capability; locomotion remains outside the first manipulation milestone.
See the [physical-intelligence architecture](../../docs/COGNITIVE-ARCHITECTURE.md).

Navigation layer of the Husky **a200-0553**: the 2D scan on the RoboSense RS-LiDAR-16's points, and Nav2. Runs in
the offboard container against the mock platform **and** onboard on the real robot — same configuration. The sensor
itself — models, driver, self filter — is [husky_lidar](../../src/husky_lidar/README.md).

## Features

- **The scan on the lidar's points.** `launch/scan.launch.py` includes husky_lidar's `lidar.launch.py` (driver
  and self filter, for `lidar_model:=`) and adds `pointcloud_to_laserscan` on `…/lidar3d_0/points_filtered`, the
  points without the robot's body: AMCL and slam_toolbox read nothing but a LaserScan. On the robot Clearpath's
  `clearpath-platform-extras` service starts this launch (robot.yaml, `platform.extras.launch`), so the scan is there
  before Nav2 is.
- **Nav2** with switchable localization (`localization:=none|slam|amcl`): a static identity `map→odom`,
  `slam_toolbox` while mapping, or AMCL against a stored map.
- **ROS-free core.** The decisions live in `src/clair/navigation/` and are
  testable without ROS; the launch files import them.

## Status

**Nav2 drives in the container, not yet on the robot.** Evidenced, not claimed: on the plain mock a
`NavigateToPose` over 1 m ends with `SUCCEEDED`, and the EKF odometry before and after confirms the distance; on the
ManiSkill stack it reaches goals around the furniture of an ArchitecTHOR scene (see the CHANGELOG). On the robot the
RS16 chain runs as a service of the installer (`scan.launch.py`, scans at 10.6 Hz), but `nav` has no start path
there and there is no map of the lab yet (R36).

**Localization is wheel odometry everywhere; what corrects it depends on where it runs.** `start_nav` in `clair.stack`
passes `localization:=slam` to a sim plant with the lidar on; everything else gets the launch default `none`.

| Where | `odom→base_link` (EKF input) | `map→odom` | Obstacles in the costmaps |
|---|---|---|---|
| plain mock (`plant:=mock`) | mock wheels | static identity | none, synthetic map |
| sim plant, lidar on (the default) | mock wheels + sim UM7 yaw rate | `slam_toolbox` | the cast RS16 |
| sim plant, `--no-lidar` | mock wheels + sim UM7 yaw rate | static identity | none |
| real robot | wheel encoders (UM7 meant, not wired: R66) | — (Nav2 not started) | — |

**The sim's odometry errs as the robot's does.** The mock wheels (`mock_components/GenericSystem`) turn exactly as
commanded, and the world does not follow the odometry but `base-truth`, which moves the body for the same wheel speeds
with the robot profile's skid steer (`sim.base.true_separation_multiplier`, 1.875 × 1.10 until R37 measures it). So
the diff drive's odometry turns 10 % too far, the slip scatters about that by the profile's sigmas, and SLAM has
something to correct; the RS16's ranges err by 3 cm. `up.sh --sim-errors off` makes the sim plant exact, live.
`imu-sim` publishes the UM7 on `sensors/imu_0/data` from the world's motion, and the container's EKF fuses its yaw
rate. Clearpath's generator leaves the `imu0` topic out whenever `robot.yaml` carries `ekf_node` extras, which ours
do (R66), so the container adds it for the sim plants — **the robot's generated `localization.yaml` lacks it too**
(R66).

**Standing still, the sim's scan turns against the map; the robot's does not, yet.** The EKF weighs the UM7's yaw
rate about 9 000 times the wheels' (the driver's `angular_velocity_stdev` against the diff drive's twist covariance),
so at standstill it integrates the gyro's zeroed residual (`sim.imu.gyro_zeroed_residual_rad_s`, an assumption until
R75) — measured 5.6·10⁻⁵ rad/s, 3.2° after 13 minutes. `slam_toolbox` matches a scan only after 0.3 m or 0.3 rad of
odometry travel, so meanwhile `map→odom` stands and the scan turns against the map, until the drift itself reaches
0.3 rad. The robot's EKF fuses no IMU today (R66) and so has no such drift; once `imu0` is wired there, it has the
sim's. The ground truth is on `/sim/base_truth` and, with ManiSkill, the world's root on `/sim/base_pose`; neither
reaches Nav2.

## Frames — the robot sits in the floor, and that is not one

In RViz and Foxglove the Husky visibly stands **10.1 cm below** the map's
ground plane. That looks like a broken URDF and is not one:

```
base_link → base_footprint    z = −0.101       (URDF, ride-height patch)
wheel axle → base_link        z = +0.033       (URDF)
loaded wheel radius               0.134        (hubs 13–14 cm above the floor)
                              0.033 − 0.134 = −0.101   ✓
odom      → base_link         z =  0.000       (EKF)
map       → base_footprint    z = −0.101       ← hence
```

So `base_footprint` is defined **correctly** — exactly where the loaded wheels touch the ground.

That is Clearpath's convention from the generated `localization.yaml`. It is
not settable via `robot.yaml` (which only carries `enable_ekf: true`), and
patching the generated file would be exactly the drift source this workspace
avoids. **For Nav2 it has no consequence** — what counts there is x, y and
yaw.

Two places where it does count after all:

- **Height bands are measured from `base_link`, not from the ground.**
  `pointcloud_to_laserscan` filters with `min_height: -0.07`; that is 3 cm
  above the ground, not 7 cm below it. It is written down in
  `husky_navigation.wiring` together with this calculation.
- **A ground plane as a collision object at `map` z = 0 would sit 10.1 cm too
  high** — in the middle of the robot. Anyone adding one for the arm puts it
  on `base_footprint`.

`test_the_ground_frame_and_the_odometry_carry_the_measured_wheel` pins the two radii of the same tire, both
measured on the robot (R58): the LOADED one (0.134 m) between axle and `base_footprint`, and the ROLLING one
(0.1477 m, a 2.515 m run against 17.026 rad of wheel turn) that `robot.yaml` gives the DiffDriveController as
`wheel_radius` in place of Clearpath's nominal 0.1651. Whoever changes the wheels measures both again.

## Tech Stack

ROS 2 Jazzy · `nav2` · `slam_toolbox` · `pointcloud_to_laserscan` · [husky_lidar](../../src/husky_lidar/README.md) ·
Python 3.12+

## Installation

Part of the clearpath uv workspace (`uv sync` at the root). The repo gets into
the offboard image via `additional_contexts` during `docker compose build`.

## Usage

See `deploy/offboard/scripts/nav` in the container.

## Running Tests

```bash
uv run pytest tests/navigation            # from the husky root: without ROS, without container
```

The end-to-end tests against the driving base in the offboard container live in clearpath's `deploy/offboard/tests`
(`uv run pytest -m e2e_container_nav deploy/offboard/tests` there).

## Related

- [husky_lidar](../../src/husky_lidar/README.md) — the sensor the scan is derived from
- [Design spec](../../docs/superpowers/specs/2026-08-22-nav2-rs16-design.md)
- [robot-contract profile](../../robot/contract/src/clair/robot/contract/profiles/bindings/a200_0553.ros2.yaml)

## Versioning

[SemVer](https://semver.org/), history in the [CHANGELOG](CHANGELOG.md).

## License

See the workspace.
