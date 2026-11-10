# husky_manipulator_diagnostics

The a200-0553's UR5 (CB3) and OnRobot RG6 as `diagnostic_msgs` on `/a200_0553/diagnostics`, the topic Clearpath's
`diagnostic_aggregator` reads. Without it the manipulator is missing from `diagnostics_agg` and so from Cockpit: the
generator writes no analyzers for it, the arm's `controller_manager` publishes into the manipulators namespace, and
the driver and the gripper bridge report in their own message types. Why each status is judged the way it is stands
in the module docstrings.

| Module | What it holds |
|---|---|
| `verdict.py` | the levels, `Verdict`, and the `display=inactive` convention for "out of service, not broken" |
| `arm.py` | robot/safety mode in plain text, powered or off on purpose, verdicts for mode, link, joints, controllers |
| `gripper.py` | the bridge's JSON state, the tool-signal check, the gripper's verdict and summary |
| `node.py` | the ROS node: subscriptions, `list_controllers` polling, one `DiagnosticArray` at `rate_hz` |

`verdict`, `arm` and `gripper` import no ROS, so the tests run on the workstation. The status names
(`manipulator_diagnostics: Arm Mode` and the four others) are what `setup/config/robot.yaml`'s analyzers and the
Cockpit panel match on; the node name `manipulator_diagnostics` is their prefix.

## Build and run

```bash
cd ~/husky && colcon build --packages-select husky_manipulator_diagnostics
python3 -m husky_manipulator_diagnostics.node --ros-args -p manipulator_ns:=/a200_0553/manipulators
```

On the robot `clearpath-custom-manipulator-diagnostics.service` starts it (husky/setup's installer, optional step),
hung off `clearpath-manipulators` so it restarts with the arm driver. Operation and checks:
[setup/README.md](../../setup/README.md).

## Tests

`tests/manipulator_diagnostics/` at the repository root, ROS-free: every verdict, including the cases measured on
the a200-0553.
