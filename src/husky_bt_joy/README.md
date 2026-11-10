# husky_bt_joy

Echo for Android as the a200-0553's Bluetooth joystick. The robot has Bluetooth but no screen, so a phone's pairing
request waits for an agent that never answers; `bt_joy_pairing` is that agent, and a narrow one. Why it is gated the
way it is stands in the module docstring of `husky_bt_joy/pairing.py`.

| What | Where |
|---|---|
| `<ns>/bt_joy/open_pairing` (`std_srvs/Trigger`) | opens a 60 s pairing window, answers flat JSON |
| HID only | `AuthorizeService` lets the HID profile through from a paired device and trusts it, refuses the rest |
| The phone's teleop | a second `joy_node` on `/dev/input/bt_joy` and `teleop_twist_joy` onto `joy_teleop/cmd_vel` |
| `launch/bt_joy.launch.py` | what `robot.yaml`'s `platform.extras.launch` starts, with `namespace`; respawns the node |

**The installer's D-Bus policy is the opt-in.** `/etc/dbus-1/system.d/clearpath-custom-bt-joy.conf` lets the robot
user talk to `org.bluez`; without it the agent could not register, so the launch starts nothing and says so. The
installer's optional pairing step writes it; operating it from the phone and checking it on the robot are in
[setup/README.md](../../setup/README.md).

## Build and run

```bash
cd ~/husky && colcon build --packages-select husky_bt_joy
ros2 run husky_bt_joy bt_joy_pairing --ros-args -r __ns:=/a200_0553 -p window_s:=60
```

Parameters: `window_s` (default 60), `teleop_config` (default `/etc/clearpath/platform/config/teleop_joy.yaml`, the
generated file whose `joy_node` and `teleop_twist_joy_node` parameters the phone's chain takes).

## Tests

`tests/bt_joy/` at the repository root: the window, the HID gate, the reply the app parses, the phone's teleop
parameters, and that the launch waits for the policy path the installer writes. ROS-free.
