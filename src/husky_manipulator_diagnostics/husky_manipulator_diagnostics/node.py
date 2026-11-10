"""manipulator_diagnostics: feed UR5 + OnRobot RG6 into the Clearpath
diagnostics pipeline as ``diagnostic_msgs``.

Why this is needed
------------------
The ``diagnostic_aggregator`` subscribes to ``<ns>/diagnostics`` and fans the
result out to ``<ns>/diagnostics_agg`` -- the topic the Cockpit extension
reads over the foxglove_bridge.  The manipulator is missing from that chain
entirely:

* ``clearpath_generator_common`` only generates analyzers for platform (power,
  e-stop, drive) and sensors -- arm and gripper do not appear in the generator.
* The ``controller_manager`` of the arm publishes its diagnostics into the
  *manipulators* namespace (``/a200_0553/manipulators/diagnostics``), so NOT
  onto the topic the aggregator subscribes to.
* The ``ur_robot_driver`` does not publish mode/safety/external control as
  ``diagnostic_msgs`` at all, but as its own ``ur_dashboard_msgs``.
* The RG6 state arrives as JSON from ``rg6_grip_bridge`` on
  ``rg6/bridge_state``, not as a typed message.

This node translates all of that into ``diagnostic_msgs/DiagnosticArray``. Together with the analyzer block that the
boot patcher (``clearpath-custom-setup.py``, step 6) writes into the generated ``diagnostic_aggregator.yaml``, the
manipulator thereby appears in *every* diagnostics consumer.

Delivered statuses (prefix = node name, as the analyzer expects it)
-------------------------------------------------------------------
``manipulator_diagnostics: Arm Mode``
    ``robot_mode`` + ``safety_mode`` (latched topics of the
    io_and_status_controller).
``manipulator_diagnostics: Arm Control``
    The *actual* health indicator: is the joint_state stream of the
    ``joint_state_broadcaster`` flowing?  It only flows with an active
    ros2_control hardware interface -- ``robot_program_running`` alone is NOT
    a valid signal (it stays true while the motion link is dead).
``manipulator_diagnostics: Arm Joints``
    Joint angles/velocities, rate, moving yes/no.
``manipulator_diagnostics: Arm Controllers``
    ``controller_manager/list_controllers`` -- which command controller is
    active, is one missing?
``manipulator_diagnostics: Gripper``
    RG6: width, force signal, grip_detected, busy, tool power, last command.

Invocation (service clearpath-custom-manipulator-diagnostics, see the installer)::

    python3 -m husky_manipulator_diagnostics.node --ros-args -p manipulator_ns:=/a200_0553/manipulators
"""

import math
import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .arm import (
    arm_control_level,
    arm_controllers_level,
    arm_is_off,
    arm_is_powered,
    arm_joints_level,
    arm_mode_level,
    robot_mode_name,
    safety_mode_name,
)
from .gripper import GRIPPER_COMMANDS, gripper_level, gripper_signal_valid, gripper_summary, parse_bridge_state
from .verdict import DISPLAY_INACTIVE, DISPLAY_KEY, ERROR, OK, STALE, Verdict

# Optional dependencies: if one is missing, ONLY the affected status drops out (with plain text), the rest keeps
# running.  That keeps the node startable on a robot without the RG6 workspace.
try:
    from ur_dashboard_msgs.msg import RobotMode, SafetyMode

    UR_MSGS_ERROR = None
except ImportError as exc:  # pragma: no cover - depends on the installation
    RobotMode = SafetyMode = None
    UR_MSGS_ERROR = str(exc)

# ToolDataMsg carries AI2/AI3 and the tool voltage -- the only gripper numbers that do NOT come from the bridge.
# rg6_msgs/GripperState is deliberately absent: that package is not in the boot path of the robot, and the state
# arrives as JSON from rg6_grip_bridge.
try:
    from ur_msgs.msg import ToolDataMsg

    TOOL_MSGS_ERROR = None
except ImportError as exc:  # pragma: no cover
    ToolDataMsg = None
    TOOL_MSGS_ERROR = str(exc)

try:
    from controller_manager_msgs.srv import ListControllers

    CM_MSGS_ERROR = None
except ImportError as exc:  # pragma: no cover
    ListControllers = None
    CM_MSGS_ERROR = str(exc)

# Latched (transient_local) + publish-on-change: exactly how the gpio_controller publishes
# robot_mode/safety_mode/robot_program_running.  A VOLATILE subscriber would miss the current value at startup and
# stay blind until the next change.
LATCHED = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def kv(values):
    """dict ─▶ KeyValue[]; everything as a string, as diagnostic_msgs wants."""
    return [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]


# DiagnosticStatus.level is a 'byte' in the .msg -- rosidl_generator_py maps that onto a bytes object of length 1,
# NOT onto int.  Instead of guessing, it is read off the real constant (which also holds if a future distro changes
# it).
LEVEL_AS_BYTES = isinstance(DiagnosticStatus.OK, bytes)


def level_field(level):
    return bytes([int(level)]) if LEVEL_AS_BYTES else int(level)


class ManipulatorDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("manipulator_diagnostics")

        ns = self.declare_parameter("manipulator_ns", "/a200_0553/manipulators").value.rstrip("/")
        self.diagnostics_topic = self.declare_parameter("diagnostics_topic", "/a200_0553/diagnostics").value
        self.rate_hz = float(self.declare_parameter("rate_hz", 1.0).value)
        self.arm_prefix = self.declare_parameter("arm_prefix", "arm_0_").value
        self.robot_ip = self.declare_parameter("robot_ip", "192.168.131.40").value
        # The joint_state stream runs at 125 Hz; 2 s of tolerance covers a zenoh hiccup without masking a real
        # breakdown.
        self.js_timeout = float(self.declare_parameter("joint_state_timeout", 2.0).value)
        self.gripper_timeout = float(self.declare_parameter("gripper_timeout", 2.0).value)
        self.stroke_m = float(self.declare_parameter("stroke_m", 0.160).value)
        # Below this voltage the RG6 delivers no valid tool signal (measured 2026-07-29: powered AI2 10.00 V,
        # unpowered ~0.056 V).
        self.dead_input_threshold = float(self.declare_parameter("dead_input_threshold", 0.2).value)
        # Noise of the joint velocity on a STANDING arm: powered exactly 0.0000, unpowered up to 0.055 rad/s
        # measured.  The threshold only covers the powered case -- unpowered, "moving" is suppressed anyway (the
        # status is 'out of service' then).
        self.motion_eps = float(self.declare_parameter("motion_eps_rad_s", 0.02).value)
        self.required_controllers = list(
            self.declare_parameter(
                "required_controllers",
                ["joint_state_broadcaster", "arm_0_joint_trajectory_controller", "io_and_status_controller"],
            ).value
        )
        self.controller_poll_s = float(self.declare_parameter("controller_poll_period", 5.0).value)
        self.arm_hardware_id = self.declare_parameter("arm_hardware_id", f"UR5 CB3 @ {self.robot_ip}").value
        self.gripper_hardware_id = self.declare_parameter("gripper_hardware_id", "OnRobot RG6 @ UR Tool-I/O").value

        # ---- state ---------------------------------------------------
        self._robot_mode = None
        self._safety_mode = None
        self._program_running = None
        self._js_stamps = deque(maxlen=64)  # monotonic receive times (arm)
        self._joints = {}  # short name ─▶ (pos, vel, eff)
        self._gripper = None  # last bridge state (dict)
        self._gripper_time = None
        self._tool = None  # last ToolDataMsg (AI2/AI3)
        self._controllers = None  # {name: state} | None
        self._controllers_time = None
        self._cm_future = None

        # ---- Subscriptions -------------------------------------------
        if RobotMode is not None:
            self.create_subscription(
                RobotMode, f"{ns}/io_and_status_controller/robot_mode", self._on_robot_mode, LATCHED
            )
            self.create_subscription(
                SafetyMode, f"{ns}/io_and_status_controller/safety_mode", self._on_safety_mode, LATCHED
            )
        self.create_subscription(
            Bool, f"{ns}/io_and_status_controller/robot_program_running", self._on_program_running, LATCHED
        )
        self.create_subscription(JointState, f"{ns}/joint_states", self._on_joint_states, 10)
        self.create_subscription(String, f"{ns}/rg6/bridge_state", self._on_gripper, 10)
        if ToolDataMsg is not None:
            self.create_subscription(ToolDataMsg, f"{ns}/io_and_status_controller/tool_data", self._on_tool_data, 10)

        # ---- controller_manager --------------------------------------
        self._cm_client = None
        if ListControllers is not None:
            self._cm_client = self.create_client(ListControllers, f"{ns}/controller_manager/list_controllers")
            self.create_timer(self.controller_poll_s, self._poll_controllers)

        # ---- output --------------------------------------------------
        self._pub = self.create_publisher(DiagnosticArray, self.diagnostics_topic, 10)
        self.create_timer(1.0 / max(self.rate_hz, 0.1), self._tick)

        self.get_logger().info(f"manipulator_diagnostics: {ns} ─▶ {self.diagnostics_topic} @ {self.rate_hz:.1f} Hz")
        for missing, what in (
            (UR_MSGS_ERROR, "ur_dashboard_msgs"),
            (TOOL_MSGS_ERROR, "ur_msgs"),
            (CM_MSGS_ERROR, "controller_manager_msgs"),
        ):
            if missing:
                self.get_logger().error(
                    f"{what} not importable ({missing}) - the corresponding "
                    "status reports that as an error. Workspace sourced?"
                )

    # ---- Callbacks ---------------------------------------------------
    def _on_robot_mode(self, msg) -> None:
        self._robot_mode = int(msg.mode)

    def _on_safety_mode(self, msg) -> None:
        self._safety_mode = int(msg.mode)

    def _on_program_running(self, msg: Bool) -> None:
        self._program_running = bool(msg.data)

    def _on_joint_states(self, msg: JointState) -> None:
        """Count arm joints only.

        TWO sources sit on ``<ns>/joint_states``: the arm JSB and the finger value of the gripper bridge.  For
        judging the motion link only the arm stream counts -- the bridge keeps polling its XML-RPC endpoint even
        when the arm hardware interface is dead, and would otherwise mask a breakdown.
        """
        found = False
        for i, name in enumerate(msg.name):
            if not name.startswith(self.arm_prefix):
                continue
            found = True
            self._joints[name[len(self.arm_prefix) :]] = (
                msg.position[i] if i < len(msg.position) else float("nan"),
                msg.velocity[i] if i < len(msg.velocity) else float("nan"),
                msg.effort[i] if i < len(msg.effort) else float("nan"),
            )
        if found:
            self._js_stamps.append(time.monotonic())

    def _on_gripper(self, msg) -> None:
        """JSON from rg6_grip_bridge.  Garbage does NOT change the state.

        The old timestamp then stays and ages -- so the panel reports "silent" instead of "OK with nonsense", and
        that is the right statement: an unreadable payload is not a state.
        """
        state = parse_bridge_state(msg.data)
        if state is None:
            self.get_logger().warn(f"unreadable rg6/bridge_state: {msg.data[:120]!r}", throttle_duration_sec=10.0)
            return
        self._gripper = state
        self._gripper_time = time.monotonic()

    def _on_tool_data(self, msg) -> None:
        self._tool = msg

    def _poll_controllers(self) -> None:
        """Query list_controllers asynchronously (never block in the timer)."""
        if self._cm_future is not None and not self._cm_future.done():
            return  # the previous call still hangs ─▶ the service is gone
        if not self._cm_client.service_is_ready():
            self._controllers = None
            return
        self._cm_future = self._cm_client.call_async(ListControllers.Request())
        self._cm_future.add_done_callback(self._on_controllers)

    def _on_controllers(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # defensive: never let the service callback die
            self.get_logger().warning(f"list_controllers failed: {exc}", throttle_duration_sec=30.0)
            self._controllers = None
            return
        self._controllers = {c.name: c.state for c in result.controller}
        self._controllers_time = time.monotonic()

    # ---- building the statuses ---------------------------------------
    def _age(self, stamp):
        return None if stamp is None else time.monotonic() - stamp

    def _joint_state_age(self):
        return self._age(self._js_stamps[-1] if self._js_stamps else None)

    def _joint_state_rate(self):
        """Rate from the time window of the last messages (0.0 = unknown)."""
        if len(self._js_stamps) < 2:
            return 0.0
        span = self._js_stamps[-1] - self._js_stamps[0]
        return (len(self._js_stamps) - 1) / span if span > 0.0 else 0.0

    def _status(self, name, verdict, hardware_id, values):
        """Verdict + values ─▶ DiagnosticStatus.

        'inactive' travels as a value (see DISPLAY_INACTIVE), not as an own level -- the level stays standard
        conformant.
        """
        status = DiagnosticStatus()
        status.name = f"{self.get_name()}: {name}"
        status.level = level_field(verdict.level)
        status.message = verdict.message
        status.hardware_id = hardware_id
        if verdict.inactive:
            values = dict(values, **{DISPLAY_KEY: DISPLAY_INACTIVE})
        status.values = kv(values)
        return status

    def _arm_off(self):
        return arm_is_off(self._robot_mode)

    def _arm_mode_status(self):
        if RobotMode is None:
            return self._status(
                "Arm Mode", Verdict(ERROR, f"ur_dashboard_msgs missing ({UR_MSGS_ERROR})"), self.arm_hardware_id, {}
            )
        verdict = arm_mode_level(self._robot_mode, self._safety_mode)
        powered = arm_is_powered(self._robot_mode)
        return self._status(
            "Arm Mode",
            verdict,
            self.arm_hardware_id,
            {
                "robot_mode": robot_mode_name(self._robot_mode),
                "robot_mode_id": self._robot_mode,
                "safety_mode": safety_mode_name(self._safety_mode),
                "safety_mode_id": self._safety_mode,
                # Derived for the UI: co-decides the gripper display.
                "arm_powered": ("unknown" if powered is None else str(powered).lower()),
                "robot_ip": self.robot_ip,
            },
        )

    def _arm_control_status(self):
        age = self._joint_state_age()
        verdict = arm_control_level(self._program_running, age, self.js_timeout, arm_off=self._arm_off())
        return self._status(
            "Arm Control",
            verdict,
            self.arm_hardware_id,
            {
                "external_control": {True: "running", False: "stopped", None: "unknown"}[self._program_running],
                "joint_state_rate_hz": f"{self._joint_state_rate():.1f}",
                "joint_state_age_s": "never" if age is None else f"{age:.2f}",
                "joint_state_timeout_s": f"{self.js_timeout:.1f}",
                "motion_interface": ("live" if age is not None and age <= self.js_timeout else "dead"),
            },
        )

    def _arm_joints_status(self):
        age = self._joint_state_age()
        arm_off = self._arm_off()
        verdict = arm_joints_level(len(self._joints), age, self.js_timeout, arm_off=arm_off)
        if verdict.level == STALE:
            return self._status(
                "Arm Joints",
                verdict,
                self.arm_hardware_id,
                {"joints": "", "joint_state_age_s": "never" if age is None else f"{age:.2f}"},
            )

        values = {"joints": ", ".join(self._joints)}
        moving = False
        for short, (pos, vel, eff) in self._joints.items():
            values[f"{short}_rad"] = f"{pos:.4f}"
            values[f"{short}_deg"] = f"{math.degrees(pos):.1f}"
            values[f"{short}_vel_rad_s"] = f"{vel:.4f}"
            if math.isfinite(eff):
                values[f"{short}_effort"] = f"{eff:.2f}"
            if math.isfinite(vel) and abs(vel) > self.motion_eps:
                moving = True
        # On an unpowered arm the velocity is pure noise -- the statement about motion would be made up.
        values["moving"] = "unknown" if arm_off else str(moving).lower()
        values["rate_hz"] = f"{self._joint_state_rate():.1f}"
        if not arm_off:
            verdict = Verdict(verdict.level, f"{verdict.message}, " + ("moving" if moving else "at rest"))
        return self._status("Arm Joints", verdict, self.arm_hardware_id, values)

    def _arm_controllers_status(self):
        if ListControllers is None:
            return self._status(
                "Arm Controllers",
                Verdict(ERROR, f"controller_manager_msgs missing ({CM_MSGS_ERROR})"),
                self.arm_hardware_id,
                {},
            )
        verdict = arm_controllers_level(self._controllers, self.required_controllers, arm_off=self._arm_off())
        values = {"required": ", ".join(self.required_controllers)}
        if self._controllers:
            values.update(self._controllers)
            active = [n for n, s in self._controllers.items() if s == "active" and n not in self.required_controllers]
            values["active_optional"] = ", ".join(sorted(active)) or "-"
        return self._status("Arm Controllers", verdict, self.arm_hardware_id, values)

    def _gripper_status(self):
        """Two sources, deliberately kept apart.

        The STATE (width, busy, grip_detected) comes from the bridge and thus from the device itself.  The VOLTAGES
        AI2/AI3 come from ``tool_data`` and answer a different question: is there any supply at the tool connector
        at all?  Merging them here would make the AI2 curve -- measured on 2026-08-19 as mis-calibrated by up to 17
        mm (R19) -- a width source again.
        """
        unknown = "unknown"
        age = self._age(self._gripper_time)
        state = self._gripper
        tool = self._tool
        dead = self.dead_input_threshold
        width_raw = None if tool is None else float(tool.analog_input2)
        force_raw = None if tool is None else float(tool.analog_input3)
        # Base validity exclusively on the analog signal: it is the only HARDWARE feedback about the tool supply.
        valid = gripper_signal_valid(width_raw, force_raw, dead)
        verdict = gripper_level(age, self.gripper_timeout, valid, self._robot_mode, width_raw, dead)
        if state is None:
            return self._status(
                "Gripper",
                verdict,
                self.gripper_hardware_id,
                {"state_age_s": "never", "width_raw_v": (unknown if width_raw is None else f"{width_raw:.3f}")},
            )

        width = state["width_m"] if valid else None
        if verdict.level == OK and not verdict.inactive:
            verdict = Verdict(
                verdict.level, gripper_summary(width, state["grip_detected"], state["busy"], self.stroke_m)
            )
        last = state["last_command"]
        values = {
            "width_m": unknown if width is None else f"{width:.4f}",
            "width_mm": unknown if width is None else f"{width * 1000.0:.1f}",
            "width_percent": (
                unknown if width is None or self.stroke_m <= 0.0 else f"{100.0 * width / self.stroke_m:.0f}"
            ),
            "stroke_mm": f"{self.stroke_m * 1000.0:.0f}",
            # Always show the raw values: they are the diagnosis itself. They do NOT come from the same source as
            # the width -- which is what makes them useful as a cross-check.
            "width_raw_v": unknown if width_raw is None else f"{width_raw:.3f}",
            "force_raw_v": unknown if force_raw is None else f"{force_raw:.3f}",
            "signal_valid": str(valid).lower(),
            "dead_input_threshold_v": f"{dead:.2f}",
            # Without tool voltage the device reports nothing trustworthy either.
            "grip_detected": (
                unknown if not valid or state["grip_detected"] is None else str(state["grip_detected"]).lower()
            ),
            "busy": (unknown if not valid or state["busy"] is None else str(state["busy"]).lower()),
            # Device status, straight from the endpoint (rg_get_status / rg_get_safety_failed).
            "device_status": (unknown if state["status"] is None else str(state["status"])),
            "safety_failed": (unknown if state["safety_failed"] is None else str(state["safety_failed"]).lower()),
            "last_command": (unknown if last is None else GRIPPER_COMMANDS.get(last, str(last))),
            # REAL hardware feedback: the measured voltage, not a commanded setpoint.
            "tool_output_voltage_v": (unknown if tool is None else f"{float(tool.tool_output_voltage):.0f}"),
            "state_age_s": f"{age:.2f}",
        }
        return self._status("Gripper", verdict, self.gripper_hardware_id, values)

    def _tick(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [
            self._arm_mode_status(),
            self._arm_control_status(),
            self._arm_joints_status(),
            self._arm_controllers_status(),
            self._gripper_status(),
        ]
        self._pub.publish(array)


def main() -> int:
    rclpy.init()
    node = ManipulatorDiagnostics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # normal stop (Ctrl+C / systemd)
    except Exception:
        # SIGTERM shutdown race as in octomap_feed: rclpy's signal handler invalidates the context while spin is still
        # building a wait set.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
