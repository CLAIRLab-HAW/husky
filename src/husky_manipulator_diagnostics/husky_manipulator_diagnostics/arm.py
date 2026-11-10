"""The UR5 side: robot_mode/safety_mode in plain text, whether the arm is powered or off on purpose, and the
verdicts for its mode, its motion link, its joints and its controllers.  ROS-free."""

from .verdict import ERROR, OK, STALE, WARN, Verdict, inactive

# The ur_dashboard_msgs/RobotMode and SafetyMode values in plain text, duplicated here so the evaluation and its tests
# run without ROS; the node receives only the integers.
ROBOT_MODE_NAMES = {
    -1: "NO_CONTROLLER",
    0: "DISCONNECTED",
    1: "CONFIRM_SAFETY",
    2: "BOOTING",
    3: "POWER_OFF",
    4: "POWER_ON",
    5: "IDLE",
    6: "BACKDRIVE",
    7: "RUNNING",
    8: "UPDATING_FIRMWARE",
}

SAFETY_MODE_NAMES = {
    1: "NORMAL",
    2: "REDUCED",
    3: "PROTECTIVE_STOP",
    4: "RECOVERY",
    5: "SAFEGUARD_STOP",
    6: "SYSTEM_EMERGENCY_STOP",
    7: "ROBOT_EMERGENCY_STOP",
    8: "VIOLATION",
    9: "FAULT",
    10: "VALIDATE_JOINT_ID",
    11: "UNDEFINED_SAFETY_MODE",
}

# Safety states that require intervention ─▶ ERROR.
SAFETY_ERROR = {3, 5, 6, 7, 8, 9}
# Safety states that restrict operation ─▶ WARN.
SAFETY_WARN = {2, 4, 10, 11}

# An e-stop can NOT be released by software ─▶ its own plain-text message.
SAFETY_ESTOP = {6, 7}


# robot_mode values in which the arm is powered -- and in which the 24 V tool supply of the RG6 CAN therefore be present
# at all (the gripper hangs off the UR tool connector).  BACKDRIVE (freedrive) counts: motors unpowered, but controller
# and tool connector supplied.
ARM_POWERED_MODES = {4, 5, 6, 7}  # POWER_ON, IDLE, BACKDRIVE, RUNNING

# POWER_OFF is an OPERATOR DECISION, not an error: the arm was switched off on purpose (maintenance/end of day).
# Everything that follows from it -- no external control, no trustworthy joint values, an unpowered gripper -- is then
# as expected and is reported as "out of service" (grey), not as a warning.  The manipulators watchdog behaves the same
# way: at POWER_OFF no recovery runs.
ARM_OFF_MODE = 3  # POWER_OFF


def arm_is_powered(robot_mode):
    """Can there be any voltage at the tool connector? ─▶ True/False/None."""
    if robot_mode is None:
        return None
    return robot_mode in ARM_POWERED_MODES


def arm_is_off(robot_mode):
    """Switched off on purpose (POWER_OFF)? ─▶ bool."""
    return robot_mode == ARM_OFF_MODE


def robot_mode_name(mode):
    """Int ─▶ plain text, unknown values stay readable."""
    if mode is None:
        return "UNKNOWN"
    return ROBOT_MODE_NAMES.get(mode, f"UNKNOWN({mode})")


def safety_mode_name(mode):
    if mode is None:
        return "UNKNOWN"
    return SAFETY_MODE_NAMES.get(mode, f"UNKNOWN({mode})")


def arm_mode_level(robot_mode, safety_mode):
    """Evaluation of robot_mode/safety_mode ─▶ Verdict.

    Both topics are latched and only published on change -- a ``None`` therefore means "never received"
    (driver/controller absent), not "stale". Hence STALE instead of ERROR: the information is missing, which does not
    necessarily mean the arm is broken.
    """
    if robot_mode is None and safety_mode is None:
        return Verdict(STALE, "no robot_mode/safety_mode received - is the io_and_status_controller running?")

    rm, sm = robot_mode_name(robot_mode), safety_mode_name(safety_mode)

    if safety_mode in SAFETY_ESTOP:
        return Verdict(ERROR, f"emergency stop active ({sm}) - releasable only physically")
    if safety_mode in SAFETY_ERROR:
        return Verdict(ERROR, f"safety stop: {sm} (robot_mode {rm})")
    if robot_mode is not None and robot_mode < 3:
        # NO_CONTROLLER / DISCONNECTED / CONFIRM_SAFETY: no connection to the control box, or confirmation at the teach
        # pendant required.
        return Verdict(ERROR, f"arm not reachable: {rm}")
    if safety_mode in SAFETY_WARN:
        return Verdict(WARN, f"safety restricted: {sm} (robot_mode {rm})")
    if robot_mode == 7:
        return Verdict(OK, f"{rm}, safety {sm}")
    if arm_is_off(robot_mode):
        # Switched off on purpose ─▶ grey, no warning (see ARM_OFF_MODE).
        return inactive(f"arm switched off ({rm})")
    # BOOTING / POWER_ON / IDLE / BACKDRIVE / UPDATING_FIRMWARE: a transitional or special state in which the arm is not
    # ready to be driven from ROS.
    return Verdict(WARN, f"not ready to drive: {rm} (safety {sm})")


def arm_control_level(program_running, joint_state_age, timeout, arm_off=False):
    """Evaluation of the motion link ─▶ Verdict.

    ``joint_state_age`` is the age of the last joint_states message carrying arm joints (``None`` = never received one).
    That stream is the trustworthy signal: it only flows with an active ros2_control hardware interface, whereas
    ``program_running`` can wrongly stay true.

    With the arm switched off, BOTH are as expected -- turning that into a warning or even an error paints the panel red
    on a deliberately powered down arm.  The watchdog deliberately keeps its hands off at POWER_OFF too.
    """
    if arm_off:
        return inactive("arm switched off - external control stopped as expected")
    if joint_state_age is None:
        return Verdict(
            ERROR,
            "no joint_state stream from the arm - hardware interface "
            "not activated (arm powered up late? "
            "restart clearpath-manipulators)",
        )
    if joint_state_age > timeout:
        return Verdict(ERROR, f"joint_state stream silent for {joint_state_age:.1f}s - motion link dead")
    if program_running is False:
        return Verdict(WARN, "external control not running (arm not commandable from ROS)")
    if program_running is None:
        return Verdict(WARN, "external control status unknown (robot_program_running missing)")
    return Verdict(OK, "external control active, joint_state stream running")


def arm_joints_level(joint_count, joint_state_age, timeout, arm_off=False):
    """Evaluation of the joint values ─▶ Verdict.

    With the arm switched off the POSITIONS stay valid (absolute encoders), but velocity and effort are only noise --
    measured up to 0.05 rad/s with the arm completely at rest.  Hence grey instead of green: the numbers are there, but
    "moving"/"at rest" says nothing.
    """
    if not joint_count or joint_state_age is None or joint_state_age > timeout:
        return Verdict(STALE, "no current joint values")
    if arm_off:
        return inactive(f"arm switched off - {joint_count} joints, values are the last encoder positions")
    return Verdict(OK, f"{joint_count} joints")


def arm_controllers_level(controllers, required, arm_off=False):
    """Evaluation of list_controllers ─▶ Verdict.

    ``controllers``: ``{name: state}`` or ``None`` (service unreachable). ``required``: controllers that MUST be active
    (broadcasters plus the default command controller).  Inactive command controllers are explicitly normal -- the
    controller_mode_manager keeps the mutually exclusive modes parked.

    Real controller problems stay WARN/ERROR even with the arm switched off (they concern the ROS side, not the power);
    only the good case turns grey, so that the arm tile shows "out of service" as a whole.
    """
    if controllers is None:
        return Verdict(STALE, "controller_manager/list_controllers unreachable")
    if not controllers:
        return Verdict(ERROR, "controller_manager knows no controllers")

    missing = [c for c in required if c not in controllers]
    stopped = [c for c in required if controllers.get(c) not in (None, "active")]
    if missing:
        return Verdict(ERROR, "controllers missing: " + ", ".join(sorted(missing)))
    if stopped:
        return Verdict(ERROR, "controllers not active: " + ", ".join(sorted(stopped)))

    unconfigured = sorted(n for n, s in controllers.items() if s not in ("active", "inactive"))
    if unconfigured:
        return Verdict(WARN, "controllers in unexpected state: " + ", ".join(unconfigured))

    active = sorted(n for n, s in controllers.items() if s == "active")
    summary = f"{len(active)}/{len(controllers)} controllers active"
    if arm_off:
        return inactive(f"{summary} (arm switched off)")
    return Verdict(OK, summary)
