"""The RG6 side: the bridge's JSON state, whether the tool connector carries a signal at all, and the gripper's
verdict and summary.  ROS-free."""

import json

from .arm import arm_is_off, arm_is_powered, robot_mode_name
from .verdict import ERROR, OK, WARN, Verdict, inactive

# What went to the gripper last.  The bridge sends the plain text directly (rg6_grip_bridge.COMMAND_*); the numeric
# values are translated along so that archived recordings stay readable.
GRIPPER_COMMANDS = {0: "NONE", 1: "OPEN", 2: "CLOSE", 3: "GRIP"}

#: Fields that ``rg6_grip_bridge.status_payload`` delivers, with the type they
#: must have.  What is missing becomes None -- a bridge that does not send a
#: field should empty only that row, not the whole panel.
BRIDGE_FIELDS = {
    "width_m": (int, float),
    "busy": bool,
    "grip_detected": bool,
    "status": int,
    "safety_failed": bool,
    "last_command": str,
}


def parse_bridge_state(data):
    """JSON from ``<ns>/rg6/bridge_state`` ─▶ dict, or None if unusable.

    Why it is parsed at all:  ``rg6_grip_bridge`` reports the gripper state as JSON inside a ``std_msgs/String``, not as
    a typed message.  In exchange the string costs the type check a .msg gets for free, so that check lives here.

    None of this may raise:  a callback that dies on a foreign payload takes the whole diagnostics node with it -- and
    then the statement about the ARM is missing too, which has nothing to do with the gripper.
    """
    try:
        raw = json.loads(data)
    except (TypeError, ValueError):
        return None
    if not isinstance(raw, dict):
        return None
    out = {}
    for name, want in BRIDGE_FIELDS.items():
        value = raw.get(name)
        if value is None:
            out[name] = None
        elif want is bool:
            out[name] = value if isinstance(value, bool) else None
        elif isinstance(value, bool):
            # In Python bool is an int subclass -- without this branch a "width_m": true would pass as a number.
            out[name] = None
        else:
            out[name] = value if isinstance(value, want) else None
    if out["width_m"] is None and raw.get("width_m") is not None:
        return None  # a width that is not a number: unusable
    return out


def gripper_signal_valid(width_raw, force_raw, dead_threshold):
    """Does the RG6 deliver a valid tool signal at all? ─▶ bool.

    The probe is the voltage against ``dead_input_threshold`` (0.2 V): with no 24 V tool supply present, AI2 (width) and
    AI3 (force) drop to ~0.05 V.

    Why the VOLTAGE and not the answer of the XML-RPC endpoint:  the endpoint sits in the control box and answers even
    when nothing is powered at the tool connector.  It knows what it commanded last -- AI2/AI3 know what the hardware
    does.
    """
    if width_raw is None or force_raw is None:
        return False
    return width_raw >= dead_threshold or force_raw >= dead_threshold


def gripper_level(state_age, timeout, signal_valid, robot_mode, width_raw, dead_threshold):
    """Evaluation of the RG6 state ─▶ Verdict.

    The RG6 hangs off the UR tool connector: without a powered arm it cannot have any supply at all.  That is then not a
    gripper fault but a consequence of the arm state -- hence grey (arm deliberately off) or WARN (arm in a state in
    which it should be powered).
    """
    if state_age is None:
        return Verdict(ERROR, "no rg6/bridge_state - is rg6-grip-bridge running?")
    if state_age > timeout:
        # The bridge STAYS SILENT when the XML-RPC endpoint does not answer (it prefers reporting nothing over an old
        # value).  A status that is too old is therefore exactly the signal for "endpoint gone".
        return Verdict(
            ERROR, f"rg6/bridge_state silent for {state_age:.1f}s - rg6-grip-bridge dead or URCap endpoint gone?"
        )

    if not signal_valid:
        raw = "n/a" if width_raw is None else f"{width_raw:.2f} V"
        if arm_is_off(robot_mode):
            return inactive(f"arm switched off - gripper without supply (tool signal {raw})")
        if arm_is_powered(robot_mode) is False:
            return Verdict(
                WARN,
                f"arm not powered ({robot_mode_name(robot_mode)}) - gripper without supply (tool signal {raw})",
            )
        # Arm powered, still no signal: the 24 V tool supply is not present. Switching it on is the business of the
        # OnRobot URCap -- the ROS route there would go over a tool DO, and the URCap occupies that itself.  No ROS
        # service can repair this; the place to look is the teach pendant.
        return Verdict(
            WARN,
            f"no valid tool signal ({raw} < "
            f"{dead_threshold:.2f} V) - tool unpowered: "
            "is the URCap program running on the pendant?",
        )
    return Verdict(OK, "ready")


def gripper_summary(width_m, grip_detected, busy, stroke_m):
    """Short text for the OK message: 'open, 158mm (99%)' / 'grasped, 42mm'."""
    if width_m is None:
        return "width unknown"
    mm = width_m * 1000.0
    pct = 100.0 * width_m / stroke_m if stroke_m > 0.0 else 0.0
    if busy:
        state = "moving"
    elif grip_detected:
        state = "grasped"
    elif width_m >= 0.95 * stroke_m:
        state = "open"
    elif width_m <= 0.02:
        state = "closed"
    else:
        state = "part open"
    return f"{state}, {mm:.0f}mm ({pct:.0f}%)"
