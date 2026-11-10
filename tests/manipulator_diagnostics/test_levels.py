"""The rules that turn the arm's and the gripper's state into diagnostic levels."""

from __future__ import annotations

import pytest
from husky_manipulator_diagnostics.arm import (
    arm_control_level,
    arm_controllers_level,
    arm_is_off,
    arm_is_powered,
    arm_joints_level,
    arm_mode_level,
    robot_mode_name,
    safety_mode_name,
)
from husky_manipulator_diagnostics.gripper import (
    gripper_level,
    gripper_signal_valid,
    gripper_summary,
    parse_bridge_state,
)
from husky_manipulator_diagnostics.verdict import ERROR, OK, STALE, WARN

#: The RG6's dead input threshold [V] the node defaults to.
DEAD_V = 0.2
REQUIRED = ["joint_state_broadcaster", "arm_0_joint_trajectory_controller"]


def test_the_measured_cases():
    """The cases measured on the a200-0553 (2026-07-29): powered AI2 10.00 V / AI3 1.33 V, unpowered both ~0.056 V
    while tool_power_on=true, *_received=true, busy=true at the same time."""
    DEAD = 0.2

    # --- Arm Mode ---------------------------------------------------------
    assert arm_mode_level(7, 1).level == OK, "RUNNING/NORMAL must be OK"
    assert arm_mode_level(7, 2).level == WARN, "REDUCED -> WARN"
    assert arm_mode_level(7, 3).level == ERROR, "PROTECTIVE_STOP -> ERROR"
    assert arm_mode_level(0, 1).level == ERROR, "DISCONNECTED -> ERROR"
    assert arm_mode_level(None, None).level == STALE, "nothing received -> STALE"
    assert "emergency stop" in arm_mode_level(7, 7).message, "e-stop needs plain text"
    # Safety beats robot_mode: a p-stop while RUNNING stays ERROR.
    assert arm_mode_level(5, 8).level == ERROR, "VIOLATION -> ERROR"
    # POWER_OFF is an operator decision ─▶ grey, not yellow.
    off = arm_mode_level(3, 1)
    assert off.inactive and off.level == OK, "POWER_OFF -> inactive (grey), not WARN"
    assert arm_mode_level(4, 1).level == WARN, "POWER_ON (transition) -> WARN"
    assert arm_mode_level(5, 1).level == WARN, "IDLE (transition) -> WARN"
    # A safety problem stays visible even while the arm is switched off.
    assert arm_mode_level(3, 8).level == ERROR, "VIOLATION beats POWER_OFF"

    # --- Arm Control ------------------------------------------------------
    assert arm_control_level(True, 0.01, 2.0).level == OK
    assert arm_control_level(False, 0.01, 2.0).level == WARN, "EC off -> WARN"
    assert arm_control_level(True, 5.0, 2.0).level == ERROR, "stream broken -> ERROR"
    # Exactly watchdog case (b): EC reports 'running', the motion link is dead.
    assert arm_control_level(True, None, 2.0).level == ERROR, "never a JS -> ERROR"
    assert arm_control_level(None, 0.01, 2.0).level == WARN
    # With the arm switched off, "EC stopped" is as expected -- and a dead joint_state stream is not an error either
    # (the watchdog rests too).
    assert arm_control_level(False, 0.01, 2.0, arm_off=True).inactive
    assert arm_control_level(False, None, 2.0, arm_off=True).inactive

    # --- Arm Joints -------------------------------------------------------
    assert arm_joints_level(6, 0.01, 2.0).level == OK
    assert arm_joints_level(6, 5.0, 2.0).level == STALE
    assert arm_joints_level(0, 0.01, 2.0).level == STALE
    assert arm_joints_level(6, 0.01, 2.0, arm_off=True).inactive, "arm off -> grey"

    # --- Arm Controllers --------------------------------------------------
    req = ["joint_state_broadcaster", "arm_0_joint_trajectory_controller"]
    all_ok = {
        "joint_state_broadcaster": "active",
        "arm_0_joint_trajectory_controller": "active",
        "freedrive_mode_controller": "inactive",
    }
    assert arm_controllers_level(all_ok, req).level == OK, "parked modes are normal"
    missing = dict(all_ok)
    del missing["joint_state_broadcaster"]
    assert arm_controllers_level(missing, req).level == ERROR
    stopped = dict(all_ok, joint_state_broadcaster="inactive")
    assert arm_controllers_level(stopped, req).level == ERROR
    weird = dict(all_ok, freedrive_mode_controller="unconfigured")
    assert arm_controllers_level(weird, req).level == WARN
    assert arm_controllers_level(None, req).level == STALE
    assert arm_controllers_level(all_ok, req, arm_off=True).inactive
    # A real controller problem stays red even with the arm switched off.
    assert arm_controllers_level(stopped, req, arm_off=True).level == ERROR

    # --- Gripper: signal validity -----------------------------------------
    assert gripper_signal_valid(10.0, 1.334, DEAD), "powered (measured)"
    assert not gripper_signal_valid(0.056, 0.053, DEAD), "unpowered (measured)"
    assert not gripper_signal_valid(0.0, 0.0, DEAD), "arm POWER_OFF (measured)"
    assert not gripper_signal_valid(None, None, DEAD)
    # Gripper fully closed: AI2 drops to the calibration lower bound (0.56 V) but stays above the dead threshold -- must
    # NOT count as unpowered.
    assert gripper_signal_valid(0.56, 0.9, DEAD), "a closed gripper is not dead"

    # --- Gripper: state from the bridge ------------------------------------
    # The gripper state arrives as JSON from rg6_grip_bridge.
    good = parse_bridge_state(
        '{"width_m": 0.1032, "busy": false, "grip_detected": true,'
        ' "status": 0, "safety_failed": false, "last_command": "GRIP"}'
    )
    assert good["width_m"] == 0.1032 and good["grip_detected"] is True, good
    # A broken or foreign payload must NOT kill the node -- it is the same as "no status": better report grey/red than
    # crash.
    assert parse_bridge_state("not json") is None
    assert parse_bridge_state("[1, 2, 3]") is None
    assert parse_bridge_state('{"width_m": "wide"}') is None, "width must be a number"
    # Missing fields are allowed and become None -- a bridge that does not send a field should not empty the panel.
    assert parse_bridge_state('{"width_m": 0.05}')["busy"] is None

    # --- Gripper: evaluation ----------------------------------------------
    assert gripper_level(0.05, 2.0, True, 7, 10.0, DEAD).level == OK
    stale = gripper_level(9.0, 2.0, True, 7, 10.0, DEAD)
    assert stale.level == ERROR and "bridge" in stale.message, stale.message
    never = gripper_level(None, 2.0, True, 7, None, DEAD)
    assert never.level == ERROR and "bridge" in never.message, never.message
    # THE case from the bug report: arm POWER_OFF, gripper without supply.
    dead_off = gripper_level(0.05, 2.0, False, 3, 0.0, DEAD)
    assert dead_off.inactive and dead_off.level == OK, "arm off -> gripper grey, not OK"
    assert "switched off" in dead_off.message
    # Arm powered, gripper still without signal ─▶ a real warning with a recipe.
    dead_on = gripper_level(0.05, 2.0, False, 7, 0.056, DEAD)
    assert dead_on.level == WARN and not dead_on.inactive
    # The URCap sets the tool voltage, no ROS service does -- so the warning must point at the pendant and must not name
    # a service.
    assert "URCap" in dead_on.message, "the warning must name the way out"
    assert "set_tool_power" not in dead_on.message, "no such service exists"
    # Arm in an unpowered non-POWER_OFF state (e.g. BOOTING).
    assert gripper_level(0.05, 2.0, False, 2, 0.0, DEAD).level == WARN

    assert "open" in gripper_summary(0.159, False, False, 0.160)
    assert "closed" in gripper_summary(0.0, False, False, 0.160)
    assert "grasped" in gripper_summary(0.042, True, False, 0.160)
    assert "moving" in gripper_summary(0.042, False, True, 0.160)
    assert gripper_summary(None, False, False, 0.160) == "width unknown"

    # --- arm state --------------------------------------------------------
    assert arm_is_powered(7) and arm_is_powered(6) and arm_is_powered(4)
    assert not arm_is_powered(3) and not arm_is_powered(2)
    assert arm_is_powered(None) is None
    assert arm_is_off(3) and not arm_is_off(7)


@pytest.mark.parametrize(
    ("robot_mode", "safety_mode", "level", "says"),
    [
        (0, 6, ERROR, "emergency stop"),  # an e-stop is named even while the arm is unreachable
        (1, 2, ERROR, "not reachable"),  # unreachable beats a reduced safety mode
        (None, 1, WARN, "not ready to drive: UNKNOWN"),  # half the latched pair is not "never received"
        (8, 1, WARN, "UPDATING_FIRMWARE"),
        (7, 4, WARN, "RECOVERY"),
    ],
)
def test_the_arm_mode_ranks_safety_then_reachability_then_readiness(robot_mode, safety_mode, level, says):
    verdict = arm_mode_level(robot_mode, safety_mode)
    assert (verdict.level, verdict.inactive) == (level, False)
    assert says in verdict.message


def test_unknown_modes_stay_readable():
    assert robot_mode_name(42) == "UNKNOWN(42)"
    assert safety_mode_name(None) == "UNKNOWN"
    assert safety_mode_name(12) == "UNKNOWN(12)"


def test_a_joint_state_exactly_at_the_timeout_still_counts_as_flowing():
    assert arm_control_level(True, 2.0, 2.0).level == OK
    assert arm_control_level(True, 2.01, 2.0).level == ERROR
    assert arm_joints_level(6, 2.0, 2.0).level == OK


def test_stale_joints_stay_stale_on_a_switched_off_arm():
    verdict = arm_joints_level(6, None, 2.0, arm_off=True)
    assert (verdict.level, verdict.inactive) == (STALE, False)


def test_controller_problems_are_listed_sorted_and_outrank_the_switched_off_arm():
    assert arm_controllers_level({}, REQUIRED).level == ERROR
    missing = arm_controllers_level({"other": "active"}, REQUIRED)
    assert missing.message == "controllers missing: arm_0_joint_trajectory_controller, joint_state_broadcaster"
    odd = {name: "active" for name in REQUIRED} | {"zeta": "unconfigured", "alpha": "finalized"}
    verdict = arm_controllers_level(odd, REQUIRED, arm_off=True)
    assert (verdict.level, verdict.inactive) == (WARN, False)
    assert verdict.message.endswith("alpha, zeta")


@pytest.mark.parametrize(
    ("payload", "field", "kept"),
    [
        ('{"width_m": 0}', "width_m", 0),  # an int width is a number
        ('{"width_m": 0.1, "busy": "yes"}', "busy", None),
        ('{"width_m": 0.1, "status": 1.5}', "status", None),
        ('{"width_m": 0.1, "status": true}', "status", None),  # a bool is not a status code
        ('{"width_m": 0.1, "last_command": 2}', "last_command", None),
        ('{"width_m": 0.1, "safety_failed": false}', "safety_failed", False),
    ],
)
def test_a_field_of_the_wrong_type_empties_only_its_row(payload, field, kept):
    state = parse_bridge_state(payload)
    assert state[field] == kept
    assert set(state) == {"width_m", "busy", "grip_detected", "status", "safety_failed", "last_command"}


@pytest.mark.parametrize("payload", [None, '{"width_m": true}', '"text"', "null"])
def test_an_unusable_bridge_state_is_none_and_never_raises(payload):
    assert parse_bridge_state(payload) is None


def test_one_live_channel_or_one_at_the_threshold_makes_a_valid_tool_signal():
    assert gripper_signal_valid(0.05, 0.9, DEAD_V)
    assert gripper_signal_valid(DEAD_V, 0.0, DEAD_V)
    assert not gripper_signal_valid(10.0, None, DEAD_V)


def test_without_a_known_robot_mode_a_dead_gripper_points_at_the_urcap():
    verdict = gripper_level(0.05, 2.0, False, None, None, DEAD_V)
    assert verdict.level == WARN and "URCap" in verdict.message and "n/a" in verdict.message


@pytest.mark.parametrize(
    ("width_m", "grip_detected", "busy", "stroke_m", "summary"),
    [
        (0.08, False, False, 0.160, "part open, 80mm (50%)"),
        (0.042, True, True, 0.160, "moving, 42mm (26%)"),  # busy beats grasped
        (0.152, False, False, 0.160, "open, 152mm (95%)"),
        (0.02, False, False, 0.160, "closed, 20mm (12%)"),
        (0.05, False, False, 0.0, "open, 50mm (0%)"),  # no stroke: nothing to divide by, every width is all of it
    ],
)
def test_the_gripper_summary_names_the_state_and_the_width(width_m, grip_detected, busy, stroke_m, summary):
    assert gripper_summary(width_m, grip_detected, busy, stroke_m) == summary
