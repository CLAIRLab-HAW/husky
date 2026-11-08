"""The rules that turn the arm's and the gripper's state into diagnostic levels, beyond what ``--selftest`` asserts."""

from __future__ import annotations

import pytest
from manipulator_diagnostics import (
    ERROR,
    OK,
    STALE,
    WARN,
    arm_control_level,
    arm_controllers_level,
    arm_joints_level,
    arm_mode_level,
    gripper_level,
    gripper_signal_valid,
    gripper_summary,
    parse_bridge_state,
    robot_mode_name,
    safety_mode_name,
    selftest,
)

#: The RG6's dead input threshold [V] the node defaults to.
DEAD_V = 0.2
REQUIRED = ["joint_state_broadcaster", "arm_0_joint_trajectory_controller"]


def test_selftest_passes():
    assert selftest() == 0


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
