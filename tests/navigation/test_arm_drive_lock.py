"""arm_drive_lock's core: free only with the arm in a drive pose and its joint states fresh; locked otherwise."""

from __future__ import annotations

from pathlib import Path

import pytest

from husky_navigation import arm_drive_lock as adl

ROBOT_YAML = Path(__file__).resolve().parents[2] / "setup" / "config" / "robot.yaml"
POSES = adl.drive_poses_from_robot_yaml(ROBOT_YAML.read_text(encoding="utf-8"))


def _lock_at(pose: dict[str, float], now: float = 10.0, stamp: float = 10.0) -> adl.ArmDriveLock:
    lock = adl.ArmDriveLock(POSES)
    lock.joints(stamp, list(pose), list(pose.values()))
    return lock


def test_both_drive_poses_come_from_robot_yaml_with_the_arms_joint_names():
    assert set(POSES) == set(adl.DRIVE_POSES)
    for pose in POSES.values():
        assert sorted(pose) == sorted(f"arm_0_{j}" for j in adl.UR_JOINTS)


@pytest.mark.parametrize("name", adl.DRIVE_POSES)
def test_in_a_drive_pose_the_base_may_drive(name):
    locked, why = _lock_at(POSES[name]).verdict(10.0)
    assert not locked and name in why


def test_within_the_tolerance_counts_as_in_the_pose():
    pose = dict(POSES["forward"])
    pose["arm_0_wrist_3_joint"] += 0.8 * adl.JOINT_TOLERANCE_RAD
    assert not _lock_at(pose).verdict(10.0)[0]


def test_out_of_the_drive_poses_locks_and_names_the_nearest():
    pose = dict(POSES["forward"])
    pose["arm_0_elbow_joint"] -= 0.4  # the forearm swung out
    locked, why = _lock_at(pose).verdict(10.0)
    assert locked and "forward" in why and "arm_0_elbow_joint" in why


def test_silent_joint_states_lock():
    assert _lock_at(POSES["forward"], stamp=10.0).verdict(10.0 + 2 * adl.JOINT_STALE_S)[0]
    assert adl.ArmDriveLock(POSES).verdict(0.0) == (True, "no fresh joint states of the arm")


def test_joint_states_without_the_arm_lock():
    lock = adl.ArmDriveLock(POSES)
    lock.joints(10.0, ["front_left_wheel_joint"], [1.0])
    assert lock.verdict(10.0)[0]


def test_a_missing_drive_pose_refuses_to_start():
    with pytest.raises(KeyError):
        adl.drive_poses_from_robot_yaml(ROBOT_YAML.read_text(encoding="utf-8"), names=("forward", "nowhere"))
