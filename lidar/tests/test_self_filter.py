"""The self filter drops the lidar's points of the robot itself, and only near the sensor."""

from __future__ import annotations

import numpy as np

from clair.lidar.self_filter import NEAR_M, robot_points


def test_a_point_inside_the_robot_near_the_sensor_is_dropped() -> None:
    xyz = np.array([[0.5, 0.0, 0.0], [0.5, 1.0, 0.0], [NEAR_M + 1.0, 0.0, 0.0]])
    arm = lambda p: np.abs(p[:, 1]) < 0.2  # noqa: E731 -- a slab along x: the arm reaching out ahead
    assert robot_points(xyz, arm).tolist() == [True, False, False], "the arm, beside it, far out along it"


def test_far_points_never_reach_the_hulls() -> None:
    asked: list[int] = []

    def inside(p: np.ndarray) -> np.ndarray:
        asked.append(len(p))
        return np.ones(len(p), bool)

    xyz = np.array([[0.3, 0.0, 0.0], [5.0, 0.0, 0.0], [0.0, 9.0, 0.0]])
    assert robot_points(xyz, inside).tolist() == [True, False, False]
    assert asked == [1], "a wall 5 m away is no part of the robot and costs no hull test"
