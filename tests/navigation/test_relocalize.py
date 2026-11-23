"""relocalize's core: from one scan and the map, back to the pose the scan was taken at."""

from __future__ import annotations

import math

import numpy as np
import pytest

from husky_navigation import relocalize as rl

RES = 0.05


def _room() -> np.ndarray:
    """6 x 4 m, walls all round, a cabinet and a pillar inside -- asymmetric, so one pose explains the scan."""
    g = np.zeros((80, 120), dtype=bool)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = True
    g[10:30, 80:84] = True  # cabinet
    g[55:58, 25:28] = True  # pillar
    return g


def _scan(occupied: np.ndarray, pose) -> np.ndarray:
    """Every wall cell within 5 m, as the robot at ``pose`` would see it (no occlusion -- the fit needs none)."""
    x, y, yaw = pose
    cy, cx = np.nonzero(occupied)
    wx, wy = (cx + 0.5) * RES, (cy + 0.5) * RES
    keep = np.hypot(wx - x, wy - y) < 5.0
    dx, dy = wx[keep] - x, wy[keep] - y
    c, s = math.cos(-yaw), math.sin(-yaw)
    return np.c_[c * dx - s * dy, s * dx + c * dy][::3]


def test_the_search_finds_the_pose_the_scan_was_taken_at():
    occupied = _room()
    grid = rl.hit_grid(occupied, round(rl.HIT_RADIUS_M / RES))
    truth = (2.4, 1.7, math.radians(25.0))
    points = _scan(occupied, truth)
    start = (truth[0] + 0.2, truth[1] - 0.15, truth[2] + math.radians(8.0))

    fit_start = rl.fit(grid, (0.0, 0.0), RES, points, start)
    fit_best, (x, y, yaw) = rl.search(grid, (0.0, 0.0), RES, points, start)

    assert fit_best > 0.95 > fit_start
    assert math.hypot(x - truth[0], y - truth[1]) < 0.05
    assert abs(math.degrees(yaw - truth[2])) < 1.5
    assert rl.verdict(fit_start, fit_best) == "publish"


def test_the_hit_grid_grows_a_cell_by_its_radius():
    g = np.zeros((9, 9), dtype=bool)
    g[4, 4] = True
    grown = rl.hit_grid(g, 2)
    assert grown[4, 6] and grown[6, 4] and not grown[6, 6] and grown.sum() == 13


@pytest.mark.parametrize(
    ("now", "best", "said"), [(0.28, 0.96, "publish"), (0.90, 0.95, "kept"), (0.20, 0.45, "doubtful")]
)
def test_only_a_clear_and_better_match_is_published(now, best, said):
    assert rl.verdict(now, best) == said
