"""frontiers' core: free cells beside unknown ones, grouped, short ones dropped, the goal point free."""

from __future__ import annotations

import numpy as np

from husky_navigation import frontiers as fr

RES = 0.05


def _room() -> np.ndarray:
    """4 x 3 m seen, walls all round but for a 1 m doorway on the right, unknown beyond it and above a 0.2 m gap."""
    g = np.full((80, 120), -1, dtype=np.int16)
    g[10:70, 10:90] = 0  # seen free
    g[10, 10:90] = g[69, 10:90] = 100  # walls
    g[10:70, 10] = g[10:70, 89] = 100
    g[30:50, 89] = 0  # a 1 m doorway east into the unknown
    g[10, 40:44] = 0  # a 0.2 m gap north -- shorter than MIN_SIZE_M
    return g


def test_the_doorway_is_a_frontier_and_the_gap_is_not():
    found = fr.find_frontiers(_room(), RES, (0.0, 0.0))
    assert len(found) == 1
    (door,) = found
    assert abs(door["x"] - 89.5 * RES) < 0.01 and 30 * RES <= door["y"] <= 50 * RES
    assert door["size_m"] >= 0.9


def test_the_goal_point_is_a_free_cell():
    g = _room()
    for f in fr.find_frontiers(g, RES, (0.0, 0.0)):
        r, c = int(f["y"] / RES), int(f["x"] / RES)
        assert 0 <= g[r, c] <= fr.FREE_MAX


def test_a_fully_seen_room_has_no_frontiers():
    g = _room()
    g[30:50, 89] = 100
    g[10, 40:44] = 100
    assert fr.find_frontiers(g, RES, (0.0, 0.0)) == []


def test_the_largest_comes_first():
    g = _room()
    g[69, 20:30] = 0  # a 0.5 m opening south
    sizes = [f["size_m"] for f in fr.find_frontiers(g, RES, (0.0, 0.0))]
    assert sizes == sorted(sizes, reverse=True) and len(sizes) == 2
