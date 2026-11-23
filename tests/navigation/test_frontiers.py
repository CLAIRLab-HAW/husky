"""frontiers' core: free cells beside unknown ones, grouped, short ones dropped, each with a viewpoint the base can
stand at -- clear of obstacles and the unknown, in sight of the frontier -- or left out."""

from __future__ import annotations

import math

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


def _cell(x, y):
    return int(y / RES), int(x / RES)


def _clearance(g, x, y, mask):
    r, c = _cell(x, y)
    rr, cc = np.nonzero(mask)
    return np.min(np.hypot(rr - r, cc - c)) * RES


def test_the_doorway_is_a_frontier_and_the_gap_is_not():
    found = fr.find_frontiers(_room(), RES, (0.0, 0.0))
    assert len(found) == 1
    (door,) = found
    assert abs(door["fx"] - 89.5 * RES) < 0.01 and 30 * RES <= door["fy"] <= 50 * RES
    assert door["size_m"] >= 0.9


def test_the_viewpoint_keeps_clear_of_walls_and_the_unknown_and_sees_the_frontier():
    g = _room()
    (door,) = fr.find_frontiers(g, RES, (0.0, 0.0))
    assert _clearance(g, door["x"], door["y"], g > fr.FREE_MAX) >= fr.CLEAR_OF_OCCUPIED_M - RES
    assert _clearance(g, door["x"], door["y"], g < 0) >= fr.CLEAR_OF_UNKNOWN_M - RES
    assert math.hypot(door["x"] - door["fx"], door["y"] - door["fy"]) <= fr.VIEW_RANGE_M
    assert fr._in_sight(g > fr.FREE_MAX, _cell(door["x"], door["y"]), _cell(door["fx"], door["fy"]))
    assert 0 <= g[_cell(door["x"], door["y"])] <= fr.FREE_MAX


def test_a_frontier_in_a_corridor_too_narrow_to_stand_in_is_left_out():
    g = np.full((60, 120), -1, dtype=np.int16)
    g[20:40, 10:100] = 100
    g[28:32, 10:100] = 0  # a 0.2 m slot, open at its east end into the unknown
    assert fr.find_frontiers(g, RES, (0.0, 0.0)) == []


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
