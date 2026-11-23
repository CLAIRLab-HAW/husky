"""The map's frontiers -- where known free space meets unknown -- for an explorer to drive to.

slam_toolbox's map grows wherever the robot looks; what it has not seen yet is ``-1``.  A frontier is a free cell
with an unknown neighbor, and the cells that touch each other form one; a frontier shorter than ``MIN_SIZE_M`` is
noise along a wall, not a way on.

The frontier itself is a poor goal: in a furnished room most frontiers are the scan shadows behind furniture, and
their cells stand right beside the obstacle that casts them -- on the a200-0553 33 of 37 frontier points answered
``inscribed`` and none was driven to (2026-11-23).  Each frontier therefore goes out with a **viewpoint**: the free
cell nearest it that keeps ``CLEAR_OF_OCCUPIED_M`` from every obstacle and ``CLEAR_OF_UNKNOWN_M`` from the unknown,
within ``VIEW_RANGE_M`` and with a clear line of sight to it -- the RS16 sees all round, and from there it maps past
the frontier.  A frontier without one is left out.  Whether the base fits at the viewpoint is still the costmap's
question, asked by the explorer (``cost``, ``plan``) before it drives.

Published as JSON on ``frontiers_topic()`` (std_msgs/String, the text the SDK's transport carries), on every map and
again every second, since a late subscriber gets nothing old::

    {"frame": "map", "stamp_s": 1795.2, "frontiers": [{"x": 0.6, "y": -0.1, "fx": 1.2, "fy": -0.4, "size_m": 1.35}]}

``x``/``y`` the viewpoint, ``fx``/``fy`` the frontier's own cell nearest its centroid.

largest first.  Checked and not taken: explore_lite (m-explore-ros2) finds frontiers and drives Nav2 to them itself
-- past the plan server, its arm interlock and the explorer's look around -- and is not packaged for Jazzy in the
robot's snapshot (2026-11-23).

The core (:func:`find_frontiers`) is ROS-free (numpy); :func:`main` is the node, started with slam_toolbox by
``launch/navigation.launch.py``.
"""

from __future__ import annotations

import json
from collections import deque

import numpy as np

from husky_navigation.relocalize import hit_grid

#: A frontier shorter than this is the edge of a wall's shadow, not a way into the unknown [m].
MIN_SIZE_M = 0.40
#: OccupancyGrid values up to this count as free: slam_toolbox writes 0 for seen-free, 100 for occupied, -1 unknown.
FREE_MAX = 25
#: How far a viewpoint stays from any obstacle [m]: past the footprint's inscribed radius (0.39 m), so its cell is not
#: ``inscribed``; the plan server moves the goal up to 0.5 m on to where the whole footprint fits at its heading.
CLEAR_OF_OCCUPIED_M = 0.7
#: How far a viewpoint stays from the unknown [m]: the costmap knows nothing there, and the footprint reaches 0.87 m.
CLEAR_OF_UNKNOWN_M = 0.5
#: How far from its frontier a viewpoint may stand [m]: the RS16 sees 25 m, and 3 m keeps the frontier in plain view.
VIEW_RANGE_M = 3.0
#: How many of the nearest viewpoint cells are checked for a line of sight before a frontier is given up.
SIGHT_TRIES = 60
#: How often the frontiers go out again between map updates [s] -- a late subscriber gets nothing old.
REPUBLISH_S = 1.0


def _in_sight(occupied: np.ndarray, a, b) -> bool:
    """Whether the straight line from cell ``a`` to cell ``b`` crosses no occupied cell (the ends left out)."""
    steps = int(2 * max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
    for t in np.linspace(0.0, 1.0, steps)[1:-1]:
        if occupied[round(a[0] + t * (b[0] - a[0])), round(a[1] + t * (b[1] - a[1]))]:
            return False
    return True


def find_frontiers(grid: np.ndarray, resolution_m: float, origin_xy, min_size_m: float = MIN_SIZE_M) -> list[dict]:
    """The frontiers of an OccupancyGrid's data that have a viewpoint, largest first.

    :param grid: ``(height, width)`` int, the map's values (-1 unknown, 0..100 occupancy).
    :returns: ``[{"x", "y", "fx", "fy", "size_m"}]`` in the map's frame: the viewpoint, the frontier point, the length.
    """
    free = (grid >= 0) & (grid <= FREE_MAX)
    unknown = grid < 0
    occupied = grid > FREE_MAX
    standable = free & ~hit_grid(occupied, round(CLEAR_OF_OCCUPIED_M / resolution_m))
    standable &= ~hit_grid(unknown, round(CLEAR_OF_UNKNOWN_M / resolution_m))
    stand_cells = np.argwhere(standable).astype(float)
    view_cells = VIEW_RANGE_M / resolution_m
    padded = np.pad(unknown, 1)
    touches_unknown = padded[:-2, 1:-1] | padded[2:, 1:-1] | padded[1:-1, :-2] | padded[1:-1, 2:]
    frontier = free & touches_unknown

    seen = np.zeros_like(frontier)
    h, w = frontier.shape
    out = []
    for start in zip(*np.nonzero(frontier), strict=True):
        if seen[start]:
            continue
        cells, queue = [], deque([start])
        seen[start] = True
        while queue:
            r, c = queue.popleft()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and frontier[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        queue.append((rr, cc))
        size_m = len(cells) * resolution_m
        if size_m < min_size_m:
            continue
        rc = np.array(cells, dtype=float)
        fr, fc = cells[int(np.argmin(np.hypot(*(rc - rc.mean(axis=0)).T)))]
        if not len(stand_cells):
            continue
        dist = np.hypot(stand_cells[:, 0] - fr, stand_cells[:, 1] - fc)
        view = None
        for k in np.argsort(dist)[:SIGHT_TRIES]:
            if dist[k] > view_cells:
                break
            if _in_sight(occupied, (fr, fc), stand_cells[k]):
                view = stand_cells[k]
                break
        if view is None:
            continue
        cell_xy = lambda r, c: (  # noqa: E731
            round(origin_xy[0] + (c + 0.5) * resolution_m, 3),
            round(origin_xy[1] + (r + 0.5) * resolution_m, 3),
        )
        (x, y), (fx, fy) = cell_xy(*view), cell_xy(fr, fc)
        out.append({"x": x, "y": y, "fx": fx, "fy": fy, "size_m": round(size_m, 2)})
    return sorted(out, key=lambda f: -f["size_m"])


def main(argv: list[str] | None = None) -> None:  # pragma: no cover - needs a running robot
    """The node: the map in, its frontiers out as JSON."""
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    from husky_navigation.wiring import NAMESPACE, frontiers_topic

    rclpy.init(args=argv)
    node = rclpy.create_node("frontiers")
    publisher = node.create_publisher(String, frontiers_topic(), 10)
    last = {"text": None}

    def on_map(msg: OccupancyGrid) -> None:
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        found = find_frontiers(grid, info.resolution, (info.origin.position.x, info.origin.position.y))
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        last["text"] = json.dumps({"frame": msg.header.frame_id or "map", "stamp_s": stamp_s, "frontiers": found})
        publisher.publish(String(data=last["text"]))

    def again() -> None:
        if last["text"] is not None:
            publisher.publish(String(data=last["text"]))

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, f"/{NAMESPACE}/map", on_map, latched)
    node.create_timer(REPUBLISH_S, again)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C, or the SIGTERM systemd stops Nav2 with
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
