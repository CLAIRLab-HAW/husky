"""The map's frontiers -- where known free space meets unknown -- for an explorer to drive to.

slam_toolbox's map grows wherever the robot looks; what it has not seen yet is ``-1``.  A frontier is a free cell
with an unknown neighbor, and the cells that touch each other form one; a frontier shorter than ``MIN_SIZE_M`` is
noise along a wall, not a way on.  Each goes out with its length and a goal point: its own cell nearest its centroid,
so the point is free by construction -- whether the base fits there is the costmap's question, asked by the explorer
(``cost``, ``plan``) before it drives.

Published as JSON on ``frontiers_topic()`` (std_msgs/String, the text the SDK's transport carries), on every map and
again every second, since a late subscriber gets nothing old::

    {"frame": "map", "stamp_s": 1795.2, "frontiers": [{"x": 1.2, "y": -0.4, "size_m": 1.35}, ...]}

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

#: A frontier shorter than this is the edge of a wall's shadow, not a way into the unknown [m].
MIN_SIZE_M = 0.40
#: OccupancyGrid values up to this count as free: slam_toolbox writes 0 for seen-free, 100 for occupied, -1 unknown.
FREE_MAX = 25
#: How often the frontiers go out again between map updates [s] -- a late subscriber gets nothing old.
REPUBLISH_S = 1.0


def find_frontiers(grid: np.ndarray, resolution_m: float, origin_xy, min_size_m: float = MIN_SIZE_M) -> list[dict]:
    """The frontiers of an OccupancyGrid's data, largest first.

    :param grid: ``(height, width)`` int, the map's values (-1 unknown, 0..100 occupancy).
    :returns: ``[{"x", "y", "size_m"}]`` in the map's frame; ``x``/``y`` a frontier cell's center.
    """
    free = (grid >= 0) & (grid <= FREE_MAX)
    unknown = grid < 0
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
        r, c = cells[int(np.argmin(np.hypot(*(rc - rc.mean(axis=0)).T)))]
        out.append(
            {
                "x": round(origin_xy[0] + (c + 0.5) * resolution_m, 3),
                "y": round(origin_xy[1] + (r + 0.5) * resolution_m, 3),
                "size_m": round(size_m, 2),
            }
        )
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
