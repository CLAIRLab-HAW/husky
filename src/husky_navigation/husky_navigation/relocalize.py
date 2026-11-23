"""Find where the robot stands in the stored map from one scan, and tell AMCL.

AMCL starts at the pose its parameters name -- the map's origin -- and corrects only while the robot moves.  A robot
that stands elsewhere at the start stays localized wrong: on the a200-0553 on 2026-11-23, after a stack restart
had stopped the SLAM that would have handed its pose over, AMCL held (0, 0, 0) while the robot stood at
(0.14, 0.18, 6.7 deg); 28 % of the scan lay on the map's walls there, 96 % at the right pose.

The search scores a pose by the share of scan points that land on an occupied cell or within ``HIT_RADIUS_M`` of
one, around AMCL's pose, coarse then fine.  A pose is published as ``initialpose`` only when it clears ``MIN_FIT``
and beats AMCL's by ``MIN_GAIN`` -- a doubtful match is not a correction.

The core is ROS-free (numpy); :func:`main` is the ROS part, run by ``husky/setup/tools/nav.sh`` as
``python3 -m husky_navigation.relocalize``.
"""

from __future__ import annotations

import math
import sys
from collections.abc import Sequence

import numpy as np

#: How near a wall a scan point may land and still count as on it [m]: two cells at the maps' 5 cm, the RS16's noise
#: and the map's own blur.
HIT_RADIUS_M = 0.10
#: The fit a published pose must reach: 96 % at the right pose, 28 % at a wrong one on the a200-0553 (2026-11-23).
MIN_FIT = 0.60
#: How much better than AMCL's own pose a correction must fit -- a few percent are noise, not a better pose.
MIN_GAIN = 0.10
#: The search, coarse then fine: (step in x/y [m], step in yaw [rad], half span in x/y [m], half span in yaw [rad]).
#: The coarse stage covers a start 0.6 m and 30 deg off; the fine one settles to 2 cm and 0.5 deg.
STAGES = ((0.10, math.radians(2.0), 0.6, math.radians(30.0)), (0.02, math.radians(0.5), 0.12, math.radians(3.0)))
#: Scan returns farther than this are left out [m]: the map's far edges are where SLAM saw least.
MAX_RANGE_M = 8.0
#: initialpose's doubt: 5 cm and 1.7 deg -- AMCL's particles start around a measured pose, not a guess.
POSE_STDEV_M = 0.05
YAW_STDEV_RAD = 0.03


def hit_grid(occupied: np.ndarray, radius_cells: int) -> np.ndarray:
    """The cells a scan point may land in and count as on a wall: the occupied ones, grown by a disc.

    :param occupied: ``(height, width)`` bool, True where the map is occupied.
    :param radius_cells: the disc's radius in cells.
    :returns: the grown grid, same shape.
    """
    grown = np.zeros_like(occupied)
    padded = np.pad(occupied, radius_cells)
    h, w = occupied.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                grown |= padded[radius_cells + dy : radius_cells + dy + h, radius_cells + dx : radius_cells + dx + w]
    return grown


def fit(grid: np.ndarray, origin_xy: Sequence[float], resolution_m: float, points: np.ndarray, pose) -> float:
    """The share of ``points`` (robot frame, ``(n, 2)``) on ``grid`` when the robot stands at ``pose`` (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + c * points[:, 0] - s * points[:, 1]
    wy = y + s * points[:, 0] + c * points[:, 1]
    cx = np.floor((wx - origin_xy[0]) / resolution_m).astype(int)
    cy = np.floor((wy - origin_xy[1]) / resolution_m).astype(int)
    inside = (cx >= 0) & (cx < grid.shape[1]) & (cy >= 0) & (cy < grid.shape[0])
    hits = np.zeros(len(points), dtype=bool)
    hits[inside] = grid[cy[inside], cx[inside]]
    return float(hits.mean()) if len(points) else 0.0


def search(grid: np.ndarray, origin_xy, resolution_m: float, points: np.ndarray, start, stages=STAGES):
    """The best-fitting pose around ``start``, coarse then fine.

    :returns: ``(fit, (x, y, yaw))``.
    """
    best = (fit(grid, origin_xy, resolution_m, points, start), tuple(start))
    for step_xy, step_yaw, span_xy, span_yaw in stages:
        bx, by, byaw = best[1]
        offsets_xy = np.arange(-span_xy, span_xy + 1e-9, step_xy)
        for dyaw in np.arange(-span_yaw, span_yaw + 1e-9, step_yaw):
            for dx in offsets_xy:
                for dy in offsets_xy:
                    pose = (bx + dx, by + dy, byaw + dyaw)
                    f = fit(grid, origin_xy, resolution_m, points, pose)
                    if f > best[0]:
                        best = (f, pose)
    return best


def verdict(fit_now: float, fit_best: float) -> str:
    """Whether to publish: ``publish``, ``kept`` (AMCL's pose fits as well) or ``doubtful`` (nothing fits)."""
    if fit_best < MIN_FIT:
        return "doubtful"
    if fit_best - fit_now < MIN_GAIN:
        return "kept"
    return "publish"


def main(argv: list[str] | None = None) -> int:  # pragma: no cover - needs a running robot
    """Read map, scan and AMCL's pose, search, and publish ``initialpose`` when the match is clear."""
    import rclpy
    import tf2_ros
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan

    from husky_navigation.wiring import BASE_FRAME, NAMESPACE, scan_topic

    rclpy.init(args=["--ros-args", "-r", f"/tf:=/{NAMESPACE}/tf", "-r", f"/tf_static:=/{NAMESPACE}/tf_static"])
    node = rclpy.create_node("relocalize")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    got: dict = {}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, f"/{NAMESPACE}/map", lambda m: got.setdefault("map", m), latched)
    node.create_subscription(LaserScan, scan_topic(), lambda m: got.__setitem__("scan", m), qos_profile_sensor_data)
    publisher = node.create_publisher(PoseWithCovarianceStamped, f"/{NAMESPACE}/initialpose", 1)

    deadline = node.get_clock().now().nanoseconds + 15e9
    start = None
    while node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(got) < 2:
            continue
        try:  # map->odom and odom->base_link apart: AMCL stamps map->odom ahead, and a chained lookup misses it
            a = buf.lookup_transform("map", "odom", Time()).transform
            b = buf.lookup_transform("odom", BASE_FRAME, Time()).transform
        except tf2_ros.TransformException:
            continue
        ya, yb = 2 * math.atan2(a.rotation.z, a.rotation.w), 2 * math.atan2(b.rotation.z, b.rotation.w)
        start = (
            a.translation.x + math.cos(ya) * b.translation.x - math.sin(ya) * b.translation.y,
            a.translation.y + math.sin(ya) * b.translation.x + math.cos(ya) * b.translation.y,
            ya + yb,
        )
        break
    if start is None:
        print("[relocalize] no map, scan or AMCL pose within 15 s", file=sys.stderr)
        return 1

    scan, grid_msg = got["scan"], got["map"]
    info = grid_msg.info
    r = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    ok = np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max, MAX_RANGE_M))
    points = np.c_[r[ok] * np.cos(angles[ok]), r[ok] * np.sin(angles[ok])]
    occupied = np.asarray(grid_msg.data, dtype=np.int16).reshape(info.height, info.width) >= 65
    grid = hit_grid(occupied, round(HIT_RADIUS_M / info.resolution))
    origin = (info.origin.position.x, info.origin.position.y)

    fit_now = fit(grid, origin, info.resolution, points, start)
    fit_best, (x, y, yaw) = search(grid, origin, info.resolution, points, start)
    said = verdict(fit_now, fit_best)
    print(
        f"[relocalize] AMCL ({start[0]:.2f}, {start[1]:.2f}, {math.degrees(start[2]):.1f} deg) fits {fit_now:.0%}; "
        f"best ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg) fits {fit_best:.0%} -> {said}"
    )
    if said == "publish":
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = POSE_STDEV_M**2
        cov[35] = YAW_STDEV_RAD**2
        msg.pose.covariance = cov
        end = node.get_clock().now().nanoseconds + 5e9
        while publisher.get_subscription_count() == 0 and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if said != "doubtful" else 1


if __name__ == "__main__":
    raise SystemExit(main())
