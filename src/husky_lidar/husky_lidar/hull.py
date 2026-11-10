"""The robot's body as convex hulls: which points of a sensor are the robot itself.

Each ``<collision>`` of the robot description becomes a convex hull (a mesh's vertices, a primitive's corners) in its
link's frame, grown by a padding; where the links stand comes from the caller (TF in the self filter).  A convex hull
is looser than the triangles; the chassis' hull fills the space above its deck, which is the robot's own anyway.

The same computation as ``clair.twin.body.RobotBody`` in the clearpath workspace, which the splat map and the
camera masks use there.  husky must not import clair, so it carries its own, and the workspace's
``twin/twinlink/tests/test_body_self_filter.py`` holds the two to the same answer.  ``robot_body_filter`` would be the
package to take, but has no ROS 2 port: its ``ros2`` branch is the catkin package with one CI commit on top,
unreleased for Jazzy (checked 2026-09-27).
"""

from __future__ import annotations

import logging
import re
import xml.etree.ElementTree as ET
from collections.abc import Callable
from dataclasses import dataclass
from functools import cache
from pathlib import Path

import numpy as np

log = logging.getLogger("husky_lidar.hull")


@dataclass(frozen=True)
class Hull:
    """One collision element as a convex hull in its link's frame: ``normals @ p + offsets <= 0`` inside."""

    link: str
    normals: np.ndarray  # (F, 3)
    offsets: np.ndarray  # (F,)
    center: np.ndarray  # (3,)
    radius_m: float


class Body:
    """The robot's collision hulls, and which points fall inside them."""

    def __init__(self, urdf: str, find_package: Callable[[str], Path | None], padding_m: float) -> None:
        self.padding_m = padding_m
        self.hulls: list[Hull] = []
        for link in ET.fromstring(urdf).iter("link"):
            for collision in link.findall("collision"):
                points = _collision_points(collision, find_package)
                if points is not None and len(points) >= 4:
                    self.hulls.append(_hull(link.get("name", ""), points))
        self.links = sorted({h.link for h in self.hulls})

    def inside(self, points: np.ndarray, poses: dict[str, tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
        """Which of the ``points`` (N, 3) lie within a hull, the links at ``poses`` (link ─▶ (position, rotation
        link ─▶ the points' frame)); a link without a pose is left out."""
        hit = np.zeros(len(points), bool)
        for hull in self.hulls:
            pose = poses.get(hull.link)
            if pose is None:
                continue
            position, rotation = pose
            local = (points - position) @ rotation
            near = np.linalg.norm(local - hull.center, axis=1) <= hull.radius_m + self.padding_m
            near &= ~hit
            if near.any():
                within = np.all(local[near] @ hull.normals.T + hull.offsets <= self.padding_m, axis=1)
                hit[np.flatnonzero(near)[within]] = True
        return hit


def _collision_points(collision: ET.Element, find_package: Callable[[str], Path | None]) -> np.ndarray | None:
    """The points whose hull a ``<collision>`` fills, in its link's frame."""
    geometry = collision.find("geometry")
    shape = geometry[0] if geometry is not None and len(geometry) else None
    if shape is None:
        return None
    if shape.tag == "mesh":
        match = re.match(r"(?:package|file)://([^/]+)/(.+)", shape.get("filename", ""))
        directory = find_package(match.group(1)) if match is not None else None
        path = directory / match.group(2) if directory is not None and match is not None else None
        if path is None or not path.is_file():
            log.warning("no mesh for %s -- that part of the robot stays in the scan", shape.get("filename"))
            return None
        scale = np.array([float(v) for v in shape.get("scale", "1 1 1").split()])
        points = _mesh_vertices(str(path)) * scale
    elif shape.tag == "box":
        half = np.array([float(v) for v in shape.get("size", "0 0 0").split()]) / 2
        points = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * half
    elif shape.tag == "cylinder":
        r, half = float(shape.get("radius", 0)), float(shape.get("length", 0)) / 2
        angles = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        ring = np.column_stack([r * np.cos(angles), r * np.sin(angles)])
        points = np.vstack([np.column_stack([ring, np.full(16, z)]) for z in (-half, half)])
    elif shape.tag == "sphere":
        r = float(shape.get("radius", 0))
        points = np.vstack([np.eye(3), -np.eye(3)]) * r
        points = np.vstack([points, np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * r])
    else:
        return None
    origin = collision.find("origin")
    xyz = np.array([float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()])
    rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
    return points @ _rpy_matrix(*rpy).T + xyz


@cache
def _mesh_vertices(path: str) -> np.ndarray:
    """A mesh file's hull vertices, read once."""
    import trimesh

    mesh = trimesh.load(path, force="mesh")
    return np.asarray(mesh.convex_hull.vertices, float)


def _hull(link: str, points: np.ndarray) -> Hull:
    from scipy.spatial import ConvexHull, QhullError

    center = points.mean(axis=0)
    radius = float(np.linalg.norm(points - center, axis=1).max())
    try:
        hull = ConvexHull(points)
    except QhullError:  # flat -- a plate: its bounding sphere alone stands in
        return Hull(link, np.zeros((1, 3)), np.array([0.0]), center, radius)
    return Hull(link, hull.equations[:, :3], hull.equations[:, 3], center, radius)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF's fixed-axis roll, pitch, yaw: Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
