#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["numpy", "pycollada"]
# ///
"""Writes ``rg6_cable.dae``: the short M8 cable from the RG6's quick-changer outlet to the UR5's tool connector.

Both ends ride on links joined rigidly to ``arm_0_tool0`` (the RG6 bracket hangs on it, and it on the wrist 3), so the
cable is a static tube in that frame, which is the frame this file is written in. The two connectors, read off the
meshes the container installs (2026-10-10), in ``arm_0_tool0`` [mm]:

- The UR5's tool connector is the 8-pin M8 socket on the wrist 3 housing (``ur_description``'s ``ur5/visual/
  wrist3.dae``): a stub of r 3.5 standing radially out of the housing's r 37.5 along -y, its mating face at
  (0, -43.0, -19.48). That face is bare in the mesh, so the cable ends in a straight plug here: a coupling nut
  (:data:`NUT_DIAMETER_M`) over the stub from 2 mm off the housing, then a strain relief down to the cable.
- The RG6's outlet is on its quick-changer bracket (``rg6_description``'s ``rg6_v2/visual/single_bracket.stl``): a
  stub of r 5 along -x out of the bracket at z 10, an elbow, a body of r 5 along +y, then a taper whose end face
  (r 2.5, a 5 mm cable) is at (-56.44, 37.08, 10.00), facing +y. The mesh draws the plug and its bend relief already,
  so the cable leaves that face along +y with no plug of its own.

The two point away from each other: the UR socket along -y, the RG6 outlet along +y, on the other side of the wrist.
So the cable leaves the plug along -y, turns round the wrist's -x side at z -19 to -14, below the bracket (z >= -2.5)
and above the wrist 2 (z <= -35 at any wrist 3 angle), runs along +y past the outlet and turns back up into it in a U
of r 12 in the yz plane: 204 mm from the plug to the outlet. No turn is tighter than r 11.9, about twice the cable's
diameter. The route is drawn, not traced on the robot; diameter 5.5 mm, black (sRGB 0.04).

Measured 2026-10-10 against the visual and collision meshes of the wrist 3, the bracket with its ring, the RG6 body,
the D435 and its bracket, and against the wrist 2 at every wrist 3 angle in 5 deg steps, from 6 mm off each mating
end: the tube comes closest to the wrist 3 housing, 4.75 mm (4.85 to its collision mesh), along its run at x -45;
3.6 mm to the bracket where it leaves the outlet's face; 19.4 mm to the wrist 2.

    uv run husky/extras/tools/rg6_cable_mesh.py
"""

from __future__ import annotations

import argparse
import datetime
from pathlib import Path

import numpy as np
from collada import Collada, asset, geometry, material, scene, source

#: The written file.
DAE = Path(__file__).resolve().parents[1] / "src/husky_extras_description/meshes/rg6_cable.dae"
#: The UR5 tool connector's mating face and its axis, in ``arm_0_tool0`` [m].
UR_SOCKET_M, UR_AXIS = np.array([0.0, -0.0430, -0.019475]), np.array([0.0, -1.0, 0.0])
#: The RG6 outlet's end face and its axis, in ``arm_0_tool0`` [m].
RG6_OUTLET_M, RG6_AXIS = np.array([-0.05644, 0.03708, 0.0100]), np.array([0.0, 1.0, 0.0])
#: The cable [m]: an M8 sensor cable.
CABLE_DIAMETER_M = 0.0055
#: The UR end's plug along the socket's axis, measured from its mating face [m]: the coupling nut from 2 mm off the
#: housing (y -37.5) over the stub, then the strain relief, after which the cable begins.
NUT_DIAMETER_M, NUT_SPAN_M = 0.010, (-0.0035, 0.0090)
RELIEF_DIAMETER_M, RELIEF_SPAN_M = 0.0075, (0.0090, 0.0170)
#: The route between the UR plug's end and the RG6 outlet: points [mm] and the cable's direction there. The two ends
#: come from the connectors above, and cubic Bezier segments whose handles approximate circular arcs join them all.
ROUTE_MM = (
    ((-14.0, -74.0, -18.5), (-1.0, 0.0, 0.0)),
    ((-31.0, -74.0, -17.0), (-1.0, 0.0, 0.0)),
    ((-45.0, -60.0, -15.5), (0.0, 1.0, 0.0)),
    ((-45.0, 0.0, -14.0), (0.0, 1.0, 0.0)),
    ((-56.44, 40.0, -14.0), (0.0, 1.0, 0.0)),
    ((-56.44, 52.0, -2.0), (0.0, 0.0, 1.0)),
    ((-56.44, 40.0, 10.0), (0.0, -1.0, 0.0)),
)
#: The handle length as a fraction of the chord: 0.39 draws a quarter circle within 0.03 % of its radius.
HANDLE = 0.39
#: Segments around the tube and the longest step along it [m].
SIDES, STEP_M = 16, 0.001
#: sRGB diffuse, specular, shininess: a matte black cable.
LOOK = ((0.04, 0.04, 0.04), 0.15, 12.0)
#: Written into ``<created>``/``<modified>``, so a second run writes the same file.
STAMP = datetime.datetime(2026, 10, 10)


def bezier(p0: np.ndarray, t0: np.ndarray, p1: np.ndarray, t1: np.ndarray) -> np.ndarray:
    """One segment from ``p0`` leaving along ``t0`` to ``p1`` arriving along ``t1``, sampled at most
    :data:`STEP_M` apart, without its last point."""
    handle = HANDLE * np.linalg.norm(p1 - p0)
    c = np.array([p0, p0 + handle * t0, p1 - handle * t1, p1])
    n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) * 1.3 / STEP_M)))
    s = np.linspace(0.0, 1.0, n, endpoint=False)[:, None]
    return (1 - s) ** 3 * c[0] + 3 * (1 - s) ** 2 * s * c[1] + 3 * (1 - s) * s**2 * c[2] + s**3 * c[3]


def centerline() -> np.ndarray:
    """The cable's axis from the UR plug's end to the RG6 outlet, in ``arm_0_tool0`` [m]."""
    plug_end = (UR_SOCKET_M + RELIEF_SPAN_M[1] * UR_AXIS, UR_AXIS)
    route = [(np.array(p) / 1000.0, np.array(t) / np.linalg.norm(t)) for p, t in ROUTE_MM]
    nodes = [plug_end, *route, (RG6_OUTLET_M, -RG6_AXIS)]
    parts = [bezier(*a, *b) for a, b in zip(nodes, nodes[1:], strict=False)]
    return np.concatenate([*parts, nodes[-1][0][None]])


def tube(axis: np.ndarray, radius: float) -> tuple[np.ndarray, np.ndarray]:
    """A closed tube of ``radius`` around the polyline ``axis``: triangles and one normal per corner.

    The rings are oriented by parallel transport, so the tube does not twist along a curve; both ends are capped flat.
    """
    tangents = np.gradient(axis, axis=0)
    tangents /= np.linalg.norm(tangents, axis=1, keepdims=True)
    seed = np.cross(tangents[0], [0.0, 0.0, 1.0] if abs(tangents[0][2]) < 0.9 else [1.0, 0.0, 0.0])
    normals = [seed / np.linalg.norm(seed)]
    for current in tangents[1:]:
        n = normals[-1] - np.dot(normals[-1], current) * current
        normals.append(n / np.linalg.norm(n))
    u = np.array(normals)
    v = np.cross(tangents, u)
    angle = np.linspace(0.0, 2 * np.pi, SIDES, endpoint=False)
    radial = np.cos(angle)[None, :, None] * u[:, None] + np.sin(angle)[None, :, None] * v[:, None]
    rings = axis[:, None] + radius * radial

    def quads(grid: np.ndarray) -> np.ndarray:
        """Two triangles per quad between neighboring rings and sides."""
        a, b = grid[:-1], grid[1:]
        a2, b2 = np.roll(a, -1, axis=1), np.roll(b, -1, axis=1)
        return np.concatenate([np.stack([a, b, b2], axis=2), np.stack([a, b2, a2], axis=2)]).reshape(-1, 3, 3)

    caps = [cap(rings[0], axis[0], -tangents[0]), cap(rings[-1], axis[-1], tangents[-1])]
    triangles = np.concatenate([quads(rings), *(c[0] for c in caps)])
    corner_normals = np.concatenate([quads(radial), *(c[1] for c in caps)])
    return triangles, corner_normals


def cap(ring: np.ndarray, center: np.ndarray, outward: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The flat disc closing ``ring`` around ``center``, wound to face ``outward``."""
    nxt = np.roll(ring, -1, axis=0)
    triangles = np.stack([np.broadcast_to(center, ring.shape), ring, nxt], axis=1)
    facing = np.cross(ring - center, nxt - center) @ outward
    triangles[facing < 0] = triangles[facing < 0][:, [0, 2, 1]]
    return triangles, np.broadcast_to(outward, triangles.shape).copy()


def outward(triangles: np.ndarray, normals: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """``triangles`` and their corner ``normals``, each triangle's corners ordered so its face normal agrees with its
    corner normals."""
    face = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    flip = np.einsum("ij,ij->i", face, normals.mean(axis=1)) < 0
    triangles, normals = triangles.copy(), normals.copy()
    triangles[flip] = triangles[flip][:, [0, 2, 1]]
    normals[flip] = normals[flip][:, [0, 2, 1]]
    return triangles, normals


def plug() -> list[tuple[np.ndarray, float]]:
    """The UR end's coupling nut and strain relief: each an axis and a radius."""
    return [
        (np.array([UR_SOCKET_M + d * UR_AXIS for d in np.linspace(*span, 8)]), diameter / 2)
        for diameter, span in ((NUT_DIAMETER_M, NUT_SPAN_M), (RELIEF_DIAMETER_M, RELIEF_SPAN_M))
    ]


def cable() -> tuple[np.ndarray, np.ndarray]:
    """The whole cable, plug included: triangles and corner normals in ``arm_0_tool0`` [m]."""
    parts = [tube(centerline(), CABLE_DIAMETER_M / 2), *(tube(axis, radius) for axis, radius in plug())]
    triangles = np.concatenate([p[0] for p in parts])
    normals = np.concatenate([p[1] for p in parts])
    return outward(triangles, normals)


def write(triangles: np.ndarray, normals: np.ndarray, path: Path) -> None:
    """One geometry, one material, meters, ``Z_UP``, smooth normals along the tube."""
    dae = Collada()
    dae.assetInfo.upaxis = asset.UP_AXIS.Z_UP
    dae.assetInfo.created = dae.assetInfo.modified = STAMP
    color, specular, shininess = LOOK
    effect = material.Effect(
        "cable-effect",
        [],
        "phong",
        diffuse=(*color, 1.0),
        specular=(specular, specular, specular, 1.0),
        shininess=shininess,
        transparent=None,
        transparency=None,
    )
    mat = material.Material("cable-material", "cable", effect)
    points, point_index = np.unique(triangles.reshape(-1, 3).astype(np.float32), axis=0, return_inverse=True)
    directions, direction_index = np.unique(
        np.round(normals.reshape(-1, 3), 4).astype(np.float32), axis=0, return_inverse=True
    )
    positions = source.FloatSource("cable-positions", points.ravel(), ("X", "Y", "Z"))
    normal_source = source.FloatSource("cable-normals", directions.ravel(), ("X", "Y", "Z"))
    geom = geometry.Geometry(dae, "cable-mesh", "cable", [positions, normal_source])
    inputs = source.InputList()
    inputs.addInput(0, "VERTEX", "#cable-positions")
    inputs.addInput(1, "NORMAL", "#cable-normals")
    index = np.column_stack([point_index.ravel(), direction_index.ravel()]).astype(np.int32)
    geom.primitives.append(geom.createTriangleSet(index.ravel(), inputs, "cable"))
    dae.effects.append(effect)
    dae.materials.append(mat)
    dae.geometries.append(geom)
    node = scene.Node("cable", children=[scene.GeometryNode(geom, [scene.MaterialNode("cable", mat, inputs=[])])])
    dae.scenes.append(scene.Scene("rg6_cable", [node]))
    dae.scene = dae.scenes[0]
    dae.write(str(path))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("dae", nargs="?", default=str(DAE), help="where to write rg6_cable.dae")
    args = parser.parse_args()
    triangles, normals = cable()
    write(triangles, normals, Path(args.dae))
    axis = centerline()
    length = np.linalg.norm(np.diff(axis, axis=0), axis=1).sum()
    bounds = np.round(np.array([triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))]) * 1000, 2).tolist()
    print(f"{len(triangles)} triangles, cable {length * 1000:.0f} mm, bounds {bounds} mm")


if __name__ == "__main__":
    raise SystemExit(main())
