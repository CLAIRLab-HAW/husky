#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["mapbox-earcut", "numpy", "pycollada", "scipy", "shapely", "trimesh"]
# ///
"""Draws the sensor arch's right-angled bars in ``husky_sensor_arch.dae`` as the 20x20 mm slot profiles they are.

The arch's legs, top cross beam and feet are aluminum system profiles, 20 x 20 mm with a slot on each face, butted as
on the robot (the owner, 2026-10-09): the legs run whole from the top plate to the top, the cross beam under the lidar
sits between them, and the feet run backward from the legs' rear faces. The Onshape model the mesh came from drew
plain square bars of the same size, joined the other way round (sectioned 2026-10-09: legs x 35-55 and 565-585 mm,
y 35-55, z 20-480; beam x 35-585, z 480-500; feet y 35-265, z 0-20). This script cuts those bars out and puts the
profiles in their place; the diagonal struts, the plates on the legs' outer faces and the lidar plate on the beam
stay as the mesh has them. Every outer face stays where it was, so nothing that stands on the arch moves.

The cross section is a generic slot-6 profile of that size, drawn, not taken from a maker's drawing: a 6.2 mm slot
opening behind a 1.8 mm lip, an 11 mm undercut, its floor sloping in to the core, which leaves the four diagonal webs
that read as an X, and a 4.2 mm core bore. It is a visual; the mass stays R47's estimate.

Run it again on its output and it writes the same file: the cut boxes (:data:`CUTS`) hold the Onshape bars, the
profiles of every earlier run and these alike.

    uv run onboard/extras/tools/sensor_arch_profiles.py \\
        onboard/extras/src/husky_extras_description/meshes/husky_sensor_arch.dae
"""

from __future__ import annotations

import argparse

import collada
import numpy as np
import trimesh
from collada import asset, geometry, material, scene, source
from shapely.geometry import Polygon

#: One slot, on the face y = +10 mm of a profile centered on the origin, from the corner (10, 10) leftward [mm]:
#: opening 6.2 wide, lip 1.8 deep, undercut 11 wide, its floor sloping to a 4 mm wide bottom 6 mm below the face.
SLOT_MM = (
    (10.0, 10.0),
    (3.1, 10.0),
    (3.1, 8.2),
    (5.5, 8.2),
    (5.5, 6.8),
    (2.0, 4.0),
    (-2.0, 4.0),
    (-5.5, 6.8),
    (-5.5, 8.2),
    (-3.1, 8.2),
    (-3.1, 10.0),
)
#: The core bore's radius [mm] and the segments that draw it.
BORE_MM, BORE_SEGMENTS = 2.1, 24

#: The bars as boxes in the mesh frame [m]: (min corner, max corner, axis). The legs run whole, the beam between them,
#: the feet behind them.
BARS = (
    ((0.035, 0.035, 0.000), (0.055, 0.055, 0.500), 2),  # leg, left
    ((0.565, 0.035, 0.000), (0.585, 0.055, 0.500), 2),  # leg, right
    ((0.055, 0.035, 0.480), (0.565, 0.055, 0.500), 0),  # cross beam
    ((0.035, 0.055, 0.000), (0.055, 0.265, 0.020), 1),  # foot, left
    ((0.565, 0.055, 0.000), (0.585, 0.265, 0.020), 1),  # foot, right
)
#: What is cut out [m]: each leg's column from the top plate to the top, the beam, the feet -- as wide as the 30 mm
#: profiles a first run of this script drew (onboard ``b22f531``), so a file it wrote is cleaned too. A column, because
#: a leg reaches into the beam's and the feet's boxes, and a triangle is cut only when one box holds it whole.
CUTS = (
    ((0.035, 0.035, 0.000), (0.065, 0.065, 0.500)),
    ((0.555, 0.035, 0.000), (0.585, 0.065, 0.500)),
    ((0.035, 0.035, 0.470), (0.585, 0.065, 0.500)),
    ((0.035, 0.035, 0.000), (0.065, 0.265, 0.030)),
    ((0.555, 0.035, 0.000), (0.585, 0.265, 0.030)),
)
#: A vertex on a box's face counts as inside: the old bars' faces lie exactly on the profiles' outer faces.
CUT_TOLERANCE_M = 1e-4


def cross_section() -> Polygon:
    """The profile's section, centered on the origin, in meters."""
    outline = []
    for quarter in range(4):
        c, s = np.cos(quarter * np.pi / 2), np.sin(quarter * np.pi / 2)
        outline += [(c * x - s * y, s * x + c * y) for x, y in SLOT_MM]
    angles = np.linspace(0.0, 2 * np.pi, BORE_SEGMENTS, endpoint=False)
    bore = [(BORE_MM * np.cos(a), BORE_MM * np.sin(a)) for a in angles]
    return Polygon(np.array(outline) / 1000.0, [np.array(bore[::-1]) / 1000.0])


def profile(low: tuple[float, ...], high: tuple[float, ...], axis: int, section: Polygon) -> trimesh.Trimesh:
    """The profile filling the box ``low``-``high`` along ``axis``."""
    low_a, high_a = np.array(low), np.array(high)
    bar = trimesh.creation.extrude_polygon(section, high_a[axis] - low_a[axis])  # along +z from 0
    # The extrusion's z onto the bar's axis, its x and y onto the other two, in cyclic order (a rotation, not a mirror).
    turn = np.zeros((4, 4))
    turn[axis, 2] = turn[(axis + 1) % 3, 0] = turn[(axis + 2) % 3, 1] = turn[3, 3] = 1.0
    bar.apply_transform(turn)
    center = (low_a + high_a) / 2.0
    center[axis] = low_a[axis]
    bar.apply_translation(center)
    return bar


def outside_bars(triangles: np.ndarray) -> np.ndarray:
    """The triangles not wholly inside one of :data:`CUTS`."""
    keep = np.ones(len(triangles), dtype=bool)
    for low, high in CUTS:
        above = triangles >= np.array(low) - CUT_TOLERANCE_M
        below = triangles <= np.array(high) + CUT_TOLERANCE_M
        keep &= ~np.all(above & below, axis=(1, 2))
    return triangles[keep]


def read(dae_path: str) -> tuple[np.ndarray, tuple[float, ...]]:
    """The mesh's triangles and its diffuse color."""
    dae = collada.Collada(dae_path)
    triangles = np.concatenate(
        [p.vertex[p.vertex_index].reshape(-1, 3, 3) for g in dae.scene.objects("geometry") for p in g.primitives()]
    )
    diffuse = dae.materials[0].effect.diffuse
    return triangles, tuple(diffuse) if isinstance(diffuse, tuple) else (0.8, 0.8, 0.8, 1.0)


def write(mesh: trimesh.Trimesh, color: tuple[float, ...], dae_path: str) -> None:
    """One geometry, one material, meters, ``Z_UP`` -- the shape ``clair.twin.collada`` reads the arch's color from."""
    dae = collada.Collada()
    dae.assetInfo.upaxis = asset.UP_AXIS.Z_UP
    effect = material.Effect(
        "arch-effect", [], "phong", diffuse=color, specular=(0.1, 0.1, 0.1, 1.0), transparent=None, transparency=None
    )
    mat = material.Material("arch-material", "arch", effect)
    positions = source.FloatSource("arch-positions", mesh.vertices.astype(np.float32).ravel(), ("X", "Y", "Z"))
    normals = source.FloatSource("arch-normals", mesh.vertex_normals.astype(np.float32).ravel(), ("X", "Y", "Z"))
    geom = geometry.Geometry(dae, "arch-mesh", "arch", [positions, normals])
    inputs = source.InputList()
    inputs.addInput(0, "VERTEX", "#arch-positions")
    inputs.addInput(0, "NORMAL", "#arch-normals")
    geom.primitives.append(geom.createTriangleSet(mesh.faces.astype(np.int32).ravel(), inputs, "arch"))
    dae.effects.append(effect)
    dae.materials.append(mat)
    dae.geometries.append(geom)
    node = scene.Node("arch", children=[scene.GeometryNode(geom, [scene.MaterialNode("arch", mat, inputs=[])])])
    dae.scenes.append(scene.Scene("arch", [node]))
    dae.scene = dae.scenes[0]
    dae.write(dae_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("dae", help="husky_sensor_arch.dae, rewritten in place")
    args = parser.parse_args()
    triangles, color = read(args.dae)
    rest = outside_bars(triangles)
    section = cross_section()
    # Each profile on its own: merging vertices across them would weld the beam's end faces to the legs' tops.
    parts = [trimesh.Trimesh(**trimesh.triangles.to_kwargs(rest))]
    parts += [profile(low, high, axis, section) for low, high, axis in BARS]
    # Flat shading per part: a profile's faces meet at right angles, and shared vertex normals would round them.
    flat = [
        trimesh.Trimesh(p.triangles.reshape(-1, 3), np.arange(3 * len(p.faces)).reshape(-1, 3), process=False)
        for p in parts
    ]
    mesh = trimesh.util.concatenate(flat)
    write(mesh, color, args.dae)
    print(
        f"kept {len(rest)} of {len(triangles)} triangles outside the bars, {len(BARS)} profiles of "
        f"{section.area * 1e6:.0f} mm^2 section, {len(mesh.faces)} triangles, "
        f"bounds {np.round(mesh.bounds * 1000, 1).tolist()} mm"
    )


if __name__ == "__main__":
    main()
