#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["mapbox-earcut", "numpy", "pycollada", "shapely", "trimesh"]
# ///
"""Writes ``d435_bracket.dae``: the 3D-printed L bracket that holds the wrist D435 on the RG6, corners rounded.

The bracket's shape is the owner's description (2026-10-10), its numbers ``tools/rg6.urdf.xacro``'s comment: a base
lying flat on the RG6 body's +y face, 28 mm wide over the body's two front M3 holes, from behind the camera to 5 mm
past the holes, and an upright on the base's rear edge, 56 mm wide for the D435's rear screws, up past them; both
4 mm thick, silver-gray like the lidar spacer on the arch. Every corner of the two plates' outlines is rounded
(:data:`CORNER_RADIUS_M`; the owner asked for slightly rounded corners, 2026-10-10). The frame is
``rg6_gripper_body``'s, which ``camera_0_bracket`` hangs on unmoved.

    uv run husky/extras/tools/d435_bracket_mesh.py
"""

from __future__ import annotations

import argparse
import datetime
from pathlib import Path

import numpy as np
import trimesh
from collada import Collada, asset, geometry, material, scene, source
from shapely.geometry import box

#: The written file.
DAE = Path(__file__).resolve().parents[1] / "src/husky_extras_description/meshes/d435_bracket.dae"
#: The base on the RG6 [m]: x across, z along the gripper, y its thickness up from the body's face.
BASE_X_M, BASE_Z_M, BASE_Y_M = (-0.014, 0.014), (0.0597, 0.1041), (0.0215, 0.0255)
#: The upright behind the camera [m]: x across, y up from the base's top, z its thickness.
UPRIGHT_X_M, UPRIGHT_Y_M, UPRIGHT_Z_M = (-0.031, 0.025), (0.0255, 0.0550), (0.0597, 0.0637)
#: The corners' radius [m], and the segments a quarter circle is drawn with.
CORNER_RADIUS_M, CORNER_SEGMENTS = 0.003, 6
#: The arch's ARCH_LOOK (sensor_arch_profiles.py): sRGB diffuse, specular, shininess.
LOOK = ((0.74, 0.76, 0.79), 0.50, 30.0)
#: Written into ``<created>``/``<modified>``, so a second run writes the same file.
STAMP = datetime.datetime(2026, 10, 10)


def plate(span_a: tuple[float, float], span_b: tuple[float, float], thickness: tuple[float, float]) -> np.ndarray:
    """A plate whose outline spans ``span_a`` x ``span_b`` with rounded corners, ``thickness`` along the third axis,
    as triangles in (a, b, thickness) coordinates."""
    outline = box(span_a[0], span_b[0], span_a[1], span_b[1])
    rounded = outline.buffer(-CORNER_RADIUS_M).buffer(CORNER_RADIUS_M, quad_segs=CORNER_SEGMENTS)
    solid = trimesh.creation.extrude_polygon(rounded, thickness[1] - thickness[0], engine="earcut")
    solid.apply_translation([0.0, 0.0, thickness[0]])
    return solid.triangles


def bracket() -> np.ndarray:
    """Both plates in ``rg6_gripper_body``'s frame."""
    base = plate(BASE_X_M, BASE_Z_M, BASE_Y_M)[:, :, [0, 2, 1]]  # (x, z, y) -> (x, y, z)
    upright = plate(UPRIGHT_X_M, UPRIGHT_Y_M, UPRIGHT_Z_M)  # (x, y, z) as it stands
    # Swapping two axes mirrors the base; turning its corners back keeps its faces outward.
    return np.concatenate([base[:, ::-1], upright])


def write(triangles: np.ndarray, path: Path) -> None:
    """One geometry, one material, meters, ``Z_UP``, flat normals."""
    dae = Collada()
    dae.assetInfo.upaxis = asset.UP_AXIS.Z_UP
    dae.assetInfo.created = dae.assetInfo.modified = STAMP
    color, specular, shininess = LOOK
    effect = material.Effect(
        "bracket-effect",
        [],
        "phong",
        diffuse=(*color, 1.0),
        specular=(specular, specular, specular, 1.0),
        shininess=shininess,
        transparent=None,
        transparency=None,
    )
    mat = material.Material("bracket-material", "bracket", effect)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    points, point_index = np.unique(triangles.reshape(-1, 3).astype(np.float32), axis=0, return_inverse=True)
    directions, direction_index = np.unique(normals.astype(np.float32), axis=0, return_inverse=True)
    positions = source.FloatSource("bracket-positions", points.ravel(), ("X", "Y", "Z"))
    normal_source = source.FloatSource("bracket-normals", directions.ravel(), ("X", "Y", "Z"))
    geom = geometry.Geometry(dae, "bracket-mesh", "bracket", [positions, normal_source])
    inputs = source.InputList()
    inputs.addInput(0, "VERTEX", "#bracket-positions")
    inputs.addInput(1, "NORMAL", "#bracket-normals")
    index = np.column_stack([point_index.ravel(), np.repeat(direction_index.ravel(), 3)]).astype(np.int32)
    geom.primitives.append(geom.createTriangleSet(index.ravel(), inputs, "bracket"))
    dae.effects.append(effect)
    dae.materials.append(mat)
    dae.geometries.append(geom)
    node = scene.Node("bracket", children=[scene.GeometryNode(geom, [scene.MaterialNode("bracket", mat, inputs=[])])])
    dae.scenes.append(scene.Scene("d435_bracket", [node]))
    dae.scene = dae.scenes[0]
    dae.write(str(path))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("dae", nargs="?", default=str(DAE), help="where to write d435_bracket.dae")
    args = parser.parse_args()
    triangles = bracket()
    write(triangles, Path(args.dae))
    bounds = np.round(np.array([triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))]) * 1000, 2).tolist()
    print(f"{len(triangles)} triangles, bounds {bounds} mm")


if __name__ == "__main__":
    raise SystemExit(main())
