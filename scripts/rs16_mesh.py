#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["cadquery-ocp==7.8.1.1.post1", "numpy", "pycollada", "scipy", "trimesh"]
# ///
"""Writes ``rs16_description/meshes/rs16.dae`` from RoboSense's CAD model of the RS-LiDAR-16.

The source is ``RS16.stp`` from robosense.ai (Downloads, "RS-LiDAR-16 3D Model", 2020-07-23), a Creo assembly in mm
of 15 parts. It is not in the repo: RoboSense's download page states no license for redistributing it. This script
keeps the four parts one sees from outside -- housing, cap, optical window, M12 cable gland -- and drops the inner ones
(seal ring, screw, pins, bracket). Their look is :data:`LOOK`, not the STEP's colors: the STEP paints the window
brown and the housing a flat light gray, where the sensor is matte metallic gray with a metallic blue window.

The frame is the one ``rs16.urdf.xacro`` hangs the mesh on: the STEP's axis of rotation is its -y, the mounting face
at y = +7.7 mm. Rotated -90 deg about x, that face lies on z = 0 and the axis on +z; the gland then points along -x,
which is the RS16's own -x in ROS (user manual Fig. 14 with note 1: 0 deg = ROS +x, opposite the cable outlet).
Checked against the manual's Fig. E-1 on 2026-10-09: the gland's center 8.9 mm above the face (8.9 +/- 0.2), the
window's middle at 38.9 mm (optical center 39 mm, section 8.1 note 2), 80.8 mm high (80.7 +/- 0.5).

The DAE is written with pycollada directly, not through ``trimesh.exchange.dae``: trimesh 5.0 gave every geometry the
same source ids (``verts-array`` four times) and wrote ``Y_UP`` (2026-10-09).

    uv run husky/scripts/rs16_mesh.py RS16.stp husky/src/rs16_description/meshes/rs16.dae
"""

from __future__ import annotations

import argparse

import collada
import numpy as np
import trimesh
from collada import asset, geometry, material, scene, source
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Quantity import Quantity_Color, Quantity_TypeOfColor
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool

#: The STEP's part names of what one sees from outside, and the name each gets in the DAE.
VISIBLE = {
    "20202-0148-00_16_Y0001": "housing",
    "20202-0144-00_16_Y0001": "cap",
    "160001": "window",
    "M12X1-5_5_0001": "gland",
}
#: How each part looks [sRGB diffuse, specular, shininess], read off RoboSense's product photo with the owner on
#: 2026-10-09: housing and cap matte metallic gray, the window metallic blue, the gland bright metal. The twin
#: (``clair.twin.collada``) takes the diffuse alone; Foxglove and RViz shade with all three.
LOOK = {
    "housing": ((0.50, 0.52, 0.55), 0.35, 12.0),
    "cap": ((0.58, 0.60, 0.63), 0.35, 12.0),
    "window": ((0.05, 0.45, 0.82), 0.60, 40.0),
    "gland": ((0.67, 0.70, 0.74), 0.50, 30.0),
}
#: Where the STEP puts the mounting face along its axis [mm].
FACE_Y_MM = 7.7
#: Tessellation: 0.15 mm chord, 0.35 rad between neighbors -- 17 749 triangles, 0.8 MB (2026-10-09).
LINEAR_DEFLECTION_MM = 0.15
ANGULAR_DEFLECTION_RAD = 0.35


def read_parts(step_path: str) -> dict[str, tuple[np.ndarray, np.ndarray, list[float]]]:
    """The visible parts of the STEP, placed: ``{name: (vertices_mm, faces, srgb)}``."""
    doc = TDocStd_Document(TCollection_ExtendedString("rs16"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    if reader.ReadFile(step_path) != 1 or not reader.Transfer(doc):
        raise SystemExit(f"cannot read {step_path}")
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colors = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    def name(label: TDF_Label) -> str:
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ""

    def srgb(label: TDF_Label) -> list[float] | None:
        color = Quantity_Color()
        for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
            if colors.GetColor_s(label, kind, color):
                return list(color.Values(Quantity_TypeOfColor.Quantity_TOC_sRGB))
        return None

    def components(label: TDF_Label) -> list[tuple[TDF_Label, TopLoc_Location]]:
        seq = TDF_LabelSequence()
        shapes.GetComponents_s(label, seq)
        out = []
        for i in range(1, seq.Length() + 1):
            ref = TDF_Label()
            shapes.GetReferredShape_s(seq.Value(i), ref)
            out.append((ref, shapes.GetLocation_s(seq.Value(i))))
        return out

    parts = {}

    def walk(label: TDF_Label, loc: TopLoc_Location) -> None:
        if name(label) in VISIBLE:
            # A part carries its color on itself or, for the housing and the cap, on the solid below it.
            color = srgb(label) or next((c for c in (srgb(ref) for ref, _ in components(label)) if c), None)
            parts[VISIBLE[name(label)]] = (*triangles(shapes.GetShape_s(label).Moved(loc)), color)
        for ref, sub in components(label):
            walk(ref, loc.Multiplied(sub))

    roots = TDF_LabelSequence()
    shapes.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location())
    if parts.keys() != set(VISIBLE.values()):
        raise SystemExit(f"parts missing from {step_path}: {set(VISIBLE.values()) - parts.keys()}")
    return parts


def triangles(shape) -> tuple[np.ndarray, np.ndarray]:
    """Tessellates ``shape``: ``(vertices_mm, faces)``, wound outwards."""
    BRepMesh_IncrementalMesh(shape, LINEAR_DEFLECTION_MM, False, ANGULAR_DEFLECTION_RAD, True)
    vertices, faces, offset = [], [], 0
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            moved = loc.Transformation()
            nodes = (tri.Node(i).Transformed(moved) for i in range(1, tri.NbNodes() + 1))
            v = np.array([[p.X(), p.Y(), p.Z()] for p in nodes])
            f = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)]) - 1
            if face.Orientation() == TopAbs_REVERSED:
                f = f[:, ::-1]
            vertices.append(v)
            faces.append(f + offset)
            offset += len(v)
        explorer.Next()
    return np.vstack(vertices), np.vstack(faces)


def write_dae(parts: dict[str, tuple[np.ndarray, np.ndarray, list[float]]], dae_path: str) -> None:
    """One geometry and one material per part, in meters, ``Z_UP``, mounting face on z = 0."""
    dae = collada.Collada()
    dae.assetInfo.upaxis = asset.UP_AXIS.Z_UP
    nodes = []
    for key, (v, f, _step_color) in parts.items():
        color, specular, shininess = LOOK[key]
        placed = np.c_[v[:, 0], v[:, 2], FACE_Y_MM - v[:, 1]] / 1000.0  # -90 deg about x, face onto z = 0
        mesh = trimesh.Trimesh(placed, f, process=True)
        effect = material.Effect(
            f"{key}-effect",
            [],
            "phong",
            diffuse=(*color, 1.0),
            specular=(specular, specular, specular, 1.0),
            shininess=shininess,
            transparent=None,
            transparency=None,
        )
        mat = material.Material(f"{key}-material", key, effect)
        positions = source.FloatSource(f"{key}-positions", mesh.vertices.astype(np.float32).ravel(), ("X", "Y", "Z"))
        normals = source.FloatSource(f"{key}-normals", mesh.vertex_normals.astype(np.float32).ravel(), ("X", "Y", "Z"))
        geom = geometry.Geometry(dae, f"{key}-mesh", key, [positions, normals])
        inputs = source.InputList()
        inputs.addInput(0, "VERTEX", f"#{key}-positions")
        inputs.addInput(0, "NORMAL", f"#{key}-normals")
        geom.primitives.append(geom.createTriangleSet(mesh.faces.astype(np.int32).ravel(), inputs, key))
        dae.effects.append(effect)
        dae.materials.append(mat)
        dae.geometries.append(geom)
        nodes.append(scene.Node(key, children=[scene.GeometryNode(geom, [scene.MaterialNode(key, mat, inputs=[])])]))
        bounds_mm = np.round(mesh.bounds * 1000, 1).tolist()
        print(f"{key:8s} {len(mesh.faces):6d} triangles, srgb {np.round(color, 3).tolist()}, bounds {bounds_mm} mm")
    dae.scenes.append(scene.Scene("rs16", nodes))
    dae.scene = dae.scenes[0]
    dae.write(dae_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("step", help="RoboSense's RS16.stp")
    parser.add_argument("dae", help="where to write rs16.dae")
    args = parser.parse_args()
    write_dae(read_parts(args.step), args.dae)


if __name__ == "__main__":
    main()
