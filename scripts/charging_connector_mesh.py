#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["cadquery-ocp==7.8.1.1.post1", "embreex", "networkx", "numpy", "pycollada", "scipy", "trimesh"]
# ///
"""Writes ``charging_connector.dae``: the a200-0553's charging socket with its plug in it, from the vendor's CAD.

The sources are two STEP files (SolidWorks 2011, AP214, mm, exported 2012-06-14), one solid each, both with nothing
but a default color: ``SK2-076C04_mount.STEP``, the panel-mount SOCKET -- a square flange with four screw holes, its
body through the panel along -z, four power contacts and two rows of four signal pins facing +z inside a keyed hood --
and ``SK6-076C04_plug.STEP``, the cable PLUG with its gland, mating face at z = 0 facing +z, cable toward -z. They are
NOT in the repo: the vendor states no license for redistributing them. The owner keeps them at
``husky/SK2-076C04_mount.STEP`` and ``husky/SK6-076C04_plug.STEP`` (untracked, 2026-10-10), and the derived DAE is
committed, like ``rs16.dae`` from RoboSense's STEP. On the robot both sit at the rear, the plug permanently in the
socket, both black (the owner, 2026-10-10): the DAE holds the two MATED.

THE FRAME is the socket's: the origin on the center of the flange's panel-facing face (its -z face at
:data:`FLANGE_FACE_Z_MM`, 80.96 mm square, four holes on a 60 mm square), +z out of the panel. The STEP's x and y are
kept: the latches are on +-x, the key (the hood's flat, wider side) on -y.

THE MATING POSE is found from the geometry, not set (:func:`seat`), and was checked in sections on 2026-10-10:

- Turn: the plug's mating face has to face -z, so it is turned by pi about an axis in the face. About y, its key
  (on -y in its STEP too) lands on the socket's; about x it lands on +y. Each of the four turns about z was slid down
  onto the socket: only pi about y enters, the other three stop on the hood's rim with the plug's face 26.9 mm above
  the socket's origin. The script keeps the check (:data:`KEYED_MIN_MM3`).
- Depth: the plug is lowered until it touches. The B-rep intersection (``BRepAlgoAPI_Common``) of the two solids is
  bisected over the plug's height to 5 um. The stop is face to face: the bottom of the plug's insulator walls, at
  y +-26.0 mm over x -9.1..9.1 mm, sits on the socket's contact floor (z 9.20 mm), with the plug's face 7.81 mm above
  the STEP's z = 0. There the socket's four power-contact housings (up to z 22.4 mm) stand 10 mm deep between the
  plug's contact towers (down to z 12.2 mm), the signal pins (z 18.0 mm) 6.5 mm short of the plug's insulator face,
  and the plug's shroud 5.4 mm above the floor of the hood's groove (z 2.38 mm).
- The two snap latches are left out of the stop and checked apart (:data:`LATCH_X_MM`): each is a wedge on the socket
  at x +-33 mm that the plug's latch arm snaps over, modeled relaxed, so the arm's lower bar stands 1.0 mm into the
  wedge, 5.6 mm^3 on each side. More than :data:`LATCH_OVERLAP_MAX_MM3` there, or any overlap elsewhere beyond
  :data:`CLEAR_MM3`, stops the script.

Both parts get :data:`LOOK`, black plastic. Tessellated at :data:`LINEAR_DEFLECTION_MM` and
:data:`ANGULAR_DEFLECTION_RAD`, the socket has 11 730 triangles and the plug 12 670; of these, the CAD faces one can
see from outside the mated pair are written (:func:`outside`), 4 882 and 5 340, 0.54 MB (2026-10-10). The normals are
smoothed within each CAD face only, so the edges stay sharp. Positions and normals are indexed apart, as in
``a200_chassis_mesh.py``. The DAE is written with pycollada directly (``rs16_mesh.py`` says why), dated
:data:`STAMP`, so a second run writes the same bytes.

    uv run husky/scripts/charging_connector_mesh.py husky/SK2-076C04_mount.STEP \\
        husky/SK6-076C04_plug.STEP husky/src/husky_extras_description/meshes/charging_connector.dae

``--chassis`` prints the joint origin ``clearpath_extras.urdf.xacro`` hangs the DAE on: the socket over the lid
``hole_caps()`` puts where the right bump of Clearpath's top chassis stood (``REMOVED["bump_right"]`` in
``a200_chassis_mesh.py``), moved by :data:`DOWN_SLOPE_M` and :data:`LEFT_M`. ``top_chassis.dae`` as in that script's
docstring.
"""

from __future__ import annotations

import argparse
import datetime
import math
import sys
from pathlib import Path

import collada
import numpy as np
import trimesh
from collada import asset, geometry, material, scene, source
from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
from OCP.GProp import GProp_GProps
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Shape

#: The socket's panel-facing flange face in its STEP [mm]: the -z face of 3585 mm^2 (2026-10-10). The DAE's z = 0.
FLANGE_FACE_Z_MM = -6.12
#: Overlap of the parts within this distance of the axis' plane x = 0 is a collision; beyond it lie the snap latches
#: (wedges at |x| 32.3..35.6 mm on the socket, the plug's latch arms at |x| 33.0..34.0 mm; 2026-10-10).
LATCH_X_MM = 32.0
#: What the relaxed latch arms may stand in the wedges in all [mm^3]: 10.9 measured on 2026-10-10.
LATCH_OVERLAP_MAX_MM3 = 12.0
#: What counts as no overlap away from the latches [mm^3]: the faces that meet at the stop overlap by 0.01 mm^3 with
#: the plug 0.05 mm deeper (2026-10-10).
CLEAR_MM3 = 0.001
#: What each wrong turn of the plug must overlap the socket by at the right turn's depth [mm^3]: the key and the hood
#: block all three (2026-10-10), so a smaller number means the turn did not decide anything.
KEYED_MIN_MM3 = 100.0
#: Where the bisection for the plug's depth starts and stops [mm]: at 0 its face is at the socket's, deep in it; at
#: 27 above the hood's rim (z 26.88).
SHIFT_RANGE_MM = (0.0, 27.0)
SHIFT_TOLERANCE_MM = 0.005
#: Black plastic [sRGB diffuse, sRGB specular, shininess] for both parts (the owner, 2026-10-10: both black). The twin
#: (``clair.twin.collada``) takes the diffuse alone; Foxglove and RViz shade with all three.
LOOK = ((0.05, 0.05, 0.05), (0.35, 0.35, 0.35), 30.0)
#: Tessellation: 0.5 mm chord, 0.8 rad between neighbors keeps each part under 15k triangles (2026-10-10). At 0.15 mm
#: and 0.35 rad (``rs16_mesh.py``'s) it was 30.0k and 34.5k: the count follows the ~1000 small CAD faces per part, not
#: the chord.
LINEAR_DEFLECTION_MM = 0.5
ANGULAR_DEFLECTION_RAD = 0.8
#: How many directions :func:`outside` looks for the open along, spread evenly over the sphere.
VIEW_DIRECTIONS = 64
#: Positions are written to 10 um [m], normals to 0.001: with all 24 400 triangles at float32's nine digits the DAE
#: was 1.64 MB, rounded 1.22 MB (2026-10-10), and 10 um is a fiftieth of the chord the tessellation allows.
POSITION_DECIMALS = 5
#: Written into every run's ``<created>``/``<modified>``, so a second run on the same sources writes the same file.
STAMP = datetime.datetime(2026, 10, 10)
#: The sloped rear face's normal ``hole_caps()`` lays the lids along, and how far the flange stands proud of the lid
#: [m]: the lid lies 0.1 mm proud of the slope, and the flange's face is kept off it rather than coplanar.
SLOPE_NORMAL = (-0.64, 0.0, 0.77)
FLANGE_LIFT_M = 0.0001
#: How far the socket sits from the lid's middle [m]. Down the slope so the plug clears the top plate's rear edge by
#: 2.0 mm and the hood the plate's underside by 2.4 mm: centered on the lid, both reach up to 23 mm past that edge
#: (python-fcl against ``large_top_plate.dae`` and its collision STL at the plate's pose, 2026-10-10); the flange still
#: covers the lid with 4.7 mm to spare up the slope. Toward +y, the robot's left: the owner, 2026-10-10.
DOWN_SLOPE_M = 0.0213
LEFT_M = 0.005


def read_solid(step_path: str) -> TopoDS_Shape:
    """The one solid of ``step_path``."""
    reader = STEPControl_Reader()
    if reader.ReadFile(step_path) != 1 or not reader.TransferRoots():
        raise SystemExit(f"cannot read {step_path}")
    shape = reader.OneShape()
    solids = TopExp_Explorer(shape, TopAbs_SOLID)
    count = 0
    while solids.More():
        count += 1
        solids.Next()
    if count != 1:
        raise SystemExit(f"{step_path}: {count} solids, expected 1 -- another export?")
    return shape


def placed(plug: TopoDS_Shape, turn_rad: float, shift_mm: float) -> TopoDS_Shape:
    """The plug face down (pi about y), turned by ``turn_rad`` about z, its face ``shift_mm`` above z = 0."""
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(0.0, 0.0, shift_mm))
    for axis, angle in (((0.0, 0.0, 1.0), turn_rad), ((0.0, 1.0, 0.0), math.pi)):
        rotation = gp_Trsf()
        rotation.SetRotation(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(*axis)), angle)
        trsf.Multiply(rotation)
    return BRepBuilderAPI_Transform(plug, trsf, True).Shape()


def overlap(socket: TopoDS_Shape, plug: TopoDS_Shape) -> tuple[float, float, list[str]]:
    """The volume both solids fill [mm^3], at the latches and elsewhere, and where each piece of it lies."""
    common = BRepAlgoAPI_Common(socket, plug)
    common.Build()
    at_latches, elsewhere, pieces = 0.0, 0.0, []
    solids = TopExp_Explorer(common.Shape(), TopAbs_SOLID)
    while solids.More():
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(solids.Current(), props)
        box = Bnd_Box()
        BRepBndLib.Add_s(solids.Current(), box)
        x0, y0, z0, x1, y1, z1 = box.Get()
        if abs(x0 + x1) / 2 > LATCH_X_MM:
            at_latches += props.Mass()
        else:
            elsewhere += props.Mass()
        pieces.append(f"{props.Mass():.2f} mm^3 in x {x0:.1f}..{x1:.1f} y {y0:.1f}..{y1:.1f} z {z0:.1f}..{z1:.1f}")
        solids.Next()
    return at_latches, elsewhere, pieces


def seat(socket: TopoDS_Shape, plug: TopoDS_Shape) -> float:
    """How high above z = 0 the plug's face stops [mm]: the least height at which it overlaps the socket nowhere but
    at the latches. Checks the turn and the latches on the way (module docstring)."""
    low, high = SHIFT_RANGE_MM
    if overlap(socket, placed(plug, 0.0, low))[1] <= CLEAR_MM3 or overlap(socket, placed(plug, 0.0, high))[1] > 0:
        raise SystemExit(f"the plug does not enter between {low} and {high} mm -- other sources?")
    while high - low > SHIFT_TOLERANCE_MM:
        middle = (low + high) / 2
        if overlap(socket, placed(plug, 0.0, middle))[1] > CLEAR_MM3:
            low = middle
        else:
            high = middle
    print(f"plug face {high:.3f} mm above the socket's z = 0, stopped by:")
    for piece in overlap(socket, placed(plug, 0.0, high - 0.05))[2]:
        print(f"  {piece} (0.05 mm deeper)")
    at_latches, elsewhere, pieces = overlap(socket, placed(plug, 0.0, high))
    print(f"overlap at the latches {at_latches:.2f} mm^3, elsewhere {elsewhere:.4f} mm^3")
    if at_latches > LATCH_OVERLAP_MAX_MM3:
        raise SystemExit("the latches overlap more than the relaxed arms explain:\n  " + "\n  ".join(pieces))
    for quarter in (1, 2, 3):
        wrong = sum(overlap(socket, placed(plug, quarter * math.pi / 2, high))[:2])
        print(f"turned {90 * quarter} deg about z: {wrong:.0f} mm^3 overlap")
        if wrong < KEYED_MIN_MM3:
            raise SystemExit(f"the plug fits turned by {90 * quarter} deg too -- the key decides nothing")
    return high


def triangles(shape: TopoDS_Shape) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Tessellates ``shape``: corners and their normals, two ``(n, 3, 3)`` arrays [mm], wound outwards, and each
    triangle's CAD face. Each corner's normal is the area-weighted mean over its own CAD face, so the edges between
    faces stay sharp."""
    BRepMesh_IncrementalMesh(shape, LINEAR_DEFLECTION_MM, False, ANGULAR_DEFLECTION_RAD, True)
    corners, normals, faces = [], [], []
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
            weighted = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
            at_node = np.zeros_like(v)
            for k in range(3):
                np.add.at(at_node, f[:, k], weighted)
            at_node /= np.linalg.norm(at_node, axis=1, keepdims=True) + 1e-12
            corners.append(v[f])
            normals.append(at_node[f])
            faces.append(np.full(len(f), len(faces)))
        explorer.Next()
    return np.concatenate(corners), np.concatenate(normals), np.concatenate(faces)


def outside(corners: np.ndarray, faces: np.ndarray) -> np.ndarray:
    """Which of the mated pair's triangles [m] one can see: a CAD face (``faces``) is kept whole if a ray leaves any
    of its triangles for the open, along one of :data:`VIEW_DIRECTIONS` directions from its center or a point toward
    one of its corners. What is dropped is closed in: the contacts and the floors inside the hood, the plug's
    insulator."""
    mesh = trimesh.Trimesh(**trimesh.triangles.to_kwargs(corners), process=False)
    normals = mesh.face_normals
    k = np.arange(VIEW_DIRECTIONS) + 0.5
    polar, azimuth = np.arccos(1 - 2 * k / VIEW_DIRECTIONS), np.pi * (1 + 5**0.5) * k
    directions = np.column_stack([np.cos(azimuth) * np.sin(polar), np.sin(azimuth) * np.sin(polar), np.cos(polar)])
    middle = corners.mean(axis=1)
    starts = [middle] + [0.75 * corners[:, i] + 0.25 * middle for i in range(3)]
    seen = np.zeros(len(corners), dtype=bool)
    for direction in directions:
        for start in starts:
            todo = np.flatnonzero(~seen & (normals @ direction > 0.05))
            if len(todo):
                origins = start[todo] + normals[todo] * 1e-5
                seen[todo[~mesh.ray.intersects_any(origins, np.tile(direction, (len(todo), 1)))]] = True
    return np.isin(faces, faces[seen])


def write(parts: dict[str, tuple[np.ndarray, np.ndarray]], dae_path: str) -> None:
    """One geometry per part, both in :data:`LOOK`, meters, ``Z_UP``."""
    dae = collada.Collada()
    dae.assetInfo.upaxis = asset.UP_AXIS.Z_UP
    dae.assetInfo.created = dae.assetInfo.modified = STAMP
    dae.assetInfo.contributors.append(
        asset.Contributor(
            authoring_tool="husky/scripts/charging_connector_mesh.py",
            source_data="SK2-076C04_mount.STEP, SK6-076C04_plug.STEP",
        )
    )
    color, specular, shininess = LOOK
    nodes = []
    for key, (corners, normals) in parts.items():
        points, point_index = np.unique(
            np.round(corners.reshape(-1, 3), POSITION_DECIMALS), axis=0, return_inverse=True
        )
        directions, direction_index = np.unique(np.round(normals.reshape(-1, 3), 3), axis=0, return_inverse=True)
        effect = material.Effect(
            f"{key}-effect",
            [],
            "phong",
            diffuse=(*color, 1.0),
            specular=(*specular, 1.0),
            shininess=shininess,
            transparent=None,
            transparency=None,
        )
        mat = material.Material(f"{key}-material", key, effect)
        positions = source.FloatSource(f"{key}-positions", points.ravel(), ("X", "Y", "Z"))
        normal_source = source.FloatSource(f"{key}-normals", directions.ravel(), ("X", "Y", "Z"))
        geom = geometry.Geometry(dae, f"{key}-mesh", key, [positions, normal_source])
        inputs = source.InputList()
        inputs.addInput(0, "VERTEX", f"#{key}-positions")
        inputs.addInput(1, "NORMAL", f"#{key}-normals")
        index = np.column_stack([point_index.ravel(), direction_index.ravel()]).astype(np.int32)
        geom.primitives.append(geom.createTriangleSet(index.ravel(), inputs, key))
        dae.effects.append(effect)
        dae.materials.append(mat)
        dae.geometries.append(geom)
        nodes.append(scene.Node(key, children=[scene.GeometryNode(geom, [scene.MaterialNode(key, mat, inputs=[])])]))
        bounds_mm = np.round(np.array([corners.min(axis=(0, 1)), corners.max(axis=(0, 1))], float) * 1000, 1).tolist()
        print(f"{key:6s} {len(corners):6d} triangles, bounds {bounds_mm} mm")
    dae.scenes.append(scene.Scene("charging_connector", nodes))
    dae.scene = dae.scenes[0]
    dae.write(dae_path)
    print(f"{dae_path}: {Path(dae_path).stat().st_size / 1e6:.2f} MB")


def placement(chassis_dae: str) -> None:
    """Prints the socket's joint origin on ``top_chassis_link``: the middle of the right lid's outline in the slope's
    plane, :data:`FLANGE_LIFT_M` proud of the lid, moved by :data:`DOWN_SLOPE_M` and :data:`LEFT_M`, z along the slope's
    normal, y up the slope."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import a200_chassis_mesh as chassis

    corners, _ = chassis.read(chassis_dae)
    labels = chassis.components(corners)
    lids, _ = chassis.hole_caps(corners, labels, chassis.assign(corners, labels)["chassis"])
    lid = lids[lids[:, :, 1].mean(axis=1) < 0].reshape(-1, 3)
    normal = np.array(SLOPE_NORMAL) / np.linalg.norm(SLOPE_NORMAL)
    up_slope = np.cross([0.0, 1.0, 0.0], normal)
    u, y = lid @ up_slope, lid[:, 1]
    print(f"right lid: {np.ptp(u) * 1000:.1f} mm up the slope, {np.ptp(y) * 1000:.1f} mm across")
    origin = up_slope * (u.max() + u.min()) / 2 + np.array([0.0, (y.max() + y.min()) / 2, 0.0])
    origin += normal * (float((lid @ normal).max()) + FLANGE_LIFT_M) - up_slope * DOWN_SLOPE_M
    origin[1] += LEFT_M
    rotation = np.column_stack([np.cross(up_slope, normal), up_slope, normal])
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    pitch = -math.asin(rotation[2, 0]) + 0.0  # no -0.0
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    print(f'<origin xyz="{origin[0]:.5f} {origin[1]:.5f} {origin[2]:.5f}" rpy="{roll:.5f} {pitch:.5f} {yaw:.5f}"/>')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("socket", help="SK2-076C04_mount.STEP")
    parser.add_argument("plug", help="SK6-076C04_plug.STEP")
    parser.add_argument("dae", help="where to write charging_connector.dae")
    parser.add_argument("--chassis", help="Clearpath's top_chassis.dae: print the joint origin as well")
    args = parser.parse_args()
    socket, plug = read_solid(args.socket), read_solid(args.plug)
    shift_mm = seat(socket, plug)
    lift = np.array([0.0, 0.0, FLANGE_FACE_Z_MM])
    parts = {}
    for key, shape in (("socket", socket), ("plug", placed(plug, 0.0, shift_mm))):
        corners, normals, faces = triangles(shape)
        parts[key] = ((corners - lift) / 1000.0, normals, faces + 100_000 * len(parts))
    kept = outside(np.concatenate([p[0] for p in parts.values()]), np.concatenate([p[2] for p in parts.values()]))
    for key, (corners, normals, _) in parts.items():
        mask, kept = kept[: len(corners)], kept[len(corners) :]
        print(f"{key:6s} {len(corners):6d} triangles, {mask.sum():6d} of them seen from outside")
        parts[key] = (corners[mask], normals[mask])
    write(parts, args.dae)
    if args.chassis:
        placement(args.chassis)


if __name__ == "__main__":
    raise SystemExit(main())
