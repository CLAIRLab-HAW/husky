#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["numpy", "open3d==0.19.0", "pycollada", "rtree", "scipy", "trimesh"]
# ///
"""Writes ``d435.dae``: the Intel RealSense D435 in its real materials, in the frame of realsense2_description's mesh.

``realsense2_description/meshes/d435.dae`` (4.58.4, 15.8 MB) is 231186 triangles under ONE effect, diffuse 0.749
gray: the black front, the lenses and the aluminum casing all the same gray (measured 2026-10-10). MuJoCo Menagerie's
``realsense_d435i`` is that very mesh with its parts told apart and colored: "Imported the camera mesh from the URDF
description into Blender and assigned colors", by Binit Shah (its README, read 2026-10-10). Its MJCF ``d435i.xml``
places nine OBJ parts (``d435i_0..8``, 208463 triangles) under seven materials. This script assembles them by the
MJCF's geom poses, registers them onto realsense's mesh, decimates them and writes one geometry and one material per
Menagerie material, so ``clearpath_sensors_description``'s visual origin for ``d435.dae`` holds for this file
unchanged. ``a200_look_patch`` in ``onboard/setup`` points the camera's visual here.

THE FRAME is realsense's, not the Menagerie's: :func:`register` centers the parts' bounding box on realsense's,
tries the four proper sign flips of the principal axes and refines the best by rigid point-to-plane ICP (no scale, no
mirror), and stops when the residual exceeds :data:`MAX_RESIDUAL_MM` or the bounding boxes disagree. On 2026-10-10
both boxes were x -44.9..45.0, y -12.5..12.5, z -25.1..0.0 mm (the front face on z = 0, the casing behind it), and
the pose came out at 0.0086 deg and 0.85 um: the parts lie 0.0014 mm RMS (0.100 mm max) from realsense's surface,
0.0011 mm RMS unmoved. Blender kept the frame, and the registration only confirms it.

THE COLORS are the Menagerie's ``rgba`` as stored (:data:`MATERIALS`): MuJoCo hands them to OpenGL without a color
space conversion, so these are the values the Menagerie's picture of the camera shows, and the twin
(``clair.twin.collada``) shows them the same way. One is changed (:data:`OVERRIDES`): the casing's white saturated in
a render. The Menagerie's default material has ``specular="0" shininess="0.25"``, kept as a black specular and
MuJoCo's shininess scale times 128.

THE PARTS ARE DECIMATED (:data:`PARTS`): all nine are 208463 triangles, the casing alone 134753, and realsense's own
mesh is 15.8 MB. Each part is welded (:func:`welded`) and, if it has more triangles than its target, decimated on its
own by quadric error (Open3D, in millimeters, as ``a200_wheel_mesh.py`` does it and for its reasons). The parts are
cracked along the CAD's faces -- after welding the casing still has 51821 open edges -- and Open3D's default boundary
weight kept them within 0.044 mm where a boundary weight of 1e6 left 1.9 mm (Cameras_Gray at 20000 triangles,
2026-10-10). On 2026-10-10 the DAE came to 23326 triangles and 1.46 MB; every part stayed within 0.042 mm RMS of the
Menagerie's surface (:data:`MAX_RMS_DEVIATION_MM`), and the few sampled points off by more than 0.2 mm, 1.09 mm at
most on Cameras_Gray and 2.21 mm on the casing, all lie 1.9 mm or more inside the camera's bounding box, on the
inner structure behind the front: a render front, back and top showed the lens rims, the vent slots and the screw
holes as in realsense's mesh. Normals are made per corner with a crease angle (:func:`corner_normals`).

SOURCES AND LICENSE. The Menagerie's ``realsense_d435i`` is released under the Apache License 2.0, "Copyright 2023
Intel Corporation. All Rights Reserved." (its ``LICENSE``; the model has no ``NOTICE``; upstream
google-deepmind/mujoco_menagerie, read 2026-10-10), and realsense2_description's mesh it was made from is Apache-2.0
as well (realsense-ros). The license allows redistributing a derived work if it carries the license, the copyright
notice and a statement of the changes (section 4): the DAE's ``<asset><contributor><copyright>`` carries the notice
and the changes (:data:`NOTICE`), and the license text lies beside the mesh as ``meshes/d435.LICENSE``. The local copy
of the Menagerie model is ``~/Desktop/Projekte/mujoco-mia/assets/realsense`` (the MJCF there is upstream's plus a
camera element; the OBJ parts upstream's).

    docker cp offboard-plant-mock-1:/opt/ros/jazzy/share/realsense2_description/meshes/d435.dae /tmp/d435.dae
    uv run onboard/extras/tools/d435_mesh.py /tmp/d435.dae ~/Desktop/Projekte/mujoco-mia/assets/realsense/d435i.xml \\
        onboard/extras/src/husky_extras_description/meshes/d435.dae
"""

from __future__ import annotations

import argparse
import datetime
import os
import xml.etree.ElementTree as ET

import collada
import numpy as np
import open3d as o3d
import trimesh
from collada import asset, geometry, material, scene, source
from scipy.spatial import cKDTree

#: The Menagerie's materials [rgb as stored in ``d435i.xml``], checked against the MJCF, each written as one material
#: of the DAE. The parts that share a material are joined by it: ``IR_Lens`` (``d435i_0``, ``d435i_3``) and
#: ``Black_Acrylic`` (``d435i_5``, ``d435i_6``).
MATERIALS = {
    "IR_Lens": (0.035601, 0.035601, 0.035601),
    "IR_Emitter_Lens": (0.287440, 0.665387, 0.327778),
    "IR_Rim": (0.799102, 0.806952, 0.799103),
    "Cameras_Gray": (0.296138, 0.296138, 0.296138),
    "Black_Acrylic": (0.070360, 0.070360, 0.070360),
    "RGB_Pupil": (0.087140, 0.002866, 0.009346),
    "Metal_Casing": (1.0, 1.0, 1.0),
}
#: Where the DAE's color differs from the Menagerie's [rgb]. ``Metal_Casing``'s white saturates as soon as a light
#: falls on it: rendered through ``clair.twin.collada`` in MuJoCo on 2026-10-10, the casing's back and top were one
#: flat white with no shading left, where the D435's casing is bead-blasted aluminum, a light silver gray. Taken a
#: little below the ``IR_Rim``'s 0.8, so the polished rims around the lenses still stand out from it.
OVERRIDES = {"Metal_Casing": (0.72, 0.73, 0.75)}
#: The Menagerie's ``<default class="d435i"><material specular="0" shininess="0.25"/>``: specular as a gray, shininess
#: on MuJoCo's 0..1 scale, which maps onto OpenGL's 0..128.
SPECULAR = 0.0
SHININESS = 0.25 * 128
#: The MJCF's visual geoms: each OBJ part's material, and the triangle count it is decimated to when it has more
#: (of 200, 1273, 3492, 316, 55527, 4940, 7250, 712, 134753 on 2026-10-10). A part missing, one more or another
#: material stops the script.
PARTS = {
    "d435i_0": ("IR_Lens", None),
    "d435i_1": ("IR_Emitter_Lens", 600),
    "d435i_2": ("IR_Rim", 1000),
    "d435i_3": ("IR_Lens", None),
    "d435i_4": ("Cameras_Gray", 5000),
    "d435i_5": ("Black_Acrylic", 2000),
    "d435i_6": ("Black_Acrylic", 1500),
    "d435i_7": ("RGB_Pupil", None),
    "d435i_8": ("Metal_Casing", 12000),
}
#: How far the registered parts may lie from realsense's surface, RMS [mm]; 0.0014 mm on 2026-10-10.
MAX_RESIDUAL_MM = 0.2
#: How far the two meshes' bounding boxes may differ after registration [mm].
BOX_TOLERANCE_MM = 0.5
#: How far ICP pairs a point with realsense's surface [mm].
ICP_REACH_MM = 1.0
#: How many points :func:`register` and :func:`deviation` sample on a surface, with a fixed seed.
SAMPLES = 20000
#: How far a decimated part's surface may lie from the Menagerie's, either way, at most and RMS [mm]: 2.21 and 0.042 mm
#: on 2026-10-10, the maximum on the casing's inner structure (see the module docstring).
MAX_DEVIATION_MM = 3.0
MAX_RMS_DEVIATION_MM = 0.1
#: Two faces at a corner share its normal up to this angle between them [rad]; beyond it the edge is shaded sharp.
CREASE_RAD = np.radians(45.0)
#: Written into every run's ``<created>``/``<modified>``, so a second run on the same sources writes the same file.
STAMP = datetime.datetime(2026, 10, 10)
#: The sources' license, kept with the derived file as its section 4 asks: the notice and what was changed.
NOTICE = """Derived from MuJoCo Menagerie's realsense_d435i (google-deepmind/mujoco_menagerie, d435i.xml and
assets/d435i_0..8.obj), itself derived from realsense2_description/meshes/d435.dae (IntelRealSense/realsense-ros)
with colors assigned by Binit Shah. Changed by onboard/extras/tools/d435_mesh.py (2026-10-10): the parts registered
onto realsense2_description's d435.dae (4.58.4), the large ones decimated, written as one COLLADA geometry and
material per Menagerie material, the Metal_Casing's white darkened to a light aluminum gray.

Copyright 2023 Intel Corporation. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

A copy of the License lies beside this file as d435.LICENSE.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License."""

Part = tuple[np.ndarray, np.ndarray]
"""A part's triangles and their corner normals, two ``(n, 3, 3)`` arrays [mm]."""


def read_realsense(dae_path: str) -> trimesh.Trimesh:
    """realsense2_description's mesh in its scene's frame [mm], its one material checked."""
    dae = collada.Collada(dae_path)
    triangles, effects = [], set()
    for geom in dae.scene.objects("geometry"):
        for prim in geom.primitives():
            triangles.append(prim.vertex[prim.vertex_index])
            effects.add(prim.material.effect.id)
    if len(effects) != 1:
        raise SystemExit(f"realsense: {len(effects)} effects in {dae_path}, expected the one gray -- source changed?")
    mesh = welded(np.concatenate(triangles) * 1000.0)
    print(f"realsense: {len(mesh.faces)} triangles, bounds {bounds_mm(mesh.vertices)} mm")
    return mesh


def read_menagerie(xml_path: str) -> dict[str, trimesh.Trimesh]:
    """The MJCF's visual parts [mm], each placed by its geom's ``pos``/``quat`` in the body's frame, keyed by mesh
    name; the material each names and its color checked against :data:`PARTS` and :data:`MATERIALS`."""
    root = ET.parse(xml_path).getroot()
    meshdir = os.path.join(os.path.dirname(xml_path), root.find("compiler").get("meshdir", ""))
    colors = {
        m.get("name"): tuple(float(v) for v in m.get("rgba").split()[:3]) for m in root.iterfind("asset/material")
    }
    for name, rgb in MATERIALS.items():
        if not np.allclose(colors.get(name, ()), rgb):
            raise SystemExit(f"menagerie: material {name} is {colors.get(name)}, expected {rgb} -- model changed?")
    parts = {}
    for geom in root.iter("geom"):
        if geom.get("class") != "visual":
            continue
        name = geom.get("mesh")
        if PARTS.get(name, (None,))[0] != geom.get("material"):
            raise SystemExit(f"menagerie: geom {name} with material {geom.get('material')} -- model changed?")
        if any(geom.get(key) for key in ("euler", "axisangle", "xyaxes", "zaxis")):
            raise SystemExit(f"menagerie: geom {name} is oriented other than by quat -- not handled")
        pose = trimesh.transformations.quaternion_matrix([float(v) for v in geom.get("quat", "1 0 0 0").split()])
        pose[:3, 3] = [float(v) for v in geom.get("pos", "0 0 0").split()]
        obj = trimesh.load(os.path.join(meshdir, f"{name}.obj"), force="mesh", process=False)
        parts[name] = welded(trimesh.transform_points(obj.triangles.reshape(-1, 3), pose) * 1000.0)
    if parts.keys() != PARTS.keys():
        raise SystemExit(f"menagerie: parts {sorted(parts)}, expected {sorted(PARTS)} -- model changed?")
    return parts


def welded(corners: np.ndarray) -> trimesh.Trimesh:
    """A mesh of the triangles whose corners ``corners`` lists three by three [mm], corners that agree to 0.1 um
    shared. The OBJ parts give every face its own corners (they carry per-face normals and texture coordinates), and
    a triangle soup cannot be decimated."""
    points, index = np.unique(np.round(corners.reshape(-1, 3), 4), axis=0, return_inverse=True)
    return trimesh.Trimesh(points, index.reshape(-1, 3), process=False)


def bounds_mm(points: np.ndarray) -> list:
    """Min and max corner of ``points`` [mm], to 0.1 mm."""
    corners = points.reshape(-1, 3)
    return np.round([corners.min(axis=0), corners.max(axis=0)], 1).tolist()


def register(parts: dict[str, trimesh.Trimesh], realsense: trimesh.Trimesh) -> np.ndarray:
    """The rigid pose that puts the assembled parts onto realsense's mesh [mm]: bounding boxes centered, the best of
    the four proper sign flips of the principal axes, refined by point-to-plane ICP (Open3D); checked against
    :data:`MAX_RESIDUAL_MM` and :data:`BOX_TOLERANCE_MM`."""
    assembled = trimesh.util.concatenate(list(parts.values()))
    points, _ = trimesh.sample.sample_surface(assembled, SAMPLES, seed=0)
    target, faces = trimesh.sample.sample_surface(realsense, 10 * SAMPLES, seed=0)
    tree = cKDTree(target)

    def axes(cloud: np.ndarray) -> np.ndarray:
        _, vectors = np.linalg.eigh(np.cov((cloud - cloud.mean(axis=0)).T))
        return vectors * np.sign(np.linalg.det(vectors))

    here, there = axes(points), axes(target)
    middle_here = (assembled.bounds[0] + assembled.bounds[1]) / 2
    middle_there = (realsense.bounds[0] + realsense.bounds[1]) / 2
    candidates = []
    for flip in ((1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)):
        coarse = np.eye(4)
        coarse[:3, :3] = there @ np.diag(flip) @ here.T
        coarse[:3, 3] = middle_there - coarse[:3, :3] @ middle_here
        rms = float(np.sqrt(np.mean(tree.query(trimesh.transform_points(points, coarse))[0] ** 2)))
        candidates.append((rms, flip, coarse))
    coarse_rms, flip, coarse = min(candidates, key=lambda c: c[0])
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    surface = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target))
    surface.normals = o3d.utility.Vector3dVector(realsense.face_normals[faces])
    fit = o3d.pipelines.registration.registration_icp(
        cloud,
        surface,
        ICP_REACH_MM,
        coarse,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
    )
    pose = np.asarray(fit.transformation)
    distance = trimesh.proximity.closest_point(realsense, trimesh.transform_points(points, pose))[1]
    rms = float(np.sqrt(np.mean(distance**2)))
    unmoved = trimesh.proximity.closest_point(realsense, points)[1]
    angle = np.degrees(np.arccos(np.clip((np.trace(pose[:3, :3]) - 1) / 2, -1, 1)))
    print(
        f"register: principal axes flipped by {flip} ({coarse_rms:.3f} mm RMS to realsense's samples); "
        f"ICP's pose turns {angle:.4f} deg and moves {np.linalg.norm(pose[:3, 3]) * 1000:.2f} um; "
        f"residual {rms:.4f} mm RMS, {distance.max():.3f} mm max (unmoved {np.sqrt(np.mean(unmoved**2)):.4f} mm RMS)"
    )
    if rms > MAX_RESIDUAL_MM:
        raise SystemExit(f"register: residual {rms:.3f} mm RMS, more than {MAX_RESIDUAL_MM} mm")
    placed = trimesh.transform_points(assembled.vertices, pose)
    box = np.array([placed.min(axis=0), placed.max(axis=0)])
    if np.abs(box - realsense.bounds).max() > BOX_TOLERANCE_MM:
        raise SystemExit(f"register: bounding boxes {box.tolist()} and {realsense.bounds.tolist()} mm disagree")
    print(f"register: the parts' bounds {bounds_mm(placed)} mm after registration")
    return pose


def deviation(a: trimesh.Trimesh, b: trimesh.Trimesh) -> tuple[float, float]:
    """The sampled Hausdorff distance between two surfaces and the RMS of the same distances [mm]: from
    :data:`SAMPLES` points on each to the other.

    In millimeters on purpose: trimesh 5.1.1's ``closest_point`` missed the closest triangle for 12405 of 20000
    points on a part in meters, 0.28 mm off for a mesh against itself, and found it for all in millimeters
    (2026-10-10)."""
    distances = []
    for here, there in ((a, b), (b, a)):
        points, _ = trimesh.sample.sample_surface(here, SAMPLES, seed=0)
        distances.append(trimesh.proximity.closest_point(there, points)[1])
    distances = np.concatenate(distances)
    return float(distances.max()), float(np.sqrt(np.mean(distances**2)))


def corner_normals(mesh: trimesh.Trimesh) -> np.ndarray:
    """Each corner's normal, ``(n, 3, 3)``: the area-weighted mean of the normals of the faces at its vertex that lie
    within :data:`CREASE_RAD` of its own face -- ``a200_wheel_mesh.py``'s, for its reasons."""
    weighted = mesh.face_normals * mesh.area_faces[:, None]
    adjacent = mesh.vertex_faces[mesh.faces]  # (n, 3, k), padded with -1
    gentle = (mesh.face_normals[adjacent] * mesh.face_normals[:, None, None, :]).sum(-1) > np.cos(CREASE_RAD)
    normals = (weighted[adjacent] * ((adjacent >= 0) & gentle)[..., None]).sum(axis=2)
    length = np.linalg.norm(normals, axis=-1, keepdims=True)
    if not length.all():
        raise SystemExit("normals: a corner has no normal -- a degenerate triangle?")
    return normals / length


def decimate(name: str, mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """``mesh`` [mm] decimated to its count in :data:`PARTS` by quadric error; as it is if it has fewer. Checked
    against :data:`MAX_DEVIATION_MM`."""
    target = PARTS[name][1]
    if target is None or len(mesh.faces) <= target:
        return mesh
    simple = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(mesh.vertices), o3d.utility.Vector3iVector(mesh.faces)
    ).simplify_quadric_decimation(target)
    simple = trimesh.Trimesh(np.asarray(simple.vertices), np.asarray(simple.triangles), process=False)
    simple.remove_unreferenced_vertices()
    worst, rms = deviation(mesh, simple)
    print(
        f"decimate: {name} {len(mesh.faces)} -> {len(simple.faces)} triangles, "
        f"deviation {worst:.3f} mm, {rms:.4f} mm RMS"
    )
    if worst > MAX_DEVIATION_MM or rms > MAX_RMS_DEVIATION_MM:
        raise SystemExit(
            f"decimate: {name} deviates {worst:.2f} mm, {rms:.3f} mm RMS -- more than {MAX_DEVIATION_MM} mm or "
            f"{MAX_RMS_DEVIATION_MM} mm RMS"
        )
    return simple


def write(looks: dict[str, Part], dae_path: str) -> None:
    """One geometry and one material per Menagerie material [mm in, m out], ``Z_UP`` as realsense's source -- the shape
    ``clair.twin.collada`` reads the colors from. A corner's position and normal share one index."""
    dae = collada.Collada()
    dae.assetInfo.upaxis = asset.UP_AXIS.Z_UP
    dae.assetInfo.created = dae.assetInfo.modified = STAMP
    dae.assetInfo.contributors.append(
        asset.Contributor(
            author="Intel Corporation; Binit Shah (colors, MuJoCo Menagerie); d435_mesh.py (registration, decimation)",
            authoring_tool="onboard/extras/tools/d435_mesh.py",
            copyright=NOTICE,
            source_data="https://github.com/google-deepmind/mujoco_menagerie/tree/main/realsense_d435i",
        )
    )
    nodes = []
    for key, (triangles, normals) in looks.items():
        color = OVERRIDES.get(key, MATERIALS[key])
        # Positions to 10 um and normals to 0.01: the file is text, and more digits are bytes no viewer shows. One
        # index per corner for both: a corner's position and normal are one vertex.
        corners = np.c_[np.round(triangles.reshape(-1, 3) / 1000.0, 5), np.round(normals.reshape(-1, 3), 2)]
        vertices, index = np.unique(corners, axis=0, return_inverse=True)
        effect = material.Effect(
            f"{key}-effect",
            [],
            "phong",
            diffuse=(*color, 1.0),
            specular=(SPECULAR, SPECULAR, SPECULAR, 1.0),
            shininess=SHININESS,
            transparent=None,
            transparency=None,
        )
        mat = material.Material(f"{key}-material", key, effect)
        positions = source.FloatSource(f"{key}-positions", vertices[:, :3].ravel(), ("X", "Y", "Z"))
        normal_source = source.FloatSource(f"{key}-normals", vertices[:, 3:].ravel(), ("X", "Y", "Z"))
        geom = geometry.Geometry(dae, f"{key}-mesh", key, [positions, normal_source])
        inputs = source.InputList()
        inputs.addInput(0, "VERTEX", f"#{key}-positions")
        inputs.addInput(0, "NORMAL", f"#{key}-normals")
        geom.primitives.append(geom.createTriangleSet(index.ravel().astype(np.int32), inputs, key))
        dae.effects.append(effect)
        dae.materials.append(mat)
        dae.geometries.append(geom)
        nodes.append(scene.Node(key, children=[scene.GeometryNode(geom, [scene.MaterialNode(key, mat, inputs=[])])]))
        print(f"{key:15s} {len(triangles):6d} triangles, rgb {list(color)}, bounds {bounds_mm(triangles)} mm")
    dae.scenes.append(scene.Scene("d435", nodes))
    dae.scene = dae.scenes[0]
    dae.write(dae_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("realsense", help="realsense2_description's d435.dae")
    parser.add_argument("menagerie", help="MuJoCo Menagerie's realsense_d435i/d435i.xml")
    parser.add_argument("dae", help="where to write d435.dae")
    args = parser.parse_args()
    realsense = read_realsense(args.realsense)
    parts = read_menagerie(args.menagerie)
    pose = register(parts, realsense)
    looks: dict[str, list[Part]] = {key: [] for key in MATERIALS}
    for name, mesh in parts.items():
        simple = decimate(name, mesh).apply_transform(pose)
        looks[PARTS[name][0]].append((simple.triangles, corner_normals(simple)))
    write({key: tuple(np.concatenate(side) for side in zip(*found)) for key, found in looks.items()}, args.dae)
    print(f"{args.dae}: {os.path.getsize(args.dae) / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
