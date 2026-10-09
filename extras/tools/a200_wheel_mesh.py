#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["numpy", "open3d==0.19.0", "pycollada", "rtree", "scipy", "trimesh"]
# ///
"""Writes ``a200_indoor_wheel.dae``: Clearpath's a200 indoor tire on the a200-0553's real rim.

Clearpath's ``clearpath_platform_description/meshes/a200/wheels/indoor.dae`` (2.9.15) draws a two-piece rim with a hub
face on the outer side; the a200-0553's rims look different (the owner, 2026-10-10). The owner exported the real one
from Clearpath's own CAD, "034991-MOBILITY_Simplified mCAD, STEP-Husky A200, Skid Steer", as a glTF of ONE wheel's rim
in the robot's base frame: the front-right wheel. This script keeps Clearpath's tire with its own look and puts that
rim into it.

THE TIRE is the one of the source's three geometries whose outer radius exceeds :data:`TIRE_MIN_OUTER_M`:
``Shape_IndexedFaceSet_313`` (r 75.0..162.5 mm, y +-52.5 mm, diffuse 0.023), checked in a section through the axis on
2026-10-10. The other two (311, 312, r 32.5..89.4 mm) are Clearpath's rim halves and are left out. Its inner face, the
bead seat, is a cylinder of r 75.0 mm over y +-31.5 mm.

THE RIM: the glTF holds four bodies (:data:`BODIES`), each split by trimesh into one geometry per CAD face, all
baseColor (49, 52, 52)/255 with metallic 0. The two rim halves are pressed shells bolted together at a central web
(4 mm, r 22.0..65.6 mm, bore r 22.0 mm). The two smaller bodies looked off-center by their bounding boxes, but both are
round about the wheel axis and turn with the wheel, so both are kept (2026-10-10, section and radii about the axis):

- ``mesh0``, the hub: a cylinder r 22.2 mm, 25.4 mm long, pressed into the web's bore (r 22.0 mm) and standing inboard
  of it, with a 17 mm bore and a 5 x 2.7 mm keyway -- keyed to the drive shaft, so it is what turns the wheel.
- ``mesh8``, the hub plate: 6.3 mm thick, a disk r 60 mm with a flat and three tabs at one side, a 17 mm bore on the
  axis, lying against the web's OUTER face with no gap. Nothing that stands still can sit outboard of the wheel's web:
  it is clamped to the web.

The rim's axis comes from two of its round shells (:data:`AXIS_SHELLS`, the flat annuli r 78.1..83.0 mm where the
flanges step): each must be round about its bounding box's center to :data:`ROUND_TOLERANCE_M`, and both centers must
agree. On 2026-10-10 that put the axis at x 256.0, z 36.0 mm (base frame), the front axle at ``wheel_base/2``. The
rim is moved so the axis lies on the wheel link's y axis, and turned by pi about z, so its OUTER half (``mesh44``,
toward -y on the right wheel) lies on +y like the outer face of Clearpath's mesh (``a200_look_patch`` turns the right
wheels back by pi).

Along the axle the rim is CENTERED ON THE TIRE'S BEAD SEAT, not kept at its offset from the wheel link: the rim's
halves span y -314..-236 mm, symmetric about -275.0 mm, where the URDF's wheel link sits at -277.5 mm
(``wheel_track/2``). Kept at that offset, the outer flange reached 3.4 mm into the tire's bead and the inner one stood
1.6 mm off it (section, 2026-10-10). Centered, the flanges' inner walls, 61.2 mm apart, clamp the tire's 63.0 mm bead
by 0.9 mm on each side: a real tire sits centered on its rim, and Clearpath's tire is centered on the link.

The fit is checked and printed (:func:`check_fit`): the rim's bead seat must stay below the tire's (the radial gap,
0.8 mm on 2026-10-10), and the flanges may reach into the tire's bead faces by no more than
:data:`MAX_FLANGE_OVERLAP_M`. Otherwise the script stops.

THE RIM IS DECIMATED to a tenth (:func:`decimate`): the CAD's tessellation gave it 115106 triangles and the DAE 7.1 MB.
Each body is decimated on its own by quadric error to its count in :data:`BODIES`, 10650 in all, and must stay within
:data:`MAX_DEVIATION_M` of the CAD; on 2026-10-10 the four stayed within 0.15 mm, the hub plate's flat and tabs,
the bores and the keyway kept (the web has no openings), and the DAE came to 1.2 MB. The tire is not decimated
(3952 triangles).

The rim's look is :data:`RIM_LOOK`: black painted steel, as on the robot, set apart from the tire's near-black rubber
by a lighter diffuse and a highlight. The tire keeps the source's own effect and normals; the rim's normals are made
per corner by :func:`corner_normals`.

SOURCES AND LICENSES. Clearpath's ``indoor.dae`` is BSD-3-Clause (``LICENSE`` of ``clearpath_common``, "Copyright (c)
2023, clearpathrobotics"); the DAE's first ``<asset><contributor>`` carries the notice in full (:data:`NOTICE`). The
rim's CAD comes from Clearpath's download without a BSD license, so the glTF is NOT in the repo -- the same practice as
``onboard/rs16/tools/rs16_mesh.py`` with RoboSense's STEP: the owner keeps it at ``onboard/a200_rims.gltf`` (untracked,
4.8 MB, 2026-10-10), and the derived, decimated DAE is committed, like ``rs16.dae``. The second
contributor names the CAD.

    docker cp offboard-plant-mock-1:/opt/ros/jazzy/share/clearpath_platform_description/meshes/a200/wheels/indoor.dae \\
        /tmp/indoor.dae
    uv run onboard/extras/tools/a200_wheel_mesh.py /tmp/indoor.dae onboard/a200_rims.gltf \\
        onboard/extras/src/husky_extras_description/meshes/a200_indoor_wheel.dae

``indoor.dae`` is not patched by anything here, so the apt copy is the source.
"""

from __future__ import annotations

import argparse
import datetime

import collada
import numpy as np
import open3d as o3d
import trimesh
from collada import asset, geometry, material, scene, source

#: The glTF's bodies by their mesh name (Onshape export, 2026-10-10): what each is, and how many triangles it is
#: decimated to (of 56448, 56070, 1116, 1472). A body missing or one more stops the script.
BODIES = {
    "mesh16": ("inner rim half", 5000),
    "mesh44": ("outer rim half", 5000),
    "mesh0": ("hub, keyed to the shaft", 280),
    "mesh8": ("hub plate, on the web's outer face", 370),
}
#: The rim half that faces outward on the wheel: toward -y in the glTF, the front-right wheel.
OUTER_HALF = "mesh44"
#: The rim's round shells the axis is taken from: the flat annuli r 78.1..83.0 mm where the flanges step, one per half.
AXIS_SHELLS = ("mesh16_mesh_9", "mesh44_mesh_9")
#: How far a round shell's corners may lie off its two radii, and its center off the other shell's [m]; the export is
#: round to 0.01 mm (2026-10-10).
ROUND_TOLERANCE_M = 0.0001
#: How far a decimated body's surface may lie from the CAD's, either way [m]; on 2026-10-10 the four bodies stayed
#: within 0.15 mm.
MAX_DEVIATION_M = 0.0005
#: How many points :func:`deviation` samples on each surface, with a fixed seed.
DEVIATION_SAMPLES = 20000
#: Two faces at a corner of the decimated rim share its normal up to this angle between them [rad]; beyond it the edge
#: is shaded sharp, as the CAD's faces meet.
CREASE_RAD = np.radians(60.0)
#: The tire's tread radius [m]: the a200-0553's ROLLING radius, ``platform_velocity_controller.wheel_radius`` in
#: ``onboard/setup/config/robot.yaml`` -- a straight run of 2.515 m on a folding rule turned the wheels by 17.026 rad
#: (2026-10-07, R58), and the odometry agrees to 0.1 % since; by tape 29-30 cm across unloaded. Clearpath's tire
#: reaches 162.5 mm (the macro's nominal 0.1651 m, 13 inch).
TREAD_RADIUS_M = 0.1477
#: A geometry of Clearpath's mesh reaching beyond this radius is the tire [m]: the tire's outer radius is 162.5 mm,
#: Clearpath's rim halves end at 89.4 mm (2026-10-10).
TIRE_MIN_OUTER_M = 0.15
#: The tire's corners within this radius of its smallest one are its bead seat [m].
BEAD_TOLERANCE_M = 0.0005
#: How far inside the tire's bead faces the rim's seat is measured [m]: the flanges' fillets rise past the bead radius
#: from 1.7 mm inside them (2026-10-10).
FILLET_M = 0.003
#: How far the flanges' inner walls may reach into the tire's bead faces [m]: the tire's bead is 63.0 mm wide, the rim
#: 61.2 mm between its flanges, 0.9 mm on each side (2026-10-10).
MAX_FLANGE_OVERLAP_M = 0.0015
#: The rim's look [sRGB diffuse, sRGB specular, shininess]: black painted steel, as the a200-0553's rims are (the owner,
#: 2026-10-10; the glTF's dark gray (49, 52, 52)/255 is not). Black, but apart from the tire's rubber: a diffuse of
#: 0.06 against the tire's 0.023, a touch of blue, and a highlight the tire's ``lambert`` has none of. The twin
#: (``clair.twin.collada``) takes the diffuse alone; Foxglove and RViz shade with all three.
RIM_LOOK = ((0.06, 0.06, 0.07), (0.5, 0.5, 0.5), 40.0)
#: Written into every run's ``<created>``/``<modified>``, so a second run on the same sources writes the same file.
STAMP = datetime.datetime(2026, 10, 10)
#: The CAD the rim comes from, as Clearpath names it.
CAD = "034991-MOBILITY_Simplified mCAD, STEP-Husky A200, Skid Steer"
#: The tire's license, kept with the derived file as its clause 2 asks.
NOTICE = """Tire derived from clearpath_platform_description/meshes/a200/wheels/indoor.dae (clearpath_common 2.9.15):
its tire geometry (Shape_IndexedFaceSet_313) taken as is by onboard/extras/tools/a200_wheel_mesh.py.

BSD 3-Clause License

Copyright (c) 2023, clearpathrobotics

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."""
#: Where the rim comes from: Clearpath's CAD, not BSD-licensed, so not redistributed as a source.
RIM_NOTE = f"""Rim (two halves, hub, hub plate) from Clearpath Robotics' CAD "{CAD}", exported by the owner
as a glTF (Onshape, 2026-10-10), decimated per body, recentered on the wheel axis and turned by pi about z by
onboard/extras/tools/a200_wheel_mesh.py. The CAD itself is not redistributed."""


Part = tuple[np.ndarray, np.ndarray]
"""A part's triangles and their corner normals, two ``(n, 3, 3)`` arrays [m]."""


def radius(points: np.ndarray) -> np.ndarray:
    """Distance from the y axis, the wheel's."""
    return np.hypot(points[..., 0], points[..., 2])


def read_tire(dae_path: str) -> tuple[Part, dict]:
    """The tire of Clearpath's wheel and its effect's look: the one geometry reaching beyond
    :data:`TIRE_MIN_OUTER_M`, centered on the y axis."""
    dae = collada.Collada(dae_path)
    found = []
    for geom in dae.scene.objects("geometry"):
        prims = list(geom.primitives())
        triangles = np.concatenate([prim.vertex[prim.vertex_index] for prim in prims])
        if radius(triangles).max() > TIRE_MIN_OUTER_M:
            normals = np.concatenate([prim.normal[prim.normal_index] for prim in prims])
            found.append((geom.original.id, (triangles, normals), prims[0].material.effect))
    if len(found) != 1:
        raise SystemExit(f"tire: {len(found)} geometries beyond r {TIRE_MIN_OUTER_M} m, expected 1 -- source changed?")
    name, (triangles, normals), effect = found[0]
    middle = (triangles.min(axis=(0, 1)) + triangles.max(axis=(0, 1))) / 2
    if np.abs(middle).max() > ROUND_TOLERANCE_M:
        raise SystemExit(f"tire: {name} is centered at {middle} m, not on the wheel link's origin -- source changed?")
    look = {"diffuse": tuple(effect.diffuse[:3]), "shading": effect.shadingtype}
    print(f"tire: {name}, r {radius(triangles).min() * 1000:.1f}..{radius(triangles).max() * 1000:.1f} mm")
    return (triangles, normals), look


def read_rim(gltf_path: str) -> dict[str, Part]:
    """Each glTF body's triangles and corner normals in the glTF's frame, keyed by its CAD face's geometry name."""
    loaded = trimesh.load(gltf_path, force="scene", process=False)
    faces: dict[str, Part] = {}
    for node in loaded.graph.nodes_geometry:
        transform, name = loaded.graph[node]
        mesh = loaded.geometry[name]
        rotation = transform[:3, :3]
        points = trimesh.transform_points(mesh.vertices, transform)
        normals = mesh.vertex_normals @ rotation.T
        faces[name] = (points[mesh.faces], normals[mesh.faces])
    bodies = {body_of(name) for name in faces}
    if bodies != set(BODIES):
        raise SystemExit(f"rim: bodies {sorted(bodies)} in the glTF, expected {sorted(BODIES)} -- export changed?")
    return faces


def body_of(name: str) -> str:
    """The glTF body a CAD face's geometry belongs to: ``mesh16_mesh_9`` -> ``mesh16``."""
    return name.split("_mesh")[0]


def deviation(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    """The larger of the two one-sided distances between two surfaces [m]: from :data:`DEVIATION_SAMPLES` points on
    each to the other, a sampled Hausdorff distance."""
    worst = 0.0
    for here, there in ((a, b), (b, a)):
        points, _ = trimesh.sample.sample_surface(here, DEVIATION_SAMPLES, seed=0)
        worst = max(worst, float(trimesh.proximity.closest_point(there, points)[1].max()))
    return worst


def corner_normals(mesh: trimesh.Trimesh) -> np.ndarray:
    """Each corner's normal, ``(n, 3, 3)``: the area-weighted mean of the normals of the faces at its vertex that lie
    within :data:`CREASE_RAD` of its own face.

    ``trimesh.graph.smooth_shade`` was looked at and not taken: it smooths over connected regions of gentle edges, so a
    sharp edge is smoothed as soon as its two faces connect through gentle ones elsewhere (2026-10-10).
    """
    weighted = mesh.face_normals * mesh.area_faces[:, None]
    adjacent = mesh.vertex_faces[mesh.faces]  # (n, 3, k), padded with -1
    gentle = (mesh.face_normals[adjacent] * mesh.face_normals[:, None, None, :]).sum(-1) > np.cos(CREASE_RAD)
    normals = (weighted[adjacent] * ((adjacent >= 0) & gentle)[..., None]).sum(axis=2)
    length = np.linalg.norm(normals, axis=-1, keepdims=True)
    if not length.all():
        raise SystemExit("rim: a decimated corner has no normal -- a degenerate triangle?")
    return normals / length


def decimate(faces: dict[str, Part]) -> dict[str, Part]:
    """Each glTF body decimated to its triangle count in :data:`BODIES`, by quadric error (Open3D's
    ``simplify_quadric_decimation``), with :func:`corner_normals`; keyed by body, in the glTF's frame.

    A body is decimated as a whole, its CAD faces welded where they share an edge, so the decimation may cross them;
    bodies are decimated apart. The deviation from the CAD must stay below :data:`MAX_DEVIATION_M`. Open3D, not the
    ``fast-simplification`` behind ``trimesh.Trimesh.simplify_quadric_decimation`` nor ``pyfqmr``: both are the
    fast-quadric-mesh-simplification scheme of rising error thresholds, and on a rim half they stalled at 8400..9500
    triangles at 0.2..0.9 mm, where Open3D's ordered collapses reach 5000 within 0.15 mm (2026-10-10).
    """
    decimated = {}
    for body, (what, target) in BODIES.items():
        triangles = np.concatenate([part[0] for name, part in faces.items() if body_of(name) == body])
        # The CAD faces share their edges' corners to 0.1 um; welded, a body is closed.
        points, index = np.unique(np.round(triangles.reshape(-1, 3), 7), axis=0, return_inverse=True)
        cad = trimesh.Trimesh(points, index.reshape(-1, 3), process=False)
        if not cad.is_watertight:
            raise SystemExit(f"rim: {body} ({what}) is not closed after welding its CAD faces -- export changed?")
        # Decimated in millimeters: in meters Open3D's quadrics, of squared distances near 1e-8, kept the rim halves
        # only within 0.21 and 0.27 mm, in millimeters within 0.14 and 0.15 mm (2026-10-10).
        simple = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(cad.vertices * 1000), o3d.utility.Vector3iVector(cad.faces)
        )
        simple = simple.simplify_quadric_decimation(target)
        mesh = trimesh.Trimesh(np.asarray(simple.vertices) / 1000, np.asarray(simple.triangles), process=False)
        off = deviation(cad, mesh)
        print(f"rim: {body} ({what}) {len(cad.faces)} -> {len(mesh.faces)} triangles, deviation {off * 1000:.3f} mm")
        if off > MAX_DEVIATION_M:
            raise SystemExit(
                f"rim: {body} deviates {off * 1000:.2f} mm from the CAD, more than {MAX_DEVIATION_M * 1000} mm"
            )
        decimated[body] = (mesh.triangles, corner_normals(mesh))
    return decimated


def axis(faces: dict[str, Part]) -> np.ndarray:
    """The rim's axis as its (x, z) in the glTF's frame [m], from :data:`AXIS_SHELLS`: each shell flat across y and
    round about its bounding box's center, and both centers the same."""
    centers = []
    for name in AXIS_SHELLS:
        if name not in faces:
            raise SystemExit(f"axis: no shell {name} in the glTF -- export changed?")
        corners = faces[name][0].reshape(-1, 3)
        if np.ptp(corners[:, 1]) > ROUND_TOLERANCE_M:
            raise SystemExit(f"axis: {name} spans {np.ptp(corners[:, 1]) * 1000:.2f} mm in y, not flat across the axle")
        center = (corners[:, [0, 2]].min(axis=0) + corners[:, [0, 2]].max(axis=0)) / 2
        radii = np.hypot(*(corners[:, [0, 2]] - center).T)
        off = np.minimum(np.abs(radii - radii.min()), np.abs(radii - radii.max())).max()
        if off > ROUND_TOLERANCE_M:
            raise SystemExit(f"axis: {name} is {off * 1000:.2f} mm off round about {center} -- not a round shell")
        centers.append(center)
    if np.abs(centers[0] - centers[1]).max() > ROUND_TOLERANCE_M:
        raise SystemExit(f"axis: the shells' centers {centers} disagree -- not one axis")
    return np.mean(centers, axis=0)


def place_rim(faces: dict[str, Part], bodies: dict[str, Part], tire: Part) -> Part:
    """The rim's triangles and normals in the wheel link's frame: its axis on the y axis, its halves centered on the
    tire's bead seat, turned by pi about z so the outer half lies on +y. The axis and the middle are taken from the
    CAD's ``faces``, what is placed are the decimated ``bodies``."""
    center_xz = axis(faces)
    halves = np.concatenate([part[0] for name, part in faces.items() if body_of(name) in ("mesh16", "mesh44")])
    rim_middle_y = (halves[..., 1].min() + halves[..., 1].max()) / 2
    lo, hi = bead(tire)[1]
    shift = np.array([center_xz[0], rim_middle_y - (lo + hi) / 2, center_xz[1]])
    print(
        f"rim: axis at x {center_xz[0] * 1000:.2f}, z {center_xz[1] * 1000:.2f} mm, middle at y "
        f"{rim_middle_y * 1000:.2f} mm (glTF frame), moved onto the tire's bead seat at y {(lo + hi) / 2 * 1000:.2f} mm"
    )
    turn = np.array([-1.0, -1.0, 1.0])  # Rz(pi)
    triangles = np.concatenate([(part[0] - shift) * turn for part in bodies.values()])
    normals = np.concatenate([part[1] * turn for part in bodies.values()])
    if ((bodies[OUTER_HALF][0] - shift) * turn)[..., 1].mean() <= 0:
        raise SystemExit("rim: the outer half does not lie on +y after the turn")
    return triangles, normals


def bead(tire: Part) -> tuple[float, tuple[float, float]]:
    """The tire's bead seat: its radius and its span along y [m]."""
    corners = tire[0].reshape(-1, 3)
    r = radius(corners)
    seat = corners[r < r.min() + BEAD_TOLERANCE_M]
    return float(r.min()), (float(seat[:, 1].min()), float(seat[:, 1].max()))


def shrink_tire(tire: Part, hold_r: float) -> Part:
    """The tire with its tread at :data:`TREAD_RADIUS_M`: every corner beyond ``hold_r`` moved toward the axis by a
    linear map of its radius, ``hold_r`` and below left where they are, y and the angle about the axis unchanged.

    ``hold_r`` is the rim's outer radius, so the bead seat and the part of the bead faces the flanges reach stay as
    Clearpath drew them -- the fit :func:`check_fit` measures is the source's. The normals follow the map's inverse
    transpose: in the cylinder frame (radial, tangential, axial) the map stretches by ``(scale, r'/r, 1)``.
    """
    triangles, normals = tire
    r = radius(triangles)
    outer = r.max()
    scale = (TREAD_RADIUS_M - hold_r) / (outer - hold_r)
    if not 0 < scale <= 1:
        raise SystemExit(f"tire: tread r {outer} m cannot shrink to {TREAD_RADIUS_M} m above the rim's r {hold_r} m")
    moved = r > hold_r
    r_new = np.where(moved, hold_r + (r - hold_r) * scale, r)
    radial = np.stack([triangles[..., 0], np.zeros_like(r), triangles[..., 2]], axis=-1) / r[..., None]
    tangential = np.cross([0.0, 1.0, 0.0], radial)
    n_radial = (normals * radial).sum(-1)
    n_tangential = (normals * tangential).sum(-1)
    # The inverse transpose of diag(scale, r'/r, 1), only where the corner moved.
    n_radial = np.where(moved, n_radial / scale, n_radial)
    n_tangential = np.where(moved, n_tangential * r / r_new, n_tangential)
    new_normals = n_radial[..., None] * radial + n_tangential[..., None] * tangential
    new_normals[..., 1] = normals[..., 1]
    new_normals /= np.linalg.norm(new_normals, axis=-1, keepdims=True)
    new_triangles = triangles.copy()
    new_triangles[..., 0] *= r_new / r
    new_triangles[..., 2] *= r_new / r
    print(
        f"tire: tread r {outer * 1000:.1f} -> {TREAD_RADIUS_M * 1000:.1f} mm, held up to the rim's r "
        f"{hold_r * 1000:.1f} mm (radii beyond it x{scale:.3f}), bead r {r.min() * 1000:.1f} mm kept, width kept"
    )
    return new_triangles, new_normals


def check_fit(rim: Part, tire: Part) -> None:
    """Print the rim's fit in the tire, and stop the script where it does not fit.

    Radially, the rim's bead seat must stay below the tire's bead seat; it is taken :data:`FILLET_M` inside the
    tire's bead faces, where the flanges' fillets begin to rise. Axially, the flanges' inner walls may reach into the
    tire's bead faces by :data:`MAX_FLANGE_OVERLAP_M`: that is the bead clamped between the flanges, and both faces
    stay enclosed -- the wall inside the rubber, the bead face inside the flange's sheet.
    """
    bead_r, (lo, hi) = bead(tire)
    middle = (lo + hi) / 2
    corners = rim[0].reshape(-1, 3)
    r = radius(corners)
    seat = r[np.abs(corners[:, 1] - middle) < (hi - lo) / 2 - FILLET_M]
    radial = bead_r - seat.max()
    # A flange's inner wall: the rim's corners in the radii of the tire's bead face, nearest the middle on each side.
    face = (r > bead_r + BEAD_TOLERANCE_M) & (r < bead_r + 0.01)
    walls = (
        corners[face & (corners[:, 1] < middle), 1].max(),
        corners[face & (corners[:, 1] > middle), 1].min(),
    )
    overlap = max(walls[0] - lo, hi - walls[1])
    print(
        f"fit: tire bead r {bead_r * 1000:.1f} mm, {(hi - lo) * 1000:.1f} mm wide; rim seat up to r "
        f"{seat.max() * 1000:.1f} mm -> radial gap {radial * 1000:.2f} mm; flanges {(walls[1] - walls[0]) * 1000:.1f} "
        f"mm apart -> {overlap * 1000:.2f} mm into the bead faces; rim outer r {r.max() * 1000:.1f} mm"
    )
    if radial < 0:
        raise SystemExit("fit: the rim's seat reaches into the tire's bead")
    if overlap > MAX_FLANGE_OVERLAP_M:
        raise SystemExit(f"fit: the flanges reach more than {MAX_FLANGE_OVERLAP_M * 1000} mm into the tire's bead")


def write(parts: dict[str, tuple[Part, dict]], dae_path: str) -> None:
    """One geometry and one material per part, meters, ``Z_UP`` -- the shape ``clair.twin.collada`` reads the colors
    from. Positions and normals are indexed apart, as in the sources."""
    dae = collada.Collada()
    dae.assetInfo.upaxis = asset.UP_AXIS.Z_UP
    dae.assetInfo.created = dae.assetInfo.modified = STAMP
    dae.assetInfo.contributors.append(
        asset.Contributor(
            author="Clearpath Robotics (tire); a200_wheel_mesh.py (rim placed in it)",
            authoring_tool="onboard/extras/tools/a200_wheel_mesh.py",
            copyright=NOTICE,
            source_data="package://clearpath_platform_description/meshes/a200/wheels/indoor.dae",
        )
    )
    dae.assetInfo.contributors.append(
        asset.Contributor(
            author="Clearpath Robotics (rim CAD)",
            authoring_tool="onboard/extras/tools/a200_wheel_mesh.py",
            copyright=RIM_NOTE,
            source_data=CAD,
        )
    )
    nodes = []
    for key, ((triangles, normals), look) in parts.items():
        points, point_index = np.unique(triangles.reshape(-1, 3).astype(np.float32), axis=0, return_inverse=True)
        directions, direction_index = np.unique(normals.reshape(-1, 3).astype(np.float32), axis=0, return_inverse=True)
        if look["shading"] == "lambert":
            effect = material.Effect(f"{key}-effect", [], "lambert", diffuse=(*look["diffuse"], 1.0))
        else:
            effect = material.Effect(
                f"{key}-effect",
                [],
                "phong",
                diffuse=(*look["diffuse"], 1.0),
                specular=(*look["specular"], 1.0),
                shininess=look["shininess"],
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
        bounds_mm = np.round(np.array([triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))], float) * 1000, 1)
        diffuse = np.round(look["diffuse"], 3).tolist()
        print(f"{key:5s} {len(triangles):6d} triangles, diffuse {diffuse}, bounds {bounds_mm.tolist()} mm")
    dae.scenes.append(scene.Scene("a200_indoor_wheel", nodes))
    dae.scene = dae.scenes[0]
    dae.write(dae_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("source", help="Clearpath's meshes/a200/wheels/indoor.dae")
    parser.add_argument("rims", help="the owner's glTF of the front-right rim (onboard/a200_rims.gltf)")
    parser.add_argument("dae", help="where to write a200_indoor_wheel.dae")
    args = parser.parse_args()
    tire, tire_look = read_tire(args.source)
    faces = read_rim(args.rims)
    rim = place_rim(faces, decimate(faces), tire)
    tire = shrink_tire(tire, float(radius(rim[0]).max()))
    check_fit(rim, tire)
    rim_look = dict(zip(("diffuse", "specular", "shininess"), RIM_LOOK), shading="phong")
    write({"tire": (tire, tire_look), "rim": (rim, rim_look)}, args.dae)


if __name__ == "__main__":
    raise SystemExit(main())
