#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["networkx", "numpy", "pycollada", "scipy", "trimesh"]
# ///
"""Writes ``a200_top_chassis.dae``: Clearpath's a200 top chassis with the real chassis yellow and its controls colored.

Clearpath's ``clearpath_platform_description/meshes/a200/top_chassis.dae`` (2.9.15) is ONE geometry with ONE
material, ``Yellow``, diffuse ``0.8 0.8 0 1`` -- a greenish yellow, and the emergency stop, the power button, the key
lock, the panels they sit on and the status LEDs at the rear in the same yellow. The real a200-0553's chassis is
#E29000, its emergency stop red, its power button and key lock metallic silver, the panels dark gray, the status LEDs
a lighter gray (the owner, 2026-10-09). One material cannot carry that, so this script splits the mesh into its
connected components (12, measured 2026-10-09) and writes one geometry and one material per part (:data:`LOOK`).

The controls are identified by SHAPE, from a render of the components on 2026-10-09, and matched here by their
bounding box in the mesh frame [mm] (:data:`CONTROLS`), so the order trimesh splits them in does not matter: the
emergency stop is the big mushroom on a collar, the key lock the domed knob with a ring below it (both on +y, the
robot's left), the power button the cylinder with a flat ringed face (on -y), and the two flat panels they sit on,
one per side. The owner named them. The two bumps between the panels (:data:`REMOVED`) the a200-0553 does not have:
they are left out, and as each stands in a closed tube through the rear face, a flat lid in the chassis' color covers
where it stood (:func:`hole_caps`). Everything else is the chassis, the small pin beside the power button included.
A box that matches no component, or more than one, stops the script: the source mesh changed, and nothing is guessed.

The status LEDs are no component of their own: they are small boxes in the chassis body's sloped rear face (normal
~(-0.64, 0, 0.77)), on -y under the power button -- four stacked along the slope and a longer field right under the
button, all 11 mm wide at y -145.5..-134.5 mm, found as the chassis triangles whose three corners lie in
:data:`STATUS_LEDS`. Measured along the slope's normal on 2026-10-09, those boxes reach from the slope's surface
1.6 mm INTO the body, under its own triangles and the right panel, so no color shows them. Their footprints are drawn
anew instead, as fields :data:`LED_HEIGHT_M` proud of the right panel's face (:func:`led_fields`), and the sunken
boxes are not written.

The normals are the source's own, per corner: the chassis' edges stay as sharp as Clearpath shaded them.

The source is BSD-3-Clause (``LICENSE`` of ``clearpath_common``, "Copyright (c) 2023, clearpathrobotics"), and the
license asks a redistribution to keep that notice; the DAE's ``<asset><contributor>`` carries it in full and names the
source file (:data:`NOTICE`).

    docker cp offboard-plant-mock-1:/opt/ros/jazzy/share/clearpath_platform_description/meshes/a200/top_chassis.dae \\
        /tmp/top_chassis.dae
    uv run onboard/extras/tools/a200_chassis_mesh.py /tmp/top_chassis.dae \\
        onboard/extras/src/husky_extras_description/meshes/a200_top_chassis.dae

The script colors by geometry, not by the source's material, so any copy of the source does: the apt file, its
``.bak`` or a copy a patcher has already recolored.
"""

from __future__ import annotations

import argparse
import datetime

import collada
import numpy as np
import trimesh
from collada import asset, geometry, material, scene, source
from scipy.spatial import ConvexHull

#: The controls' and their panels' bounding boxes in the mesh frame [mm] as (min corner, max corner), read off the
#: split on 2026-10-09 (faces: 2768, 2430, 3808, 20, 20), and the tolerance a box is matched with [mm]. Every other
#: component is ``chassis``.
CONTROLS = {
    "emergency_stop": ((-326, 133, 191), (-299, 162, 218)),
    "key_lock": ((-346, 137, 153), (-318, 158, 181)),
    "power_button": ((-316, -158, 175), (-286, -137, 206)),
    # The two flat panels the controls sit on, on the sloped rear face: under the emergency stop and the key lock on
    # +y, under the power button on -y (20 faces each; the owner, 2026-10-09).
    "panel_left": ((-398, 125, 121), (-283, 176, 217)),
    "panel_right": ((-398, -176, 121), (-283, -125, 217)),
}
#: The two bumps between the panels, at y +-66..98 mm (1870 faces each): the a200-0553 has none (the owner,
#: 2026-10-09), so they are left out.
REMOVED = {
    "bump_left": ((-332, 66, 156), (-284, 98, 209)),
    "bump_right": ((-332, -98, 156), (-284, -66, 209)),
}
#: The middle panel of the rear face, between the two bumps (300 faces): a separate plate whose sides stop at
#: y +-96.8 mm in the rear body's opening, whose walls stand at y +-97.35 mm, both flush with the slope at
#: n.p 347.64 mm (measured 2026-10-10). The 0.55 mm slits show as seams, and one looks into the body through them;
#: on the a200-0553 the rear face is one piece (the owner, 2026-10-10). Its sides are moved out to the walls.
MIDDLE_PANEL = ((-406.0, -96.8, 71.0), (-272.3, 96.8, 224.1))
MIDDLE_PANEL_SIDE_MM = 97.35
BOX_TOLERANCE_MM = 1.5
#: The box the source's status LED boxes lie in, in the mesh frame [mm] as (min corner, max corner) (2026-10-09). They
#: sit 1.6 mm INTO the sloped rear face, under its own triangles and the right panel, so no color shows them; their
#: footprints are taken from them and drawn anew as :data:`LED_HEIGHT_M` high fields on the right panel's face.
STATUS_LEDS = ((-380.0, -145.6, 134.0), (-329.0, -134.4, 178.0))
LED_HEIGHT_M = 0.0005
#: Metallic silver, the power button's and the key lock's look.
SILVER = ((0.75, 0.76, 0.78), (0.6, 0.6, 0.6), 40.0)
#: Matte dark gray, the panels' look: told apart from the silver controls on them by half the lightness and a dull
#: finish.
PANEL_GRAY = ((0.33, 0.34, 0.36), (0.25, 0.25, 0.25), 20.0)
#: Each part's look [sRGB diffuse, sRGB specular, shininess]. The chassis #E29000 (226/255, 144/255, 0; the owner,
#: 2026-10-09) with the source's specular strength and shininess; the emergency stop glossy red plastic; the power
#: button and the key lock metallic silver; the status LEDs black behind a glossy lens. The twin
#: (``clair.twin.collada``) takes the diffuse alone; Foxglove and RViz shade with all three.
LOOK = {
    "chassis": ((0.886, 0.565, 0.0), (0.4, 0.4, 0.4), 50.0),
    "emergency_stop": ((0.80, 0.05, 0.05), (0.5, 0.5, 0.5), 60.0),
    "power_button": SILVER,
    "key_lock": SILVER,
    "panel_left": PANEL_GRAY,
    "panel_right": PANEL_GRAY,
    # A lighter gray than the panel they sit on (the owner, 2026-10-09), still well below the silver.
    "status_leds": ((0.50, 0.51, 0.53), (0.4, 0.4, 0.4), 40.0),
}
#: Written into every run's ``<created>``/``<modified>``, so a second run on the same source writes the same file.
STAMP = datetime.datetime(2026, 10, 9)
#: The source's license, kept with the derived file as its clause 2 asks.
NOTICE = """Derived from clearpath_platform_description/meshes/a200/top_chassis.dae (clearpath_common 2.9.15):
split into components and recolored by onboard/extras/tools/a200_chassis_mesh.py.

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


def read(dae_path: str) -> tuple[np.ndarray, np.ndarray]:
    """Every triangle of the source in its scene's frame, and its corner normals: two ``(n, 3, 3)`` arrays [m]."""
    dae = collada.Collada(dae_path)
    triangles, normals = [], []
    for geom in dae.scene.objects("geometry"):
        for prim in geom.primitives():
            triangles.append(prim.vertex[prim.vertex_index])
            normals.append(prim.normal[prim.normal_index])
    return np.concatenate(triangles), np.concatenate(normals)


def close_middle_panel(triangles: np.ndarray, labels: np.ndarray) -> np.ndarray:
    """``triangles`` with the middle panel's sides (:data:`MIDDLE_PANEL`) moved out to the opening's walls."""
    hits = [
        label
        for label in np.unique(labels)
        if np.allclose(
            [triangles[labels == label].min(axis=(0, 1)) * 1000, triangles[labels == label].max(axis=(0, 1)) * 1000],
            MIDDLE_PANEL,
            atol=BOX_TOLERANCE_MM,
        )
    ]
    if len(hits) != 1:
        raise SystemExit(f"middle panel: {len(hits)} components in the box {MIDDLE_PANEL} mm -- source changed?")
    closed = triangles.copy()
    panel = closed[labels == hits[0]]
    side = np.abs(panel[..., 1]) * 1000.0 > MIDDLE_PANEL[1][1] - 0.1
    panel[..., 1] = np.where(side, np.sign(panel[..., 1]) * MIDDLE_PANEL_SIDE_MM / 1000.0, panel[..., 1])
    closed[labels == hits[0]] = panel
    return closed


def components(triangles: np.ndarray) -> np.ndarray:
    """Each triangle's connected component, in the source's triangle order."""
    mesh = trimesh.Trimesh(**trimesh.triangles.to_kwargs(triangles), process=False)
    mesh.merge_vertices()
    return trimesh.graph.connected_component_labels(mesh.face_adjacency, node_count=len(mesh.faces))


def assign(triangles: np.ndarray, labels: np.ndarray) -> dict[str, np.ndarray]:
    """Each part's triangle mask: the components :data:`CONTROLS` names by their box, the status LEDs the chassis
    triangles wholly inside :data:`STATUS_LEDS`, the chassis the rest."""
    masks = {key: np.zeros(len(triangles), dtype=bool) for key in LOOK}
    claimed = np.zeros(len(triangles), dtype=bool)
    bounds = {
        label: np.array([triangles[labels == label].min(axis=(0, 1)), triangles[labels == label].max(axis=(0, 1))])
        for label in np.unique(labels)
    }
    for control, box in {**CONTROLS, **REMOVED}.items():
        hits = [label for label, b in bounds.items() if np.allclose(b * 1000.0, box, atol=BOX_TOLERANCE_MM)]
        if len(hits) != 1:
            raise SystemExit(f"{control}: {len(hits)} components in the box {box} mm, expected 1 -- source changed?")
        part = labels == hits[0]
        if control in masks:
            masks[control] = part
        claimed |= part
    low, high = (np.array(corner) / 1000.0 for corner in STATUS_LEDS)
    leds = ~claimed & np.all((triangles >= low) & (triangles <= high), axis=(1, 2))
    if not leds.any():
        raise SystemExit(f"status_leds: no chassis triangle in the box {STATUS_LEDS} mm -- source changed?")
    masks["status_leds"] = leds  # the sunken source boxes: replaced by :func:`led_fields`, never written
    masks["chassis"] = ~claimed & ~leds
    return masks


def hole_caps(triangles: np.ndarray, labels: np.ndarray, chassis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Flat lids where the left-out bumps (:data:`REMOVED`) were: each bump stands in a closed tube through the rear
    face, and without it one looks through. The lid is the bump's outline seen along the slope's normal, laid
    0.1 mm proud of the face around it, in the chassis' color."""
    normal = np.array([-0.64, 0.0, 0.77])
    normal /= np.linalg.norm(normal)
    along = np.cross([0.0, 1.0, 0.0], normal)
    faces = triangles[chassis]
    face_normals = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
    face_normals /= np.linalg.norm(face_normals, axis=1, keepdims=True) + 1e-12
    on_slope = faces[face_normals @ normal > 0.99].reshape(-1, 3)
    lids, lid_normals = [], []
    for low, high in REMOVED.values():
        bump = [
            label
            for label in np.unique(labels)
            if np.allclose(
                [
                    triangles[labels == label].min(axis=(0, 1)) * 1000,
                    triangles[labels == label].max(axis=(0, 1)) * 1000,
                ],
                [low, high],
                atol=BOX_TOLERANCE_MM,
            )
        ]
        corners = triangles[labels == bump[0]].reshape(-1, 3)
        near = on_slope[np.abs(on_slope[:, 1] - corners[:, 1].mean()) < 0.03]
        offset = float((near @ normal).max()) + 0.0001
        flat = np.c_[corners @ along, corners[:, 1]]
        ring = flat[ConvexHull(flat).vertices]  # the outline's corners alone, counterclockwise
        points = np.outer(ring[:, 0], along) + np.outer(ring[:, 1], [0.0, 1.0, 0.0]) + normal * offset
        middle = points.mean(axis=0)
        fan = np.array([[middle, a, b] for a, b in zip(points, np.roll(points, -1, axis=0))])
        if (np.cross(fan[:, 1] - fan[:, 0], fan[:, 2] - fan[:, 0]) @ normal).sum() < 0:
            fan = fan[:, ::-1]
        lids.append(fan)
        lid_normals.append(np.broadcast_to(normal, fan.shape))
    return np.concatenate(lids), np.concatenate(lid_normals)


def led_fields(triangles: np.ndarray, sunken: np.ndarray, panel: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The status LEDs as fields on the right panel's face: each sunken source box's footprint in the face's plane,
    :data:`LED_HEIGHT_M` proud of it. Triangles and their corner normals, as :func:`read` gives them."""
    faces = triangles[panel]
    normals = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    # The panel's outer face: its triangles facing up the slope, back and up (the source's slope normal is
    # (-0.64, 0, 0.77)); the plane through them.
    outer = normals @ np.array([-0.64, 0.0, 0.77]) > 0.99
    if not outer.any():
        raise SystemExit("status_leds: the right panel has no face along the rear slope -- source changed?")
    normal = normals[outer].mean(axis=0)
    normal /= np.linalg.norm(normal)
    offset = float((faces[outer].reshape(-1, 3) @ normal).max())
    along = np.cross([0.0, 1.0, 0.0], normal)  # up the slope, in the x-z plane
    # One field per source box. The boxes are joined by 1.5 mm steps between them, so they are told apart by the
    # boxes' own sides alone -- the triangles that span more than 4 mm up the slope -- and those overlapping merged
    # (five boxes, 6 to 21 mm long, measured 2026-10-09).
    sunken_faces = triangles[sunken]
    low_u, high_u = (sunken_faces @ along).min(axis=1), (sunken_faces @ along).max(axis=1)
    sides = np.flatnonzero(high_u - low_u > 0.004)
    groups: list[list[int]] = []
    reach = -np.inf
    for index in sides[np.argsort(low_u[sides])]:
        if low_u[index] > reach + 1e-5:
            groups.append([])
        groups[-1].append(index)
        reach = max(reach, high_u[index])
    fields, corner_normals = [], []
    for group in groups:
        corners = sunken_faces[group].reshape(-1, 3)
        u, v = corners @ along, corners[:, 1]
        size = (u.max() - u.min(), v.max() - v.min(), LED_HEIGHT_M)
        frame = np.eye(4)
        frame[:3, 0], frame[:3, 1], frame[:3, 2] = along, (0.0, 1.0, 0.0), normal
        frame[:3, 3] = along * (u.max() + u.min()) / 2 + np.array([0, 1.0, 0]) * (v.max() + v.min()) / 2
        frame[:3, 3] += normal * (offset + LED_HEIGHT_M / 2 - frame[:3, 3] @ normal)
        field = trimesh.creation.box(extents=size, transform=frame)
        fields.append(field.triangles)
        corner_normals.append(np.repeat(field.face_normals[:, None, :], 3, axis=1))
    return np.concatenate(fields), np.concatenate(corner_normals)


def write(parts: dict[str, tuple[np.ndarray, np.ndarray]], dae_path: str) -> None:
    """One geometry and one material per part, meters, ``Z_UP`` -- the shape ``clair.twin.collada`` reads the colors
    from. Positions and normals are indexed apart, as in the source: a corner shares its position with the faces
    around it and its normal only with the faces of its own flat."""
    dae = collada.Collada()
    dae.assetInfo.upaxis = asset.UP_AXIS.Z_UP
    dae.assetInfo.created = dae.assetInfo.modified = STAMP
    dae.assetInfo.contributors.append(
        asset.Contributor(
            author="Clearpath Robotics (source); a200_chassis_mesh.py (split and recolor)",
            authoring_tool="onboard/extras/tools/a200_chassis_mesh.py",
            copyright=NOTICE,
            source_data="package://clearpath_platform_description/meshes/a200/top_chassis.dae",
        )
    )
    nodes = []
    for key, (triangles, normals) in parts.items():
        color, specular, shininess = LOOK[key]
        points, point_index = np.unique(triangles.reshape(-1, 3).astype(np.float32), axis=0, return_inverse=True)
        directions, direction_index = np.unique(normals.reshape(-1, 3).astype(np.float32), axis=0, return_inverse=True)
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
        bounds_mm = np.round(np.array([triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))], float) * 1000).tolist()
        print(f"{key:15s} {len(triangles):6d} triangles, srgb {list(color)}, bounds {bounds_mm} mm")
    dae.scenes.append(scene.Scene("a200_top_chassis", nodes))
    dae.scene = dae.scenes[0]
    dae.write(dae_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("source", help="Clearpath's top_chassis.dae")
    parser.add_argument("dae", help="where to write a200_top_chassis.dae")
    args = parser.parse_args()
    triangles, normals = read(args.source)
    labels = components(triangles)
    triangles = close_middle_panel(triangles, labels)
    print(f"{len(triangles)} triangles in {len(np.unique(labels))} components")
    masks = assign(triangles, labels)
    parts = {key: (triangles[mask], normals[mask]) for key, mask in masks.items()}
    lids, lid_normals = hole_caps(triangles, labels, masks["chassis"])
    parts["chassis"] = (np.concatenate([parts["chassis"][0], lids]), np.concatenate([parts["chassis"][1], lid_normals]))
    parts["status_leds"] = led_fields(triangles, masks["status_leds"], masks["panel_right"])
    write(parts, args.dae)


if __name__ == "__main__":
    main()
