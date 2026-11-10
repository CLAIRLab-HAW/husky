# rs16_description

The RoboSense RS-LiDAR-16 as a URDF macro, `rs16(name, parent_link, *origin)` in `urdf/rs16.urdf.xacro`:

| Link | What sits there |
|---|---|
| `${name}_link` | on the mounting face: the mesh, a collision cylinder Ø109 × 80.7 mm, 0.87 kg |
| `${name}_laser` | 39 mm above it, the optical center; +x is the 0° direction, opposite the cable outlet |

The figures are the RS-LiDAR-16 user manual's (§3, §8.1 and Fig. E-1). `meshes/rs16.dae` is the housing, cap,
window and cable gland of RoboSense's CAD model, each in its own color; [tools/rs16_mesh.py](../../tools/rs16_mesh.py)
writes it from `RS16.stp` (robosense.ai, Downloads, "RS-LiDAR-16 3D Model"), which is not in the repo.

This package describes the SENSOR and nothing else, like `rg6_description` the hand. Where it is mounted is the
robot's: on the a200-0553, `robot.yaml` places the frame `lidar3d_0_mount_link` and
[husky_extras_description](../husky_extras_description/README.md) hangs `rs16` on it with `name="lidar3d_0"`.
