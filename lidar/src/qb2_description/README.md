# qb2_description

The Blickfeld Qb2 as a URDF macro, `qb2(name, parent_link, *origin)` in `urdf/qb2.urdf.xacro`:

| Link | What sits there |
|---|---|
| `${name}_link` | the back face's middle (the M4 threads), x along the boresight: a box 53 × 111 × 75 mm, 0.40 kg |
| `${name}_laser` | about 20 mm ahead of it at half the height, the device's own frame: x right, y ahead, z up |

The box is the ST housing's; the figures are Blickfeld's (datasheet 51204-ETR-v2.2, the Qb2 coordinate-system
drawing on docs.blickfeld.com). The laser frame is turned -90° about z because `blickfeld_qb2_ros2_driver` hands on the
device's coordinates unrotated, and those put the boresight on +y. What is read off a drawing rather than measured —
the origin's 20 mm, a boresight some 5° below the housing's datum, which variant (ST or the 83 mm ETR) the lab has — is
R88 in [ROBOTER-TODO.md](../../../../ROBOTER-TODO.md).

The visual is a box: Blickfeld's CAD (`Qb_CAD_model.zip` on the Qb2 product page) is a STEP without a stated license,
and no mesh is derived from it yet.

This package describes the SENSOR and nothing else, like `rs16_description`. It is the RS16's alternative in the same
slot: `robot.yaml`'s `lidar_model` picks one, and [husky_extras_description](../../../extras/README.md) hangs that
macro on `lidar3d_0_mount_link` with `name="lidar3d_0"`.
