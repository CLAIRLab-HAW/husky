"""The lidar models this package can bring up, and what a simulator casts for each.

ROS-free.  A model names its driver and the figures a sim plant needs to cast it (the scan lines, the azimuths, the
rate, the blind zone); ``launch/lidar.launch.py`` picks the driver by ``model:=<name>``.  The robot carries ONE lidar,
in the slot ``lidar3d_0``, and names its model in ``robot.yaml`` (``platform.extras.launch``, ``lidar_model``):
:func:`robot_model` reads it there, for the container's sim plants as for anything else that has to agree with the
robot.

clearpath_config brings Velodyne, Ouster and Seyond up under ``sensors.lidar3d`` itself; a lidar of those makes goes
there, not here.  This registry carries what clearpath_config has no model for (RoboSense, Blickfeld; 2.9.x).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

#: The drivers launch/lidar.launch.py can start.  ``rslidar``: RoboSense's ``rslidar_sdk`` (apt
#: ``ros-jazzy-rslidar-sdk``), one binary for the whole RoboSense line, told apart by ``lidar_type``.
#: ``blickfeld_qb2``: Blickfeld's ``blickfeld_qb2_ros2_driver`` (v2.1.0, no apt package: the installer builds it).
DRIVERS = ("rslidar", "blickfeld_qb2")


@dataclass(frozen=True)
class LidarModel:
    """One lidar model: its driver and the sweep a simulator casts in its stead."""

    #: The name ``model:=`` and robot.yaml's ``lidar_model`` take.
    name: str
    #: One of :data:`DRIVERS`.
    driver: str
    #: What the driver calls the device (``lidar_type`` for rslidar_sdk).
    device_type: str
    #: The scan lines' elevations [deg] in the lidar's frame, bottom to top: a spinning lidar's rings.
    rings_deg: tuple[float, ...]
    #: The azimuths the lines cover [deg] in the lidar's frame, from ``[0]`` counterclockwise to ``[1]``.
    azimuth_deg: tuple[float, float]
    rate_hz: float
    #: The blind zone: nearer than this the device returns nothing.
    range_min_m: float


#: The RS-LiDAR-16: 16 rings from -15° to +15°, 2° apart, the full turn, 10 revolutions a second, a 0.2 m blind zone
#: (datasheet).  The rings are this unit's own: the vertical calibration its DIFOP packets carry (``pitch_cali``, read
#: 2026-10-09 out of data/recordings/rs16_labor.pcap) lies within 0.003° of the nominal angles.  Elevations fitted to
#: the points disagree by up to 0.27° near the sensor, and that is not the beams: rs_driver casts each return from the
#: lens center, 38 mm ahead of the axis and 11 mm beside it.
RS16 = LidarModel(
    name="rs16",
    driver="rslidar",
    device_type="RS16",
    rings_deg=tuple(float(e) for e in range(-15, 16, 2)),
    azimuth_deg=(0.0, 360.0),
    rate_hz=10.0,
    range_min_m=0.2,
)

#: The Blickfeld Qb2, a MEMS scanner: 90° × 50° (datasheet 51204-ETR-v2.2), boresight along its frame's +y -- the
#: driver hands on the device's own coordinates, x right, y ahead, z up, so the azimuths are 45°..135°.  The 80 lines
#: are the driver's default pattern (``scanlines_up`` 40 + ``scanlines_down`` 40, ``include/qb2_ros2_type.h``, v2.1.0),
#: cast evenly over the 50°: the device's pattern is eye-shaped, narrower at the left and right edges, which this
#: leaves out.  ASSUMED, not read off the lab's unit (R88): its frame rate (1-50 Hz by pattern; 10 Hz here) and the
#: 1 m blind zone, the datasheet's typical application range.
QB2 = LidarModel(
    name="qb2",
    driver="blickfeld_qb2",
    device_type="Qb2",
    rings_deg=tuple(-25.0 + 50.0 * k / 79 for k in range(80)),
    azimuth_deg=(45.0, 135.0),
    rate_hz=10.0,
    range_min_m=1.0,
)

MODELS = {model.name: model for model in (RS16, QB2)}

#: What the launch starts without ``model:=``: the RS16, whose recording (data/recordings/rs16_labor.pcap) the
#: container replays.
DEFAULT_MODEL = RS16.name

#: What a simulator makes of any model: 0.4° a shot rather than the RS16's 0.2° (14 400 raycasts, 21 ms a sweep in
#: ManiSkill, 2026-09-27), and 30 m rather than 150 -- no room in a simulated house is larger.
SIM_AZIMUTH_STEP_DEG = 0.4
SIM_RANGE_MAX_M = 30.0


def model(name: str) -> LidarModel:
    """The model called ``name``; a :class:`ValueError` naming the known ones otherwise."""
    if name not in MODELS:
        raise ValueError(f"no lidar model {name!r}; known: {', '.join(sorted(MODELS))}")
    return MODELS[name]


def robot_model(robot_yaml: str | Path) -> LidarModel:
    """The one model ``robot_yaml`` names: the ``lidar_model`` of its ``platform.extras.launch`` entries.

    :raises ValueError: when it names none, more than one, or an unknown one -- the robot has one lidar slot.
    """
    launches = yaml.safe_load(Path(robot_yaml).read_text(encoding="utf-8"))["platform"]["extras"].get("launch") or []
    names = [launch["args"]["lidar_model"] for launch in launches if "lidar_model" in (launch.get("args") or {})]
    if len(names) != 1:
        raise ValueError(f"{robot_yaml} names {len(names)} lidar models {names}; the robot carries exactly one")
    return model(str(names[0]))
