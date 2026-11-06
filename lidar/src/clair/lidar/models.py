"""The lidar models this package can bring up, and what a simulator casts for each.

ROS-free.  A model names its driver and the figures a sim plant needs to cast it (the rings, the rate, the blind zone);
``launch/lidar.launch.py`` picks the driver by ``model:=<name>``, and the sim feeds take the sweep from here.  The
robot names its model in ``robot.yaml`` (``platform.extras.launch``, ``lidar_model``), which a test holds to
:data:`DEFAULT_MODEL`.

clearpath_config brings Velodyne, Ouster and Seyond up under ``sensors.lidar3d`` itself; a lidar of those makes goes
there, not here.  This registry carries what clearpath_config has no model for (RoboSense, 2.9.x).
"""

from __future__ import annotations

from dataclasses import dataclass

#: The drivers launch/lidar.launch.py can start.  ``rslidar``: RoboSense's ``rslidar_sdk`` (apt
#: ``ros-jazzy-rslidar-sdk``), one binary for the whole RoboSense line, told apart by ``lidar_type``.
DRIVERS = ("rslidar",)


@dataclass(frozen=True)
class LidarModel:
    """One lidar model: its driver and the sweep a simulator casts in its stead."""

    #: The name ``model:=`` and robot.yaml's ``lidar_model`` take.
    name: str
    #: One of :data:`DRIVERS`.
    driver: str
    #: What the driver calls the device (``lidar_type`` for rslidar_sdk).
    device_type: str
    #: The rings' elevations [deg], bottom to top.
    rings_deg: tuple[float, ...]
    rate_hz: float
    #: The blind zone: nearer than this the device returns nothing.
    range_min_m: float


#: The RS-LiDAR-16: 16 rings from -15° to +15°, 2° apart, 10 revolutions a second, a 0.2 m blind zone (datasheet).
#: The rings are this unit's own: the vertical calibration its DIFOP packets carry (``pitch_cali``, read 2026-10-09 out
#: of data/recordings/rs16_labor.pcap) lies within 0.003° of the nominal angles.  Elevations fitted to the points
#: disagree by up to 0.27° near the sensor, and that is not the beams: rs_driver casts each return from the lens
#: center, 38 mm ahead of the axis and 11 mm beside it.
RS16 = LidarModel(
    name="rs16",
    driver="rslidar",
    device_type="RS16",
    rings_deg=tuple(float(e) for e in range(-15, 16, 2)),
    rate_hz=10.0,
    range_min_m=0.2,
)

MODELS = {model.name: model for model in (RS16,)}

#: The a200-0553's: what the launch starts without ``model:=`` and what the container's sim plants cast.
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
