"""The a200-0553's lidar: its models, their drivers, the sensor's names, its self filter.

The sensor path and nothing that consumes it.  Navigation derives its 2D scan from these points, perception reads them
as they are; both import from here, and neither from the other.  The modules hold the decisions ROS-free, so that they
can be checked on the Mac; ``self_filter`` imports rclpy inside its ``main`` only.
"""

from pathlib import Path

__version__ = "0.1.0"

#: The launch that starts a model's driver and the self filter, beside the source tree wherever that lies
#: (``/opt/clair/lidar`` in the container, ``~/onboard/lidar`` on the robot).
LAUNCH_PATH = Path(__file__).resolve().parents[3] / "launch" / "lidar.launch.py"

#: How long a crashed driver, filter or scan waits before it comes back [s].  On the robot the launches run under
#: Clearpath's clearpath-platform-extras service, whose unit restarts nothing, so the nodes respawn themselves; 5 s
#: is the delay the custom systemd unit gave the same chain.
RESPAWN_DELAY_S = 5.0
