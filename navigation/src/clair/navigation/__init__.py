"""Navigation layer of the a200-0553.

ROS-FREE.  This package holds the decisions (topic and frame names, the scan's parameters, the synthetic map), not the
nodes -- so that they can be checked on the Mac without ROS.  The launch files in the repo's ``launch/`` import from
here instead of stating the values a second time.  The lidar's sensor path the scan builds on is ``clair.lidar``'s.
"""

from pathlib import Path

__version__ = "0.1.0"

#: The scan on the lidar's sensor path, beside the source tree wherever that lies (``/opt/clair/navigation`` in the
#: container, ``~/husky/navigation`` on the robot).  husky_extras_bringup includes it on the robot.
SCAN_LAUNCH_PATH = Path(__file__).resolve().parents[3] / "launch" / "scan.launch.py"
