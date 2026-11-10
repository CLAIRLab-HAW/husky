"""Navigation layer of the a200-0553.

ROS-FREE.  This package holds the decisions (topic and frame names, the scan's parameters, the synthetic map), not the
nodes -- so that they can be checked on the Mac without ROS.  The launch files in the repo's ``launch/`` import from
here instead of stating the values a second time.  The lidar's sensor path the scan builds on is ``husky_lidar``'s.
"""

from pathlib import Path

__version__ = "0.2.0"


def share() -> Path:
    """The package's share directory: the ament install's where it is installed, else the source tree around this
    module -- the Mac's tests run on the source, and so does a robot that has not built it."""
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    except ImportError:
        return Path(__file__).resolve().parents[1]
    try:
        return Path(get_package_share_directory(__name__))
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1]


#: The scan on the lidar's sensor path.  husky_bringup includes it on the robot.
SCAN_LAUNCH_PATH = share() / "launch" / "scan.launch.py"
