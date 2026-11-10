"""The a200-0553's lidar: its models, their drivers, the sensor's names, its self filter.

The sensor path and nothing that consumes it.  Navigation derives its 2D scan from these points, perception reads them
as they are; both import from here, and neither from the other.  The modules hold the decisions ROS-free, so that they
can be checked on the Mac.  The self filter is the package's C++ node, ``launch/self_filter.launch.py`` starts it.
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


#: The launch that starts a model's driver and the self filter.
LAUNCH_PATH = share() / "launch" / "lidar.launch.py"
#: The launch that starts the self filter alone, the one place its names are mapped onto :mod:`husky_lidar.wiring`.
SELF_FILTER_LAUNCH_PATH = share() / "launch" / "self_filter.launch.py"

#: How long a crashed driver, filter or scan waits before it comes back [s].  On the robot the launches run under
#: Clearpath's clearpath-platform-extras service, whose unit restarts nothing, so the nodes respawn themselves; 5 s
#: is the delay the custom systemd unit gave the same chain.
RESPAWN_DELAY_S = 5.0
