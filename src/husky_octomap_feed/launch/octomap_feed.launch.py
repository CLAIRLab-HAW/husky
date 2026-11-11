"""The octomap feed, as robot.yaml starts it: the wrist D435's depth as a throttled PointCloud2 for move_group.

robot.yaml's ``platform.extras.launch`` names this file; Clearpath's clearpath-platform-extras service runs it.  The
node keeps its own name, ``/octomap_feed``, outside the robot's namespace -- ``robot:octomap`` sets its ``enabled``
under that name -- and its topics are absolute (the defaults in ``husky_octomap_feed.feed``).

No restart along with the camera: after ``systemctl restart clearpath-sensors`` the feed resubscribed on its own and
was back at 5 Hz within 12 s (2026-11-11, R89).  It respawns because that service's unit carries no Restart=.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

#: How long a crashed feed waits before it comes back [s] -- the 5 s its former systemd unit gave it.
RESPAWN_DELAY_S = 5.0


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="husky_octomap_feed",
                executable="octomap_feed",
                respawn=True,
                respawn_delay=RESPAWN_DELAY_S,
                output="screen",
            )
        ]
    )
