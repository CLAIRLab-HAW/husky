"""The sensor path: real driver, real packets, the points raw and without the robot's body.

Skips itself with a named cause as long as the RS16 recording is missing (R-point c).  A test that went green without
a recording would be worse than none: it would report a chain as driven that has never seen a packet.
"""

import subprocess
from pathlib import Path

import pytest

pytestmark = pytest.mark.e2e_container_nav

CONTAINER = "offboard-plant-mock-1"
#: The mount from docker-compose.yml (../../data/recordings ─▶ /data/recordings).
PCAP_HOST = Path(__file__).resolve().parents[3] / "data" / "recordings" / "rs16_labor.pcap"
PCAP_CONTAINER = "/data/recordings/rs16_labor.pcap"


def _exec(script: str, timeout: int = 120) -> str:
    proc = subprocess.run(
        ["docker", "exec", CONTAINER, "bash", "-lc", script], capture_output=True, text=True, timeout=timeout
    )
    return proc.stdout


@pytest.fixture(scope="module")
def replay():
    if subprocess.run(["docker", "inspect", CONTAINER], capture_output=True).returncode != 0:
        pytest.skip(f"Container {CONTAINER} is not running.")
    if not PCAP_HOST.is_file():
        pytest.skip(
            f"No RS16 recording under {PCAP_HOST} -- R-point c in "
            f"ROBOTER-TODO.md (tcpdump on UDP 6699 + 7788 at the robot). "
            f"Without it the mock has no lidar data."
        )
    _exec("pkill -f rslidar_sdk_node || true; sleep 2")
    _exec(
        f"source ros-env; nohup ros2 launch "
        f"/opt/clair/rs16/launch/lidar.launch.py "
        f"pcap:={PCAP_CONTAINER} > /tmp/lidar.log 2>&1 & sleep 15; echo ok"
    )
    return PCAP_CONTAINER


def test_the_driver_reads_from_the_recording(replay):
    """Verifies the msg_source mapping ON THE RUNNING NODE.

    The values 1/2/3 are in RoboSense's documentation; here stands what the driver actually does.
    """
    log = _exec("cat /tmp/lidar.log")
    assert "Receive Packets From : Pcap" in log, f"The driver does not read from the recording. Log:\n{log[-2000:]}"


def test_the_point_cloud_arrives(replay):
    out = _exec("source ros-env; timeout 15 ros2 topic hz /a200_0553/sensors/lidar3d_0/points 2>&1 | head -5")
    assert "average rate" in out, f"No point cloud. Output:\n{out}"


def test_the_cloud_carries_the_canonical_frame(replay):
    out = _exec(
        "source ros-env; timeout 10 ros2 topic echo /a200_0553/sensors/lidar3d_0/points --once --field header.frame_id"
    )
    assert "lidar3d_0_laser" in out


def test_the_filtered_cloud_arrives(replay):
    out = _exec("source ros-env; timeout 15 ros2 topic hz /a200_0553/sensors/lidar3d_0/points_filtered 2>&1 | head -5")
    assert "average rate" in out, f"No filtered point cloud -- is the self filter running? Output:\n{out}"
