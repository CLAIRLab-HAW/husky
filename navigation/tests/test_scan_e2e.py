"""The scan on the sensor path: onboard/rs16's driver on real packets, and the 2D scan derived from its points.

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


#: The replay runs in a process group of its own, whose leader's pid stands here: one SIGINT to the group stops launch,
#: driver and filters alike.  A SIGTERM to the launch alone does not -- three rslidar_sdk_node outlived the suites that
#: way and published side by side (2026-10-09).  The rs16 and navigation suites share the file, so each stops what
#: the other left before it starts.
REPLAY_PID = "/tmp/rs16_replay.pid"


def _stop_group(pid_file: str) -> None:
    """SIGINT to the group recorded in ``pid_file``, SIGKILL to what is left of it after 10 s."""
    _exec(
        f"[ -f {pid_file} ] || exit 0; g=$(cat {pid_file}); kill -INT -- -$g 2>/dev/null; "
        f"for _ in $(seq 20); do kill -0 -- -$g 2>/dev/null || break; sleep 0.5; done; "
        f"kill -KILL -- -$g 2>/dev/null; rm -f {pid_file}"
    )


def _replay(launch: str):
    """``launch`` on the recording for the module's tests, stopped after them."""
    if subprocess.run(["docker", "inspect", CONTAINER], capture_output=True).returncode != 0:
        pytest.skip(f"Container {CONTAINER} is not running.")
    if not PCAP_HOST.is_file():
        pytest.skip(
            f"No RS16 recording under {PCAP_HOST} -- R-point c in "
            f"ROBOTER-TODO.md (tcpdump on UDP 6699 + 7788 at the robot). "
            f"Without it the mock has no lidar data."
        )
    _stop_group(REPLAY_PID)
    _exec(
        f"source ros-env; setsid nohup ros2 launch {launch} pcap:={PCAP_CONTAINER} > /tmp/lidar.log 2>&1 & "
        f"echo $! > {REPLAY_PID}; sleep 15"
    )
    yield PCAP_CONTAINER
    _stop_group(REPLAY_PID)


@pytest.fixture(scope="module")
def replay():
    yield from _replay("/opt/clair/navigation/launch/scan.launch.py")


def test_the_derived_scan_is_not_all_infinite(replay):
    """A scan of nothing but inf looks like a broken driver and is in truth
    a height band that misses the driving plane."""
    script = r"""
source ros-env
timeout 15 ros2 topic echo /a200_0553/sensors/lidar3d_0/scan --once --field ranges \
  > /tmp/scan_ranges.txt 2>&1
python3 -c "
import re
text = open('/tmp/scan_ranges.txt').read()
vals = [float(v) for v in re.findall(r'-?\d+\.\d+', text)]
finite = [v for v in vals if v == v and v != float('inf')]
print(len(finite), len(vals))
"
"""
    finite, total = (int(x) for x in _exec(script).strip().splitlines()[-1].split())
    assert total > 0, "No LaserScan received."
    assert finite > total * 0.05, (
        f"Only {finite} of {total} rays are finite. Either the height band "
        f"(min_height/max_height in wiring.py) misses the driving plane, or "
        f"the recording shows open field."
    )


#: The AMCL navigation this module starts, in a group of its own like the replay.
AMCL_PID = "/tmp/nav_amcl.pid"


@pytest.fixture
def amcl_nav(replay):
    """Nav2 with AMCL in place of the stack's own, and the stack's own back afterwards, with the arguments it ran with
    -- or none, if none ran."""
    found = _exec("pgrep -af '[n]avigation.launch.py' | head -1").strip()
    args = found.split("navigation.launch.py", 1)[1].strip() if found else None
    _exec("pkill -INT -f '[n]avigation.launch.py'; sleep 5")
    _exec(f"setsid nohup nav localization:=amcl > /tmp/nav-amcl.log 2>&1 & echo $! > {AMCL_PID}; sleep 35", timeout=120)
    yield
    _stop_group(AMCL_PID)
    if args is not None:
        _exec(f"setsid nohup nav {args} > /tmp/nav.log 2>&1 & sleep 15")


def test_amcl_publishes_the_map_to_odom_transform(amcl_nav):
    """Only with scans does AMCL become the source of map ─▶ odom."""
    out = _exec(
        "source ros-env; timeout 10 ros2 run tf2_ros tf2_echo map odom "
        "--ros-args -r /tf:=/a200_0553/tf "
        "-r /tf_static:=/a200_0553/tf_static 2>&1 | head -20"
    )
    assert "Translation" in out, f"AMCL does not publish map ─▶ odom. Log:\n{_exec('tail -40 /tmp/nav-amcl.log')}"
