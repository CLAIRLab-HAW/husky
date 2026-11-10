"""The lidar launch must not state the wiring a second time.

On the Mac launch_ros is not installed -- a launch file cannot be imported here and its content cannot be checked.
What remains checkable is provenance: if a topic name stands as a string literal IN the launch file, it is stated there
a second time and can drift away from the checked version without any test noticing.
"""

from pathlib import Path

import pytest

LAUNCH_DIR = Path(__file__).resolve().parents[2] / "src" / "husky_lidar" / "launch"

#: Values that may stand in exactly one place: in husky_lidar.wiring.
WIRED_LITERALS = ("/a200_0553/sensors/lidar3d_0/points", "lidar3d_0_laser")


@pytest.mark.parametrize("launch_file", sorted(LAUNCH_DIR.glob("*.launch.py")), ids=lambda p: p.name)
def test_the_launch_file_imports_the_wiring(launch_file):
    assert "from husky_lidar import" in launch_file.read_text(encoding="utf-8"), (
        f"{launch_file.name} does not import husky_lidar -- the wiring then stands there a second time."
    )


@pytest.mark.parametrize("launch_file", sorted(LAUNCH_DIR.glob("*.launch.py")), ids=lambda p: p.name)
def test_the_launch_file_restates_no_wired_literal(launch_file):
    restated = [lit for lit in WIRED_LITERALS if lit in launch_file.read_text(encoding="utf-8")]
    assert not restated, f"{launch_file.name} states {restated} itself; they belong in husky_lidar.wiring."
