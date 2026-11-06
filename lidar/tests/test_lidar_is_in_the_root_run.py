"""This suite has to come along in the ROOT run, not only in the package run.

A test that ran only in the package run would fall silent exactly when someone needs it.  Needs neither ROS nor Docker
-- it reads text.
"""

import tomllib
from pathlib import Path

PYPROJECT = Path(__file__).resolve().parents[3] / "pyproject.toml"


def test_this_package_is_a_workspace_member():
    members = tomllib.loads(PYPROJECT.read_text(encoding="utf-8"))["tool"]["uv"]["workspace"]["members"]
    assert "onboard/lidar" in members, "Without a member entry `uv sync` does not resolve clair-lidar."
