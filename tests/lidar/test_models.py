"""The model registry, and the robot naming one of its models.

robot.yaml names the a200-0553's ONE lidar (``platform.extras.launch``, ``lidar_model``); the container's sim plants
read it there through :func:`~husky_lidar.models.robot_model`.  Needs neither ROS nor Docker.
"""

from pathlib import Path

import pytest
import yaml

from husky_lidar import models

ROBOT_YAML = Path(__file__).resolve().parents[2] / "setup" / "config" / "robot.yaml"
LAUNCH = Path(__file__).resolve().parents[2] / "src" / "husky_lidar" / "launch" / "lidar.launch.py"


def _robot_yaml_with(tmp_path: Path, *launch_args: dict) -> Path:
    robot_yaml = tmp_path / "robot.yaml"
    launches = [{"package": "p", "path": "launch/x.launch.py", "args": args} for args in launch_args]
    robot_yaml.write_text(yaml.safe_dump({"platform": {"extras": {"launch": launches}}}), encoding="utf-8")
    return robot_yaml


def test_every_model_is_listed_under_its_name():
    assert all(name == model.name for name, model in models.MODELS.items())


def test_every_model_has_a_driver_the_launch_starts():
    """The launch holds the other half of each driver; launch_ros is not on the Mac, so its text is read."""
    assert {model.driver for model in models.MODELS.values()} <= set(models.DRIVERS)
    launch = LAUNCH.read_text(encoding="utf-8")
    assert all(f'"{driver}":' in launch for driver in models.DRIVERS)


def test_the_lines_go_bottom_to_top_and_the_azimuths_counterclockwise():
    for model in models.MODELS.values():
        assert list(model.rings_deg) == sorted(model.rings_deg), model.name
        start, end = model.azimuth_deg
        assert 0.0 <= start < end <= start + 360.0, model.name


def test_the_rs16_sweeps_the_full_turn_and_the_qb2_its_90_degrees_ahead():
    assert models.RS16.azimuth_deg == (0.0, 360.0)
    assert models.QB2.azimuth_deg == (45.0, 135.0), "the Qb2's boresight is its frame's +y"
    assert len(models.QB2.rings_deg) == 80
    assert (models.QB2.rings_deg[0], models.QB2.rings_deg[-1]) == pytest.approx((-25.0, 25.0))


def test_an_unknown_model_is_refused_by_name():
    with pytest.raises(ValueError, match="known: qb2, rs16"):
        models.model("vlp16")


def test_robot_yaml_names_exactly_one_known_lidar():
    assert models.robot_model(ROBOT_YAML).name in models.MODELS


def test_robot_model_reads_the_one_lidar_model(tmp_path):
    robot_yaml = _robot_yaml_with(tmp_path, {"lidar_model": "qb2"}, {"other": "x"})
    assert models.robot_model(robot_yaml) is models.QB2


@pytest.mark.parametrize("args", [(), ({"lidar_model": "rs16"}, {"lidar_model": "qb2"})], ids=["none", "two"])
def test_robot_model_refuses_anything_but_one_lidar(tmp_path, args):
    """One slot, one lidar: the Qb2 is the RS16's alternative, not a second sensor."""
    with pytest.raises(ValueError, match="exactly one"):
        models.robot_model(_robot_yaml_with(tmp_path, *args))
