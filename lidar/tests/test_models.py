"""The model registry, and the robot naming one of its models.

robot.yaml names the a200-0553's lidar (``platform.extras.launch``, ``lidar_model``); the container's sim plants cast
:data:`~clair.lidar.models.DEFAULT_MODEL`.  If the two diverge, the sim casts another sweep than the robot sees -- and
nothing fails, the costmap just fills differently.  Needs neither ROS nor Docker.
"""

from pathlib import Path

import pytest
import yaml

from clair.lidar import models

ROBOT_YAML = Path(__file__).resolve().parents[2] / "setup" / "config" / "robot.yaml"


def _robot_lidar_models() -> list[str]:
    launches = yaml.safe_load(ROBOT_YAML.read_text(encoding="utf-8"))["platform"]["extras"].get("launch") or []
    return [launch["args"]["lidar_model"] for launch in launches if "lidar_model" in (launch.get("args") or {})]


def test_every_model_is_listed_under_its_name():
    assert all(name == model.name for name, model in models.MODELS.items())


def test_every_model_has_a_driver_the_launch_knows():
    assert {model.driver for model in models.MODELS.values()} <= set(models.DRIVERS)


def test_the_rings_go_bottom_to_top():
    for model in models.MODELS.values():
        assert list(model.rings_deg) == sorted(model.rings_deg), model.name


def test_an_unknown_model_is_refused_by_name():
    with pytest.raises(ValueError, match="known: rs16"):
        models.model("vlp16")


def test_robot_yaml_names_exactly_one_lidar_and_it_is_known():
    assert _robot_lidar_models() == [models.DEFAULT_MODEL], (
        f"robot.yaml's lidar_model {_robot_lidar_models()} is not the model the sim plants cast "
        f"({models.DEFAULT_MODEL})."
    )
