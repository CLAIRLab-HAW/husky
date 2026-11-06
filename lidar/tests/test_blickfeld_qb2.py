"""The Qb2's driver parameters: one file for the device, the wiring for the frame and the topic.  ROS-free."""

import dataclasses

import pytest
import yaml

from clair.lidar import blickfeld_qb2, models, wiring


def test_the_driver_publishes_on_the_wired_topic_in_the_wired_frame():
    parameters = blickfeld_qb2.parameters(models.QB2)
    assert parameters["point_cloud_topic"] == wiring.points_topic()
    assert parameters["frame_id"] == wiring.LIDAR_FRAME


def test_the_template_leaves_frame_and_topic_to_the_wiring():
    template = yaml.safe_load(blickfeld_qb2.TEMPLATE_PATH.read_text(encoding="utf-8"))
    assert not {"frame_id", "point_cloud_topic"} & set(template)


def test_the_template_names_the_device():
    assert blickfeld_qb2.parameters(models.QB2)["fqdn"]


def test_a_model_of_another_driver_is_refused():
    with pytest.raises(ValueError, match="blickfeld_qb2_ros2_driver"):
        blickfeld_qb2.parameters(models.RS16)


def test_a_template_without_a_device_is_refused(tmp_path, monkeypatch):
    template = tmp_path / "blickfeld_qb2.yaml"
    template.write_text("fqdn: ''\n", encoding="utf-8")
    monkeypatch.setattr(blickfeld_qb2, "TEMPLATE_PATH", template)
    with pytest.raises(ValueError, match="fqdn"):
        blickfeld_qb2.parameters(dataclasses.replace(models.QB2))
