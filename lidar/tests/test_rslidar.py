"""Mock and robot drive the SAME driver -- that is the whole point.

On 2026-08-19 the gripper demonstrated what happens when mock and real diverge: the container delivered no finger
joint at all to the twin, and it did so silently, because there was no E2E test for the gripper.  Here the
non-divergence therefore becomes the test condition: everything except the packet source has to be byte for byte the
same in both cases.

Needs neither ROS nor Docker.
"""

import copy
import dataclasses

import pytest
import yaml

from clair.lidar import models
from clair.lidar import rslidar as rc


@pytest.fixture
def template() -> dict:
    return yaml.safe_load(rc.TEMPLATE_PATH.read_text(encoding="utf-8"))


def test_online_selects_the_device(template):
    out = rc.render(template, models.RS16)
    assert out["common"]["msg_source"] == rc.MSG_SOURCE_ONLINE


def test_pcap_selects_the_file(template):
    out = rc.render(template, models.RS16, pcap_path="/data/rs16_labor.pcap")
    assert out["common"]["msg_source"] == rc.MSG_SOURCE_PCAP
    assert out["lidar"][0]["driver"]["pcap_path"] == "/data/rs16_labor.pcap"


def test_online_carries_no_pcap_path(template):
    out = rc.render(template, models.RS16)
    assert "pcap_path" not in out["lidar"][0]["driver"], (
        "A pcap_path in the online configuration is a trap: it looks harmless "
        "and decides nothing, until somebody changes msg_source."
    )


def test_only_the_packet_source_differs(template):
    """The core: everything except the source is identical in mock and live."""
    online = rc.render(copy.deepcopy(template), models.RS16)
    pcap = rc.render(copy.deepcopy(template), models.RS16, pcap_path="/data/x.pcap")

    online["common"].pop("msg_source")
    pcap["common"].pop("msg_source")
    pcap["lidar"][0]["driver"].pop("pcap_path")
    pcap["lidar"][0]["driver"].pop("pcap_repeat")
    pcap["lidar"][0]["driver"].pop("pcap_rate")

    assert online == pcap


def test_the_device_type_is_the_models_on_both_paths(template):
    for pcap in (None, "/data/x.pcap"):
        out = rc.render(copy.deepcopy(template), models.RS16, pcap_path=pcap)
        assert out["lidar"][0]["driver"]["lidar_type"] == "RS16"


def test_the_template_names_no_device_type(template):
    """The model is robot.yaml's to name; a lidar_type in the template could disagree with it."""
    assert "lidar_type" not in template["lidar"][0]["driver"]


def test_a_model_of_another_driver_is_refused(template):
    other = dataclasses.replace(models.RS16, name="other", driver="other")
    with pytest.raises(ValueError, match="rslidar_sdk"):
        rc.render(template, other)


def test_the_frame_stays_canonical_on_both_paths(template):
    """lidar3d_0_laser is the name robot.yaml generates -- if the driver
    deviates from it, tf2 does not find the point cloud and nobody says so."""
    for out in (
        rc.render(copy.deepcopy(template), models.RS16),
        rc.render(copy.deepcopy(template), models.RS16, pcap_path="/data/x.pcap"),
    ):
        assert out["lidar"][0]["ros"]["ros_frame_id"] == "lidar3d_0_laser"


def test_a_template_without_lidar_section_is_refused():
    with pytest.raises(ValueError, match="lidar"):
        rc.render({"common": {}}, models.RS16)


def test_a_template_without_common_section_is_refused():
    with pytest.raises(ValueError, match="common"):
        rc.render({"lidar": [{}]}, models.RS16)


def test_write_resolved_produces_loadable_yaml(tmp_path):
    out = rc.write_resolved(tmp_path / "resolved.yaml", models.RS16, pcap_path="/data/rs16_labor.pcap")
    loaded = yaml.safe_load(out.read_text(encoding="utf-8"))
    assert loaded["common"]["msg_source"] == rc.MSG_SOURCE_PCAP
