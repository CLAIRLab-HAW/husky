"""The lidar's topic and frame names are contracts, not spellings.

The graph of this robot is namespaced throughout; tf2 however broadcasts on the ABSOLUTE names /tf and /tf_static,
where the node namespace does not take effect.  If the remap is missing, a node publishes globally while all the others
listen on /a200_0553/tf -- and there is no error message, only an empty TF chain.

Needs neither ROS nor Docker.
"""

import yaml

from clair.lidar import LAUNCH_PATH, rslidar, wiring


def _template() -> dict:
    return yaml.safe_load(rslidar.TEMPLATE_PATH.read_text(encoding="utf-8"))


def test_the_points_topic_follows_the_clearpath_convention():
    assert wiring.points_topic() == "/a200_0553/sensors/lidar3d_0/points"


def test_the_filtered_points_sit_beside_the_raw_ones():
    assert wiring.filtered_points_topic() == wiring.points_topic() + "_filtered"


def test_the_driver_template_publishes_on_the_wired_points_topic():
    """Driver template and wiring must not diverge."""
    assert _template()["lidar"][0]["ros"]["ros_send_point_cloud_topic"] == wiring.points_topic()


def test_the_driver_template_uses_the_wired_frame():
    assert _template()["lidar"][0]["ros"]["ros_frame_id"] == wiring.LIDAR_FRAME


def test_tf_is_remapped_into_the_namespace():
    assert ("/tf", "tf") in wiring.TF_REMAPS
    assert ("/tf_static", "tf_static") in wiring.TF_REMAPS


def test_the_launch_path_points_at_the_launch_file():
    assert LAUNCH_PATH.is_file()
