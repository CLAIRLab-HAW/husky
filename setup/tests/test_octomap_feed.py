"""octomap-feed's depth image to point cloud, beyond what ``--selftest`` asserts."""

from __future__ import annotations

import numpy as np
import pytest
from octomap_feed import depth_to_cloud, selftest

#: A 4x6 camera with its principal point on pixel (u 2, v 1) and different focal lengths, so x and y cannot swap.
K = np.array([[2.0, 0.0, 2.0], [0.0, 4.0, 1.0], [0.0, 0.0, 1.0]])


def test_selftest_passes():
    assert selftest() == 0


def test_each_pixel_lands_on_its_ray_in_the_optical_frame():
    depth = np.zeros((4, 6), dtype=np.float32)
    depth[1, 2] = 1.0  # the principal point: on the optical axis
    depth[3, 5] = 2.0  # right of and below it: x and y positive
    cloud = depth_to_cloud(depth, K, stride=1)
    assert cloud.dtype == np.float32
    assert cloud.tolist() == [[0.0, 0.0, 1.0], [(5 - 2) / 2.0 * 2.0, (3 - 1) / 4.0 * 2.0, 2.0]]


def test_the_band_is_open_at_both_ends_and_invalid_depths_drop():
    depth = np.array([[0.15, 0.16, 2.49, 2.5, np.nan, np.inf]], dtype=np.float32)
    cloud = depth_to_cloud(depth, np.eye(3), stride=1, min_depth=0.15, max_depth=2.5)
    assert cloud[:, 2] == pytest.approx([0.16, 2.49])


def test_millimeters_arrive_as_meters():
    depth = np.full((4, 6), 1234, dtype=np.uint16)
    assert depth_to_cloud(depth, K, stride=1)[:, 2] == pytest.approx(np.full(24, 1.234))


@pytest.mark.parametrize(("stride", "points"), [(2, 6), (3, 4), (0, 24), (-1, 24)])
def test_the_stride_keeps_every_nth_row_and_column_and_never_less_than_every_one(stride, points):
    assert len(depth_to_cloud(np.ones((4, 6), dtype=np.float32), K, stride=stride)) == points


@pytest.mark.parametrize(
    ("depth", "k"),
    [
        (np.zeros((4, 6), dtype=np.float32), K),  # nothing in the band
        (np.ones((4, 6), dtype=np.float32), np.zeros((3, 3))),  # no camera_info intrinsics yet
    ],
)
def test_nothing_usable_gives_an_empty_cloud_of_the_right_shape(depth, k):
    cloud = depth_to_cloud(depth, k)
    assert cloud.shape == (0, 3) and cloud.dtype == np.float32
