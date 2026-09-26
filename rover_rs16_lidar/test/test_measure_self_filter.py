# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""Tests for scripts/measure_self_filter.py - the voting, not the ROS wiring."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import measure_self_filter as tool  # noqa: E402, I100


def turn(accumulator, clouds_at_heading, degrees=300.0, steps=100):
    """Turn the rover in place, feeding the cloud each heading produces."""
    for k in range(steps):
        yaw = math.radians(degrees * k / steps)
        accumulator.add_yaw(yaw)
        accumulator.add_cloud(np.array(clouds_at_heading(k, yaw), dtype=np.float32))


def wall(yaw, distance=0.8):
    """Return a 1.2 m wall at `distance` along world +x, as the turned lidar sees it."""
    points = []
    for s in np.linspace(-0.6, 0.6, 25):
        x = math.cos(-yaw) * distance - math.sin(-yaw) * s
        y = math.sin(-yaw) * distance + math.cos(-yaw) * s
        points.append([x, y, 0.0])
    return points


def post(k):
    """Return Rover A1's support post as measured: 0.39 m to the lidar's left, 1 in 4 clouds."""
    rng = np.random.default_rng(k)
    if k % 4:
        return []
    return [[0.01 * rng.standard_normal(), 0.39 + 0.01 * rng.standard_normal(), z]
            for z in (-0.05, 0.0, 0.05)]


def test_finds_the_post_and_not_the_wall():
    accumulator = tool.SelfFilterAccumulator(tool.parse_args([]))
    turn(accumulator, lambda k, yaw: post(k) + wall(yaw) + [[math.nan] * 3])

    boxes = accumulator.boxes()

    assert len(boxes) == 1
    low, high, coverage = boxes[0]
    assert low[0] < 0.0 < high[0] and low[1] < 0.39 < high[1]
    # Cut to the points plus the margin, not to whole voxels.
    assert (high - low)[:2].max() < 0.15
    assert coverage >= 0.75


def test_reports_nothing_when_only_the_room_is_near():
    accumulator = tool.SelfFilterAccumulator(tool.parse_args([]))
    turn(accumulator, lambda k, yaw: wall(yaw) + wall(yaw + math.pi / 2, distance=0.5))

    assert accumulator.boxes() == []


def test_refuses_to_judge_without_a_turn():
    accumulator = tool.SelfFilterAccumulator(tool.parse_args([]))
    turn(accumulator, lambda k, yaw: post(k) + wall(yaw), degrees=90.0)

    with pytest.raises(RuntimeError):
        accumulator.boxes()


def test_reads_the_nodes_point_cloud_layout():
    from sensor_msgs.msg import PointCloud2, PointField

    points = np.array([[1.0, 2.0, 3.0, 7.0], [math.nan, 0.0, 0.0, 0.0]], dtype=np.float32)
    cloud = PointCloud2()
    cloud.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(["x", "y", "z", "intensity"])]
    cloud.point_step = 16
    cloud.width = 2
    cloud.height = 1
    cloud.data = points.tobytes()

    xyz = tool.xyz_of(cloud)

    assert xyz.shape == (2, 3)
    assert xyz[0].tolist() == [1.0, 2.0, 3.0]
    assert math.isnan(xyz[1][0])
