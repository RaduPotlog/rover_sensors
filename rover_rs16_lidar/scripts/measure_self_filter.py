#!/usr/bin/env python3

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

"""
Measure the scan.self_filter boxes for rover_rs16_lidar_node from the live point cloud.

Run it, then turn the rover in place (drive UI or RC) in open space, slowly, through at least
--min-rotation-deg. While the rover turns, everything around it sweeps through the lidar
frame, but parts of the rover itself (a support post, a mast, a cable) stay in the same
place. A voxel near the lidar counts as the rover when it is hit throughout the turn - in at
least --min-coverage of the 30 degree heading steps the rover passed through - not merely
often: a wall close by lights up a whole ring of voxels, but each one only while the rover
faces it. Rover voxels are grouped into boxes and printed as a scan.self_filter block to
paste into config/rover_rs16_lidar.yaml.

    ros2 run rover_rs16_lidar measure_self_filter.py --ros-args -r __ns:=/rover

Re-run it whenever the lidar or its mount moves: the boxes are in the lidar's own frame.
"""

import argparse
import itertools
import math
import sys

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


# Heading steps for the coverage test: a rover-fixed voxel must be hit in most of them.
HEADING_STEP_DEG = 30.0


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--cloud-topic", default="rslidar_points")
    parser.add_argument("--odom-topic", default="odom")
    parser.add_argument("--max-range", type=float, default=1.0,
                        help="Only returns this close to the lidar can be the rover [m].")
    parser.add_argument("--min-z", type=float, default=-0.25,
                        help="Bottom of the scan slice (scan.min_height) [m].")
    parser.add_argument("--max-z", type=float, default=0.25,
                        help="Top of the scan slice (scan.max_height) [m].")
    parser.add_argument("--voxel", type=float, default=0.05, help="Voxel edge [m].")
    parser.add_argument("--min-coverage", type=float, default=0.75,
                        help="Share of the turn's heading steps a voxel must be hit in.")
    parser.add_argument("--min-fraction", type=float, default=0.1,
                        help="Share of clouds a voxel must also be hit in.")
    parser.add_argument("--margin", type=float, default=0.03,
                        help="Padding added to every side of a box [m].")
    parser.add_argument("--min-rotation-deg", type=float, default=270.0,
                        help="How far the rover must turn before the result means anything.")
    parser.add_argument("--timeout", type=float, default=180.0, help="Give up after [s].")
    return parser.parse_args(argv)


def xyz_of(cloud):
    """Return the cloud's x, y, z as an (N, 3) float32 array."""
    offsets = {field.name: field.offset for field in cloud.fields
               if field.datatype == PointField.FLOAT32}
    if not all(axis in offsets for axis in "xyz"):
        raise RuntimeError("the cloud has no float32 x/y/z fields")
    dtype = np.dtype({"names": ["x", "y", "z"],
                      "formats": [np.float32] * 3,
                      "offsets": [offsets["x"], offsets["y"], offsets["z"]],
                      "itemsize": cloud.point_step})
    points = np.frombuffer(bytes(cloud.data), dtype=dtype, count=cloud.width * cloud.height)
    return np.stack([points["x"], points["y"], points["z"]], axis=1)


def clusters(voxels):
    """Group voxel indices into 26-connected clusters."""
    remaining = set(voxels)
    neighbours = [d for d in itertools.product((-1, 0, 1), repeat=3) if d != (0, 0, 0)]
    while remaining:
        seed = remaining.pop()
        group, frontier = [seed], [seed]
        while frontier:
            x, y, z = frontier.pop()
            for dx, dy, dz in neighbours:
                other = (x + dx, y + dy, z + dz)
                if other in remaining:
                    remaining.remove(other)
                    group.append(other)
                    frontier.append(other)
        yield group


class SelfFilterAccumulator:
    """Votes rover-fixed voxels from clouds taken while the rover turns; no ROS in here."""

    def __init__(self, args):
        self._args = args
        self._counts = {}
        self._low = {}
        self._high = {}
        self._headings = {}
        self._visited = set()
        self._clouds = 0
        self._last_yaw = None
        self._rotation = 0.0

    @property
    def rotation_deg(self):
        return math.degrees(self._rotation)

    @property
    def clouds(self):
        return self._clouds

    def add_yaw(self, yaw):
        """Accumulate how far the rover has turned, in either direction."""
        if self._last_yaw is not None:
            step = math.atan2(math.sin(yaw - self._last_yaw), math.cos(yaw - self._last_yaw))
            self._rotation += abs(step)
        self._last_yaw = yaw

    def add_cloud(self, xyz):
        """Vote with one cloud, given as an (N, 3) array in the lidar frame."""
        a = self._args
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        horizontal = np.hypot(xyz[:, 0], xyz[:, 1])
        near = xyz[(horizontal <= a.max_range) & (xyz[:, 2] >= a.min_z) & (xyz[:, 2] <= a.max_z)]
        heading = int(self.rotation_deg // HEADING_STEP_DEG)
        self._visited.add(heading)
        voxels = [tuple(v) for v in np.floor(near / a.voxel).astype(int)]
        # Keep the extent of the points actually seen in each voxel - boxes are cut to those,
        # not to whole voxels.
        for voxel, point in zip(voxels, near):
            if voxel not in self._low:
                self._low[voxel] = point.copy()
                self._high[voxel] = point.copy()
            else:
                np.minimum(self._low[voxel], point, out=self._low[voxel])
                np.maximum(self._high[voxel], point, out=self._high[voxel])
        # One vote per voxel per cloud, however many points of that cloud land in it.
        for voxel in set(voxels):
            self._counts[voxel] = self._counts.get(voxel, 0) + 1
            self._headings.setdefault(voxel, set()).add(heading)
        self._clouds += 1

    def boxes(self):
        """Return one (min, max, coverage) box per cluster of rover-fixed voxels."""
        a = self._args
        # Without a turn nothing sweeps: the whole room would stay put and read as the rover.
        if self.rotation_deg < a.min_rotation_deg:
            raise RuntimeError(
                f"the rover turned only {self.rotation_deg:.0f} of the "
                f"{a.min_rotation_deg:.0f} deg needed to tell it apart from the room")
        # A return jitters across voxel boundaries, splitting its votes, so a voxel is judged
        # by its 3x3x3 neighbourhood.
        offsets = list(itertools.product((-1, 0, 1), repeat=3))

        def neighbourhood(voxel):
            x, y, z = voxel
            return [(x + dx, y + dy, z + dz) for dx, dy, dz in offsets]

        def share(voxel):
            votes = sum(self._counts.get(v, 0) for v in neighbourhood(voxel))
            return votes / self._clouds

        def coverage(voxel):
            seen = set().union(*(self._headings.get(v, set()) for v in neighbourhood(voxel)))
            return len(seen & self._visited) / len(self._visited)

        # The own-count floor keeps a stray return next to the rover out of its box.
        fixed = {v for v, n in self._counts.items()
                 if n >= 0.05 * self._clouds and share(v) >= a.min_fraction
                 and coverage(v) >= a.min_coverage}
        result = []
        for group in clusters(fixed):
            low = np.min([self._low[v] for v in group], axis=0) - a.margin
            high = np.max([self._high[v] for v in group], axis=0) + a.margin
            result.append((low, high, max(coverage(v) for v in group)))
        return sorted(result, key=lambda box: math.hypot(*(box[0][:2] + box[1][:2]) / 2.0))


class SelfFilterMeasurement(Node):

    def __init__(self, args):
        super().__init__("measure_self_filter")
        self.accumulator = SelfFilterAccumulator(args)
        self.create_subscription(
            PointCloud2, args.cloud_topic, self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.accumulator.add_yaw(
            math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _on_cloud(self, msg):
        self.accumulator.add_cloud(xyz_of(msg))


def print_config(boxes):
    names = [f"self_{i}" for i in range(len(boxes))]
    print("\n# Paste under rover_rs16_lidar_node: ros__parameters: scan: in rover_rs16_lidar.yaml")
    print("      self_filter:")
    print(f'        boxes: [{", ".join(names)}]')
    for name, (low, high, coverage) in zip(names, boxes):
        centre = (low + high) / 2.0
        print(f"        # hit through {coverage:.0%} of the turn, centre {centre[0]:+.2f} "
              f"{centre[1]:+.2f} {centre[2]:+.2f} m, "
              f"{math.hypot(centre[0], centre[1]):.2f} m from the lidar")
        print(f"        {name}:")
        for axis, lo, hi in zip("xyz", low, high):
            print(f"          min_{axis}: {lo:.2f}")
            print(f"          max_{axis}: {hi:.2f}")


def main(argv=None):
    rclpy.init(args=argv)
    args = parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])
    node = SelfFilterMeasurement(args)
    acc = node.accumulator
    start = node.get_clock().now()
    print(f"Turn the rover in place, at least {args.min_rotation_deg:.0f} deg...")
    try:
        last_report = -1
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            elapsed = (node.get_clock().now() - start).nanoseconds * 1e-9
            if int(elapsed) // 5 != last_report:
                last_report = int(elapsed) // 5
                print(f"  {elapsed:5.0f} s: {acc.clouds} clouds, "
                      f"turned {acc.rotation_deg:.0f} deg")
            if acc.rotation_deg >= args.min_rotation_deg:
                break
            if elapsed > args.timeout:
                print(f"Timed out after turning only {acc.rotation_deg:.0f} deg: a return that "
                      "stays put without rotation cannot be told apart from the room.",
                      file=sys.stderr)
                return 1
        if acc.clouds == 0:
            print("No point cloud received; is the lidar running?", file=sys.stderr)
            return 1
        boxes = acc.boxes()
        if not boxes:
            print("Nothing near the lidar stayed put while turning: no self-filter box needed.")
        else:
            print_config(boxes)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
