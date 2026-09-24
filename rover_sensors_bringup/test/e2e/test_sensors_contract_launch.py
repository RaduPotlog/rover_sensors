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
Contract test for the sensor payload, started the way the sensors container starts it.

The payload and the platform are joined only by topics. This test pins that contract down:
the payload publishes the agreed topics with the agreed types and QoS, never publishes TF, and
its nodes subscribe to nothing the payload does not publish itself (so never to a platform
topic; rover_gps_node reading rover_gps_driver's gps/fix is fine).

No sensor is needed. Without data both drivers still come up active and report the missing
stream through diagnostics, which this test does not look at. Every check is made per payload
node, so a live rover graph on the same Zenoh router can neither satisfy nor break it.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import pytest
import rclpy
from rclpy.qos import ReliabilityPolicy

NAMESPACE = 'sensors_contract_e2e'

# The lists below are the reviewed contract, written out by hand on purpose rather than
# discovered from whatever runs. A new sensor package added to rover_sensors.launch.py must add
# its nodes, topics and reliability here and to the "Topic contract" table in the repo README,
# and turn its use_<sensor> argument on in generate_test_description(). Otherwise its topics are
# never contract-checked.
LIFECYCLE_NODES = ('rover_gps_driver', 'rover_rs16_lidar_node')
PAYLOAD_NODES = LIFECYCLE_NODES + ('rover_gps_node',)

# Topic (relative to the namespace) -> message type the platform and the autonomy expect.
EXPECTED_PUBLICATIONS = {
    'gps/fix': 'sensor_msgs/msg/NavSatFix',
    'gps/vel': 'geometry_msgs/msg/TwistStamped',
    'gps/heading': 'geometry_msgs/msg/QuaternionStamped',
    'gps/time_reference': 'sensor_msgs/msg/TimeReference',
    'rslidar_points': 'sensor_msgs/msg/PointCloud2',
    'scan': 'sensor_msgs/msg/LaserScan',
    'diagnostics': 'diagnostic_msgs/msg/DiagnosticArray',
}

# Sensor streams are best-effort. The GNSS topics are reliable on purpose, so that reliable
# subscribers such as navsat_transform_node still match (see rover_gps/README.md).
EXPECTED_RELIABILITY = {
    'scan': ReliabilityPolicy.BEST_EFFORT,
    'rslidar_points': ReliabilityPolicy.BEST_EFFORT,
    'gps/fix': ReliabilityPolicy.RELIABLE,
    'gps/vel': ReliabilityPolicy.RELIABLE,
    'gps/heading': ReliabilityPolicy.RELIABLE,
    'gps/time_reference': ReliabilityPolicy.RELIABLE,
}

TF_TOPICS = ('tf', 'tf_static')

# ROS plumbing every node may touch; not part of the payload contract.
INFRASTRUCTURE_TOPICS = {'/parameter_events', '/clock', '/rosout'}


def _absolute(topic):
    return f'/{NAMESPACE}/{topic}'


@pytest.mark.launch_test
def generate_test_description():
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rover_sensors_bringup'),
            'launch', 'rover_sensors.launch.py')),
        launch_arguments={
            'namespace': NAMESPACE,
            'use_gps': 'true',
            'use_lidar': 'true',
        }.items(),
    )

    return LaunchDescription([sensors, launch_testing.actions.ReadyToTest()])


class TestSensorsContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Outside the payload namespace, so the probe never counts as a payload node.
        cls.node = rclpy.create_node('sensors_contract_probe')
        cls._wait_for_payload()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _spin_until(cls, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(cls.node, timeout_sec=0.05)
        return predicate()

    @classmethod
    def _wait_for_payload(cls):
        """Wait until every payload node is up and both drivers are active."""
        for name in LIFECYCLE_NODES:
            client = cls.node.create_client(GetState, f'/{NAMESPACE}/{name}/get_state')
            if not client.wait_for_service(timeout_sec=20.0):
                raise AssertionError(f'{name}/get_state never appeared')

            state = {'id': None}

            def is_active():
                future = client.call_async(GetState.Request())
                if not cls._spin_until(future.done, 2.0) or future.result() is None:
                    return False
                state['id'] = future.result().current_state.id
                return state['id'] == State.PRIMARY_STATE_ACTIVE

            if not cls._spin_until(is_active, 20.0):
                raise AssertionError(
                    f'{name} never became active (last state id: {state["id"]})')
            cls.node.destroy_client(client)

        # Every expected publication must be visible before the per-node checks run, so a
        # slow graph update cannot pass for a missing topic.
        if not cls._spin_until(
                lambda: EXPECTED_PUBLICATIONS.keys() <= cls._published_topics().keys(), 10.0):
            missing = EXPECTED_PUBLICATIONS.keys() - cls._published_topics().keys()
            raise AssertionError(f'payload never published {sorted(missing)}')

    @classmethod
    def _payload_nodes(cls):
        return [
            name for name, namespace in cls.node.get_node_names_and_namespaces()
            if namespace == f'/{NAMESPACE}'
        ]

    @classmethod
    def _by_node(cls, query):
        """(node, topic, types) for every endpoint of every payload node."""
        for name in cls._payload_nodes():
            for topic, types in query(name, f'/{NAMESPACE}'):
                yield name, topic, types

    @classmethod
    def _published_topics(cls):
        """Relative topic -> types, for topics the payload publishes inside its namespace."""
        prefix = f'/{NAMESPACE}/'
        return {
            topic[len(prefix):]: types
            for _, topic, types in cls._by_node(cls.node.get_publisher_names_and_types_by_node)
            if topic.startswith(prefix)
        }

    def test_only_the_expected_nodes_run(self):
        self.assertCountEqual(self._payload_nodes(), PAYLOAD_NODES)

    def test_publishes_the_agreed_topics_and_types(self):
        published = self._published_topics()
        for topic, msg_type in EXPECTED_PUBLICATIONS.items():
            with self.subTest(topic=topic):
                self.assertIn(topic, published)
                self.assertEqual(published[topic], [msg_type])

    def test_never_publishes_tf(self):
        offenders = [
            (name, topic)
            for name, topic, _ in self._by_node(self.node.get_publisher_names_and_types_by_node)
            if topic.rsplit('/', 1)[-1] in TF_TOPICS
        ]
        self.assertEqual(offenders, [], 'payload drivers must never publish TF')

    def test_subscribes_only_to_its_own_topics(self):
        own = {
            topic
            for _, topic, _ in self._by_node(self.node.get_publisher_names_and_types_by_node)
        }
        offenders = [
            (name, topic)
            for name, topic, _ in self._by_node(self.node.get_subscriber_names_and_types_by_node)
            if topic not in own and topic not in INFRASTRUCTURE_TOPICS
        ]
        self.assertEqual(
            offenders, [],
            'payload nodes must only subscribe to topics the payload itself publishes, '
            'never to platform (or any foreign) topics')

    def test_publisher_reliability(self):
        payload = set(self._payload_nodes())
        for topic, reliability in EXPECTED_RELIABILITY.items():
            with self.subTest(topic=topic):
                endpoints = [
                    info for info in self.node.get_publishers_info_by_topic(_absolute(topic))
                    if info.node_name in payload and info.node_namespace == f'/{NAMESPACE}'
                ]
                self.assertEqual(len(endpoints), 1, f'expected one payload publisher on {topic}')
                self.assertEqual(endpoints[0].qos_profile.reliability, reliability)


@launch_testing.post_shutdown_test()
class TestSensorsContractShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
