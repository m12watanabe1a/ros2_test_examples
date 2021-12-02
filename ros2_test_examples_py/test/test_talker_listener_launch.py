import time

import unittest
import uuid

import launch
import launch_ros
import launch_testing
import launch_testing.actions
import launch_testing_ros

import pytest

import rclpy
from rclpy.node import Node

import std_msgs.msg


@pytest.mark.rostest
def generate_test_description():
    """Generate test description."""

    this_pkg_name = 'ros2_test_examples_py'

    talker_node = launch_ros.actions.Node(
        package=this_pkg_name,
        executable='talker',
        remappings=[('chatter', 'talker_chatter')]
    )

    listener_node = launch_ros.actions.Node(
        package=this_pkg_name,
        executable='listener',
        remappings=[('chatter', 'listener_chatter')]
    )

    return (
        launch.LaunchDescription(
            [
                talker_node,
                listener_node,
                launch_testing.actions.ReadyToTest()
            ]
        ),
        {
            'talker': talker_node,
            'listener': listener_node,
        }
    )


class TestTalkerListenerLink(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        # Create a ROS node for tests
        self.node: Node = rclpy.create_node('test_talker_listener_link')

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_talker_transmits(
            self,
            launch_service: launch.LaunchService,
            talker: Node,
            proc_output: launch_testing.ActiveIoHandler):
        # Except the talker to publish string  on '/talker_chatter' and also write to stdout
        msgs_rx = []

        sub = self.node.create_subscription(
            std_msgs.msg.String,
            'talker_chatter',
            lambda msg: msgs_rx.append(msg),
            10
        )

        try:
            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msgs_rx) > 2:
                    break
            self.assertGreater(len(msgs_rx),  2)

            for msg in msgs_rx:
                proc_output.assertWaitFor(
                    expected_output=msg.data, process=talker
                )
        finally:
            self.node.destroy_subscription(sub)

    def test_listener_receives(
        self,
            launch_service: launch.LaunchService,
            listener: Node,
            proc_output: launch_testing.ActiveIoHandler):
        pub = self.node.create_publisher(
            std_msgs.msg.String,
            'listener_chatter',
            10
        )

        try:
            msg = std_msgs.msg.String()
            msg.data = str(uuid.uuid4())
            for _ in range(10):
                pub.publish(msg)
                success = proc_output.waitFor(
                    expected_output=msg.data,
                    process=listener,
                    timeout=1.0
                )
                if success:
                    break
            assert success, "Waiting for output time out"
        finally:
            self.node.destroy_publisher(pub)

    def test_fuzzy_data(
        self,
        launch_service: launch.LaunchService,
        listener: Node,
        proc_output: launch_testing.ActiveIoHandler
    ):
        def data_manager(msg: std_msgs.msg.String):
            msg.data = msg.data.replace('Hello', 'Aloha')
            return msg

        republisher = launch_testing_ros.DataRepublisher(
            self.node,
            'talker_chatter',
            'listener_chatter',
            std_msgs.msg.String,
            data_manager
        )

        try:
            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if republisher.get_num_republished() > 2:
                    break

            self.assertGreater(republisher.get_num_republished(), 2)

            proc_output.assertWaitFor('Aloha World')

            for msg in republisher.get_republished():
                proc_output.assertWaitFor(msg.data, listener)
        finally:
            republisher.shutdown()
