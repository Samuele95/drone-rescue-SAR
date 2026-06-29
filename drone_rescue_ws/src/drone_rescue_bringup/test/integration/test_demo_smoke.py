#!/usr/bin/env python3
"""
Smoke test for demo components, validates critical services start and publish.

Tests that the storytelling components (camera_director, telemetry_overlay)
initialize correctly alongside coordination nodes.

Does NOT launch full Gazebo simulation (too heavy for testing).
Validates node startup, topic publication, and parameter loading.
"""

import pytest
import unittest
import time
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node


def generate_test_description():
    """
    Launch minimal test configuration for demo smoke test validation.

    Launches demo components without full Gazebo simulation (too heavy for CI):
    - pheromone_server (LifecycleNode)
    - Single drone_controller (LifecycleNode)
    - telemetry_overlay (regular Node)
    - camera_director (regular Node) with camera control disabled
    - lifecycle_manager to orchestrate transitions
    """
    # Pheromone server (single instance) - Lifecycle node
    pheromone_server = launch_ros.actions.LifecycleNode(
        package='drone_rescue_coordination',
        executable='pheromone_server',
        name='pheromone_server',
        namespace='',
        parameters=[{
            'use_sim_time': False,  # No Gazebo in test
            'grid_width': 400,
            'grid_height': 400,
            'cell_resolution': 1.0,
            'origin_x': -200.0,
            'origin_y': -200.0,
            'decay_rate': 0.995,
            'update_rate': 2.0,
            'deposit_value': 1.0,
            'coverage_threshold': 0.1,
        }],
        output='screen',
    )

    # Single drone controller for testing
    controller = launch_ros.actions.LifecycleNode(
        package='drone_rescue_coordination',
        executable='drone_controller',
        name='drone1_controller',
        namespace='',
        parameters=[{
            'drone_name': 'drone1',
            'use_sim_time': False,
            'takeoff_altitude': 10.0,
            'control_rate': 30.0,
            'position_tolerance': 0.5,
            'max_horizontal_speed': 3.0,
            'max_vertical_speed': 2.0,
            'pid_xy_p': 0.8,
            'pid_xy_i': 0.0,
            'pid_xy_d': 0.3,
            'pid_z_p': 1.0,
            'pid_z_i': 0.0,
            'pid_z_d': 0.4,
        }],
        output='screen',
    )

    # Telemetry overlay node (regular node, not lifecycle)
    telemetry_overlay = launch_ros.actions.Node(
        package='drone_rescue_viz',
        executable='telemetry_overlay',
        name='telemetry_overlay',
        namespace='',
        parameters=[{
            'use_sim_time': False,
            'drone_names': ['drone1'],
            'update_rate': 2.0,
        }],
        output='screen',
    )

    # Camera director node (regular node, not lifecycle)
    # Disable camera control to avoid Gazebo CLI dependency
    camera_director = launch_ros.actions.Node(
        package='drone_rescue_coordination',
        executable='camera_director',
        name='camera_director',
        namespace='',
        parameters=[{
            'use_sim_time': False,
            'enable_camera_control': False,  # No gz CLI in test
            'shot_config': '/tmp/dummy_camera_shots.yaml',  # Not used when disabled
            'drone_names': ['drone1'],
        }],
        output='screen',
    )

    # Lifecycle manager to orchestrate transitions
    lifecycle_manager = launch_ros.actions.Node(
        package='drone_rescue_coordination',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{
            'use_sim_time': False,
            'drone_names': ['drone1'],
            'transition_timeout': 10.0,
            'auto_startup': True,
        }],
        output='screen',
    )

    return (
        launch.LaunchDescription([
            pheromone_server,
            controller,
            telemetry_overlay,
            camera_director,
            lifecycle_manager,
            # Longer delay for all nodes to initialize and publish
            launch.actions.TimerAction(
                period=5.0,
                actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]),
        {
            'pheromone_server': pheromone_server,
            'controller': controller,
            'telemetry_overlay': telemetry_overlay,
            'camera_director': camera_director,
            'lifecycle_manager': lifecycle_manager,
        }
    )


class TestDemoSmoke(unittest.TestCase):
    """Active tests that run concurrently with launched nodes."""

    @classmethod
    def setUpClass(cls):
        """Initialize rclpy once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown rclpy after all tests."""
        rclpy.shutdown()

    def setUp(self):
        """Create test node before each test."""
        self.node = rclpy.create_node('test_demo_smoke')

    def tearDown(self):
        """Destroy test node after each test."""
        self.node.destroy_node()

    def test_telemetry_topics_exist(self):
        """
        Verify telemetry overlay topics exist.

        Checks that /telemetry/mission_overlay and /telemetry/drone_status_overlay
        topics appear in the topic list within 15s timeout.
        """
        expected_topics = [
            '/telemetry/mission_overlay',
            '/telemetry/drone_status_overlay',
        ]

        # Wait for topics to appear
        end_time = time.time() + 15.0
        topics_found = {topic: False for topic in expected_topics}

        while time.time() < end_time and not all(topics_found.values()):
            # Get current topic list
            topic_names_and_types = self.node.get_topic_names_and_types()
            current_topics = [name for name, _ in topic_names_and_types]

            # Check which topics exist
            for topic in expected_topics:
                if topic in current_topics:
                    if not topics_found[topic]:
                        topics_found[topic] = True
                        print(f"INFO: Found topic {topic}")

            time.sleep(0.2)

        # Assert all topics found
        for topic, found in topics_found.items():
            self.assertTrue(
                found,
                f"Topic {topic} not found within 15s timeout"
            )

    def test_camera_director_initialized(self):
        """
        Verify camera_director node is running.

        Checks that camera_director appears in the node list within 15s.
        """
        end_time = time.time() + 15.0
        camera_director_found = False

        while time.time() < end_time and not camera_director_found:
            # Get current node list
            node_names = self.node.get_node_names()

            if 'camera_director' in node_names:
                camera_director_found = True
                print("INFO: camera_director node found")
                break

            time.sleep(0.2)

        self.assertTrue(
            camera_director_found,
            "camera_director node not found within 15s timeout"
        )

    def test_critical_topics_available(self):
        """
        Verify critical coordination topics exist.

        Checks that /coverage/metrics and /diagnostics topics appear
        in the topic list within 15s timeout.
        """
        expected_topics = [
            '/coverage/metrics',
            '/diagnostics',
        ]

        # Wait for topics to appear
        end_time = time.time() + 15.0
        topics_found = {topic: False for topic in expected_topics}

        while time.time() < end_time and not all(topics_found.values()):
            # Get current topic list
            topic_names_and_types = self.node.get_topic_names_and_types()
            current_topics = [name for name, _ in topic_names_and_types]

            # Check which topics exist
            for topic in expected_topics:
                if topic in current_topics:
                    if not topics_found[topic]:
                        topics_found[topic] = True
                        print(f"INFO: Found topic {topic}")

            time.sleep(0.2)

        # Assert all topics found
        for topic, found in topics_found.items():
            self.assertTrue(
                found,
                f"Topic {topic} not found within 15s timeout"
            )


@launch_testing.post_shutdown_test()
class TestDemoSmokeAfterShutdown(unittest.TestCase):
    """Post-shutdown tests verify clean exit behavior."""

    def test_all_nodes_exited_cleanly(self, proc_info):
        """Verify all processes exited cleanly without crashes."""
        launch_testing.asserts.assertExitCodes(proc_info)
