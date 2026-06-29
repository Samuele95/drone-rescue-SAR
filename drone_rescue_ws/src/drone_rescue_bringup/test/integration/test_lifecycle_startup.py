#!/usr/bin/env python3
"""
Integration test for lifecycle node startup validation.

Tests that all lifecycle nodes (pheromone_server, controllers, surveyors)
successfully transition to ACTIVE state and report OK diagnostics status.

This validates INTG-02: Multi-drone startup validated via launch_testing.
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
from lifecycle_msgs.msg import State, TransitionEvent
from lifecycle_msgs.srv import GetState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


def generate_test_description():
    """
    Launch minimal test configuration for lifecycle startup validation.

    Launches lifecycle nodes without full Gazebo simulation (too heavy for CI):
    - pheromone_server (LifecycleNode)
    - Single drone_controller (LifecycleNode)
    - Single surveyor (LifecycleNode)
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

    # Single surveyor for testing
    surveyor = launch_ros.actions.LifecycleNode(
        package='drone_rescue_coordination',
        executable='surveyor',
        name='drone1_surveyor',
        namespace='',
        parameters=[{
            'drone_name': 'drone1',
            'use_sim_time': False,
            'update_rate': 2.0,
            'survey_altitude': 10.0,
            'survey_speed': 2.0,
            # weight params renamed to slides B1..B5 vocabulary
            # (Marcelletti pp. 96-100); see surveyor.py.
            'b1_avoid_visited_weight': 0.6,
            'b2_explore_unvisited_weight': 0.4,
            'pheromone_repel_threshold': 0.1,
            'unexplored_attract_threshold': 0.3,
            'low_battery_threshold': 0.2,
            'repulsion_radius': 5,
            'boundary_search_radius': 20,
            'grid_width': 400,
            'grid_height': 400,
            'cell_resolution': 1.0,
            'origin_x': -200.0,
            'origin_y': -200.0,
            'collision_avoidance_distance': 5.0,
            'b3_avoid_peers_weight': 1.5,
            'all_drone_names': ['drone1'],
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
            surveyor,
            lifecycle_manager,
            # Short delay before tests start
            launch.actions.TimerAction(
                period=0.5,
                actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]),
        {
            'pheromone_server': pheromone_server,
            'controller': controller,
            'surveyor': surveyor,
            'lifecycle_manager': lifecycle_manager,
        }
    )


class TestLifecycleStartup(unittest.TestCase):
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
        self.node = rclpy.create_node('test_lifecycle_startup')

    def tearDown(self):
        """Destroy test node after each test."""
        self.node.destroy_node()

    def test_nodes_reach_active_state(self):
        """
        Verify all lifecycle nodes reach ACTIVE state.

        Monitors lifecycle_state topics for pheromone_server, controller,
        and surveyor. Waits up to 30s for all nodes to reach PRIMARY_STATE_ACTIVE.

        Uses fallback to get_state service if topic subscription misses transition.
        """
        nodes_to_check = {
            'pheromone_server': '/pheromone_server/transition_event',
            'drone1_controller': '/drone1_controller/transition_event',
            'drone1_surveyor': '/drone1_surveyor/transition_event',
        }

        # Track received state messages per node
        state_messages = {name: [] for name in nodes_to_check.keys()}

        # Create subscriptions (before transitions to avoid missing events)
        subscriptions = {}
        for node_name, topic in nodes_to_check.items():
            sub = self.node.create_subscription(
                TransitionEvent,
                topic,
                lambda msg, name=node_name: state_messages[name].append(msg),
                10
            )
            subscriptions[node_name] = sub

        try:
            # Wait for all nodes to reach ACTIVE state
            end_time = time.time() + 30.0  # 30s timeout
            nodes_active = {name: False for name in nodes_to_check.keys()}

            while time.time() < end_time and not all(nodes_active.values()):
                rclpy.spin_once(self.node, timeout_sec=0.1)

                # Check received messages for ACTIVE state
                for node_name, messages in state_messages.items():
                    if not nodes_active[node_name]:
                        for msg in messages:
                            if msg.goal_state.id == State.PRIMARY_STATE_ACTIVE:
                                nodes_active[node_name] = True
                                print(f"INFO: {node_name} reached ACTIVE state")
                                break

            # Fallback: Query state via service for nodes not confirmed active
            for node_name, is_active in nodes_active.items():
                if not is_active:
                    # Try get_state service
                    service_name = f'/{node_name}/get_state'
                    client = self.node.create_client(GetState, service_name)

                    if client.wait_for_service(timeout_sec=1.0):
                        request = GetState.Request()
                        future = client.call_async(request)
                        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

                        if future.done():
                            response = future.result()
                            if response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                                nodes_active[node_name] = True
                                print(f"INFO: {node_name} ACTIVE (via service)")

                    self.node.destroy_client(client)

            # Assert all nodes reached ACTIVE state
            for node_name, is_active in nodes_active.items():
                self.assertTrue(
                    is_active,
                    f"{node_name} failed to reach ACTIVE state within 30s timeout"
                )

        finally:
            # Cleanup subscriptions
            for sub in subscriptions.values():
                self.node.destroy_subscription(sub)

    def test_diagnostics_ok_after_startup(self):
        """
        Verify all nodes report OK status in diagnostics after startup.

        Subscribes to /diagnostics topic and collects messages for 10s.
        Verifies that pheromone_server, drone1_controller, and drone1_surveyor
        all report DiagnosticStatus.level == 0 (OK).

        Per user decision: "Verify OK status" - check level == 0.
        """
        expected_nodes = ['pheromone_server', 'drone1_controller', 'drone1_surveyor']
        diag_messages = []

        # Subscribe to diagnostics
        sub = self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            lambda msg: diag_messages.append(msg),
            10
        )

        try:
            # Collect diagnostics for 10s
            end_time = time.time() + 10.0
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            # Verify each expected node reported OK status
            nodes_ok = {name: False for name in expected_nodes}

            for msg in diag_messages:
                for status in msg.status:
                    for expected_node in expected_nodes:
                        # Check if node name appears in status.name
                        if expected_node in status.name:
                            # Check if level is OK (0)
                            if status.level == DiagnosticStatus.OK:
                                nodes_ok[expected_node] = True
                                print(f"INFO: {expected_node} reports OK diagnostics")

            # Assert all nodes reported OK
            for node_name, is_ok in nodes_ok.items():
                self.assertTrue(
                    is_ok,
                    f"{node_name} did not report OK status in diagnostics within 10s"
                )

        finally:
            # Cleanup subscription
            self.node.destroy_subscription(sub)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Post-shutdown tests verify clean exit behavior."""

    def test_exit_codes(self, proc_info):
        """Verify all processes exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
