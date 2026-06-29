#!/usr/bin/env python3
"""
Integration test for use_sim_time consistency.

Verifies INTG-03: All nodes use simulation time consistently (use_sim_time=true).

This test launches a minimal subset of nodes and audits their use_sim_time parameter
to ensure consistent simulation time usage across the system.
"""

import pytest
import unittest
import subprocess
import time
from typing import List, Dict

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from launch.actions import TimerAction
from launch_testing.actions import ReadyToTest


def generate_test_description():
    """
    Generate test launch description.

    Launches minimal node set without Gazebo to verify parameter configuration:
    - pheromone_server (lifecycle)
    - One controller (lifecycle)
    - One surveyor (lifecycle)
    - lifecycle_manager

    NOTE: This test verifies use_sim_time PARAMETER is set, not that /clock is used.
    Gazebo is not launched to avoid heavyweight simulation overhead.
    """

    # Pheromone server (lifecycle node)
    pheromone_server = launch_ros.actions.LifecycleNode(
        package='drone_rescue_coordination',
        executable='pheromone_server',
        name='pheromone_server',
        namespace='',
        parameters=[{
            'use_sim_time': True,
            'grid_width': 100,
            'grid_height': 100,
            'cell_resolution': 1.0,
            'origin_x': -50.0,
            'origin_y': -50.0,
            'decay_rate': 0.995,
            'update_rate': 2.0,
        }],
        output='screen',
    )

    # Single drone controller (lifecycle node)
    controller = launch_ros.actions.LifecycleNode(
        package='drone_rescue_coordination',
        executable='drone_controller',
        name='drone1_controller',
        namespace='',
        parameters=[{
            'drone_name': 'drone1',
            'use_sim_time': True,
            'takeoff_altitude': 10.0,
            'control_rate': 30.0,
        }],
        output='screen',
    )

    # Single surveyor (lifecycle node)
    surveyor = launch_ros.actions.LifecycleNode(
        package='drone_rescue_coordination',
        executable='surveyor',
        name='drone1_surveyor',
        namespace='',
        parameters=[{
            'drone_name': 'drone1',
            'use_sim_time': True,
            'update_rate': 2.0,
            'grid_width': 100,
            'grid_height': 100,
        }],
        output='screen',
    )

    # Lifecycle manager
    lifecycle_manager = launch_ros.actions.Node(
        package='drone_rescue_coordination',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{
            'use_sim_time': True,
            'drone_names': ['drone1'],
            'transition_timeout': 10.0,
            'auto_startup': False,  # Don't auto-start to avoid missing odom topics
        }],
        output='screen',
    )

    return (
        launch.LaunchDescription([
            pheromone_server,
            controller,
            surveyor,
            lifecycle_manager,
            # Wait for nodes to fully initialize before starting tests
            TimerAction(
                period=3.0,
                actions=[ReadyToTest()],
            ),
        ]),
        {
            'pheromone_server': pheromone_server,
            'controller': controller,
            'surveyor': surveyor,
            'lifecycle_manager': lifecycle_manager,
        }
    )


class TestSimTimeConsistency(unittest.TestCase):
    """Test that all nodes have use_sim_time parameter set correctly."""

    def test_all_nodes_have_sim_time(self):
        """
        Audit use_sim_time parameter on all launched nodes.

        Per user decision: Log ERROR and continue (don't fail immediately).
        At end, assert no mismatches found.
        """
        # Get list of all nodes
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(result.returncode, 0, "Failed to get node list")

        node_list = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
        self.assertGreater(len(node_list), 0, "No nodes found")

        print(f"\nFound {len(node_list)} nodes: {node_list}")

        # Expected nodes from our launch file
        expected_nodes = [
            '/pheromone_server',
            '/drone1_controller',
            '/drone1_surveyor',
            '/lifecycle_manager',
        ]

        # Track mismatches
        mismatches: List[Dict[str, str]] = []
        missing_param: List[str] = []

        for node in expected_nodes:
            if node not in node_list:
                print(f"WARNING: Expected node {node} not found in node list")
                continue

            # Retry parameter check with timeout (node might still be initializing)
            max_retries = 5
            retry_delay = 0.5
            param_value = None

            for attempt in range(max_retries):
                try:
                    result = subprocess.run(
                        ['ros2', 'param', 'get', node, 'use_sim_time'],
                        capture_output=True,
                        text=True,
                        timeout=5,
                    )

                    if result.returncode == 0:
                        # Parse output: "Boolean value is: true" or "Boolean value is: false"
                        output = result.stdout.strip()
                        if 'true' in output.lower():
                            param_value = True
                        elif 'false' in output.lower():
                            param_value = False
                        else:
                            print(f"  Unexpected param output for {node}: {output}")
                        break
                    else:
                        # Parameter might not exist yet
                        if attempt < max_retries - 1:
                            time.sleep(retry_delay)
                        else:
                            print(f"  Node {node} param check stderr: {result.stderr}")

                except subprocess.TimeoutExpired:
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                    else:
                        print(f"  Timeout getting param for {node}")

            # Check result
            if param_value is None:
                # Some nodes (like test infrastructure) might not have this parameter
                # Only flag as missing if it's one of our core nodes
                missing_param.append(node)
                print(f"  Node {node}: use_sim_time parameter not found")
            elif param_value is False:
                # ERROR: Parameter exists but set to false
                mismatches.append({
                    'node': node,
                    'expected': 'true',
                    'actual': 'false',
                })
                print(f"ERROR: Node {node} has use_sim_time=false (expected true)")
            else:
                print(f"  Node {node}: use_sim_time=true ✓")

        # Report summary
        print("\n=== Use Sim Time Audit Summary ===")
        print(f"Nodes checked: {len(expected_nodes)}")
        print(f"Correct (use_sim_time=true): {len(expected_nodes) - len(mismatches) - len(missing_param)}")
        print(f"Mismatches (use_sim_time=false): {len(mismatches)}")
        print(f"Missing parameter: {len(missing_param)}")

        if mismatches:
            print("\nERROR: The following nodes have incorrect use_sim_time setting:")
            for mismatch in mismatches:
                print(f"  - {mismatch['node']}: expected {mismatch['expected']}, got {mismatch['actual']}")

        # Final assertion: no mismatches allowed
        self.assertEqual(
            len(mismatches), 0,
            f"Found {len(mismatches)} node(s) with use_sim_time=false (expected true)"
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Check that nodes exited cleanly."""

    def test_exit_codes(self, proc_info):
        """Verify all processes exited with code 0."""
        launch_testing.asserts.assertExitCodes(proc_info)
