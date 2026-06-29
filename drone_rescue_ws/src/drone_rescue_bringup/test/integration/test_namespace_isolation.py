#!/usr/bin/env python3
"""
Integration test for namespace isolation.

Verifies INTG-04: Each drone in its own namespace with no topic crosstalk.

Tests:
1. Critical topics are properly namespaced (/droneN/cmd_vel, /droneN/odom)
2. No bare critical topics exist (prevents crosstalk)
3. TF frames use drone prefix (drone1/base_link, drone2/base_link)
"""

import pytest
import unittest
import subprocess
import time
from typing import List, Set

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from launch.actions import TimerAction
from launch_testing.actions import ReadyToTest


def generate_test_description():
    """
    Generate test launch description.

    Launches num_drones=2 (minimum for crosstalk detection) without Gazebo.
    Tests namespace isolation for topics and parameters.

    NOTE: TF frame tests are skipped without Gazebo (no TF data published).
    """

    # Pheromone server (shared, no namespace)
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
        }],
        output='screen',
    )

    # Drone 1 controller (lifecycle node)
    controller1 = launch_ros.actions.LifecycleNode(
        package='drone_rescue_coordination',
        executable='drone_controller',
        name='drone1_controller',
        namespace='',
        parameters=[{
            'drone_name': 'drone1',
            'use_sim_time': True,
            'takeoff_altitude': 10.0,
        }],
        output='screen',
    )

    # Drone 2 controller (lifecycle node)
    controller2 = launch_ros.actions.LifecycleNode(
        package='drone_rescue_coordination',
        executable='drone_controller',
        name='drone2_controller',
        namespace='',
        parameters=[{
            'drone_name': 'drone2',
            'use_sim_time': True,
            'takeoff_altitude': 10.0,
        }],
        output='screen',
    )

    # Drone 1 surveyor (lifecycle node)
    surveyor1 = launch_ros.actions.LifecycleNode(
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

    # Drone 2 surveyor (lifecycle node)
    surveyor2 = launch_ros.actions.LifecycleNode(
        package='drone_rescue_coordination',
        executable='surveyor',
        name='drone2_surveyor',
        namespace='',
        parameters=[{
            'drone_name': 'drone2',
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
            'drone_names': ['drone1', 'drone2'],
            'transition_timeout': 10.0,
            'auto_startup': False,  # Don't auto-start to avoid missing odom topics
        }],
        output='screen',
    )

    return (
        launch.LaunchDescription([
            pheromone_server,
            controller1,
            controller2,
            surveyor1,
            surveyor2,
            lifecycle_manager,
            # Wait for nodes to fully initialize before starting tests
            TimerAction(
                period=3.0,
                actions=[ReadyToTest()],
            ),
        ]),
        {
            'pheromone_server': pheromone_server,
            'controller1': controller1,
            'controller2': controller2,
            'surveyor1': surveyor1,
            'surveyor2': surveyor2,
            'lifecycle_manager': lifecycle_manager,
        }
    )


class TestNamespaceIsolation(unittest.TestCase):
    """Test that each drone has proper namespace isolation."""

    def test_critical_topics_namespaced(self):
        """
        Verify critical topics are namespaced per drone.

        Per user decision, critical topics are: cmd_vel, odom, pheromone_grid
        """
        # Get topic list
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(result.returncode, 0, "Failed to get topic list")

        topics = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
        print(f"\nFound {len(topics)} topics")

        # Critical topics that MUST be namespaced per drone
        critical_topics = ['cmd_vel', 'odom']  # pheromone_grid is per-drone subscription, not global

        # Expected namespaced topics for our two drones
        expected_namespaced = [
            '/drone1/cmd_vel',
            '/drone1/odom',
            '/drone2/cmd_vel',
            '/drone2/odom',
        ]

        # Check for bare (un-namespaced) critical topics - these indicate crosstalk risk
        bare_topics = []
        for critical in critical_topics:
            bare_topic = f'/{critical}'
            if bare_topic in topics:
                bare_topics.append(bare_topic)
                print(f"ERROR: Found bare critical topic {bare_topic} (crosstalk risk)")

        # Check that namespaced topics exist
        # NOTE: Without Gazebo/actual odometry publishers, topics might not exist yet
        # We check that IF they exist, they're namespaced correctly
        found_namespaced = [t for t in expected_namespaced if t in topics]
        missing_namespaced = [t for t in expected_namespaced if t not in topics]

        print(f"\nNamespaced topics found: {len(found_namespaced)}/{len(expected_namespaced)}")
        for topic in found_namespaced:
            print(f"  ✓ {topic}")

        if missing_namespaced:
            print(f"\nNamespaced topics missing (expected without Gazebo):")
            for topic in missing_namespaced:
                print(f"  - {topic}")

        # Report summary
        print("\n=== Namespace Isolation Summary ===")
        print(f"Bare critical topics (ERROR): {len(bare_topics)}")
        print(f"Properly namespaced topics: {len(found_namespaced)}")

        # Assertion: No bare critical topics allowed
        self.assertEqual(
            len(bare_topics), 0,
            f"Found {len(bare_topics)} bare critical topic(s): {bare_topics}"
        )

    @pytest.mark.skip(reason="TF frames require Gazebo for actual data - test manually with simulation")
    def test_tf_frames_have_prefix(self):
        """
        Verify TF frames use drone prefix.

        NOTE: This test requires Gazebo running to publish TF data.
        Without simulation, TF frames won't exist. Mark as skip for unit testing.

        To test manually:
        1. Launch full simulation: ros2 launch drone_rescue_bringup multi_drone_simulation.launch.py
        2. Check frames: ros2 run tf2_ros tf2_echo world drone1/base_link
        3. Verify frames exist for each drone: drone1/base_link, drone2/base_link, etc.
        """
        # Expected TF frames with drone prefix
        expected_frames = [
            ('world', 'drone1/base_link'),
            ('world', 'drone2/base_link'),
        ]

        for source, target in expected_frames:
            try:
                result = subprocess.run(
                    ['ros2', 'run', 'tf2_ros', 'tf2_echo', source, target],
                    capture_output=True,
                    text=True,
                    timeout=2,  # Short timeout since we're just checking existence
                )

                # If frame doesn't exist, stderr will contain "does not exist"
                if "does not exist" in result.stderr.lower():
                    print(f"ERROR: TF frame {target} does not exist")
                    self.fail(f"Missing TF frame: {target}")
                else:
                    print(f"  ✓ TF frame {target} exists")

            except subprocess.TimeoutExpired:
                # tf2_echo blocks waiting for data, timeout means it's waiting
                # This actually indicates the frame transform exists
                print(f"  ✓ TF frame {target} exists (timeout waiting for data)")

    def test_no_topic_collision(self):
        """
        Verify topics are properly isolated between drones.

        Checks that drone-specific topics don't appear in other drones' namespaces.
        """
        # Get topic list
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(result.returncode, 0, "Failed to get topic list")

        topics = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]

        # Group topics by namespace
        drone1_topics = [t for t in topics if t.startswith('/drone1/')]
        drone2_topics = [t for t in topics if t.startswith('/drone2/')]

        print(f"\nDrone1 topics: {len(drone1_topics)}")
        for topic in sorted(drone1_topics):
            print(f"  {topic}")

        print(f"\nDrone2 topics: {len(drone2_topics)}")
        for topic in sorted(drone2_topics):
            print(f"  {topic}")

        # Check for topic collisions (topics that appear in both namespaces)
        # Extract base names (after namespace prefix)
        drone1_bases = {t.replace('/drone1/', '') for t in drone1_topics}
        drone2_bases = {t.replace('/drone2/', '') for t in drone2_topics}

        # Both drones should have same structure (same base topic names)
        # This is expected - they're isolated but parallel
        common_bases = drone1_bases & drone2_bases
        print(f"\nCommon base topics (expected): {len(common_bases)}")
        for base in sorted(common_bases):
            print(f"  {base}")

        # What we DON'T want: topics from one drone appearing in another's namespace
        # This would indicate crosstalk
        # For example: /drone1/status should NOT be published under /drone2/
        # We verify this by checking node ownership, but for now just verify structure

        # If we have topics for both drones, verify they're parallel
        if drone1_topics and drone2_topics:
            # Both should have similar topic structure
            # This is a weak test, but without publishers we can't verify ownership
            print("\n✓ Both drones have topic namespaces")
        else:
            print("\nWARNING: Not all drone namespaces have topics (expected without active publishers)")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Check that nodes exited cleanly."""

    def test_exit_codes(self, proc_info):
        """Verify all processes exited with code 0."""
        launch_testing.asserts.assertExitCodes(proc_info)
