#!/usr/bin/env python3
"""
Camera Director Node

Orchestrates cinematic camera shots for the drone rescue demo.
Uses Gazebo GUI services via gz CLI to control camera positioning.

Event-driven shot transitions based on:
- Coverage milestones (25%, 75%)
- Victim detection events
- Drone state changes (deployment)
"""

import subprocess
import time
from pathlib import Path

from drone_rescue_coordination.lib.domain.fleet import default_drone_names_list
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import yaml

from drone_rescue_msgs.msg import CoverageMetrics, VictimDetection
from rosgraph_msgs.msg import Clock


class CameraDirector(Node):
    """
    Orchestrates camera shots for storytelling demo.

    Uses subprocess calls to gz CLI for camera control (avoids DDS overhead).
    Shot sequence driven by YAML config and simulation events.
    """

    def __init__(self):
        super().__init__('camera_director')

        # Parameters
        self.declare_parameter('shot_config', '')
        self.declare_parameter('drone_names', default_drone_names_list())
        self.declare_parameter('enable_camera_control', True)
        self.declare_parameter('demo_timeout_sim_seconds', 480.0)  # 8 minutes sim time
        self.declare_parameter('coverage_completion_threshold', 60.0)  # Lower from 75%
        self.declare_parameter('coverage_survey_threshold', 20.0)  # Lower from 25%

        self.shot_config_path = self.get_parameter('shot_config').value
        self.drone_names = self.get_parameter('drone_names').value
        self.enabled = self.get_parameter('enable_camera_control').value
        self.demo_timeout_sim_seconds = self.get_parameter('demo_timeout_sim_seconds').value
        self.coverage_completion_threshold = self.get_parameter('coverage_completion_threshold').value
        self.coverage_survey_threshold = self.get_parameter('coverage_survey_threshold').value

        if not self.enabled:
            self.get_logger().info('Camera control disabled')
            return

        # Load shot sequence
        self.shots = []
        self.current_shot_index = -1
        self.executed_shots = set()  # Track which shots have been executed
        self.deployment_triggered = False

        if self.shot_config_path:
            self._load_shots()
        else:
            self.get_logger().warn('No shot config provided, camera director idle')
            return

        # State tracking
        self.coverage_percentage = 0.0
        self.drones_surveying = 0
        self.last_detection_time = None
        self.detection_shot_active = False
        self.pre_detection_shot = None
        self.demo_start_sim_time = None
        self.current_sim_time = None

        # Callback group for parallel subscription processing
        callback_group = ReentrantCallbackGroup()

        # Subscribe to simulation events
        self.coverage_sub = self.create_subscription(
            CoverageMetrics,
            '/coverage/metrics',
            self._coverage_callback,
            10,
            callback_group=callback_group
        )

        self.victim_sub = self.create_subscription(
            VictimDetection,
            '/victims/detections',
            self._victim_callback,
            10,
            callback_group=callback_group
        )

        # Subscribe to /clock for sim time tracking
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self._clock_callback,
            10,
            callback_group=callback_group
        )

        # Timer for checking shot transitions and detection timeout
        self.create_timer(2.0, self._check_transitions)

        self.get_logger().info(f'Camera director initialized with {len(self.shots)} shots')

        # Execute establishing shot on startup
        self._execute_shot_by_trigger('start')

    def _load_shots(self):
        """Load shot sequence from YAML configuration."""
        try:
            with open(self.shot_config_path, 'r') as f:
                config = yaml.safe_load(f)
                self.shots = config.get('shots', [])
                self.get_logger().info(f'Loaded {len(self.shots)} camera shots from {self.shot_config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load shot config: {e}')
            self.shots = []

    def _coverage_callback(self, msg: CoverageMetrics):
        """Track coverage progress for milestone-based transitions."""
        self.coverage_percentage = msg.percentage_covered
        self.drones_surveying = msg.drones_surveying

    def _victim_callback(self, msg: VictimDetection):
        """Handle victim detection event - trigger detection shot."""
        if 'detection' in self.executed_shots:
            return  # Already executed detection shot

        self.get_logger().info(f'Victim detected by {msg.drone_name}, switching to detection shot')

        # Save current shot to return to
        if not self.detection_shot_active:
            self.pre_detection_shot = self.current_shot_index

        # Execute detection shot (follow the drone that detected)
        self.detection_shot_active = True
        self.last_detection_time = time.time()

        # Find detection shot
        for i, shot in enumerate(self.shots):
            if shot['name'] == 'detection':
                # Update entity to follow the detecting drone
                if shot.get('entity') == 'auto':
                    shot['entity'] = msg.drone_name
                self._execute_shot(i)
                break

    def _clock_callback(self, msg: Clock):
        """Track simulation time for timeout-based completion."""
        sim_time_seconds = msg.clock.sec + msg.clock.nanosec / 1e9
        self.current_sim_time = sim_time_seconds

        # Record start time on first clock message
        if self.demo_start_sim_time is None:
            self.demo_start_sim_time = sim_time_seconds
            self.get_logger().info(f'Demo start time recorded: {sim_time_seconds:.2f}s sim time')

    def _check_transitions(self):
        """Check if shot transitions should occur based on current state."""
        if not self.enabled or not self.shots:
            return

        # Check timeout-based forced completion (uses sim time)
        if self.demo_start_sim_time is not None and self.current_sim_time is not None:
            elapsed_sim_time = self.current_sim_time - self.demo_start_sim_time
            if elapsed_sim_time >= self.demo_timeout_sim_seconds and 'completion' not in self.executed_shots:
                self.get_logger().warn(
                    f'Demo timeout reached ({elapsed_sim_time:.1f}s sim time), forcing completion shot'
                )
                self._execute_shot_by_trigger('coverage_75')  # Trigger completion shot
                # Log completion
                self.get_logger().info(
                    f'Demo complete: {len(self.executed_shots)} shots executed, coverage {self.coverage_percentage:.1f}%'
                )

        # Check if detection shot should end
        if self.detection_shot_active and self.last_detection_time:
            detection_duration = 20.0  # from YAML
            if time.time() - self.last_detection_time > detection_duration:
                self.get_logger().info('Detection shot complete, returning to previous shot')
                self.detection_shot_active = False
                # Return to pre-detection shot if valid
                if self.pre_detection_shot is not None and self.pre_detection_shot >= 0:
                    self._execute_shot(self.pre_detection_shot)
                self.last_detection_time = None

        # Deployment trigger: first time drones start surveying
        if self.drones_surveying > 0 and not self.deployment_triggered:
            self.deployment_triggered = True
            self._execute_shot_by_trigger('event')

        # Coverage milestone triggers (use configurable thresholds)
        if self.coverage_percentage >= self.coverage_survey_threshold:
            self._execute_shot_by_trigger('coverage_25')

        if self.coverage_percentage >= self.coverage_completion_threshold:
            self._execute_shot_by_trigger('coverage_75')
            # Log completion when triggered by coverage
            if 'completion' in self.executed_shots:
                self.get_logger().info(
                    f'Demo complete: {len(self.executed_shots)} shots executed, coverage {self.coverage_percentage:.1f}%'
                )

    def _execute_shot_by_trigger(self, trigger_name: str):
        """Execute the first shot matching the given trigger (if not already executed)."""
        for i, shot in enumerate(self.shots):
            if shot.get('trigger') == trigger_name and shot['name'] not in self.executed_shots:
                self._execute_shot(i)
                break

    def _execute_shot(self, shot_index: int):
        """Execute a specific shot from the sequence."""
        if shot_index < 0 or shot_index >= len(self.shots):
            return

        shot = self.shots[shot_index]
        shot_name = shot['name']

        # Skip if already executed (unless it's a detection shot)
        if shot_name in self.executed_shots and shot_name != 'detection':
            return

        self.current_shot_index = shot_index
        self.executed_shots.add(shot_name)

        self.get_logger().info(f'Camera: {shot_name} shot - {shot.get("description", "")}')

        shot_type = shot.get('type')

        if shot_type == 'move_to_pose':
            pose = shot.get('pose', {})
            self._move_to_pose(
                pose.get('x', 0.0),
                pose.get('y', 0.0),
                pose.get('z', 10.0),
                pose.get('roll', 0.0),
                pose.get('pitch', 0.0),
                pose.get('yaw', 0.0)
            )

        elif shot_type == 'follow':
            entity = shot.get('entity', 'drone1')
            offset = shot.get('offset', {})
            self._follow_entity(
                entity,
                offset.get('x', 0.0),
                offset.get('y', 0.0),
                offset.get('z', 5.0)
            )

    def _move_to_pose(self, x: float, y: float, z: float,
                      roll: float, pitch: float, yaw: float):
        """
        Move camera to specific pose via gz CLI service call.

        Uses /gui/move_to/pose service with gz.msgs.GUICamera message.
        """
        if not self.enabled:
            return

        # Construct GUICamera message. GUICamera uses pose (position +
        # orientation in quaternion); euler angles are passed and Gazebo
        # handles the conversion.
        msg = f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}}}'

        try:
            result = subprocess.run(
                ['gz', 'service', '-s', '/gui/move_to/pose',
                 '--reqtype', 'gz.msgs.GUICamera',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000',
                 '--req', msg],
                capture_output=True, text=True, timeout=5
            )

            if result.returncode != 0:
                self.get_logger().warn(f'Camera move failed: {result.stderr}')
            else:
                self.get_logger().debug(f'Camera moved to ({x}, {y}, {z})')

        except subprocess.TimeoutExpired:
            self.get_logger().warn('Camera service call timed out')
        except FileNotFoundError:
            self.get_logger().error('gz CLI not found - is Gazebo Harmonic installed?')
        except Exception as e:
            self.get_logger().error(f'Camera control error: {e}')

    def _follow_entity(self, entity_name: str, offset_x: float = 0.0,
                       offset_y: float = 0.0, offset_z: float = 5.0):
        """
        Follow entity via gz CLI service calls.

        Uses /gui/follow and /gui/follow/offset services.
        """
        if not self.enabled:
            return

        try:
            # First, set the entity to follow
            result = subprocess.run(
                ['gz', 'service', '-s', '/gui/follow',
                 '--reqtype', 'gz.msgs.StringMsg',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000',
                 '--req', f'data: "{entity_name}"'],
                capture_output=True, text=True, timeout=5
            )

            if result.returncode != 0:
                self.get_logger().warn(f'Camera follow failed: {result.stderr}')
                return

            # Then set the follow offset
            result = subprocess.run(
                ['gz', 'service', '-s', '/gui/follow/offset',
                 '--reqtype', 'gz.msgs.Vector3d',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000',
                 '--req', f'x: {offset_x}, y: {offset_y}, z: {offset_z}'],
                capture_output=True, text=True, timeout=5
            )

            if result.returncode != 0:
                self.get_logger().warn(f'Camera offset failed: {result.stderr}')
            else:
                self.get_logger().debug(f'Camera following {entity_name} with offset ({offset_x}, {offset_y}, {offset_z})')

        except subprocess.TimeoutExpired:
            self.get_logger().warn('Camera follow service call timed out')
        except FileNotFoundError:
            self.get_logger().error('gz CLI not found - is Gazebo Harmonic installed?')
        except Exception as e:
            self.get_logger().error(f'Camera follow error: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = CameraDirector()

    # Use multi-threaded executor for parallel callback processing
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
