#!/usr/bin/env python3
"""
Flight Test Node

Demonstrates basic drone flight capabilities:
1. Enable motors
2. Takeoff to specified altitude
3. Navigate through waypoints
4. Return and land

This node is useful for testing the drone controller and verifying
the simulation setup before implementing more complex behaviors.
"""

import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry

from drone_rescue_msgs.msg import DroneStatus


class FlightTest(Node):
    """
    Flight test node for demonstrating drone capabilities.

    This node runs through a pre-defined flight sequence:
    1. Wait for odometry
    2. Enable motors
    3. Takeoff
    4. Fly through waypoints
    5. Return to start
    6. Land
    """

    def __init__(self):
        super().__init__('flight_test')

        # Declare parameters
        self.declare_parameter('drone_name', 'drone1')
        self.declare_parameter('takeoff_altitude', 10.0)
        self.declare_parameter('waypoint_tolerance', 1.0)
        self.declare_parameter('hover_time', 3.0)  # seconds at each waypoint

        # Get parameters
        self.drone_name = self.get_parameter('drone_name').value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.hover_time = self.get_parameter('hover_time').value

        # Define waypoints (relative to start position)
        # Format: (x_offset, y_offset, altitude)
        self.waypoints: List[Tuple[float, float, float]] = [
            (10.0, 0.0, 10.0),    # Forward 10m
            (10.0, 10.0, 15.0),   # Right 10m, up 5m
            (0.0, 10.0, 10.0),    # Back to y=10, down 5m
            (0.0, 0.0, 10.0),     # Return to start xy
        ]

        # State variables
        self.current_pose = None
        self.start_position = None
        self.drone_state = None
        self.current_waypoint_idx = 0
        self.state = 'INIT'  # INIT, ENABLING, TAKEOFF, NAVIGATING, LANDING, DONE
        self.state_start_time = None

        # QoS profile
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.drone_name}/odom',
            self.odom_callback,
            sensor_qos
        )

        self.status_sub = self.create_subscription(
            DroneStatus,
            f'/{self.drone_name}/status',
            self.status_callback,
            10
        )

        # Publishers
        self.enable_pub = self.create_publisher(
            Bool,
            f'/{self.drone_name}/enable',
            10
        )

        self.takeoff_pub = self.create_publisher(
            Float32,
            f'/{self.drone_name}/takeoff',
            10
        )

        self.target_pub = self.create_publisher(
            PoseStamped,
            f'/{self.drone_name}/target_pose',
            10
        )

        self.land_pub = self.create_publisher(
            Bool,
            f'/{self.drone_name}/land',
            10
        )

        # State machine timer
        self.timer = self.create_timer(0.5, self.state_machine)

        self.get_logger().info(
            f'Flight test initialized for {self.drone_name}'
        )
        self.get_logger().info(
            f'Waypoints: {len(self.waypoints)} points defined'
        )

    def odom_callback(self, msg: Odometry):
        """Store current position."""
        self.current_pose = msg.pose.pose

        # Capture start position on first callback
        if self.start_position is None:
            self.start_position = Point()
            self.start_position.x = self.current_pose.position.x
            self.start_position.y = self.current_pose.position.y
            self.start_position.z = self.current_pose.position.z
            self.get_logger().info(
                f'Start position captured: '
                f'({self.start_position.x:.2f}, {self.start_position.y:.2f})'
            )

    def status_callback(self, msg: DroneStatus):
        """Track drone controller state."""
        self.drone_state = msg.state

    def get_distance_to_point(self, target: Point) -> float:
        """Calculate 3D distance to a point."""
        if self.current_pose is None:
            return float('inf')

        dx = target.x - self.current_pose.position.x
        dy = target.y - self.current_pose.position.y
        dz = target.z - self.current_pose.position.z
        return (dx*dx + dy*dy + dz*dz) ** 0.5

    def send_waypoint(self, x: float, y: float, z: float):
        """Send a target position to the drone controller."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0  # No rotation
        self.target_pub.publish(msg)
        self.get_logger().info(f'Sent waypoint: ({x:.2f}, {y:.2f}, {z:.2f})')

    def state_machine(self):
        """Main state machine for flight test."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.state == 'INIT':
            # Wait for odometry and start position
            if self.start_position is not None:
                self.get_logger().info('Starting flight test sequence...')
                self.state = 'ENABLING'
                self.state_start_time = current_time

        elif self.state == 'ENABLING':
            # Enable motors
            msg = Bool()
            msg.data = True
            self.enable_pub.publish(msg)

            # Wait a moment for motors to enable
            if current_time - self.state_start_time > 1.0:
                self.get_logger().info('Motors enabled, initiating takeoff...')
                self.state = 'TAKEOFF'
                self.state_start_time = current_time

                # Send takeoff command
                takeoff_msg = Float32()
                takeoff_msg.data = self.takeoff_altitude
                self.takeoff_pub.publish(takeoff_msg)

        elif self.state == 'TAKEOFF':
            # Wait for drone to reach altitude
            if self.current_pose is not None:
                altitude = self.current_pose.position.z
                if altitude >= self.takeoff_altitude - 1.0:
                    self.get_logger().info(
                        f'Takeoff complete at altitude {altitude:.1f}m'
                    )
                    self.state = 'NAVIGATING'
                    self.current_waypoint_idx = 0
                    self.state_start_time = current_time

                    # Send first waypoint
                    wp = self.waypoints[0]
                    target_x = self.start_position.x + wp[0]
                    target_y = self.start_position.y + wp[1]
                    target_z = wp[2]
                    self.send_waypoint(target_x, target_y, target_z)

        elif self.state == 'NAVIGATING':
            # Check if at current waypoint
            wp = self.waypoints[self.current_waypoint_idx]
            target = Point()
            target.x = self.start_position.x + wp[0]
            target.y = self.start_position.y + wp[1]
            target.z = wp[2]

            distance = self.get_distance_to_point(target)

            if distance < self.waypoint_tolerance:
                # Hover at waypoint briefly
                if current_time - self.state_start_time > self.hover_time:
                    self.current_waypoint_idx += 1

                    if self.current_waypoint_idx >= len(self.waypoints):
                        # All waypoints complete
                        self.get_logger().info('All waypoints complete, landing...')
                        self.state = 'LANDING'
                        land_msg = Bool()
                        land_msg.data = True
                        self.land_pub.publish(land_msg)
                    else:
                        # Send next waypoint
                        wp = self.waypoints[self.current_waypoint_idx]
                        target_x = self.start_position.x + wp[0]
                        target_y = self.start_position.y + wp[1]
                        target_z = wp[2]
                        self.send_waypoint(target_x, target_y, target_z)
                        self.state_start_time = current_time
                        self.get_logger().info(
                            f'Waypoint {self.current_waypoint_idx}/{len(self.waypoints)}'
                        )

        elif self.state == 'LANDING':
            # Check if landed
            if self.current_pose is not None:
                if self.current_pose.position.z < 0.3:
                    self.get_logger().info('Landing complete!')
                    self.state = 'DONE'

                    # Disable motors
                    msg = Bool()
                    msg.data = False
                    self.enable_pub.publish(msg)

        elif self.state == 'DONE':
            self.get_logger().info('Flight test completed successfully!')
            self.timer.cancel()
            # Keep node alive for a moment to ensure messages are sent
            self.create_timer(2.0, lambda: rclpy.shutdown())


def main(args=None):
    rclpy.init(args=args)
    node = FlightTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
