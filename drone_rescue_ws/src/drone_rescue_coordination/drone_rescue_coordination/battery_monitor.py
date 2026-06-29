#!/usr/bin/env python3
"""
Battery Monitor Node

Simulates battery drain for a drone based on:
- Base drain rate (hovering)
- Movement speed (higher speed = more drain)
- Weather conditions (wind increases drain)
- Payload/sensor usage

Publishes battery status and triggers return-to-base when low.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from drone_rescue_msgs.msg import DroneStatus, WeatherState


class BatteryMonitor(LifecycleNode):
    """
    Battery monitoring and simulation node.

    Subscribes to:
        - /droneN/odom: Current velocity (affects drain)
        - /droneN/cmd_vel: Command velocity (affects drain)
        - /environment/weather: Weather conditions (affects drain)

    Publishes:
        - /droneN/battery_level: Current battery level (0.0-1.0)
        - /droneN/battery_low: True when battery below threshold
        - /droneN/land: Triggers landing on critical battery
    """

    def __init__(self):
        super().__init__('battery_monitor')

        # Declare parameters
        self.declare_parameter('drone_name', 'drone1')
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('initial_level', 1.0)  # 100%
        self.declare_parameter('base_drain_rate', 0.0005)  # per second at hover
        self.declare_parameter('movement_drain_factor', 0.0002)  # additional per m/s
        self.declare_parameter('low_battery_threshold', 0.2)  # 20%
        self.declare_parameter('critical_battery_threshold', 0.1)  # 10%
        self.declare_parameter('wind_drain_multiplier_base', 1.0)

        # Get parameters
        self.drone_name = self.get_parameter('drone_name').value
        self.update_rate = self.get_parameter('update_rate').value
        self.battery_level = self.get_parameter('initial_level').value
        self.base_drain_rate = self.get_parameter('base_drain_rate').value
        self.movement_drain_factor = self.get_parameter('movement_drain_factor').value
        self.low_battery_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_battery_threshold = self.get_parameter('critical_battery_threshold').value

        # State variables
        self.current_velocity = Twist()
        self.weather_drain_multiplier = 1.0
        self.motors_enabled = False
        self.low_battery_triggered = False
        self.critical_battery_triggered = False
        self.last_update_time = None
        self.survey_active = False  # No drain until survey starts

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

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            f'/{self.drone_name}/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.enable_sub = self.create_subscription(
            Bool,
            f'/{self.drone_name}/enable',
            self.enable_callback,
            10
        )

        self.weather_sub = self.create_subscription(
            WeatherState,
            '/environment/weather',
            self.weather_callback,
            10
        )

        # Subscribe to survey start to enable battery drain
        self.survey_start_sub = self.create_subscription(
            Bool,
            '/survey/start',
            self.survey_start_callback,
            10
        )

        # Publishers
        self.battery_pub = self.create_publisher(
            Float32,
            f'/{self.drone_name}/battery_level',
            10
        )

        self.battery_low_pub = self.create_publisher(
            Bool,
            f'/{self.drone_name}/battery_low',
            10
        )

        self.land_pub = self.create_publisher(
            Bool,
            f'/{self.drone_name}/land',
            10
        )

        # Update timer
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_battery
        )

        self.get_logger().info(
            f'Battery monitor initialized for {self.drone_name} '
            f'(initial level: {self.battery_level*100:.0f}%)'
        )

    # LifecycleNode protocol callbacks. Battery drain callbacks remain
    # free-running for now; lifecycle_manager can advertise change_state
    # for SAFE-mode coordination once the mission orchestrator wires it in.
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def odom_callback(self, msg: Odometry):
        """Update velocity from odometry."""
        self.current_velocity = msg.twist.twist

    def cmd_vel_callback(self, msg: Twist):
        """Track commanded velocity."""
        # Could use this for predictive drain calculation
        pass

    def enable_callback(self, msg: Bool):
        """Track motor state."""
        self.motors_enabled = msg.data
        if self.motors_enabled:
            self.get_logger().debug('Motors enabled - battery drain active')

    def weather_callback(self, msg: WeatherState):
        """Update weather-based drain multiplier."""
        self.weather_drain_multiplier = msg.battery_drain_multiplier
        self.get_logger().debug(
            f'Weather drain multiplier updated: {self.weather_drain_multiplier:.2f}'
        )

    def survey_start_callback(self, msg: Bool):
        """Enable battery drain when survey starts."""
        if msg.data and not self.survey_active:
            self.survey_active = True
            self.get_logger().info('Survey started - battery drain activated')

    def calculate_drain_rate(self) -> float:
        """
        Calculate current battery drain rate based on conditions.

        Returns:
            Drain rate in battery units per second (0.0-1.0 scale)
        """
        # No drain before survey starts (realistic: motors not running)
        if not self.survey_active:
            return 0.0

        if not self.motors_enabled:
            # Minimal drain when motors off (electronics only)
            return self.base_drain_rate * 0.1

        # Base drain for hovering
        drain = self.base_drain_rate

        # Additional drain from movement
        speed = math.sqrt(
            self.current_velocity.linear.x ** 2 +
            self.current_velocity.linear.y ** 2 +
            self.current_velocity.linear.z ** 2
        )
        drain += speed * self.movement_drain_factor

        # Additional drain from rotation
        yaw_rate = abs(self.current_velocity.angular.z)
        drain += yaw_rate * self.movement_drain_factor * 0.5

        # Weather multiplier
        drain *= self.weather_drain_multiplier

        return drain

    def update_battery(self):
        """Update battery level based on drain rate."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.last_update_time is None:
            self.last_update_time = current_time
            return

        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Calculate and apply drain
        drain_rate = self.calculate_drain_rate()
        self.battery_level -= drain_rate * dt
        self.battery_level = max(0.0, min(1.0, self.battery_level))

        # Publish battery level
        level_msg = Float32()
        level_msg.data = self.battery_level
        self.battery_pub.publish(level_msg)

        # Check thresholds
        self.check_battery_thresholds()

    def check_battery_thresholds(self):
        """Check battery level and trigger warnings/actions."""
        # Low battery warning
        if (self.battery_level <= self.low_battery_threshold and
                not self.low_battery_triggered):
            self.low_battery_triggered = True
            self.get_logger().warn(
                f'LOW BATTERY: {self.battery_level*100:.1f}% - '
                f'Consider returning to base'
            )
            low_msg = Bool()
            low_msg.data = True
            self.battery_low_pub.publish(low_msg)

        # Critical battery - force landing
        if (self.battery_level <= self.critical_battery_threshold and
                not self.critical_battery_triggered):
            self.critical_battery_triggered = True
            self.get_logger().error(
                f'CRITICAL BATTERY: {self.battery_level*100:.1f}% - '
                f'Initiating emergency landing!'
            )
            land_msg = Bool()
            land_msg.data = True
            self.land_pub.publish(land_msg)

        # Reset triggers if battery recovered (e.g., simulation reset)
        if self.battery_level > self.low_battery_threshold:
            self.low_battery_triggered = False
            self.critical_battery_triggered = False

    def reset_battery(self, level: float = 1.0):
        """Reset battery to specified level (for testing/simulation reset)."""
        self.battery_level = max(0.0, min(1.0, level))
        self.low_battery_triggered = False
        self.critical_battery_triggered = False
        self.get_logger().info(f'Battery reset to {self.battery_level*100:.0f}%')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
