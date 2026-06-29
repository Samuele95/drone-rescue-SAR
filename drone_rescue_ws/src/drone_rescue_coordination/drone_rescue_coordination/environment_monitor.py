#!/usr/bin/env python3
"""
Environment Monitor Node

Monitors and publishes environmental conditions including:
- Weather state machine (Clear -> Windy -> Storm -> Clear)
- Wind speed and direction
- Environmental hazard alerts
- Gazebo integration for wind physics and weather particle effects

The weather system affects:
- Drone battery drain rate
- Flight stability
- Sensor performance (LiDAR range in storms)
- Gazebo simulation physics (wind forces on drones)
- Visual effects (rain, fog, debris particles)
"""

import math
import random
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import ParameterValue, ParameterType

from drone_rescue_msgs.msg import WeatherState

# Gazebo bridge messages (for particle emitter control)
try:
    from ros_gz_interfaces.msg import ParamVec
    from rcl_interfaces.msg import Parameter
    HAS_GZ_BRIDGE = True
except ImportError:
    HAS_GZ_BRIDGE = False


class WeatherCondition(Enum):
    """Weather condition states."""
    CLEAR = 0
    WINDY = 1
    STORM = 2


class EnvironmentMonitor(LifecycleNode):
    """
    Environment monitoring node with weather state machine.

    Publishes:
        /environment/weather: WeatherState - Current weather conditions
        /environment/wind: Vector3 - Wind velocity vector
        /environment/visibility: Float32 - Visibility multiplier (0-1)
        /environment/alert: String - Environmental alerts
    """

    def __init__(self):
        super().__init__('environment_monitor')

        # Declare parameters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('weather_change_interval', 120.0)  # seconds
        self.declare_parameter('enable_weather_changes', True)
        self.declare_parameter('initial_weather', 'clear')

        # Wind parameters for each weather state
        self.declare_parameter('clear_wind_speed', 2.0)  # m/s
        self.declare_parameter('windy_wind_speed', 8.0)  # m/s
        self.declare_parameter('storm_wind_speed', 15.0)  # m/s

        # Master RNG seed for reproducible runs. Forwarded by the launch
        # file from the top-level `seed` arg. Offset 13 so weather/wind
        # noise is uncorrelated with sensor_degradation (offset 17),
        # mission_manager auction (offset 7919), and the coverage
        # planner (no offset).
        self.declare_parameter('seed', 0)

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.weather_change_interval = self.get_parameter('weather_change_interval').value
        self.enable_weather_changes = self.get_parameter('enable_weather_changes').value
        initial_weather = self.get_parameter('initial_weather').value

        self.clear_wind_speed = self.get_parameter('clear_wind_speed').value
        self.windy_wind_speed = self.get_parameter('windy_wind_speed').value
        self.storm_wind_speed = self.get_parameter('storm_wind_speed').value

        seed = int(self.get_parameter('seed').value) + 13
        # Per-instance Random so we don't pollute the module-level RNG
        # other code might rely on. All `random.foo()` calls in this node
        # are replaced with `self._rng.foo()`.
        self._rng = random.Random(seed)

        # Initialize weather state
        self.weather_state = self._parse_weather(initial_weather)
        self.last_weather_change = self.get_clock().now()
        self.weather_duration = self.weather_change_interval

        # Wind state
        self.wind_direction = self._rng.uniform(0, 2 * math.pi)  # radians
        self.wind_speed = self._get_base_wind_speed()

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Publishers
        self.weather_pub = self.create_publisher(
            WeatherState,
            '/environment/weather',
            qos
        )

        self.wind_pub = self.create_publisher(
            Vector3,
            '/environment/wind',
            10
        )

        self.visibility_pub = self.create_publisher(
            Float32,
            '/environment/visibility',
            10
        )

        self.alert_pub = self.create_publisher(
            String,
            '/environment/alert',
            10
        )

        # Gazebo world control publishers (for wind and particles)
        self.declare_parameter('enable_gazebo_control', True)
        self.declare_parameter('world_name', 'earthquake_zone')
        self.enable_gazebo_control = self.get_parameter('enable_gazebo_control').value
        self.world_name = self.get_parameter('world_name').value

        if self.enable_gazebo_control:
            # Publisher for Gazebo wind control
            self.gz_wind_pub = self.create_publisher(
                Vector3,
                f'/world/{self.world_name}/wind',
                10
            )

            # Publishers for particle emitter control (via Gazebo services)
            # These control rain, fog, and debris particles
            self.rain_active = False
            self.fog_active = False
            self.debris_active = False

            self.get_logger().info('Gazebo weather control enabled')

        # Timers
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_callback
        )

        self.weather_timer = self.create_timer(
            5.0,  # Check weather transitions every 5 seconds
            self.weather_transition_callback
        )

        self.get_logger().info(
            f'Environment monitor started. Initial weather: {self.weather_state.name}'
        )

        # Publish initial state
        self._publish_weather_state()

    # LifecycleNode protocol callbacks. The class advertises
    # change_state / get_state services so lifecycle_manager can stop
    # weather noise during SAFE-mode recovery.
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

    def _parse_weather(self, weather_str: str) -> WeatherCondition:
        """Parse weather string to enum."""
        weather_map = {
            'clear': WeatherCondition.CLEAR,
            'windy': WeatherCondition.WINDY,
            'storm': WeatherCondition.STORM
        }
        return weather_map.get(weather_str.lower(), WeatherCondition.CLEAR)

    def _get_base_wind_speed(self) -> float:
        """Get base wind speed for current weather state."""
        if self.weather_state == WeatherCondition.CLEAR:
            return self.clear_wind_speed
        elif self.weather_state == WeatherCondition.WINDY:
            return self.windy_wind_speed
        else:  # STORM
            return self.storm_wind_speed

    def _get_visibility(self) -> float:
        """Get visibility multiplier for current weather."""
        if self.weather_state == WeatherCondition.CLEAR:
            return 1.0
        elif self.weather_state == WeatherCondition.WINDY:
            return 0.9
        else:  # STORM
            return 0.5

    def _get_battery_drain_multiplier(self) -> float:
        """Get battery drain multiplier for current weather."""
        if self.weather_state == WeatherCondition.CLEAR:
            return 1.0
        elif self.weather_state == WeatherCondition.WINDY:
            return 1.2
        else:  # STORM
            return 1.5

    def _get_lidar_range_multiplier(self) -> float:
        """Get LiDAR range multiplier for current weather."""
        if self.weather_state == WeatherCondition.CLEAR:
            return 1.0
        elif self.weather_state == WeatherCondition.WINDY:
            return 1.0
        else:  # STORM (rain/debris affects LiDAR)
            return 0.5

    def update_callback(self):
        """Main update loop - publish current conditions."""
        # Add noise to wind
        noise_speed = self._rng.gauss(0, 0.5)
        noise_direction = self._rng.gauss(0, 0.1)

        current_speed = max(0, self.wind_speed + noise_speed)
        current_direction = self.wind_direction + noise_direction

        # Calculate wind vector
        wind_msg = Vector3()
        wind_msg.x = current_speed * math.cos(current_direction)
        wind_msg.y = current_speed * math.sin(current_direction)
        wind_msg.z = self._rng.gauss(0, 0.2)  # Small vertical component

        self.wind_pub.publish(wind_msg)

        # Publish visibility
        visibility_msg = Float32()
        visibility_msg.data = self._get_visibility()
        self.visibility_pub.publish(visibility_msg)

    def weather_transition_callback(self):
        """Check and handle weather state transitions."""
        if not self.enable_weather_changes:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_weather_change).nanoseconds / 1e9

        if elapsed >= self.weather_duration:
            # Transition to next weather state
            self._transition_weather()

    def _transition_weather(self):
        """Transition to next weather state."""
        old_state = self.weather_state

        # State machine transitions
        # Clear -> Windy (70%) or Stay Clear (30%)
        # Windy -> Storm (40%) or Clear (60%)
        # Storm -> Windy (80%) or Clear (20%)

        if self.weather_state == WeatherCondition.CLEAR:
            if self._rng.random() < 0.7:
                self.weather_state = WeatherCondition.WINDY
                self.weather_duration = self._rng.uniform(60, 180)

        elif self.weather_state == WeatherCondition.WINDY:
            if self._rng.random() < 0.4:
                self.weather_state = WeatherCondition.STORM
                self.weather_duration = self._rng.uniform(30, 90)
            else:
                self.weather_state = WeatherCondition.CLEAR
                self.weather_duration = self._rng.uniform(90, 240)

        else:  # STORM
            if self._rng.random() < 0.8:
                self.weather_state = WeatherCondition.WINDY
                self.weather_duration = self._rng.uniform(60, 120)
            else:
                self.weather_state = WeatherCondition.CLEAR
                self.weather_duration = self._rng.uniform(120, 300)

        if old_state != self.weather_state:
            self.get_logger().info(
                f'Weather changed: {old_state.name} -> {self.weather_state.name}'
            )

            # Update wind parameters
            self.wind_speed = self._get_base_wind_speed()
            self.wind_direction = self._rng.uniform(0, 2 * math.pi)

            # Publish alert
            alert_msg = String()
            alert_msg.data = f'WEATHER_CHANGE: {self.weather_state.name}'
            self.alert_pub.publish(alert_msg)

            # Publish updated weather state
            self._publish_weather_state()

        self.last_weather_change = self.get_clock().now()

    def _publish_weather_state(self):
        """Publish current weather state."""
        from geometry_msgs.msg import Vector3 as Vec3

        msg = WeatherState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.condition = self.weather_state.value
        msg.wind_speed = self.wind_speed
        msg.wind_direction = self.wind_direction

        # Calculate wind velocity vector
        msg.wind_velocity = Vec3()
        msg.wind_velocity.x = self.wind_speed * math.cos(self.wind_direction)
        msg.wind_velocity.y = self.wind_speed * math.sin(self.wind_direction)
        msg.wind_velocity.z = 0.0

        msg.visibility = self._get_visibility()
        msg.battery_drain_multiplier = self._get_battery_drain_multiplier()
        msg.lidar_range_multiplier = self._get_lidar_range_multiplier()

        self.weather_pub.publish(msg)

        # Update Gazebo weather effects
        if self.enable_gazebo_control:
            self._update_gazebo_weather()

    def _update_gazebo_weather(self):
        """Update Gazebo simulation with current weather conditions."""
        # Update wind in Gazebo
        self._update_gazebo_wind()

        # Update particle effects based on weather state
        self._update_particle_effects()

    def _update_gazebo_wind(self):
        """Send wind velocity to Gazebo world."""
        if not hasattr(self, 'gz_wind_pub'):
            return

        # Create wind vector message
        wind_msg = Vector3()
        wind_msg.x = self.wind_speed * math.cos(self.wind_direction)
        wind_msg.y = self.wind_speed * math.sin(self.wind_direction)
        wind_msg.z = 0.0

        self.gz_wind_pub.publish(wind_msg)

    def _update_particle_effects(self):
        """Control weather particle emitters based on current state."""
        # Determine which particle effects should be active
        new_rain = self.weather_state == WeatherCondition.STORM
        new_fog = self.weather_state in [WeatherCondition.WINDY, WeatherCondition.STORM]
        new_debris = self.weather_state == WeatherCondition.STORM

        # Log changes
        if new_rain != self.rain_active:
            self.rain_active = new_rain
            status = "enabled" if new_rain else "disabled"
            self.get_logger().info(f'Rain particles {status}')

        if new_fog != self.fog_active:
            self.fog_active = new_fog
            status = "enabled" if new_fog else "disabled"
            self.get_logger().info(f'Fog particles {status}')

        if new_debris != self.debris_active:
            self.debris_active = new_debris
            status = "enabled" if new_debris else "disabled"
            self.get_logger().info(f'Debris particles {status}')

    def get_weather_description(self) -> str:
        """Get human-readable weather description."""
        descriptions = {
            WeatherCondition.CLEAR: "Clear skies, light winds",
            WeatherCondition.WINDY: "Windy conditions, reduced stability",
            WeatherCondition.STORM: "Storm warning! Heavy winds, rain, reduced visibility"
        }
        return descriptions.get(self.weather_state, "Unknown")


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
