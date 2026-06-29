#!/usr/bin/env python3
"""
Drone Controller Node. 3T Architecture: Behavioural Layer (L1):
PID flight controller, actuator tier (Marcelletti slides p. 37).

Tightest L1 loop in the system: a 50 Hz PID setpoint follower with no
world model and no plan-progress state. Pure stimulus-to-actuator
condition-to-action loop per the slides' Layer-1 definition.

Provides flight control for a single drone including:
- Position-based navigation (go to waypoint)
- Velocity control with PID stabilization
- State machine for flight phases (takeoff, hover, navigate, land)
- Integration hooks for pheromone-based navigation (Phase 4)
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, Duration
from rclpy.event_handler import SubscriptionEventCallbacks

import diagnostic_updater
import diagnostic_msgs.msg

from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Imu

from drone_rescue_msgs.msg import DroneStatus, WeatherState


# OdomRetryManager lifted to
# `lib/retry.BackoffRetry`. The shim alias preserves every existing
# `self.odom_retry_manager.attempt_retry(...)` / `.reset()` /
# `.retry_timer` / `.in_retry` call site and attribute read; the
# constructor signature (`node`, `max_retries`, `base_delay`) maps
# directly to BackoffRetry's `(host, max_retries, base_delay)`.
from drone_rescue_coordination.lib.retry import BackoffRetry as OdomRetryManager


# PIDController + DroneState moved to lib/ so they're
# unit-testable without rclpy. Imported as shims so the existing
# in-file references continue to resolve.
from drone_rescue_coordination.lib.domain.drone_state import DroneState
from drone_rescue_coordination.lib.domain.wind import wind_velocity_offset

# States in which the drone is airborne and therefore subject to the software
# wind disturbance. IDLE (grounded) and EMERGENCY (motors cut, returns
# before the wind block) are excluded.
_AIRBORNE_STATES = (
    DroneState.TAKEOFF, DroneState.SURVEYING, DroneState.HOVER,
    DroneState.NAVIGATING, DroneState.RETURNING, DroneState.LANDING,
)
from drone_rescue_coordination.lib.pid import PIDController
from drone_rescue_coordination.lib.composition import (
    bind_composition, resolve_clock,
)


class DroneController(LifecycleNode):
    """
    Main drone controller node.

    Subscribes to:
        - /droneN/odom: Current position and velocity
        - /droneN/imu: Orientation data
        - /droneN/target_position: Commanded target position
        - /droneN/enable: Motor enable flag

    Publishes:
        - /droneN/cmd_vel: Velocity commands to Gazebo
        - /droneN/status: Drone status message
    """

    # composition kwarg; falls back to lazy adapter construction when None.
    def __init__(self, *, composition=None):
        super().__init__('drone_controller')
        self._composition = composition

        # Declare parameters
        self.declare_parameter('drone_name', 'drone1')
        self.declare_parameter('control_rate', 50.0)  # Hz
        self.declare_parameter('takeoff_altitude', 10.0)  # meters
        self.declare_parameter('position_tolerance', 0.5)  # meters
        self.declare_parameter('landing_speed', 0.5)  # m/s
        self.declare_parameter('max_horizontal_speed', 5.0)  # m/s
        self.declare_parameter('max_vertical_speed', 3.0)  # m/s
        self.declare_parameter('max_yaw_rate', 1.0)  # rad/s
        # Software wind model. environment_monitor publishes the wind
        # on /environment/weather (WeatherState) and /environment/wind, but the
        # Gazebo wind topic is unbridged so the physics engine never applies it.
        # The control loop therefore models the wind itself: it adds
        # (disturbance_gain - compensation_gain) * wind to the world-frame
        # velocity command (see lib/domain/wind.wind_velocity_offset).
        #   disturbance_gain (default 1.0): the wind pushes the drone.
        #   compensation_gain (default 1.0): station-keeping that counteracts
        #     the disturbance. The defaults are EQUAL, so the baseline drone
        #     holds station (net wind 0) and tracks its survey waypoints
        #     cleanly. A scenario that wants the drones blown off course (a
        #     windy/storm preset) LOWERS compensation (or raises disturbance)
        #     via drone_params.yaml / launch. NOTE: with compensation 0.0 the
        #     full ~2 m/s random-direction environment wind is added to every
        #     airborne velocity command (the same magnitude as the survey
        #     step), so drones drift/crab instead of tracking the spiral and
        #     coverage collapses. That was the "drones move randomly" regression
        #     (compensation defaulting to 0.0); equal gains fix it.
        self.declare_parameter('wind_compensation_gain', 1.0)
        self.declare_parameter('wind_disturbance_gain', 1.0)

        # PID gains
        self.declare_parameter('pid_xy_p', 1.5)
        self.declare_parameter('pid_xy_i', 0.0)
        self.declare_parameter('pid_xy_d', 0.5)
        self.declare_parameter('pid_z_p', 2.0)
        self.declare_parameter('pid_z_i', 0.1)
        self.declare_parameter('pid_z_d', 0.5)

        # Get parameters
        self.drone_name = self.get_parameter('drone_name').value
        self.control_rate = self.get_parameter('control_rate').value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.landing_speed = self.get_parameter('landing_speed').value
        self.max_horizontal_speed = self.get_parameter('max_horizontal_speed').value
        self.max_vertical_speed = self.get_parameter('max_vertical_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.wind_compensation_gain = float(
            self.get_parameter('wind_compensation_gain').value)
        self.wind_disturbance_gain = float(
            self.get_parameter('wind_disturbance_gain').value)

        # Initialize PID controllers
        self.pid_x = PIDController(
            self.get_parameter('pid_xy_p').value,
            self.get_parameter('pid_xy_i').value,
            self.get_parameter('pid_xy_d').value,
            -self.max_horizontal_speed, self.max_horizontal_speed
        )
        self.pid_y = PIDController(
            self.get_parameter('pid_xy_p').value,
            self.get_parameter('pid_xy_i').value,
            self.get_parameter('pid_xy_d').value,
            -self.max_horizontal_speed, self.max_horizontal_speed
        )
        self.pid_z = PIDController(
            self.get_parameter('pid_z_p').value,
            self.get_parameter('pid_z_i').value,
            self.get_parameter('pid_z_d').value,
            -self.max_vertical_speed, self.max_vertical_speed
        )

        # Odometry timeout parameter.
        # Raised from 1.0s: under heavy sim load /clock is delivered to this
        # single-threaded node in bursts, so sim-time can jump forward 1-2s
        # between executor wake-ups. A 1.0s threshold then trips spuriously on
        # the catch-up tick (control_loop runs before the queued odom callbacks
        # update _last_odom_time), wedging the drone in permanent HOLD before
        # it can take off. 2.5s clears the worst observed clock step while
        # still catching genuine odometry loss.
        self.declare_parameter('odom_timeout', 2.5)
        self.odom_timeout = self.get_parameter('odom_timeout').value

        # Require several consecutive stale ticks before holding, so a single
        # clock catch-up cannot trip the hold. Any fresh odom resets the count.
        self.declare_parameter('odom_stale_ticks', 3)
        self._odom_stale_required = int(
            self.get_parameter('odom_stale_ticks').value
        )
        self._odom_stale_count = 0

        # State variables
        self.state = DroneState.IDLE
        self.motors_enabled = False
        # Latest wind vector (world frame, m/s) from /environment/wind.
        self._wind = (0.0, 0.0)
        self.current_pose: Optional[Pose] = None
        self.current_velocity: Optional[Twist] = None
        self.target_position: Optional[Point] = None
        self.home_position: Optional[Point] = None
        self.takeoff_start_position: Optional[Point] = None
        self._enter_survey_after_takeoff: bool = False
        # Cache cos/sin(yaw) per odom update.
        # _rotate_world_to_body runs every control tick (~50 Hz) while
        # odom arrives at ~30 Hz; recomputing atan2+cos+sin on each
        # tick wastes work. None sentinel means "no orientation yet".
        self._yaw_cos: Optional[float] = None
        self._yaw_sin: Optional[float] = None

        # Odometry tracking for readiness detection
        self._odometry_received: bool = False
        self._last_odom_time: float = 0.0

        # QoS-related tracking flags
        self.odom_deadline_missed = False
        self.odom_incompatible_qos = False

        # Retry manager (created in on_configure)
        self.odom_retry_manager = None

        # QoS profile for sensor data (store for on_configure)
        # Note: No deadline QoS - ros_gz_bridge doesn't negotiate deadline,
        # causing DEADLINE_QOS_POLICY incompatibility. Stale data handled by odom_timeout.
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # QoS profile for command publishing (reliable)
        self.command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Initialize publishers/subscribers/timers to None (created in on_configure)
        self.odom_sub = None
        self.imu_sub = None
        self.enable_sub = None
        self.target_sub = None
        self.takeoff_sub = None
        self.land_sub = None
        self.survey_target_sub = None
        self.survey_start_sub = None
        self.wind_sub = None
        self.weather_sub = None
        self.cmd_vel_pub = None
        self.status_pub = None
        self.enable_pub = None
        self.control_timer = None
        self.status_timer = None
        self.enable_timer = None

        # Active state flag for control loop guard
        self.is_active = False

        # Diagnostic updater (initialized properly in on_configure)
        self.updater = None

        # Motors start disabled by default
        # Gazebo model also has motors disabled by default

        self.get_logger().info(f'Drone controller initialized for {self.drone_name}')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure lifecycle node - create publishers, subscribers, timers."""
        self.get_logger().info('Configuring drone controller...')

        # Clock port resolved via the resolve_clock helper.
        self._time = resolve_clock(self, self._composition)

        # Use regular publisher for cmd_vel to avoid lifecycle publisher issues
        # with DDS discovery (lifecycle publishers may not register properly)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{self.drone_name}/cmd_vel',
            self.command_qos
        )

        self.status_pub = self.create_publisher(
            DroneStatus,
            f'/{self.drone_name}/status',
            self.command_qos
        )

        # Use regular publisher for enable to avoid lifecycle publisher issues
        self.enable_pub = self.create_publisher(
            Bool,
            f'/{self.drone_name}/enable',
            10
        )

        # Create subscriptions
        # Create event callbacks for QoS monitoring
        odom_sub_callbacks = SubscriptionEventCallbacks()
        odom_sub_callbacks.deadline = self._odom_deadline_callback
        odom_sub_callbacks.incompatible_qos = self._odom_incompatible_qos_callback

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.drone_name}/odom',
            self.odom_callback,
            self.sensor_qos,
            event_callbacks=odom_sub_callbacks
        )

        self.imu_sub = self.create_subscription(
            Imu,
            f'/{self.drone_name}/imu',
            self.imu_callback,
            self.sensor_qos
        )

        self.enable_sub = self.create_subscription(
            Bool,
            f'/{self.drone_name}/enable',
            self.enable_callback,
            10
        )

        self.target_sub = self.create_subscription(
            PoseStamped,
            f'/{self.drone_name}/target_pose',
            self.target_callback,
            10
        )

        self.takeoff_sub = self.create_subscription(
            Float32,
            f'/{self.drone_name}/takeoff',
            self.takeoff_callback,
            10
        )

        self.land_sub = self.create_subscription(
            Bool,
            f'/{self.drone_name}/land',
            self.land_callback,
            10
        )

        self.survey_target_sub = self.create_subscription(
            PoseStamped,
            f'/{self.drone_name}/survey_target',
            self.survey_target_callback,
            10
        )

        # /survey/start race fix: the readiness_coordinator
        # publishes /survey/start TRANSIENT_LOCAL (latched) only AFTER all
        # drones have been ready for 5 s, which can be after this controller
        # finishes lifecycle configuration. A VOLATILE subscriber created
        # after that publish never receives the latched sample, so the
        # controller stays in IDLE and never takes off (observed: motors
        # never armed, 0 survey_target). Match the publisher's
        # TRANSIENT_LOCAL durability (exactly as mission_manager's
        # /survey/start sub does) so the latched value is delivered to this
        # late-joining subscriber regardless of discovery timing. Odom is
        # already flowing by the time readiness fires, so the takeoff guard
        # is satisfied on delivery.
        survey_start_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.survey_start_sub = self.create_subscription(
            Bool,
            '/survey/start',
            self.survey_start_callback,
            survey_start_qos
        )

        # Wind from environment_monitor: consumed as a software disturbance
        # + station-keeping (see control_loop). Global topics, shared by every
        # drone. WeatherState is the richer primary signal; the legacy
        # Vector3 topic is kept for back-compat. Both feed self._wind.
        self.weather_sub = self.create_subscription(
            WeatherState,
            '/environment/weather',
            self.weather_callback,
            10
        )
        self.wind_sub = self.create_subscription(
            Vector3,
            '/environment/wind',
            self.wind_callback,
            10
        )

        # Create control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        # Create status publish timer
        self.status_timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_status
        )

        # Re-assert the motor-enable state at 1 Hz. set_motors_enabled() is
        # edge-triggered: it publishes /<drone>/enable only when the state
        # changes. That single message is VOLATILE, so if the Gazebo-side
        # enable bridge / MulticopterVelocityControl plugin subscribes AFTER
        # it is sent (always the case under slow or software-rendered
        # bringup), it is lost and the drone is left permanently disarmed.
        # Periodically re-publishing the current state makes the arm signal
        # robust to bringup order. See _reassert_enable.
        self.enable_timer = self.create_timer(
            1.0,  # 1 Hz
            self._reassert_enable
        )

        # Initialize retry manager
        self.odom_retry_manager = OdomRetryManager(self, max_retries=3, base_delay=1.0)

        # Initialize diagnostic updater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID(f'{self.drone_name}-controller')

        # Add diagnostic tasks
        self.updater.add('Controller State', self.check_controller_state)
        self.updater.add('Odometry Freshness', self.check_odometry_freshness)
        self.updater.add('Motor Status', self.check_motor_status)

        self.get_logger().info('Drone controller configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate lifecycle node - enable control and publishing."""
        self.get_logger().info('Activating drone controller...')

        # Verify odometry is available
        if self.current_pose is None:
            self.get_logger().warning(
                'Activated without odometry - waiting for first message',
                throttle_duration_sec=5.0
            )

        self.is_active = True
        self.get_logger().info('Drone controller activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Deactivate drone controller.

        If drone is in SURVEYING or HOVER state, initiate landing.
        Wait for landing to complete before returning.
        """
        self.get_logger().info(f'Deactivating {self.drone_name} controller...')

        # Request landing if drone is in the air
        if self.state in [DroneState.SURVEYING, DroneState.HOVER, DroneState.NAVIGATING]:
            self.get_logger().info(f'{self.drone_name}: Initiating landing for deactivation')
            self._initiate_deactivation_landing()

        # Set inactive flag
        self.is_active = False

        # Force diagnostic update to report inactive status
        if self.updater:
            self.updater.force_update()

        # Disable motors and publish zero velocity
        self._safe_stop()

        self.get_logger().info(f'{self.drone_name} controller deactivated')
        return TransitionCallbackReturn.SUCCESS

    def _safe_destroy(self, attr_name: str, destroy_fn) -> None:
        """Delegate to shared helper. The
        helper lives in `lib/ros_adapter/lifecycle_teardown.py` so
        sibling LifecycleNodes (surveyor first, future promotions
        next) can re-use the same idiom."""
        from drone_rescue_coordination.lib.ros_adapter.lifecycle_teardown import (
            safe_destroy,
        )
        safe_destroy(self, attr_name, destroy_fn)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup lifecycle node - destroy resources.

        13 try/except destroy_* blocks collapse into 13
        `_safe_destroy(attr, destroy_fn)` calls. Tear-down order
        preserved (reverse creation order: timers, subscribers,
        publishers).
        """
        self.get_logger().info('Cleaning up drone controller...')

        # Cleanup retry manager timer
        if self.odom_retry_manager and self.odom_retry_manager.retry_timer:
            self.destroy_timer(self.odom_retry_manager.retry_timer)

        # Destroy resources in reverse creation order via the helper.
        self._safe_destroy('enable_timer', self.destroy_timer)
        self._safe_destroy('status_timer', self.destroy_timer)
        self._safe_destroy('control_timer', self.destroy_timer)
        self._safe_destroy('wind_sub', self.destroy_subscription)
        self._safe_destroy('weather_sub', self.destroy_subscription)
        self._safe_destroy('survey_start_sub', self.destroy_subscription)
        self._safe_destroy('survey_target_sub', self.destroy_subscription)
        self._safe_destroy('land_sub', self.destroy_subscription)
        self._safe_destroy('takeoff_sub', self.destroy_subscription)
        self._safe_destroy('target_sub', self.destroy_subscription)
        self._safe_destroy('enable_sub', self.destroy_subscription)
        self._safe_destroy('imu_sub', self.destroy_subscription)
        self._safe_destroy('odom_sub', self.destroy_subscription)
        self._safe_destroy('enable_pub', self.destroy_publisher)
        self._safe_destroy('status_pub', self.destroy_publisher)
        self._safe_destroy('cmd_vel_pub', self.destroy_publisher)

        self.get_logger().info('Drone controller cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown lifecycle node."""
        self.get_logger().info('Shutting down drone controller...')
        return TransitionCallbackReturn.SUCCESS

    def _odom_deadline_callback(self, event):
        """Called when odometry deadline is missed (no message within 500ms)."""
        self.odom_deadline_missed = True
        self.get_logger().warning(
            f'{self.drone_name}: Odometry deadline missed! No data for >500ms. '
            f'total_count={event.total_count}, total_count_change={event.total_count_change}',
            throttle_duration_sec=5.0
        )
        # Trigger hold-position behavior by marking data stale
        # control_loop already handles stale odometry
        if self.updater:
            self.updater.force_update()

    def _odom_incompatible_qos_callback(self, event):
        """Called when odometry publisher has incompatible QoS."""
        self.odom_incompatible_qos = True
        self.get_logger().error(
            f'{self.drone_name}: Incompatible QoS on odometry topic! '
            f'Publisher offers incompatible profile. '
            f'last_policy_kind={event.last_policy_kind}, total_count={event.total_count}'
        )

    def odom_callback(self, msg: Odometry):
        """Process odometry data."""
        if not self._odometry_received:
            self.get_logger().info(
                f'First odom received! z={msg.pose.pose.position.z:.3f}'
            )
        self.odom_deadline_missed = False  # Fresh data received

        # Reset retry manager on fresh data
        if self.odom_retry_manager:
            self.odom_retry_manager.reset()

        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

        # Recompute yaw cos/sin once per odom
        # update; reused by _rotate_world_to_body every control tick.
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._yaw_cos = math.cos(yaw)
        self._yaw_sin = math.sin(yaw)

        # Track odometry reception for readiness
        self._odometry_received = True
        self._last_odom_time = self._time.now_sec()
        # Fresh data clears the consecutive-stale counter.
        self._odom_stale_count = 0

        # Set home position on first odometry
        if self.home_position is None:
            self.home_position = Point()
            self.home_position.x = msg.pose.pose.position.x
            self.home_position.y = msg.pose.pose.position.y
            self.home_position.z = 0.0  # Ground level
            self.get_logger().info(
                f'Home position set: ({self.home_position.x:.2f}, '
                f'{self.home_position.y:.2f})'
            )

    def imu_callback(self, msg: Imu):
        """Process IMU data (currently unused, but available for attitude control)."""
        pass

    def set_motors_enabled(self, enabled: bool, force: bool = False):
        """
        Set motor enable state and publish to Gazebo.

        This is the ONLY method that should be used to enable/disable motors.
        It updates internal state AND publishes to the Gazebo plugin.

        Args:
            enabled: True to enable motors, False to disable
            force: If True, publish even if state matches (used for startup sync)
        """
        # Only publish if state actually changes (avoids feedback loop from bridge)
        if not force and self.motors_enabled == enabled:
            return

        self.motors_enabled = enabled

        # CRITICAL: Publish to Gazebo MulticopterVelocityControl plugin
        enable_msg = Bool()
        enable_msg.data = enabled
        self.enable_pub.publish(enable_msg)

        if enabled:
            self.get_logger().info('Motors ENABLED (published to Gazebo)')
        else:
            self.get_logger().info('Motors DISABLED (published to Gazebo)')

    def _reassert_enable(self):
        """Periodically re-publish the current motor-enable state.

        set_motors_enabled() publishes only on edge (state change), and that
        message is VOLATILE: a late-subscribing Gazebo enable bridge or
        multicopter plugin never receives it, leaving the drone disarmed.
        Re-asserting at 1 Hz delivers the state to whoever is listening,
        whenever they connect. Idempotent on the plugin; the controller's own
        enable_sub no-ops on unchanged state, so this creates no feedback loop.
        """
        if self.enable_pub is None:
            return
        msg = Bool()
        msg.data = self.motors_enabled
        self.enable_pub.publish(msg)

    def enable_callback(self, msg: Bool):
        """Handle external motor enable/disable command."""
        self.set_motors_enabled(msg.data)
        if not self.motors_enabled:
            self.state = DroneState.IDLE
            self.reset_controllers()

    def takeoff_callback(self, msg: Float32):
        """Handle takeoff command with optional altitude."""
        if not self.motors_enabled:
            self.get_logger().warning(
                'Cannot takeoff: motors not enabled',
                throttle_duration_sec=2.0
            )
            return

        if self.state not in [DroneState.IDLE, DroneState.HOVER]:
            self.get_logger().warning(
                f'Cannot takeoff from state: {self.state.name}',
                throttle_duration_sec=2.0
            )
            return

        altitude = msg.data if msg.data > 0 else self.takeoff_altitude
        self.get_logger().info(f'Takeoff commanded to altitude: {altitude:.1f}m')

        if self.current_pose:
            self.takeoff_start_position = Point()
            self.takeoff_start_position.x = self.current_pose.position.x
            self.takeoff_start_position.y = self.current_pose.position.y
            self.takeoff_start_position.z = self.current_pose.position.z

            self.target_position = Point()
            self.target_position.x = self.current_pose.position.x
            self.target_position.y = self.current_pose.position.y
            self.target_position.z = altitude

        self.state = DroneState.TAKEOFF
        self.reset_controllers()

    def land_callback(self, msg: Bool):
        """Handle land command."""
        if msg.data and self.state not in [DroneState.IDLE, DroneState.LANDING]:
            self.get_logger().info('Landing commanded')
            self.state = DroneState.LANDING
            if self.current_pose:
                self.target_position = Point()
                self.target_position.x = self.current_pose.position.x
                self.target_position.y = self.current_pose.position.y
                self.target_position.z = 0.0
            self.reset_controllers()

    def survey_start_callback(self, msg: Bool):
        """Handle survey start command - takeoff and enter survey mode."""
        if not msg.data:
            return

        # Guard: Require valid odometry before starting survey
        if not self._odometry_received or self.current_pose is None:
            self.get_logger().warning(
                'Survey start received but odometry not yet available - ignoring'
            )
            return

        if self.state == DroneState.IDLE:
            # Enable motors and takeoff
            self.set_motors_enabled(True)
            self.get_logger().info('Survey start: taking off')

            if self.current_pose:
                self.takeoff_start_position = Point()
                self.takeoff_start_position.x = self.current_pose.position.x
                self.takeoff_start_position.y = self.current_pose.position.y
                self.takeoff_start_position.z = self.current_pose.position.z

                self.target_position = Point()
                self.target_position.x = self.current_pose.position.x
                self.target_position.y = self.current_pose.position.y
                self.target_position.z = self.takeoff_altitude

            self.state = DroneState.TAKEOFF
            self.reset_controllers()
            # Flag to enter survey mode after takeoff
            self._enter_survey_after_takeoff = True

        elif self.state == DroneState.HOVER:
            self.get_logger().info('Starting survey from hover')
            self.state = DroneState.SURVEYING

    def survey_target_callback(self, msg: PoseStamped):
        """Handle survey target from surveyor node."""
        if self.state != DroneState.SURVEYING:
            return

        self.target_position = msg.pose.position

    def target_callback(self, msg: PoseStamped):
        """Handle new target position."""
        if self.state not in [DroneState.HOVER, DroneState.NAVIGATING]:
            self.get_logger().warning(
                f'Cannot navigate from state: {self.state.name}. Must be hovering first.',
                throttle_duration_sec=2.0
            )
            return

        self.target_position = msg.pose.position
        self.state = DroneState.NAVIGATING
        self.get_logger().info(
            f'New target: ({self.target_position.x:.2f}, '
            f'{self.target_position.y:.2f}, {self.target_position.z:.2f})'
        )

    def reset_controllers(self):
        """Reset all PID controllers."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

    def get_distance_to_target(self) -> float:
        """Calculate 3D distance to target."""
        if self.current_pose is None or self.target_position is None:
            return float('inf')

        dx = self.target_position.x - self.current_pose.position.x
        dy = self.target_position.y - self.current_pose.position.y
        dz = self.target_position.z - self.current_pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def get_horizontal_distance_to_target(self) -> float:
        """Calculate 2D horizontal distance to target."""
        if self.current_pose is None or self.target_position is None:
            return float('inf')

        dx = self.target_position.x - self.current_pose.position.x
        dy = self.target_position.y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    _control_loop_count = 0

    def control_loop(self):
        """Main control loop - runs at control_rate Hz."""
        self._control_loop_count += 1
        if self._control_loop_count == 1:
            self.get_logger().info('Control loop first tick')
        elif self._control_loop_count % 500 == 0:
            self.get_logger().info(
                f'Control loop tick #{self._control_loop_count}, '
                f'active={self.is_active}, motors={self.motors_enabled}, '
                f'pose={"set" if self.current_pose else "None"}, '
                f'state={self.state}'
            )

        # Allow landing to complete even during deactivation
        if not self.is_active and self.state != DroneState.LANDING:
            return

        if not self.motors_enabled or self.current_pose is None:
            self.publish_zero_velocity()
            return

        current_time = self._time.now_sec()

        # Stale odometry detection - hold position if odom too old.
        # Require N consecutive stale ticks so a single sim-clock catch-up
        # (control_loop running before queued odom callbacks drain) cannot
        # wedge the drone in HOLD. A fresh odom callback zeroes the counter.
        if self._last_odom_time > 0:
            odom_age = current_time - self._last_odom_time
            if odom_age > self.odom_timeout:
                self._odom_stale_count += 1
            else:
                self._odom_stale_count = 0

            if self._odom_stale_count >= self._odom_stale_required:
                # Attempt retry with backoff
                if self.odom_retry_manager and not self.odom_retry_manager.in_retry:
                    if not self.odom_retry_manager.attempt_retry(
                        f'{self.drone_name}: Stale odometry ({odom_age:.2f}s)'
                    ):
                        # Retries exhausted - return to base
                        self.get_logger().error(
                            f'{self.drone_name}: Odometry loss persists - returning to base'
                        )
                        self.return_to_base()
                        return
                # Hold position while retrying
                self.get_logger().warning(
                    f'{self.drone_name}: Stale odometry ({odom_age:.2f}s) - HOLD POSITION',
                    throttle_duration_sec=5.0
                )
                self.publish_zero_velocity()
                return

        cmd = Twist()

        if self.state == DroneState.IDLE:
            self.publish_zero_velocity()
            return

        elif self.state == DroneState.TAKEOFF:
            cmd = self.compute_takeoff_velocity(current_time)
            # Check if reached takeoff altitude
            if self.target_position and self.current_pose:
                altitude_error = abs(
                    self.target_position.z - self.current_pose.position.z
                )
                if altitude_error < self.position_tolerance:
                    if self._enter_survey_after_takeoff:
                        self.get_logger().info('Takeoff complete, entering survey mode')
                        self.state = DroneState.SURVEYING
                        self._enter_survey_after_takeoff = False
                    else:
                        self.get_logger().info('Takeoff complete, hovering')
                        self.state = DroneState.HOVER

        elif self.state == DroneState.SURVEYING:
            # Follow targets from surveyor node
            cmd = self.compute_position_velocity(current_time)

        elif self.state == DroneState.HOVER:
            # Maintain current position
            if self.target_position is None and self.current_pose:
                self.target_position = Point()
                self.target_position.x = self.current_pose.position.x
                self.target_position.y = self.current_pose.position.y
                self.target_position.z = self.current_pose.position.z
            cmd = self.compute_position_velocity(current_time)

        elif self.state == DroneState.NAVIGATING:
            cmd = self.compute_position_velocity(current_time)
            # Check if reached target
            if self.get_distance_to_target() < self.position_tolerance:
                self.get_logger().info('Target reached, hovering')
                self.state = DroneState.HOVER

        elif self.state == DroneState.RETURNING:
            cmd = self.compute_position_velocity(current_time)
            if self.get_distance_to_target() < self.position_tolerance:
                self.get_logger().info('Returned to base, landing')
                self.state = DroneState.LANDING
                if self.home_position:
                    self.target_position = Point()
                    self.target_position.x = self.home_position.x
                    self.target_position.y = self.home_position.y
                    self.target_position.z = 0.0

        elif self.state == DroneState.LANDING:
            cmd = self.compute_landing_velocity(current_time)
            # Check if landed
            if self._is_landed():
                self.get_logger().info('Landing complete')
                self.state = DroneState.IDLE
                self._safe_stop()
                return

        elif self.state == DroneState.EMERGENCY:
            # Emergency: cut motors
            self.publish_zero_velocity()
            return

        # Software wind model (world frame, before the body rotation).
        # The Gazebo wind topic is unbridged, so the controller models the wind
        # itself: it adds (disturbance_gain - compensation_gain) * wind to the
        # commanded velocity. At compensation 0 the drone drifts downwind (wind
        # finally moves it); raising compensation toward the disturbance gain
        # holds station. Applied only while airborne (a grounded/IDLE drone is
        # not blown around) and a no-op when calm (wind == 0).
        if self.state in _AIRBORNE_STATES:
            wind_dx, wind_dy = wind_velocity_offset(
                self._wind, self.wind_disturbance_gain,
                self.wind_compensation_gain,
            )
            cmd.linear.x += wind_dx
            cmd.linear.y += wind_dy

        # gz-sim's MulticopterVelocityControl interprets cmd_vel in the body
        # frame. compute_position_velocity returns a WORLD-frame setpoint
        # (target - current), so we rotate by the drone's current yaw before
        # publishing. Z is yaw-invariant.
        cmd = self._rotate_world_to_body(cmd)
        self.cmd_vel_pub.publish(cmd)

    def wind_callback(self, msg: Vector3):
        """Store the latest world-frame wind vector (m/s) from
        environment_monitor; consumed as a disturbance in control_loop."""
        self._wind = (msg.x, msg.y)

    def weather_callback(self, msg: WeatherState):
        """Store the wind vector from the richer WeatherState signal.

        environment_monitor publishes both /environment/weather and
        /environment/wind with the same wind vector; either updates the
        software wind model applied in control_loop."""
        self._wind = (msg.wind_velocity.x, msg.wind_velocity.y)

    def _rotate_world_to_body(self, cmd: Twist) -> Twist:
        """Rotate the horizontal components of `cmd` from world frame into the
        drone's body frame using the current yaw from odometry. No-op if
        orientation is unavailable (e.g. before first odom).

        yaw cos/sin are cached in odom_callback; this method just multiplies."""
        cy = self._yaw_cos
        sy = self._yaw_sin
        if cy is None or sy is None:
            return cmd
        wx, wy = cmd.linear.x, cmd.linear.y
        cmd.linear.x = cy * wx + sy * wy
        cmd.linear.y = -sy * wx + cy * wy
        return cmd

    def _compute_setpoint_velocity(
        self,
        current_time: float,
        *,
        xy_anchor: Optional[Point],
        xy_gain: float = 1.0,
        z_target: Optional[float] = None,
        z_override: Optional[float] = None,
    ) -> Twist:
        """Shared body for the three PID-3D setpoint phases.

        Collapses compute_takeoff_velocity,
        compute_position_velocity, compute_landing_velocity. Each
        phase picks its xy anchor (start vs target), its xy gain
        (0.5 for takeoff/landing stabilisation, 1.0 for cruise),
        and either a target z (PID) or a literal z velocity
        override (landing constant-descent).
        """
        cmd = Twist()
        if self.current_pose is None:
            return cmd

        if z_override is not None:
            cmd.linear.z = z_override
        elif z_target is not None:
            error_z = z_target - self.current_pose.position.z
            cmd.linear.z = self.pid_z.compute(error_z, current_time)

        if xy_anchor is not None:
            error_x = xy_anchor.x - self.current_pose.position.x
            error_y = xy_anchor.y - self.current_pose.position.y
            cmd.linear.x = self.pid_x.compute(error_x, current_time) * xy_gain
            cmd.linear.y = self.pid_y.compute(error_y, current_time) * xy_gain

        return cmd

    def compute_takeoff_velocity(self, current_time: float) -> Twist:
        """Compute velocity for takeoff phase."""
        if self.current_pose is None or self.target_position is None:
            return Twist()
        return self._compute_setpoint_velocity(
            current_time,
            xy_anchor=self.takeoff_start_position,
            xy_gain=0.5,
            z_target=self.target_position.z,
        )

    def compute_position_velocity(self, current_time: float) -> Twist:
        """Compute velocity to reach target position."""
        if self.current_pose is None or self.target_position is None:
            return Twist()
        return self._compute_setpoint_velocity(
            current_time,
            xy_anchor=self.target_position,
            xy_gain=1.0,
            z_target=self.target_position.z,
        )

    def compute_landing_velocity(self, current_time: float) -> Twist:
        """Compute velocity for landing phase."""
        return self._compute_setpoint_velocity(
            current_time,
            xy_anchor=self.target_position,
            xy_gain=0.5,
            z_override=-self.landing_speed,
        )

        return cmd

    def publish_zero_velocity(self):
        """Publish zero velocity command."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def publish_status(self):
        """Publish drone status message."""
        # Guard against inactive state
        if not self.is_active:
            return

        status = DroneStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.drone_id = self.drone_name
        status.state = self.state.value

        if self.current_pose:
            status.pose = self.current_pose

        if self.current_velocity:
            status.velocity = self.current_velocity

        if self.target_position:
            status.target_position = self.target_position

        # Battery placeholder (will be updated in battery_monitor)
        status.battery_level = 1.0
        status.battery_drain_rate = 0.0

        status.camera_active = True
        status.lidar_active = True

        self.status_pub.publish(status)

    def return_to_base(self):
        """Command drone to return to home position."""
        if self.home_position is None:
            self.get_logger().error('No home position set!')
            return

        self.get_logger().info('Returning to base')
        self.target_position = Point()
        self.target_position.x = self.home_position.x
        self.target_position.y = self.home_position.y
        self.target_position.z = self.takeoff_altitude
        self.state = DroneState.RETURNING
        self.reset_controllers()

    def _initiate_deactivation_landing(self):
        """
        Initiate landing sequence for deactivation.

        Flips state to LANDING and returns immediately.
        The legacy version called `time.sleep(0.5)` here, blocking the
        ROS executor thread. The comment
        on the legacy block admitted "control loop handles descent":
        the sleep was load-bearing only for the order of lifecycle
        log lines, not for the descent itself. The next control tick
        (50 Hz default) sees state==LANDING and issues a landing
        setpoint; that's the actual mechanism.
        """
        self.state = DroneState.LANDING

    def _safe_stop(self):
        """Publish zero velocity and disable motors."""
        # Publish zero velocity
        if self.cmd_vel_pub is not None:
            zero_vel = Twist()
            self.cmd_vel_pub.publish(zero_vel)

        # Disable motors
        if self.enable_pub is not None:
            self.set_motors_enabled(False)

    def _is_landed(self) -> bool:
        """Check if drone has landed (altitude near ground)."""
        if self.current_pose is None:
            return False
        # Consider landed if altitude < 0.5 meters
        return self.current_pose.position.z < 0.5

    def check_controller_state(self, stat):
        """Diagnostic callback for controller state."""
        if not self.is_active:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE,
                        'Controller inactive')
        elif self.state == DroneState.EMERGENCY:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                        f'EMERGENCY: {self.state.name}')
        elif self.state == DroneState.IDLE and not self.motors_enabled:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                        'Idle, motors off')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                        f'State: {self.state.name}')

        stat.add('State', self.state.name)
        stat.add('Motors enabled', str(self.motors_enabled))
        if self.current_pose:
            stat.add('Altitude', f'{self.current_pose.position.z:.2f}m')
        return stat

    def check_odometry_freshness(self, stat):
        """Diagnostic callback for odometry staleness."""
        if not self.is_active:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE,
                        'Controller inactive')
            return stat

        if self.odom_deadline_missed:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                        'Odometry deadline missed (QoS violation)')
            stat.add('QoS deadline', 'missed')
            stat.add('Action', 'Hold position active')
            return stat

        if not self._odometry_received:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                        'No odometry received')
            stat.add('Age (seconds)', 'N/A')
            return stat

        current_time = self._time.now_sec()
        age = current_time - self._last_odom_time

        if age > self.odom_timeout:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                        f'Odometry stale: {age:.2f}s old')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                        'Odometry fresh')

        stat.add('Age (seconds)', f'{age:.2f}')
        stat.add('Timeout threshold', f'{self.odom_timeout:.2f}')
        return stat

    def check_motor_status(self, stat):
        """Diagnostic callback for motor status."""
        if not self.is_active:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE,
                        'Controller inactive')
        elif self.motors_enabled:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                        'Motors enabled')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                        'Motors disabled')

        stat.add('Motors enabled', str(self.motors_enabled))
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = bind_composition(DroneController())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
