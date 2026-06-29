"""Per-drone health monitor.

3T Architecture Executive Layer (L2): anomaly detection feeding
RecoveryPolicy (Marcelletti slides p. 38: "monitoring and handling
exceptions").

L2 supervisory function: watches sensor streams, classifies drone
state, and reports anomalies that the executive's ``RecoveryPolicy``
(``lib/lifecycle/recovery_policy.py``) consumes. Hands off to the L3
planner via ``ExecutiveSupervisor`` when an anomaly requires
re-allocation.

Watches a single drone's IMU, odom, battery and LiDAR streams and publishes
`/<drone>/health` (DroneHealth) at a fixed rate. Also publishes a one-shot
MissionEvent on `/mission/events` the FIRST time the drone becomes
unrecoverable, so the dashboard log shows a `DRONE_DAMAGE_REPORT` entry
before mission_manager reassigns the sector.

Anomaly inputs and thresholds (parameterised; defaults match the plan):

| Input                | Anomaly trigger                                    |
|----------------------|----------------------------------------------------|
| /<drone>/imu         | |linear_acceleration| ≥ `imu_spike_g` for 1 sample |
| /<drone>/odom        | message age ≥ `odom_stale_s`                       |
| /<drone>/odom (vel)  | |body velocity| < `vel_freeze_m_s` for             |
|                      |   `vel_freeze_window_s` while a setpoint is active |
| /<drone>/battery_low | latched True (treated as a single anomaly)         |
| /<drone>/scan        | min(ranges) < `lidar_imminent_m` for                |
|                      |   `lidar_imminent_window_s`                        |

`unrecoverable=True` when ≥ `unrecoverable_anomaly_count` distinct anomalies
are simultaneously raised AND the drone is at z < `grounded_altitude_m`
(i.e. it's NOT just briefly perturbed in flight, it's stuck on the ground
with multiple problems). That's the right test for "human pickup needed".

The node publishes at `publish_rate_hz` regardless of anomaly state so the
dashboard always has a fresh row.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist

from drone_rescue_msgs.msg import DroneHealth, MissionEvent

from drone_rescue_coordination.lib.ros_adapter.ros_clock import RosClock
from drone_rescue_coordination.lib.composition import (
    bind_composition, resolve_clock,
)


def _clock_seconds(node: Node) -> float:
    """Thin shim over the Clock port. Existing call sites
    (`now = _clock_seconds(self)`) keep working unchanged; the inline
    `node.get_clock().now().nanoseconds / 1e9` idiom is no longer
    hand-rolled here. Tests can override by setting `node._time` to a
    FakeClock before invoking the callback."""
    # RosClock import hoisted to module top.
    # Guard reads `_time`; the legacy `_clock` lookup always found the
    # rclpy Node._clock and shadowed the lazy construct path.
    clock = getattr(node, '_time', None)
    if clock is None:
        clock = RosClock(node)
        node._time = clock
    return clock.now_sec()


class DroneHealthMonitor(Node):
    # composition kwarg; when provided, the composition.clock
    # pre-populates ``self._time`` so the `_clock_seconds(self)` helper
    # picks it up via getattr fallback.
    def __init__(self, *, composition=None):
        super().__init__('drone_health_monitor')
        # store the composition so the on_configure path can read
        # event_port from it.
        self._composition = composition
        # unify clock resolution with the other LifecycleNodes;
        # ``_clock_seconds`` still works via the
        # ``getattr(node, '_time', None)`` lookup. Import hoisted to
        # module top.
        self._time = resolve_clock(self, composition)

        self.declare_parameter('drone_name', 'drone1')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('imu_spike_g', 30.0)            # m/s² (≈3g)
        self.declare_parameter('odom_stale_s', 2.0)
        self.declare_parameter('vel_freeze_m_s', 0.2)
        self.declare_parameter('vel_freeze_window_s', 4.0)
        # Position freeze: a far better "drone is stuck" signal than body
        # velocity. A grounded drone with motors still spinning oscillates at
        # 0.5-1 m/s body velocity but its world XY barely moves. Flag if the
        # XY moves < `pos_freeze_radius_m` over `pos_freeze_window_s` while
        # the controller is commanding non-zero motion.
        self.declare_parameter('pos_freeze_radius_m', 1.0)
        self.declare_parameter('pos_freeze_window_s', 6.0)
        self.declare_parameter('lidar_imminent_m', 1.0)
        self.declare_parameter('lidar_imminent_window_s', 1.0)
        self.declare_parameter('unrecoverable_anomaly_count', 2)
        self.declare_parameter('grounded_altitude_m', 5.0)
        self.declare_parameter('battery_curve_window_s', 30.0)
        self.declare_parameter('battery_remaining_threshold_s', 60.0)

        self.drone_name = str(self.get_parameter('drone_name').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.imu_spike_g = float(self.get_parameter('imu_spike_g').value)
        self.odom_stale_s = float(self.get_parameter('odom_stale_s').value)
        self.vel_freeze_m_s = float(self.get_parameter('vel_freeze_m_s').value)
        self.vel_freeze_window_s = float(self.get_parameter('vel_freeze_window_s').value)
        self.pos_freeze_radius_m = float(self.get_parameter('pos_freeze_radius_m').value)
        self.pos_freeze_window_s = float(self.get_parameter('pos_freeze_window_s').value)
        self.lidar_imminent_m = float(self.get_parameter('lidar_imminent_m').value)
        self.lidar_imminent_window_s = float(self.get_parameter('lidar_imminent_window_s').value)
        self.unrecoverable_anomaly_count = int(self.get_parameter('unrecoverable_anomaly_count').value)
        self.grounded_altitude_m = float(self.get_parameter('grounded_altitude_m').value)
        self.battery_curve_window_s = float(self.get_parameter('battery_curve_window_s').value)
        self.battery_remaining_threshold_s = float(
            self.get_parameter('battery_remaining_threshold_s').value
        )

        # State
        self._first_seen_t: Optional[float] = None
        self._last_odom_t: Optional[float] = None
        self._last_pose_x: float = 0.0
        self._last_pose_y: float = 0.0
        self._last_pose_z: float = 0.0
        # Position-history ring for the position-freeze check: list of
        # (t, x, y) older than pos_freeze_window_s gets evicted on each odom.
        self._pose_history: deque = deque()
        self._last_vel_norm: float = 0.0
        self._last_imu_norm: float = 0.0
        self._battery_low: bool = False
        # Sliding histories for "for N seconds" anomaly windows.
        self._vel_freeze_since: Optional[float] = None
        self._lidar_imminent_since: Optional[float] = None
        # Battery curve: list of (t, level) used to estimate remaining seconds.
        self._battery_history: deque = deque()
        self._lidar_min_range: float = -1.0
        # One-shot DRONE_DAMAGE_REPORT.
        self._damage_emitted: bool = False
        # Set True when the drone receives any cmd_vel; used to decide whether
        # vel_freeze applies (no point flagging "frozen" when nobody asked it
        # to move).
        self._has_recent_cmd: bool = False
        self._last_cmd_t: float = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=10,
        )
        self.create_subscription(
            Imu, f'/{self.drone_name}/imu', self._on_imu, sensor_qos,
        )
        self.create_subscription(
            Odometry, f'/{self.drone_name}/odom', self._on_odom, sensor_qos,
        )
        self.create_subscription(
            LaserScan, f'/{self.drone_name}/scan',
            self._on_scan, QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, depth=1,
            ),
        )
        self.create_subscription(
            Bool, f'/{self.drone_name}/battery_low',
            self._on_battery_low, 10,
        )
        self.create_subscription(
            Float32, f'/{self.drone_name}/battery_level',
            self._on_battery_level, 10,
        )
        # cmd_vel is the right place to learn "we asked the drone to move".
        # We only flag vel_freeze when the controller is actually commanding
        # motion, otherwise hovering looks like a stall.
        self.create_subscription(
            Twist, f'/{self.drone_name}/cmd_vel',
            self._on_cmd_vel, 10,
        )

        self._health_pub = self.create_publisher(
            DroneHealth, f'/{self.drone_name}/health',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE, depth=10),
        )
        # consume composition.event_port when available; fall back to
        # inline publisher build for tests.
        if (self._composition is not None
                and self._composition.event_port is not None):
            self._event_port = self._composition.event_port
            self._event_pub = None
        else:
            self._event_pub = self.create_publisher(
                MissionEvent, '/mission/events',
                QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE, depth=50),
            )
            from drone_rescue_coordination.lib.ros_adapter.event_publisher import (
                RosEventPublisherAdapter,
            )
            self._event_port = RosEventPublisherAdapter(self._event_pub)

        self._timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 0.5), self._tick,
        )
        self.get_logger().info(
            f'drone_health_monitor[{self.drone_name}] up — rate {self.publish_rate_hz}Hz, '
            f'imu_spike={self.imu_spike_g}m/s², odom_stale={self.odom_stale_s}s, '
            f'unrecoverable_count={self.unrecoverable_anomaly_count}'
        )

    # subs
    def _on_imu(self, msg: Imu) -> None:
        a = msg.linear_acceleration
        self._last_imu_norm = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)

    def _on_odom(self, msg: Odometry) -> None:
        now = _clock_seconds(self)
        self._first_seen_t = self._first_seen_t or now
        self._last_odom_t = now
        self._last_pose_x = msg.pose.pose.position.x
        self._last_pose_y = msg.pose.pose.position.y
        self._last_pose_z = msg.pose.pose.position.z
        v = msg.twist.twist.linear
        self._last_vel_norm = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        # Position history for the position-freeze check.
        self._pose_history.append((now, self._last_pose_x, self._last_pose_y))
        cutoff = now - self.pos_freeze_window_s
        while self._pose_history and self._pose_history[0][0] < cutoff:
            self._pose_history.popleft()

    def _on_scan(self, msg: LaserScan) -> None:
        # numpy mask drops the per-scan Python list + min() allocation.
        # On a 360-beam LiDAR at 10 Hz this halves the hot-path cost.
        arr = np.asarray(msg.ranges, dtype=np.float32)
        mask = (arr > msg.range_min) & (arr < msg.range_max)
        if mask.any():
            self._lidar_min_range = float(arr[mask].min())
        else:
            self._lidar_min_range = -1.0

    def _on_battery_low(self, msg: Bool) -> None:
        if msg.data:
            self._battery_low = True

    def _on_battery_level(self, msg: Float32) -> None:
        now = _clock_seconds(self)
        self._battery_history.append((now, float(msg.data)))
        # Trim window.
        while self._battery_history and self._battery_history[0][0] < now - self.battery_curve_window_s:
            self._battery_history.popleft()

    def _on_cmd_vel(self, msg: Twist) -> None:
        v = msg.linear
        if math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) > 0.05:
            self._has_recent_cmd = True
            self._last_cmd_t = _clock_seconds(self)

    # tick
    def _tick(self) -> None:
        now = _clock_seconds(self)
        if self._first_seen_t is None:
            # No odom yet: publish a stub so the dashboard sees the drone exists.
            self._publish_health(now, anomalies=[], unrecoverable=False, battery_remaining_s=float('nan'))
            return

        anomalies: list = []

        # IMU spike: sampled instantaneously; no time window required.
        if self._last_imu_norm >= self.imu_spike_g:
            anomalies.append('imu_spike')

        # Odom freshness.
        odom_age = now - (self._last_odom_t or now)
        if odom_age > self.odom_stale_s:
            anomalies.append('odom_stale')

        # Velocity freeze: only when commanded to move recently.
        recent_cmd = (now - self._last_cmd_t) < 2.0 and self._has_recent_cmd
        if recent_cmd and self._last_vel_norm < self.vel_freeze_m_s:
            self._vel_freeze_since = self._vel_freeze_since or now
            if now - self._vel_freeze_since >= self.vel_freeze_window_s:
                anomalies.append('vel_freeze')
        else:
            self._vel_freeze_since = None

        # Position freeze: most reliable "drone is stuck" signal. A drone
        # bouncing on the ground with motors running shows non-zero body
        # velocity (motor wash / oscillation) but its world XY barely moves.
        # Compare oldest-in-window pose vs newest; flag if the spread is
        # below `pos_freeze_radius_m` while we've been commanded to move.
        if recent_cmd and len(self._pose_history) >= 5:
            oldest_t, oldest_x, oldest_y = self._pose_history[0]
            window_dt = now - oldest_t
            if window_dt >= self.pos_freeze_window_s * 0.8:  # full window
                dx = self._last_pose_x - oldest_x
                dy = self._last_pose_y - oldest_y
                if math.hypot(dx, dy) < self.pos_freeze_radius_m:
                    anomalies.append('pos_freeze')

        # LiDAR imminent collision.
        if 0 < self._lidar_min_range < self.lidar_imminent_m:
            self._lidar_imminent_since = self._lidar_imminent_since or now
            if now - self._lidar_imminent_since >= self.lidar_imminent_window_s:
                anomalies.append('lidar_imminent')
        else:
            self._lidar_imminent_since = None

        # Battery: both the latched low flag and the curve fit count as one.
        battery_remaining_s = self._estimate_battery_remaining_s()
        if self._battery_low or (
            not math.isnan(battery_remaining_s)
            and battery_remaining_s < self.battery_remaining_threshold_s
        ):
            anomalies.append('battery_critical')

        # Decide unrecoverable. Need both anomalies AND the drone to actually
        # be on the ground (z<grounded_altitude_m). A drone bouncing off a
        # building in flight may briefly trigger anomalies and recover,
        # don't write it off.
        unrecoverable = (
            len(anomalies) >= self.unrecoverable_anomaly_count
            and self._last_pose_z < self.grounded_altitude_m
        )

        self._publish_health(now, anomalies, unrecoverable, battery_remaining_s)

        if unrecoverable and not self._damage_emitted:
            self._damage_emitted = True
            self._emit_event(
                'DRONE_DAMAGE_REPORT',
                detail=','.join(anomalies),
                severity=MissionEvent.SEVERITY_ERROR,
            )

    def _estimate_battery_remaining_s(self) -> float:
        if len(self._battery_history) < 5:
            return float('nan')
        t0, b0 = self._battery_history[0]
        t1, b1 = self._battery_history[-1]
        dt = t1 - t0
        if dt <= 0.5 or b1 >= b0:
            return float('nan')
        slope = (b0 - b1) / dt        # battery units per second (positive)
        # remaining = current_level / slope
        if slope <= 0:
            return float('nan')
        return b1 / slope

    def _publish_health(self, now: float, anomalies: list,
                        unrecoverable: bool, battery_remaining_s: float) -> None:
        msg = DroneHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.drone_name = self.drone_name
        msg.alive_for_s = (now - self._first_seen_t) if self._first_seen_t else 0.0
        msg.anomaly_score = min(1.0, len(anomalies) / 4.0)
        msg.reason = ','.join(anomalies)
        msg.unrecoverable = unrecoverable
        msg.battery_remaining_s = battery_remaining_s
        msg.imu_accel_norm = self._last_imu_norm
        msg.odom_age_s = (now - self._last_odom_t) if self._last_odom_t else float('nan')
        msg.lidar_min_range_m = self._lidar_min_range
        msg.vel_command_divergence_m_s = 0.0   # reserved; populated when we
                                               # cross-check cmd vs actual.
        self._health_pub.publish(msg)

    def _emit_event(self, event_type: str, detail: str, severity: int) -> None:
        """Emit through the EventPort. The two events this monitor emits
        (DRONE_DOWN, BATTERY_RTH) map to their typed variants; anything
        else falls through to UnknownEvent so the forward-compat path
        stays intact."""
        if getattr(self, '_event_port', None) is None:
            return
        from drone_rescue_coordination.lib.domain.events import (
            BatteryRTH, DroneDown, UnknownEvent,
        )
        base = dict(severity=severity, raw_detail=detail,
                    drone_name=self.drone_name)
        if event_type == 'DRONE_DOWN':
            variant = DroneDown(**base)
        elif event_type == 'BATTERY_RTH':
            variant = BatteryRTH(**base)
        else:
            variant = UnknownEvent(event_type=event_type, **base)
        self._event_port.emit(variant)


def main(args=None):
    rclpy.init(args=args)
    node = bind_composition(DroneHealthMonitor())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
