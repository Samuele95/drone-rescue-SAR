#!/usr/bin/env python3
"""
Readiness Coordinator Node. 3T Architecture: Executive Layer (L2).

The pre-mission admission gate. This node is part of the project's L2
(Executive Layer) implementation per the 3T taxonomy (Marcelletti,
p. 38, "monitoring and handling exceptions"). The admission gate is
the executive layer's first exception-handling duty: the deliberative
planner is not allowed to dispatch tasks until the L2 readiness gate
confirms L1 is ready.

Event-driven survey start coordinator that monitors drone odometry
and triggers survey when all drones are ready. Replaces the unreliable
hardcoded 120-second delay approach.
"""

from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from drone_rescue_msgs.msg import FleetReadiness, DroneReadiness

# Readiness threshold logic lifted to lib/readiness.py.
# ``DroneReadinessState`` replaces the local ``DroneState`` (the
# previous name collided with the operating-mode Enum in
# lib/domain/drone_state.py).
from drone_rescue_coordination.lib.readiness import (
    DroneReadinessState, ReadinessPolicy,
)
from drone_rescue_coordination.lib.composition import (
    bind_composition, resolve_clock,
)


class ReadinessCoordinator(Node):
    """
    Monitors drone odometry and triggers survey when all drones ready.

    This node provides event-driven survey startup instead of hardcoded delays.
    It ensures all drones have valid odometry before triggering the survey,
    making the simulation reliable on both fast and slow machines.

    Parameters:
        drone_names: List of drone names to monitor (default: ['drone1', 'drone2'])
        min_odom_count: Minimum odometry messages required per drone (default: 10)
        odom_timeout: Maximum age of odometry data in seconds (default: 2.0)
        min_ready_duration: All drones must be ready for this long (default: 5.0)
    """

    # composition kwarg; falls back to lazy adapter construction when None.
    def __init__(self, *, composition=None):
        super().__init__('readiness_coordinator')
        self._composition = composition

        # Clock port via resolve_clock helper.
        self._time = resolve_clock(self, self._composition)

        # Declare parameters
        self.declare_parameter('drone_names', ['drone1', 'drone2'])
        self.declare_parameter('min_odom_count', 10)
        self.declare_parameter('odom_timeout', 2.0)
        self.declare_parameter('min_ready_duration', 5.0)

        # Get parameters
        self.drone_names: List[str] = self.get_parameter('drone_names').value
        # Bundle the threshold knobs into a frozen ReadinessPolicy VO.
        # Individual attributes remain accessible for the readiness check.
        self._policy = ReadinessPolicy(
            min_odom_count=self.get_parameter('min_odom_count').value,
            odom_timeout=self.get_parameter('odom_timeout').value,
            min_ready_duration=self.get_parameter('min_ready_duration').value,
        )
        self.min_odom_count = self._policy.min_odom_count
        self.odom_timeout = self._policy.odom_timeout
        self.min_ready_duration = self._policy.min_ready_duration

        # Initialize drone tracking
        self.drones: Dict[str, DroneReadinessState] = {
            name: DroneReadinessState() for name in self.drone_names
        }

        # Track when all drones first became ready
        self._all_ready_since: float = 0.0
        self._survey_started: bool = False

        # QoS profile for sensor data (matches drone_controller)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribe to each drone's odometry
        self._odom_subs = []
        for drone_name in self.drone_names:
            sub = self.create_subscription(
                Odometry,
                f'/{drone_name}/odom',
                lambda msg, name=drone_name: self.odom_callback(msg, name),
                sensor_qos
            )
            self._odom_subs.append(sub)

        # Publisher for survey start. TRANSIENT_LOCAL so late-joining
        # subscribers (notably mission_manager, which finishes lifecycle
        # configuration after readiness fires) receive the latched value.
        survey_start_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.survey_start_pub = self.create_publisher(
            Bool, '/survey/start', survey_start_qos,
        )

        # Publisher for fleet readiness status
        self.readiness_pub = self.create_publisher(
            FleetReadiness, '/fleet/readiness', 10
        )

        # Readiness check timer (2 Hz)
        self.check_timer = self.create_timer(0.5, self.check_readiness)

        # Status publish timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_readiness)

        self.get_logger().info(
            f'Readiness coordinator started, monitoring {len(self.drone_names)} drones: '
            f'{self.drone_names}'
        )

    def odom_callback(self, msg: Odometry, drone_name: str):
        """Process odometry from a drone."""
        if drone_name not in self.drones:
            return

        drone = self.drones[drone_name]
        drone.odom_count += 1
        drone.last_odom_time = self._time.now_sec()

    def is_drone_ready(self, drone: DroneReadinessState) -> bool:
        """Check if a drone is ready for survey.

        Delegates to ReadinessPolicy."""
        return self._policy.is_drone_ready(drone, self._time.now_sec())

    def check_readiness(self):
        """Check if all drones are ready and trigger survey if so."""
        if self._survey_started:
            return

        current_time = self._time.now_sec()

        # Update readiness status for each drone
        all_ready = True
        for name, drone in self.drones.items():
            drone.ready = self.is_drone_ready(drone)
            if not drone.ready:
                all_ready = False

        if all_ready:
            # Track how long all drones have been ready
            if self._all_ready_since == 0.0:
                self._all_ready_since = current_time
                self.get_logger().info(
                    f'All {len(self.drone_names)} drones reporting odometry...'
                )

            ready_duration = current_time - self._all_ready_since

            if ready_duration >= self.min_ready_duration:
                self.get_logger().info(
                    f'All drones ready for {ready_duration:.1f}s - STARTING SURVEY!'
                )
                self._survey_started = True

                # Publish survey start (multiple times to ensure reception)
                msg = Bool(data=True)
                for _ in range(3):
                    self.survey_start_pub.publish(msg)
        else:
            # Reset ready timer if any drone becomes not ready
            if self._all_ready_since > 0.0:
                self.get_logger().warn('Drone readiness lost - waiting...')
            self._all_ready_since = 0.0

    def publish_readiness(self):
        """Publish fleet readiness status."""
        msg = FleetReadiness()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.all_ready = all(d.ready for d in self.drones.values())
        msg.survey_started = self._survey_started

        for name, drone in self.drones.items():
            dr = DroneReadiness()
            dr.drone_id = name
            dr.odometry_received = drone.odom_count > 0
            dr.odom_count = drone.odom_count
            dr.ready = drone.ready
            msg.drones.append(dr)

        self.readiness_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = bind_composition(ReadinessCoordinator())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
