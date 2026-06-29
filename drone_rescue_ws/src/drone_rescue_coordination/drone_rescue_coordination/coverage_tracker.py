#!/usr/bin/env python3
"""
Coverage Tracker Node

Monitors survey mission progress and publishes metrics including:
- Coverage percentage
- Elapsed time
- Active drones
- Victims found

Provides real-time feedback on multi-drone survey performance.
"""

import numpy as np
from typing import Dict, List

from drone_rescue_coordination.lib.domain.coverage_eta import (
    estimate_remaining, prune_window, windowed_rate,
)
from drone_rescue_coordination.lib.domain.fleet import default_drone_names_list
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point, PoseStamped

from drone_rescue_msgs.msg import PheromoneMap, DroneStatus, CoverageMetrics


class CoverageTracker(Node):
    """
    Coverage tracking and metrics node.

    Publishes:
        /coverage/percentage: Simple coverage percentage (Float32)
        /coverage/metrics: Full coverage metrics (CoverageMetrics)

    Subscribes:
        /pheromone/map: Pheromone grid for coverage calculation
        /droneN/status: Status from each drone
        /victims/detected: Victim detection events
        /survey/start: Survey start trigger
    """

    def __init__(self):
        super().__init__('coverage_tracker')

        # Declare parameters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('drone_names', default_drone_names_list())
        self.declare_parameter('coverage_threshold', 0.1)  # cell visited if pheromone > this
        self.declare_parameter('grid_width', 200)
        self.declare_parameter('grid_height', 200)
        # Mission-zone params: coverage % is reported over this disk only,
        # not the whole grid. If mission_radius <= 0, fall back to whole-grid.
        self.declare_parameter('cell_resolution', 1.0)
        self.declare_parameter('origin_x', -100.0)
        self.declare_parameter('origin_y', -100.0)
        self.declare_parameter('mission_center_x', 0.0)
        self.declare_parameter('mission_center_y', 0.0)
        self.declare_parameter('mission_radius', 0.0)  # meters; 0 = whole grid

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.drone_names = self.get_parameter('drone_names').value
        self.coverage_threshold = self.get_parameter('coverage_threshold').value
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.cell_resolution = float(self.get_parameter('cell_resolution').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.mission_center_x = float(self.get_parameter('mission_center_x').value)
        self.mission_center_y = float(self.get_parameter('mission_center_y').value)
        self.mission_radius = float(self.get_parameter('mission_radius').value)

        # Pre-compute mission-zone mask (cells whose center is within mission_radius
        # of mission_center). All-True if mission_radius <= 0.
        if self.mission_radius > 0.0:
            rows = np.arange(self.grid_height)
            cols = np.arange(self.grid_width)
            xs = self.origin_x + (cols + 0.5) * self.cell_resolution  # shape (W,)
            ys = self.origin_y + (rows + 0.5) * self.cell_resolution  # shape (H,)
            xx, yy = np.meshgrid(xs, ys)
            dx = xx - self.mission_center_x
            dy = yy - self.mission_center_y
            self.mission_mask = (dx * dx + dy * dy) <= (self.mission_radius ** 2)
        else:
            self.mission_mask = np.ones(
                (self.grid_height, self.grid_width), dtype=bool
            )
        self.mission_cells = int(np.sum(self.mission_mask))

        # State variables. Initialise pheromone_grid to a zero array of
        # the right shape rather than None: the type annotation
        # `: np.ndarray = None` was a type-lie and the
        # `if self.pheromone_grid is not None` guard at the end of
        # pheromone_callback was a no-op. The boolean
        # `_received_first_map` distinguishes "never received a map"
        # from "received an all-zeros map" for any caller that needs it.
        self.pheromone_grid: np.ndarray = np.zeros(
            (self.grid_height, self.grid_width), dtype=np.float32
        )
        self._received_first_map: bool = False
        self.visited_cells: np.ndarray = np.zeros(
            (self.grid_height, self.grid_width), dtype=bool
        )
        self.drone_statuses: Dict[str, DroneStatus] = {}
        self.victim_locations: List[Point] = []
        self.survey_started: bool = False
        self.survey_start_time = None
        # Sliding window of (elapsed_s, coverage_pct) samples for the
        # ETA's windowed rate (replaces the lagging whole-mission linear rate).
        self._coverage_samples: List = []
        self.declare_parameter('eta_window_seconds', 30.0)
        self._eta_window_s = float(
            self.get_parameter('eta_window_seconds').value)

        # QoS profiles
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Publishers
        self.percentage_pub = self.create_publisher(
            Float32,
            '/coverage/percentage',
            10
        )

        self.metrics_pub = self.create_publisher(
            CoverageMetrics,
            '/coverage/metrics',
            10
        )

        # Subscribers
        self.pheromone_sub = self.create_subscription(
            PheromoneMap,
            '/pheromone/map',
            self.pheromone_callback,
            map_qos
        )

        self.victim_sub = self.create_subscription(
            PoseStamped,
            '/victims/detected',
            self.victim_callback,
            10
        )

        self.survey_start_sub = self.create_subscription(
            Bool,
            '/survey/start',
            self.survey_start_callback,
            10
        )

        # Subscribe to each drone's status. TopicFactory replaces the
        # hand-rolled f-string + lambda late-binding closure pattern.
        from drone_rescue_coordination.lib.ros_adapter.topic_factory import (
            TopicFactory,
        )
        self._topic_factory = TopicFactory(self, self.drone_names)
        self._status_subs = self._topic_factory.per_drone_subs(
            'status', DroneStatus, self.status_callback,
        )

        # Update timer
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_callback
        )

        self.get_logger().info(
            f'Coverage tracker started, monitoring {len(self.drone_names)} drones'
        )

    def pheromone_callback(self, msg: PheromoneMap):
        """Process pheromone map update."""
        self.pheromone_grid = np.array(msg.data).reshape(
            (msg.height, msg.width)
        )
        self._received_first_map = True

        # Update visited cells (any cell that has ever had pheromone).
        # The no-op `is not None` guard was removed: pheromone_grid is
        # now initialised to zeros at __init__ time, so it's never None here.
        newly_visited = self.pheromone_grid > self.coverage_threshold
        self.visited_cells = np.logical_or(self.visited_cells, newly_visited)

    def status_callback(self, msg: DroneStatus, drone_name: str):
        """Process drone status update."""
        self.drone_statuses[drone_name] = msg

    def victim_callback(self, msg: PoseStamped):
        """Process victim detection."""
        # Check if this is a new victim (not already in list)
        for existing in self.victim_locations:
            dx = existing.x - msg.pose.position.x
            dy = existing.y - msg.pose.position.y
            if dx*dx + dy*dy < 4.0:  # Within 2m, same victim
                return

        self.victim_locations.append(msg.pose.position)
        self.get_logger().info(
            f'New victim recorded at ({msg.pose.position.x:.1f}, '
            f'{msg.pose.position.y:.1f}). Total: {len(self.victim_locations)}'
        )

    def survey_start_callback(self, msg: Bool):
        """Handle survey start trigger."""
        if msg.data and not self.survey_started:
            self.survey_started = True
            self.survey_start_time = self.get_clock().now()
            self.visited_cells.fill(False)
            self.victim_locations.clear()
            self.get_logger().info('Survey mission started - tracking coverage')

    def update_callback(self):
        """Main update loop - compute and publish metrics."""
        # Coverage is reported relative to the mission zone (mission_mask), not the
        # whole grid, otherwise drones confined to a small disk inside a large grid
        # can never approach 100 %.
        visited_in_mission = np.logical_and(self.visited_cells, self.mission_mask)
        visited_count = int(np.sum(visited_in_mission))
        coverage_pct = (visited_count / max(self.mission_cells, 1)) * 100.0

        # Publish simple percentage
        pct_msg = Float32()
        pct_msg.data = float(coverage_pct)
        self.percentage_pub.publish(pct_msg)

        # Build full metrics message
        metrics = CoverageMetrics()
        metrics.header.stamp = self.get_clock().now().to_msg()

        metrics.percentage_covered = float(coverage_pct)
        metrics.cells_visited = int(visited_count)
        metrics.total_cells = int(self.mission_cells)

        # Elapsed time
        if self.survey_start_time is not None:
            elapsed = (self.get_clock().now() - self.survey_start_time).nanoseconds / 1e9
            metrics.elapsed_time_seconds = float(elapsed)

            # ETA from a windowed coverage rate, not the lagging
            # whole-mission linear rate. Record this sample, prune the window,
            # and estimate from the recent slope.
            self._coverage_samples.append((elapsed, coverage_pct))
            prune_window(self._coverage_samples, elapsed, self._eta_window_s)
            if coverage_pct > 1.0 and elapsed > 10.0:
                rate = windowed_rate(self._coverage_samples)
                metrics.estimated_time_remaining = float(
                    estimate_remaining(coverage_pct, rate))
            else:
                metrics.estimated_time_remaining = 0.0
        else:
            metrics.elapsed_time_seconds = 0.0
            metrics.estimated_time_remaining = 0.0

        # Victims
        metrics.victims_found = len(self.victim_locations)
        metrics.victim_locations = self.victim_locations.copy()

        # Drone status
        active_drones = []
        surveying_count = 0
        returning_count = 0
        landed_count = 0

        for drone_name, status in self.drone_statuses.items():
            # States: 0=IDLE, 1=TAKEOFF, 2=SURVEYING, 3=RETURNING, 4=LANDING
            if status.state == 2:  # SURVEYING
                active_drones.append(drone_name)
                surveying_count += 1
            elif status.state == 3:  # RETURNING
                returning_count += 1
            elif status.state == 0:  # IDLE (landed)
                landed_count += 1

        metrics.active_drones = active_drones
        metrics.drones_surveying = surveying_count
        metrics.drones_returning = returning_count
        metrics.drones_landed = landed_count

        self.metrics_pub.publish(metrics)

        # Log progress periodically
        if self.survey_started and int(metrics.elapsed_time_seconds) % 30 == 0:
            self.get_logger().info(
                f'Coverage: {coverage_pct:.1f}% | '
                f'Active: {surveying_count} | '
                f'Victims: {len(self.victim_locations)} | '
                f'Time: {metrics.elapsed_time_seconds:.0f}s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CoverageTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
