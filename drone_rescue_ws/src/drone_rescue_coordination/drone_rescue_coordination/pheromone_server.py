#!/usr/bin/env python3
"""
Pheromone Server Node: 3T Architecture Behavioural Layer (L1)
infrastructure, the stigmergic medium (Marcelletti slides pp. 123-126).

Per the slides' swarm-robotics chapter: "desired collective behaviour
emerges from the interaction between the robots and the interaction of
robots with the environment". This node is the shared environment
medium, the stigmergy substrate the Surveyor (L1) reads via the
``StigmergyPort``. No deliberation, no executive logic; pure decay +
broadcast.

Central server for stigmergic coordination of multi-drone survey.
Maintains a 2D grid of pheromone values that decay over time.
Drones deposit pheromones as they survey, creating a shared map
that enables emergent coordination without direct communication.
"""

import threading
import numpy as np
from typing import Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, Duration
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks

import diagnostic_updater
import diagnostic_msgs.msg

from std_msgs.msg import Float32
from geometry_msgs.msg import Point, PointStamped
from std_srvs.srv import Empty

from drone_rescue_msgs.msg import PheromoneMap


class PheromoneServer(LifecycleNode):
    """
    Pheromone grid server for stigmergic coordination.

    Maintains a 2D grid where:
    - Drones deposit pheromone (1.0) at their current position
    - Pheromone decays over time (multiplied by decay_rate each tick)
    - Drones read the grid to navigate away from high-pheromone areas

    Publishes:
        /pheromone/map: Full pheromone grid (PheromoneMap)
        /pheromone/coverage: Percentage of cells visited (Float32)

    Subscribes:
        /pheromone/deposit: Deposit requests from drones (PointStamped)

    Services:
        /pheromone/reset: Reset grid to zeros
    """

    def __init__(self):
        super().__init__('pheromone_server')

        # Declare parameters
        self.declare_parameter('grid_width', 200)  # cells
        self.declare_parameter('grid_height', 200)  # cells
        self.declare_parameter('cell_resolution', 1.0)  # meters per cell
        self.declare_parameter('origin_x', -100.0)  # world X of grid origin
        self.declare_parameter('origin_y', -100.0)  # world Y of grid origin
        self.declare_parameter('decay_rate', 0.995)  # per tick
        self.declare_parameter('update_rate', 2.0)  # Hz
        self.declare_parameter('deposit_value', 1.0)
        self.declare_parameter('coverage_threshold', 0.1)  # cell considered visited if > this
        self.declare_parameter('deposit_radius_cells', 3)  # Gaussian stamp radius
        self.declare_parameter('deposit_sigma_cells', 1.5)  # Gaussian sigma in cells

        # Get parameters
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.cell_resolution = self.get_parameter('cell_resolution').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.decay_rate = self.get_parameter('decay_rate').value
        self.update_rate = self.get_parameter('update_rate').value
        self.deposit_value = self.get_parameter('deposit_value').value
        self.coverage_threshold = self.get_parameter('coverage_threshold').value
        self.deposit_radius_cells = int(self.get_parameter('deposit_radius_cells').value)
        self.deposit_sigma_cells = float(self.get_parameter('deposit_sigma_cells').value)

        # Pre-compute Gaussian stamp kernel (depends only on params)
        r = self.deposit_radius_cells
        rr, cc = np.ogrid[-r:r + 1, -r:r + 1]
        kernel = np.exp(-(rr ** 2 + cc ** 2) / (2.0 * self.deposit_sigma_cells ** 2))
        kernel *= self.deposit_value / kernel.max()
        self._deposit_kernel = kernel.astype(np.float32)

        # Initialize pheromone grid
        self.grid = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)

        # Track cells that have ever been visited (for coverage calculation)
        self.visited_mask = np.zeros((self.grid_height, self.grid_width), dtype=bool)

        # Lock for thread-safe grid access
        self._grid_lock = threading.Lock()

        # QoS for reliable map publishing (store for on_configure)
        #
        # No `deadline` policy: this node runs with use_sim_time=True, so its
        # publish timer ticks on simulation time, but DDS enforces a deadline
        # against wall-clock time. Whenever the sim runs below ~0.5x real-time
        # (expected with software-rendered Gazebo) a wall-clock deadline is
        # structurally impossible to meet even though publishing is on schedule
        # in sim time. Staleness is monitored correctly, in sim time, by the
        # `check_data_freshness` diagnostic instead.
        self.map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
            lifespan=Duration(seconds=5.0)   # Data stale after 5s
        )

        # QoS for coverage publisher
        self.coverage_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Initialize publishers/subscribers/services/timers to None (created in on_configure)
        self.map_pub = None
        self.coverage_pub = None
        self.deposit_sub = None
        self.reset_srv = None
        self.update_timer = None

        # Active state flag for timer callback guard
        self.is_active = False

        # Diagnostic updater (initialized properly in on_configure)
        self.updater = None
        self.last_update_time = None  # Track last grid update for staleness

        # QoS event tracking flags
        self.incompatible_qos_detected = False

        self.get_logger().info(
            f'Pheromone server initialized: {self.grid_width}x{self.grid_height} grid, '
            f'{self.cell_resolution}m resolution, origin=({self.origin_x}, {self.origin_y})'
        )

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure lifecycle node - create publishers, subscribers, services, timers."""
        self.get_logger().info('Configuring pheromone server...')

        # Create event callbacks for QoS monitoring
        map_pub_callbacks = PublisherEventCallbacks()
        map_pub_callbacks.incompatible_qos = self._map_incompatible_qos_callback

        # Create lifecycle publishers
        self.map_pub = self.create_publisher(
            PheromoneMap,
            '/pheromone/map',
            self.map_qos,
            event_callbacks=map_pub_callbacks
        )

        self.coverage_pub = self.create_publisher(
            Float32,
            '/pheromone/coverage',
            self.coverage_qos
        )

        # Create subscriptions
        self.deposit_sub = self.create_subscription(
            PointStamped,
            '/pheromone/deposit',
            self.deposit_callback,
            10
        )

        # Create services
        self.reset_srv = self.create_service(
            Empty,
            '/pheromone/reset',
            self.reset_callback
        )

        # Create timer
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_callback
        )

        # Initialize diagnostic updater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID(f'{self.get_namespace()}/pheromone-grid')

        # Add diagnostic tasks
        self.updater.add('Grid Health', self.check_grid_health)
        self.updater.add('Data Freshness', self.check_data_freshness)

        self.get_logger().info('Pheromone server configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate lifecycle node - enable publishing."""
        self.get_logger().info('Activating pheromone server...')
        self.is_active = True
        self.get_logger().info('Pheromone server activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate lifecycle node - disable publishing."""
        self.get_logger().info('Deactivating pheromone server...')
        self.is_active = False

        # Force diagnostic update to report inactive status
        if self.updater:
            self.updater.force_update()

        self.get_logger().info('Pheromone server deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup lifecycle node - destroy resources."""
        self.get_logger().info('Cleaning up pheromone server...')

        # Destroy resources in reverse creation order
        try:
            if self.update_timer is not None:
                self.destroy_timer(self.update_timer)
                self.update_timer = None
        except Exception as e:
            self.get_logger().warning(f'Error destroying update_timer: {e}')

        try:
            if self.reset_srv is not None:
                self.destroy_service(self.reset_srv)
                self.reset_srv = None
        except Exception as e:
            self.get_logger().warning(f'Error destroying reset_srv: {e}')

        try:
            if self.deposit_sub is not None:
                self.destroy_subscription(self.deposit_sub)
                self.deposit_sub = None
        except Exception as e:
            self.get_logger().warning(f'Error destroying deposit_sub: {e}')

        try:
            if self.coverage_pub is not None:
                self.destroy_publisher(self.coverage_pub)
                self.coverage_pub = None
        except Exception as e:
            self.get_logger().warning(f'Error destroying coverage_pub: {e}')

        try:
            if self.map_pub is not None:
                self.destroy_publisher(self.map_pub)
                self.map_pub = None
        except Exception as e:
            self.get_logger().warning(f'Error destroying map_pub: {e}')

        self.get_logger().info('Pheromone server cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown lifecycle node."""
        self.get_logger().info('Shutting down pheromone server...')
        return TransitionCallbackReturn.SUCCESS

    def _map_incompatible_qos_callback(self, event):
        """Called when subscriber has incompatible QoS."""
        self.incompatible_qos_detected = True
        self.get_logger().error(
            f'Incompatible QoS detected on /pheromone/map topic! '
            f'Subscriber with incompatible profile detected. '
            f'last_policy_kind={event.last_policy_kind}, total_count={event.total_count}'
        )

    def check_grid_health(self, stat):
        """Diagnostic callback for grid health."""
        if not self.is_active:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE,
                        'Node inactive')
        else:
            # Check if grid has reasonable values
            with self._grid_lock:
                max_val = np.max(self.grid)
                active_cells = np.sum(self.grid > 0.001)

            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                        'Grid operational')
            stat.add('Grid size', f'{self.grid_width}x{self.grid_height}')
            stat.add('Active cells', str(int(active_cells)))
            stat.add('Max pheromone', f'{max_val:.3f}')
        return stat

    def check_data_freshness(self, stat):
        """Check for stale grid updates."""
        if not self.is_active:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE,
                        'Node inactive')
            return stat

        if self.last_update_time is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                        'No updates yet')
            stat.add('Age (seconds)', 'N/A')
            return stat

        age = self.get_clock().now() - self.last_update_time
        age_sec = age.nanoseconds / 1e9

        # Threshold: 2x update period is stale
        stale_threshold = 2.0 / self.update_rate

        if age_sec > stale_threshold:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                        f'Data stale: {age_sec:.1f}s old')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                        'Data fresh')
        stat.add('Age (seconds)', f'{age_sec:.2f}')
        stat.add('Stale threshold', f'{stale_threshold:.2f}')
        return stat

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Delegate to canonical lib helper."""
        from drone_rescue_coordination.lib.grid_utils import world_to_grid
        return world_to_grid(
            x, y, self.origin_x, self.origin_y, self.cell_resolution,
        )

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Delegate to canonical lib helper."""
        from drone_rescue_coordination.lib.grid_utils import grid_to_world
        return grid_to_world(
            row, col, self.origin_x, self.origin_y, self.cell_resolution,
        )

    def is_valid_cell(self, row: int, col: int) -> bool:
        """Bounds check, local because grid_height/grid_width are
        per-instance state."""
        return 0 <= row < self.grid_height and 0 <= col < self.grid_width

    def deposit_callback(self, msg: PointStamped):
        """Handle pheromone deposit request from a drone."""
        row, col = self.world_to_grid(msg.point.x, msg.point.y)

        if self.is_valid_cell(row, col):
            with self._grid_lock:
                self._gaussian_stamp(row, col)
        else:
            self.get_logger().warning(
                f'Deposit outside grid bounds: ({msg.point.x:.1f}, {msg.point.y:.1f})',
                throttle_duration_sec=5.0
            )

    def _gaussian_stamp(self, row: int, col: int) -> None:
        """Stamp the pre-computed Gaussian kernel onto the grid (in-place max).

        Caller must hold self._grid_lock. Updates visited_mask for any cell whose
        kernel value exceeds coverage_threshold.
        """
        r = self.deposit_radius_cells
        r0 = max(0, row - r)
        r1 = min(self.grid_height, row + r + 1)
        c0 = max(0, col - r)
        c1 = min(self.grid_width, col + r + 1)
        kr0 = r0 - (row - r)
        kc0 = c0 - (col - r)
        sub_kernel = self._deposit_kernel[kr0:kr0 + (r1 - r0), kc0:kc0 + (c1 - c0)]
        self.grid[r0:r1, c0:c1] = np.maximum(self.grid[r0:r1, c0:c1], sub_kernel)
        self.visited_mask[r0:r1, c0:c1] |= (sub_kernel > self.coverage_threshold)

    def reset_callback(self, request, response):
        """Reset pheromone grid to zeros."""
        with self._grid_lock:
            self.grid.fill(0.0)
            self.visited_mask.fill(False)
        self.get_logger().info('Pheromone grid reset')
        return response

    def update_callback(self):
        """Main update loop - decay pheromones and publish map."""
        # Guard against inactive state - timers fire even when inactive
        if not self.is_active:
            return

        with self._grid_lock:
            # Apply decay
            self.grid *= self.decay_rate

            # Clamp very small values to zero
            self.grid[self.grid < 0.001] = 0.0

        self.last_update_time = self.get_clock().now()

        # Publish map
        self.publish_map()

        # Publish coverage
        self.publish_coverage()

    def publish_map(self):
        """Publish current pheromone map."""
        msg = PheromoneMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        msg.width = self.grid_width
        msg.height = self.grid_height
        msg.resolution = self.cell_resolution

        msg.origin = Point()
        msg.origin.x = self.origin_x
        msg.origin.y = self.origin_y
        msg.origin.z = 0.0

        # Flatten grid to 1D array (row-major order).
        # Pass the numpy float32 view directly (rclpy accepts ndarrays
        # for float32[] fields). Replaces `.flatten().tolist()` which
        # materialised 40k Python floats per publish at 2 Hz (~640 KB/s
        # of throwaway PyObject headers). `.ravel().astype(np.float32,
        # copy=False)` is a zero-copy view when the grid is already
        # float32; copies once if the dtype was widened to float64.
        with self._grid_lock:
            msg.data = self.grid.ravel().astype(np.float32, copy=False)

        self.map_pub.publish(msg)

    def publish_coverage(self):
        """Publish coverage percentage."""
        total_cells = self.grid_width * self.grid_height
        with self._grid_lock:
            visited_cells = np.sum(self.visited_mask)

        coverage = (visited_cells / total_cells) * 100.0

        msg = Float32()
        msg.data = float(coverage)
        self.coverage_pub.publish(msg)

    def get_local_pheromones(self, x: float, y: float, radius: int = 1) -> List[Tuple[int, int, float]]:
        """
        Get pheromone values in cells around a position.

        Args:
            x, y: World coordinates
            radius: Number of cells around center (1 = 3x3 grid)

        Returns:
            List of (row, col, pheromone_value) tuples
        """
        center_row, center_col = self.world_to_grid(x, y)
        results = []

        # Read the shared grid under _grid_lock, like every other grid
        # access (deposit/decay run on the timer thread and mutate
        # self.grid in place). The previous unlocked read could observe a
        # half-applied decay/normalisation.
        with self._grid_lock:
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    row = center_row + dr
                    col = center_col + dc
                    if self.is_valid_cell(row, col):
                        results.append((row, col, self.grid[row, col]))

        return results


def main(args=None):
    rclpy.init(args=args)
    node = PheromoneServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
