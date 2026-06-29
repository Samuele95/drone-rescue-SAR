#!/usr/bin/env python3
"""
Pheromone map visualizer node.

Subscribes to pheromone map data and publishes:
1. RViz markers showing coverage intensity as a colored grid overlay
2. OccupancyGrid heatmap for alternative visualization

Deliberately outside the ``OperatorView`` contract (and the only viz
node with no ``drone_rescue_ui_common`` dependency), by design. The
``PheromoneMap`` it renders is a coordination-internal stigmergy
artifact with no projection on ``MissionViewModel`` and no
operator-facing mission-state meaning: this is a pure RViz debugging
tool, not an operator view. Do NOT add a ``render_from`` / PheromoneMap
projection to ``MissionViewModel`` to make it conform; that would be
architectural creep (the dashboard does not subscribe to PheromoneMap).
Mirrors the ``drone_trails`` exclusion.
"""

from drone_rescue_msgs.msg import PheromoneMap

from geometry_msgs.msg import Point, Pose

from nav_msgs.msg import OccupancyGrid

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA

from visualization_msgs.msg import Marker, MarkerArray


class PheromoneVisualizer(Node):
    """Visualizes pheromone map as colored markers and heatmap in RViz."""

    def __init__(self):
        super().__init__('pheromone_visualizer')

        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('marker_height', 0.1)
        self.declare_parameter('marker_alpha', 0.6)
        self.declare_parameter('min_intensity_threshold', 0.05)
        self.declare_parameter('heatmap_enabled', True)
        self.declare_parameter('heatmap_resolution_factor', 2)

        self.update_rate = self.get_parameter('update_rate').value
        self.marker_height = self.get_parameter('marker_height').value
        self.marker_alpha = self.get_parameter('marker_alpha').value
        self.min_threshold = self.get_parameter('min_intensity_threshold').value
        self.heatmap_enabled = self.get_parameter('heatmap_enabled').value
        self.resolution_factor = self.get_parameter('heatmap_resolution_factor').value

        self.latest_map = None

        self.map_sub = self.create_subscription(
            PheromoneMap,
            '/pheromone/map',
            self.map_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/pheromone/visualization',
            10
        )

        self.heatmap_pub = self.create_publisher(
            OccupancyGrid,
            '/pheromone/heatmap',
            10
        )

        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_visualization
        )

        self.get_logger().info(
            f'Pheromone visualizer started (heatmap={self.heatmap_enabled})'
        )

    def map_callback(self, msg: PheromoneMap):
        """Store latest pheromone map."""
        self.latest_map = msg

    def intensity_to_color(self, intensity: float) -> ColorRGBA:
        """Convert pheromone intensity to color (blue=low, green=mid, red=high)."""
        color = ColorRGBA()
        color.a = self.marker_alpha

        if intensity < 0.33:
            # Blue to Cyan
            t = intensity / 0.33
            color.r = 0.0
            color.g = t
            color.b = 1.0
        elif intensity < 0.66:
            # Cyan to Green
            t = (intensity - 0.33) / 0.33
            color.r = 0.0
            color.g = 1.0
            color.b = 1.0 - t
        else:
            # Green to Yellow to Red
            t = (intensity - 0.66) / 0.34
            color.r = t
            color.g = 1.0 - t * 0.5
            color.b = 0.0

        return color

    def publish_visualization(self):
        """Publish marker array for pheromone map visualization."""
        if self.latest_map is None:
            return

        marker_array = MarkerArray()

        # Delete old markers
        delete_marker = Marker()
        delete_marker.header.frame_id = 'world'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = 'pheromone'
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Create cube list marker for efficient rendering
        cube_marker = Marker()
        cube_marker.header.frame_id = 'world'
        cube_marker.header.stamp = self.get_clock().now().to_msg()
        cube_marker.ns = 'pheromone'
        cube_marker.id = 1
        cube_marker.type = Marker.CUBE_LIST
        cube_marker.action = Marker.ADD
        cube_marker.scale.x = self.latest_map.resolution * 0.9
        cube_marker.scale.y = self.latest_map.resolution * 0.9
        cube_marker.scale.z = self.marker_height
        cube_marker.pose.orientation.w = 1.0

        # Add points for each cell above threshold. The active-cell
        # scan is vectorised: numpy finds the above-threshold indices
        # and computes their world coordinates in four array ops, so
        # the Python-level loop runs only over the (typically sparse)
        # active cells instead of all 40k grid cells per 2 Hz tick.
        width = self.latest_map.width
        resolution = self.latest_map.resolution
        origin_x = self.latest_map.origin.x
        origin_y = self.latest_map.origin.y

        data = np.asarray(self.latest_map.data, dtype=np.float32)
        active = np.nonzero(data > self.min_threshold)[0]
        world_x = origin_x + (active % width + 0.5) * resolution
        world_y = origin_y + (active // width + 0.5) * resolution
        z = self.marker_height / 2
        for wx, wy, intensity in zip(world_x, world_y, data[active]):
            point = Point()
            point.x = float(wx)
            point.y = float(wy)
            point.z = z
            cube_marker.points.append(point)
            cube_marker.colors.append(
                self.intensity_to_color(float(intensity))
            )

        if cube_marker.points:
            marker_array.markers.append(cube_marker)

        self.marker_pub.publish(marker_array)

        # Publish heatmap if enabled
        if self.heatmap_enabled:
            self.publish_heatmap()

    def publish_heatmap(self):
        """Publish pheromone data as OccupancyGrid heatmap."""
        if self.latest_map is None:
            return

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'world'

        # Downsample for performance
        orig_width = self.latest_map.width
        orig_height = self.latest_map.height
        factor = self.resolution_factor

        new_width = orig_width // factor
        new_height = orig_height // factor

        grid.info.resolution = self.latest_map.resolution * factor
        grid.info.width = new_width
        grid.info.height = new_height

        grid.info.origin = Pose()
        grid.info.origin.position.x = self.latest_map.origin.x
        grid.info.origin.position.y = self.latest_map.origin.y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        data = np.array(self.latest_map.data, dtype=np.float32).reshape(
            (orig_height, orig_width)
        )

        # Downsample by taking the max in each (factor x factor) block.
        # Single numpy block-max (reshape to (new_height, factor,
        # new_width, factor) then max over the two block axes) replaces
        # the former O(n x m) nested Python loop. The slice drops any
        # edge remainder, identical to the previous loop's // floor div.
        downsampled = (
            data[:new_height * factor, :new_width * factor]
            .reshape(new_height, factor, new_width, factor)
            .max(axis=(1, 3))
        )

        # Convert to occupancy values (0-100, -1 for unknown)
        # Invert: high pheromone = explored = low occupancy (for visualization)
        # Or: high pheromone = high value (depends on costmap interpretation)
        # Using direct mapping: 0 = unexplored, 100 = fully explored
        occupancy = (downsampled * 100).astype(np.int8)
        occupancy = np.clip(occupancy, 0, 100)

        # hand rclpy the numpy int8 buffer directly (C fast path);
        # .tolist() boxed 40k Python ints per 2 Hz publish. Same fix as
        # pheromone_server.
        grid.data = occupancy.flatten()

        self.heatmap_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = PheromoneVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
