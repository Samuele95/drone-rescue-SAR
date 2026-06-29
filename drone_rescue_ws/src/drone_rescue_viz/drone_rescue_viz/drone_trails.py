#!/usr/bin/env python3
"""
Drone trail visualizer node.

Subscribes to drone odometry and publishes path trails
as line strips in RViz for visualization.
"""

from collections import deque

from drone_rescue_ui_common.constants import DEFAULT_DRONE_NAMES, DRONE_COLORS

from geometry_msgs.msg import Point

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA

from visualization_msgs.msg import Marker, MarkerArray


def _hex_to_rgba(hex_color: str, alpha: float = 0.8) -> ColorRGBA:
    """``#rrggbb`` to ``ColorRGBA`` (0..1 channels)."""
    h = hex_color.lstrip('#')
    return ColorRGBA(
        r=int(h[0:2], 16) / 255.0,
        g=int(h[2:4], 16) / 255.0,
        b=int(h[4:6], 16) / 255.0,
        a=alpha,
    )


class DroneTrails(Node):
    """Visualizes drone flight paths as colored trails in RViz."""

    def __init__(self):
        super().__init__('drone_trails')

        self.declare_parameter('drone_names', list(DEFAULT_DRONE_NAMES))
        self.declare_parameter('max_trail_points', 500)
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('trail_width', 0.15)
        # Minimum distance before a new trail point is recorded.
        self.declare_parameter('min_distance', 0.3)

        self.drone_names = self.get_parameter('drone_names').value
        self.max_points = self.get_parameter('max_trail_points').value
        self.update_rate = self.get_parameter('update_rate').value
        self.trail_width = self.get_parameter('trail_width').value
        self.min_distance = self.get_parameter('min_distance').value
        # squared threshold so the ~200 Hz combined odom path skips the
        # sqrt (mirrors dashboard_app).
        self._min_distance_sq = self.min_distance ** 2

        # per-drone trail colours from the single canonical hex table
        # (constants.DRONE_COLORS), converted to ColorRGBA, so the RViz
        # trails match the Qt mission scene (previously drone2/drone3
        # were swapped here vs. the dashboard, confusing operators).
        self.drone_colors = {
            name: _hex_to_rgba(DRONE_COLORS.get(name, '#cccccc'))
            for name in self.drone_names
        }

        self.trails = {}
        self.last_positions = {}
        # monotonic per-drone append counter + last-rendered cursor so the
        # 5 Hz publish tick rebuilds a trail's point list only when a point
        # actually arrived (the bounded deque makes len() unusable for
        # this, same plateau class as the dashboard trails).
        self._appended = {}
        self._rendered = {}
        self._cached_points = {}

        self.odom_subs = []
        for drone_name in self.drone_names:
            self.trails[drone_name] = deque(maxlen=self.max_points)
            self.last_positions[drone_name] = None

            sub = self.create_subscription(
                Odometry,
                f'/{drone_name}/odom',
                lambda msg, name=drone_name: self.odom_callback(msg, name),
                10
            )
            self.odom_subs.append(sub)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/drone_trails/visualization',
            10
        )

        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_visualization
        )

        self.get_logger().info(
            f'Drone trails visualizer started for {len(self.drone_names)} drones')

    def odom_callback(self, msg: Odometry, drone_name: str):
        """Store position for trail."""
        pos = msg.pose.pose.position

        # Check minimum distance from last point. Squared comparison
        # (no sqrt). The z term stays: RViz trails are 3D, unlike the
        # dashboard's 2D top-down trail.
        if self.last_positions[drone_name] is not None:
            last = self.last_positions[drone_name]
            dist_sq = ((pos.x - last.x) ** 2 + (pos.y - last.y) ** 2
                       + (pos.z - last.z) ** 2)
            if dist_sq < self._min_distance_sq:
                return

        point = Point()
        point.x = pos.x
        point.y = pos.y
        point.z = pos.z
        self.trails[drone_name].append(point)
        self.last_positions[drone_name] = pos
        self._appended[drone_name] = self._appended.get(drone_name, 0) + 1

    def publish_visualization(self):
        """Publish marker array for trail visualization."""
        marker_array = MarkerArray()

        for i, drone_name in enumerate(self.drone_names):
            trail = self.trails[drone_name]
            if len(trail) < 2:
                continue

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'drone_trails'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = self.trail_width
            marker.pose.orientation.w = 1.0

            color = self.drone_colors.get(
                drone_name,
                ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8)
            )
            marker.color = color

            # Rebuilt only when the trail gained a point since the last
            # tick; otherwise the cached list is reused (hover/steady-state
            # ticks allocate nothing).
            count = self._appended.get(drone_name, 0)
            if self._rendered.get(drone_name) != count:
                self._cached_points[drone_name] = list(trail)
                self._rendered[drone_name] = count
            marker.points = self._cached_points[drone_name]

            marker_array.markers.append(marker)

            pos_marker = Marker()
            pos_marker.header.frame_id = 'world'
            pos_marker.header.stamp = self.get_clock().now().to_msg()
            pos_marker.ns = 'drone_positions'
            pos_marker.id = i
            pos_marker.type = Marker.SPHERE
            pos_marker.action = Marker.ADD
            pos_marker.scale.x = 0.8
            pos_marker.scale.y = 0.8
            pos_marker.scale.z = 0.4
            pos_marker.pose.orientation.w = 1.0
            pos_marker.color = color
            pos_marker.color.a = 1.0

            if trail:
                pos_marker.pose.position = trail[-1]
                marker_array.markers.append(pos_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DroneTrails()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
