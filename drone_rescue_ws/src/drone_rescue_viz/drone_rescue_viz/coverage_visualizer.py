#!/usr/bin/env python3
"""
Coverage metrics visualizer node.

Subscribes to coverage metrics and displays them as text markers in
RViz for real-time monitoring.

Folds incoming ROS messages into a local ``MissionViewModel`` and
renders RViz markers via ``render_from(view)``. Closes the
parallel-projection drift with the dashboard.
"""

from drone_rescue_msgs.msg import CoverageMetrics, DroneStatus

from drone_rescue_ui_common.constants import (
    CONTROLLER_STATE_LABEL, DEFAULT_DRONE_NAMES,
)
from drone_rescue_ui_common.view_model import MissionViewModel

import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA

from visualization_msgs.msg import Marker, MarkerArray


class CoverageVisualizer(Node):
    """
    Visualizes coverage metrics and drone status in RViz.

    Implements the ``OperatorView`` Protocol: ``render_from(view)``
    draws the MarkerArray from a frozen ``MissionViewModel`` snapshot.
    """

    def __init__(self):
        super().__init__('coverage_visualizer')

        self.declare_parameter('text_height', 25.0)
        self.declare_parameter('text_scale', 1.5)
        self.declare_parameter('drone_names', list(DEFAULT_DRONE_NAMES))

        self.text_height = self.get_parameter('text_height').value
        self.text_scale = self.get_parameter('text_scale').value
        self.drone_names = list(self.get_parameter('drone_names').value)

        # local MissionViewModel; folded by callbacks, drained by render timer.
        self._mvm = MissionViewModel()

        self.create_subscription(
            CoverageMetrics, '/coverage/metrics',
            self.metrics_callback, 10,
        )
        for drone_name in self.drone_names:
            self.create_subscription(
                DroneStatus, f'/{drone_name}/status',
                lambda msg, name=drone_name: self.status_callback(msg, name),
                10,
            )

        self.marker_pub = self.create_publisher(
            MarkerArray, '/coverage/visualization', 10,
        )

        self.timer = self.create_timer(0.5, self.publish_visualization)
        self.get_logger().info('Coverage visualizer started')

    def metrics_callback(self, msg: CoverageMetrics):
        self._mvm = self._mvm.apply_coverage(msg)

    def status_callback(self, msg: DroneStatus, drone_name: str):
        # use the message's own stamp instead of an extra rclpy clock
        # call per message; this node performs no recency gating (only
        # 'ever received'), and the header stamp is the ROS-time-correct
        # choice under replay.
        now = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._mvm = self._mvm.apply_drone_status(drone_name, msg, now=now)

    def publish_visualization(self):
        self.render_from(self._mvm)

    def render_from(self, view: MissionViewModel) -> None:
        """
        Paint the MarkerArray from the frozen view snapshot.

        OperatorView Protocol implementation.
        """
        marker_array = MarkerArray()
        marker_id = 0

        # Coverage metrics panel (top-left)
        cov = view.coverage
        # Treat "zero-elapsed + zero-cells" as "no metrics yet" to
        # match the legacy behaviour where latest_metrics was None.
        if cov.elapsed_time_seconds > 0.0 or cov.cells_visited > 0:
            text_lines = [
                '=== MISSION STATUS ===',
                f'Coverage: {cov.percentage:.1f}%',
                f'Active Drones: {cov.drones_surveying}',
                f'Cells Covered: {cov.cells_visited}/{cov.total_cells}',
                f'Mission Time: {cov.elapsed_time_seconds:.0f}s',
            ]
            if cov.victims_found > 0:
                text_lines.append(f'Victims Found: {cov.victims_found}')
            metrics_text = '\n'.join(text_lines)

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'coverage_metrics'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = -90.0
            marker.pose.position.y = 90.0
            marker.pose.position.z = self.text_height
            marker.pose.orientation.w = 1.0
            marker.scale.z = self.text_scale
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            marker.text = metrics_text
            marker_array.markers.append(marker)

            bar_marker = self._create_progress_bar(
                marker_id, cov.percentage / 100.0,
                -90.0, 85.0, self.text_height - 2,
            )
            marker_array.markers.append(bar_marker)
            marker_id += 1

        # Drone status panel (top-right)
        status_lines = ['=== DRONE STATUS ===']
        for drone_name in self.drone_names:
            d = view.drones.get(drone_name)
            if d is not None and d.peer_last_seen > 0:
                state_str = CONTROLLER_STATE_LABEL.get(
                    d.controller_state, 'UNKNOWN')
                battery_pct = d.battery * 100
                status_lines.append(
                    f'{drone_name}: {state_str} | Bat: {battery_pct:.0f}%'
                )
            else:
                status_lines.append(f'{drone_name}: No data')

        status_marker = Marker()
        status_marker.header.frame_id = 'world'
        status_marker.header.stamp = self.get_clock().now().to_msg()
        status_marker.ns = 'drone_status'
        status_marker.id = marker_id
        marker_id += 1
        status_marker.type = Marker.TEXT_VIEW_FACING
        status_marker.action = Marker.ADD
        status_marker.pose.position.x = 90.0
        status_marker.pose.position.y = 90.0
        status_marker.pose.position.z = self.text_height
        status_marker.pose.orientation.w = 1.0
        status_marker.scale.z = self.text_scale * 0.8
        status_marker.color = ColorRGBA(r=0.8, g=1.0, b=0.8, a=1.0)
        status_marker.text = '\n'.join(status_lines)
        marker_array.markers.append(status_marker)

        self.marker_pub.publish(marker_array)

    def _create_progress_bar(self, marker_id: int, progress: float,
                             x: float, y: float, z: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'progress_bar'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        bar_width = 20.0
        bar_height = 0.8
        bar_depth = 0.3

        marker.pose.position.x = x + (bar_width * progress) / 2
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = bar_width * max(progress, 0.01)
        marker.scale.y = bar_height
        marker.scale.z = bar_depth

        # Red to yellow to green ramp.
        if progress < 0.5:
            marker.color = ColorRGBA(r=1.0, g=progress * 2, b=0.0, a=0.8)
        else:
            marker.color = ColorRGBA(r=1.0 - (progress - 0.5) * 2, g=1.0, b=0.0, a=0.8)

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = CoverageVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
