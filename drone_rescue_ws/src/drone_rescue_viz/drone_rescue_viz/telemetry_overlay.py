#!/usr/bin/env python3
"""
Telemetry overlay node.

Converts ROS 2 telemetry topics into OverlayText messages for RViz.
Displays live coverage %, victim count, battery levels, and drone
health.

Folds incoming ROS messages into a local ``MissionViewModel`` and
renders OverlayText via ``render_from(view)``. The optional
``/<drone>/battery_level`` Float32 stream stays on its own callback
(separate channel from DroneStatus.battery_level) as a per-node mutable
mapping consulted during render.
"""

from typing import Dict, Optional

from drone_rescue_msgs.msg import CoverageMetrics, DroneStatus

from drone_rescue_ui_common.constants import (
    CONTROLLER_STATE_LABEL, DEFAULT_DRONE_NAMES,
)
from drone_rescue_ui_common.view_model import MissionViewModel

import rclpy
from rclpy.node import Node

from rviz_2d_overlay_msgs.msg import OverlayText

from std_msgs.msg import ColorRGBA, Float32


class TelemetryOverlay(Node):
    """
    RViz OverlayText publisher.

    Implements the ``OperatorView`` Protocol via ``render_from``;
    the publish timer dispatches to it on the current local
    MissionViewModel snapshot.
    """

    def __init__(self):
        super().__init__('telemetry_overlay')

        self.declare_parameter('drone_names', list(DEFAULT_DRONE_NAMES))
        self.drone_names = list(self.get_parameter('drone_names').value)

        # local MissionViewModel; folded by callbacks, drained by render timer.
        self._mvm = MissionViewModel()
        # ``/<drone>/battery_level`` Float32 stream, distinct from
        # DroneStatus.battery_level. Kept as a separate per-node
        # mutable mapping; consulted during render with DroneStatus
        # battery as the fallback.
        self._extra_battery: Dict[str, float] = {}

        self.mission_overlay_pub = self.create_publisher(
            OverlayText, '/telemetry/mission_overlay', 10,
        )
        self.drone_status_overlay_pub = self.create_publisher(
            OverlayText, '/telemetry/drone_status_overlay', 10,
        )

        self.create_subscription(
            CoverageMetrics, '/coverage/metrics',
            self.coverage_callback, 10,
        )
        for drone_name in self.drone_names:
            self.create_subscription(
                DroneStatus, f'/{drone_name}/status',
                lambda msg, name=drone_name: self.drone_status_callback(msg, name),
                10,
            )
            self.create_subscription(
                Float32, f'/{drone_name}/battery_level',
                lambda msg, name=drone_name: self.battery_callback(msg, name),
                10,
            )

        # 2 Hz refresh to avoid DDS pressure.
        self.timer = self.create_timer(0.5, self.publish_overlays)
        self.get_logger().info('Telemetry overlay node started')
        self.get_logger().info(f'Monitoring drones: {self.drone_names}')

    def coverage_callback(self, msg: CoverageMetrics):
        self._mvm = self._mvm.apply_coverage(msg)

    def drone_status_callback(self, msg: DroneStatus, drone_name: str):
        # use the message's own stamp instead of an extra rclpy clock
        # call per message; this node performs no recency gating (only
        # 'ever received'), and the header stamp is the ROS-time-correct
        # choice under replay.
        now = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._mvm = self._mvm.apply_drone_status(drone_name, msg, now=now)

    def battery_callback(self, msg: Float32, drone_name: str):
        # ``/<drone>/battery_level`` Float32, used in preference to
        # DroneStatus.battery when both are present (the dedicated
        # stream is higher-rate).
        self._extra_battery[drone_name] = float(msg.data)

    def publish_overlays(self):
        self.render_from(self._mvm)

    def render_from(self, view: MissionViewModel) -> None:
        """
        Paint the two OverlayText panels from the frozen snapshot.

        OperatorView Protocol implementation; also consults the
        extra-battery mapping.
        """
        self._publish_mission_overlay(view)
        self._publish_drone_status_overlay(view)

    def _publish_mission_overlay(self, view: MissionViewModel) -> None:
        overlay = OverlayText()
        overlay.action = OverlayText.ADD
        overlay.width = 400
        overlay.height = 200
        overlay.horizontal_distance = 10
        overlay.vertical_distance = 10
        overlay.horizontal_alignment = OverlayText.LEFT
        overlay.vertical_alignment = OverlayText.TOP
        overlay.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.7)
        overlay.line_width = 2
        overlay.text_size = 14.0
        overlay.font = 'DejaVu Sans Mono'
        overlay.fg_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        cov = view.coverage
        if cov.elapsed_time_seconds > 0.0 or cov.cells_visited > 0:
            minutes = int(cov.elapsed_time_seconds // 60)
            seconds = int(cov.elapsed_time_seconds % 60)
            # scan-time ETA (0.0 = not yet estimable).
            eta = getattr(cov, 'estimated_time_remaining', 0.0)
            eta_line = ''
            if eta > 0.0:
                eta_line = f'ETA Remaining: {int(eta // 60):02d}:{int(eta % 60):02d}\n'
            overlay.text = (
                '=== MISSION STATUS ===\n'
                f'Coverage:    {cov.percentage:5.1f}%\n'
                f'Cells:       {cov.cells_visited}/{cov.total_cells}\n'
                f'Victims:     {cov.victims_found}\n'
                f'Mission Time: {minutes:02d}:{seconds:02d}\n'
                f'{eta_line}'
            )
        else:
            overlay.text = (
                '=== MISSION STATUS ===\n'
                'Waiting for telemetry...\n'
            )
        self.mission_overlay_pub.publish(overlay)

    def _publish_drone_status_overlay(self, view: MissionViewModel) -> None:
        overlay = OverlayText()
        overlay.action = OverlayText.ADD
        overlay.width = 350
        overlay.height = 250
        overlay.horizontal_distance = 10
        overlay.vertical_distance = 10
        overlay.horizontal_alignment = OverlayText.RIGHT
        overlay.vertical_alignment = OverlayText.TOP
        overlay.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.7)
        overlay.line_width = 2
        overlay.text_size = 12.0
        overlay.font = 'DejaVu Sans Mono'
        overlay.fg_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        text_lines = ['=== DRONE STATUS ===']
        for drone_name in self.drone_names:
            # Prefer the Float32 dedicated stream; fall back to
            # DroneStatus.battery_level from the view-model.
            battery: Optional[float] = self._extra_battery.get(drone_name)
            d = view.drones.get(drone_name)
            if battery is None and d is not None and d.peer_last_seen > 0:
                battery = d.battery

            if battery is not None:
                battery_pct = battery * 100
                if battery > 0.5:
                    health = '[OK]'
                elif battery > 0.2:
                    health = '[WARN]'
                else:
                    health = '[LOW!]'

                if d is not None and d.peer_last_seen > 0:
                    state_str = CONTROLLER_STATE_LABEL.get(
                        d.controller_state, 'UNKNOWN')
                else:
                    state_str = 'UNKNOWN'

                text_lines.append(
                    f'{drone_name:6s}: {battery_pct:5.1f}% {health:6s} {state_str}'
                )
            else:
                text_lines.append(f'{drone_name:6s}: --- %')

        overlay.text = '\n'.join(text_lines)
        self.drone_status_overlay_pub.publish(overlay)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
