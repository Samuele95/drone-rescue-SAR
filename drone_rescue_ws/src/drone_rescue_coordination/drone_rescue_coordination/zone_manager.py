#!/usr/bin/env python3
"""
No-Fly Zone Manager Node

Manages restricted flight zones and provides:
- Zone boundary checking for drones
- Visualization markers for RViz
- Warning alerts when drones approach restricted areas
- Dynamic zone updates
"""

import yaml
from typing import List, Dict, Tuple, Optional

import numpy as np

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from std_msgs.msg import Bool, String, ColorRGBA
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from ament_index_python.packages import get_package_share_directory


# NoFlyZone VO lives in lib/domain/value_objects.py. Re-exported here so
# legacy ``from zone_manager import NoFlyZone`` importers (tests, other
# adapters) keep working.
from drone_rescue_coordination.lib.domain.value_objects import (  # noqa: F401
    NoFlyZone, ZoneShape, ZonePriority,
)
# An unconditionally-needed import belongs at module top, not in
# __init__, so a missing symbol fails at package load rather than at the
# first ZoneManager().
from drone_rescue_coordination.lib.ros_adapter.topic_factory import (
    QosName, TopicFactory,
)


# Priority to RViz marker colour mapping. Table-driven; unknown
# priorities fall through to ``low`` (green). Keyed on the ZonePriority
# str-Enum (NoFlyZone.priority is coerced to ZonePriority); a raw-string
# lookup still resolves via the str-Enum, but the canonical keys make a
# future enum rename fail loudly.
_PRIORITY_COLOR: Dict[ZonePriority, Tuple[float, float, float]] = {
    ZonePriority.CRITICAL: (1.0, 0.0, 0.0),
    ZonePriority.HIGH:     (1.0, 0.5, 0.0),
    ZonePriority.MEDIUM:   (1.0, 1.0, 0.0),
    ZonePriority.LOW:      (0.0, 1.0, 0.0),
}


class ZoneManager(LifecycleNode):
    """
    No-fly zone management node.

    Publishes:
        /zones/markers: MarkerArray - Visualization markers
        /zones/violation: String - Zone violation alerts
        /<drone>/zone_warning: Bool - Per-drone warning flag

    Subscribes:
        /<drone>/odom: Odometry - Drone position for checking
    """

    def __init__(self):
        super().__init__('zone_manager')

        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('drone_names', ['drone1'])
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('visualization_enabled', True)
        self.declare_parameter('warning_distance', 5.0)

        # Get parameters
        config_file = self.get_parameter('config_file').value
        self.drone_names = self.get_parameter('drone_names').value
        self.update_rate = self.get_parameter('update_rate').value
        self.visualization_enabled = self.get_parameter('visualization_enabled').value
        self.warning_distance = self.get_parameter('warning_distance').value

        # Load zones from config
        self.zones: List[NoFlyZone] = []
        # Per-zone numpy precompute keyed by zone.name. Populated by
        # `_compute_zone_np_state` at load time so the 10 Hz check loop
        # reads vectorised state, not Python lists.
        self._zone_np: Dict[str, Dict] = {}
        if config_file:
            self._load_zones(config_file)
        else:
            # Try default location
            try:
                pkg_dir = get_package_share_directory('drone_rescue_bringup')
                default_config = f'{pkg_dir}/config/no_fly_zones.yaml'
                self._load_zones(default_config)
            except Exception as e:
                self.get_logger().warn(f'Could not load default config: {e}')

        # Drone positions
        self.drone_positions: Dict[str, Pose] = {}

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/zones/markers',
            10
        )

        self.violation_pub = self.create_publisher(
            String,
            '/zones/violation',
            10
        )

        # TopicFactory centralises per-drone subscription wiring.
        # Default odom QoS is `SENSOR` (BEST_EFFORT, depth=10), which
        # matches what zone checks need; slightly different from the
        # legacy default-`10` which was RELIABLE. The check-zones loop
        # only cares about the latest position, so BEST_EFFORT is the
        # right fit.
        self._topic_factory = TopicFactory(self, self.drone_names)
        self.warning_pubs = self._topic_factory.per_drone_pubs(
            'zone_warning', Bool,
        )
        # Preserve legacy `10` default semantics (RELIABLE, VOLATILE,
        # depth=10). The registry's SENSOR default is BEST_EFFORT;
        # SENSOR_RELIABLE matches legacy. The check-zones loop only
        # cares about the latest pose so the delivery guarantee is
        # overkill but preserved for safety.
        self.odom_subs = self._topic_factory.per_drone_subs(
            'odom', Odometry, self.odom_callback,
            qos_override=QosName.SENSOR_RELIABLE,
        )

        # Timers
        self.check_timer = self.create_timer(
            1.0 / self.update_rate,
            self.check_zones_callback
        )

        if self.visualization_enabled:
            self.viz_timer = self.create_timer(
                1.0,  # 1 Hz for visualization
                self.publish_markers
            )

        self.get_logger().info(
            f'Zone manager started with {len(self.zones)} zones'
        )

    # LifecycleNode protocol callbacks. The class advertises
    # `<node>/change_state` and `<node>/get_state` services so
    # `lifecycle_manager` can coordinate it through SAFE-mode entry
    # alongside the other coordination-layer lifecycle nodes. Publisher,
    # subscription and timer creation stays in __init__ for now; a
    # follow-up could migrate them to on_configure for true managed
    # bootstrap.
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

    def _load_zones(self, config_file: str):
        """Load no-fly zones from YAML config file."""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            zones_config = config.get('no_fly_zones', [])

            for zone_cfg in zones_config:
                # frozen NoFlyZone validates at construction; catch and
                # log so a single malformed zone doesn't take down the
                # whole load step.
                try:
                    zone = NoFlyZone(
                        name=zone_cfg['name'],
                        zone_type=zone_cfg['type'],
                        priority=zone_cfg.get('priority', 'medium'),
                        reason=zone_cfg.get('reason', ''),
                        vertices=tuple(
                            (v[0], v[1])
                            for v in zone_cfg.get('vertices', [])
                        ),
                        center=tuple(zone_cfg['center']) if 'center' in zone_cfg else None,
                        radius=zone_cfg.get('radius'),
                        min_altitude=zone_cfg.get('min_altitude', 0.0),
                        max_altitude=zone_cfg.get('max_altitude', 100.0),
                        buffer_distance=zone_cfg.get('buffer_distance', 2.0),
                    )
                except ValueError as e:
                    self.get_logger().warn(f'Skipping malformed zone: {e}')
                    continue
                self.zones.append(zone)
                self._compute_zone_np_state(zone)
                self.get_logger().info(f'Loaded zone: {zone.name}')

            # Load global settings
            global_settings = config.get('global_settings', {})
            self.warning_distance = global_settings.get(
                'warning_distance', self.warning_distance
            )

        except Exception as e:
            self.get_logger().error(f'Failed to load zones config: {e}')

    def odom_callback(self, msg: Odometry, drone_name: str):
        """Store drone position."""
        self.drone_positions[drone_name] = msg.pose.pose

    def check_zones_callback(self):
        """Check all drones against all zones."""
        for drone_name, pose in self.drone_positions.items():
            position = (pose.position.x, pose.position.y)
            altitude = pose.position.z

            in_warning = False
            in_violation = False
            violated_zone = None

            for zone in self.zones:
                # Check if drone is in zone
                distance = self._distance_to_zone(position, zone)

                # Check altitude
                in_altitude_range = (
                    zone.min_altitude <= altitude <= zone.max_altitude
                )

                if distance <= 0 and in_altitude_range:
                    # Inside zone - violation
                    in_violation = True
                    violated_zone = zone
                    break
                elif distance <= self.warning_distance and in_altitude_range:
                    # Near zone - warning
                    in_warning = True

            # Publish warning
            warning_msg = Bool()
            warning_msg.data = in_warning or in_violation
            if drone_name in self.warning_pubs:
                self.warning_pubs[drone_name].publish(warning_msg)

            # Publish violation
            if in_violation and violated_zone:
                alert_msg = String()
                alert_msg.data = (
                    f'ZONE_VIOLATION: {drone_name} in {violated_zone.name} '
                    f'({violated_zone.reason})'
                )
                self.violation_pub.publish(alert_msg)
                self.get_logger().warn(alert_msg.data)

    # numpy geometry lives in ``lib/domain/no_fly_zone_geometry``. The
    # methods below are thin delegations that preserve the LifecycleNode
    # call-site shape and the precompute-state-on-load logging behaviour.
    def _compute_zone_np_state(self, zone: NoFlyZone) -> None:
        from drone_rescue_coordination.lib.domain.no_fly_zone_geometry import (
            precompute_zone_state,
        )
        state = precompute_zone_state(zone)
        if not state.get('valid'):
            # Replicate the legacy warning surface; precompute itself
            # is rclpy-free.
            if zone.zone_type == ZoneShape.CIRCLE:
                self.get_logger().warn(
                    f'Zone {zone.name!r}: circle type missing center or '
                    f'radius — will be ignored (distance=+inf)'
                )
            elif zone.zone_type == ZoneShape.POLYGON:
                self.get_logger().warn(
                    f'Zone {zone.name!r}: polygon with <3 vertices — '
                    f'will be ignored (distance=+inf)'
                )
            else:
                self.get_logger().warn(
                    f'Zone {zone.name!r}: unknown zone_type {zone.zone_type!r}'
                )
        self._zone_np[zone.name] = state

    def _distance_to_zone(self, position: Tuple[float, float],
                          zone: NoFlyZone) -> float:
        """Signed distance to the zone boundary (negative inside).

        Delegates to the pure-Python ``distance_to_zone`` policy.
        """
        from drone_rescue_coordination.lib.domain.no_fly_zone_geometry import (
            distance_to_zone,
        )
        return distance_to_zone(position, zone, self._zone_np.get(zone.name))

    def publish_markers(self):
        """Publish visualization markers for all zones."""
        marker_array = MarkerArray()

        for i, zone in enumerate(self.zones):
            # Create zone boundary marker
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'no_fly_zones'
            marker.id = i
            marker.action = Marker.ADD

            # Color based on priority
            marker.color = self._get_zone_color(zone.priority)

            if zone.zone_type == ZoneShape.CIRCLE:
                marker.type = Marker.CYLINDER
                marker.pose.position.x = zone.center[0]
                marker.pose.position.y = zone.center[1]
                marker.pose.position.z = (zone.min_altitude + zone.max_altitude) / 2
                marker.scale.x = (zone.radius + zone.buffer_distance) * 2
                marker.scale.y = (zone.radius + zone.buffer_distance) * 2
                marker.scale.z = zone.max_altitude - zone.min_altitude

            elif zone.zone_type == ZoneShape.POLYGON:
                marker.type = Marker.LINE_STRIP
                marker.scale.x = 0.3  # Line width

                # Add vertices
                for vertex in zone.vertices:
                    p = Point()
                    p.x = vertex[0]
                    p.y = vertex[1]
                    p.z = (zone.min_altitude + zone.max_altitude) / 2
                    marker.points.append(p)

                # Close the polygon
                if zone.vertices:
                    p = Point()
                    p.x = zone.vertices[0][0]
                    p.y = zone.vertices[0][1]
                    p.z = (zone.min_altitude + zone.max_altitude) / 2
                    marker.points.append(p)

            marker_array.markers.append(marker)

            # Add text label
            text_marker = Marker()
            text_marker.header.frame_id = 'world'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'zone_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            if zone.zone_type == ZoneShape.CIRCLE:
                text_marker.pose.position.x = zone.center[0]
                text_marker.pose.position.y = zone.center[1]
            else:
                # Use centroid of polygon
                cx = sum(v[0] for v in zone.vertices) / len(zone.vertices)
                cy = sum(v[1] for v in zone.vertices) / len(zone.vertices)
                text_marker.pose.position.x = cx
                text_marker.pose.position.y = cy

            text_marker.pose.position.z = zone.max_altitude + 2
            text_marker.scale.z = 1.5
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = zone.name

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def _get_zone_color(self, priority: ZonePriority) -> ColorRGBA:
        """Get marker color based on zone priority.

        Table-driven lookup; unknown priorities fall through to the
        documented `low` (green) default.
        """
        r, g, b = _PRIORITY_COLOR.get(priority, _PRIORITY_COLOR[ZonePriority.LOW])
        color = ColorRGBA()
        color.r, color.g, color.b = r, g, b
        color.a = 0.4
        return color


def main(args=None):
    rclpy.init(args=args)
    node = ZoneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
