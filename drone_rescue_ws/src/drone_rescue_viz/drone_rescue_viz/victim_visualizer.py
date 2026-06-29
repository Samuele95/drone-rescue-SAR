#!/usr/bin/env python3
"""
Victim visualizer node.

Creates RViz markers for detected victims:
- Green spheres for confirmed candidates
- Orange spheres for unconfirmed candidates
- Text labels with victim ID + confidence
- First-seen flash effect (3 s expanding ring) when a new candidate
  first emerges

Switched from ``/victims/detections`` (VictimDetection) to
``/victims/candidates`` (VictimCandidate); the node now consumes the
same clustered-detection signal as the dashboard and adopts OperatorView
via ``render_from(view)``.

What the topic switch trades:
- Lost: ArUco-blue colour, priority pulse, Visual/ArUco/Thermal
  label string. (VictimCandidate carries no aruco_id /
  detection_type / priority; those are pre-clustering
  per-detection metadata.)
- Gained: shared MissionViewModel projection with the dashboard
  and mission_recorder, lower DDS rate (clusters emit at a slower
  cadence than raw detections), one canonical operator-facing
  victim view.
"""

from typing import Dict

from drone_rescue_msgs.msg import VictimCandidate

# saga_confirmed QoS comes from the shared ui_common factory; the
# literal was duplicated verbatim with dashboard_app.py.
from drone_rescue_ui_common.qos import transient_local_reliable_qos
from drone_rescue_ui_common.view_model import MissionViewModel

import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA, UInt32

from visualization_msgs.msg import Marker, MarkerArray


class VictimVisualizer(Node):
    """
    Visualizes confirmed-cluster victim candidates in RViz.

    Subscribes:
        /victims/candidates: VictimCandidate messages (multi-view
            clustered detections from detection_filter)
        /victims/saga_confirmed: UInt32 carrying the cluster_id of
            victims whose mission_manager CONFIRM task succeeded
            (TRANSIENT_LOCAL, so a late visualizer restart still
            collects the per-mission history)

    Publishes:
        /visualization/victims: MarkerArray for RViz

    Implements the ``OperatorView`` Protocol via ``render_from``:
    the publish timer dispatches to it on the current local
    MissionViewModel snapshot.
    """

    def __init__(self):
        super().__init__('victim_visualizer')

        self.declare_parameter('marker_lifetime', 0.0)  # 0 = forever
        self.declare_parameter('sphere_radius', 0.8)
        self.declare_parameter('text_height', 1.5)
        self.declare_parameter('text_scale', 0.5)
        self.declare_parameter('confirmed_color', [0.0, 1.0, 0.0, 0.9])
        self.declare_parameter('unconfirmed_color', [1.0, 0.3, 0.0, 0.8])

        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        self.sphere_radius = self.get_parameter('sphere_radius').value
        self.text_height = self.get_parameter('text_height').value
        self.text_scale = self.get_parameter('text_scale').value
        self.confirmed_color = self.get_parameter('confirmed_color').value
        self.unconfirmed_color = self.get_parameter('unconfirmed_color').value

        # local MissionViewModel; folded by the /victims/candidates
        # callback, drained by render timer.
        self._mvm = MissionViewModel()
        # Per-candidate first-seen wall-clock; drives the 3 s flash
        # ring effect for newly-emerged clusters. Keyed by candidate_id
        # (the cluster identifier from detection_filter, distinct from
        # the prior /victims/detections.victim_id).
        self._first_seen: Dict[int, float] = {}

        self.marker_pub = self.create_publisher(
            MarkerArray, '/visualization/victims', 10,
        )

        self.create_subscription(
            VictimCandidate, '/victims/candidates',
            self.candidate_callback, 10,
        )
        # TRANSIENT_LOCAL with depth >= per-mission victim count so a
        # restart of this visualizer recovers the saga-confirmed set
        # rather than starting empty. Publisher (mission_manager) uses
        # RELIABLE/TRANSIENT_LOCAL/depth=64, match it.
        self.create_subscription(
            UInt32, '/victims/saga_confirmed',
            self._on_saga_confirmed,
            transient_local_reliable_qos(depth=64),
        )

        self.timer = self.create_timer(0.5, self.publish_markers)
        self.get_logger().info('Victim visualizer started')

    def candidate_callback(self, msg: VictimCandidate):
        cid = int(msg.candidate_id)
        if cid not in self._first_seen:
            self._first_seen[cid] = self.get_clock().now().nanoseconds / 1e9
        self._mvm = self._mvm.apply_victim_candidate(msg)

    def _on_saga_confirmed(self, msg: UInt32) -> None:
        """
        Fold a saga-confirmed cluster_id into the view model.

        The parallel ``_saga_confirmed`` set is gone;
        ``MissionViewModel.apply_saga_confirmed`` (the same reducer the
        dashboard uses) ORs the flag into ``VictimViewState.confirmed``
        idempotently, so the two confirmation channels merge in exactly
        one place.
        """
        self._mvm = self._mvm.apply_saga_confirmed(int(msg.data))

    def publish_markers(self):
        self.render_from(self._mvm)

    def render_from(self, view: MissionViewModel) -> None:
        """
        Paint the MarkerArray from the frozen view snapshot.

        OperatorView Protocol implementation.
        """
        if not view.victims:
            return

        marker_array = MarkerArray()
        marker_id = 0
        # one clock read per render tick; the per-marker
        # get_clock().now() calls (4-7 per victim) are gone. All markers
        # in the batch share a consistent stamp.
        now_ros = self.get_clock().now()
        now_msg = now_ros.to_msg()
        now = now_ros.nanoseconds / 1e9
        sphere_diam = self.sphere_radius * 2

        for cid, vv in view.victims.items():
            # ``vv.confirmed`` already merges both confirmation channels:
            # multi-view fusion (>=2 reporters & high confidence) OR saga
            # completion, because both folds run through MissionViewModel.
            is_confirmed = vv.confirmed
            # Sphere marker: green if confirmed, orange if not.
            sphere = Marker()
            sphere.header.frame_id = 'world'
            sphere.header.stamp = now_msg
            sphere.ns = 'victim_spheres'
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD

            x, y = vv.position[0], vv.position[1]
            sphere.pose.position.x = float(x)
            sphere.pose.position.y = float(y)
            sphere.pose.position.z = self.sphere_radius  # Lift above ground
            sphere.pose.orientation.w = 1.0

            sphere.scale.x = sphere_diam
            sphere.scale.y = sphere_diam
            sphere.scale.z = sphere_diam

            if is_confirmed:
                sphere.color = self._color_from_list(self.confirmed_color)
            else:
                sphere.color = self._color_from_list(self.unconfirmed_color)

            if self.marker_lifetime > 0:
                sphere.lifetime.sec = int(self.marker_lifetime)

            marker_array.markers.append(sphere)
            marker_id += 1

            # Text label: ID + confidence (or CONFIRMED status).
            text = Marker()
            text.header.frame_id = 'world'
            text.header.stamp = now_msg
            text.ns = 'victim_labels'
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = float(x)
            text.pose.position.y = float(y)
            text.pose.position.z = self.sphere_radius * 2 + self.text_height
            text.pose.orientation.w = 1.0
            text.scale.z = self.text_scale

            status = (
                'CONFIRMED'
                if is_confirmed
                else f'{vv.confidence * 100:.0f}%'
            )
            text.text = f'V{cid}\n{status}'

            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.9
            if self.marker_lifetime > 0:
                text.lifetime.sec = int(self.marker_lifetime)
            marker_array.markers.append(text)
            marker_id += 1

            # Confirmed-victim ring (translucent green disc on ground).
            if is_confirmed:
                ring = Marker()
                ring.header.frame_id = 'world'
                ring.header.stamp = now_msg
                ring.ns = 'victim_rings'
                ring.id = marker_id
                ring.type = Marker.CYLINDER
                ring.action = Marker.ADD

                ring.pose.position.x = float(x)
                ring.pose.position.y = float(y)
                ring.pose.position.z = 0.05
                ring.pose.orientation.w = 1.0

                ring.scale.x = self.sphere_radius * 4
                ring.scale.y = self.sphere_radius * 4
                ring.scale.z = 0.1

                ring.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)
                if self.marker_lifetime > 0:
                    ring.lifetime.sec = int(self.marker_lifetime)
                marker_array.markers.append(ring)
                marker_id += 1

            # First-seen flash: expanding fading ring for the first
            # 3 s after a candidate first appears.
            first_seen = self._first_seen.get(cid)
            if first_seen is not None:
                age = now - first_seen
                if age < 3.0:
                    flash = Marker()
                    flash.header.frame_id = 'world'
                    flash.header.stamp = now_msg
                    flash.ns = 'victim_flash'
                    flash.id = marker_id
                    flash.type = Marker.CYLINDER
                    flash.action = Marker.ADD

                    flash.pose.position.x = float(x)
                    flash.pose.position.y = float(y)
                    flash.pose.position.z = 0.1
                    flash.pose.orientation.w = 1.0

                    progress = age / 3.0
                    scale_factor = 1.0 + (3.0 * progress)
                    flash.scale.x = sphere_diam * scale_factor
                    flash.scale.y = sphere_diam * scale_factor
                    flash.scale.z = 0.15

                    flash.color.r = 1.0
                    flash.color.g = 1.0
                    flash.color.b = 0.0
                    flash.color.a = 0.8 * (1.0 - progress)

                    marker_array.markers.append(flash)
                    marker_id += 1

        self.marker_pub.publish(marker_array)

    # ----------------------------------------------------------- helpers
    def _color_from_list(self, color_list) -> ColorRGBA:
        color = ColorRGBA()
        color.r = float(color_list[0])
        color.g = float(color_list[1])
        color.b = float(color_list[2])
        color.a = float(color_list[3])
        return color


def main(args=None):
    rclpy.init(args=args)
    node = VictimVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
