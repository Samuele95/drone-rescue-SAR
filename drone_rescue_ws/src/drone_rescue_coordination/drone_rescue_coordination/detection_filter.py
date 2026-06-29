"""Detection filter: turns the noisy per-drone /detections_raw stream into
multi-view-confirmed VictimCandidate messages.

Pipeline (in order):
  1. Self-filter  : drop sightings within `min_distance_from_drones` of any
                    drone's current pose (the colored drone bodies fool the
                    HSV detector at takeoff).
  2. Confidence   : drop sightings below `confidence_floor`.
  3. DBSCAN       : run on a sliding `cluster_window_seconds` of accepted
                    sightings.
  4. Multi-view   : a cluster becomes a VictimCandidate only when ≥ 2 distinct
                    drones have contributed at least one sighting AND the
                    fused confidence ≥ `confirmation_threshold`.
  5. Bayesian fuse: P = 1 - prod(1 - p_i); position = confidence-weighted
                    centroid (computed by detection_cluster.Cluster).

For backward compatibility with coverage_tracker / victim_visualizer (which
listen to PoseStamped on /victims/detected), we publish a passthrough on
/victims/detected the FIRST time a cluster becomes confirmed.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

from drone_rescue_msgs.msg import VictimDetection, VictimCandidate

from drone_rescue_coordination.lib.detection_cluster import (
    Sighting, Cluster, dbscan,
)
from drone_rescue_coordination.lib.domain.fleet import default_drone_names_list
from drone_rescue_coordination.lib.domain.scenario_schema import ParamScope
from drone_rescue_coordination.lib.ros_adapter.parameter_declarer import (
    declare_for_scope,
)
from drone_rescue_coordination.lib.ros_adapter.topic_factory import (
    QosName, TopicFactory,
)
from drone_rescue_coordination.lib.composition import (
    bind_composition, resolve_clock,
)


class DetectionFilter(LifecycleNode):
    """Deliberative-perception node: 3T Layer 3 (planning-layer input).

    Promoted from plain Node to LifecycleNode so ``lifecycle_manager`` can
    coordinate it through the standard ``change_state`` services, consistent
    with the shallow-promotion pattern (subs/pubs/timers stay in ``__init__``
    so launch behaviour is unchanged; lifecycle callbacks are stub-SUCCESS
    until the dedicated ``lifecycle_manager`` coordination pass).

    Slides anchor: 3T architecture (p. 33), deliberative perception belongs to
    the planning-layer startup phase; Planning Layer Integration (p. 44),
    "planning component is invoked as needed by the executive." Promoting this
    node aligns its lifecycle with the L3 / L2 / L1 boundaries already managed
    by the lifecycle_manager.
    """

    # Composition kwarg; falls back to lazy adapter construction when None.
    def __init__(self, *, composition=None):
        super().__init__('detection_filter')
        self._composition = composition

        # Schema-registered DETECTION params come from PARAM_SCHEMA via
        # declare_for_scope. Per-name overrides preserve the legacy runtime
        # defaults until PARAM_SCHEMA is reconciled. dbscan_min_samples +
        # publish_rate_hz now live in PARAM_SCHEMA so declare_for_scope picks
        # them up. The only remaining inline declaration is drone_names (a
        # fleet-level concern not in any scope).
        declare_for_scope(
            self, ParamScope.DETECTION,
            defaults_override={
                'min_distance_from_drones': 6.0,
                'confidence_floor': 0.4,
                'cluster_window_seconds': 60.0,
                'dbscan_eps_m': 10.0,
                'confirmation_threshold': 0.7,
            },
        )
        self.declare_parameter('drone_names', default_drone_names_list())
        # Central staging-area exclusion radius (m). The map centre is the
        # urban staging area the drones launch from and overfly: the orange
        # landing-pad ring (origin), the yellow road centerlines, two traffic
        # lights with red lenses (r~8 m), a fallen street sign (r~9 m) and a
        # burst red hydrant (r~13 m) all read as victim colours and pick up
        # multi-witness false confirmations. None of these are victims (the
        # real victims start at ~20 m radius), so dropping sightings inside
        # this radius removes the centre false-positives at no recall cost.
        # 15 m covers all the central clutter with a 5 m margin to the nearest
        # victim. 0 disables.
        self.declare_parameter('launch_pad_exclusion_m', 15.0)
        # NOTE: ArUco area weighting now lives in victim_detector itself: the
        # area-graded confidence is set there before publishing, so the filter
        # just consumes the float on /<drone>/detections_raw.

        self.drone_names: List[str] = list(self.get_parameter('drone_names').value)
        self.min_distance_from_drones = float(self.get_parameter('min_distance_from_drones').value)
        self.launch_pad_exclusion_m = float(self.get_parameter('launch_pad_exclusion_m').value)
        self.confidence_floor = float(self.get_parameter('confidence_floor').value)
        self.cluster_window_seconds = float(self.get_parameter('cluster_window_seconds').value)
        self.dbscan_eps_m = float(self.get_parameter('dbscan_eps_m').value)
        self.dbscan_min_samples = int(self.get_parameter('dbscan_min_samples').value)
        self.confirmation_threshold = float(self.get_parameter('confirmation_threshold').value)
        self.min_confirm_observations = int(self.get_parameter('min_confirm_observations').value)
        self.min_multi_witnesses = int(self.get_parameter('min_multi_witnesses').value)
        self.min_sightings_per_witness = int(self.get_parameter('min_sightings_per_witness').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.lidar_depth_tolerance_m = float(self.get_parameter('lidar_depth_tolerance_m').value)
        self.lidar_corroboration_boost = float(self.get_parameter('lidar_corroboration_boost').value)

        # Mission Control may tweak detection sensitivity mid-run via
        # `ros2 param set`. Whitelist the safe knobs (re-clustering happens
        # on the next tick, so the new values take effect within ~500 ms).
        self.add_on_set_parameters_callback(self._on_runtime_params)

        # State
        self.drone_positions: Dict[str, Point] = {}
        # Drone (x, y) cache as an (N, 2) ndarray, refreshed in _on_odom.
        # self-filter then runs one vectorised distance test per sighting
        # instead of a Python loop over self.drone_positions.values().
        self._drone_xy: Optional[np.ndarray] = None
        # Most recent LaserScan per drone (used for LiDAR corroboration). We
        # only keep one (corroboration is "did this scan have any return at
        # the expected ground range?") so freshness matters more than history.
        self.drone_scans: Dict[str, LaserScan] = {}
        # Corroboration boolean cache; populated per scan.
        self._lidar_corroborates_cache: Dict[str, bool] = {}
        self.sightings: List[Sighting] = []
        # Long-lived candidate registry: id, last position, last confirmed flag.
        self._next_candidate_id = 1
        # Maps stable candidate_id → last published (position, confirmed). Used to
        # decide whether a re-publish is needed and to assign stable ids across
        # cluster re-runs.
        self.candidates: Dict[int, Dict] = {}
        self._already_passed_through: set = set()  # candidate_ids that already
                                                    # got a /victims/detected blast

        # QoS: per-drone raw detections are bursty + best-effort tolerable.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=20,
        )

        # Publishers
        self.candidate_pub = self.create_publisher(
            VictimCandidate, '/victims/candidates', 10,
        )
        # Backward-compat for coverage_tracker / victim_visualizer (PoseStamped)
        # and camera_director (VictimDetection). Both fire only when a
        # candidate becomes CONFIRMED, so the legacy topics now signal
        # "high-confidence find" instead of "raw HSV fire".
        self.legacy_pose_pub = self.create_publisher(
            PoseStamped, '/victims/detected', 10,
        )
        self.legacy_detection_pub = self.create_publisher(
            VictimDetection, '/victims/detections', 10,
        )

        # Adopt TopicFactory. raw + odom share the project's RELIABLE+depth=20
        # profile (SENSOR_RELIABLE, added because detection_filter is the first
        # consumer that wants ordered, lossless odom). scan uses SENSOR_HOT
        # (BEST_EFFORT,depth=1) which matches the gz bridge default.
        self._time = resolve_clock(self, self._composition)
        self._topic_factory = TopicFactory(self, self.drone_names)
        # `_on_raw_detection` only takes one arg, so use make_sub per
        # drone instead of per_drone_subs which would pass a name kwarg.
        self._raw_subs = [
            self._topic_factory.make_sub(
                f'/{d}/detections_raw', VictimDetection,
                self._on_raw_detection, QosName.SENSOR_RELIABLE,
            )
            for d in self.drone_names
        ]
        self._odom_subs = self._topic_factory.per_drone_subs(
            'odom', Odometry, self._on_odom,
            qos_override=QosName.SENSOR_RELIABLE,
        )
        self._scan_subs = self._topic_factory.per_drone_subs(
            'scan', LaserScan, self._on_scan,
            qos_override=QosName.SENSOR_HOT,
        )

        # Timer
        self._cluster_timer = self.create_timer(1.0 / publish_rate_hz, self._on_cluster_tick)

        self.get_logger().info(
            f'detection_filter ready: {len(self.drone_names)} drones, '
            f'eps={self.dbscan_eps_m}m, min_samples={self.dbscan_min_samples}, '
            f'confirm@{self.confirmation_threshold}, '
            f'self-filter {self.min_distance_from_drones}m, '
            f'window {self.cluster_window_seconds}s'
        )

    def _on_odom(self, msg: Odometry, drone_name: str) -> None:
        self.drone_positions[drone_name] = msg.pose.pose.position
        # Refresh (N, 2) cache for vectorised self-filter. Updating per-odom
        # (~30 Hz) is cheap vs. running this on every raw detection (~80 Hz).
        if self.drone_positions:
            self._drone_xy = np.fromiter(
                (c for p in self.drone_positions.values() for c in (p.x, p.y)),
                dtype=np.float64,
                count=2 * len(self.drone_positions),
            ).reshape(-1, 2)

    def _on_scan(self, msg: LaserScan, drone_name: str) -> None:
        self.drone_scans[drone_name] = msg
        # Cache the corroboration boolean per scan instead of re-scanning the
        # full ranges array per sighting. On busy detection streams
        # (~80 raw/s × 6000 entries) this cuts ~480k Python comparisons/s to
        # one numpy mask per scan. Uses the drone's current odom altitude;
        # reads the same (altitude, scan) pair the legacy `_lidar_corroborates`
        # did.
        pos = self.drone_positions.get(drone_name)
        if pos is None or pos.z < 1.0:
            self._lidar_corroborates_cache[drone_name] = False
            return
        import numpy as np
        arr = np.asarray(msg.ranges, dtype=np.float32)
        lo = pos.z - self.lidar_depth_tolerance_m
        hi = pos.z + self.lidar_depth_tolerance_m
        self._lidar_corroborates_cache[drone_name] = bool(
            ((arr >= lo) & (arr <= hi)).any()
        )

    def _lidar_corroborates(self, drone_name: str) -> bool:
        """True if the reporting drone's most recent LiDAR scan has at
        least one return at roughly the drone's current altitude (i.e.
        there is SOMETHING on the ground below us). Cache populated by
        `_on_scan`; O(1) lookup per sighting.
        """
        return self._lidar_corroborates_cache.get(drone_name, False)

    def _on_raw_detection(self, msg: VictimDetection) -> None:
        # ArUco confidence is now graded by pixel area in victim_detector: a
        # 4 px marker no longer carries the same weight as a 30 px one. The
        # filter just consumes the value as-is.
        confidence = float(msg.confidence)

        # 1. Confidence floor
        if confidence < self.confidence_floor:
            return

        # 2. Self-filter: reject if within min_distance_from_drones of any
        # drone. Vectorised distance check against the cached drone-xy ndarray
        # (refreshed in _on_odom).
        if self._drone_xy is not None:
            d2 = (self._drone_xy[:, 0] - msg.position.x) ** 2 \
                + (self._drone_xy[:, 1] - msg.position.y) ** 2
            if bool((d2 < self.min_distance_from_drones ** 2).any()):
                return

        # 2b. Launch-pad exclusion: drop sightings near the world origin, where
        # the orange landing_pad ring (and the road centerline through it) read
        # as victim clothing and every drone overflies them. Real victims are
        # well outside this radius, so this costs no recall.
        if self.launch_pad_exclusion_m > 0.0 and (
            msg.position.x ** 2 + msg.position.y ** 2
            < self.launch_pad_exclusion_m ** 2
        ):
            return

        # 3. LiDAR corroboration: boost confidence if the reporting drone's
        # latest scan has a ground return at expected altitude.
        if self._lidar_corroborates(msg.drone_name or ''):
            confidence = min(0.99, confidence + self.lidar_corroboration_boost)

        t = self._time.now_sec()
        self.sightings.append(Sighting(
            x=msg.position.x,
            y=msg.position.y,
            confidence=confidence,
            drone_name=msg.drone_name or 'unknown',
            t_seen=t,
            detection_type=int(msg.detection_type),
        ))

    def _on_cluster_tick(self) -> None:
        now = self._time.now_sec()
        # Trim old sightings
        self.sightings = [s for s in self.sightings
                          if now - s.t_seen <= self.cluster_window_seconds]

        if not self.sightings:
            return

        clusters = dbscan(
            self.sightings,
            eps=self.dbscan_eps_m,
            min_samples=self.dbscan_min_samples,
        )

        for cluster in clusters:
            distinct = cluster.distinct_drones
            cx, cy = cluster.position
            fused = cluster.fused_confidence
            # Emit a candidate as soon as DBSCAN finds a dense cluster (≥
            # min_samples sightings). This lets mission_manager INVESTIGATE
            # the spot with a 2nd drone, which becomes the multi-view
            # confirmation. Mark `confirmed` only once we actually have ≥ 2
            # distinct drones AND high fused confidence: that's the gate
            # that fires the back-compat /victims/detected blast.
            # Fidelity note: this is the FAITHFUL per-witness confirmation
            # gate (Horn confirmed_cluster_full/1 in docs/domain/model.html):
            # ≥ min_multi_witnesses distinct drones each contributing ≥
            # min_sightings_per_witness sightings, NOT the looser SWRL S1
            # ConfirmedCluster, which over-approximates (it omits the per-witness
            # count). S1 is kept as an intentional, documented superset; every
            # code consumer of "confirmed cluster" must use this count, never S1.
            multi_witnesses = cluster.witnesses_with_at_least(
                self.min_sightings_per_witness
            )
            confirmed = (len(distinct) >= 2
                         and fused >= self.confirmation_threshold
                         and cluster.observation_count >= self.min_confirm_observations
                         and multi_witnesses >= self.min_multi_witnesses)

            cid = self._stable_candidate_id(cx, cy)
            existing = self.candidates.get(cid)
            self.candidates[cid] = {
                'x': cx, 'y': cy, 'confidence': fused, 'confirmed': confirmed,
                'distinct_drones': distinct, 'observation_count': cluster.observation_count,
            }

            self._publish_candidate(cid, cx, cy, fused, distinct,
                                    cluster.observation_count, confirmed)

            # Backward compat passthrough: only the first time a candidate
            # becomes confirmed.
            if confirmed and cid not in self._already_passed_through:
                self._already_passed_through.add(cid)
                self._publish_legacy(cx, cy)

    def _stable_candidate_id(self, x: float, y: float) -> int:
        """Assign or look up a stable id for a cluster centred at (x, y).

        Reuses any existing id whose last position is within
        ``dbscan_eps_m`` of (x, y). The eps tie is intentional: a
        DBSCAN cluster's centroid can drift across ticks by up to one
        eps as new sightings arrive; using a smaller match radius here
        would mint a fresh candidate_id for what is physically the
        same victim, inflating false-positive counts and triggering
        redundant INVESTIGATE dispatches. Using ``dbscan_eps_m``
        guarantees that any two centroids DBSCAN itself would have
        merged into one cluster also collapse to one candidate_id
        here.
        """
        match_r2 = self.dbscan_eps_m * self.dbscan_eps_m
        for cid, info in self.candidates.items():
            if (info['x'] - x) ** 2 + (info['y'] - y) ** 2 < match_r2:
                return cid
        cid = self._next_candidate_id
        self._next_candidate_id += 1
        return cid

    def _publish_candidate(self, cid: int, x: float, y: float, conf: float,
                           drones: List[str], obs_count: int, confirmed: bool) -> None:
        msg = VictimCandidate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.candidate_id = cid
        msg.position.x = x
        msg.position.y = y
        msg.position.z = 0.0
        msg.confidence = float(conf)
        msg.observation_count = int(obs_count)
        msg.reporting_drones = list(drones)
        msg.confirmed = bool(confirmed)
        self.candidate_pub.publish(msg)

    def _publish_legacy(self, x: float, y: float) -> None:
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.orientation.w = 1.0
        self.legacy_pose_pub.publish(m)
        # camera_director listens to /victims/detections (VictimDetection).
        d = VictimDetection()
        d.header.stamp = m.header.stamp
        d.header.frame_id = 'world'
        d.position.x = x
        d.position.y = y
        d.confirmed = True
        d.detection_type = VictimDetection.DETECTION_VISUAL
        d.confidence = 0.99
        self.legacy_detection_pub.publish(d)
        self.get_logger().info(f'CONFIRMED candidate at ({x:.1f}, {y:.1f})')

    # Derived from `lib/domain/scenario_schema`.
    from drone_rescue_coordination.lib.domain.scenario_schema import (
        ParamScope as _ParamScope,
        runtime_tweakable_for_scope as _runtime_tweakable_for_scope,
    )
    _RUNTIME_PARAMS = _runtime_tweakable_for_scope(_ParamScope.DETECTION)
    del _ParamScope, _runtime_tweakable_for_scope

    def _on_runtime_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name not in self._RUNTIME_PARAMS:
                # Allow unknown params silently (might be use_sim_time or
                # other ROS-internal things). Only reject known launch-time
                # ones.
                continue
            value = p.value
            if p.name == 'confidence_floor':
                self.confidence_floor = float(value)
            elif p.name == 'dbscan_eps_m':
                self.dbscan_eps_m = float(value)
            elif p.name == 'confirmation_threshold':
                self.confirmation_threshold = float(value)
            elif p.name == 'cluster_window_seconds':
                self.cluster_window_seconds = float(value)
            elif p.name == 'min_distance_from_drones':
                self.min_distance_from_drones = float(value)
            elif p.name == 'lidar_corroboration_boost':
                self.lidar_corroboration_boost = float(value)
            elif p.name == 'lidar_depth_tolerance_m':
                self.lidar_depth_tolerance_m = float(value)
            elif p.name == 'min_confirm_observations':
                self.min_confirm_observations = int(value)
            elif p.name == 'min_multi_witnesses':
                self.min_multi_witnesses = int(value)
            elif p.name == 'min_sightings_per_witness':
                self.min_sightings_per_witness = int(value)
            elif p.name == 'dbscan_min_samples':
                # Close the silent-accept bug: previously _RUNTIME_PARAMS
                # accepted this name (via PARAM_SCHEMA) but no `elif` branch
                # wrote it back to self.dbscan_min_samples, so the tweak had
                # no effect at the next _on_cluster_tick.
                self.dbscan_min_samples = int(value)
            self.get_logger().info(
                f'detection_filter runtime param updated: {p.name} = {value}'
            )
        return SetParametersResult(successful=True)

    # Shallow LifecycleNode promotion. Mirrors the precedent (zone_manager,
    # environment_monitor, sensor_degradation, battery_monitor): the
    # subscriptions / publishers / timer remain created in __init__ so launch
    # behaviour is unchanged; the lifecycle callbacks stub SUCCESS so the
    # change_state service interface is available for the lifecycle_manager to
    # coordinate. The deep-promotion version (move construction into
    # on_configure, full deactivate semantics) is the natural sequel once the
    # lifecycle_manager coordination pass adds detection_filter to
    # _build_node_list.
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


def main(args=None):
    rclpy.init(args=args)
    node = bind_composition(DetectionFilter())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
