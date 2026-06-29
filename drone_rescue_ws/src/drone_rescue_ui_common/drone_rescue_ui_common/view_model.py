"""MissionViewModel: the projection both UI surfaces fold over.

The live dashboard's ``StateCache`` and Mission Control's analytics
pipeline both reduce the same upstream streams (``/mission/events``,
``/<drone>/peer_state``, ``/coverage/metrics``, ``/victims/candidates``)
into per-drone + per-victim + coverage projections. They do this with
hand-written callbacks today; this module gives them one reducer.

Pure FP-style fold: ``MissionViewModel.apply(event) -> MissionViewModel``
returns a NEW view model rather than mutating in place, testable,
trivially time-travellable, and the natural shape for an event log
replay path.

This is a scaffolding pass; the existing dashboard/StateCache and
mission_control/widgets continue to use their hand-rolled reducers.
The new MissionViewModel exists so future operator-facing fields land
in one place; legacy reducers can migrate as they're touched for
other reasons.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any, Mapping, Optional, Tuple


@dataclass(frozen=True)
class DroneViewState:
    """One drone's projection: what the dashboard's per-drone widget
    cares about. Sourced from the latest ``/<drone>/peer_state`` and
    ``/<drone>/health`` messages.
    """
    name: str
    battery: float = 1.0       # 0.0..1.0
    task_type: int = 5         # IDLE; mirrors TaskAssignment.IDLE
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_z: float = 0.0
    # yaw radians extracted from the peer state's orientation
    # quaternion. scene_view's cursor heading arrow reads this; the
    # legacy peer-dict access shadowed it.
    yaw: float = 0.0
    is_down: bool = False
    busy_with_victim: int = 0  # 0 means not assigned
    wp_index: int = 0
    wp_total: int = 0
    anomaly_score: float = 0.0
    # health-reason text + unrecoverable flag promoted onto the typed
    # projection so dashboard widgets consume `state.view.drones[...]`
    # exclusively (no parallel `state.health[name].reason` access).
    health_reason: str = ''
    unrecoverable: bool = False
    # Paired with the (battery, task_type, ...) fields above so the
    # reader gets a tearing-free snapshot via the frozen `replace()`
    # atomic store. 0.0 means "no peer_state / health received yet";
    # readers should treat it as stale-of-infinite-age.
    peer_last_seen: float = 0.0
    health_last_seen: float = 0.0
    # DroneStatus.state mirror (controller operating-state machine:
    # IDLE/TAKEOFF/SURVEY/RETURN/LANDING/...), distinct from
    # ``task_type`` (mission task assignment). Folded by
    # ``apply_drone_status`` from ``/<drone>/status``; 0 = IDLE / not
    # reported. Diagnostic-useful in StateTableWidget too.
    controller_state: int = 0


@dataclass(frozen=True)
class VictimViewState:
    """One victim's projection: for the dashboard's victim table and
    Mission Control's analytics. Sourced from
    ``/victims/candidates``."""
    candidate_id: int
    position: Tuple[float, float] = (0.0, 0.0)
    confidence: float = 0.0
    confirmed: bool = False
    reporting_drones: Tuple[str, ...] = ()


@dataclass(frozen=True)
class CoverageViewState:
    """Coverage projection from ``/coverage/metrics``."""
    percentage: float = 0.0
    cells_visited: int = 0
    # total_cells carried so dashboard widgets consume
    # state.view.coverage exclusively (was reading
    # state.coverage.total_cells off the raw ROS message).
    total_cells: int = 0
    elapsed_time_seconds: float = 0.0
    # scan-time ETA. coverage_tracker computes and publishes this on
    # /coverage/metrics.estimated_time_remaining, but the view model previously
    # dropped it, so no widget could render it. 0.0 = not yet estimable.
    estimated_time_remaining: float = 0.0
    # viz-overlay-driven additions; the dashboard's CoverageBanner can
    # surface these too (number of drones currently actively surveying,
    # victims found per the /coverage/metrics counter, distinct from the
    # candidate-stream count which CoverageBanner currently shows).
    drones_surveying: int = 0
    victims_found: int = 0


@dataclass(frozen=True)
class MissionStateView:
    """Projection of ``/mission/state`` (MissionState, 1 Hz).

    Drives the dashboard's mission phase strip and sector progress.
    ``status`` mirrors the MissionState enum
    (INIT=0, ARMING=1, DEPLOYING=2, SCANNING=3, INVESTIGATING=4,
    COMPLETE=5, ABORTED=6); ``received`` distinguishes a genuine INIT
    from "no publisher yet".
    """
    status: int = 0
    received: bool = False
    sectors_total: int = 0
    sectors_completed: int = 0
    victims_found: int = 0
    victims_confirmed: int = 0
    active_tasks_summary: Tuple[str, ...] = ()


@dataclass(frozen=True)
class MissionViewModel:
    """Operator-facing projection of mission state.

    Construct empty; fold events with ``apply()``. Both the live
    dashboard and the mission_recorder finalise step can use the
    same reducer: live operates on streaming messages; recorder
    operates on the JSONL replay.
    """
    drones: Mapping[str, DroneViewState] = field(default_factory=dict)
    victims: Mapping[int, VictimViewState] = field(default_factory=dict)
    coverage: CoverageViewState = field(default_factory=CoverageViewState)
    mission: MissionStateView = field(default_factory=MissionStateView)
    log: Tuple[Any, ...] = ()    # bounded by caller; event records appended in order

    @property
    def confirmed_victim_count(self) -> int:
        """Number of victims confirmed via either channel (multi-view
        fusion OR saga completion; both fold into
        ``VictimViewState.confirmed``).

        Single definition; CoverageBanner and LiveTrendWidget
        previously each recomputed this sum inline. Distinct from
        ``coverage.victims_found`` (coverage_tracker's legacy
        passthrough counter, which misses auto-confirmed candidates).
        """
        return sum(1 for vv in self.victims.values() if vv.confirmed)

    def apply_peer_state(
        self, msg: Any, now: float = 0.0,
    ) -> 'MissionViewModel':
        """Update the per-drone projection from a DronePeerState.

        `now` is captured atomically into `peer_last_seen` alongside
        the rest of the projection. The reader gets a tearing-free
        snapshot when it inspects the frozen DroneViewState. Legacy
        callers (no `now`) still work; they just don't update the
        timestamp.
        """
        drone_name = getattr(msg, 'drone_name', None)
        if not drone_name:
            return self
        cur = self.drones.get(drone_name, DroneViewState(name=drone_name))
        # Extract yaw from the orientation quaternion. Standard z-axis
        # yaw formula: yaw = atan2(2(w*z + x*y), 1 - 2(y² + z²)).
        # Defensive: tests construct synthetic peer-state shapes
        # without an orientation; fall back to 0.0 then.
        q = getattr(msg.pose, 'orientation', None)
        if q is None:
            yaw = cur.yaw
        else:
            qw = float(getattr(q, 'w', 1.0))
            qx = float(getattr(q, 'x', 0.0))
            qy = float(getattr(q, 'y', 0.0))
            qz = float(getattr(q, 'z', 0.0))
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            )
        new = replace(
            cur,
            battery=float(getattr(msg, 'battery', cur.battery)),
            task_type=int(getattr(msg, 'task_type', cur.task_type)),
            pose_x=float(msg.pose.position.x),
            pose_y=float(msg.pose.position.y),
            pose_z=float(msg.pose.position.z),
            yaw=yaw,
            is_down=bool(getattr(msg, 'is_down', cur.is_down)),
            busy_with_victim=int(getattr(msg, 'busy_with_victim', 0)),
            wp_index=int(getattr(msg, 'wp_index', cur.wp_index)),
            wp_total=int(getattr(msg, 'wp_total', cur.wp_total)),
            peer_last_seen=float(now) if now > 0 else cur.peer_last_seen,
        )
        new_drones = dict(self.drones)
        new_drones[drone_name] = new
        return replace(self, drones=new_drones)

    def apply_health(
        self, drone_name: str, anomaly_score: float, now: float = 0.0,
        *, reason: str = '', unrecoverable: bool = False,
    ) -> 'MissionViewModel':
        """Update a drone's anomaly score from DroneHealth.

        `now` is captured atomically into `health_last_seen` for a
        tearing-free read. ``reason`` and ``unrecoverable`` keyword
        args carry the human-readable health text + hard-failure
        flag the dashboard's status column needs. Default-empty so
        legacy call sites (tests, in-flight migration) continue
        without behaviour change.
        """
        cur = self.drones.get(drone_name, DroneViewState(name=drone_name))
        new_drones = dict(self.drones)
        new_drones[drone_name] = replace(
            cur,
            anomaly_score=float(anomaly_score),
            health_reason=str(reason),
            unrecoverable=bool(unrecoverable),
            health_last_seen=float(now) if now > 0 else cur.health_last_seen,
        )
        return replace(self, drones=new_drones)

    def peer_age(self, drone_name: str, now: float) -> float:
        """Atomic age lookup. Returns infinity for unknown drones or
        those that haven't reported yet. Reader sees a paired
        (peer_last_seen, peer state) snapshot because the
        DroneViewState is frozen and replaced atomically."""
        d = self.drones.get(drone_name)
        if d is None or d.peer_last_seen <= 0:
            return float('inf')
        return now - d.peer_last_seen

    def health_age(self, drone_name: str, now: float) -> float:
        """Paired age lookup for the health stream."""
        d = self.drones.get(drone_name)
        if d is None or d.health_last_seen <= 0:
            return float('inf')
        return now - d.health_last_seen

    def apply_victim_candidate(self, msg: Any) -> 'MissionViewModel':
        """Update the victim projection from a VictimCandidate.

        The incoming ``msg.confirmed`` is the detection_filter
        multi-view fusion flag (≥2 distinct reporters AND combined
        confidence ≥ confirmation_threshold).
        It does NOT reflect saga-CONFIRM completion; that flows in
        through ``apply_saga_confirmed`` below. To avoid a saga-
        confirmed victim flipping back to unconfirmed when a fresh
        low-confidence candidate message arrives for the same cid
        (sector-scan re-detection), we preserve the saga flag with
        an OR rather than overwriting it.
        """
        cid = int(msg.candidate_id)
        new_victims = dict(self.victims)
        prior = self.victims.get(cid)
        confirmed = bool(msg.confirmed) or (
            prior is not None and prior.confirmed
        )
        new_victims[cid] = VictimViewState(
            candidate_id=cid,
            position=(float(msg.position.x), float(msg.position.y)),
            confidence=float(msg.confidence),
            confirmed=confirmed,
            reporting_drones=tuple(msg.reporting_drones),
        )
        return replace(self, victims=new_victims)

    def apply_saga_confirmed(self, cluster_id: int) -> 'MissionViewModel':
        """Mark a cluster_id as saga-confirmed (mission_manager
        dispatched + completed a CONFIRM task for it).

        The visualizer/dashboard subscribes to ``/victims/saga_confirmed``
        and folds each received UInt32 here. If the candidate hasn't
        been seen yet (this would happen on a TRANSIENT_LOCAL replay
        for a future subscriber), we create a placeholder VictimViewState
        with confirmed=True so the green marker still draws when the
        position arrives moments later via ``/victims/candidates``.

        Idempotent: already-confirmed cids are a no-op.
        """
        cid = int(cluster_id)
        existing = self.victims.get(cid)
        if existing is not None and existing.confirmed:
            return self
        new_victims = dict(self.victims)
        if existing is None:
            new_victims[cid] = VictimViewState(
                candidate_id=cid,
                confirmed=True,
            )
        else:
            new_victims[cid] = replace(existing, confirmed=True)
        return replace(self, victims=new_victims)

    def apply_coverage(self, msg: Any) -> 'MissionViewModel':
        """Update the coverage projection from CoverageMetrics.

        Folds ``drones_surveying`` and ``victims_found`` so the viz
        overlay nodes can render via ``render_from(view)`` instead of
        holding their own copy of the raw ROS message."""
        return replace(
            self,
            coverage=CoverageViewState(
                percentage=float(getattr(msg, 'percentage_covered', 0.0)),
                cells_visited=int(getattr(msg, 'cells_visited', 0)),
                total_cells=int(getattr(msg, 'total_cells', 0)),
                elapsed_time_seconds=float(getattr(msg, 'elapsed_time_seconds', 0.0)),
                estimated_time_remaining=float(
                    getattr(msg, 'estimated_time_remaining', 0.0)),
                drones_surveying=int(getattr(msg, 'drones_surveying', 0)),
                victims_found=int(getattr(msg, 'victims_found', 0)),
            ),
        )

    def apply_drone_status(
        self, drone_name: str, msg: Any, now: float = 0.0,
    ) -> 'MissionViewModel':
        """Update a drone's controller-state projection from
        ``DroneStatus`` (the controller's operating-state stream,
        distinct from peer_state's mission-task stream).

        The viz overlay nodes
        (``coverage_visualizer``, ``telemetry_overlay``) subscribe
        ``/<drone>/status`` and need a typed fold to call
        ``render_from(view)`` without holding the raw message. Folds
        ``state`` (controller_state int) + ``battery_level``. The
        ``peer_last_seen`` timestamp is shared because the operator-
        facing "this drone is reporting" gate is one signal regardless
        of which stream carries it; freshness gates that need a
        narrower DroneStatus-only timestamp can split later.
        """
        cur = self.drones.get(drone_name, DroneViewState(name=drone_name))
        new = replace(
            cur,
            controller_state=int(getattr(msg, 'state', cur.controller_state)),
            battery=float(getattr(msg, 'battery_level', cur.battery)),
            peer_last_seen=float(now) if now > 0 else cur.peer_last_seen,
        )
        new_drones = dict(self.drones)
        new_drones[drone_name] = new
        return replace(self, drones=new_drones)

    def apply_mission_state(self, msg: Any) -> 'MissionViewModel':
        """Update the mission projection from a MissionState message.

        Folded from ``/mission/state`` (RELIABLE/TRANSIENT_LOCAL,
        1 Hz); the dashboard's phase strip and sector-progress widgets
        read ``view.mission``.
        """
        return replace(
            self,
            mission=MissionStateView(
                status=int(getattr(msg, 'status', 0)),
                received=True,
                sectors_total=int(getattr(msg, 'sectors_total', 0)),
                sectors_completed=int(getattr(msg, 'sectors_completed', 0)),
                victims_found=int(getattr(msg, 'victims_found', 0)),
                victims_confirmed=int(getattr(msg, 'victims_confirmed', 0)),
                active_tasks_summary=tuple(
                    getattr(msg, 'active_tasks_summary', ()) or ()
                ),
            ),
        )

    def append_event(self, event: Any, max_log: int = 200) -> 'MissionViewModel':
        """Append an event to the bounded log."""
        new_log = (self.log + (event,))[-max_log:]
        return replace(self, log=new_log)


# Staleness thresholds for the operator-facing drone status fold,
# shared by StateTableWidget and the fleet rail.
PEER_STALE_S = 4.0
HEALTH_STALE_S = 3.0
ANOMALY_WARN_THRESHOLD = 0.25


def drone_status(view: MissionViewModel, name: str, now: float,
                 *, peer_stale_s: float = PEER_STALE_S,
                 health_stale_s: float = HEALTH_STALE_S,
                 ) -> Tuple[str, str]:
    """Derive the operator-facing (label, severity) for one drone.

    Single source for the status vocabulary the state table and the
    fleet rail both render; previously this decision tree lived inline
    in ``StateTableWidget._refresh``. ``severity`` is a palette token
    name: 'ok' | 'warn' | 'error' | 'muted'.
    """
    d = view.drones.get(name)
    if d is None or (d.peer_last_seen <= 0 and d.health_last_seen <= 0):
        return '—', 'muted'
    peer_alive = (d.peer_last_seen > 0
                  and view.peer_age(name, now) < peer_stale_s)
    health_alive = (d.health_last_seen > 0
                    and view.health_age(name, now) < health_stale_s)
    if d.unrecoverable:
        return 'DOWN', 'error'
    if not peer_alive and health_alive:
        return 'EXEC DOWN', 'error'
    if not health_alive and not peer_alive:
        return 'OFFLINE', 'error'
    if not health_alive:
        return 'SENSOR LOSS', 'warn'
    if d.anomaly_score > ANOMALY_WARN_THRESHOLD:
        return 'WARN', 'warn'
    return 'OK', 'ok'
