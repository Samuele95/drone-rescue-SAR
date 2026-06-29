"""Mission aggregate root.

The aggregate owns the fleet (`Dict[str, Drone]`), the victim registry
(`Dict[int, Victim]`), the per-victim saga aggregates
(`Dict[int, VictimSubMission]`), the `MissionStage` cursor, and the
`CoveragePlan`. Methods return `Sequence[OutgoingTask]` rather than
publishing directly, so the saga is testable without rclpy.

This skeleton is the seam. Full migration of mission_manager.py's saga
methods into `Mission` is multi-day work. This file exists so:

1. ``MissionPort.Protocol`` has a concrete reference implementation.
2. Subsequent work can collapse into ``Mission``'s shape instead of
   into the 1267-LOC LifecycleNode.
3. Unit tests can construct a ``Mission`` directly and exercise the
   saga with a synthetic clock.

``mission_manager.MissionManager`` continues to be the authoritative
orchestrator at runtime; ``Mission`` is a parallel data-only layer that
the legacy code mirrors into via the adapter helpers (e.g.
``Mission.from_legacy(drone_records, victim_records)``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import (
    TYPE_CHECKING, Callable, Dict, List, Optional, Sequence, Tuple,
)

from .entities import Drone, Victim
from .state_machines import (
    MissionStage, MissionStateMachine, TransitionEvent, VictimStage,
    VictimStateMachine,
)
from .task_type import TaskType
from .value_objects import (
    DEFAULT_INVESTIGATE_ANGLES, OutgoingTask, MissionStateSnapshot, Position,
)
from .victim_sub_mission import VictimSubMission

if TYPE_CHECKING:
    # WorldModel is forward-referenced because the saga lift introduces
    # the planner API surface before the actual cutover; this lets the
    # Mission class be the concrete ``DeliberativePlanner`` implementation
    # without depending on the WorldModel cutover having landed first.
    from .world_model import WorldModel
    from ..allocation import AllocationBidder
    from ..ports.sector_owner_policy import SectorOwnerPolicy
    from ..sar_patterns import CoveragePlan
    from .incoming import IncomingCandidate, IncomingTaskStatus


@dataclass
class Mission:
    """Aggregate root for one SAR mission.

    Construct empty, then populate the fleet and tick. The aggregate
    is NOT thread-safe: the ROS adapter ensures all calls happen on a
    single executor thread (the same model mission_manager.py uses
    today).
    """
    stage: MissionStage = MissionStage.INIT
    drones: Dict[str, Drone] = field(default_factory=dict)
    victims: Dict[int, Victim] = field(default_factory=dict)
    sub_missions: Dict[int, VictimSubMission] = field(default_factory=dict)
    mission_start_sec: Optional[float] = None
    sectors_total: int = 0
    # Planning-layer collaborators + config, injected by the composition
    # root (the L2 adapter sets them from its on_configure-built strategy
    # and its runtime-tweakable parameters). Defaults keep an un-wired
    # Mission inert: plan() with no strategy returns ().
    _allocation_strategy: Optional['AllocationBidder'] = None
    _sector_owner_policy: Optional['SectorOwnerPolicy'] = None
    investigate_confidence_floor: float = 0.90
    max_concurrent_investigations: int = 1
    investigate_hover_seconds: float = 4.0
    # The multi-view INVESTIGATE plan the L3 layer commits per candidate
    # and stamps onto the INVESTIGATE OutgoingTask: orbit radius,
    # per-angle dwell, and the angle set (4 cardinals by default). The
    # executor falls back to its own config when these are absent on the
    # wire (bag-replay safe).
    investigate_radius_m: float = 5.0
    investigate_dwell_s: float = 2.0
    investigate_angles: Tuple[float, ...] = DEFAULT_INVESTIGATE_ANGLES
    # CONFIRM-dispatch config (cross-drone handoff).
    confirm_hover_seconds: float = 6.0
    confirm_orbit_radius: float = 4.0
    # Tick config + telemetry sink. ``_emit_event`` is the
    # legacy-signature emit callable injected by the L2 adapter (or a
    # recorder in tests); ``_strategy_is_batch`` selects the batch vs
    # per-victim re-dispatch path; ``_published_complete`` guards single
    # MISSION_COMPLETE/TIMEOUT emission.
    reject_age_seconds: float = 60.0
    task_status_timeout_seconds: float = 30.0
    mission_timeout_seconds: float = 600.0
    _strategy_is_batch: bool = False
    _published_complete: bool = False
    _emit_event: Optional[Callable[..., None]] = None

    def register_drone(self, drone: Drone) -> None:
        """Add a drone to the fleet at startup."""
        self.drones[drone.name] = drone

    def transition(self, event: TransitionEvent) -> None:
        """Apply a transition to the mission's stage; raises on illegal."""
        self.stage = MissionStateMachine.transition(self.stage, event)

    def ensure_sub_mission(self, victim: Victim) -> VictimSubMission:
        """Get or create the VictimSubMission for a victim.

        Called by the ``on_candidate`` callback when a new
        ``VictimCandidate`` arrives. Idempotent: repeated calls return
        the same aggregate.
        """
        sub = self.sub_missions.get(victim.candidate_id)
        if sub is not None:
            sub.victim = victim   # refresh latest position/confidence
            return sub
        sub = VictimSubMission(victim=victim)
        self.sub_missions[victim.candidate_id] = sub
        self.victims[victim.candidate_id] = victim
        return sub

    # CQRS read API on the Mission aggregate. The strictly-read
    # precursor to the deferred saga lift. These methods walk the same
    # dicts the saga mutates today; mission_manager._publish_state and
    # friends delegate to them so the read shape is unified before the
    # writes migrate.
    def confirmed_count(self) -> int:
        """Number of victims whose sub-mission reached CONFIRMED.

        Mirrors ``mission_manager._victims_confirmed_count`` (which
        walks ``self._victims`` checking ``confirmed``)."""
        return sum(
            1 for sm in self.sub_missions.values()
            if sm.stage == VictimStage.CONFIRMED
        )

    def unconfirmed_candidates(self) -> Sequence[Victim]:
        """Victims still in DETECTED / INVESTIGATING, i.e. not yet
        CONFIRMED and not REJECTED. Ordering: by candidate_id (stable
        across ticks)."""
        out = []
        for vid in sorted(self.victims.keys()):
            v = self.victims[vid]
            if v.stage in (VictimStage.DETECTED, VictimStage.INVESTIGATING):
                out.append(v)
        return tuple(out)

    def busy_count(self) -> int:
        """Number of drones currently assigned to a non-IDLE task.

        Counts any drone with ``current_task_type != IDLE`` (includes
        SCAN_WAYPOINTS). Use ``dispatched_investigate_count`` for the
        narrower "drones currently mid-INVESTIGATE / mid-CONFIRM"
        question.
        """
        return sum(
            1 for d in self.drones.values()
            if d.current_task_type != TaskType.IDLE
        )

    def dispatched_investigate_count(self) -> int:
        """Number of drones currently assigned to a victim, i.e.
        mid-INVESTIGATE or mid-CONFIRM (``busy_with_victim is not None``).

        The narrower query ``mission_manager._dispatch_investigate``
        needs to enforce ``max_concurrent_investigations``. Distinct from
        ``busy_count`` (which also counts SCAN_WAYPOINTS).
        """
        return sum(
            1 for d in self.drones.values()
            if d.busy_with_victim is not None
        )

    def survivor_set(self, dead_drone: str) -> Sequence[Drone]:
        """The drones that should pick up the sector reassignment
        after ``dead_drone`` goes down. Excludes the dead drone and
        any other ``is_down`` drones; returns the remaining fleet."""
        out = []
        for name in sorted(self.drones.keys()):
            if name == dead_drone:
                continue
            d = self.drones[name]
            if d.is_down:
                continue
            out.append(d)
        return tuple(out)

    def remaining_scan_waypoints_total(self) -> int:
        """Sum of remaining scan-plan waypoints across the fleet:
        the dashboard's coverage-progress denominator."""
        return sum(
            d.remaining_scan_waypoints for d in self.drones.values()
        )

    # Separate multi-robot allocation (L3 cooperative coordination,
    # slides p. 140-145) from per-victim saga (L2 coordinative execution,
    # slides p. 38). Pre saga lift these are seams in the Mission API
    # surface, not yet a behavioural change: the runtime code in
    # mission_manager.py continues to dispatch through the legacy path.
    # Later work wires these as the actual dispatch entry points.
    def candidates_awaiting_allocation(self) -> Sequence[Victim]:
        """L3 collective-allocation input: the unconfirmed candidates
        ordered for the next auction round. Slides p. 145 (Level-3
        automation): "Robots autonomously decide both which tasks to
        perform and which robots should collaborate."

        Pure read; equivalent to ``unconfirmed_candidates()`` but named
        to make the L3 allocation intent visible at the call site. The
        auction engine consumes the returned tuple as its candidate set.
        """
        return self.unconfirmed_candidates()

    def free_drones(self) -> Sequence[Drone]:
        """L3 allocation input: the drones eligible to bid in the next
        allocation round. Ordering by name (stable across ticks).
        Excludes ``is_down`` and currently-busy drones (the
        ``current_task_type != IDLE`` predicate matches the legacy
        ``MissionManager._free_drones`` filter).
        """
        return tuple(
            self.drones[name] for name in sorted(self.drones.keys())
            if not self.drones[name].is_down
            and self.drones[name].current_task_type == TaskType.IDLE
        )

    def ensure_sub_mission_for_each(
        self, candidates: Sequence[Victim],
    ) -> Sequence[VictimSubMission]:
        """L2 saga preparation: for each freshly-allocated candidate,
        idempotently obtain its ``VictimSubMission``. Slides p. 38:
        per-victim saga is executive-layer (L2) coordinative state.

        Distinct from ``ensure_sub_mission(victim)`` (singular,
        existing) by being the batch entry point the planner calls after
        the allocation auction returns. The returned sequence is
        parallel to the input ``candidates``.
        """
        return tuple(self.ensure_sub_mission(c) for c in candidates)

    # Pure-domain port of mission_manager._reassign_sector +
    # _nearest_survivor + _absorb_wedge (lines 758-821 / 701-756). Works
    # on Drone entities (sector_wedge is a SectorWedge VO, scan_waypoints
    # the dispatched 3D list). Returns OutgoingTask records: the L2
    # adapter publishes them and emits the SECTOR_REASSIGNED telemetry
    # event.
    def handle_drone_lost(
        self, dead_drone_name: str, now_sec: float,
    ) -> Sequence[OutgoingTask]:
        """Hand a lost drone's remaining scan waypoints, as one
        contiguous ordered run, to the angularly-nearest surviving
        sector owner, widen that survivor's wedge, and (if the survivor
        is mid-SCAN) return a preempting SCAN task so it picks up the
        extended tail immediately.

        Survivors mid-INVESTIGATE / mid-CONFIRM are left alone; they
        see the appended tail when ``on_task_completed`` re-dispatches
        SCAN. Returns ``()`` when there are no survivors or no
        remaining waypoints.
        """
        dead = self.drones.get(dead_drone_name)
        if dead is None:
            return ()
        # Receivers must be staying AND able to take work: not down, and not
        # themselves returning home (battery_ok is the "no further tasks" gate
        # set on RTH). Without the battery_ok guard a sector could be handed to
        # a peer that is itself flying home, orphaning it again.
        survivors = [
            d for d in self.drones.values()
            if not d.is_down and d.battery_ok and d.name != dead_drone_name
        ]
        if not survivors:
            return ()
        # Orphan tail: every waypoint not yet COMPLETED. scan_cursor only
        # advances on COMPLETED dispatch, so [cursor:] is the safe
        # known-unfinished run.
        remaining = list(dead.scan_waypoints[dead.scan_cursor:])
        if not remaining:
            return ()

        receiver = self._nearest_survivor(dead, survivors, remaining[0])
        receiver.append_scan_tail(remaining)
        self._absorb_wedge(receiver, dead)
        dead.clear_scan_state()

        tasks: List[OutgoingTask] = []
        if receiver.current_task_type == TaskType.SCAN:
            tail = receiver.scan_waypoints[receiver.scan_cursor:]
            tasks.append(OutgoingTask(
                drone_name=receiver.name,
                task_type=int(TaskType.SCAN),
                waypoints=tuple((p.x, p.y, p.z) for p in tail),
                target=None,
                victim_id=0,
                priority=1,
                hover_seconds=0.0,
            ))
        return tuple(tasks)

    # Recovery callbacks (MissionPort). These compose the lifted recovery
    # logic into the callback-level entry points the post-cutover
    # ``mission_manager_node`` will call. They are pure-domain +
    # rclpy-free; the L2 adapter translates the inbound ROS
    # health/battery messages and publishes the returned tasks. The
    # remaining MissionPort surface (on_candidate / on_task_status /
    # on_survey_start + the _begin_scan coverage port) and the
    # legacy-record retirement + flag flip require the ROS smoke gate and
    # are deferred to that environment.
    def on_drone_health(
        self, drone_name: str, unrecoverable: bool, now_sec: float,
    ) -> Sequence[OutgoingTask]:
        """A drone reported unrecoverable: mark it DOWN and reassign
        its remaining sector to survivors (composes
        ``handle_drone_lost``). Idempotent: a second unrecoverable
        report for an already-down drone is a no-op. Returns the
        preempting SCAN task(s) for the survivor, if any.
        """
        d = self.drones.get(drone_name)
        if d is None or not unrecoverable or d.is_down:
            return ()
        d.is_down = True
        return self.handle_drone_lost(drone_name, now_sec)

    def on_battery_low(
        self, drone_name: str, now_sec: float = 0.0,
    ) -> Sequence[OutgoingTask]:
        """Battery-low admission gate fired: mark the drone
        battery-not-ok, return an RTH task, AND hand its remaining sector
        to a healthy peer so its area is still covered. Mirrors
        ``mission_manager._on_battery``'s RTH dispatch. Idempotent on
        the ``battery_ok`` flag (returns () if already not-ok).

        Regression fix: a returning drone used to keep (orphan) its
        sector; only the DOWN path reassigned. Now RTH composes
        ``handle_drone_lost`` too (battery_ok=False excludes it as a
        receiver), so its waypoints are picked up by a survivor."""
        d = self.drones.get(drone_name)
        if d is None or not d.battery_ok:
            return ()
        d.battery_ok = False
        rth = OutgoingTask(
            drone_name=drone_name,
            task_type=int(TaskType.RTH),
            waypoints=(), target=None,
            victim_id=0, priority=3, hover_seconds=0.0,
        )
        return (rth, *self.handle_drone_lost(drone_name, now_sec))

    def on_candidate(
        self, incoming: 'IncomingCandidate', now_sec: float,
    ) -> Sequence[OutgoingTask]:
        """Register / update a victim candidate and (if DETECTED)
        dispatch INVESTIGATE. Pure-domain port of
        ``mission_manager._on_candidate``.

        - New candidate → register Victim + sub-mission, emit
          CANDIDATE_DETECTED.
        - Existing → refresh position / confidence / last_update.
        - Already confirmed upstream (detection_filter) → auto-CONFIRM,
          emit VICTIM_CONFIRMED, skip the INVESTIGATE step.
        - Otherwise, if DETECTED, dispatch via the batch or
          per-candidate path (matching the strategy).

        Emits telemetry via the injected ``_emit_event`` callable.
        """
        cid = incoming.candidate_id
        v = self.victims.get(cid)
        if v is None:
            v = Victim(
                candidate_id=cid, position=incoming.position,
                confidence=incoming.confidence, last_update_sec=now_sec,
            )
            self.victims[cid] = v
            self.ensure_sub_mission(v)
            self._emit(
                'CANDIDATE_DETECTED',
                detail=(f'conf={incoming.confidence:.2f}, '
                        f'reporters={list(incoming.reporting_drones)}'),
                victim_id=cid, position=incoming.position,
                confidence=incoming.confidence,
            )
        else:
            v.position = incoming.position
            v.confidence = incoming.confidence
            v.last_update_sec = now_sec

        if incoming.confirmed and v.stage == VictimStage.DETECTED:
            v.stage = VictimStateMachine.transition(
                v.stage, TransitionEvent.CONFIRMED,
            )
            self.ensure_sub_mission(v).stage = v.stage
            self._emit(
                'VICTIM_CONFIRMED',
                detail='auto-confirmed by detection_filter',
                victim_id=cid, position=incoming.position,
            )
            return ()

        if v.stage != VictimStage.DETECTED:
            return ()
        world = self.snapshot_world(now_sec)
        if self._strategy_is_batch:
            tasks = list(self.plan(world))
        else:
            tasks = list(self.plan_for(v, world))
        for t in tasks:
            self._emit(
                'INVESTIGATE_DISPATCHED', drone_name=t.drone_name,
                detail=f'auctioned to {t.drone_name}',
                victim_id=t.victim_id,
                position=(Position(*t.target) if t.target else None),
            )
        return tuple(tasks)

    def on_task_status(
        self, incoming: 'IncomingTaskStatus', now_sec: float,
    ) -> Sequence[OutgoingTask]:
        """Route an inbound task-status update. Pure-domain port of
        ``mission_manager._on_task_status``.

        - Bumps the drone's watchdog clock (proof of life) on ANY status.
        - Ignores stale status (task_id != the drone's current task).
        - IN_PROGRESS for SCAN with ``detail='wp=N'`` advances the
          absolute scan cursor (offset + local index).
        - COMPLETED → ``on_task_completed``; FAILED → ``replan``.

        Status codes per ``IncomingTaskStatus`` (IN_PROGRESS=1,
        COMPLETED=2, FAILED=3).
        """
        d = self.drones.get(incoming.drone_name)
        if d is None:
            return ()
        d.clock = replace(d.clock, last_status_t=now_sec)
        if incoming.task_id != d.current_task_id:
            return ()  # stale status from a preempted task

        if (incoming.status == 1
                and d.current_task_type == TaskType.SCAN
                and incoming.detail.startswith('wp=')):
            try:
                idx = int(incoming.detail.split('=', 1)[1])
                d.scan_cursor = max(
                    d.scan_cursor, d.clock.last_dispatch_offset + idx,
                )
            except ValueError:
                pass
            return ()

        completed = OutgoingTask(
            drone_name=d.name, task_type=int(d.current_task_type),
            waypoints=(), target=None,
            victim_id=d.busy_with_victim or 0, priority=0, hover_seconds=0.0,
        )
        world = self.snapshot_world(now_sec)
        if incoming.status == 2:        # COMPLETED
            return self.on_task_completed(world, completed)
        if incoming.status == 3:        # FAILED
            return self.replan(world, completed)
        return ()

    def begin_scan(
        self,
        coverage_plan: 'CoveragePlan',
        elevation_at: Callable[[float, float], float],
        survey_altitude: float,
        drone_order: Optional[Sequence[str]] = None,
    ) -> Sequence[OutgoingTask]:
        """Assign each drone its coverage scan plan + return one SCAN
        task per drone. Pure-domain port of the entity-mutation core of
        ``mission_manager._begin_scan``.

        The L2 adapter builds the ``PlannerInput`` from mission config,
        calls ``CoverageStrategy.plan_v2`` (both rclpy-free already),
        and passes the resulting ``CoveragePlan`` here along with an
        ``elevation_at(x, y) -> float`` callable and the survey
        altitude (AGL). Waypoints get ``z = survey_altitude +
        elevation_at(x, y)`` so the drone holds constant AGL over
        terrain. ``ScanPlan.wedge`` (``None`` for non-angular patterns)
        is assigned to ``Drone.sector_wedge``.

        Transitions the mission to SCANNING and emits SCANNING_STARTED.
        Drone order defaults to registration order (dict insertion).
        """
        per_drone = coverage_plan.per_drone
        self.sectors_total = len(per_drone)
        order = list(drone_order) if drone_order is not None \
            else list(self.drones.keys())

        tasks: List[OutgoingTask] = []
        for drone_name, plan in zip(order, per_drone):
            d = self.drones.get(drone_name)
            if d is None:
                continue
            wps = [
                Position(x, y, survey_altitude + elevation_at(x, y))
                for (x, y) in plan.waypoints
            ]
            d.scan_waypoints = wps
            d.scan_cursor = 0
            d.sector_wedge = plan.wedge
            tasks.append(OutgoingTask(
                drone_name=drone_name,
                task_type=int(TaskType.SCAN),
                waypoints=tuple((p.x, p.y, p.z) for p in wps),
                target=None, victim_id=0, priority=1, hover_seconds=0.0,
            ))

        self.stage = MissionStateMachine.transition(
            self.stage, TransitionEvent.SURVEY_STARTED,
        )
        self._emit(
            'SCANNING_STARTED',
            detail=f'{self.sectors_total} sectors, {len(order)} drones',
        )
        return tuple(tasks)

    def _nearest_survivor(
        self, dead: Drone, survivors: Sequence[Drone], first_wp: Position,
    ) -> Drone:
        """The survivor that should absorb the dead drone's orphan run.

        Angular coverage: hand to the survivor whose wedge midpoint is
        angularly closest to the dead drone's, so the orphan arc
        extends a genuinely adjacent survivor. Non-angular (no wedge):
        fall back to the survivor whose pose is nearest the run's
        start. Mirrors mission_manager._nearest_survivor.
        """
        dead_has_wedge = dead.sector_wedge is not None
        wedge_survivors = [s for s in survivors if s.sector_wedge is not None]
        if dead_has_wedge and wedge_survivors:
            dead_mid = dead.sector_wedge.midpoint()

            def ang_gap(s: Drone) -> float:
                s_mid = s.sector_wedge.midpoint()
                return abs(math.atan2(
                    math.sin(s_mid - dead_mid), math.cos(s_mid - dead_mid),
                ))

            return min(wedge_survivors, key=ang_gap)

        def pose_gap(s: Drone) -> float:
            if s.pose is None:
                return float('inf')
            return math.hypot(s.pose.x - first_wp.x, s.pose.y - first_wp.y)

        return min(survivors, key=pose_gap)

    def _absorb_wedge(self, receiver: Drone, dead: Drone) -> None:
        """Extend the receiver's wedge over the dead drone's adjacent
        wedge (``SectorWedge.absorb``). No-op when either wedge is
        absent (non-angular pattern). Mirrors
        mission_manager._absorb_wedge but operates on the VO directly.
        """
        if receiver.sector_wedge is None or dead.sector_wedge is None:
            return
        receiver.sector_wedge = receiver.sector_wedge.absorb(dead.sector_wedge)

    def snapshot(self) -> MissionStateSnapshot:
        """Return a read-only view for the adapter to publish on
        operator topics. Decouples publish cadence from mutation.
        """
        sectors_completed = sum(
            1 for d in self.drones.values()
            if d.scan_plan is not None and d.scan_cursor >= d.scan_plan.length
        )
        victims_found = sum(
            1 for v in self.victims.values()
            if v.stage != VictimStage.REJECTED
        )
        # Delegate to the read-side query helper.
        victims_confirmed = self.confirmed_count()
        # Per-drone summary string (compact, dashboard-friendly).
        summary: List[str] = []
        for d in self.drones.values():
            cursor = (f'({d.scan_cursor}/{d.scan_plan.length})'
                      if d.scan_plan is not None else '')
            # `is not None` (not a falsy check) so victim id 0 still gets
            # a `[v0]` indicator. Mirrors the correct guard in
            # DroneRecord.__repr__.
            busy = (
                f'[v{d.busy_with_victim}]'
                if d.busy_with_victim is not None else ''
            )
            summary.append(f'{d.name}:t{d.current_task_type}{cursor}{busy}')
        return MissionStateSnapshot(
            stage=int(self.stage),
            sectors_total=self.sectors_total,
            sectors_completed=sectors_completed,
            victims_found=victims_found,
            victims_confirmed=victims_confirmed,
            active_tasks_summary=tuple(summary),
        )

    def tick(self, now_sec: float) -> Sequence[OutgoingTask]:
        """One step of the mission loop. Pure-domain port of
        ``mission_manager._tick`` (4 sub-tasks):

          1. decay stale DETECTED candidates to REJECTED
          2. re-attempt INVESTIGATE dispatch (batch or per-victim)
          3. task watchdog: emit TASK_TIMEOUT on prolonged silence
          4. mission completion / timeout transition

        Returns the re-attempt dispatch ``OutgoingTask`` records (the L2
        adapter publishes them); emits telemetry via the injected
        ``_emit_event`` callable. The adapter publishes mission state
        separately after the tick.
        """
        # 1. Decay stale candidates that never confirmed.
        if self.mission_start_sec is not None:
            for v in list(self.victims.values()):
                if (v.stage == VictimStage.DETECTED
                        and now_sec - v.last_update_sec
                        > self.reject_age_seconds):
                    v.stage = VictimStateMachine.transition(
                        v.stage, TransitionEvent.REJECTED,
                    )
                    self._emit(
                        'CANDIDATE_REJECTED',
                        detail=(f'no follow-up in '
                                f'{self.reject_age_seconds:.0f}s'),
                        victim_id=v.candidate_id, position=v.position,
                    )

        # 2. Re-attempt dispatch. Batch-capable strategies assign the
        # DETECTED pool jointly; greedy / round-robin keep the
        # per-candidate path so the RNG stream stays byte-identical.
        world = self.snapshot_world(now_sec)
        if self._strategy_is_batch:
            dispatched = list(self.plan(world))
        else:
            dispatched = []
            for v in self.victims.values():
                if (v.stage == VictimStage.DETECTED
                        and v.assigned_drone is None):
                    dispatched.extend(self.plan_for(v, world))
        for t in dispatched:
            self._emit(
                'INVESTIGATE_DISPATCHED', drone_name=t.drone_name,
                detail=f'auctioned to {t.drone_name}',
                victim_id=t.victim_id,
                position=(Position(*t.target) if t.target else None),
            )

        # 3. Task watchdog: emit TASK_TIMEOUT on prolonged silence and
        # reset the clock so we don't re-emit every tick (the actual
        # reassignment is health_monitor + handle_drone_lost's job).
        for d in self.drones.values():
            if d.is_down or d.current_task_type == TaskType.IDLE:
                continue
            silence = d.clock.silence(now_sec)
            if silence > self.task_status_timeout_seconds:
                self._emit(
                    'TASK_TIMEOUT', drone_name=d.name,
                    detail=(f'silent {silence:.0f}s on task '
                            f'#{d.current_task_id}'),
                )
                d.clock = replace(d.clock, last_status_t=now_sec)

        # 4. Mission completion / timeout.
        if self.mission_start_sec is not None and not self._published_complete:
            elapsed = now_sec - self.mission_start_sec
            all_idle = self.busy_count() == 0
            no_pending = len(self.unconfirmed_candidates()) == 0
            if all_idle and no_pending:
                self.stage = MissionStateMachine.transition(
                    self.stage, TransitionEvent.MISSION_COMPLETE,
                )
                self._published_complete = True
                self._emit(
                    'MISSION_COMPLETE',
                    detail=(f'{self.confirmed_count()} confirmed in '
                            f'{elapsed:.0f}s'),
                )
            elif elapsed > self.mission_timeout_seconds:
                self.stage = MissionStateMachine.transition(
                    self.stage, TransitionEvent.MISSION_TIMEOUT,
                )
                self._published_complete = True
                self._emit(
                    'MISSION_TIMEOUT',
                    detail=(f'{self.confirmed_count()} confirmed in '
                            f'{elapsed:.0f}s'),
                )

        return tuple(dispatched)

    def _emit(self, event_type: str, **kwargs) -> None:
        """Route a telemetry event through the injected emit callable
        (the L2 adapter's ``_emit_event`` facade, or a recorder in
        tests). No-op when un-wired so a bare Mission stays inert."""
        if self._emit_event is not None:
            self._emit_event(event_type, **kwargs)

    # These two methods are the concrete-Mission implementation of the
    # ``DeliberativePlanner`` Protocol (``lib/ports/deliberative_planner.py``).
    # Adding the signatures now means the Protocol surface is real
    # (Mission IS-A DeliberativePlanner once these are filled in), so
    # future calls to ``DeliberativePlanner.plan(world)`` can be
    # type-checked against the same shape they will resolve to once the
    # cutover lands.
    def plan(self, world: 'WorldModel') -> Sequence[OutgoingTask]:
        """Layer-3 planning entry point. See slides p. 42-44.

        Batch INVESTIGATE dispatch (the joint cost-minimising pass for
        batch-capable strategies, e.g. hungarian). Pure-domain port of
        ``mission_manager._drain_investigate_batch``:

          Pass A: sector-owner first refusal (direct dispatch).
          Pass B: joint optimal assignment of the owner-unavailable
                  candidates, bounded by the remaining concurrency
                  budget.

        Greedy / round-robin strategies keep the per-candidate
        ``plan_for`` path (called by the L2 adapter) so their RNG
        tie-break stream stays byte-identical. Returns the INVESTIGATE
        ``OutgoingTask`` records; the L2 adapter publishes them and
        emits the per-dispatch telemetry events.
        """
        if self._allocation_strategy is None:
            return ()
        free = (self.max_concurrent_investigations
                - self.dispatched_investigate_count())
        if free <= 0:
            return ()
        queue = sorted(
            (v for v in self.victims.values()
             if v.stage == VictimStage.DETECTED
             and v.assigned_drone is None
             and v.confidence >= self.investigate_confidence_floor),
            key=lambda v: v.candidate_id,
        )
        if not queue:
            return ()

        tasks: List[OutgoingTask] = []
        used: set = set()
        pooled: List[Victim] = []
        # Pass A: sector-owner first refusal.
        for v in queue:
            if free <= 0:
                return tuple(tasks)
            owner = self._owner_if_available(v, used)
            if owner is not None:
                tasks.append(self._commit_investigate(v, owner))
                used.add(owner)
                free -= 1
                continue
            pooled.append(v)

        # Pass B: joint optimal assignment of the owner-unavailable
        # candidates, bounded by the remaining concurrency budget.
        if not pooled or free <= 0:
            return tuple(tasks)
        batch = pooled[:free]
        # Hand the allocator domain Position VOs (z=0); the auction only
        # reads .x/.y.
        winners = self._allocation_strategy.assign(
            [Position(v.position.x, v.position.y, 0.0) for v in batch],
            priority=2, exclude=used,
        )
        for v, winner in zip(batch, winners):
            if winner is not None:
                tasks.append(self._commit_investigate(v, winner))
        return tuple(tasks)

    def plan_for(
        self, victim: Victim, world: 'WorldModel',
    ) -> Sequence[OutgoingTask]:
        """Single-victim INVESTIGATE dispatch. Pure-domain port of
        ``mission_manager._dispatch_investigate``.

        Applies the confidence floor, the concurrency cap, then sector-
        owner first refusal with auction fallback. Returns a 0- or
        1-element tuple. The L2 adapter calls this per new candidate
        (and per DETECTED candidate on a greedy/round-robin tick) so
        the bid order + RNG stream stay byte-identical to the legacy
        path.
        """
        if self._allocation_strategy is None:
            return ()
        if victim.confidence < self.investigate_confidence_floor:
            return ()
        if (self.dispatched_investigate_count()
                >= self.max_concurrent_investigations):
            return ()
        owner = self._owner_if_available(victim, used=set())
        if owner is not None:
            winner: Optional[str] = owner
        else:
            winner = self._allocation_strategy.bid(victim.position, 2)
        if winner is None:
            return ()
        return (self._commit_investigate(victim, winner),)

    def _owner_if_available(
        self, victim: Victim, used: set,
    ) -> Optional[str]:
        """Sector owner of ``victim``, but only if it's a usable winner
        (up, battery-ok, free, and not already ``used`` this pass).
        ``None`` → caller opens / falls through to the auction.
        """
        if self._sector_owner_policy is None:
            return None
        owner = self._sector_owner_policy.owner_for(victim.position)
        if owner is None or owner in used:
            return None
        d = self.drones.get(owner)
        if d is None or d.is_down or not d.battery_ok \
                or d.busy_with_victim is not None:
            return None
        return owner

    def _commit_investigate(
        self, victim: Victim, winner: str,
    ) -> OutgoingTask:
        """Dispatch tail shared by ``plan`` (batch) and ``plan_for``
        (single): cache the CONFIRM witness runner-up on the victim's
        ``VictimSubMission``, transition the victim into INVESTIGATING,
        mark the winner busy, advance the mission stage, and return the
        INVESTIGATE ``OutgoingTask``.

        Pure-domain port of ``mission_manager._commit_investigate``.
        Event emission stays in the L2 adapter until the ``EventPort`` is
        wired onto the aggregate.
        """
        # Cache the runner-up for the CONFIRM witness handover.
        # ``exclude={winner}`` keeps the witness distinct from the
        # investigator (whether the winner came from the auction or the
        # sector-owner path).
        runner_up_bids = self._allocation_strategy.top_bids(
            victim.position, priority=2, n=2, exclude={winner},
        )
        sub = self.ensure_sub_mission(victim)
        sub.witness_drone = (
            runner_up_bids[0].bidder if runner_up_bids else None
        )

        victim.stage = VictimStateMachine.transition(
            victim.stage, TransitionEvent.INVESTIGATE_BEGAN,
        )
        victim.assigned_drone = winner
        d = self.drones[winner]
        d.busy_with_victim = victim.candidate_id

        if self.stage == MissionStage.SCANNING:
            self.stage = MissionStateMachine.transition(
                self.stage, TransitionEvent.INVESTIGATE_DISPATCHED,
            )

        return OutgoingTask(
            drone_name=winner,
            task_type=int(TaskType.INVESTIGATE),
            waypoints=(),
            target=(victim.position.x, victim.position.y, victim.position.z),
            victim_id=victim.candidate_id,
            priority=2,
            hover_seconds=self.investigate_hover_seconds,
            # Stamp the multi-view orbit so the L1 executor flies the plan
            # the deliberative layer chose, not its own defaults.
            investigate_radius=self.investigate_radius_m,
            dwell_s=self.investigate_dwell_s,
            investigate_angles=self.investigate_angles,
        )

    def on_task_completed(
        self, world: 'WorldModel', completed_task: OutgoingTask,
    ) -> Sequence[OutgoingTask]:
        """L3 progress monitor. Pure-domain port of
        ``mission_manager._on_task_completed`` (3 branches).

        - INVESTIGATE completed: dispatch CONFIRM to a *different*
          drone (cross-drone witness handoff) and return immediately.
        - CONFIRM completed: mark the victim CONFIRMED, free the
          confirmer.
        - SCAN completed: cursor to end of dispatched list.

        Then (for INVESTIGATE/CONFIRM) advance the scan cursor to the
        nearest unvisited waypoint and re-issue SCAN, or IDLE when the
        sector is exhausted. Returns the follow-on ``OutgoingTask``
        records; the L2 adapter publishes them and emits telemetry
        (CONFIRM_DISPATCHED / VICTIM_CONFIRMED / DRONE_SECTOR_COMPLETE)
        reconstructed from the completed-task type + returned tasks.
        """
        d = self.drones.get(completed_task.drone_name)
        if d is None:
            return ()
        ttype = TaskType(completed_task.task_type)
        vid = d.busy_with_victim if d.busy_with_victim is not None else -1

        if ttype == TaskType.INVESTIGATE:
            v = self.victims.get(vid)
            if v is not None:
                # The investigator MUST also be re-engaged once the
                # witness takes over CONFIRM, otherwise it hovers at the
                # investigation point and the greedy auction repeatedly
                # picks it for the next nearby candidate (observed in the
                # live run: drone2 stuck investigating, drone3 stuck
                # confirming, while drones with remote scan sectors went
                # silent). The degenerate "no other drone available, so
                # confirmer == investigator" case
                # (test_..._reuses_investigator_when_alone) returns only
                # CONFIRM: the drone is now busy with the new task and
                # must not be told to scan in parallel.
                confirm_task = self._dispatch_confirm(v, d)
                if confirm_task.drone_name == d.name:
                    return (confirm_task,)
                return (confirm_task, self._follow_on_task(d, ttype))
            # No victim record; fall through to resume scan.

        if ttype == TaskType.CONFIRM:
            v = self.victims.get(vid)
            if v is not None:
                v.stage = VictimStateMachine.transition(
                    v.stage, TransitionEvent.CONFIRMED,
                )
            d.busy_with_victim = None
            # Notify the allocation strategy of a successful intention so
            # its per-intention path memory accumulates. Only
            # ``MotivationWorkspaceStrategy`` implements this hook today;
            # other strategies are unaffected by the hasattr guard.
            strat = self._allocation_strategy
            if strat is not None and hasattr(strat, 'on_intention_succeeded'):
                strat.on_intention_succeeded(d.name)

        if ttype == TaskType.SCAN:
            d.scan_cursor = len(d.scan_waypoints)

        return (self._follow_on_task(d, ttype),)

    def _follow_on_task(self, d: Drone, ttype: TaskType) -> OutgoingTask:
        """Compute the next task for a drone whose previous one just
        completed: SCAN tail if the sector still has unvisited waypoints,
        IDLE otherwise. For INVESTIGATE/CONFIRM completions, advance the
        cursor to the nearest unvisited waypoint first (minimises the
        fly-back leg and avoids the "jump to a random arc point"
        appearance).

        Previously this logic lived inline at the bottom of
        ``on_task_completed``; the inline form silently returned ``()``
        for INVESTIGATE/CONFIRM-with-exhausted-cursor, leaving the drone
        in limbo. Extracting it as a helper and always returning either a
        SCAN or an IDLE closes that gap.
        """
        if (ttype in (TaskType.INVESTIGATE, TaskType.CONFIRM)
                and d.pose is not None
                and d.scan_cursor < len(d.scan_waypoints)):
            self._advance_scan_cursor_to_nearest(d)

        if d.scan_cursor < len(d.scan_waypoints):
            tail = d.scan_waypoints[d.scan_cursor:]
            return OutgoingTask(
                drone_name=d.name,
                task_type=int(TaskType.SCAN),
                waypoints=tuple((p.x, p.y, p.z) for p in tail),
                target=None, victim_id=0, priority=1, hover_seconds=0.0,
            )
        return OutgoingTask(
            drone_name=d.name,
            task_type=int(TaskType.IDLE),
            waypoints=(), target=None,
            victim_id=0, priority=0, hover_seconds=0.0,
        )

    def _dispatch_confirm(self, v: Victim, investigator: Drone) -> OutgoingTask:
        """CONFIRM must be done by a drone OTHER than the investigator.
        Re-check the cached witness on the victim's sub-mission; if it's
        no longer eligible (or is the investigator), run a fresh top-bid
        excluding the investigator; if nobody else is available, reuse
        the investigator. Hands the victim's busy slot to the confirmer
        when it differs.
        """
        sub = self.ensure_sub_mission(v)
        witness = self._still_eligible_witness(sub.witness_drone)
        if witness is None or witness == investigator.name:
            fresh = self._allocation_strategy.top_bids(
                v.position, priority=2, n=1, exclude={investigator.name},
            )
            witness = fresh[0].bidder if fresh else None
        confirmer = witness or investigator.name
        if witness is not None:
            investigator.busy_with_victim = None
            self.drones[witness].busy_with_victim = v.candidate_id
        return OutgoingTask(
            drone_name=confirmer,
            task_type=int(TaskType.CONFIRM),
            waypoints=(),
            target=(v.position.x, v.position.y, v.position.z),
            victim_id=v.candidate_id,
            priority=2,
            hover_seconds=self.confirm_hover_seconds,
            confirm_orbit_radius=self.confirm_orbit_radius,
        )

    def _still_eligible_witness(self, name: Optional[str]) -> Optional[str]:
        """Verify the cached CONFIRM runner-up is still a valid bidder
        (up, battery-ok, free). ``None`` means the caller falls back to a
        fresh top-bid query."""
        if name is None or name not in self.drones:
            return None
        c = self.drones[name]
        if c.is_down or not c.battery_ok or c.busy_with_victim is not None:
            return None
        return name

    def _advance_scan_cursor_to_nearest(self, d: Drone) -> None:
        """Advance ``d.scan_cursor`` to the unvisited waypoint nearest
        the drone's current pose (minimises the post-INVESTIGATE/CONFIRM
        fly-back leg). Mirrors the nearest-waypoint loop in
        ``mission_manager._on_task_completed``.
        """
        best_i = d.scan_cursor
        best_d2 = float('inf')
        for i in range(d.scan_cursor, len(d.scan_waypoints)):
            wp = d.scan_waypoints[i]
            dx = wp.x - d.pose.x
            dy = wp.y - d.pose.y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        if best_i > d.scan_cursor:
            d.scan_cursor = best_i

    def replan(
        self,
        world: 'WorldModel',
        failed_task: OutgoingTask,
    ) -> Sequence[OutgoingTask]:
        """Layer-3 compensation entry point. See slides p. 44 (type-2
        planning integration).

        Compensating action: free the victim back to DETECTED (saga
        compensation via ``VictimSubMission.compensate``) so it is
        re-auctioned next tick, then resume SCAN if the drone has
        remaining waypoints. Pure-domain port of
        ``mission_manager._on_task_failed``.
        """
        d = self.drones.get(failed_task.drone_name)
        if d is None:
            return ()
        vid = d.busy_with_victim
        if vid is not None and vid in self.victims:
            v = self.victims[vid]
            if v.stage == VictimStage.INVESTIGATING:
                # Saga compensation: VictimSubMission.compensate()
                # transitions the sub-mission back to DETECTED and
                # clears the assigned drone on both sub + victim.
                sub = self.ensure_sub_mission(v)
                sub.compensate()
                v.stage = sub.stage
            d.busy_with_victim = None
        if d.scan_cursor < len(d.scan_waypoints):
            tail = d.scan_waypoints[d.scan_cursor:]
            return (OutgoingTask(
                drone_name=d.name,
                task_type=int(TaskType.SCAN),
                waypoints=tuple((p.x, p.y, p.z) for p in tail),
                target=None, victim_id=0, priority=1, hover_seconds=0.0,
            ),)
        return ()

    def snapshot_world(self, now_sec: float) -> 'WorldModel':
        """Return the frozen ``WorldModel`` VO the planner consumes.

        Distinct from ``snapshot()`` (which returns
        ``MissionStateSnapshot`` for the operator-publish path): this is
        the L3 input shape.
        """
        # Local import: WorldModel is added in the same change, so
        # importing at module top would race the file creation order on
        # first import. Import-on-call is the safest pattern until both
        # ship together, after which this can be hoisted.
        from .world_model import WorldModel  # noqa: F401
        return _build_world_model(self, now_sec)


def _build_world_model(mission: 'Mission', now_sec: float) -> 'WorldModel':
    """Construct a ``WorldModel`` snapshot from a ``Mission`` aggregate.

    Lives as a module-level helper rather than a Mission method so the
    aggregate keeps its existing surface; the helper is the read-side
    projection callable from ``Mission.snapshot_world(now_sec)``.
    """
    from .world_model import WorldModel
    confirmed = tuple(
        v for v in mission.victims.values()
        if v.stage == VictimStage.CONFIRMED
    )
    unconfirmed = tuple(
        v for v in mission.victims.values()
        if v.stage in (VictimStage.DETECTED, VictimStage.INVESTIGATING)
    )
    coverage_total = max(1, mission.sectors_total)
    sectors_done = sum(
        1 for d in mission.drones.values()
        if d.scan_plan is not None and d.scan_cursor >= d.scan_plan.length
    )
    coverage_pct = 100.0 * sectors_done / coverage_total
    return WorldModel(
        fleet=dict(mission.drones),
        confirmed_victims=confirmed,
        unconfirmed_candidates=unconfirmed,
        coverage_pct=coverage_pct,
        active_tasks=tuple(),         # Populated post-saga-lift; today
                                      # OutgoingTask records live in
                                      # the legacy mission_manager only.
        no_fly_zones=tuple(),         # This L3 snapshot field is unused by
                                      # consumers; live no-fly-zone enforcement
                                      # (waypoint filtering + breach RTH)
                                      # lives in mission_manager, which loads
                                      # the zones directly. See
                                      # lib/domain/no_fly_zone_filter.
        now_sec=now_sec,
    )
