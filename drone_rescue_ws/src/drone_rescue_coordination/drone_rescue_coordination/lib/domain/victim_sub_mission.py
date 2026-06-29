"""VictimSubMission aggregate root.

The (Victim + assigned Drone + dispatched-at + dwell-clock + saga step)
cluster is a DDD aggregate: there is a transactional boundary
(INVESTIGATE -> CONFIRM must succeed or compensate via RTH). The saga
state otherwise lives spread across DroneRecord.busy_with_victim,
VictimRecord.stage, VictimRecord.assigned_drone, and three methods on
mission_manager. This file gathers them into one aggregate root.

Skeleton at this stage: the full migration of mission_manager's saga
methods (`_dispatch_investigate`, `_dispatch_confirm`,
`_on_task_completed`, `_on_task_failed`) into this class is gated on the
mission_manager deconstruction. Until then, this aggregate exists so the
Mission.tick() path can be expressed in clean terms; legacy paths in
mission_manager.py continue to work unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from .entities import Drone, Victim
from .state_machines import (
    VictimStage, VictimStateMachine, TransitionEvent,
)
from .task_type import TaskType
from .value_objects import DEFAULT_INVESTIGATE_ANGLES, OutgoingTask


@dataclass
class VictimSubMission:
    """One per-victim saga: 3T Layer 2 coordinative execution.

    Per-victim coordinative state in the multi-robot taxonomy
    (Marcelletti, pp. 140-141): cooperative multi-robot allocation
    lives on the Mission aggregate
    (``Mission.candidates_awaiting_allocation`` / ``free_drones``);
    coordinative per-victim execution (INVESTIGATE -> CONFIRM handover,
    witness tracking, compensation) lives here.

    Owns the dispatch -> execute -> confirm | compensate flow. Methods
    return ``OutgoingTask`` records that the caller (the Mission
    aggregate or the legacy mission_manager) translates into ROS
    messages.
    """
    victim: Victim
    assigned_drone: Optional[Drone] = None
    stage: VictimStage = VictimStage.DETECTED
    dispatched_at_sec: float = 0.0
    witnesses: List[str] = field(default_factory=list)
    # Runner-up cache. At INVESTIGATE dispatch the planner caches the
    # auction's second-place bidder here so the CONFIRM handoff goes to
    # a DIFFERENT drone without re-running the auction. Distinct from
    # ``witnesses`` (the dispatch-time roster of every drone that has
    # contributed a sighting).
    witness_drone: Optional[str] = None
    # Per-victim INVESTIGATE multi-view state: the 4 cardinal orbit
    # angles still to visit plus the per-angle dwell clock. Populated by
    # ``dispatch_investigate``; the BT reads them off the OutgoingTask.
    # Saga state belongs on the per-victim aggregate, not on the
    # per-drone ExecCtx.
    investigate_angles: Tuple[float, ...] = ()
    dwell_until: float = 0.0

    def dispatch_investigate(
        self, drone: Drone, now_sec: float, hover_seconds: float = 4.0,
        investigate_radius: float = 5.0, dwell_s: float = 2.0,
        investigate_angles: Sequence[float] = DEFAULT_INVESTIGATE_ANGLES,
    ) -> OutgoingTask:
        """Mark this sub-mission INVESTIGATING and emit the task record.

        The multi-view orbit (radius, per-angle dwell, angle set) is
        recorded as saga state on this aggregate and stamped onto the
        task so the L1 executor flies the plan the coordinative layer
        chose. Raises ``IllegalTransition`` if the saga isn't in a state
        that permits dispatch (e.g. already CONFIRMED).
        """
        self.stage = VictimStateMachine.transition(
            self.stage, TransitionEvent.INVESTIGATE_BEGAN,
        )
        self.assigned_drone = drone
        self.victim.assigned_drone = drone.name
        self.dispatched_at_sec = now_sec
        self.investigate_angles = tuple(investigate_angles)
        if drone.name not in self.witnesses:
            self.witnesses.append(drone.name)
        return OutgoingTask(
            drone_name=drone.name,
            task_type=int(TaskType.INVESTIGATE),
            waypoints=(),
            target=(self.victim.position.x, self.victim.position.y, 0.0),
            victim_id=self.victim.candidate_id,
            priority=2,
            hover_seconds=hover_seconds,
            confirm_orbit_radius=0.0,
            investigate_radius=investigate_radius,
            dwell_s=dwell_s,
            investigate_angles=tuple(investigate_angles),
        )

    def dispatch_confirm(
        self, drone: Drone, now_sec: float,
        hover_seconds: float = 6.0, confirm_orbit_radius: float = 4.0,
    ) -> OutgoingTask:
        """Hand off to a SECOND drone for the CONFIRM orbit.

        Cross-drone CONFIRM: the witness drone must differ from the
        original investigator. The caller is responsible for picking
        ``drone`` via auction with ``exclude={previous_drone}``; this
        method just records the handover.
        """
        # No state-machine change here: still INVESTIGATING until the
        # CONFIRMED transition fires on completion.
        self.assigned_drone = drone
        self.victim.assigned_drone = drone.name
        if drone.name not in self.witnesses:
            self.witnesses.append(drone.name)
        return OutgoingTask(
            drone_name=drone.name,
            task_type=int(TaskType.CONFIRM),
            waypoints=(),
            target=(self.victim.position.x, self.victim.position.y, 0.0),
            victim_id=self.victim.candidate_id,
            priority=2,
            hover_seconds=hover_seconds,
            confirm_orbit_radius=confirm_orbit_radius,
        )

    def on_complete(self) -> None:
        """Saga completed cleanly: mark CONFIRMED."""
        self.stage = VictimStateMachine.transition(
            self.stage, TransitionEvent.CONFIRMED,
        )

    def compensate(self) -> None:
        """Saga failed (drone down, task timeout, battery RTH).

        Returns the victim to DETECTED so the mission auctioner can
        re-dispatch on the next tick. This is the canonical Saga
        compensation step.
        """
        self.stage = VictimStateMachine.transition(
            self.stage, TransitionEvent.INVESTIGATE_FAILED,
        )
        self.assigned_drone = None
        self.victim.assigned_drone = None
