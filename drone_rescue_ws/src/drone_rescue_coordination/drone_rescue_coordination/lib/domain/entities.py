"""SAR domain entities.

- Drone (entity, identity=name). Mutable lifecycle. Owns its own
  ScanPlan + cursor + Optional[SectorWedge].
- Victim (entity, identity=candidate_id). Mutable lifecycle.

Both are still backwards-compatible shells around the existing
``DroneRecord`` / ``VictimRecord`` dataclasses for now; decomposing
DroneRecord's seven concerns into Drone (E) + ScanPlan (V) +
Optional[SectorWedge] (V) + WatchdogClock (V) is not yet finished.

These are NOT yet a hard replacement: the legacy ``DroneRecord`` /
``VictimRecord`` records continue to exist in mission_manager.py and
remain the source of truth at runtime. The new entities exist so the
Mission aggregate and the saga aggregate can be expressed in clean
terms; the deconstruction wires them up as later changes collapse the
legacy records.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import List, Optional, Sequence

from .state_machines import VictimStage
from .task_type import TaskType
from .value_objects import Position, ScanPlan, SectorWedge, WatchdogClock


@dataclass
class Drone:
    """A single fleet member, identified by ``name``.

    The mutable lifecycle attributes (pose, battery_ok, current_task,
    is_down, scan_cursor, watchdog clock) live here. The immutable
    ScanPlan is replaced on each new SCAN dispatch via
    ``set_plan(plan)``.

    `pose` is a domain `Position`; the ROS adapter translates inbound
    `geometry_msgs.msg.Point`. `clock` is a frozen ``WatchdogClock``
    VO; advance via
    ``d.clock = replace(d.clock, last_status_t=now)``.
    """
    name: str
    pose: Optional[Position] = None
    battery_ok: bool = True
    is_down: bool = False
    current_task_id: int = 0
    # IntEnum: legacy raw ints from TaskAssignment.* assign cleanly
    # (TaskType mirrors the ROS message values 1:1).
    current_task_type: TaskType = TaskType.IDLE
    busy_with_victim: Optional[int] = None
    scan_plan: Optional[ScanPlan] = None
    scan_cursor: int = 0
    sector_wedge: Optional[SectorWedge] = None
    clock: WatchdogClock = field(default_factory=WatchdogClock)
    # The dispatched-and-altitude-injected 3D waypoint list. Distinct
    # from ``scan_plan`` (the strategy-emitted 2D plan): survey altitude
    # + terrain elevation are baked into z at dispatch time, so the
    # saga's reassignment logic (Mission.handle_drone_lost) needs the
    # exact dispatched points. Mirrors the legacy
    # ``DroneRecord.scan_waypoints`` (parallel altitude carrier).
    scan_waypoints: List[Position] = field(default_factory=list)

    def set_plan(self, plan: ScanPlan) -> None:
        """Replace the scan plan; resets cursor + dispatch offset."""
        self.scan_plan = plan
        self.scan_cursor = 0
        self.clock = replace(self.clock, last_dispatch_offset=0)
        if plan.wedge is not None:
            self.sector_wedge = plan.wedge

    def append_scan_tail(self, extra: 'Sequence[Position]') -> None:
        """Append more waypoints to the dispatched scan list.

        Mirrors ``DroneRecord.append_scan_tail``. Used by
        ``Mission.handle_drone_lost`` to hand a dead drone's
        remaining waypoints to a survivor as one contiguous run.
        """
        self.scan_waypoints = list(self.scan_waypoints) + list(extra)

    def clear_scan_state(self) -> None:
        """Drain the scan list + reset the cursor.

        Mirrors ``DroneRecord.clear_scan_state``. Used when a drone is
        lost (its sector is reassigned) or when a scan completes.
        """
        self.scan_waypoints = []
        self.scan_cursor = 0

    @property
    def remaining_scan_waypoints(self) -> int:
        # Prefer the dispatched ``scan_waypoints`` list when populated
        # (the saga's authoritative count); fall back to the ScanPlan
        # length for the pre-dispatch scaffold path.
        if self.scan_waypoints:
            return max(0, len(self.scan_waypoints) - self.scan_cursor)
        if self.scan_plan is None:
            return 0
        return max(0, self.scan_plan.length - self.scan_cursor)


@dataclass
class Victim:
    """A candidate / confirmed victim, identified by ``candidate_id``.

    Stage transitions are owned by the post-finding-3 VictimStateMachine;
    direct field mutation is preserved for legacy paths during migration.
    """
    candidate_id: int
    position: Position
    confidence: float
    stage: VictimStage = VictimStage.DETECTED
    assigned_drone: Optional[str] = None
    last_update_sec: float = 0.0
    reporting_drones: List[str] = field(default_factory=list)
