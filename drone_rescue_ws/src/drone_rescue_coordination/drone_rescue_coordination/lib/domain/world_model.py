"""WorldModel: Layer-3 deliberative belief-state snapshot (3T architecture).

The slides (Marcelletti, "Autonomous and Collaborative Robotics",
A.Y. 2025/26) require deliberative planning to operate on an
internal symbolic representation of the world:

> Slides p. 76, Deliberative ("Think, Then Act"): "Planning is
>   fundamental and requires the existence of an internal, symbolic
>   representation of the world, internal model must be updated."
>
> Slides p. 25, Sensing & Perception: "update the model with new
>   information contained in the sensor data."

Today the deliberative planner (mission_manager.py) reads scattered
fields (``self._drones``, ``self._victims``, ``self._stage``, ...) to
make planning decisions. There is no named type for "the planner's
current belief state". This module fills that gap.

``WorldModel`` is the single shape the
``DeliberativePlanner.plan(world)`` (lib/ports/deliberative_planner.py)
Protocol consumes. It is produced by ``Mission.snapshot_world(now_sec)``
(see ``lib/domain/mission.py``), the read-side projection of the
``Mission`` aggregate.

Frozen: the planner sees a snapshot, never a live mutable mirror.
CQRS read-side discipline.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Mapping, Tuple

if TYPE_CHECKING:
    from .entities import Drone, Victim
    from .value_objects import NoFlyZone, OutgoingTask


@dataclass(frozen=True)
class WorldModel:
    """Frozen snapshot of the deliberative planner's belief state.

    All collections are immutable: ``Mapping`` for fleet (planner
    iterates by name), ``Tuple`` for victims / tasks / zones (order
    is stable across snapshots).

    Field justification (slides anchor in parens):

    - ``fleet``: current drone roster + per-drone task state. The
      planner consults this to decide who is free for dispatch
      (slides p. 145, Level-3 task allocation).
    - ``confirmed_victims``: victims that have completed the
      INVESTIGATE->CONFIRM saga. The planner does not re-dispatch
      these (slides p. 79, drone mission planning).
    - ``unconfirmed_candidates``: victims pending investigation or
      mid-INVESTIGATE. The planner's primary input for the next
      dispatch decision.
    - ``coverage_pct``: pheromone-derived disk coverage. Used to
      decide when to terminate the mission (slides p. 124,
      stigmergy coverage).
    - ``active_tasks``: what is currently on the wire. The planner
      uses this to avoid double-dispatch (slides p. 38, executive
      layer monitors).
    - ``no_fly_zones``: geometric constraints applied to candidate
      waypoints (slides p. 27, collision avoidance).
    - ``now_sec``: the snapshot's wall-clock; used for time-bounded
      planning decisions (battery-low predictions, timeouts).
    """
    fleet: Mapping[str, 'Drone']
    confirmed_victims: Tuple['Victim', ...]
    unconfirmed_candidates: Tuple['Victim', ...]
    coverage_pct: float
    active_tasks: Tuple['OutgoingTask', ...]
    no_fly_zones: Tuple['NoFlyZone', ...]
    now_sec: float

    @property
    def fleet_size(self) -> int:
        return len(self.fleet)

    @property
    def victims_seen(self) -> int:
        """Number of distinct victims known (confirmed + pending)."""
        return len(self.confirmed_victims) + len(self.unconfirmed_candidates)

    def idle_drones(self) -> Tuple['Drone', ...]:
        """Sub-set of ``fleet`` currently IDLE (free for dispatch)."""
        # Local import to avoid TYPE_CHECKING circularity at runtime.
        from .task_type import TaskType
        return tuple(
            d for d in self.fleet.values()
            if d.current_task_type == TaskType.IDLE and not d.is_down
        )
