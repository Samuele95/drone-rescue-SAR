"""MissionPortAdapter: wires the Mission aggregate to the MissionPort
contract the L2 ROS node + translator expect.

The ``MissionPort`` Protocol (lib/ports/mission_port.py) assumes a
self-contained implementor: callbacks take only the inbound VO (the
node supplies no clock), and ``on_survey_start`` takes just
``now_sec`` (the implementor knows how to plan coverage). The
``Mission`` aggregate, by contrast, is a *pure* domain object: it
takes ``now_sec`` explicitly and needs a pre-computed ``CoveragePlan``
for ``begin_scan``, so the strategy/config plumbing stays out of the
domain layer.

This adapter bridges the two: it holds a ``Mission`` + a ``Clock``
port + the coverage collaborators, and implements ``MissionPort``
exactly by supplying the clock's ``now_sec()`` and the computed
``CoveragePlan`` to the aggregate's methods. It is rclpy-free:
``Clock`` is the domain clock port, the coverage provider + elevation
are plain callables the L2 node injects. The ROS node therefore only
has to: build this adapter, create subscriptions/publishers, and route
each callback through ``mission_manager_translator`` (which calls this
adapter's MissionPort methods); no saga logic in the node.

3T boundary: L2 to L3. The adapter is the seam where the executive
layer (clock + coverage strategy + ROS plumbing) meets the
deliberative planner (the pure ``Mission`` aggregate).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Optional, Sequence

if TYPE_CHECKING:
    from .domain.incoming import (
        IncomingCandidate, IncomingHealth, IncomingTaskStatus,
    )
    from .domain.mission import Mission
    from .domain.value_objects import MissionStateSnapshot, OutgoingTask
    from .ports.clock import Clock
    from .sar_patterns import CoveragePlan


# (x, y) -> terrain elevation. The L2 node passes
# ``elevation_model.elevation_at``.
ElevationFn = Callable[[float, float], float]
# () -> the freshly-computed coverage plan for the fleet. The L2 node
# captures the PlannerInput + strategy and exposes this thunk.
CoveragePlanProvider = Callable[[], 'CoveragePlan']


class MissionPortAdapter:
    """Concrete ``MissionPort`` over a pure ``Mission`` aggregate.

    Construction (all rclpy-free):
    - ``mission``: the ``lib.domain.Mission`` aggregate (already wired
      with its allocation strategy / sector-owner policy / config /
      ``_emit_event`` callable by the composition root).
    - ``clock``: the domain ``Clock`` port, supplies ``now_sec()`` so
      the MissionPort callbacks don't have to carry a timestamp.
    - ``coverage_plan_provider``: thunk returning the ``CoveragePlan``
      for ``on_survey_start`` (the node builds the ``PlannerInput`` +
      calls ``CoverageStrategy.plan_v2``).
    - ``elevation_at`` + ``survey_altitude``: AGL waypoint-z inputs for
      ``begin_scan``.
    """

    def __init__(
        self,
        mission: 'Mission',
        clock: 'Clock',
        *,
        coverage_plan_provider: Optional[CoveragePlanProvider] = None,
        elevation_at: ElevationFn = lambda x, y: 0.0,
        survey_altitude: float = 25.0,
    ):
        self._mission = mission
        self._clock = clock
        self._coverage_plan_provider = coverage_plan_provider
        self._elevation_at = elevation_at
        self._survey_altitude = survey_altitude

    # MissionPort
    def on_candidate(
        self, c: 'IncomingCandidate',
    ) -> Sequence['OutgoingTask']:
        return self._mission.on_candidate(c, self._clock.now_sec())

    def on_task_status(
        self, s: 'IncomingTaskStatus',
    ) -> Sequence['OutgoingTask']:
        return self._mission.on_task_status(s, self._clock.now_sec())

    def on_health(
        self, h: 'IncomingHealth',
    ) -> Sequence['OutgoingTask']:
        # DroneHealth carries both the unrecoverable flag and battery
        # state; route each to the matching aggregate callback.
        tasks = list(self._mission.on_drone_health(
            h.drone_name, h.is_down, self._clock.now_sec(),
        ))
        if not h.battery_ok:
            tasks.extend(self._mission.on_battery_low(h.drone_name))
        return tuple(tasks)

    def on_battery_low(
        self, drone_name: str,
    ) -> Sequence['OutgoingTask']:
        return self._mission.on_battery_low(drone_name)

    def on_survey_start(
        self, now_sec: float,
    ) -> Sequence['OutgoingTask']:
        # Record the mission start time so the aggregate's tick() decay
        # + completion sub-tasks run, then assign coverage.
        self._mission.mission_start_sec = now_sec
        if self._coverage_plan_provider is None:
            return ()
        plan = self._coverage_plan_provider()
        return self._mission.begin_scan(
            plan, self._elevation_at, self._survey_altitude,
        )

    def tick(self, now_sec: float) -> Sequence['OutgoingTask']:
        return self._mission.tick(now_sec)

    def state_snapshot(self) -> 'MissionStateSnapshot':
        return self._mission.snapshot()
