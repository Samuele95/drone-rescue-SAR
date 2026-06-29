"""MissionPort: driver port from ROS adapter into the Mission aggregate.

The ROS adapter (post-deconstruction ``mission_manager_node.py``)
translates topic callbacks into port calls and publishes whatever
``OutgoingTask`` records the port returns. Tests can implement this
Protocol directly with a fake or use the real ``Mission`` aggregate
with a synthetic clock.

The Protocol is structural: anything with these methods qualifies.
The concrete production implementation is ``lib.domain.Mission``.

Anti-corruption invariant: this module imports no
``drone_rescue_msgs.msg.*`` and no ``geometry_msgs.msg.*``. The
adapter translates inbound ROS messages into ``IncomingCandidate``,
``IncomingTaskStatus``, ``IncomingHealth`` before calling in.
"""

from __future__ import annotations

from typing import Protocol, Sequence

from ..domain.incoming import (
    IncomingCandidate, IncomingHealth, IncomingTaskStatus,
)
from ..domain.value_objects import OutgoingTask, MissionStateSnapshot


# 3T boundary annotation: the driver port from the L2
# mission_manager_node ROS adapter into the L3 Mission aggregate.
# Mirrors ``DeliberativePlanner`` (also L2-L3) but on the
# inbound-callback side rather than the planner-query side.
LAYER_BOUNDARY = 'L2-L3'


class MissionPort(Protocol):
    """Pure-Python entry point into the SAR mission aggregate.

    No rclpy in the implementation. Each callback returns a sequence
    of ``OutgoingTask`` records; the adapter is responsible for
    converting these to ``TaskAssignment`` messages and publishing
    them on the per-drone task topics.
    """

    def on_candidate(
        self, c: IncomingCandidate,
    ) -> Sequence[OutgoingTask]: ...

    def on_task_status(
        self, s: IncomingTaskStatus,
    ) -> Sequence[OutgoingTask]: ...

    def on_health(
        self, h: IncomingHealth,
    ) -> Sequence[OutgoingTask]: ...

    def on_battery_low(
        self, drone_name: str,
    ) -> Sequence[OutgoingTask]: ...

    def on_survey_start(
        self, now_sec: float,
    ) -> Sequence[OutgoingTask]: ...

    def tick(
        self, now_sec: float,
    ) -> Sequence[OutgoingTask]: ...

    def state_snapshot(self) -> MissionStateSnapshot: ...
