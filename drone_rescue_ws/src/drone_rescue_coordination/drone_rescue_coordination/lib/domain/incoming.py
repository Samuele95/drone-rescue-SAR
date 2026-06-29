"""Domain-typed inputs to the MissionPort.

The driver port `MissionPort` must not type its callbacks in
`drone_rescue_msgs.msg.*` wire-format messages: the
anti-corruption-layer invariant is that `lib/domain/` and
`lib/ports/` are rclpy-free. The ROS adapter translates incoming ROS
messages into these frozen records before calling into the port.

Production translation lives in `lib/ros_adapter/translators.py`.
Tests construct these directly from primitives.

Field shapes mirror the actual `.msg` definitions. Any wire-format
change reflects here first.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from .value_objects import Position


@dataclass(frozen=True)
class IncomingCandidate:
    """Translated `drone_rescue_msgs.msg.VictimCandidate`."""
    candidate_id: int
    position: Position
    confidence: float = 0.0
    observation_count: int = 0
    reporting_drones: Tuple[str, ...] = ()
    confirmed: bool = False


@dataclass(frozen=True)
class IncomingTaskStatus:
    """Translated `drone_rescue_msgs.msg.TaskStatus`.

    The wire format does not carry `task_type`; consumers correlate
    `task_id` to the dispatch record they hold for that drone.
    """
    drone_name: str
    task_id: int
    status: int          # ACCEPTED=0, IN_PROGRESS=1, COMPLETED=2, FAILED=3, PREEMPTED=4
    detail: str = ''


@dataclass(frozen=True)
class IncomingHealth:
    """Translated `drone_rescue_msgs.msg.DroneHealth`.

    `is_down` is derived from `unrecoverable`; `battery_ok` from
    `battery_remaining_s` plus the mission_manager's RTH threshold
    when the adapter knows it (otherwise a conservative True).
    """
    drone_name: str
    anomaly_score: float
    is_down: bool         # mirrors DroneHealth.unrecoverable
    reason: str = ''
    battery_remaining_s: float = float('nan')
    battery_ok: bool = True
