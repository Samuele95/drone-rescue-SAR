"""EventPort: driven port for the operator event sink.

The Mission aggregate (post-deconstruction) and the saga aggregates
emit typed ``MissionEventVariant`` values into an ``EventPort``.
Concrete adapters publish on ``/mission/events`` (production), append
to a list (tests), or write to JSONL (replay).

Three nodes (``mission_manager``, ``drone_executor``,
``drone_health_monitor``) each own their own publisher to
``/mission/events`` and emit events directly. ``mission_manager``
even has a method ``_emit_event`` AND a module-level shim
``_emit_event_to_pub``, pulling the function out for testability
without committing to dependency-injection. This module commits to
DI: domain code depends on ``EventPort`` (the Protocol), the ROS
adapter implements it as a thin wrapper around the rclpy publisher.
Tests use ``InMemoryEventCapture`` to assert emissions directly
without rclpy spin-up.

The Protocol's ``emit`` takes a typed ``MissionEventVariant``
sum-type, not a stringly-typed dict.

``RosEventPublisherAdapter`` lives in
``lib/ros_adapter/event_publisher.py``: ports declare contracts; the
production adapter lives with its rclpy siblings. The Protocol and
the in-memory test fake stay here.
"""

from __future__ import annotations

from typing import List, Protocol

from ..domain.events import MissionEventVariant


# 3T boundary annotation: operator-event sink consumed across all
# three layers (Mission L3 emits saga events; DroneExecutor L1 emits
# drone-up/down; DroneHealthMonitor L2 emits diagnostics).
# Cross-cutting.
LAYER_BOUNDARY = 'cross-cutting'


class EventPort(Protocol):
    """Driven port: where mission events go.

    Concrete production implementation: ``RosEventPublisherAdapter``
    in ``lib/ros_adapter/event_publisher.py``. Tests use
    ``InMemoryEventCapture``. Replay tools wire a ``JsonlEventWriter``
    to the same port for off-line synthesis.
    """

    def emit(self, event: MissionEventVariant) -> None: ...


class InMemoryEventCapture:
    """Test fake accumulating emitted events into a list.

    Lets unit tests assert "the saga emitted a VictimConfirmed"
    without a rclpy node + publisher + subscriber dance.
    """

    def __init__(self) -> None:
        self.events: List[MissionEventVariant] = []

    def emit(self, event: MissionEventVariant) -> None:
        self.events.append(event)

    def clear(self) -> None:
        self.events.clear()


# The back-compat re-export of ``RosEventPublisherAdapter`` is gone.
# Import the production adapter from
# ``lib.ros_adapter.event_publisher`` directly. This keeps the
# dependency arrow ports -> adapters in the right direction;
# ``lib.ports`` is now importable without dragging rclpy into the
# transitive closure.
