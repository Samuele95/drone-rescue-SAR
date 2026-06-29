"""ExecutiveSupervisor: Layer-2 escalation port (3T architecture).

The slides (Marcelletti, "Autonomous and Collaborative Robotics",
A.Y. 2025/26) describe the Executive Layer as:

> Slides p. 38, Executive Layer: "Interface between behavioural and
>   planning layers, translates high-level plans into low-level
>   invocations also taking care of monitoring and handling
>   exceptions."

This Protocol names the L2-side of the L2->L3 escalation surface:
when the executive layer detects an anomaly that requires planner
re-allocation (drone lost, task failed), it reports it via this
Protocol. The deliberative planner (Layer 3) is the consumer.

Concrete implementations:
- ``RecoveryPolicy`` + ``ModeManager`` (existing) together fulfil
  this Protocol: the watchdog calls into RecoveryPolicy on missed
  heartbeats; ModeManager is the source of ``current_mode()``.
- ``InMemoryExecutiveCapture`` (in this module): test fake that
  records every call for assertions.

3T boundary: ``LAYER_BOUNDARY = 'L2-L3'``. The DeliberativePlanner
Protocol carries the same annotation; they are the two sides of the
same architectural seam.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, List, Protocol, Tuple

if TYPE_CHECKING:
    from ..domain.system_mode_machine import SystemMode


LAYER_BOUNDARY = 'L2-L3'   # 3T architecture annotation.


class ExecutiveSupervisor(Protocol):
    """Layer-2 supervisor reporting interface.

    The executive layer calls these methods to inform the
    deliberative planner (L3) of L1 events that require re-planning
    or merely status tracking. Implementations are expected to be
    non-blocking: the executive may call from a watchdog callback
    or a lifecycle transition.
    """

    def on_drone_lost(self, drone_name: str) -> None:
        """A drone has missed enough heartbeats that the watchdog
        marks it unrecoverable. The planner should replan the
        drone's open tasks onto survivors."""
        ...

    def on_task_failed(self, task_id: int, reason: str) -> None:
        """A dispatched task did not complete (timeout, executor
        FAILED, drone RTH due to battery, etc.). The planner should
        decide whether to compensate (re-dispatch) or accept the
        failure."""
        ...

    def on_task_completed(self, task_id: int) -> None:
        """A dispatched task reached SUCCESS at L1. The planner may
        use this to advance the relevant ``VictimSubMission`` saga
        and free the drone for the next dispatch."""
        ...

    def current_mode(self) -> 'SystemMode':
        """The executive's current operating mode: NORMAL /
        DEGRADED / SAFE. The planner consults this to decide
        whether to dispatch new work (NORMAL only) or hold pending
        recovery (DEGRADED/SAFE)."""
        ...


class InMemoryExecutiveCapture:
    """Test fake recording every call. Mirrors ``InMemoryEventCapture``
    in shape so tests construct it the same way."""

    def __init__(self, mode: 'SystemMode | None' = None):
        self._mode = mode
        self.drone_lost: List[str] = []
        self.task_failed: List[Tuple[int, str]] = []
        self.task_completed: List[int] = []

    def on_drone_lost(self, drone_name: str) -> None:
        self.drone_lost.append(drone_name)

    def on_task_failed(self, task_id: int, reason: str) -> None:
        self.task_failed.append((task_id, reason))

    def on_task_completed(self, task_id: int) -> None:
        self.task_completed.append(task_id)

    def current_mode(self) -> 'SystemMode':
        # Lazy import: SystemMode is a string-enum, no rclpy needed.
        if self._mode is not None:
            return self._mode
        from ..domain.system_mode_machine import SystemMode
        return SystemMode.NORMAL

    # Test helpers
    def set_mode(self, mode: 'SystemMode') -> None:
        self._mode = mode
