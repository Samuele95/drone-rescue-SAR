"""Clock driven port.

A domain-level abstraction over the wall clock. The aggregate and
saga consume `Clock.now_sec()` instead of calling
`node.get_clock().now().nanoseconds / 1e9`, so they run in unit
tests with a `FakeClock(t=42.0)` and stay rclpy-free.

Production adapter: `lib/ros_adapter/ros_clock.RosClock`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


# 3T boundary annotation: wall-clock abstraction consumed by L1
# (Surveyor, DroneExecutor), L2 (LifecycleManager,
# ReadinessCoordinator), and L3 (Mission aggregate, AuctionEngine).
# Pure cross-cutting infrastructure.
LAYER_BOUNDARY = 'cross-cutting'


class Clock(Protocol):
    """Read-only wall clock."""

    def now_sec(self) -> float: ...


@dataclass
class FakeClock:
    """Deterministic test double. ``advance(dt)`` to move time forward."""
    t: float = 0.0

    def now_sec(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt
