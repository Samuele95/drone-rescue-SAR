"""Readiness policy: pure Python, no rclpy dependency.

Lifted out of ``readiness_coordinator``
so the per-drone readiness rule (minimum odom messages received +
freshness window) is unit-testable in isolation. The inner per-drone
state struct is renamed ``DroneReadinessState`` to avoid the
collision with ``lib.domain.drone_state.DroneState`` (an Enum of
operating modes).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DroneReadinessState:
    """Per-drone readiness counter. The coordinator mutates this
    on every odom callback; the policy reads it during the timer
    tick."""
    odom_count: int = 0
    last_odom_time: float = 0.0
    ready: bool = False


@dataclass(frozen=True)
class ReadinessPolicy:
    """The readiness threshold policy.

    ``min_odom_count``: a drone must have received this many odom
    messages before it is considered ready.

    ``odom_timeout``: if the most recent odom is older than this
    many seconds, the drone is considered not ready.

    ``min_ready_duration``: every drone must be ready continuously
    for this many seconds before the survey starts. Guards against
    a single transient ready-blink triggering a premature start.
    """
    min_odom_count: int = 10
    odom_timeout: float = 2.0
    min_ready_duration: float = 5.0

    def is_drone_ready(
        self,
        drone: DroneReadinessState,
        current_time: float,
    ) -> bool:
        """True iff this drone has cleared both the message-count
        floor and the freshness window."""
        if drone.odom_count < self.min_odom_count:
            return False
        if drone.last_odom_time > 0:
            odom_age = current_time - drone.last_odom_time
            if odom_age > self.odom_timeout:
                return False
        return True
