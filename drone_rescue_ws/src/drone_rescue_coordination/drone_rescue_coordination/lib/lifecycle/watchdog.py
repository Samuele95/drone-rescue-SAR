"""Node heartbeat watchdog helpers: pure Python, no rclpy.

The heartbeat-tracking and node-classification pieces of the
lifecycle_manager watchdog subsystem. Recovery handlers stay on the
LifecycleManager class (they reach into node-owned state like
`mode_manager` and ROS publishers); the typed classifier and the
heartbeat bookkeeping are the parts that lifted cleanly.

`NodeKind` was inline on `LifecycleManager._NodeKind` when typed
recovery routing was introduced; lifting it to a module-level enum
makes it referenceable from tests and future RecoveryDispatcher
extensions without importing the LifecycleNode shell.
"""

from __future__ import annotations

from enum import IntEnum
from typing import Dict, Optional


class NodeKind(IntEnum):
    """Classification of managed nodes for recovery routing.

    Promoted from a private inner class on `LifecycleManager` to the
    lifecycle subdomain so tests can reference it without instantiating
    a LifecycleNode.
    """
    PHEROMONE = 1
    CONTROLLER = 2
    EXECUTOR_OR_SURVEYOR = 3
    UNKNOWN = 99


def classify_node(node_name: str) -> NodeKind:
    """Classify a node by name for recovery routing.

    Substring-suffix-aware: a node literally named
    `pheromone_controller` resolves to `PHEROMONE` (the
    fleet-wide-critical kind is checked first), not `CONTROLLER`.
    Names without a recognisable suffix resolve to `UNKNOWN`.
    """
    nm = node_name.lower()
    if 'pheromone' in nm:
        return NodeKind.PHEROMONE
    if 'controller' in nm:
        return NodeKind.CONTROLLER
    if 'executor' in nm or 'surveyor' in nm:
        return NodeKind.EXECUTOR_OR_SURVEYOR
    return NodeKind.UNKNOWN


class HeartbeatTracker:
    """Tracks last-seen timestamps per monitored node, flags
    unresponsive nodes after a timeout.

    Pure Python: accepts a `now()` callable so the tracker is
    testable with a FakeClock. The LifecycleManager passes
    `self.get_clock().now().nanoseconds / 1e9` or its `self._clock
    .now_sec()` as the clock source.
    """

    def __init__(
        self,
        monitored: list,
        timeout_s: float,
        clock_fn=None,
    ):
        self._monitored = list(monitored)
        self._timeout_s = float(timeout_s)
        self._clock_fn = clock_fn or (lambda: 0.0)
        self._last_seen: Dict[str, float] = {}
        self._unresponsive: set = set()

    def set_monitored(self, names) -> None:
        """Replace the monitored-node list and seed every name's
        last-seen timestamp with the current clock reading.

        The LifecycleManager populates its monitored list after
        lifecycle activation; this method lets it hand the names over
        once start_watchdog_monitoring runs.
        """
        self._monitored = list(names)
        now = self._clock_fn()
        for name in self._monitored:
            self._last_seen[name] = now

    def record_heartbeat(self, node_id: str) -> Optional[str]:
        """Match `node_id` to a monitored prefix and update its
        timestamp. Returns the matched node name if a recovery flag
        was cleared as a side effect; None otherwise."""
        for monitored in self._monitored:
            if monitored in node_id:
                self._last_seen[monitored] = self._clock_fn()
                if monitored in self._unresponsive:
                    self._unresponsive.discard(monitored)
                    return monitored
                return None
        return None

    def find_newly_unresponsive(self) -> list:
        """Returns a list of (node_name, age_s) for nodes that have
        crossed the timeout threshold this tick and weren't already
        flagged."""
        now = self._clock_fn()
        out = []
        for node_name, last_seen in self._last_seen.items():
            age = now - last_seen
            if age > self._timeout_s and node_name not in self._unresponsive:
                self._unresponsive.add(node_name)
                out.append((node_name, age))
        return out

    @property
    def unresponsive(self) -> set:
        return set(self._unresponsive)

    @property
    def heartbeats(self) -> Dict[str, float]:
        return dict(self._last_seen)

    def all_seen(self) -> bool:
        """True when every monitored node has reported at least once."""
        return all(n in self._last_seen for n in self._monitored)
