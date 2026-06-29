"""RecoveryPolicy: pure-Python watchdog recovery decisions.

The three handlers (`_recovery_pheromone / _recovery_controller /
_recovery_executor_or_surveyor`) live in this module. Their decision
logic is pure-Python; the I/O effects go through a constructor-injected
``RecoveryDispatcher`` Protocol. The legacy ``LifecycleManager``
becomes the thin adapter that constructs the policy and routes the
watchdog's NodeKind classifier into the right ``handle_*`` method.

Sibling to ``lib.lifecycle.system_mode_machine`` (typed FSM): same
shape, pure data-driven, unit-testable, no rclpy import.
"""

from __future__ import annotations

from typing import Callable, Optional, Sequence

from drone_rescue_coordination.lib.domain.system_mode_machine import SystemMode
from drone_rescue_coordination.lib.ports.recovery_dispatcher import (
    RecoveryDispatcher,
)


class RecoveryPolicy:
    """Per-NodeKind recovery routing.

    Hardware_id format contract: nodes are identified by the
    diagnostic ``hardware_id`` string, which the project's launch
    files produce as ``<drone_name>-<role>`` (e.g.
    ``drone1-controller``, ``drone2-executor``). The legacy
    underscore separator ``drone1_controller`` is also accepted as
    a fallback. ``handle_controller`` extracts ``drone_id`` via the
    split rule documented inline below.

    Dispatcher: the ROS-facing side effects (publish + mode transition).
    ``unresponsive_provider``: returns the current list of unresponsive
        node names, used by ``handle_controller`` to count controller
        outages.
    ``mode_provider``: returns the current ``SystemMode``, used by
        ``handle_executor_or_surveyor`` to gate the NORMAL->DEGRADED
        transition (we only escalate from NORMAL; already SAFE/DEGRADED
        stays put).
    ``known_drone_names``: optional whitelist. When supplied,
        ``handle_controller`` validates the extracted
        ``drone_id`` against this set and emits a logger warning when
        the format drifts (e.g. ROS namespace prefix). When None
        (current default), the extraction is best-effort.
    ``logger_fn``: optional callable used to surface the drift warning.
        Signature: ``Callable[[str], None]``. Default no-op (so unit
        tests don't need a real logger).
    """

    def __init__(
        self,
        dispatcher: RecoveryDispatcher,
        unresponsive_provider: Callable[[], Sequence[str]],
        mode_provider: Callable[[], SystemMode],
        *,
        known_drone_names: Optional[Sequence[str]] = None,
        logger_fn: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._dispatcher = dispatcher
        self._unresponsive = unresponsive_provider
        self._mode = mode_provider
        self._known_drone_names = (
            frozenset(known_drone_names) if known_drone_names else None
        )
        self._log_warn = logger_fn or (lambda msg: None)

    # ----------------------------------------------------------- per-NodeKind
    def handle_pheromone(self, node_name: str) -> None:
        """Pheromone server is critical: full SAFE-mode recovery."""
        self._dispatcher.trigger_safe_mode(
            f'Pheromone server {node_name} unresponsive'
        )

    def handle_controller(self, node_name: str) -> None:
        """One drone controller out: command that drone to land.
        Two or more out: escalate to DEGRADED.

        Validates the extracted drone_id
        against ``known_drone_names`` when supplied. Emits a logger
        warning + skips the land command when the format drifts
        (avoids silently calling ``command_drone_land('fleet/drone1')``
        when ROS namespace remapping produces a hardware_id like
        ``fleet/drone1-controller``).
        """
        drone_id = (
            node_name.split('-')[0] if '-' in node_name
            else node_name.split('_')[0]
        )
        if (self._known_drone_names is not None
                and drone_id not in self._known_drone_names):
            self._log_warn(
                f'RecoveryPolicy: hardware_id {node_name!r} produced '
                f'drone_id {drone_id!r} which is not in the known fleet '
                f'{sorted(self._known_drone_names)} — skipping land command'
            )
            return
        self._dispatcher.command_drone_land(drone_id)
        controller_unresponsive = [
            n for n in self._unresponsive() if 'controller' in n.lower()
        ]
        if len(controller_unresponsive) >= 2:
            self._dispatcher.transition_to(
                SystemMode.DEGRADED,
                f'{len(controller_unresponsive)} drone controllers unresponsive',
            )

    def handle_executor_or_surveyor(self, node_name: str) -> None:
        """Executor or surveyor out: escalate NORMAL to DEGRADED.
        SAFE/DEGRADED stay put (no re-entry)."""
        if self._mode() == SystemMode.NORMAL:
            self._dispatcher.transition_to(
                SystemMode.DEGRADED,
                f'Executor/surveyor {node_name} unresponsive',
            )

    def handle_unknown(self, node_name: str) -> None:
        """Unknown NodeKind: no side effect. Lifecycle manager keeps
        a separate log line for this branch."""
        pass
