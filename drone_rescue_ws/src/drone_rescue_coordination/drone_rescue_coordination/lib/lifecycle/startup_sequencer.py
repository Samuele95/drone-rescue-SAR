"""StartupSequencer: pure-Python lifecycle phase orchestrator.

Lifts ``LifecycleManager._do_startup`` / ``_do_shutdown`` out of the
rclpy LifecycleNode so the phase ordering is unit-testable against
``InMemoryChangeStateRecorder``. Pairs with ``build_node_list``
(orchestrator.py) and the ``ChangeStateClient`` Protocol
(lib/ports/change_state_client.py).

The sequencer is rclpy-free. The production LifecycleManager wires
``RclpyChangeStateClient`` and a logger Protocol in at construction
time; the Mission Control bench harness can wire its own in-process
adapter.
"""

from __future__ import annotations

from typing import Callable, Optional, Sequence

from lifecycle_msgs.msg import State, Transition

from drone_rescue_coordination.lib.ports.change_state_client import (
    ChangeStateClient,
)


class StartupSequencer:
    """Drive the fleet through configure → activate (startup) and
    deactivate → cleanup (shutdown) in dependency order.

    Startup short-circuits on first failure (cannot bring up a half-
    activated fleet). Shutdown accumulates failures (must keep
    deactivating downstream nodes even if one ack times out, so the
    drones still get the land trigger)."""

    def __init__(
        self,
        change_state_client: ChangeStateClient,
        managed_nodes: Sequence[str],
        drone_names: Sequence[str],
        *,
        logger,
        transition_timeout_s: float = 10.0,
    ) -> None:
        self._client = change_state_client
        self._managed = list(managed_nodes)
        self._drones = list(drone_names)
        self._logger = logger
        self._timeout = float(transition_timeout_s)

    # ------------------------------------------------------------- startup
    def startup(self) -> bool:
        self._logger.info('Starting coordinated startup sequence...')

        for node_name in self._managed:
            if not self._configure(node_name):
                return False

        if not self._activate('pheromone_server'):
            return False

        for drone in self._drones:
            if not self._activate(f'{drone}_controller'):
                return False

        if 'mission_manager' in self._managed:
            if not self._activate('mission_manager'):
                return False

        for drone in self._drones:
            executor_name = f'{drone}_executor'
            if executor_name in self._managed:
                if not self._activate(executor_name):
                    return False

        return True

    # ------------------------------------------------------------- shutdown
    def shutdown(
        self,
        *,
        wait_for_landing: Optional[Callable[[], None]] = None,
    ) -> bool:
        self._logger.info('=' * 50)
        self._logger.info('Starting coordinated shutdown sequence...')
        self._logger.info('=' * 50)

        success = True

        # Phase 1: executors then mission_manager, stop new tasks first.
        self._logger.info(
            'Phase 1: Deactivating drone executors + mission_manager...'
        )
        for drone in reversed(self._drones):
            node_name = f'{drone}_executor'
            if node_name not in self._managed:
                continue
            if not self._deactivate(node_name, indent='  '):
                success = False
        if 'mission_manager' in self._managed:
            if not self._deactivate('mission_manager', indent='  '):
                success = False

        # Phase 2: controllers (triggers landing).
        self._logger.info(
            'Phase 2: Deactivating controllers (landing drones)...'
        )
        for drone in reversed(self._drones):
            node_name = f'{drone}_controller'
            if not self._deactivate(node_name, indent='  '):
                success = False

        # Phase 3: wait for the drones to actually touch down.
        self._logger.info('Phase 3: Waiting for drones to land...')
        if wait_for_landing is not None:
            wait_for_landing()

        # Phase 4: pheromone_server (anything still subscribing to it
        # is already deactivated).
        self._logger.info('Phase 4: Deactivating pheromone_server...')
        if not self._deactivate('pheromone_server'):
            success = False

        # Phase 5: cleanup every node in reverse order.
        self._logger.info('Phase 5: Cleaning up all nodes...')
        for node_name in reversed(self._managed):
            # Skip CLEANUP for a node already (known) UNCONFIGURED: cleanup from
            # there is an unregistered transition that crashes the node (e.g. a
            # stale manager's shutdown reaching a freshly-started node of the
            # same name). INACTIVE/ACTIVE/UNKNOWN still attempt.
            if self._state(node_name) == State.PRIMARY_STATE_UNCONFIGURED:
                self._logger.info(
                    f'  {node_name} already unconfigured — skipping cleanup'
                )
                continue
            self._logger.info(f'  Cleaning up {node_name}...')
            if not self._transition(node_name, Transition.TRANSITION_CLEANUP):
                self._logger.warn(f'  Failed to cleanup {node_name}')
                success = False
            else:
                self._logger.info(f'  {node_name} cleaned up')

        return success

    # ------------------------------------------------------------- helpers
    def _transition(self, node_name: str, transition_id: int) -> bool:
        return self._client.transition(
            node_name, transition_id, timeout_s=self._timeout,
        )

    def _state(self, node_name: str) -> int:
        """Current primary lifecycle state id, or UNKNOWN(0) if the query
        fails, in which case callers fall back to attempting the transition
        (preserving the original, non-idempotent behaviour)."""
        try:
            return int(self._client.get_state(node_name, timeout_s=self._timeout))
        except Exception:
            return State.PRIMARY_STATE_UNKNOWN

    def _configure(self, node_name: str) -> bool:
        """Idempotent configure: skip when the node is already configured or
        active. Without this, a second startup pass (auto-start + a startup
        service call, or a re-trigger) sends CONFIGURE to an already-active
        node, which is an unregistered transition and KILLS the node."""
        st = self._state(node_name)
        if st in (State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE):
            self._logger.info(f'{node_name} already configured (state {st}) — skipping')
            return True
        self._logger.info(f'Configuring {node_name}...')
        if not self._transition(node_name, Transition.TRANSITION_CONFIGURE):
            self._logger.error(f'Failed to configure {node_name}')
            return False
        self._logger.info(f'{node_name} configured')
        return True

    def _activate(self, node_name: str) -> bool:
        if self._state(node_name) == State.PRIMARY_STATE_ACTIVE:
            self._logger.info(f'{node_name} already active — skipping')
            return True
        self._logger.info(f'Activating {node_name}...')
        if not self._transition(node_name, Transition.TRANSITION_ACTIVATE):
            self._logger.error(f'Failed to activate {node_name}')
            return False
        self._logger.info(f'{node_name} activated')
        return True

    def _deactivate(self, node_name: str, *, indent: str = '') -> bool:
        # Cross-talk-safe: skip when the node is in a KNOWN non-active state
        # (UNCONFIGURED/INACTIVE): DEACTIVATE from there is an unregistered
        # transition that KILLS the node. This guards a freshly-started fleet
        # against a stale lifecycle manager from a previous run (same
        # ROS_DOMAIN_ID, same node names) issuing shutdown transitions into it.
        # UNKNOWN (state query failed) still attempts, so a genuine shutdown
        # with a slow query keeps issuing the controllers' land trigger.
        st = self._state(node_name)
        if st in (State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE):
            self._logger.info(
                f'{indent}{node_name} not active (state {st}) — skipping deactivate'
            )
            return True
        self._logger.info(f'{indent}Deactivating {node_name}...')
        if not self._transition(node_name, Transition.TRANSITION_DEACTIVATE):
            self._logger.warn(f'{indent}Failed to deactivate {node_name}')
            return False
        self._logger.info(f'{indent}{node_name} deactivated')
        return True
