"""RclpyChangeStateClient: production ChangeStateClient adapter.

Wraps the LifecycleManager._change_state body
(poll-wait-call-poll-done rclpy service-client orchestration) behind
the typed ``ChangeStateClient`` Protocol. The legacy LifecycleManager
delegates to an instance; a future ``StartupSequencer`` consumes
the port without binding to rclpy or to the LifecycleNode shell.

Construction takes the owning ``Node`` (for ``create_client``) and
optional service-client cache so successive transitions on the same
target node reuse a single client. The cache is owned by the caller
(LifecycleManager) so its existing back-compat surface
(``self._change_state_clients``) keeps working unchanged.
"""

from __future__ import annotations

import time
from typing import Dict, Optional

from lifecycle_msgs.srv import ChangeState, GetState


class RclpyChangeStateClient:
    """Production adapter: same behaviour as the legacy
    ``LifecycleManager._change_state``."""

    def __init__(self, node, *, client_cache: Optional[Dict] = None,
                 callback_group=None, logger=None):
        self._node = node
        self._client_cache = client_cache if client_cache is not None else {}
        self._callback_group = callback_group
        self._logger = logger or node.get_logger()

    def _get_change_state_client(self, node_name: str):
        if node_name not in self._client_cache:
            self._client_cache[node_name] = self._node.create_client(
                ChangeState,
                f'/{node_name}/change_state',
                callback_group=self._callback_group,
            )
        return self._client_cache[node_name]

    def transition(
        self,
        node_name: str,
        transition_id: int,
        timeout_s: float = 10.0,
    ) -> bool:
        """Body lifted from ``lifecycle_manager._change_state``."""
        client = self._get_change_state_client(node_name)
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.5):
                break
        else:
            self._logger.error(
                f'Service not available within {timeout_s}s: '
                f'/{node_name}/change_state'
            )
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id

        future = client.call_async(request)
        call_deadline = max(time.monotonic() + 2.0, deadline)
        while time.monotonic() < call_deadline and not future.done():
            time.sleep(0.05)

        if not future.done() or future.result() is None:
            self._logger.error(f'Transition call failed for {node_name}')
            return False

        return future.result().success

    def get_state(
        self,
        node_name: str,
        timeout_s: float = 5.0,
    ) -> Optional[int]:
        """Query the lifecycle state of ``node_name``.

        Returns ``None`` on timeout. The Port declares this method for
        completeness; the legacy LifecycleManager doesn't use it today.
        """
        cache_key = f'__get_state__{node_name}'
        if cache_key not in self._client_cache:
            self._client_cache[cache_key] = self._node.create_client(
                GetState,
                f'/{node_name}/get_state',
                callback_group=self._callback_group,
            )
        client = self._client_cache[cache_key]
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        future = client.call_async(GetState.Request())
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.05)
        if not future.done() or future.result() is None:
            return None
        return int(future.result().current_state.id)
