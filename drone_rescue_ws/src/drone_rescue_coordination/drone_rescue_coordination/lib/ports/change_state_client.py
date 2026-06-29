"""ChangeStateClient: driven port for ROS 2 lifecycle transitions.

Lifts ``lifecycle_manager._change_state`` (the
poll-wait-call-poll-done rclpy service-client orchestration) behind
a Protocol so the Mission Control bench harness can drive the same
lifecycle path via its own in-process ``RosControl.call_service``
adapter. Pairs with the (future) ``StartupSequencer`` that consumes
this Protocol + ``build_node_list``.

The Protocol stays pure-Python; concrete adapters live in
``lib/ros_adapter/change_state_client.py``.
"""

from __future__ import annotations

from typing import Optional, Protocol


# 3T boundary annotation: driving port consumed by the L2 Executive
# Layer (StartupSequencer, LifecycleManager) to actuate ROS 2
# lifecycle transitions on L1/L2 nodes. Lives at the executive
# layer's bottom edge.
LAYER_BOUNDARY = 'L2-driving'


class ChangeStateClient(Protocol):
    """Request a /<node>/change_state transition and wait for ack.

    Implementations:
    - ``RclpyChangeStateClient`` (production: wraps the legacy
      LifecycleManager._change_state body).
    - ``InMemoryChangeStateRecorder`` (test fake: records every
      ``transition(node_name, transition_id)`` call so unit tests
      can assert the sequencer's ordering without rclpy).
    """

    def transition(
        self,
        node_name: str,
        transition_id: int,
        timeout_s: float = 10.0,
    ) -> bool:
        """Return True iff the node acked the transition within
        ``timeout_s``."""
        ...

    def get_state(
        self,
        node_name: str,
        timeout_s: float = 5.0,
    ) -> Optional[int]:
        """Return the node's current lifecycle state id, or None on
        timeout."""
        ...


class InMemoryChangeStateRecorder(ChangeStateClient):
    """Test double recording the call sequence; ``transition`` always
    returns True unless ``fail_for`` matches the (node_name,
    transition_id) tuple."""

    def __init__(self, fail_for=None):
        self.calls = []
        self._fail_for = set(fail_for or ())

    def transition(self, node_name, transition_id, timeout_s=10.0):
        self.calls.append(('transition', node_name, int(transition_id)))
        return (node_name, int(transition_id)) not in self._fail_for

    def get_state(self, node_name, timeout_s=5.0):
        self.calls.append(('get_state', node_name))
        return 1   # PRIMARY_STATE_UNCONFIGURED
