"""RecoveryDispatcher: driven port for the lifecycle-manager
watchdog's recovery side-effects.

An earlier change split the NodeKind classification but left the
three recovery handlers attached to the LifecycleNode because they
reach into mode_manager + per-drone publishers. The dispatcher
Protocol decouples the policy decision ("on pheromone loss, trigger
SAFE mode") from the I/O side effect ("publish on
/<drone>/return_home"). The policy lives in
``lib.lifecycle.recovery_policy``; the adapter in
``lib.ros_adapter.recovery_dispatcher``.
"""

from __future__ import annotations

from typing import Protocol

from drone_rescue_coordination.lib.domain.system_mode_machine import SystemMode


# 3T boundary annotation: driven output port for L2 executive
# recovery handlers. The RecoveryPolicy emits side-effects (SAFE-mode
# trigger, per-drone RTH publishes) through this Protocol. Output
# side of L2.
LAYER_BOUNDARY = 'L2-output'


class RecoveryDispatcher(Protocol):
    """Three operations the watchdog recovery handlers need.

    Concrete implementations:
    - ``lib.ros_adapter.recovery_dispatcher.RosRecoveryDispatcher``:
      forwards each call into the running ``LifecycleManager``.
    - ``InMemoryRecoveryRecorder`` (test helper): records the call
      sequence so unit tests can assert per-NodeKind dispatch order.
    """

    def trigger_safe_mode(self, reason: str) -> None:
        ...

    def command_drone_land(self, drone_name: str) -> None:
        ...

    def transition_to(self, mode: SystemMode, reason: str) -> None:
        ...


class InMemoryRecoveryRecorder:
    """Test double recording the dispatcher call sequence."""

    def __init__(self) -> None:
        self.calls: list = []

    def trigger_safe_mode(self, reason: str) -> None:
        self.calls.append(('trigger_safe_mode', reason))

    def command_drone_land(self, drone_name: str) -> None:
        self.calls.append(('command_drone_land', drone_name))

    def transition_to(self, mode: SystemMode, reason: str) -> None:
        self.calls.append(('transition_to', mode, reason))
