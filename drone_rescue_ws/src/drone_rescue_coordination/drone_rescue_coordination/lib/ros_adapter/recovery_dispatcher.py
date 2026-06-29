"""RosRecoveryDispatcher: adapter that wires the
``RecoveryDispatcher`` Protocol to a live ``LifecycleManager``.

Keeps the policy (``lib.lifecycle.recovery_policy``) free of rclpy
and ROS-message imports by forwarding each of the three dispatcher
operations into the LifecycleNode's existing methods. The adapter
holds a reference to its owner LifecycleNode and translates each
call into the corresponding `_trigger_safe_mode` /
`_command_drone_land` / mode-transition + publish sequence the
legacy handlers performed inline.
"""

from __future__ import annotations

from drone_rescue_coordination.lib.domain.system_mode_machine import SystemMode
from drone_rescue_coordination.lib.ports.recovery_dispatcher import (
    RecoveryDispatcher,
)


class RosRecoveryDispatcher(RecoveryDispatcher):
    """Concrete dispatcher bound to a LifecycleManager.

    The owner reference is duck-typed: anything exposing
    ``_trigger_safe_mode(reason)``, ``_command_drone_land(drone)``,
    ``mode_manager.transition_to(mode_enum, reason)``, and
    ``_publish_system_mode()`` works. The owner today is the
    LifecycleManager LifecycleNode itself.
    """

    def __init__(self, owner) -> None:
        self._owner = owner

    def trigger_safe_mode(self, reason: str) -> None:
        self._owner._trigger_safe_mode(reason)

    def command_drone_land(self, drone_name: str) -> None:
        self._owner._command_drone_land(drone_name)

    def transition_to(self, mode: SystemMode, reason: str) -> None:
        # The owner's ``mode_manager`` works in its own SystemModeEnum
        # (the legacy LifecycleNode-side enum, value-aligned with the
        # typed ``SystemMode`` lib enum via .value). Use the legacy enum
        # so the mode_manager + system_mode topic emit matches existing
        # operator-facing values.
        legacy_enum = type(self._owner.mode_manager.current_mode)
        self._owner.mode_manager.transition_to(legacy_enum(mode.value), reason)
        self._owner._publish_system_mode()
