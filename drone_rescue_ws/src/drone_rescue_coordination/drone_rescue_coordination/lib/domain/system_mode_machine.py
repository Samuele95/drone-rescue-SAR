"""System-mode typed state machine: NORMAL / DEGRADED / SAFE.

Pattern reuse from `MissionStateMachine` and `VictimStateMachine`.
Replaces the inline `if/elif` chain inside
`lifecycle_manager.ModeManager.update_from_diagnostics`. The
transition rules become enumerable (one table) and the legality of
each (mode, trigger) pair is testable in isolation.

The caller's job is to classify a diagnostic array into one of the
`ModeTrigger` values; the machine's job is to look up the next mode
or raise `IllegalTransition`. The split decouples diagnostic-shape
analysis (mutable, ROS-tied) from mode-graph policy (pure, typed).
"""

from __future__ import annotations

from enum import IntEnum

from .state_machines import IllegalTransition


class SystemMode(IntEnum):
    """Operational mode pillars for the coordination layer.

    Values mirror `lifecycle_manager.SystemModeEnum` integers so the
    two enums interoperate during the migration window.
    """
    NORMAL = 0
    DEGRADED = 1
    SAFE = 2


class ModeTrigger(IntEnum):
    """Closed enum of triggers the mode FSM accepts.

    The diagnostic classifier in `lifecycle_manager.ModeManager`
    inspects a `DiagnosticArray` and returns exactly one trigger per
    update tick. Adding a new trigger requires a row in the table.
    """
    PERSISTENT_WARN = 1     # WARN-level diagnostics sustained past threshold
    MULTI_DRONE_WARN = 2    # >=2 distinct drones with WARN
    ERROR = 3               # ERROR-level diagnostic anywhere
    ALL_CLEAR = 4           # no warnings or errors


# (current_mode, trigger) -> next_mode.
# SAFE is sticky on ERROR (idempotent self-loop); the only way out of
# SAFE is operator intervention via the cancel_recovery service.
_MODE_TABLE = {
    (SystemMode.NORMAL, ModeTrigger.PERSISTENT_WARN): SystemMode.DEGRADED,
    (SystemMode.NORMAL, ModeTrigger.MULTI_DRONE_WARN): SystemMode.DEGRADED,
    (SystemMode.NORMAL, ModeTrigger.ERROR): SystemMode.SAFE,
    (SystemMode.DEGRADED, ModeTrigger.ERROR): SystemMode.SAFE,
    (SystemMode.DEGRADED, ModeTrigger.ALL_CLEAR): SystemMode.NORMAL,
    # Idempotent ERROR-in-SAFE so operator-confirm logic can keep
    # firing the trigger without raising.
    (SystemMode.SAFE, ModeTrigger.ERROR): SystemMode.SAFE,
}


class SystemModeMachine:
    """Pure-function transition table for the system mode."""

    @staticmethod
    def transition(mode: SystemMode, trigger: ModeTrigger) -> SystemMode:
        """Return the next mode for (mode, trigger). Raises
        `IllegalTransition` if no rule covers the pair."""
        nxt = _MODE_TABLE.get((mode, trigger))
        if nxt is None:
            raise IllegalTransition(
                f'no system-mode transition from {mode.name} '
                f'on {trigger.name}; check _MODE_TABLE in '
                f'lib/domain/system_mode_machine.py'
            )
        return nxt

    @staticmethod
    def can_transition(mode: SystemMode, trigger: ModeTrigger) -> bool:
        return (mode, trigger) in _MODE_TABLE
