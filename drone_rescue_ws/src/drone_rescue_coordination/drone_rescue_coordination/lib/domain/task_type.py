"""Task type IntEnum: pure-Python mirror of TaskAssignment.task_type.

Lifted from lib/auction.py to lib/domain so the domain layer is the
single source of truth. lib/auction.py re-exports the symbol for
backward compatibility. The mission_manager import-time assertion
that the ints match the ROS TaskAssignment constants keeps the two
in sync; divergence raises at package import, not at runtime.
"""

from __future__ import annotations

from enum import IntEnum


class TaskType(IntEnum):
    """Integer-valued task identities.

    Mirrors the ``TaskAssignment.task_type`` ROS message enum. Kept
    locally so this module retains its no-rclpy invariant. ``IntEnum``
    means values compare equal to their underlying ``int``
    (``TaskType.INVESTIGATE == 1`` is True), so any caller already
    holding the int from the ROS message works unchanged.
    """
    SCAN = 0
    INVESTIGATE = 1
    CONFIRM = 2
    RTH = 3
    LAND = 4
    IDLE = 5

    @property
    def label(self) -> str:
        """Canonical display name. Single source of truth for the
        task-type to string mapping previously hand-rolled in
        mission_manager._task_type_name, dashboard's _TASK_LABEL, and
        analytics' _TASK_NAMES. The variant names (full 'INVESTIGATE',
        not abbreviated 'INVEST') are the canonical vocabulary."""
        return self.name


def task_type_label(t: int) -> str:
    """Module-level helper for callers holding a raw int (e.g. from a
    ROS message field). Returns the canonical label for known task
    types and a `?(n)` fallback for unknown ints, the same fallback
    contract the legacy hand-rolled helpers offered."""
    try:
        return TaskType(t).label
    except ValueError:
        return f'?({t})'
