"""Tests for the SystemModeMachine typed FSM.

Exhaustive over (SystemMode, ModeTrigger): every cell of the
table covered with the expected next-mode or the expected
IllegalTransition raise.
"""

from __future__ import annotations

import pytest

from drone_rescue_coordination.lib.domain.state_machines import (
    IllegalTransition,
)
from drone_rescue_coordination.lib.domain.system_mode_machine import (
    ModeTrigger,
    SystemMode,
    SystemModeMachine,
)


# legal table

def test_normal_to_degraded_on_persistent_warn():
    assert SystemModeMachine.transition(
        SystemMode.NORMAL, ModeTrigger.PERSISTENT_WARN,
    ) == SystemMode.DEGRADED


def test_normal_to_degraded_on_multi_drone_warn():
    assert SystemModeMachine.transition(
        SystemMode.NORMAL, ModeTrigger.MULTI_DRONE_WARN,
    ) == SystemMode.DEGRADED


def test_normal_to_safe_on_error():
    assert SystemModeMachine.transition(
        SystemMode.NORMAL, ModeTrigger.ERROR,
    ) == SystemMode.SAFE


def test_degraded_to_safe_on_error():
    assert SystemModeMachine.transition(
        SystemMode.DEGRADED, ModeTrigger.ERROR,
    ) == SystemMode.SAFE


def test_degraded_to_normal_on_all_clear():
    assert SystemModeMachine.transition(
        SystemMode.DEGRADED, ModeTrigger.ALL_CLEAR,
    ) == SystemMode.NORMAL


def test_safe_idempotent_on_error():
    """SAFE+ERROR self-loop so repeated triggers don't raise."""
    assert SystemModeMachine.transition(
        SystemMode.SAFE, ModeTrigger.ERROR,
    ) == SystemMode.SAFE


# illegal table

def test_normal_all_clear_is_a_noop_not_a_transition():
    """Per the legacy implementation, ALL_CLEAR in NORMAL is silent:
    not represented in the table; callers must check
    `can_transition` first or accept the raise."""
    assert not SystemModeMachine.can_transition(
        SystemMode.NORMAL, ModeTrigger.ALL_CLEAR,
    )
    with pytest.raises(IllegalTransition):
        SystemModeMachine.transition(
            SystemMode.NORMAL, ModeTrigger.ALL_CLEAR,
        )


def test_safe_does_not_auto_recover():
    """SAFE → NORMAL requires operator confirm; the FSM has no
    automatic ALL_CLEAR escape from SAFE."""
    assert not SystemModeMachine.can_transition(
        SystemMode.SAFE, ModeTrigger.ALL_CLEAR,
    )
    with pytest.raises(IllegalTransition):
        SystemModeMachine.transition(
            SystemMode.SAFE, ModeTrigger.ALL_CLEAR,
        )


def test_safe_does_not_demote_on_warnings():
    for t in (ModeTrigger.PERSISTENT_WARN, ModeTrigger.MULTI_DRONE_WARN):
        assert not SystemModeMachine.can_transition(SystemMode.SAFE, t)


def test_degraded_no_redundant_warn_transitions():
    """DEGRADED+WARN is already-DEGRADED: the FSM has no row to
    re-fire the same transition, so the caller's per-tick check
    silently no-ops."""
    for t in (ModeTrigger.PERSISTENT_WARN, ModeTrigger.MULTI_DRONE_WARN):
        assert not SystemModeMachine.can_transition(SystemMode.DEGRADED, t)


# enum sanity

def test_system_mode_values_match_legacy_int_layout():
    """The legacy `SystemModeEnum` uses NORMAL=0/DEGRADED=1/SAFE=2.
    The two enums must stay int-compatible during the migration."""
    assert SystemMode.NORMAL.value == 0
    assert SystemMode.DEGRADED.value == 1
    assert SystemMode.SAFE.value == 2
