"""The operator-UI mission lifecycle as a typed FSM, unit-tested
without Qt."""

import pytest

from drone_rescue_ui_common.mission_lifecycle import (
    IllegalUiTransition, MissionLifecycleState, MissionPhase,
)


def test_default_is_idle():
    s = MissionLifecycleState()
    assert s.phase is MissionPhase.IDLE
    assert not s.is_active()


def test_happy_path_transitions():
    s = MissionLifecycleState()
    s = s.transition(MissionPhase.SPAWNING)
    s = s.transition(MissionPhase.ACTIVATING)
    s = s.transition(MissionPhase.RUNNING, 'Mission in progress.')
    assert s.phase is MissionPhase.RUNNING
    assert s.detail == 'Mission in progress.'
    assert s.is_active()
    s = s.transition(MissionPhase.DONE)
    assert not s.is_active()
    assert s.transition(MissionPhase.IDLE).phase is MissionPhase.IDLE


def test_illegal_transition_raises_by_default():
    s = MissionLifecycleState()                       # IDLE
    with pytest.raises(IllegalUiTransition):
        s.transition(MissionPhase.RUNNING)            # IDLE→RUNNING skips spawn


def test_any_active_phase_can_fail_to_error():
    for phase in (MissionPhase.SPAWNING, MissionPhase.ACTIVATING,
                  MissionPhase.RUNNING):
        s = MissionLifecycleState(phase=phase)
        assert s.transition(MissionPhase.ERROR).phase is MissionPhase.ERROR


def test_raise_on_invalid_false_forces_the_move():
    s = MissionLifecycleState()                       # IDLE
    forced = s.transition(MissionPhase.RUNNING, raise_on_invalid=False)
    assert forced.phase is MissionPhase.RUNNING       # no raise, forced


def test_state_is_frozen():
    s = MissionLifecycleState()
    with pytest.raises((AttributeError, Exception)):
        s.phase = MissionPhase.RUNNING
