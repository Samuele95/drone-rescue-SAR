"""Regression tests for the stage-transition event channel.

Why: mission_manager mutates ``self._stage`` in four places
(on_configure, on_activate, _sync_to_mission, _begin_scan). Before
this change, those mutations were invisible to mission_recorder:
the JSONL recorded individual mission events but not the
deliberative-layer stage timeline. A run whose recorded events go
straight from CANDIDATE_DETECTED → CONFIRM without ever passing
through ``SCANNING`` was indistinguishable in the recording from a
healthy run. Now every stage change emits a ``STAGE_TRANSITION``
event with ``detail = "<old>.name → <new>.name"`` so post-run
analysis can flag "deliberative layer never engaged" as a single
grep.

These tests pin (a) every distinct transition emits exactly one
event, (b) idempotent re-assignment is a no-op (the mirror-from-
Mission path in _sync_to_mission calls _set_stage every tick),
(c) the event detail string is the documented "from → to" format,
(d) pre-activation transitions don't emit (the event port is only
wired in on_configure).

Pure pytest; no rclpy.init(); we exercise the unbound method.
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import List, Tuple

from drone_rescue_coordination.lib.domain.state_machines import (
    MissionStage,
)
from drone_rescue_coordination.mission_manager import MissionManager


# ----------------------------------------------------------- fixtures

def _node(initial_stage: MissionStage, *, is_active: bool = True):
    """Minimal duck-typed surface for ``_set_stage``: an ``_emit_event``
    capture that records (event_type, kwargs), plus a truthy
    ``_event_port`` so the helper's port-existence guard passes."""
    captured: List[Tuple[str, dict]] = []

    def emit(event_type: str, **kw) -> None:
        captured.append((event_type, kw))

    ns = SimpleNamespace(
        _stage=initial_stage,
        _is_active=is_active,
        _event_port=object(),   # any non-None sentinel; only its
                                # presence is checked, not its methods
        _emit_event=emit,
    )
    return ns, captured


# --------------------------------------------------------- happy path

def test_transition_emits_stage_transition_event():
    """The canonical ARMING → DEPLOYING transition (on_activate)
    must emit exactly one STAGE_TRANSITION event."""
    ns, captured = _node(MissionStage.ARMING)
    MissionManager._set_stage(ns, MissionStage.DEPLOYING)
    assert ns._stage == MissionStage.DEPLOYING
    assert len(captured) == 1
    event_type, kwargs = captured[0]
    assert event_type == 'STAGE_TRANSITION'
    assert kwargs['detail'] == 'ARMING → DEPLOYING'


def test_each_transition_in_the_canonical_flow_emits():
    """Walk the full INIT → ARMING → DEPLOYING → SCANNING →
    INVESTIGATING → COMPLETE timeline; assert one event per step."""
    ns, captured = _node(MissionStage.INIT)
    for stage in (
        MissionStage.ARMING,
        MissionStage.DEPLOYING,
        MissionStage.SCANNING,
        MissionStage.INVESTIGATING,
        MissionStage.COMPLETE,
    ):
        MissionManager._set_stage(ns, stage)
    # 5 transitions, 5 events.
    assert len(captured) == 5
    details = [kw['detail'] for (_t, kw) in captured]
    assert details == [
        'INIT → ARMING',
        'ARMING → DEPLOYING',
        'DEPLOYING → SCANNING',
        'SCANNING → INVESTIGATING',
        'INVESTIGATING → COMPLETE',
    ]


def test_scanning_to_aborted_emits():
    """An ABORT mid-mission is also a stage change and must show up
    in the timeline."""
    ns, captured = _node(MissionStage.SCANNING)
    MissionManager._set_stage(ns, MissionStage.ABORTED)
    assert len(captured) == 1
    assert captured[0][1]['detail'] == 'SCANNING → ABORTED'


# ----------------------------------------------------------- idempotence

def test_reassigning_same_stage_is_silent():
    """``_sync_to_mission`` runs every tick and re-assigns
    ``self._stage`` to ``self._mission.stage``. When nothing has
    changed, the helper must not emit an event, otherwise the
    event log fills with same-stage spam at the tick rate."""
    ns, captured = _node(MissionStage.SCANNING)
    for _ in range(10):
        MissionManager._set_stage(ns, MissionStage.SCANNING)
    assert captured == []
    assert ns._stage == MissionStage.SCANNING


def test_change_then_reassign_only_emits_once():
    """A → B emits once; B → B is silent; B → C emits again."""
    ns, captured = _node(MissionStage.DEPLOYING)
    MissionManager._set_stage(ns, MissionStage.SCANNING)
    MissionManager._set_stage(ns, MissionStage.SCANNING)   # noop
    MissionManager._set_stage(ns, MissionStage.SCANNING)   # noop
    MissionManager._set_stage(ns, MissionStage.INVESTIGATING)
    assert [kw['detail'] for (_t, kw) in captured] == [
        'DEPLOYING → SCANNING',
        'SCANNING → INVESTIGATING',
    ]


# -------------------------------------------------- pre-activation gate

def test_pre_activation_transitions_do_not_emit():
    """The event_port is only wired in ``on_configure``; the
    pre-existing on_configure call site uses ``_set_stage`` to move
    INIT → ARMING, but at that point ``_is_active`` is still False.
    The helper must defer emission until activation so test setups
    that bypass lifecycle don't crash on a missing port."""
    ns, captured = _node(MissionStage.INIT, is_active=False)
    MissionManager._set_stage(ns, MissionStage.ARMING)
    # Stage mutated for real ...
    assert ns._stage == MissionStage.ARMING
    # ... but no event emitted (would have hit a None port otherwise).
    assert captured == []


def test_missing_event_port_attribute_is_silent_not_crashing():
    """If an integration-test setup constructs MissionManager via
    ``object.__new__`` and doesn't set ``_event_port`` at all, the
    helper must NOT crash on AttributeError; the stage mutation
    is the contract, observability is secondary."""
    captured: List[Tuple[str, dict]] = []
    ns = SimpleNamespace(
        _stage=MissionStage.ARMING,
        _is_active=True,            # active ...
        _emit_event=lambda *a, **kw: captured.append((a, kw)),
        # ... but _event_port attribute is missing entirely
    )
    # Must not raise AttributeError.
    MissionManager._set_stage(ns, MissionStage.DEPLOYING)
    assert ns._stage == MissionStage.DEPLOYING
    assert captured == []
