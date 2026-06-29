"""Tests for the per-drone IntentionWorkspace.

The workspace's job is reconciliation + path memory. The tests cover:
the empty (no motivations) case, single-motivation pass-through,
multi-motivation summation, inhibition (negative strengths), path
memory boost on positive intentions only, the boost cap, and the
fixed evaluation order (deterministic for the seeded harness).
"""

from __future__ import annotations

import math

from drone_rescue_coordination.lib.domain.intention_workspace import (
    IntentionWorkspace,
)
from drone_rescue_coordination.lib.domain.motivations import (
    INVESTIGATE,
    CoverageMotivation,
    SafetyMotivation,
    VictimMotivation,
)
from drone_rescue_coordination.lib.ports.motivation import (
    Desire,
    MotivationContext,
)


def _ctx(**kw):
    base = dict(
        drone_name='drone1',
        target=(0.0, 0.0),
        target_priority=2,
        distance_to_target=10.0,
    )
    base.update(kw)
    return MotivationContext(**base)


# --------------------------------------------------------------- empty / single

def test_empty_workspace_reconciles_to_no_strengths():
    ws = IntentionWorkspace('drone1', motivations=())
    assert ws.reconcile(_ctx()) == {}
    assert ws.strength_for(INVESTIGATE, _ctx()) == 0.0


def test_single_motivation_passes_through():
    ws = IntentionWorkspace('drone1', motivations=(VictimMotivation(),))
    assert math.isclose(
        ws.strength_for(INVESTIGATE, _ctx(target_priority=4, distance_to_target=8.0)),
        0.5, abs_tol=1e-12,
    )


# --------------------------------------------------------------- combination

def test_inhibition_subtracts_from_pull():
    ws = IntentionWorkspace(
        'drone1',
        motivations=(VictimMotivation(), CoverageMotivation(coverage_pull=0.05)),
    )
    # 2/10 = 0.2 ; coverage inhibits 0.05 -> net 0.15
    assert math.isclose(
        ws.strength_for(INVESTIGATE, _ctx()), 0.15, abs_tol=1e-12,
    )


def test_strong_inhibition_can_flip_to_negative():
    ws = IntentionWorkspace(
        'drone1',
        motivations=(VictimMotivation(), CoverageMotivation(coverage_pull=10.0)),
    )
    assert ws.strength_for(INVESTIGATE, _ctx()) < 0


def test_safety_absent_when_no_affect_injected():
    ws = IntentionWorkspace(
        'drone1', motivations=(VictimMotivation(), SafetyMotivation()),
    )
    # Safety is silent without an AffectMonitor -> equals greedy strength.
    assert math.isclose(ws.strength_for(INVESTIGATE, _ctx()),
                        0.2, abs_tol=1e-12)


# --------------------------------------------------------------- path memory

def test_no_path_memory_default():
    ws = IntentionWorkspace('drone1', motivations=(VictimMotivation(),))
    assert ws.successes(INVESTIGATE) == 0


def test_record_success_increments_counter():
    ws = IntentionWorkspace('drone1', motivations=(VictimMotivation(),))
    ws.record_success(INVESTIGATE)
    ws.record_success(INVESTIGATE)
    assert ws.successes(INVESTIGATE) == 2


def test_path_memory_boost_applies_to_positive_intention_only():
    ws = IntentionWorkspace('drone1', motivations=(VictimMotivation(),))
    base = ws.strength_for(INVESTIGATE, _ctx())   # 0.2
    ws.record_success(INVESTIGATE)
    boosted = ws.strength_for(INVESTIGATE, _ctx())
    # 0.2 * (1 + 0.05) = 0.21
    assert math.isclose(boosted, base * 1.05, abs_tol=1e-12)


def test_path_memory_boost_caps_at_fifty_percent():
    ws = IntentionWorkspace('drone1', motivations=(VictimMotivation(),))
    base = ws.strength_for(INVESTIGATE, _ctx())
    for _ in range(50):
        ws.record_success(INVESTIGATE)
    boosted = ws.strength_for(INVESTIGATE, _ctx())
    # 50 successes -> 0.05 * 50 = 2.5 raw, capped to 0.5 -> max 1.5x base.
    assert math.isclose(boosted, base * 1.5, abs_tol=1e-12)


def test_path_memory_does_not_boost_inhibited_intention():
    ws = IntentionWorkspace(
        'drone1',
        motivations=(VictimMotivation(), CoverageMotivation(coverage_pull=10.0)),
    )
    net_before = ws.strength_for(INVESTIGATE, _ctx())     # negative
    ws.record_success(INVESTIGATE)
    net_after = ws.strength_for(INVESTIGATE, _ctx())
    assert net_before == net_after        # boost gates on positive only


# --------------------------------------------------------------- determinism

def _ConstantMotivation(name: str, strength: float):
    """Local helper: tiny motivation that always emits one fixed desire."""
    class _M:
        def __init__(self):
            self.name = name
        def propose(self, ctx):
            return (Desire(INVESTIGATE, strength, name),)
    return _M()


def test_evaluation_order_is_motivation_registration_order():
    """Re-ordering registration changes the source-attribution order but
    not the sum, which is what determinism requires."""
    a = _ConstantMotivation('a', 0.3)
    b = _ConstantMotivation('b', -0.1)
    ws_ab = IntentionWorkspace('drone1', motivations=(a, b))
    ws_ba = IntentionWorkspace('drone1', motivations=(b, a))
    s_ab = ws_ab.strength_for(INVESTIGATE, _ctx())
    s_ba = ws_ba.strength_for(INVESTIGATE, _ctx())
    assert math.isclose(s_ab, s_ba, abs_tol=1e-12)
