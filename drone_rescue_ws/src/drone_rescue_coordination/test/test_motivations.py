"""Tests for the three concrete motivations.

Each motivation is a pure function of its ``MotivationContext``, so
the tests are tiny constructions of contexts and assertions over the
returned desire(s).
"""

from __future__ import annotations

import math

from drone_rescue_coordination.lib.domain.affect import ExploitationTracker
from drone_rescue_coordination.lib.domain.motivations import (
    INVESTIGATE,
    CoverageMotivation,
    SafetyMotivation,
    VictimMotivation,
)
from drone_rescue_coordination.lib.ports.affect_monitor import ExploitationSample
from drone_rescue_coordination.lib.ports.motivation import MotivationContext


# helpers

def _ctx(**kw):
    base = dict(
        drone_name='drone1',
        target=(0.0, 0.0),
        target_priority=2,
        distance_to_target=10.0,
        battery_ok=True,
        is_down=False,
        affect=None,
    )
    base.update(kw)
    return MotivationContext(**base)


# VictimMotivation

def test_victim_motivation_proposes_priority_over_distance():
    desires = list(VictimMotivation().propose(_ctx(target_priority=2, distance_to_target=10.0)))
    assert len(desires) == 1
    d = desires[0]
    assert d.intention_key == INVESTIGATE
    assert d.source == 'victim'
    assert math.isclose(d.strength, 0.2, abs_tol=1e-12)   # 2 / 10


def test_victim_motivation_clamps_short_distance_to_one_metre():
    """Mirrors the auction's ``priority / max(distance, 1)`` shape."""
    desires = list(VictimMotivation().propose(_ctx(distance_to_target=0.1)))
    assert math.isclose(desires[0].strength, 2.0, abs_tol=1e-12)


def test_victim_motivation_silent_when_no_target():
    assert tuple(VictimMotivation().propose(_ctx(target=None))) == ()


def test_victim_motivation_silent_when_drone_is_down_or_low_battery():
    assert tuple(VictimMotivation().propose(_ctx(is_down=True))) == ()
    assert tuple(VictimMotivation().propose(_ctx(battery_ok=False))) == ()


def test_victim_motivation_silent_when_distance_unknown():
    assert tuple(VictimMotivation().propose(_ctx(distance_to_target=None))) == ()


def test_victim_motivation_silent_when_priority_zero():
    assert tuple(VictimMotivation().propose(_ctx(target_priority=0))) == ()


# CoverageMotivation

def test_coverage_motivation_silent_at_default_pull_zero():
    assert tuple(CoverageMotivation().propose(_ctx())) == ()


def test_coverage_motivation_emits_negative_desire_when_pull_positive():
    desires = list(CoverageMotivation(coverage_pull=0.3).propose(_ctx()))
    assert len(desires) == 1
    d = desires[0]
    assert d.intention_key == INVESTIGATE
    assert d.strength == -0.3   # exact negative pull
    assert d.source == 'coverage'


# SafetyMotivation

def test_safety_motivation_silent_without_affect_monitor():
    assert tuple(SafetyMotivation().propose(_ctx(affect=None))) == ()


def test_safety_motivation_silent_when_drone_not_frustrated():
    affect = ExploitationTracker(stuck_threshold_s=30.0)
    # No observations -> frustration is 0 for any key
    assert tuple(SafetyMotivation().propose(_ctx(affect=affect))) == ()


def test_safety_motivation_inhibits_proportional_to_frustration():
    affect = ExploitationTracker(stuck_threshold_s=10.0)
    # Streak of 10 s -> frustration 1.0 for 'investigate:drone1'.
    affect.observe(ExploitationSample('investigate:drone1', 0.0, False))
    affect.observe(ExploitationSample('investigate:drone1', 10.0, False))
    desires = list(SafetyMotivation(frustration_inhibition_scale=5.0)
                   .propose(_ctx(affect=affect)))
    assert len(desires) == 1
    assert desires[0].intention_key == INVESTIGATE
    assert desires[0].strength == -5.0          # -1.0 * 5.0
    assert desires[0].source == 'safety'


def test_safety_motivation_scales_with_partial_frustration():
    affect = ExploitationTracker(stuck_threshold_s=10.0)
    affect.observe(ExploitationSample('investigate:drone1', 0.0, False))
    affect.observe(ExploitationSample('investigate:drone1', 3.0, False))   # 30%
    desires = list(SafetyMotivation(frustration_inhibition_scale=10.0)
                   .propose(_ctx(affect=affect)))
    assert math.isclose(desires[0].strength, -3.0, abs_tol=1e-9)


def test_safety_motivation_silent_with_zero_scale():
    affect = ExploitationTracker(stuck_threshold_s=10.0)
    affect.observe(ExploitationSample('investigate:drone1', 0.0, False))
    affect.observe(ExploitationSample('investigate:drone1', 30.0, False))
    assert tuple(SafetyMotivation(frustration_inhibition_scale=0.0)
                 .propose(_ctx(affect=affect))) == ()
