"""Tests for the model-free stuck/frustration detector.

The detector is deliberately model-free: it takes only an
``ExploitationSample`` stream with explicit ``now_sec`` values, so we
can drive it with synthetic times, no FakeClock indirection needed.

The cases cover: the empty-state default, a single observation, a
clean unproductive streak crossing the threshold, a streak that
resets on progress, mixed-key independence, and the zero-threshold
guard.
"""

from __future__ import annotations

import math

from drone_rescue_coordination.lib.domain.affect import ExploitationTracker
from drone_rescue_coordination.lib.ports.affect_monitor import (
    ExploitationSample,
    StuckSignal,
)


def _obs(key, t, progress=False):
    return ExploitationSample(key=key, now_sec=t, made_progress=progress)


def test_unknown_key_has_zero_frustration_and_no_stuck_signal():
    t = ExploitationTracker(stuck_threshold_s=30.0)
    assert t.frustration('never-seen') == 0.0
    assert t.is_stuck('never-seen') is None


def test_first_progress_sample_starts_with_zero_frustration():
    t = ExploitationTracker(stuck_threshold_s=30.0)
    t.observe(_obs('B5-goal-seek', 0.0, progress=True))
    assert t.frustration('B5-goal-seek') == 0.0
    assert t.is_stuck('B5-goal-seek') is None


def test_first_unproductive_sample_seeds_a_streak_but_not_stuck():
    t = ExploitationTracker(stuck_threshold_s=30.0)
    t.observe(_obs('B5-goal-seek', 0.0, progress=False))
    # Streak is zero-length until time advances.
    assert t.frustration('B5-goal-seek') == 0.0
    assert t.is_stuck('B5-goal-seek') is None


def test_frustration_rises_linearly_with_unproductive_duration():
    t = ExploitationTracker(stuck_threshold_s=30.0)
    t.observe(_obs('B5-goal-seek', 0.0, progress=False))    # streak starts
    t.observe(_obs('B5-goal-seek', 6.0, progress=False))    # 6/30 = 0.2
    assert math.isclose(t.frustration('B5-goal-seek'), 0.2, abs_tol=1e-9)
    t.observe(_obs('B5-goal-seek', 21.0, progress=False))   # 21/30 = 0.7
    assert math.isclose(t.frustration('B5-goal-seek'), 0.7, abs_tol=1e-9)
    assert t.is_stuck('B5-goal-seek') is None


def test_stuck_signal_fires_at_threshold_and_carries_streak_info():
    t = ExploitationTracker(stuck_threshold_s=30.0)
    for tick, ts in enumerate([0.0, 10.0, 20.0, 30.0]):
        t.observe(_obs('B5-goal-seek', ts, progress=False))
    # 30 - 0 == threshold -> stuck.
    assert math.isclose(t.frustration('B5-goal-seek'), 1.0, abs_tol=1e-9)
    sig = t.is_stuck('B5-goal-seek')
    assert isinstance(sig, StuckSignal)
    assert sig.key == 'B5-goal-seek'
    assert math.isclose(sig.stuck_for_s, 30.0, abs_tol=1e-9)
    assert sig.unproductive_samples == 4   # all four samples


def test_progress_resets_the_streak_and_clears_stuck():
    t = ExploitationTracker(stuck_threshold_s=30.0)
    for ts in (0.0, 10.0, 20.0, 30.0):
        t.observe(_obs('B5-goal-seek', ts, progress=False))
    assert t.is_stuck('B5-goal-seek') is not None
    t.observe(_obs('B5-goal-seek', 31.0, progress=True))
    assert t.frustration('B5-goal-seek') == 0.0
    assert t.is_stuck('B5-goal-seek') is None
    # A fresh unproductive run reseeds.
    t.observe(_obs('B5-goal-seek', 32.0, progress=False))
    t.observe(_obs('B5-goal-seek', 62.0, progress=False))
    assert math.isclose(t.frustration('B5-goal-seek'), 1.0, abs_tol=1e-9)


def test_keys_are_independent():
    t = ExploitationTracker(stuck_threshold_s=10.0)
    t.observe(_obs('B5-goal-seek', 0.0, progress=False))
    t.observe(_obs('B5-goal-seek', 10.0, progress=False))
    t.observe(_obs('B2-explore-unvisited', 0.0, progress=True))
    assert t.is_stuck('B5-goal-seek') is not None
    assert t.is_stuck('B2-explore-unvisited') is None
    assert t.frustration('B2-explore-unvisited') == 0.0


def test_frustration_clamps_at_one_beyond_threshold():
    t = ExploitationTracker(stuck_threshold_s=10.0)
    t.observe(_obs('k', 0.0, progress=False))
    t.observe(_obs('k', 100.0, progress=False))   # 10x threshold
    assert t.frustration('k') == 1.0
    sig = t.is_stuck('k')
    assert sig is not None and math.isclose(sig.stuck_for_s, 100.0)


def test_zero_threshold_means_any_unproductive_observation_is_stuck():
    t = ExploitationTracker(stuck_threshold_s=0.0)
    t.observe(_obs('k', 5.0, progress=False))
    # frustration short-circuits to 1.0 to avoid division by zero
    assert t.frustration('k') == 1.0
    assert t.is_stuck('k') is not None


def test_known_keys_preserves_insertion_order():
    t = ExploitationTracker(stuck_threshold_s=30.0)
    for k in ('first', 'second', 'third'):
        t.observe(_obs(k, 0.0, progress=True))
    assert t.known_keys() == ('first', 'second', 'third')


def test_tracker_implements_protocol():
    """Structural conformance with the AffectMonitor Protocol."""
    from drone_rescue_coordination.lib.ports.affect_monitor import AffectMonitor
    t: AffectMonitor = ExploitationTracker()
    t.observe(_obs('k', 0.0, progress=True))
    assert t.frustration('k') == 0.0
    assert t.is_stuck('k') is None
