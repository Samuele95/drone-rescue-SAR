"""Tests for the windowed scan-time ETA.

coverage_tracker estimated remaining time from a whole-mission linear rate
(coverage_pct / elapsed), which lags: early arming drags the average down for
the whole run. lib/domain/coverage_eta computes the rate from a recent sliding
window so the ETA tracks the current progress rate. Pure pytest.
"""

from __future__ import annotations

import pytest

from drone_rescue_coordination.lib.domain.coverage_eta import (
    estimate_remaining,
    prune_window,
    windowed_rate,
)


def test_windowed_rate_is_the_window_slope():
    # 4% gained over 8 s within the window -> 0.5 %/s.
    samples = [(10.0, 20.0), (14.0, 22.0), (18.0, 24.0)]
    assert windowed_rate(samples) == pytest.approx(0.5)


def test_windowed_rate_needs_two_samples():
    assert windowed_rate([(10.0, 20.0)]) is None
    assert windowed_rate([]) is None


def test_windowed_rate_zero_span_is_none():
    assert windowed_rate([(10.0, 20.0), (10.0, 25.0)]) is None


def test_estimate_remaining_from_rate():
    # 40% left at 0.5 %/s -> 80 s.
    assert estimate_remaining(60.0, 0.5) == pytest.approx(80.0)


def test_estimate_remaining_unknown_or_stalled_is_zero():
    assert estimate_remaining(60.0, None) == 0.0
    assert estimate_remaining(60.0, 0.0) == 0.0
    assert estimate_remaining(60.0, -0.1) == 0.0


def test_prune_drops_old_samples_keeps_recent():
    samples = [(0.0, 0.0), (5.0, 5.0), (40.0, 30.0), (50.0, 40.0)]
    prune_window(samples, now_t=50.0, window_s=30.0)
    # cutoff = 20.0 -> the 0.0 and 5.0 samples drop.
    assert samples == [(40.0, 30.0), (50.0, 40.0)]


def test_prune_always_keeps_at_least_one():
    samples = [(0.0, 0.0)]
    prune_window(samples, now_t=100.0, window_s=30.0)
    assert samples == [(0.0, 0.0)]


def test_windowed_beats_linear_on_late_speedup():
    """A run that accelerates late: the windowed rate gives a shorter (more
    accurate) ETA than the whole-mission linear rate would."""
    # Slow first 50 s (10%), then fast: +20% over the next 10 s.
    window = [(50.0, 10.0), (60.0, 30.0)]    # recent window: 2.0 %/s
    cov = 30.0
    windowed_eta = estimate_remaining(cov, windowed_rate(window))
    linear_rate = cov / 60.0                  # 0.5 %/s whole-mission
    linear_eta = estimate_remaining(cov, linear_rate)
    assert windowed_eta < linear_eta
