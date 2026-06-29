"""Tests for lib/surveyor_policy."""

from __future__ import annotations

import math

from drone_rescue_coordination.lib.surveyor_policy import (
    StuckRecoveryPolicy,
    scatter_direction_for,
)


def test_scatter_direction_drone1_NW():
    sx, sy = scatter_direction_for('drone1')
    # NW: x<0, y>0; magnitudes equal (normalised diagonal).
    assert sx < 0 and sy > 0
    assert math.isclose(math.hypot(sx, sy), 1.0, abs_tol=1e-6)


def test_scatter_direction_drone4_SE():
    sx, sy = scatter_direction_for('drone4')
    assert sx > 0 and sy < 0
    assert math.isclose(math.hypot(sx, sy), 1.0, abs_tol=1e-6)


def test_scatter_direction_drone5_W_pure_west():
    sx, sy = scatter_direction_for('drone5')
    assert math.isclose(sx, -1.0, abs_tol=1e-6)
    assert math.isclose(sy, 0.0, abs_tol=1e-6)


def test_scatter_direction_drone8_S():
    sx, sy = scatter_direction_for('drone8')
    assert math.isclose(sx, 0.0, abs_tol=1e-6)
    assert math.isclose(sy, -1.0, abs_tol=1e-6)


def test_scatter_direction_no_digits_falls_through_to_first():
    """A name without digits falls through to index 0 (NW)."""
    sx, sy = scatter_direction_for('mystery_drone')
    nw_sx, nw_sy = scatter_direction_for('drone1')
    assert math.isclose(sx, nw_sx, abs_tol=1e-6)
    assert math.isclose(sy, nw_sy, abs_tol=1e-6)


def test_scatter_direction_oversize_index_clamps():
    """A 9th drone in a future fleet doesn't IndexError; clamps to
    the last table entry (drone8 = S)."""
    sx, sy = scatter_direction_for('drone99')
    drone8 = scatter_direction_for('drone8')
    assert math.isclose(sx, drone8[0], abs_tol=1e-6)
    assert math.isclose(sy, drone8[1], abs_tol=1e-6)


def test_stuck_recovery_policy_defaults():
    p = StuckRecoveryPolicy()
    assert p.stuck_threshold_s == 30.0
    assert p.stuck_max_retries == 3
    assert p.stuck_altitude_increase_m == 2.0


def test_stuck_recovery_policy_escalated_altitude():
    p = StuckRecoveryPolicy()
    # First retry climbs 1 step above base; retry 0 + 1 step = +2.0.
    assert p.escalated_altitude(10.0, 0) == 12.0
    assert p.escalated_altitude(10.0, 1) == 14.0
    assert p.escalated_altitude(10.0, 2) == 16.0


def test_stuck_recovery_policy_is_frozen():
    p = StuckRecoveryPolicy()
    try:
        p.stuck_max_retries = 999   # type: ignore[misc]
    except Exception:
        return
    assert False, 'StuckRecoveryPolicy should be frozen'
