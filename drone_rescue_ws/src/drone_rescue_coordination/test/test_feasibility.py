"""Tests for flight-plan feasibility.

lib/domain/feasibility checks remaining battery endurance against the time to
fly the remaining scan plan plus the return leg. Pure pytest.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from drone_rescue_coordination.lib.domain.feasibility import (
    assess_feasibility,
    remaining_plan_length,
)


def _wp(x, y):
    return SimpleNamespace(x=float(x), y=float(y))


def test_remaining_plan_length_sums_legs():
    # (0,0)->(0,3)->(4,3): 3 + 4 = 7.
    assert remaining_plan_length((0.0, 0.0), [_wp(0, 3), _wp(4, 3)]) == pytest.approx(7.0)


def test_remaining_plan_length_empty_is_zero():
    assert remaining_plan_length((5.0, 5.0), []) == 0.0


def test_feasible_when_endurance_covers_plan_plus_return():
    # 100 m plan + 50 m home = 150 m at 3 m/s = 50 s; endurance 120 s.
    f = assess_feasibility(
        drone_name='drone1', remaining_plan_m=100.0, return_home_m=50.0,
        speed_mps=3.0, endurance_s=120.0, reserve_s=0.0)
    assert f.feasible is True
    assert f.time_needed_s == pytest.approx(50.0)
    assert f.margin_s == pytest.approx(70.0)


def test_infeasible_when_endurance_short():
    # need 50 s, endurance only 40 s -> no-go by 10 s.
    f = assess_feasibility(
        drone_name='drone2', remaining_plan_m=100.0, return_home_m=50.0,
        speed_mps=3.0, endurance_s=40.0)
    assert f.feasible is False
    assert f.margin_s == pytest.approx(-10.0)


def test_reserve_eats_into_margin():
    # need 50 s, endurance 60 s, reserve 30 s -> margin -20 -> no-go.
    f = assess_feasibility(
        drone_name='d', remaining_plan_m=100.0, return_home_m=50.0,
        speed_mps=3.0, endurance_s=60.0, reserve_s=30.0)
    assert f.feasible is False
    assert f.margin_s == pytest.approx(-20.0)


def test_zero_speed_is_guarded_not_divide_by_zero():
    f = assess_feasibility(
        drone_name='d', remaining_plan_m=10.0, return_home_m=0.0,
        speed_mps=0.0, endurance_s=1e9)
    # huge time_needed, but no exception; with 1e9 endurance still feasible.
    assert f.time_needed_s > 0.0
