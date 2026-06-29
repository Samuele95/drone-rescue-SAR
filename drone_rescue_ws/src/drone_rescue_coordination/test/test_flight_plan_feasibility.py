"""Tests for mission_manager flight-plan feasibility wiring.

mission_manager now assesses, per drone, whether the remaining scan plan plus
the return leg fits inside the battery endurance reported on DroneHealth, folds
a GO/NO-GO(margin) marker into the MissionState summary, and emits a one-shot
FLIGHT_PLAN_INFEASIBLE warning when a drone flips infeasible. Bare-instance
pattern (no rclpy), same as test_operator_rth.
"""

from __future__ import annotations

from types import SimpleNamespace

from drone_rescue_coordination.mission_manager import MissionManager


def _bare_mm(*, speed=3.0, reserve=0.0, center=(0.0, 0.0)):
    mm = object.__new__(MissionManager)
    mm._battery_remaining_s = {}
    mm.mission_center = center
    mm._survey_speed_mps = speed
    mm._feasibility_reserve_s = reserve
    mm._infeasible_latch = {}
    mm._events = []
    mm._emit_event = lambda *a, **k: mm._events.append((a, k))
    return mm


def _wp(x, y):
    return SimpleNamespace(x=float(x), y=float(y))


def _drone(name, pose, wps, cursor=0):
    return SimpleNamespace(
        name=name, pose=pose, scan_waypoints=wps, scan_cursor=cursor)


def test_feasibility_none_when_endurance_unknown():
    mm = _bare_mm()
    d = _drone('drone1', _wp(0, 0), [_wp(10, 0), _wp(20, 0)])
    # no battery_remaining_s recorded yet
    assert mm._assess_drone_feasibility(d) is None


def test_feasibility_none_without_remaining_waypoints():
    mm = _bare_mm()
    mm._battery_remaining_s['drone1'] = 100.0
    d = _drone('drone1', _wp(0, 0), [_wp(10, 0)], cursor=1)  # cursor past end
    assert mm._assess_drone_feasibility(d) is None


def test_feasible_plan_with_ample_battery():
    mm = _bare_mm(speed=3.0)
    mm._battery_remaining_s['drone1'] = 1000.0
    d = _drone('drone1', _wp(0, 0), [_wp(30, 0), _wp(60, 0)])  # 60 m plan
    f = mm._assess_drone_feasibility(d)
    assert f is not None and f.feasible is True


def test_infeasible_plan_with_low_battery():
    mm = _bare_mm(speed=3.0, reserve=0.0)
    mm._battery_remaining_s['drone1'] = 5.0   # only 5 s left
    d = _drone('drone1', _wp(0, 0), [_wp(60, 0)])  # 60 m + 60 m home = 40 s
    f = mm._assess_drone_feasibility(d)
    assert f is not None and f.feasible is False
    assert f.margin_s < 0.0


def test_flip_emits_one_shot_event():
    mm = _bare_mm()
    infeasible = SimpleNamespace(feasible=False, margin_s=-12.0)
    feasible = SimpleNamespace(feasible=True, margin_s=20.0)
    mm._note_feasibility_flip('drone1', infeasible)
    mm._note_feasibility_flip('drone1', infeasible)   # still infeasible -> no 2nd
    events = [a[0] for a, k in mm._events]
    assert events.count('FLIGHT_PLAN_INFEASIBLE') == 1
    # recover then flip again -> a fresh event
    mm._note_feasibility_flip('drone1', feasible)
    mm._note_feasibility_flip('drone1', infeasible)
    events = [a[0] for a, k in mm._events]
    assert events.count('FLIGHT_PLAN_INFEASIBLE') == 2


def test_flip_ignores_unassessable():
    mm = _bare_mm()
    mm._note_feasibility_flip('drone1', None)
    assert mm._events == []
