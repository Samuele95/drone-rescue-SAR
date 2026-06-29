"""Unit tests for the lib/domain layer.

No rclpy; these run pure-Python in <1 s.
"""

from __future__ import annotations

import pytest

from drone_rescue_coordination.lib.domain import (
    Drone, Victim, Position,
    SectorWedge, ScanPlan, Bid, OutgoingTask, MissionStateSnapshot,
    MissionStage, VictimStage,
    MissionStateMachine, VictimStateMachine, TransitionEvent, IllegalTransition,
    VictimSubMission, Mission,
)


# SectorWedge

def test_sector_wedge_simple_contains():
    w = SectorWedge(start_rad=0.0, end_rad=1.0)
    assert w.contains(0.5)
    assert w.contains(0.0)     # inclusive start
    assert not w.contains(1.0) # exclusive end
    assert not w.contains(2.0)


def test_sector_wedge_wraps_seam():
    """Start > end means a wedge that crosses 0/2π."""
    import math
    w = SectorWedge(start_rad=5.8, end_rad=0.5)
    # Late half (5.8 ≤ θ < 2π)
    assert w.contains(5.9)
    assert w.contains(2 * math.pi - 0.01)
    # Early half (0 ≤ θ < 0.5)
    assert w.contains(0.0)
    assert w.contains(0.4)
    # Outside both halves
    assert not w.contains(1.0)
    assert not w.contains(3.0)


# ScanPlan

def test_scan_plan_length():
    plan = ScanPlan(waypoints=((1.0, 2.0), (3.0, 4.0), (5.0, 6.0)))
    assert plan.length == 3
    assert plan.wedge is None


def test_scan_plan_with_wedge():
    w = SectorWedge(0.0, 1.57)
    plan = ScanPlan(waypoints=((1.0, 2.0),), wedge=w)
    assert plan.wedge is w


def test_scan_plan_is_frozen():
    plan = ScanPlan(waypoints=())
    with pytest.raises(Exception):
        plan.waypoints = ((1.0, 2.0),)   # type: ignore[misc]


# Drone

def test_drone_starts_with_no_plan():
    d = Drone(name='drone1')
    assert d.scan_plan is None
    assert d.scan_cursor == 0
    assert d.remaining_scan_waypoints == 0


def test_drone_set_plan_resets_cursor():
    d = Drone(name='drone1')
    from dataclasses import replace
    d.scan_cursor = 5
    d.clock = replace(d.clock, last_dispatch_offset=3)
    plan = ScanPlan(waypoints=((0, 0), (1, 1), (2, 2)))
    d.set_plan(plan)
    assert d.scan_cursor == 0
    assert d.clock.last_dispatch_offset == 0
    assert d.remaining_scan_waypoints == 3


def test_watchdog_clock_silence():
    from drone_rescue_coordination.lib.domain import WatchdogClock
    c = WatchdogClock(last_status_t=100.0, task_dispatched_t=120.0)
    assert c.silence(125.0) == 5.0
    # task_dispatched_t (120) is later than last_status_t (100); silence
    # is measured from the later of the two.
    c2 = WatchdogClock(last_status_t=200.0, task_dispatched_t=120.0)
    assert c2.silence(210.0) == 10.0


def test_drone_set_plan_with_wedge_assigns_sector():
    d = Drone(name='drone1')
    w = SectorWedge(0.0, 1.57)
    plan = ScanPlan(waypoints=((0, 0),), wedge=w)
    d.set_plan(plan)
    assert d.sector_wedge is w


# MissionStateMachine

def test_mission_fsm_canonical_path():
    """INIT → ARMING → DEPLOYING → SCANNING → COMPLETE."""
    s = MissionStage.INIT
    s = MissionStateMachine.transition(s, TransitionEvent.LIFECYCLE_CONFIGURED)
    assert s == MissionStage.ARMING
    s = MissionStateMachine.transition(s, TransitionEvent.LIFECYCLE_ACTIVATED)
    assert s == MissionStage.DEPLOYING
    s = MissionStateMachine.transition(s, TransitionEvent.SURVEY_STARTED)
    assert s == MissionStage.SCANNING
    s = MissionStateMachine.transition(s, TransitionEvent.MISSION_COMPLETE)
    assert s == MissionStage.COMPLETE


def test_mission_fsm_investigate_loop():
    """SCANNING → INVESTIGATING → SCANNING (back) → ..."""
    s = MissionStage.SCANNING
    s = MissionStateMachine.transition(s, TransitionEvent.INVESTIGATE_DISPATCHED)
    assert s == MissionStage.INVESTIGATING
    s = MissionStateMachine.transition(s, TransitionEvent.SCAN_RESUMED)
    assert s == MissionStage.SCANNING


def test_mission_fsm_illegal_transition_raises():
    """INIT can't jump straight to SCANNING."""
    with pytest.raises(IllegalTransition):
        MissionStateMachine.transition(
            MissionStage.INIT, TransitionEvent.SURVEY_STARTED,
        )


def test_mission_fsm_can_transition_inspection():
    assert MissionStateMachine.can_transition(
        MissionStage.SCANNING, TransitionEvent.MISSION_COMPLETE,
    )
    assert not MissionStateMachine.can_transition(
        MissionStage.INIT, TransitionEvent.SURVEY_STARTED,
    )


def test_mission_fsm_complete_is_terminal():
    """No transition out of COMPLETE: the silent re-entry from
    /survey/start after MISSION_COMPLETE must raise."""
    with pytest.raises(IllegalTransition):
        MissionStateMachine.transition(
            MissionStage.COMPLETE, TransitionEvent.SURVEY_STARTED,
        )


def test_mission_fsm_arming_cannot_skip_to_scanning():
    """ARMING → SCANNING requires going through DEPLOYING."""
    with pytest.raises(IllegalTransition):
        MissionStateMachine.transition(
            MissionStage.ARMING, TransitionEvent.SURVEY_STARTED,
        )


def test_victim_fsm_pre_confirmed_shortcut_legal():
    """detection_filter pre-confirmed candidates skip INVESTIGATING.
    DETECTED → CONFIRMED is an explicit table row."""
    s = VictimStateMachine.transition(
        VictimStage.DETECTED, TransitionEvent.CONFIRMED,
    )
    assert s == VictimStage.CONFIRMED


def test_victim_fsm_no_resurrect_from_rejected():
    with pytest.raises(IllegalTransition):
        VictimStateMachine.transition(
            VictimStage.REJECTED, TransitionEvent.INVESTIGATE_BEGAN,
        )


# VictimStateMachine

def test_victim_fsm_canonical_path():
    s = VictimStage.DETECTED
    s = VictimStateMachine.transition(s, TransitionEvent.INVESTIGATE_BEGAN)
    assert s == VictimStage.INVESTIGATING
    s = VictimStateMachine.transition(s, TransitionEvent.CONFIRMED)
    assert s == VictimStage.CONFIRMED


def test_victim_fsm_compensation_path():
    """INVESTIGATING → DETECTED on saga compensation (drone down)."""
    s = VictimStage.INVESTIGATING
    s = VictimStateMachine.transition(s, TransitionEvent.INVESTIGATE_FAILED)
    assert s == VictimStage.DETECTED


def test_victim_fsm_reject_from_detected():
    s = VictimStateMachine.transition(
        VictimStage.DETECTED, TransitionEvent.REJECTED,
    )
    assert s == VictimStage.REJECTED


def test_victim_fsm_no_transition_from_confirmed():
    """CONFIRMED is terminal: no further legal transitions."""
    with pytest.raises(IllegalTransition):
        VictimStateMachine.transition(
            VictimStage.CONFIRMED, TransitionEvent.INVESTIGATE_FAILED,
        )


# VictimSubMission

def _make_victim(vid=1, x=10.0, y=20.0):
    return Victim(
        candidate_id=vid,
        position=Position(x=x, y=y, z=0.0),
        confidence=0.85,
    )


def test_victim_sub_mission_dispatch_investigate():
    v = _make_victim()
    d = Drone(name='drone1')
    sm = VictimSubMission(victim=v)
    task = sm.dispatch_investigate(d, now_sec=100.0, hover_seconds=4.0)
    assert sm.stage == VictimStage.INVESTIGATING
    assert sm.assigned_drone is d
    assert task.drone_name == 'drone1'
    assert task.task_type == 1   # INVESTIGATE
    assert task.victim_id == 1
    assert task.target == (10.0, 20.0, 0.0)
    assert task.hover_seconds == 4.0
    assert 'drone1' in sm.witnesses


def test_victim_sub_mission_cross_drone_confirm():
    """The CONFIRM dispatch hands off to a SECOND drone."""
    v = _make_victim()
    d1 = Drone(name='drone1')
    d2 = Drone(name='drone2')
    sm = VictimSubMission(victim=v)
    sm.dispatch_investigate(d1, now_sec=100.0)
    confirm = sm.dispatch_confirm(d2, now_sec=110.0, confirm_orbit_radius=4.0)
    assert sm.assigned_drone is d2
    assert confirm.task_type == 2   # CONFIRM
    assert confirm.confirm_orbit_radius == 4.0
    assert 'drone1' in sm.witnesses
    assert 'drone2' in sm.witnesses


def test_victim_sub_mission_compensate_returns_to_detected():
    v = _make_victim()
    d = Drone(name='drone1')
    sm = VictimSubMission(victim=v)
    sm.dispatch_investigate(d, now_sec=100.0)
    sm.compensate()
    assert sm.stage == VictimStage.DETECTED
    assert sm.assigned_drone is None
    assert sm.victim.assigned_drone is None


def test_victim_sub_mission_on_complete():
    v = _make_victim()
    d = Drone(name='drone1')
    sm = VictimSubMission(victim=v)
    sm.dispatch_investigate(d, now_sec=100.0)
    sm.on_complete()
    assert sm.stage == VictimStage.CONFIRMED


# Mission

def test_mission_register_drone():
    m = Mission()
    m.register_drone(Drone(name='drone1'))
    m.register_drone(Drone(name='drone2'))
    assert set(m.drones.keys()) == {'drone1', 'drone2'}


def test_mission_transition():
    m = Mission()
    m.transition(TransitionEvent.LIFECYCLE_CONFIGURED)
    assert m.stage == MissionStage.ARMING


def test_mission_ensure_sub_mission_is_idempotent():
    """Same candidate_id → same VictimSubMission."""
    m = Mission()
    v = _make_victim(vid=42)
    s1 = m.ensure_sub_mission(v)
    s2 = m.ensure_sub_mission(v)
    assert s1 is s2
    assert m.victims[42] is v


def test_mission_snapshot_summary():
    m = Mission()
    m.register_drone(Drone(name='drone1', current_task_type=0))   # SCAN
    m.register_drone(Drone(name='drone2', current_task_type=1, busy_with_victim=7))
    snap = m.snapshot()
    assert snap.stage == int(MissionStage.INIT)
    assert len(snap.active_tasks_summary) == 2
    # drone2 should mention its victim
    assert any('[v7]' in s for s in snap.active_tasks_summary)


def test_mission_tick_no_op_before_start():
    """tick() on an un-started Mission (no mission_start_sec, no
    strategy) returns () without raising.
    """
    m = Mission()
    assert m.tick(now_sec=0.0) == ()
