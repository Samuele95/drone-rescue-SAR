"""Unit tests for lib/domain/world_model.py + Mission.snapshot_world.

Pure-Python; no rclpy.
"""
from __future__ import annotations

import pytest

from drone_rescue_coordination.lib.domain.entities import Drone, Victim
from drone_rescue_coordination.lib.domain.mission import Mission
from drone_rescue_coordination.lib.domain.state_machines import VictimStage
from drone_rescue_coordination.lib.domain.task_type import TaskType
from drone_rescue_coordination.lib.domain.value_objects import Position
from drone_rescue_coordination.lib.domain.world_model import WorldModel


def _make_mission_with_fleet() -> Mission:
    m = Mission()
    m.register_drone(Drone(name='drone1'))
    m.register_drone(Drone(name='drone2'))
    m.register_drone(Drone(name='drone3'))
    return m


def test_world_model_is_frozen():
    w = WorldModel(
        fleet={},
        confirmed_victims=(),
        unconfirmed_candidates=(),
        coverage_pct=0.0,
        active_tasks=(),
        no_fly_zones=(),
        now_sec=0.0,
    )
    with pytest.raises(Exception):
        w.coverage_pct = 100.0   # type: ignore[misc]


def test_world_model_fleet_size():
    fleet = {'d1': Drone(name='d1'), 'd2': Drone(name='d2')}
    w = WorldModel(
        fleet=fleet, confirmed_victims=(), unconfirmed_candidates=(),
        coverage_pct=0.0, active_tasks=(), no_fly_zones=(), now_sec=0.0,
    )
    assert w.fleet_size == 2


def test_world_model_victims_seen_counts_both():
    confirmed = (Victim(candidate_id=1, position=Position(),
                        confidence=0.9, stage=VictimStage.CONFIRMED),)
    pending = (
        Victim(candidate_id=2, position=Position(),
               confidence=0.5, stage=VictimStage.DETECTED),
        Victim(candidate_id=3, position=Position(),
               confidence=0.6, stage=VictimStage.INVESTIGATING),
    )
    w = WorldModel(
        fleet={}, confirmed_victims=confirmed, unconfirmed_candidates=pending,
        coverage_pct=0.0, active_tasks=(), no_fly_zones=(), now_sec=0.0,
    )
    assert w.victims_seen == 3


def test_world_model_idle_drones_filters_busy_and_down():
    fleet = {
        'idle1': Drone(name='idle1'),
        'busy':  Drone(name='busy', current_task_type=TaskType.INVESTIGATE),
        'idle2': Drone(name='idle2'),
        'down':  Drone(name='down', is_down=True),
    }
    w = WorldModel(
        fleet=fleet, confirmed_victims=(), unconfirmed_candidates=(),
        coverage_pct=0.0, active_tasks=(), no_fly_zones=(), now_sec=0.0,
    )
    idle = w.idle_drones()
    names = {d.name for d in idle}
    assert names == {'idle1', 'idle2'}


def test_mission_snapshot_world_separates_confirmed_from_pending():
    m = _make_mission_with_fleet()
    m.victims[1] = Victim(candidate_id=1, position=Position(10, 10, 0),
                          confidence=0.9, stage=VictimStage.CONFIRMED)
    m.victims[2] = Victim(candidate_id=2, position=Position(20, 20, 0),
                          confidence=0.5, stage=VictimStage.DETECTED)
    m.victims[3] = Victim(candidate_id=3, position=Position(30, 30, 0),
                          confidence=0.6, stage=VictimStage.INVESTIGATING)
    m.victims[4] = Victim(candidate_id=4, position=Position(40, 40, 0),
                          confidence=0.2, stage=VictimStage.REJECTED)

    w = m.snapshot_world(now_sec=10.0)

    confirmed_ids = {v.candidate_id for v in w.confirmed_victims}
    pending_ids = {v.candidate_id for v in w.unconfirmed_candidates}
    assert confirmed_ids == {1}
    assert pending_ids == {2, 3}      # REJECTED dropped from both


def test_mission_snapshot_world_carries_clock():
    m = _make_mission_with_fleet()
    w = m.snapshot_world(now_sec=123.5)
    assert w.now_sec == 123.5


def test_mission_snapshot_world_includes_full_fleet():
    m = _make_mission_with_fleet()
    w = m.snapshot_world(now_sec=0.0)
    assert set(w.fleet.keys()) == {'drone1', 'drone2', 'drone3'}


def test_mission_snapshot_world_returns_frozen_instance():
    m = _make_mission_with_fleet()
    w = m.snapshot_world(now_sec=0.0)
    assert isinstance(w, WorldModel)
    with pytest.raises(Exception):
        w.coverage_pct = 50.0   # type: ignore[misc]


def test_mission_snapshot_world_coverage_zero_when_no_sectors():
    m = _make_mission_with_fleet()
    # sectors_total defaults to 0; division-by-zero is guarded in
    # _build_world_model via max(1, sectors_total).
    w = m.snapshot_world(now_sec=0.0)
    assert w.coverage_pct == 0.0


def test_mission_plan_returns_empty_without_strategy():
    """plan() is lifted; with no allocation strategy wired it
    returns () (inert)."""
    m = _make_mission_with_fleet()
    w = m.snapshot_world(now_sec=0.0)
    assert m.plan(w) == ()


def test_mission_replan_no_op_for_idle_drone():
    """replan() is lifted; a failed task for a drone with no
    busy victim and no remaining scan returns () (no compensation
    needed)."""
    m = _make_mission_with_fleet()
    w = m.snapshot_world(now_sec=0.0)
    from drone_rescue_coordination.lib.domain.value_objects import OutgoingTask
    failed = OutgoingTask(
        drone_name='drone1', task_type=int(TaskType.SCAN),
        waypoints=(), target=None, victim_id=0, priority=0,
        hover_seconds=0.0,
    )
    assert m.replan(w, failed) == ()
