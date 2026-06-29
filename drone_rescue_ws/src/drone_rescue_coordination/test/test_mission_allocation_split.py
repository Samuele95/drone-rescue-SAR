"""Unit tests for the allocation / saga API split on Mission.

Pure-Python; no rclpy. Verifies that L3 collective-allocation queries
(``candidates_awaiting_allocation``, ``free_drones``) and the L2
saga preparation (``ensure_sub_mission_for_each``) compose
correctly with the existing aggregate state.
"""
from __future__ import annotations

from drone_rescue_coordination.lib.domain.entities import Drone, Victim
from drone_rescue_coordination.lib.domain.mission import Mission
from drone_rescue_coordination.lib.domain.state_machines import VictimStage
from drone_rescue_coordination.lib.domain.task_type import TaskType
from drone_rescue_coordination.lib.domain.value_objects import Position
from drone_rescue_coordination.lib.domain.victim_sub_mission import (
    VictimSubMission,
)


def _make_mission() -> Mission:
    m = Mission()
    m.register_drone(Drone(name='drone1'))
    m.register_drone(Drone(name='drone2'))
    m.register_drone(Drone(name='drone3'))
    return m


def test_candidates_awaiting_allocation_filters_confirmed():
    m = _make_mission()
    m.victims[1] = Victim(candidate_id=1, position=Position(),
                          confidence=0.9, stage=VictimStage.CONFIRMED)
    m.victims[2] = Victim(candidate_id=2, position=Position(),
                          confidence=0.5, stage=VictimStage.DETECTED)
    m.victims[3] = Victim(candidate_id=3, position=Position(),
                          confidence=0.6, stage=VictimStage.INVESTIGATING)
    awaiting = m.candidates_awaiting_allocation()
    ids = {v.candidate_id for v in awaiting}
    assert ids == {2, 3}


def test_free_drones_excludes_busy_and_down():
    m = _make_mission()
    m.drones['drone1'].current_task_type = TaskType.INVESTIGATE
    m.drones['drone2'].is_down = True
    free = m.free_drones()
    names = [d.name for d in free]
    assert names == ['drone3']


def test_free_drones_stable_ordering_by_name():
    m = _make_mission()
    free = m.free_drones()
    assert [d.name for d in free] == ['drone1', 'drone2', 'drone3']


def test_ensure_sub_mission_for_each_creates_one_per_candidate():
    m = _make_mission()
    v1 = Victim(candidate_id=10, position=Position(),
                confidence=0.5, stage=VictimStage.DETECTED)
    v2 = Victim(candidate_id=11, position=Position(),
                confidence=0.6, stage=VictimStage.DETECTED)
    subs = m.ensure_sub_mission_for_each((v1, v2))
    assert len(subs) == 2
    assert all(isinstance(s, VictimSubMission) for s in subs)
    # Mission's sub_missions dict updated.
    assert {s.victim.candidate_id for s in subs} == {10, 11}
    assert set(m.sub_missions.keys()) == {10, 11}


def test_ensure_sub_mission_for_each_is_idempotent():
    """Calling twice with the same candidates returns the same
    aggregates: the L3 planner can call this every tick safely."""
    m = _make_mission()
    v = Victim(candidate_id=42, position=Position(),
               confidence=0.5, stage=VictimStage.DETECTED)
    first = m.ensure_sub_mission_for_each((v,))
    second = m.ensure_sub_mission_for_each((v,))
    assert first[0] is second[0]


def test_candidates_and_free_drones_compose_with_snapshot_world():
    """The L3 surface (allocation queries + WorldModel snapshot) is
    consistent: the same drones marked free here appear in
    WorldModel.idle_drones(); the same victims awaiting allocation
    appear in WorldModel.unconfirmed_candidates."""
    m = _make_mission()
    m.drones['drone1'].current_task_type = TaskType.INVESTIGATE
    m.victims[1] = Victim(candidate_id=1, position=Position(),
                          confidence=0.5, stage=VictimStage.DETECTED)

    free = m.free_drones()
    awaiting = m.candidates_awaiting_allocation()
    world = m.snapshot_world(now_sec=0.0)

    assert {d.name for d in free} == {d.name for d in world.idle_drones()}
    assert ({v.candidate_id for v in awaiting}
            == {v.candidate_id for v in world.unconfirmed_candidates})
