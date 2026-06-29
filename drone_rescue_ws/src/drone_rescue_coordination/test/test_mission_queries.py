"""Unit coverage for Mission read-side queries, plus the
NotImplementedError skeleton marker on ``Mission.tick``.

CQRS read API on the Mission aggregate: strictly read, no mutation
on the saga side. Each helper walks the same dicts mission_manager
mutates today; tests verify the read shape is correct so callers
can delegate.
"""

from drone_rescue_coordination.lib.domain.entities import Drone, Victim
from drone_rescue_coordination.lib.domain.mission import Mission
from drone_rescue_coordination.lib.domain.value_objects import Position
from drone_rescue_coordination.lib.domain.victim_sub_mission import (
    VictimSubMission,
)


# confirmed_count

def test_confirmed_count_empty_mission():
    m = Mission()
    assert m.confirmed_count() == 0


def test_confirmed_count_counts_only_stage_2():
    m = Mission()
    # Stage 0 = DETECTED, 1 = INVESTIGATING, 2 = CONFIRMED, 3 = REJECTED.
    m.sub_missions = {
        1: VictimSubMission(victim=Victim(1, Position(0, 0, 0), 0.9), stage=2),
        2: VictimSubMission(victim=Victim(2, Position(1, 1, 0), 0.8), stage=0),
        3: VictimSubMission(victim=Victim(3, Position(2, 2, 0), 0.7), stage=2),
        4: VictimSubMission(victim=Victim(4, Position(3, 3, 0), 0.5), stage=3),
    }
    assert m.confirmed_count() == 2


# unconfirmed_candidates

def test_unconfirmed_candidates_filters_stage_in_set_0_1():
    m = Mission()
    m.victims = {
        1: Victim(1, Position(0, 0, 0), 0.9, stage=0),    # DETECTED
        2: Victim(2, Position(1, 1, 0), 0.8, stage=1),    # INVESTIGATING
        3: Victim(3, Position(2, 2, 0), 0.7, stage=2),    # CONFIRMED, excluded
        4: Victim(4, Position(3, 3, 0), 0.5, stage=3),    # REJECTED, excluded
    }
    result = m.unconfirmed_candidates()
    assert tuple(v.candidate_id for v in result) == (1, 2)


def test_unconfirmed_candidates_ordered_by_id():
    m = Mission()
    m.victims = {
        7: Victim(7, Position(0, 0, 0), 0.9, stage=0),
        2: Victim(2, Position(1, 1, 0), 0.8, stage=0),
        5: Victim(5, Position(2, 2, 0), 0.7, stage=1),
    }
    result = m.unconfirmed_candidates()
    assert tuple(v.candidate_id for v in result) == (2, 5, 7)


# busy_count

def test_busy_count_excludes_idle_drones():
    m = Mission()
    m.drones = {
        'drone1': Drone(name='drone1', current_task_type=5),   # IDLE
        'drone2': Drone(name='drone2', current_task_type=2),   # SCAN
        'drone3': Drone(name='drone3', current_task_type=3),   # INVESTIGATE
        'drone4': Drone(name='drone4', current_task_type=5),   # IDLE
    }
    assert m.busy_count() == 2


# dispatched_investigate_count

def test_dispatched_investigate_count_counts_only_busy_with_victim():
    """Narrower than busy_count: counts only drones that have a
    busy_with_victim slot set (mid-INVESTIGATE or mid-CONFIRM)."""
    m = Mission()
    m.drones = {
        'drone1': Drone(name='drone1', current_task_type=2,
                        busy_with_victim=7),       # mid-CONFIRM, count
        'drone2': Drone(name='drone2', current_task_type=2,
                        busy_with_victim=None),    # SCAN-busy but no victim
        'drone3': Drone(name='drone3', current_task_type=5,
                        busy_with_victim=None),    # IDLE
        'drone4': Drone(name='drone4', current_task_type=3,
                        busy_with_victim=11),      # mid-INVESTIGATE, count
    }
    assert m.dispatched_investigate_count() == 2


def test_dispatched_investigate_count_zero_when_none_dispatched():
    m = Mission()
    m.drones = {
        f'drone{i}': Drone(name=f'drone{i}', current_task_type=2)
        for i in range(1, 5)
    }
    assert m.dispatched_investigate_count() == 0


def test_busy_count_zero_when_all_idle():
    m = Mission()
    m.drones = {
        f'drone{i}': Drone(name=f'drone{i}', current_task_type=5)
        for i in range(1, 5)
    }
    assert m.busy_count() == 0


# survivor_set

def test_survivor_set_excludes_dead_and_down_drones():
    m = Mission()
    m.drones = {
        'drone1': Drone(name='drone1'),
        'drone2': Drone(name='drone2', is_down=True),
        'drone3': Drone(name='drone3'),
        'drone4': Drone(name='drone4'),
    }
    result = m.survivor_set('drone3')
    names = tuple(d.name for d in result)
    # drone3 dead, drone2 already down; survivors are drone1, drone4.
    assert names == ('drone1', 'drone4')


def test_survivor_set_returns_all_when_no_one_down_and_caller_alive():
    """When the caller passes a name not in the fleet (defensive),
    the result is every healthy drone."""
    m = Mission()
    m.drones = {
        'drone1': Drone(name='drone1'),
        'drone2': Drone(name='drone2'),
    }
    result = m.survivor_set('phantom')
    assert tuple(d.name for d in result) == ('drone1', 'drone2')


# remaining_scan_waypoints_total

def test_remaining_scan_waypoints_total_sums_fleet_remaining():
    """`Drone.remaining_scan_waypoints` returns 0 when scan_plan is
    None; verify the aggregate sum handles that gracefully."""
    m = Mission()
    m.drones = {
        'drone1': Drone(name='drone1'),   # no plan → 0
        'drone2': Drone(name='drone2'),   # no plan → 0
    }
    assert m.remaining_scan_waypoints_total() == 0


# snapshot delegation

# tick

def test_tick_no_op_before_mission_start():
    """With no mission_start_sec set (mission not yet begun) and no
    strategy wired, tick() returns () without raising."""
    m = Mission()
    assert m.tick(now_sec=0.0) == ()


# snapshot delegation

def test_snapshot_busy_with_victim_zero_emits_v0_indicator():
    """Falsy 0 used to silently suppress the `[v0]` indicator; the
    corrected `is not None` guard surfaces it."""
    m = Mission()
    m.drones = {'drone1': Drone(name='drone1', busy_with_victim=0)}
    snap = m.snapshot()
    summaries_joined = ' '.join(snap.active_tasks_summary)
    assert '[v0]' in summaries_joined


def test_snapshot_victims_confirmed_uses_confirmed_count():
    """Snapshot still reports the right victim count after the
    delegation rewire."""
    m = Mission()
    m.sub_missions = {
        1: VictimSubMission(victim=Victim(1, Position(0, 0, 0), 0.9), stage=2),
        2: VictimSubMission(victim=Victim(2, Position(1, 1, 0), 0.8), stage=2),
        3: VictimSubMission(victim=Victim(3, Position(2, 2, 0), 0.5), stage=0),
    }
    snap = m.snapshot()
    assert snap.victims_confirmed == 2


# pre-flight fields

def test_drone_scan_waypoints_default_empty():
    """New dispatched-waypoint carrier defaults empty."""
    d = Drone(name='drone1')
    assert d.scan_waypoints == []


def test_drone_append_scan_tail_extends_list():
    """Handover appends a survivor's orphan run."""
    d = Drone(name='drone1')
    d.append_scan_tail([Position(1, 2, 25), Position(3, 4, 25)])
    d.append_scan_tail([Position(5, 6, 25)])
    assert len(d.scan_waypoints) == 3
    assert d.scan_waypoints[-1] == Position(5, 6, 25)


def test_drone_clear_scan_state_drains_all():
    """Drone loss / scan completion drains the list."""
    d = Drone(name='drone1')
    d.append_scan_tail([Position(1, 2, 25)])
    d.scan_cursor = 1
    d.clear_scan_state()
    assert d.scan_waypoints == []
    assert d.scan_cursor == 0


def test_drone_remaining_scan_waypoints_prefers_dispatched_list():
    """Remaining count uses the dispatched list when populated, else
    falls back to ScanPlan length."""
    d = Drone(name='drone1')
    d.append_scan_tail([Position(0, 0, 25), Position(1, 1, 25),
                        Position(2, 2, 25)])
    d.scan_cursor = 1
    assert d.remaining_scan_waypoints == 2


def test_victim_sub_mission_default_witness_none():
    """witness_drone cache defaults None and is distinct from the
    witnesses roster."""
    v = Victim(candidate_id=1, position=Position(0, 0, 0), confidence=0.5)
    sm = VictimSubMission(victim=v)
    assert sm.witness_drone is None
    assert sm.witnesses == []


def test_victim_sub_mission_investigate_state_defaults():
    """Multi-view INVESTIGATE state defaults."""
    v = Victim(candidate_id=1, position=Position(0, 0, 0), confidence=0.5)
    sm = VictimSubMission(victim=v)
    assert sm.investigate_angles == ()
    assert sm.dwell_until == 0.0


# LegacyDispatcher seam

def test_plan_returns_empty_without_strategy():
    """With no allocation strategy wired, plan() returns () (inert)
    rather than raising."""
    m = Mission()
    assert m.plan(None) == ()


def test_replan_no_op_for_unknown_drone():
    """A failed task for a drone not in the fleet makes replan() return
    () (inert)."""
    from drone_rescue_coordination.lib.domain.value_objects import OutgoingTask
    m = Mission()
    failed = OutgoingTask(
        drone_name='ghost', task_type=0, waypoints=(), target=None,
        victim_id=0, priority=0, hover_seconds=0.0,
    )
    assert m.replan(None, failed) == ()


def test_tick_no_op_when_mission_not_started():
    """With mission_start_sec None the decay + completion sub-tasks are
    skipped and tick() returns ()."""
    m = Mission()
    assert m.mission_start_sec is None
    assert m.tick(now_sec=42.0) == ()
