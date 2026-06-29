"""Pure-Python unit tests for the MissionViewModel reducer.

No rclpy spin-up; messages are stubbed via SimpleNamespace.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from drone_rescue_ui_common import (
    MissionViewModel, DroneViewState, VictimViewState, CoverageViewState,
    TASK_LABEL, SEVERITY_COLOR, SEVERITY_LABEL, DEFAULT_DRONE_NAMES,
)


# constants

def test_task_label_covers_all_six_task_types():
    """0..5 should all map to a human-readable label."""
    for tt in range(6):
        assert tt in TASK_LABEL
    assert TASK_LABEL[0] == 'SCAN'
    assert TASK_LABEL[5] == 'IDLE'


def test_severity_color_and_label_consistent_keys():
    assert set(SEVERITY_COLOR.keys()) == set(SEVERITY_LABEL.keys())
    assert SEVERITY_LABEL[0] == 'INFO'


def test_default_drone_names_is_4():
    assert len(DEFAULT_DRONE_NAMES) == 4
    assert DEFAULT_DRONE_NAMES == ['drone1', 'drone2', 'drone3', 'drone4']


# MissionViewModel

def _stub_pose(x, y, z=10.0):
    return SimpleNamespace(position=SimpleNamespace(x=float(x), y=float(y), z=float(z)))


def _stub_peer_state(name, *, battery=0.95, task_type=0, x=1.0, y=2.0,
                     is_down=False, wp_index=3, wp_total=45,
                     busy_with_victim=0):
    return SimpleNamespace(
        drone_name=name,
        battery=battery,
        task_type=task_type,
        is_down=is_down,
        busy_with_victim=busy_with_victim,
        wp_index=wp_index,
        wp_total=wp_total,
        pose=_stub_pose(x, y),
    )


def test_view_model_starts_empty():
    vm = MissionViewModel()
    assert vm.drones == {}
    assert vm.victims == {}
    assert vm.coverage.percentage == 0.0
    assert vm.log == ()


def test_view_model_apply_peer_state_creates_drone():
    vm = MissionViewModel()
    msg = _stub_peer_state('drone1', battery=0.8, task_type=1, x=12.0, y=-3.5)
    vm = vm.apply_peer_state(msg)
    assert 'drone1' in vm.drones
    assert vm.drones['drone1'].battery == pytest.approx(0.8)
    assert vm.drones['drone1'].task_type == 1
    assert vm.drones['drone1'].pose_x == pytest.approx(12.0)
    assert vm.drones['drone1'].pose_y == pytest.approx(-3.5)


def test_view_model_apply_peer_state_is_pure():
    """apply_peer_state returns a new view model; original unchanged."""
    vm = MissionViewModel()
    msg = _stub_peer_state('drone1')
    vm2 = vm.apply_peer_state(msg)
    assert vm.drones == {}
    assert 'drone1' in vm2.drones
    assert vm is not vm2


def test_view_model_apply_peer_state_updates_existing():
    vm = MissionViewModel()
    msg1 = _stub_peer_state('drone1', battery=0.9, x=0.0, y=0.0)
    msg2 = _stub_peer_state('drone1', battery=0.5, x=10.0, y=10.0)
    vm = vm.apply_peer_state(msg1)
    vm = vm.apply_peer_state(msg2)
    assert vm.drones['drone1'].battery == pytest.approx(0.5)
    assert vm.drones['drone1'].pose_x == pytest.approx(10.0)


def test_view_model_apply_health_updates_anomaly_only():
    vm = MissionViewModel()
    vm = vm.apply_peer_state(_stub_peer_state('drone1', battery=0.7))
    vm = vm.apply_health('drone1', anomaly_score=0.42)
    assert vm.drones['drone1'].anomaly_score == pytest.approx(0.42)
    # battery from peer_state not clobbered:
    assert vm.drones['drone1'].battery == pytest.approx(0.7)


def test_view_model_peer_age_infinity_for_unknown_drone():
    """peer_age returns +inf for a drone never seen."""
    vm = MissionViewModel()
    assert vm.peer_age('phantom', now=100.0) == float('inf')


def test_view_model_peer_age_paired_with_state():
    """apply_peer_state(now=t) captures the timestamp atomically;
    peer_age returns now - that timestamp."""
    vm = MissionViewModel()
    vm = vm.apply_peer_state(_stub_peer_state('drone1'), now=100.0)
    assert vm.peer_age('drone1', now=105.0) == pytest.approx(5.0)
    # Subsequent apply with a newer `now` advances the timestamp.
    vm = vm.apply_peer_state(_stub_peer_state('drone1'), now=110.0)
    assert vm.peer_age('drone1', now=110.5) == pytest.approx(0.5)


def test_view_model_health_age_paired_with_anomaly():
    vm = MissionViewModel()
    vm = vm.apply_peer_state(_stub_peer_state('drone2'))
    vm = vm.apply_health('drone2', anomaly_score=0.6, now=42.0)
    assert vm.health_age('drone2', now=45.0) == pytest.approx(3.0)


def test_view_model_age_returns_infinity_when_no_timestamp():
    """A peer_state landed without `now` (legacy caller path) leaves
    peer_last_seen at 0; reader sees infinity, which the
    stale-detection logic treats as 'never seen'."""
    vm = MissionViewModel()
    vm = vm.apply_peer_state(_stub_peer_state('drone3'))   # no `now=`
    assert vm.peer_age('drone3', now=100.0) == float('inf')


def test_view_model_apply_victim_candidate():
    vm = MissionViewModel()
    msg = SimpleNamespace(
        candidate_id=42,
        position=SimpleNamespace(x=44.0, y=38.0),
        confidence=0.92,
        confirmed=True,
        reporting_drones=['drone1', 'drone2'],
    )
    vm = vm.apply_victim_candidate(msg)
    assert 42 in vm.victims
    assert vm.victims[42].confidence == pytest.approx(0.92)
    assert vm.victims[42].confirmed is True
    assert vm.victims[42].reporting_drones == ('drone1', 'drone2')


def test_view_model_apply_coverage():
    vm = MissionViewModel()
    msg = SimpleNamespace(
        percentage_covered=42.5,
        cells_visited=850,
        elapsed_time_seconds=120.0,
    )
    vm = vm.apply_coverage(msg)
    assert vm.coverage.percentage == pytest.approx(42.5)
    assert vm.coverage.cells_visited == 850


def test_view_model_apply_coverage_includes_viz_overlay_fields():
    """apply_coverage folds drones_surveying and victims_found so the
    viz overlay nodes can render via render_from(view) instead of
    holding the raw ROS message."""
    vm = MissionViewModel()
    msg = SimpleNamespace(
        percentage_covered=42.5,
        cells_visited=850,
        total_cells=2000,
        elapsed_time_seconds=120.0,
        drones_surveying=3,
        victims_found=2,
    )
    vm = vm.apply_coverage(msg)
    assert vm.coverage.drones_surveying == 3
    assert vm.coverage.victims_found == 2


def test_view_model_apply_coverage_carries_eta():
    """estimated_time_remaining is published by coverage_tracker on
    /coverage/metrics but was dropped by the view model, so no widget could
    render it. The reducer now folds it through."""
    vm = MissionViewModel()
    msg = SimpleNamespace(
        percentage_covered=42.5,
        cells_visited=850,
        elapsed_time_seconds=120.0,
        estimated_time_remaining=180.0,
    )
    vm = vm.apply_coverage(msg)
    assert vm.coverage.estimated_time_remaining == pytest.approx(180.0)


def test_view_model_apply_coverage_eta_defaults_zero():
    """A message without the field (older bag / partial msg) folds to 0.0."""
    vm = MissionViewModel().apply_coverage(SimpleNamespace(
        percentage_covered=10.0, cells_visited=10, elapsed_time_seconds=5.0))
    assert vm.coverage.estimated_time_remaining == 0.0


def test_view_model_apply_drone_status_folds_controller_state_and_battery():
    """apply_drone_status reducer folds the DroneStatus stream (distinct
    from peer_state) into the typed projection so coverage_visualizer +
    telemetry_overlay can render via render_from(view)."""
    vm = MissionViewModel()
    status = SimpleNamespace(state=2, battery_level=0.85)   # SURVEYING
    vm = vm.apply_drone_status('drone1', status, now=10.0)
    d = vm.drones['drone1']
    assert d.controller_state == 2
    assert d.battery == pytest.approx(0.85)
    assert d.peer_last_seen == pytest.approx(10.0)
    # Subsequent fold preserves prior peer_state fields.
    status2 = SimpleNamespace(state=3, battery_level=0.70)   # RETURNING
    vm = vm.apply_drone_status('drone1', status2, now=15.0)
    d = vm.drones['drone1']
    assert d.controller_state == 3
    assert d.battery == pytest.approx(0.70)
    assert d.peer_last_seen == pytest.approx(15.0)


def test_view_model_log_is_bounded():
    vm = MissionViewModel()
    for i in range(250):
        vm = vm.append_event(f'event-{i}', max_log=200)
    assert len(vm.log) == 200
    # Newest events kept; oldest dropped
    assert vm.log[-1] == 'event-249'
    assert vm.log[0] == 'event-50'


def test_view_model_dataclasses_are_frozen():
    vm = MissionViewModel()
    with pytest.raises(Exception):
        vm.coverage = CoverageViewState(percentage=99.0)   # type: ignore[misc]


# confirmed count

def test_confirmed_victim_count_folds_both_channels():
    """Single definition of 'confirmed victim count'; widgets must not
    reimplement the fold."""
    vm = MissionViewModel()
    assert vm.confirmed_victim_count == 0
    cand = SimpleNamespace(
        candidate_id=1,
        position=SimpleNamespace(x=0.0, y=0.0),
        confidence=0.9, confirmed=True, reporting_drones=['drone1'],
    )
    vm = vm.apply_victim_candidate(cand)
    assert vm.confirmed_victim_count == 1
    # saga-only confirmation counts too
    vm = vm.apply_saga_confirmed(2)
    assert vm.confirmed_victim_count == 2
    # unconfirmed candidates don't count
    cand3 = SimpleNamespace(
        candidate_id=3,
        position=SimpleNamespace(x=1.0, y=1.0),
        confidence=0.3, confirmed=False, reporting_drones=['drone2'],
    )
    vm = vm.apply_victim_candidate(cand3)
    assert vm.confirmed_victim_count == 2


# mission state

def test_apply_mission_state_folds_status_and_tallies():
    """/mission/state drives the phase strip."""
    vm = MissionViewModel()
    assert vm.mission.status == 0           # INIT default
    assert vm.mission.received is False
    msg = SimpleNamespace(
        status=3, sectors_total=12, sectors_completed=4,
        victims_found=2, victims_confirmed=1,
        active_tasks_summary=['drone1: SCAN(7/24)'],
    )
    vm = vm.apply_mission_state(msg)
    assert vm.mission.received is True
    assert vm.mission.status == 3
    assert vm.mission.sectors_total == 12
    assert vm.mission.sectors_completed == 4
    assert vm.mission.victims_confirmed == 1
    assert vm.mission.active_tasks_summary == ('drone1: SCAN(7/24)',)


# drone status fold

def test_drone_status_derivation_matches_state_table_semantics():
    """Single source for the OK/WARN/DOWN/OFFLINE/... labels the state
    table and the fleet rail both render."""
    from drone_rescue_ui_common.view_model import drone_status

    vm = MissionViewModel()
    # never reported
    label, sev = drone_status(vm, 'drone1', now=100.0)
    assert label == '—' and sev == 'muted'

    peer = SimpleNamespace(
        drone_name='drone1', battery=0.8, task_type=0, is_down=False,
        busy_with_victim=0, wp_index=1, wp_total=4,
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=5.0),
            orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
        ),
    )
    vm = vm.apply_peer_state(peer, now=100.0)
    vm = vm.apply_health(
        'drone1', 0.1, now=100.0, reason='', unrecoverable=False,
    )
    assert drone_status(vm, 'drone1', now=101.0) == ('OK', 'ok')
    # anomaly over threshold
    vm2 = vm.apply_health('drone1', 0.5, now=100.0)
    assert drone_status(vm2, 'drone1', now=101.0) == ('WARN', 'warn')
    # peer silent, health alive -> EXEC DOWN
    assert drone_status(vm, 'drone1', now=102.0 + 4.0) == ('OFFLINE', 'error')
    vm3 = vm.apply_health('drone1', 0.1, now=105.5)
    assert drone_status(vm3, 'drone1', now=106.0) == ('EXEC DOWN', 'error')
    # unrecoverable wins
    vm4 = vm.apply_health('drone1', 0.9, now=100.0, unrecoverable=True)
    assert drone_status(vm4, 'drone1', now=101.0) == ('DOWN', 'error')
