"""Unit tests for lib/run_finaliser helpers.

Pure-Python; previously reachable only through the MissionRecorder
LifecycleNode (needed rclpy.init()).
"""

from __future__ import annotations

from drone_rescue_mission_control.lib.run_finaliser import (
    compute_detection_latency,
    first_crossing,
    fold_events,
    integrate_active_time,
    score,
)


# single-fold replaces 5 separate filtered walks in
# mission_recorder._finalize. These tests pin the contract.
def test_fold_events_empty_returns_defaults():
    r = fold_events([])
    assert r.first_detection_t is None
    assert r.first_confirm_t is None
    assert r.drone_down_events == ()
    assert r.sector_reassignments == 0
    assert r.rejected == 0
    assert r.confirm_t_by_id == {}


def test_fold_events_returns_first_detection_only():
    events = [
        {'type': 'CANDIDATE_DETECTED', 't': 5.0, 'victim_id': 1},
        {'type': 'CANDIDATE_DETECTED', 't': 7.5, 'victim_id': 2},
    ]
    r = fold_events(events)
    assert r.first_detection_t == 5.0


def test_fold_events_records_first_confirm_per_id():
    events = [
        {'type': 'VICTIM_CONFIRMED', 't': 10.0, 'victim_id': 1, 'drone': '',
         'detail': ''},
        {'type': 'VICTIM_CONFIRMED', 't': 12.0, 'victim_id': 2, 'drone': '',
         'detail': ''},
        {'type': 'VICTIM_CONFIRMED', 't': 15.0, 'victim_id': 1, 'drone': '',
         'detail': ''},   # duplicate id; ignore
    ]
    r = fold_events(events)
    assert r.first_confirm_t == 10.0
    assert r.confirm_t_by_id == {1: 10.0, 2: 12.0}


def test_fold_events_drone_down_carries_drone_name_and_reason():
    events = [
        {'type': 'DRONE_DOWN', 't': 20.0,
         'drone': 'drone3', 'detail': 'lidar fault'},
    ]
    r = fold_events(events)
    assert r.drone_down_events == (
        {'drone': 'drone3', 't_s': 20.0, 'reason': 'lidar fault'},
    )


def test_fold_events_counts_sector_reassignments_and_rejected():
    events = [
        {'type': 'SECTOR_REASSIGNED', 't': 1.0},
        {'type': 'SECTOR_REASSIGNED', 't': 2.0},
        {'type': 'CANDIDATE_REJECTED', 't': 3.0},
    ]
    r = fold_events(events)
    assert r.sector_reassignments == 2
    assert r.rejected == 1


def test_fold_events_ignores_unrelated_types():
    events = [
        {'type': 'SCANNING_STARTED', 't': 0.0},
        {'type': 'MISSION_COMPLETE', 't': 120.0},
    ]
    r = fold_events(events)
    assert r.first_detection_t is None
    assert r.first_confirm_t is None
    assert r.sector_reassignments == 0


def test_first_crossing_returns_first_match():
    series = [(0.0, 10.0), (1.0, 25.0), (2.0, 60.0), (3.0, 85.0)]
    assert first_crossing(series, 50.0) == 2.0


def test_first_crossing_never_crosses():
    series = [(0.0, 1.0), (1.0, 2.0)]
    assert first_crossing(series, 100.0) is None


def test_first_crossing_at_threshold():
    """Exact equality counts as crossed."""
    series = [(5.0, 80.0)]
    assert first_crossing(series, 80.0) == 5.0


def test_integrate_active_time_simple():
    """Drone in non-idle from t=0 to t=10."""
    task_series = [(0.0, 1), (10.0, 5)]   # 1=non-idle, 5=idle
    assert integrate_active_time(task_series, idle_task_type=5) == 10.0


def test_integrate_active_time_mixed():
    task_series = [(0.0, 1), (3.0, 5), (7.0, 1), (12.0, 5)]
    # 1 from 0..3 (3 sec) + 1 from 7..12 (5 sec) = 8.
    assert integrate_active_time(task_series, idle_task_type=5) == 8.0


def test_integrate_active_time_empty():
    assert integrate_active_time([], idle_task_type=5) == 0.0


def test_integrate_active_time_single_sample():
    """No interval to integrate; returns 0.0."""
    assert integrate_active_time([(0.0, 1)], idle_task_type=5) == 0.0


def test_score_perfect_match():
    """All confirmed are within radius of distinct GT."""
    confirmed = [(1, (0.0, 0.0)), (2, (10.0, 10.0))]
    gt = [(100, (0.1, 0.1)), (101, (10.1, 10.0))]
    tp, fp, fn = score(confirmed, gt, radius_m=2.0)
    assert len(tp) == 2
    assert fp == []
    assert fn == []


def test_score_false_positive():
    """Confirmed not near any GT → FP."""
    confirmed = [(1, (1000.0, 1000.0))]
    gt = [(100, (0.0, 0.0))]
    tp, fp, fn = score(confirmed, gt, radius_m=2.0)
    assert tp == []
    assert fp == [1]
    assert fn == [100]


def test_score_nearest_pair_binds_closest_not_emit_order():
    """Two confirmed near one GT: the CLOSEST binds (order-independent), not
    whichever VICTIM_CONFIRMED arrived first. Here id 2 is the exact match."""
    confirmed = [(1, (0.1, 0.0)), (2, (0.0, 0.0))]
    gt = [(100, (0.0, 0.0))]
    tp, fp, fn = score(confirmed, gt, radius_m=1.0)
    assert len(tp) == 1
    assert tp[0]['candidate_id'] == 2   # nearest (0.0 m) wins, not emit-order
    assert fp == [1]
    assert fn == []


def test_compute_detection_latency_simple():
    """One TP pair with one drone pass-by."""
    tp_pairs = [{'candidate_id': 1, 'gt_id': 100}]
    confirm_t_by_id = {1: 50.0}
    drone_positions = {'drone1': [(10.0, 0.0, 0.0), (20.0, 50.0, 0.0), (30.0, 100.0, 0.0)]}
    gt_pos_by_id = {100: (50.0, 0.0)}
    latencies = compute_detection_latency(
        tp_pairs=tp_pairs,
        confirm_t_by_id=confirm_t_by_id,
        drone_positions_by_drone=drone_positions,
        gt_pos_by_id=gt_pos_by_id,
        gt_match_radius_m=5.0,
    )
    # Earliest pass-by: t=20 (the (50, 0) sample is at (50, 0) gt).
    # Confirm at t=50. Latency = 30.
    assert latencies == [30.0]


def test_compute_detection_latency_skips_missing():
    """Pair with no recorded drone pass-by is dropped (numeric only)."""
    tp_pairs = [{'candidate_id': 1, 'gt_id': 100}]
    confirm_t_by_id = {1: 50.0}
    # No drone ever passed near (50, 0).
    drone_positions = {'drone1': [(10.0, 0.0, 0.0), (20.0, 200.0, 0.0)]}
    gt_pos_by_id = {100: (50.0, 0.0)}
    latencies = compute_detection_latency(
        tp_pairs=tp_pairs,
        confirm_t_by_id=confirm_t_by_id,
        drone_positions_by_drone=drone_positions,
        gt_pos_by_id=gt_pos_by_id,
        gt_match_radius_m=5.0,
    )
    assert latencies == []
