"""Unit coverage for the lifted MissionCollector.

The collector is pure Python; these tests exercise it without
rclpy.init(). They verify the parallel-list series, the
all-zero-pose skip, and the cumulative-confirmed accumulator.
"""

from drone_rescue_mission_control.lib.mission_collector import MissionCollector


class _FakeCandidate:
    """Stand-in for VictimCandidate: only ``.confirmed`` and
    ``.position.x/.y`` are touched by the finaliser."""

    def __init__(self, confirmed: bool, x: float = 0.0, y: float = 0.0):
        self.confirmed = confirmed
        class _P:
            pass
        self.position = _P()
        self.position.x = x
        self.position.y = y


def test_record_event_appends_to_events_list():
    c = MissionCollector(['drone1'])
    c.record_event(1.0, {'t': 1.0, 'type': 'MISSION_START'})
    c.record_event(2.0, {'t': 2.0, 'type': 'CANDIDATE_DETECTED'})
    assert len(c.events) == 2
    assert c.events[0]['type'] == 'MISSION_START'


def test_record_coverage_pushes_both_series():
    c = MissionCollector(['drone1'])
    c.record_coverage(0.5, percentage_covered=12.5, victims_found=3)
    assert c.coverage_pct == [(0.5, 12.5)]
    assert c.candidates_count == [(0.5, 3)]


def test_record_victim_tracks_unique_confirmed_count():
    c = MissionCollector(['drone1'])
    # First two are confirmed, third candidate id 1 again still counts once.
    c.record_victim(1.0, 1, _FakeCandidate(confirmed=True))
    c.record_victim(2.0, 2, _FakeCandidate(confirmed=True))
    c.record_victim(3.0, 1, _FakeCandidate(confirmed=True))  # same id 1
    assert c.cumulative_confirmed == [(1.0, 1), (2.0, 2), (3.0, 2)]


def test_record_victim_unconfirmed_does_not_bump_count():
    c = MissionCollector(['drone1'])
    c.record_victim(1.0, 1, _FakeCandidate(confirmed=False))
    c.record_victim(2.0, 2, _FakeCandidate(confirmed=True))
    assert c.cumulative_confirmed == [(1.0, 0), (2.0, 1)]


def test_record_peer_skips_all_zero_first_pose():
    c = MissionCollector(['drone1'])
    c.record_peer(
        0.1, 'drone1',
        battery=1.0, task_type=0, wp_index=0, wp_total=10,
        pose_x=0.0, pose_y=0.0,
    )
    # All-zero first pose: skip the position append (matches pre-extraction).
    assert c.drone_series['drone1'].position == []
    # But battery / task / wp_index still append.
    assert c.drone_series['drone1'].battery == [(0.1, 1.0)]


def test_record_peer_captures_nonzero_pose_and_subsequent_zeros():
    c = MissionCollector(['drone1'])
    c.record_peer(
        0.1, 'drone1',
        battery=1.0, task_type=0, wp_index=0, wp_total=10,
        pose_x=5.0, pose_y=3.0,
    )
    # After a real pose was captured, a (0, 0) still appends.
    c.record_peer(
        0.2, 'drone1',
        battery=0.9, task_type=1, wp_index=1, wp_total=10,
        pose_x=0.0, pose_y=0.0,
    )
    assert c.drone_series['drone1'].position == [
        (0.1, 5.0, 3.0), (0.2, 0.0, 0.0),
    ]


def test_record_peer_unknown_drone_is_no_op():
    c = MissionCollector(['drone1'])
    c.record_peer(
        0.1, 'phantom',
        battery=1.0, task_type=0, wp_index=0, wp_total=10,
        pose_x=1.0, pose_y=1.0,
    )
    assert c.drone_series['drone1'].battery == []


def test_record_health_appends_anomaly_score():
    c = MissionCollector(['drone1'])
    c.record_health(1.0, 'drone1', anomaly_score=0.42)
    c.record_health(2.0, 'drone1', anomaly_score=0.55)
    assert c.drone_series['drone1'].anomaly == [(1.0, 0.42), (2.0, 0.55)]


def test_per_drone_to_dict_round_trip():
    c = MissionCollector(['drone1', 'drone2'])
    c.record_peer(
        1.0, 'drone1',
        battery=0.9, task_type=2, wp_index=3, wp_total=8,
        pose_x=4.0, pose_y=2.0,
    )
    out = c.per_drone_to_dict()
    assert set(out.keys()) == {'drone1', 'drone2'}
    assert out['drone1']['battery'] == [(1.0, 0.9)]
    assert out['drone1']['wp_total'] == 8
    assert out['drone2']['battery'] == []
