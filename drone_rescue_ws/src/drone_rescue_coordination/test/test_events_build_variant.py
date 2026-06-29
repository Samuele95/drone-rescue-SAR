"""Tests for `lib.domain.events.build_variant`.

The helper is the single dispatch table the mission_manager facade
delegates through; ensures the legacy 13-branch if/elif chain is
not needed and that field-filtering drops irrelevant kwargs.
"""

from __future__ import annotations

from drone_rescue_coordination.lib.domain.events import (
    BatteryRTH, CandidateDetected, CandidateRejected,
    ConfirmDispatched, DroneDown, InvestigateDispatched,
    MissionComplete, UnknownEvent, VictimConfirmed,
    build_variant,
)


def test_candidate_detected_full_fields():
    v = build_variant(
        'CANDIDATE_DETECTED',
        severity=0, raw_detail='x', drone_name='drone1',
        victim_id=7, position=(1.0, 2.0, 0.0), confidence=0.9,
    )
    assert isinstance(v, CandidateDetected)
    assert v.victim_id == 7
    assert v.position == (1.0, 2.0, 0.0)
    assert v.confidence == 0.9


def test_position_from_point_like_object_normalises():
    """The mission_manager facade often passes geometry_msgs.msg.Point,
    duck-typed; build_variant normalises to a (x, y, z) tuple."""
    class P:
        x, y, z = 3.0, 4.0, 5.0
    v = build_variant(
        'VICTIM_CONFIRMED', victim_id=42, position=P(),
    )
    assert isinstance(v, VictimConfirmed)
    assert v.position == (3.0, 4.0, 5.0)


def test_irrelevant_kwargs_silently_dropped():
    """MissionComplete has no victim_id/position/confidence: passing
    them should NOT raise (legacy if/elif chain depended on this
    behaviour)."""
    v = build_variant(
        'MISSION_COMPLETE',
        raw_detail='ok', severity=0, drone_name='',
        victim_id=99, position=(1.0, 2.0, 3.0), confidence=0.5,
    )
    assert isinstance(v, MissionComplete)


def test_unknown_event_type_yields_unknown_event():
    v = build_variant(
        'TOTALLY_NEW_EVENT', raw_detail='hi', severity=1, drone_name='d3',
    )
    assert isinstance(v, UnknownEvent)
    assert v.event_type == 'TOTALLY_NEW_EVENT'
    assert v.raw_detail == 'hi'


def test_candidate_rejected_keeps_victim_id_only():
    v = build_variant('CANDIDATE_REJECTED', victim_id=11, position=(1.0, 2.0, 3.0))
    assert isinstance(v, CandidateRejected)
    assert v.victim_id == 11


def test_pure_base_event_just_carries_header():
    v = build_variant('BATTERY_RTH', severity=1, raw_detail='low', drone_name='d2')
    assert isinstance(v, BatteryRTH)
    assert v.drone_name == 'd2'

    v2 = build_variant('DRONE_DOWN', severity=2, drone_name='d4')
    assert isinstance(v2, DroneDown)
    assert v2.drone_name == 'd4'


def test_investigate_and_confirm_dispatched_carry_victim_position():
    v = build_variant(
        'INVESTIGATE_DISPATCHED', victim_id=3, position=(7.0, 8.0, 9.0),
    )
    assert isinstance(v, InvestigateDispatched)
    assert v.victim_id == 3
    assert v.position == (7.0, 8.0, 9.0)

    v2 = build_variant('CONFIRM_DISPATCHED', victim_id=4, position=(1.0, 2.0, 3.0))
    assert isinstance(v2, ConfirmDispatched)
    assert v2.position == (1.0, 2.0, 3.0)


def test_known_event_types_set_matches_decoder():
    """The canonical KNOWN_EVENT_TYPES set is derived from the dispatch
    table, so every known type builds a typed (non-Unknown) variant."""
    from drone_rescue_coordination.lib.domain.events import (
        KNOWN_EVENT_TYPES, UnknownEvent, build_variant, is_known_event_type,
    )
    assert 'VICTIM_CONFIRMED' in KNOWN_EVENT_TYPES
    for et in KNOWN_EVENT_TYPES:
        assert is_known_event_type(et)
        assert not isinstance(build_variant(et), UnknownEvent)


def test_unknown_event_type_is_flagged_and_degrades():
    from drone_rescue_coordination.lib.domain.events import (
        UnknownEvent, build_variant, is_known_event_type,
    )
    assert is_known_event_type('FLIGHT_PLAN_INFEASIBLE') is False
    assert isinstance(build_variant('FLIGHT_PLAN_INFEASIBLE'), UnknownEvent)
