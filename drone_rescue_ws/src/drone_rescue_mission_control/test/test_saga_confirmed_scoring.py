"""Tests for saga-confirmation scoring.

Every recorded run scored true_positives=0 because the recorder scored TP from
the transient /victims/candidates.confirmed flag (set only by the detection_filter
multi-view gate, which sector-scanning rarely satisfies) instead of the saga's
VICTIM_CONFIRMED events. The saga genuinely confirms victims (with positions);
saga_confirmed_positions surfaces that set so TP is scored against ground truth.
This is the regression test for the falsified-headline fix.
"""

from __future__ import annotations

from drone_rescue_mission_control.lib.run_finaliser import (
    saga_confirmed_positions,
    score,
)


def _vc(vid, x, y):
    return {'type': 'VICTIM_CONFIRMED', 'victim_id': vid, 'position': [x, y, 0.0]}


def test_extracts_victim_confirmed_events_with_positions():
    events = [
        {'type': 'CANDIDATE_DETECTED', 'victim_id': 1, 'position': [9, 9, 0]},
        _vc(4, -17.9, 15.7),
        _vc(5, -10.7, 19.4),
        {'type': 'SCANNING_STARTED', 'victim_id': 0, 'position': [0, 0, 0]},
    ]
    sc = saga_confirmed_positions(events)
    assert sc == {4: (-17.9, 15.7), 5: (-10.7, 19.4)}


def test_latest_event_wins_per_victim():
    sc = saga_confirmed_positions([_vc(3, 1.0, 1.0), _vc(3, 2.0, 2.0)])
    assert sc == {3: (2.0, 2.0)}


def test_empty_when_no_confirmations():
    assert saga_confirmed_positions([{'type': 'CANDIDATE_DETECTED',
                                      'victim_id': 1, 'position': [0, 0, 0]}]) == {}


def test_saga_confirmations_score_a_true_positive():
    """The real clean-run shape: the saga confirms victim 3 (within 8 m) plus
    near-pad false positives. TP must be > 0; the FPs are surfaced, not hidden."""
    gt = [(1, (44.0, 38.0)), (2, (32.0, -28.0)), (3, (-12.0, 16.0)),
          (4, (55.0, 48.0)), (5, (-8.0, 34.0))]
    events = [
        _vc(1, -0.2, -1.4),   # near pad -> FP
        _vc(2, 2.8, 2.5),     # near pad -> FP
        _vc(4, -17.9, 15.7),  # 6.0 m from victim 3 -> the duplicate (farther)
        _vc(5, -10.7, 19.4),  # 3.7 m from victim 3 -> the TP (closest pair)
    ]
    sc = saga_confirmed_positions(events)
    tp, fp, fn = score(list(sc.items()), gt, 8.0)
    assert len(sc) == 4                  # victims_confirmed (saga)
    assert len(tp) == 1                  # ground-truth matched (victim 3)
    assert tp[0]['gt_id'] == 3
    # Nearest-pair: the CLOSEST confirmation (id 5, 3.7 m) is the true
    # positive, not whichever VICTIM_CONFIRMED happened to arrive first
    # (id 4, 6.0 m). Order-independent matching is the fix for genuinely-close
    # confirmations being scored FP just because of event ordering.
    assert tp[0]['candidate_id'] == 5
    assert tp[0]['distance_m'] < 4.0
    assert len(fp) == 3                  # 2 near-pad + the farther duplicate (id 4)
    assert 4 in fp and 5 not in fp
