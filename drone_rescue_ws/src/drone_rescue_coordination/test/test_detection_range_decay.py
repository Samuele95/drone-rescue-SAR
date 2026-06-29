"""Regression tests for the detection-range confidence decay.

The bug it closes: the area-based confidence in ``victim_detector``
treated pixel count as an absolute quality signal. With the 25 m
survey altitude + 90° camera FOV, the ground footprint is 50 m wide,
so a victim at the EDGE of the footprint produced the same area-conf
as one directly below the drone. The saga's auto-confirm gate
(``confirmation_threshold=0.80``) tripped on detections from up to
25 m horizontal range, meaning drones "saw" and confirmed victims
without ever needing to fly close to them, contradicting the realism
the thesis claims.

``_apply_range_decay`` decays the area-based confidence by the
horizontal XY distance between the drone and the projected victim
position:
  xy <= range_decay_start_m  → no decay (full area_conf)
  start < xy < max_range     → linear decay to 0
  xy >= max_range            → drop entirely (return None)

These tests pin the decay function across the full piecewise shape
and the two degenerate configurations.

Pure pytest; no rclpy.init(); we exercise the unbound method.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from drone_rescue_coordination.victim_detector import VictimDetector


# fixtures

def _node(*, start: float = 5.0, max_range: float = 12.0):
    """Minimal duck-typed surface for ``_apply_range_decay``."""
    return SimpleNamespace(
        range_decay_start_m=start,
        max_detection_range_m=max_range,
    )


def _point(x: float, y: float, z: float = 0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def _pose(x: float, y: float, z: float = 25.0):
    return SimpleNamespace(x=x, y=y, z=z, yaw=0.0)


# full-confidence band

def test_at_origin_no_decay():
    """Drone directly above the victim (xy=0) → full confidence."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=5.0, max_range=12.0)
    conf = helper(ns, 0.85, _point(10.0, 10.0), _pose(10.0, 10.0))
    assert conf == pytest.approx(0.85)


def test_inside_start_radius_no_decay():
    """Any xy <= range_decay_start gets the full base confidence."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=5.0, max_range=12.0)
    for offset in (0.0, 1.0, 3.0, 4.99):
        conf = helper(ns, 0.85, _point(offset, 0.0), _pose(0.0, 0.0))
        assert conf == pytest.approx(0.85), (
            f'xy={offset} should be in full-confidence band'
        )


def test_at_exactly_start_radius_no_decay():
    """The start boundary itself is INSIDE the no-decay band."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=5.0, max_range=12.0)
    conf = helper(ns, 0.85, _point(5.0, 0.0), _pose(0.0, 0.0))
    assert conf == pytest.approx(0.85)


# decay band

def test_midway_decay_is_half():
    """Exactly halfway between start (5) and max (12) → 0.5 * base.
    Decay band = 7 m wide; midway is xy = 5 + 3.5 = 8.5."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=5.0, max_range=12.0)
    conf = helper(ns, 0.80, _point(8.5, 0.0), _pose(0.0, 0.0))
    assert conf == pytest.approx(0.40, abs=1e-9)


def test_decay_monotone_falling():
    """Inside the decay band, confidence strictly decreases with xy."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=5.0, max_range=12.0)
    confs = []
    for xy in (5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 11.99):
        c = helper(ns, 0.90, _point(xy, 0.0), _pose(0.0, 0.0))
        confs.append(c)
    # First two equal (5.0 is start, no decay); from 6.0 onward strictly
    # less than the previous.
    assert confs[0] == pytest.approx(0.90)
    for i in range(1, len(confs)):
        assert confs[i] < confs[i - 1], (
            f'expected decay between xy={5+i-1} and xy={5+i}, '
            f'got {confs[i-1]} then {confs[i]}'
        )


def test_just_inside_max_range_returns_near_zero():
    """As xy approaches max_range from below, confidence approaches 0."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=5.0, max_range=12.0)
    conf = helper(ns, 0.99, _point(11.99, 0.0), _pose(0.0, 0.0))
    assert conf is not None
    assert conf < 0.01


# drop band

def test_at_max_range_drops():
    """xy == max_range → drop (return None)."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=5.0, max_range=12.0)
    assert helper(ns, 0.99, _point(12.0, 0.0), _pose(0.0, 0.0)) is None


def test_beyond_max_range_drops():
    """Anything beyond max_range → drop."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=5.0, max_range=12.0)
    for xy in (12.5, 15.0, 25.0, 100.0):
        assert helper(ns, 0.99, _point(xy, 0.0), _pose(0.0, 0.0)) is None


# distance is 2-D only

def test_altitude_not_included_in_decay():
    """The decay is HORIZONTAL only: altitude (z) doesn't affect the
    returned confidence even though the camera sees from above. This is
    the design choice (see _apply_range_decay docstring): the drone's
    altitude is roughly constant, so the variable that matters is XY
    proximity."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=5.0, max_range=12.0)
    # Same xy distance (3 m horizontal), but drone at z=25 vs z=5.
    high = helper(ns, 0.85, _point(3.0, 0.0), _pose(0.0, 0.0, z=25.0))
    low = helper(ns, 0.85, _point(3.0, 0.0), _pose(0.0, 0.0, z=5.0))
    assert high == low == pytest.approx(0.85)


def test_diagonal_distance_computed_correctly():
    """xy = sqrt(dx² + dy²); both axes contribute."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=5.0, max_range=12.0)
    # dx=4, dy=3 → xy=5 (exact boundary, full conf).
    conf = helper(ns, 0.90, _point(4.0, 3.0), _pose(0.0, 0.0))
    assert conf == pytest.approx(0.90)
    # dx=6, dy=8 → xy=10 (in decay band, expect 0.90 * (1 - 5/7) ≈ 0.257).
    conf = helper(ns, 0.90, _point(6.0, 8.0), _pose(0.0, 0.0))
    expected = 0.90 * (1.0 - 5.0 / 7.0)
    assert conf == pytest.approx(expected, abs=1e-9)


# degenerate configs

def test_degenerate_start_equals_max_is_cliff():
    """If a user (mis)configures start == max, the decay band has zero
    width. The helper treats it as a hard cliff: inside the boundary,
    full confidence; at-or-beyond, drop. The boundary itself returns
    None because xy >= max_range triggers the drop branch first."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=10.0, max_range=10.0)
    assert helper(ns, 0.85, _point(9.99, 0.0), _pose(0.0, 0.0)) == pytest.approx(0.85)
    assert helper(ns, 0.85, _point(10.0, 0.0), _pose(0.0, 0.0)) is None
    assert helper(ns, 0.85, _point(10.5, 0.0), _pose(0.0, 0.0)) is None


def test_degenerate_start_greater_than_max_is_safe():
    """A misconfiguration with start > max is degenerate but must NOT
    crash; the divide-by-negative is guarded. Inside max, full conf;
    beyond max, drop."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=15.0, max_range=10.0)
    # Inside the (smaller) max range: full conf (the start guard fires
    # for xy <= start, but xy < max < start so xy <= start is true).
    assert helper(ns, 0.85, _point(5.0, 0.0), _pose(0.0, 0.0)) == pytest.approx(0.85)
    assert helper(ns, 0.85, _point(9.0, 0.0), _pose(0.0, 0.0)) == pytest.approx(0.85)
    # Beyond max: drop.
    assert helper(ns, 0.85, _point(11.0, 0.0), _pose(0.0, 0.0)) is None


# realism scenario

def test_realistic_default_kills_long_range_auto_confirm():
    """Concrete scenario from the live-run analysis: at 25 m altitude
    with 90° FOV, the ground footprint is ~25 m radius. With the
    default decay (5 / 12 m), a detection at the EDGE of the camera
    footprint (xy = 20 m) is dropped; a detection from a sector away
    (xy = 8 m) is decayed to ~0.43 of base; only an overhead detection
    (xy <= 5 m) keeps full area confidence."""
    helper = VictimDetector._apply_range_decay
    ns = _node(start=5.0, max_range=12.0)
    # Edge of camera footprint: previously auto-confirmed at 0.85.
    assert helper(ns, 0.85, _point(20.0, 0.0), _pose(0.0, 0.0)) is None
    # Mid-range: previously 0.85, now decayed.
    mid = helper(ns, 0.85, _point(8.0, 0.0), _pose(0.0, 0.0))
    assert mid is not None
    assert mid < 0.5
    # Overhead: still confident.
    over = helper(ns, 0.85, _point(2.0, 0.0), _pose(0.0, 0.0))
    assert over == pytest.approx(0.85)
