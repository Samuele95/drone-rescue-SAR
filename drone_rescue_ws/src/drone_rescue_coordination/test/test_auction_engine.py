"""Standalone tests for `lib/auction.py` and `lib/sector_geometry.py`.

These exercise the extracted modules with synthetic
SimpleNamespace bidders, no rclpy.init() required, so the suite runs
in <100 ms and is suitable for IDE inner-loop iteration.

The existing `test_auction.py` keeps validating the integration through
the full MissionManager class; this file validates the pure modules in
isolation.
"""

from __future__ import annotations

import math
import random
from types import SimpleNamespace

import pytest

from drone_rescue_coordination.lib.auction import (
    AuctionEngine, TaskType, Bid,
)
from drone_rescue_coordination.lib.sector_geometry import sector_owner_for


def _drone(name, x, y, *, battery_ok=True, is_down=False,
           current_task_type=0, busy_with_victim=None):
    return SimpleNamespace(
        name=name,
        pose=SimpleNamespace(x=float(x), y=float(y), z=10.0),
        battery_ok=battery_ok,
        is_down=is_down,
        current_task_type=int(current_task_type),
        busy_with_victim=busy_with_victim,
    )


def _target(x, y):
    return SimpleNamespace(x=float(x), y=float(y))


# ------------------------------------------------------------ AuctionEngine

@pytest.fixture
def engine():
    drones = {
        'd1': _drone('d1', 10, 0),
        'd2': _drone('d2', 0, 10),
        'd3': _drone('d3', -10, 0),
        'd4': _drone('d4', 0, -10),
    }
    return AuctionEngine(drones, random.Random(42)), drones


def test_bid_picks_closest(engine):
    eng, _ = engine
    assert eng.bid(_target(20, 0), priority=2) == 'd1'


def test_bid_excludes_set(engine):
    eng, _ = engine
    assert eng.bid(_target(20, 0), priority=2, exclude={'d1'}) != 'd1'


def test_bid_skips_battery_low(engine):
    eng, drones = engine
    drones['d1'].battery_ok = False
    assert eng.bid(_target(20, 0), priority=2) != 'd1'


def test_bid_skips_is_down(engine):
    eng, drones = engine
    drones['d1'].is_down = True
    assert eng.bid(_target(20, 0), priority=2) != 'd1'


def test_bid_skips_busy_on_higher_priority(engine):
    eng, drones = engine
    drones['d1'].current_task_type = TaskType.INVESTIGATE
    drones['d1'].busy_with_victim = 7
    assert eng.bid(_target(20, 0), priority=2) != 'd1'


def test_bid_returns_none_when_empty():
    assert AuctionEngine({}, random.Random(0)).bid(_target(0, 0), 2) is None


def test_bid_tiebreak_is_seeded(engine):
    eng, _ = engine
    eng._rng = random.Random(99)
    a = [eng.bid(_target(0, 0), 2) for _ in range(20)]
    eng._rng = random.Random(99)
    b = [eng.bid(_target(0, 0), 2) for _ in range(20)]
    assert a == b
    assert len(set(a)) >= 2, f'tie-break did not vary: {set(a)}'


def test_bid_tolerates_skipped_pose(engine):
    eng, drones = engine
    drones['d1'].pose = None
    assert eng.bid(_target(20, 0), priority=2) != 'd1'


# Bid VO + best_bid

def test_best_bid_returns_winner_with_utility(engine):
    eng, _ = engine
    winner = eng.best_bid(_target(20, 0), priority=2)
    assert winner is not None
    assert isinstance(winner, Bid)
    assert winner.bidder == 'd1'   # closest to (20, 0)
    assert winner.utility > 0
    assert winner.target_x == 20.0
    assert winner.target_y == 0.0


def test_best_bid_none_when_all_excluded(engine):
    eng, _ = engine
    assert eng.best_bid(_target(20, 0), priority=2,
                        exclude={'d1', 'd2', 'd3', 'd4'}) is None


def test_top_bids_returns_descending_by_utility(engine):
    eng, _ = engine
    # Target (20, 0): d1 closest (10 m), d2 + d4 equidistant (~22 m), d3 farthest (~30 m)
    bids = eng.top_bids(_target(20, 0), priority=2, n=4)
    assert len(bids) == 4
    # Descending utility:
    for i in range(len(bids) - 1):
        assert bids[i].utility >= bids[i + 1].utility
    # d1 leads
    assert bids[0].bidder == 'd1'
    # d3 trails
    assert bids[-1].bidder == 'd3'


def test_top_bids_respects_n(engine):
    eng, _ = engine
    bids = eng.top_bids(_target(20, 0), priority=2, n=2)
    assert len(bids) == 2


def test_top_bids_exclude_works(engine):
    eng, _ = engine
    bids = eng.top_bids(_target(20, 0), priority=2, n=4, exclude={'d1'})
    bidders = {b.bidder for b in bids}
    assert 'd1' not in bidders


def test_top_bids_n_zero_returns_empty(engine):
    eng, _ = engine
    assert eng.top_bids(_target(20, 0), priority=2, n=0) == []


def test_legacy_bid_shim_returns_string(engine):
    """The legacy bid() must keep returning Optional[str]."""
    eng, _ = engine
    winner = eng.bid(_target(20, 0), priority=2)
    assert winner == 'd1'
    assert isinstance(winner, str)


# ------------------------------------------------------------ sector_owner_for

def _wedge_drone(name, start_rad, end_rad):
    return SimpleNamespace(
        name=name,
        sector_start_rad=float(start_rad),
        sector_end_rad=float(end_rad),
    )


def test_sector_owner_returns_correct_wedge():
    drones = [
        _wedge_drone('d1', 0.0, math.pi / 2),
        _wedge_drone('d2', math.pi / 2, math.pi),
        _wedge_drone('d3', math.pi, 3 * math.pi / 2),
        _wedge_drone('d4', 3 * math.pi / 2, 2 * math.pi),
    ]
    # (10, 0) is bearing 0 -> in d1's wedge [0, π/2)
    assert sector_owner_for(_target(10, 0), drones) == 'd1'
    # (-10, -10) is bearing 5π/4 -> in d3's wedge [π, 3π/2)
    assert sector_owner_for(_target(-10, -10), drones) == 'd3'


def test_sector_owner_none_at_origin():
    drones = [_wedge_drone('d1', 0.0, math.pi)]
    assert sector_owner_for(_target(0, 0), drones) is None


def test_sector_owner_skips_unassigned_wedges():
    """Drones whose sector_start_rad == sector_end_rad have no wedge:
    return None rather than misattributing."""
    drones = [
        _wedge_drone('d1', 0.0, 0.0),
        _wedge_drone('d2', 0.0, 0.0),
    ]
    assert sector_owner_for(_target(10, 0), drones) is None


def test_sector_owner_normalises_negative_bearings():
    """A southern point has math.atan2 ∈ (-π, 0); the helper must
    normalise to [0, 2π) so it lines up with the wedges."""
    drones = [
        _wedge_drone('d1', 0.0, math.pi),
        _wedge_drone('d2', math.pi, 2 * math.pi),
    ]
    # (10, -10) is bearing -π/4 -> normalised 7π/4 -> in d2
    assert sector_owner_for(_target(10, -10), drones) == 'd2'


def test_sector_owner_handles_wrap_around_wedge():
    """A wedge with start > end wraps across the
    0/2π seam. Both halves of the wedge must report the same owner.
    """
    drones = [
        # d1 owns [0, π) (no wrap)
        _wedge_drone('d1', 0.0, math.pi),
        # d2 wraps: start = 5.8 rad, end = 0.5 rad -> covers
        # [5.8, 2π) ∪ [0, 0.5).
        _wedge_drone('d2', 5.8, 0.5),
    ]
    # Bearing in the late-bearing half (5.8 <= θ < 2π). (10, -1) -> atan2(-1,10) ≈ -0.1
    # -> normalised 2π - 0.1 ≈ 6.18 -> in d2's wrap wedge.
    assert sector_owner_for(_target(10, -1), drones) == 'd2'
    # Bearing in the early half (0 <= θ < 0.5). (10, 1) -> atan2(1,10) ≈ 0.0997
    # -> ALSO in d2's wrap wedge. d1's wedge starts at 0 too but check returns
    # the first drone matched (d1 in this layout); that's intentional: when
    # wedges overlap because the operator misconfigures them, the iteration
    # order resolves the ambiguity. The point of THIS test is that the wrap
    # half of d2 IS reachable at all (the earlier code returned None
    # for any bearing > 5.8).
    # Use a configuration where d2's wrap wedge doesn't overlap d1.
    drones2 = [
        _wedge_drone('d1', 0.5, 5.8),   # the non-wrap complement
        _wedge_drone('d2', 5.8, 0.5),   # the wrap wedge
    ]
    # Both halves of d2's wrap should now report d2.
    assert sector_owner_for(_target(10, -1), drones2) == 'd2', (
        'late-bearing half of wrap wedge missed'
    )
    assert sector_owner_for(_target(10, 1), drones2) == 'd2', (
        'early-bearing half of wrap wedge missed'
    )
    # And a bearing in the non-wrap complement still resolves to d1.
    assert sector_owner_for(_target(-10, 0), drones2) == 'd1'


def test_sector_owner_uses_mission_center_offset():
    """Wedges are computed about the supplied mission_center, not (0,0)."""
    drones = [_wedge_drone('d1', 0.0, math.pi)]
    # Point at (5, 0) relative to centre (10, 0) is bearing π -> outside d1's
    # [0, π) wedge (exclusive upper bound).
    assert sector_owner_for(_target(5, 0), drones, mission_center=(10, 0)) is None
    # Point at (15, 0) relative to centre (10, 0) is bearing 0 -> in d1.
    assert sector_owner_for(_target(15, 0), drones, mission_center=(10, 0)) == 'd1'
