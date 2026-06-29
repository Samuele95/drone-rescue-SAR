"""Unit tests for the selectable task-allocation strategies (lib/allocation.py).

Pure-python pytest; no ROS node. Verifies the registry/factory and the three
registered strategies (greedy_auction, round_robin, hungarian).
"""

from __future__ import annotations

import random
from types import SimpleNamespace

import pytest

from drone_rescue_coordination.lib.allocation import (
    AllocationStrategyFactory,
    GreedyAuctionStrategy,
    RoundRobinStrategy,
    HungarianStrategy,
)


def _drone(name, x, y):
    return SimpleNamespace(
        name=name,
        pose=SimpleNamespace(x=x, y=y),
        battery_ok=True,
        is_down=False,
        current_task_type=0,
        busy_with_victim=None,
    )


def _fleet():
    # Four drones at the cardinal points, 10 m from origin.
    return {
        'drone1': _drone('drone1', 10.0, 0.0),
        'drone2': _drone('drone2', 0.0, 10.0),
        'drone3': _drone('drone3', -10.0, 0.0),
        'drone4': _drone('drone4', 0.0, -10.0),
    }


def _target(x, y):
    return SimpleNamespace(x=x, y=y)


def _rng():
    return random.Random(42)


# ------------------------------------------------------------ factory

def test_factory_lists_all_registered_strategies():
    """``motivation_workspace`` joins the three originals as the Unit-10
    distributed-goal coexistence strategy."""
    names = AllocationStrategyFactory.list_names()
    assert names == [
        'greedy_auction', 'hungarian', 'motivation_workspace', 'round_robin',
    ]


def test_factory_default_name_is_registered():
    # mission_manager declares 'greedy_auction' as the default param value.
    assert 'greedy_auction' in AllocationStrategyFactory.list_names()


def test_factory_unknown_name_raises():
    with pytest.raises(ValueError, match='unknown allocation strategy'):
        AllocationStrategyFactory.create('does_not_exist', _fleet(), _rng())


def test_factory_create_returns_right_type():
    s = AllocationStrategyFactory.create('hungarian', _fleet(), _rng())
    assert isinstance(s, HungarianStrategy)


# ------------------------------------------------------------ greedy

def test_greedy_picks_nearest():
    s = GreedyAuctionStrategy(_fleet(), _rng())
    assert s.bid(_target(20.0, 0.0), priority=2) == 'drone1'
    assert s.bid(_target(0.0, 20.0), priority=2) == 'drone2'
    assert s.bid(_target(-20.0, 0.0), priority=2) == 'drone3'


def test_greedy_none_when_fleet_empty():
    s = GreedyAuctionStrategy({}, _rng())
    assert s.bid(_target(0.0, 0.0), priority=2) is None


# ------------------------------------------------------------ hungarian

def test_hungarian_single_target_matches_greedy():
    """For one target the optimal assignment is the nearest drone, provably
    identical to greedy_auction. This is documented/expected behaviour."""
    fleet, rng = _fleet(), _rng()
    greedy = GreedyAuctionStrategy(fleet, random.Random(42))
    hungarian = HungarianStrategy(fleet, random.Random(42))
    for tx, ty in [(20.0, 0.0), (0.0, 20.0), (-20.0, 0.0), (0.0, -20.0)]:
        t = _target(tx, ty)
        assert hungarian.bid(t, priority=2) == greedy.bid(t, priority=2)


def test_hungarian_batch_assigns_jointly():
    """Two targets near drone1 and drone3 -> joint optimum assigns each to
    its nearest, and never double-books a drone."""
    s = HungarianStrategy(_fleet(), _rng())
    winners = s.assign([_target(18.0, 0.0), _target(-18.0, 0.0)], priority=2)
    assert winners == ['drone1', 'drone3']


def test_hungarian_batch_no_double_booking():
    """Two targets both nearest drone1 -> one gets drone1, the other a
    different drone (joint assignment, one task per drone)."""
    s = HungarianStrategy(_fleet(), _rng())
    winners = s.assign([_target(12.0, 1.0), _target(12.0, -1.0)], priority=2)
    assert set(winners) <= {'drone1', 'drone2', 'drone3', 'drone4'}
    assert winners[0] != winners[1]
    assert 'drone1' in winners


# ------------------------------------------------------------ round-robin

def test_round_robin_rotates_over_fleet():
    """Round-robin ignores distance and cycles drones in name order."""
    s = RoundRobinStrategy(_fleet(), _rng())
    # Target is right next to drone1 every time, but rotation ignores that.
    picks = [s.bid(_target(20.0, 0.0), priority=2) for _ in range(5)]
    assert picks == ['drone1', 'drone2', 'drone3', 'drone4', 'drone1']


def test_round_robin_none_when_fleet_empty():
    s = RoundRobinStrategy({}, _rng())
    assert s.bid(_target(0.0, 0.0), priority=2) is None


# ------------------------------------------------------------ top_bids shared

def test_top_bids_shared_across_strategies():
    """top_bids (witness/runner-up selection) is proximity-ordered for every
    strategy; only the primary bid() winner differs."""
    t = _target(20.0, 0.0)
    for cls in (GreedyAuctionStrategy, RoundRobinStrategy, HungarianStrategy):
        s = cls(_fleet(), _rng())
        bids = s.top_bids(t, priority=2, n=2)
        assert [b.bidder for b in bids] == ['drone1', 'drone2']
