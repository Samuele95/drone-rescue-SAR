"""Tests for the driven ports.

Pure-Python; no rclpy.init() required.
"""

from __future__ import annotations

import random
from types import SimpleNamespace

import pytest

from drone_rescue_coordination.lib.auction import AuctionEngine
from drone_rescue_coordination.lib.ports import (
    DictBidderRegistry, FakeClock,
)


def _mk_bidder(name, x=0.0, y=0.0):
    return SimpleNamespace(
        name=name, pose=SimpleNamespace(x=x, y=y, z=0.0),
        battery_ok=True, is_down=False,
        current_task_type=5,             # IDLE
        busy_with_victim=None,
    )


def test_fake_clock_advances():
    c = FakeClock(t=10.0)
    assert c.now_sec() == 10.0
    c.advance(2.5)
    assert c.now_sec() == 12.5


def test_dict_bidder_registry_iterates_dict_values():
    d = {'drone1': _mk_bidder('drone1', 0.0, 0.0),
         'drone2': _mk_bidder('drone2', 5.0, 0.0)}
    r = DictBidderRegistry(d)
    names = {b.name for b in r.candidates()}
    assert names == {'drone1', 'drone2'}
    assert r.get('drone2').name == 'drone2'
    assert r.get('missing') is None


def test_auction_engine_accepts_dict_through_registry_back_compat():
    """Legacy dict input still works, wrapped in DictBidderRegistry."""
    target = SimpleNamespace(x=0.0, y=0.0)
    drones = {'drone1': _mk_bidder('drone1', 0.0, 0.0),
              'drone2': _mk_bidder('drone2', 100.0, 0.0)}
    engine = AuctionEngine(drones, random.Random(42))
    winner = engine.best_bid(target, priority=2)
    assert winner is not None
    assert winner.bidder == 'drone1'   # closer


def test_auction_engine_accepts_registry_directly():
    """New code paths can pass a BidderRegistry."""
    target = SimpleNamespace(x=0.0, y=0.0)
    drones = {'drone1': _mk_bidder('drone1', 0.0, 0.0),
              'drone2': _mk_bidder('drone2', 100.0, 0.0)}
    registry = DictBidderRegistry(drones)
    engine = AuctionEngine(registry, random.Random(42))
    winner = engine.best_bid(target, priority=2)
    assert winner.bidder == 'drone1'


def test_auction_engine_sees_dict_mutations():
    """Critical invariant: mission_manager mutates the same dict in
    place and the auction sees the latest state at each call."""
    target = SimpleNamespace(x=0.0, y=0.0)
    drones = {'drone1': _mk_bidder('drone1', 0.0, 0.0),
              'drone2': _mk_bidder('drone2', 100.0, 0.0)}
    engine = AuctionEngine(drones, random.Random(42))
    assert engine.best_bid(target, priority=2).bidder == 'drone1'
    # Take drone1 out of contention by flipping its flag.
    drones['drone1'].battery_ok = False
    assert engine.best_bid(target, priority=2).bidder == 'drone2'
