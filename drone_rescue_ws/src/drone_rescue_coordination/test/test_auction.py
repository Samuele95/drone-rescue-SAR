"""Unit tests for mission_manager._auction.

Validates the auction tie-break determinism and the exclude/busy/
battery filters. Uses a real MissionManager instance (rclpy.init/shutdown
in fixtures) but never spins it; we just call the private method.
"""

from __future__ import annotations

import random

import pytest
import rclpy

from geometry_msgs.msg import Point
from drone_rescue_coordination.mission_manager import (
    MissionManager,
    DroneRecord,
)
from drone_rescue_msgs.msg import TaskAssignment


@pytest.fixture(scope='module', autouse=True)
def _rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def mm():
    """MissionManager with the four default drones, all alive at known
    positions; never activated."""
    node = MissionManager()
    # Place 4 drones at the four cardinal points, 10 m from origin.
    placements = [('drone1', 10.0, 0.0),
                  ('drone2', 0.0, 10.0),
                  ('drone3', -10.0, 0.0),
                  ('drone4', 0.0, -10.0)]
    for name, x, y in placements:
        node._drones[name] = DroneRecord(
            name=name,
            pose=Point(x=x, y=y, z=10.0),
            battery_ok=True,
            is_down=False,
        )
    yield node
    node.destroy_node()


def _target(x: float, y: float) -> Point:
    return Point(x=x, y=y, z=0.0)


# ------------------------------------------------------------ utility

def test_auction_picks_closest_drone(mm):
    """Target near drone1 should pick drone1."""
    winner = mm._auction(_target(20.0, 0.0), priority=2)
    assert winner == 'drone1'


def test_auction_picks_correct_when_target_moves(mm):
    """Sweeping the target across the cardinals, the auction winner
    should follow."""
    assert mm._auction(_target(20, 0), priority=2) == 'drone1'
    assert mm._auction(_target(0, 20), priority=2) == 'drone2'
    assert mm._auction(_target(-20, 0), priority=2) == 'drone3'
    assert mm._auction(_target(0, -20), priority=2) == 'drone4'


def test_auction_returns_none_when_no_drones(mm):
    """No drones available -> None."""
    mm._drones.clear()
    assert mm._auction(_target(0, 0), priority=2) is None


# ------------------------------------------------------------ filters

def test_auction_excludes_is_down_drones(mm):
    """A drone with is_down=True must never win."""
    # Knock drone1 down so the next-closest (drones 2/4 tied at 11.18 m,
    # drone3 at 22.36 m) wins.
    mm._drones['drone1'].is_down = True
    winner = mm._auction(_target(20.0, 0.0), priority=2)
    assert winner != 'drone1'


def test_auction_excludes_battery_low_drones(mm):
    """A drone with battery_ok=False must never win."""
    mm._drones['drone1'].battery_ok = False
    winner = mm._auction(_target(20.0, 0.0), priority=2)
    assert winner != 'drone1'


def test_auction_excludes_busy_drones(mm):
    """Drone busy on an INVESTIGATE (with a victim) is unavailable for
    a competing auction."""
    mm._drones['drone1'].current_task_type = TaskAssignment.INVESTIGATE
    mm._drones['drone1'].busy_with_victim = 7
    winner = mm._auction(_target(20.0, 0.0), priority=2)
    assert winner != 'drone1'


def test_auction_excludes_set(mm):
    """`exclude` set forbids that drone from winning even when it would
    otherwise be the best fit."""
    winner = mm._auction(_target(20.0, 0.0), priority=2,
                         exclude={'drone1'})
    assert winner != 'drone1'


def test_bid_is_a_single_value_object():
    """Bid existed twice (auction + value_objects) with identical fields;
    auction.Bid now re-exports the single domain VO."""
    from drone_rescue_coordination.lib.auction import Bid as AuctionBid
    from drone_rescue_coordination.lib.domain.value_objects import Bid as VoBid
    assert AuctionBid is VoBid


# ----------------------------------------------------- health/state gate

@pytest.mark.parametrize('state_name', ['EMERGENCY', 'LANDING', 'RETURNING'])
def test_auction_excludes_unhealthy_state_drones(mm, state_name):
    """A drone whose flight-controller state is EMERGENCY / LANDING /
    RETURNING must never win, even with a healthy battery, known pose and
    not busy. Previously _eligible_bids checked battery_ok / is_down / busy /
    pose but never DroneState, so an EMERGENCY drone could win an auction."""
    from drone_rescue_coordination.lib.domain.drone_state import DroneState
    # drone1 is the closest to the target (20, 0); without the state gate it
    # would win outright.
    mm._drones['drone1'].drone_state = getattr(DroneState, state_name)
    winner = mm._auction(_target(20.0, 0.0), priority=2)
    assert winner != 'drone1'


def test_auction_prefers_more_capable_drone(mm):
    """Capability scales the utility, so a more-capable drone beats an
    equidistant baseline drone for the same target."""
    # Place drone1 and drone3 equidistant from a target on the +x axis is not
    # possible with the cardinal layout; instead make them equidistant by
    # targeting the origin (all four are 10 m out) and boost drone3.
    for d in mm._drones.values():
        d.capability = 1.0
    mm._drones['drone3'].capability = 5.0
    winner = mm._auction(_target(0.0, 0.0), priority=2)
    assert winner == 'drone3'


def test_auction_capability_default_is_homogeneous(mm):
    """With every capability at the 1.0 default the winner is purely by
    distance; capability introduces no change for a homogeneous fleet."""
    winner = mm._auction(_target(20.0, 0.0), priority=2)
    assert winner == 'drone1'   # closest, as in the baseline


def test_auction_allows_available_states(mm):
    """States that are NOT in the unavailable set (and the default None) stay
    eligible; the gate excludes only EMERGENCY / LANDING / RETURNING."""
    from drone_rescue_coordination.lib.domain.drone_state import DroneState
    for ok_state in (DroneState.IDLE, DroneState.SURVEYING,
                     DroneState.HOVER, DroneState.NAVIGATING):
        mm._drones['drone1'].drone_state = ok_state
        winner = mm._auction(_target(20.0, 0.0), priority=2)
        assert winner == 'drone1', f'{ok_state.name} should stay eligible'


# ------------------------------------------------------------ tie-break determinism

def test_auction_tiebreak_is_seeded():
    """All four drones equidistant from the target -> RNG picks one
    deterministically. Same `rng=` constructor seed -> same sequence.

    This test exercises the `rng=` constructor kwarg instead of mutating
    the private `mm._auction_engine._rng` after construction.
    """
    target = _target(0.0, 0.0)

    def _build_mm(seed: int):
        node = MissionManager(rng=random.Random(seed))
        placements = [('drone1', 10.0, 0.0),
                      ('drone2', 0.0, 10.0),
                      ('drone3', -10.0, 0.0),
                      ('drone4', 0.0, -10.0)]
        for name, x, y in placements:
            node._drones[name] = DroneRecord(
                name=name,
                pose=Point(x=x, y=y, z=10.0),
                battery_ok=True,
                is_down=False,
            )
        return node

    mm_a = _build_mm(42)
    winners_a = [mm_a._auction(target, priority=2) for _ in range(20)]
    mm_a.destroy_node()

    mm_b = _build_mm(42)
    winners_b = [mm_b._auction(target, priority=2) for _ in range(20)]
    mm_b.destroy_node()

    assert winners_a == winners_b, (
        'tie-break is not reproducible under same seed'
    )
    # And the tie-break should sample across drones (not always the
    # same one), otherwise the RNG isn't really being consulted.
    assert len(set(winners_a)) >= 2, (
        f'tie-break did not vary across 20 calls: {set(winners_a)}'
    )
