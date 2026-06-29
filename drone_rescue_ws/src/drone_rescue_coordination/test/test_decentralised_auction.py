"""Per-drone auction over a gossiped peer state.

The headline claim: the existing ``AuctionEngine`` is already pure over
an injected registry, so decentralisation reduces to changing what the
registry holds. These tests prove the algorithm half of that claim with
no rclpy, no DDS, no async:

- a per-drone auction over the synchronised gossiped view computes
  the same winner every drone independently arrives at, byte-identical
  to a centralised auction over the same fleet: the convergence
  result the live system would need;
- a per-drone auction over an unsynchronised view (one drone missing
  from a peer's view) can diverge: the timing-robustness problem
  the live DDS adapter would have to solve, and the principled reason
  the live cutover is deferred in the thesis.

Pure-domain pytest; nothing here that the seeded evaluation harness
cannot replay.
"""

from __future__ import annotations

import random
from types import SimpleNamespace

from drone_rescue_coordination.lib.auction import AuctionEngine
from drone_rescue_coordination.lib.domain.peer_state import (
    DeterministicGossipRound,
    InMemoryPeerStateRegistry,
)
from drone_rescue_coordination.lib.ports.peer_state import (
    PeerSnapshot,
    Position2D,
)


def _snap(name, x, y, *, stamp=1.0):
    return PeerSnapshot(
        name=name, pose=Position2D(x, y),
        battery_ok=True, is_down=False,
        current_task_type=0, busy_with_victim=None,
        stamp_sec=stamp,
    )


def _fleet():
    placements = {
        'drone1': (10.0, 0.0),
        'drone2': (0.0, 10.0),
        'drone3': (-10.0, 0.0),
        'drone4': (0.0, -10.0),
    }
    return {
        name: InMemoryPeerStateRegistry(name, _snap(name, x, y))
        for name, (x, y) in placements.items()
    }


def _target(x, y):
    return SimpleNamespace(x=x, y=y)


# registry-as-bidder

def test_peer_snapshot_is_a_structural_bidder_for_the_auction():
    """``PeerSnapshot`` has the attributes the auction reads
    (``name``, ``pose.x/.y``, ``battery_ok``, ``is_down``,
    ``current_task_type``, ``busy_with_victim``), so the engine
    runs unchanged over the gossiped registry."""
    fleet = _fleet()
    DeterministicGossipRound.run_until_quiescent(fleet)
    engine = AuctionEngine(fleet['drone1'], random.Random(7))
    winner = engine.bid(_target(20.0, 0.0), priority=2)
    assert winner == 'drone1'   # geometrically closest to (20, 0)


# convergence -> identical winners

def test_per_drone_auctions_agree_after_gossip_synchronisation():
    """The headline correctness claim. Each drone runs its own
    ``AuctionEngine`` over its own gossiped registry, and once the
    fleet is synchronised every drone picks the same winner, byte
    identical to what a centralised auction over the same fleet would
    pick."""
    fleet = _fleet()
    DeterministicGossipRound.run_until_quiescent(fleet)
    target = _target(25.0, 5.0)
    # Per-drone independent decisions:
    per_drone_winners = {}
    for name, reg in fleet.items():
        engine = AuctionEngine(reg, random.Random(11))
        per_drone_winners[name] = engine.bid(target, priority=2)
    # Every drone agrees, and the choice matches the centralised auction
    # over the same state (drone1's registry is by construction the
    # canonical view post-synchronisation; any drone's view is identical).
    distinct = set(per_drone_winners.values())
    assert len(distinct) == 1
    central_engine = AuctionEngine(fleet['drone1'], random.Random(11))
    assert per_drone_winners['drone2'] == central_engine.bid(target, priority=2)


def test_synchronised_decentralised_matches_central_under_same_seed():
    """Sweep across several targets: every per-drone choice equals the
    centralised choice. The invariant the live cutover would have to
    preserve to claim a regression-free decentralisation."""
    fleet = _fleet()
    DeterministicGossipRound.run_until_quiescent(fleet)
    targets = [
        _target(20.0, 0.0),
        _target(-5.0, 12.0),
        _target(0.0, -25.0),
        _target(8.0, 8.0),
    ]
    central = AuctionEngine(fleet['drone1'], random.Random(99))
    central_winners = [central.bid(t, priority=2) for t in targets]
    for name, reg in fleet.items():
        engine = AuctionEngine(reg, random.Random(99))
        winners = [engine.bid(t, priority=2) for t in targets]
        assert winners == central_winners, f'{name} diverged'


# unsynchronised state -> documented divergence

def test_unsynchronised_views_can_diverge_naming_the_F4_gated_problem():
    """The trade-off the thesis names: before convergence the
    decentralised auction can disagree across drones, because each one
    decides on what it knows. This test demonstrates the divergence
    explicitly; it is not a bug but the property the live DDS cutover
    has to mitigate."""
    fleet = _fleet()
    # Manually deliver a partial gossip: drone1 has not yet learned
    # of drone3, but everyone else has converged on the full view.
    DeterministicGossipRound.step(fleet, gossip_known_peers=False)
    # Now wipe drone3 from drone1's view to simulate a dropped update.
    drone1_view = fleet['drone1']
    del drone1_view._snapshots['drone3']    # noqa: SLF001 (test-only surgery)
    # drone3 is geometrically closest to (-12, 0); drone1's local view
    # omits drone3, so it picks the next-nearest.
    target = _target(-12.0, 0.0)
    drone1_engine = AuctionEngine(drone1_view, random.Random(42))
    drone2_engine = AuctionEngine(fleet['drone2'], random.Random(42))
    drone1_winner = drone1_engine.bid(target, priority=2)
    drone2_winner = drone2_engine.bid(target, priority=2)
    assert drone2_winner == 'drone3'
    assert drone1_winner != 'drone3'    # divergence: partial view leads astray
