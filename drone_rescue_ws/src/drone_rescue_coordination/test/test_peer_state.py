"""Tests for the pure-domain decentralisation skeleton.

Covers the ``InMemoryPeerStateRegistry`` semantics (stamp-based merging,
self-immunity, sorted iteration) and the ``DeterministicGossipRound``
properties (convergence under quiescence, byte-identical view across
the fleet once synchronised, single-round delivery for direct gossip).
"""

from __future__ import annotations

from drone_rescue_coordination.lib.domain.peer_state import (
    DeterministicGossipRound,
    InMemoryPeerStateRegistry,
)
from drone_rescue_coordination.lib.ports.peer_state import (
    PeerGossipUpdate,
    PeerSnapshot,
    Position2D,
)


def _snap(name, x, y, *, stamp=0.0, busy=None, down=False):
    return PeerSnapshot(
        name=name, pose=Position2D(x, y),
        battery_ok=True, is_down=down,
        current_task_type=0, busy_with_victim=busy,
        stamp_sec=stamp,
    )


# InMemoryPeerStateRegistry

def test_registry_seeds_with_own_snapshot():
    reg = InMemoryPeerStateRegistry('drone1', _snap('drone1', 10.0, 0.0))
    assert reg.snapshot_self().name == 'drone1'
    assert list(reg.candidates()) == [reg.snapshot_self()]


def test_registry_rejects_init_with_mismatched_self_name():
    import pytest
    with pytest.raises(ValueError, match='does not match'):
        InMemoryPeerStateRegistry('drone1', _snap('drone2', 0.0, 0.0))


def test_apply_update_accepts_fresh_peer_snapshot():
    reg = InMemoryPeerStateRegistry('drone1', _snap('drone1', 10.0, 0.0))
    assert reg.apply_update(PeerGossipUpdate(
        from_drone='drone2', snapshot=_snap('drone2', 0.0, 10.0, stamp=1.0),
    )) is True
    assert reg.get('drone2').pose.x == 0.0


def test_apply_update_rejects_self_snapshot_from_peers():
    """A peer cannot speak for us; update_self is the only path."""
    reg = InMemoryPeerStateRegistry('drone1', _snap('drone1', 10.0, 0.0))
    assert reg.apply_update(PeerGossipUpdate(
        from_drone='drone2', snapshot=_snap('drone1', 99.0, 99.0, stamp=10.0),
    )) is False
    assert reg.snapshot_self().pose.x == 10.0   # unchanged


def test_apply_update_drops_stale_snapshot():
    reg = InMemoryPeerStateRegistry('drone1', _snap('drone1', 10.0, 0.0))
    reg.apply_update(PeerGossipUpdate(
        from_drone='drone2', snapshot=_snap('drone2', 0.0, 10.0, stamp=5.0),
    ))
    # Older snapshot at stamp=1.0 must be dropped.
    assert reg.apply_update(PeerGossipUpdate(
        from_drone='drone2', snapshot=_snap('drone2', 9.0, 9.0, stamp=1.0),
    )) is False
    assert reg.get('drone2').pose.x == 0.0   # newer snapshot retained


def test_candidates_iterate_in_sorted_name_order():
    """Determinism: AuctionEngine's tie-break depends on stable order."""
    reg = InMemoryPeerStateRegistry('drone2', _snap('drone2', 0.0, 0.0))
    for n in ('drone4', 'drone1', 'drone3'):
        reg.apply_update(PeerGossipUpdate(
            from_drone=n, snapshot=_snap(n, 1.0, 2.0, stamp=1.0),
        ))
    assert [s.name for s in reg.candidates()] == [
        'drone1', 'drone2', 'drone3', 'drone4',
    ]


def test_update_self_refreshes_own_snapshot():
    reg = InMemoryPeerStateRegistry('drone1', _snap('drone1', 10.0, 0.0))
    reg.update_self(_snap('drone1', 11.0, 1.0, stamp=2.0))
    assert reg.snapshot_self().pose.x == 11.0


def test_update_self_rejects_mismatched_name():
    import pytest
    reg = InMemoryPeerStateRegistry('drone1', _snap('drone1', 10.0, 0.0))
    with pytest.raises(ValueError, match='update_self requires'):
        reg.update_self(_snap('drone2', 0.0, 0.0))


# DeterministicGossipRound

def _fleet_registries():
    """Four drones, each seeded only with self."""
    placements = {
        'drone1': (10.0, 0.0),
        'drone2': (0.0, 10.0),
        'drone3': (-10.0, 0.0),
        'drone4': (0.0, -10.0),
    }
    return {
        name: InMemoryPeerStateRegistry(name, _snap(name, x, y, stamp=1.0))
        for name, (x, y) in placements.items()
    }


def test_one_round_of_direct_gossip_synchronises_the_fleet():
    """With ``gossip_known_peers=False`` (direct broadcast) a single
    round of N-to-N gossip is enough for every drone to learn every
    other drone's self-snapshot."""
    fleet = _fleet_registries()
    accepted = DeterministicGossipRound.step(fleet, gossip_known_peers=False)
    # 4 drones each receive 3 fresh peer snapshots = 12 accepted.
    assert accepted == 12
    for reg in fleet.values():
        assert set(reg.view().keys()) == {'drone1', 'drone2', 'drone3', 'drone4'}


def test_quiescent_after_first_full_round():
    fleet = _fleet_registries()
    rounds = DeterministicGossipRound.run_until_quiescent(fleet)
    # First round delivers everything; second round accepts nothing.
    assert rounds == 2


def test_views_are_byte_identical_after_synchronisation():
    """The headline correctness claim of the skeleton: once converged,
    every drone's view of the fleet is the same dict, so a per-drone
    auction over each view will compute identical winners."""
    fleet = _fleet_registries()
    DeterministicGossipRound.run_until_quiescent(fleet)
    views = [reg.view() for reg in fleet.values()]
    for v in views[1:]:
        assert v == views[0]


def test_gossip_is_deterministic_across_repeated_runs():
    """Same inputs -> identical accepted-update count and final views,
    every time. Determinism is the gate for the seeded harness."""
    counts = []
    final_views = []
    for _ in range(3):
        fleet = _fleet_registries()
        counts.append(DeterministicGossipRound.step(fleet))
        final_views.append({n: reg.view() for n, reg in fleet.items()})
    assert counts[0] == counts[1] == counts[2]
    assert final_views[0] == final_views[1] == final_views[2]
