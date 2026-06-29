"""Pure-domain decentralisation skeleton: gossiped peer state.

Skeleton only; live cutover deferred. Two pieces:

- ``InMemoryPeerStateRegistry``: a single drone's gossiped view of
  its peers. Implements the ``PeerStateRegistry`` Protocol; merges
  ``PeerGossipUpdate`` messages by stamp, drops stale snapshots, exposes
  ``candidates()`` so the existing ``AuctionEngine`` can run per-drone
  over this view with zero engine changes.
- ``DeterministicGossipRound``: a synchronous simulator. Given the
  fleet's registries and a fixed (sorted) delivery order, exchange
  one round of "every drone broadcasts its current view to every
  neighbour." Determinism is by construction: no wall clock, no
  RNG, no async; iteration order is sorted drone names.

This is the algorithm-correctness half of decentralisation. The
timing-robustness half (the live DDS gossip adapter, async delivery,
lost messages) lives in the ROS adapter layer, is deferred, and is
documented in the thesis as future work that trades reproducibility
for robustness by design.
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Mapping, Optional

from drone_rescue_coordination.lib.ports.peer_state import (
    PeerGossipUpdate,
    PeerSnapshot,
    PeerStateRegistry,
)


class InMemoryPeerStateRegistry(PeerStateRegistry):
    """A drone's local gossiped view of the fleet.

    Holds one ``PeerSnapshot`` per known drone, including its own.
    ``apply_update`` keeps the per-name latest snapshot by ``stamp_sec``;
    a snapshot with an earlier or equal stamp than the local copy is
    dropped (returns ``False``). Self-updates are also dropped: a
    drone never overwrites its own snapshot from a peer's gossip;
    that is what ``update_self`` is for.
    """

    def __init__(self, self_name: str, initial_self: PeerSnapshot):
        if initial_self.name != self_name:
            raise ValueError(
                f'initial_self.name={initial_self.name!r} does not match '
                f'self_name={self_name!r}'
            )
        self._self_name = self_name
        self._snapshots: Dict[str, PeerSnapshot] = {self_name: initial_self}

    # PeerStateRegistry
    def candidates(self) -> Iterable[PeerSnapshot]:
        # Sorted by name for deterministic iteration order: the
        # auction's eligibility/tie-break stability depends on this.
        for name in sorted(self._snapshots):
            yield self._snapshots[name]

    def get(self, name: str) -> Optional[PeerSnapshot]:
        return self._snapshots.get(name)

    def apply_update(self, update: PeerGossipUpdate) -> bool:
        snap = update.snapshot
        if snap.name == self._self_name:
            # A peer cannot speak about us authoritatively.
            return False
        existing = self._snapshots.get(snap.name)
        if existing is not None and snap.stamp_sec <= existing.stamp_sec:
            return False
        self._snapshots[snap.name] = snap
        return True

    def snapshot_self(self) -> PeerSnapshot:
        return self._snapshots[self._self_name]

    def view(self) -> Mapping[str, PeerSnapshot]:
        # Defensive copy so consumers cannot mutate the registry.
        return dict(self._snapshots)

    # self-update (out of gossip)
    def update_self(self, snap: PeerSnapshot) -> None:
        """Refresh the drone's own snapshot (called locally each tick,
        not via gossip). ``snap.name`` must match the registry's
        ``self_name``."""
        if snap.name != self._self_name:
            raise ValueError(
                f'update_self requires snap.name=={self._self_name!r}, '
                f'got {snap.name!r}'
            )
        self._snapshots[self._self_name] = snap


class DeterministicGossipRound:
    """Synchronous, deterministic gossip simulator over a fleet.

    Each round: every drone (in sorted-name order) broadcasts its
    current view to every other drone (in sorted-name order). The
    payload is the broadcaster's own snapshot and (optionally) its
    known peers' snapshots, one ``PeerGossipUpdate`` per snapshot
    per recipient. Recipients ``apply_update`` in order; stale or
    self-targeted updates are dropped.

    The simulator is the local-pytest stand-in for the live DDS bus;
    it lets us prove the convergence + auction-equivalence properties
    deterministically. The live cutover replaces this with real
    publish/subscribe semantics, which introduces ordering
    non-determinism the seeded harness cannot model: that trade is
    the explicit future-work boundary.
    """

    @staticmethod
    def step(registries: Mapping[str, InMemoryPeerStateRegistry],
             *, gossip_known_peers: bool = True) -> int:
        """Run one synchronous gossip round across ``registries``
        keyed by drone name. Returns the number of accepted updates
        (useful as a convergence signal).

        ``gossip_known_peers=True`` simulates transitive gossip: a
        drone forwards what it knows about other drones too. Set False
        for direct-only gossip (each drone broadcasts only itself).
        """
        names = sorted(registries.keys())
        accepted = 0
        # Snapshot the per-drone broadcast payload before any apply
        # runs, otherwise a recipient might "see" a fresher
        # broadcast a peer hasn't actually sent yet (which would
        # break determinism).
        payloads: Dict[str, List[PeerSnapshot]] = {}
        for sender in names:
            sender_reg = registries[sender]
            if gossip_known_peers:
                # All snapshots the sender knows, sorted for determinism.
                payloads[sender] = [
                    sender_reg.get(n)        # type: ignore[misc]
                    for n in sorted(sender_reg.view().keys())
                ]
            else:
                payloads[sender] = [sender_reg.snapshot_self()]
        # Apply in sorted (sender, recipient) order.
        for sender in names:
            for recipient in names:
                if recipient == sender:
                    continue
                target = registries[recipient]
                for snap in payloads[sender]:
                    if snap is None:
                        continue
                    update = PeerGossipUpdate(
                        from_drone=sender, snapshot=snap, via=(sender,),
                    )
                    if target.apply_update(update):
                        accepted += 1
        return accepted

    @staticmethod
    def run_until_quiescent(
        registries: Mapping[str, InMemoryPeerStateRegistry],
        *, max_rounds: int = 16, gossip_known_peers: bool = True,
    ) -> int:
        """Run gossip rounds until no registry accepts a new update,
        or ``max_rounds`` is reached. Returns the round count."""
        for r in range(1, max_rounds + 1):
            if DeterministicGossipRound.step(
                    registries, gossip_known_peers=gossip_known_peers) == 0:
                return r
        return max_rounds
