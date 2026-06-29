"""PeerStateRegistry: gossiped, no-central-authority view of fleet peers.

The slides (Marcelletti, "Autonomous and Collaborative Robotics",
A.Y. 2025/26, Unit 11) describe decentralised multi-robot
coordination: every robot equal and autonomous; coordination emerges
from peer-to-peer rather than a single leader. The skeleton here
names the data abstraction that decentralisation requires: each drone
keeps a gossiped view of its peers, updated by ``PeerGossipUpdate``
messages exchanged over the bus, rather than reading a single central
``mission_manager.DroneRecord`` dictionary.

The port is shaped to be a structural extension of ``BidderRegistry``:
``candidates()`` / ``get(name)`` let the existing ``AuctionEngine``
run per-drone over this view with zero changes, and ``apply_update``
absorbs gossip messages. The deliberative apex is already pure over
an injected registry, so decentralisation reduces to changing what
the registry holds, not rewriting the auction.

The live DDS-bus gossip adapter is deferred (it would route the
production surveyor LifecycleNode through ``Surveyor.tick``) and
documented as future work in the thesis. This module is the
pure-domain proof of algorithm the live work would target. Genuine
concurrency over DDS trades reproducibility for robustness; name that
trade-off explicitly rather than fight it.

3T boundary: ``LAYER_BOUNDARY = 'cross-cutting'``; peer state is
infrastructure consumed by the L3 auction and any distributed-goal
motivation, in the same family as ``BidderRegistry`` and ``Clock``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Mapping, Optional, Protocol, Tuple


LAYER_BOUNDARY = 'cross-cutting'   # 3T architecture annotation.


@dataclass(frozen=True)
class Position2D:
    """Minimal pose with ``.x`` / ``.y``: structurally compatible with
    the auction engine's ``Bidder.pose`` shape, so a ``PeerSnapshot`` is
    a ``Bidder`` without importing the domain ``Position``."""
    x: float
    y: float


@dataclass(frozen=True)
class PeerSnapshot:
    """One peer's belief about itself, gossiped to its neighbours.

    The auction reads ``name`` / ``pose`` / ``battery_ok`` / ``is_down``,
    the same surface ``BidderRegistry`` provides. ``stamp_sec`` is
    the gossip-time the snapshot was minted, which lets a registry
    drop stale updates (and lets the determinism gate hash a fleet's
    view at a fixed time).
    """

    name: str
    pose: Position2D
    battery_ok: bool = True
    is_down: bool = False
    current_task_type: int = 0
    busy_with_victim: Optional[int] = None
    stamp_sec: float = 0.0


@dataclass(frozen=True)
class PeerGossipUpdate:
    """One gossip message: a sender advertises a snapshot of itself (or
    of one of its known peers) to a recipient.

    ``via`` records the path the snapshot took: useful for a future
    loop-suppression policy; the pure-domain skeleton ignores it.
    """

    from_drone: str
    snapshot: PeerSnapshot
    via: Tuple[str, ...] = field(default_factory=tuple)


class PeerStateRegistry(Protocol):
    """A drone's gossiped view of its peers.

    Structurally extends ``BidderRegistry``: the auction engine can
    iterate ``candidates()`` exactly as it does today. ``apply_update``
    absorbs a single gossip message; the registry is responsible for
    rejecting stale snapshots (by ``stamp_sec``) and merging fresh
    ones.

    Implementations:
    - ``InMemoryPeerStateRegistry`` (this repo, pure-domain skeleton).
    - A future ``DdsGossipedRegistry``: the live ROS adapter,
      deferred.
    """

    def candidates(self) -> Iterable[PeerSnapshot]: ...

    def get(self, name: str) -> Optional[PeerSnapshot]: ...

    def apply_update(self, update: PeerGossipUpdate) -> bool:
        """Absorb a gossip message. Returns ``True`` if the snapshot
        was newer than the local view and applied; ``False`` if dropped
        (stale, self, or duplicate)."""
        ...

    def snapshot_self(self) -> PeerSnapshot:
        """The drone's own current snapshot: the seed of the gossip
        it broadcasts each round."""
        ...

    def view(self) -> Mapping[str, PeerSnapshot]:
        """A read-only mapping of name -> latest known snapshot, for
        deterministic inspection / hashing in tests."""
        ...
