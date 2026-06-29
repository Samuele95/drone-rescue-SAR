"""BidderRegistry driven port.

The AuctionEngine takes ``drones: dict[str, DroneRecord]`` and
iterates ``.values()``. Promoting that to a structural Protocol lets
the engine talk to any container that can produce eligible bidders:
a list-backed test fake, a sector-pre-filtered wrapper, or the
production ``MissionDrones(mission)``.
"""

from __future__ import annotations

from typing import Iterable, Optional, Protocol


# 3T boundary annotation: bidders feed the L3 deliberative allocation
# auction (``lib/auction.AuctionEngine``); the registry itself is a
# structural shape consumed across layers, so cross-cutting.
LAYER_BOUNDARY = 'cross-cutting'


class Bidder(Protocol):
    """Structural type of a single bidder. Same shape the auction's
    private ``_Bidder`` Protocol documents. Promoted here so the
    public Protocol type can be referenced from tests and adapters."""
    name: str
    pose: object        # geometry_msgs/Point or domain Position; the auction reads .x, .y
    battery_ok: bool
    is_down: bool
    current_task_type: int
    busy_with_victim: Optional[int]


class BidderRegistry(Protocol):
    """A container the AuctionEngine iterates each round.

    Implementations:
      * ``MissionDrones(mission)``: production; wraps the
        Mission aggregate's drones dict.
      * ``DictBidderRegistry(d)``: adapter over a raw dict (the
        legacy mission_manager DroneRecord dict implements it
        structurally already, but the adapter makes the typing
        explicit).
    """

    def candidates(self) -> Iterable[Bidder]: ...

    def get(self, name: str) -> Optional[Bidder]: ...


class DictBidderRegistry:
    """Adapter promoting a ``dict[str, Bidder]`` to a BidderRegistry.
    Used by mission_manager during the transition where the legacy
    DroneRecord dict is still the source of truth."""

    def __init__(self, drones):
        self._drones = drones

    def candidates(self) -> Iterable[Bidder]:
        return iter(self._drones.values())

    def get(self, name: str) -> Optional[Bidder]:
        return self._drones.get(name)
