"""Single-item assignment auction: pure Python, no rclpy.

Extracted from MissionManager so the auction
logic can be unit-tested in isolation without spinning up a ROS node.
The mission_manager constructs an `AuctionEngine` once at configure
time and delegates to ``bid()`` whenever a candidate needs a winner.

Reference: Gerkey & Matarić, "A Formal Analysis and Taxonomy of Task
Allocation in Multi-Robot Systems", IJRR 23(9), 2004.

The bidder protocol is intentionally minimal: anything with the four
attributes (`name`, `pose`, `battery_ok`, `is_down`) plus
`current_task_type` and `busy_with_victim` qualifies. In practice this
is `mission_manager.DroneRecord`, but tests can pass a SimpleNamespace
or a typing.Protocol implementation without depending on the full
record type.
"""

from __future__ import annotations

import math
import random
from typing import Iterable, List, Optional, Protocol

# TaskType lives in lib/domain/task_type.py (domain is the single source of
# truth); re-exported here so the legacy
# `from .auction import TaskType` import path keeps working.
from .domain.task_type import TaskType  # noqa: F401
# Bid is a single domain value object. The canonical definition lives with its
# frozen-VO peers in lib/domain/value_objects; it is re-exported
# here so the legacy ``from .auction import Bid`` path (lib/allocation) keeps
# working without a second class.
from .domain.value_objects import Bid  # noqa: F401

# Flight-controller states (DroneState.name) in which a drone must not be
# auctioned new work even with a healthy battery, known pose and no busy
# victim. EMERGENCY cuts motors; LANDING and RETURNING are committed to
# a descent / return-to-home and cannot accept a survey or investigate task.
# Compared by ``.name`` so the auction stays duck-typed (no DroneState import).
_UNAVAILABLE_DRONE_STATES = frozenset({'EMERGENCY', 'LANDING', 'RETURNING'})


class _Bidder(Protocol):
    """Structural type the auction iterates over.

    Anything with these attributes qualifies: `mission_manager.DroneRecord`
    is the production implementation; tests use SimpleNamespace.

    `current_task_type` is typed as ``int`` (not ``TaskType``) because
    the ROS message lays the field out as ``uint8``; ``IntEnum`` values
    compare equal to ints, so the membership check in ``bid()`` works
    against either representation.
    """
    name: str
    pose: object        # geometry_msgs/Point or anything with .x, .y
    battery_ok: bool
    is_down: bool
    current_task_type: int
    busy_with_victim: Optional[int]
    # Optional flight-controller state (a DroneState, or None when unknown).
    # When present and in _UNAVAILABLE_DRONE_STATES the bidder is excluded
    # from every auction. Read via getattr so older bidders without
    # the attribute remain eligible.
    drone_state: object
    # Optional capability weight: scales the auction utility so a more-
    # capable drone is preferred. Defaults to 1.0 (homogeneous fleet) via
    # getattr when the bidder doesn't carry it.
    capability: float


class AuctionEngine:
    """Stateless except for a seeded RNG used as tie-breaker.

    Construct once with the live drones dict and a seeded `random.Random`;
    call `bid(target, priority, exclude)` per dispatch. Tie-breaking is
    deterministic under the seed, which makes mission outcomes
    reproducible across runs even when DDS discovery order varies.
    """

    def __init__(self, drones, rng):
        # `drones` accepts either a ``dict[str, Bidder]`` (legacy
        # mission_manager DroneRecord dict) or a ``BidderRegistry``
        # Protocol. When a dict is passed it's
        # wrapped in ``DictBidderRegistry`` so the rest of the engine
        # consumes one shape; mutations on the underlying dict
        # propagate (same by-reference semantics as before).
        from .ports.bidder_registry import DictBidderRegistry
        if hasattr(drones, 'candidates') and callable(drones.candidates):
            self._registry = drones
        else:
            self._registry = DictBidderRegistry(drones)
        self._rng = rng

    def _eligible_bids(
        self,
        target,
        priority: int,
        exclude: Optional[Iterable[str]] = None,
    ) -> List[Bid]:
        """Return every eligible Bid for the target, unsorted.

        Internal helper used by both ``best_bid()`` and ``top_bids()``.
        """
        excl = set(exclude or ())
        tx, ty = float(target.x), float(target.y)
        out: List[Bid] = []
        for d in self._registry.candidates():
            if d.name in excl:
                continue
            if not d.battery_ok or d.is_down:
                continue
            # Health gate: exclude EMERGENCY / LANDING / RETURNING even
            # when battery_ok and not down. ``drone_state`` is optional on the
            # bidder (getattr returns None for records that don't carry it), so the
            # gate is backward-compatible and compared by enum name.
            state = getattr(d, 'drone_state', None)
            if state is not None and getattr(state, 'name', None) in _UNAVAILABLE_DRONE_STATES:
                continue
            if d.current_task_type in (TaskType.INVESTIGATE, TaskType.CONFIRM):
                if d.busy_with_victim is not None:
                    continue
            if d.pose is None:
                continue
            dist = math.hypot(d.pose.x - tx, d.pose.y - ty)
            # Capability-aware utility: a more-capable drone (faster, or
            # a better sensor) is preferred for a task. ``capability`` is
            # optional on the bidder (getattr returns 1.0, so a homogeneous fleet
            # behaves exactly as before).
            capability = float(getattr(d, 'capability', 1.0))
            utility = priority * capability / max(dist, 1.0)
            out.append(Bid(bidder=d.name, utility=utility,
                           target_x=tx, target_y=ty))
        return out

    def best_bid(
        self,
        target,
        priority: int,
        exclude: Optional[Iterable[str]] = None,
    ) -> Optional[Bid]:
        """Return the winning ``Bid``.

        Carries utility alongside bidder name so callers see why the
        winner won. Ties resolved deterministically via the seeded
        RNG. Returns ``None`` when no bidder is eligible.
        """
        bids = self._eligible_bids(target, priority, exclude)
        if not bids:
            return None
        # Sort descending; collect ties at the top via the same
        # 1e-9 float tolerance the legacy bid() used.
        bids.sort(key=lambda b: -b.utility)
        top_utility = bids[0].utility
        top_tier = [b for b in bids if b.utility >= top_utility - 1e-9]
        # sorted() over the names makes selection order-independent of
        # the underlying dict-iteration order; rng.choice then picks
        # deterministically.
        winners_by_name = sorted(top_tier, key=lambda b: b.bidder)
        return self._rng.choice(winners_by_name)

    def top_bids(
        self,
        target,
        priority: int,
        n: int,
        exclude: Optional[Iterable[str]] = None,
    ) -> List[Bid]:
        """Return up to ``n`` best Bids, descending by utility.

        Used by the witness-handover use case:
        the CONFIRM dispatch picks ``top_bids(victim, 2, n=2)[1]``
        instead of running a second auction with ``exclude=``.
        """
        if n <= 0:
            return []
        bids = self._eligible_bids(target, priority, exclude)
        bids.sort(key=lambda b: -b.utility)
        return bids[:n]

    def bid(
        self,
        target,
        priority: int,
        exclude: Optional[Iterable[str]] = None,
    ) -> Optional[str]:
        """Back-compat shim: returns just the winner's name.

        Wraps ``best_bid()``. New callers should prefer ``best_bid()``
        which returns the full ``Bid`` value object (the utility is
        useful for logging and the witness-handover decision); this
        shim exists so the older mission_manager code paths and the
        unit tests keep working unchanged.
        """
        winner = self.best_bid(target, priority, exclude)
        return winner.bidder if winner is not None else None
