"""Selectable task-allocation strategies: pure Python, no rclpy.

The mission_manager assigns each confirmed-victim INVESTIGATE/CONFIRM task to a
drone. Historically that assignment was hard-wired to a single greedy auction
(``lib/auction.py``). This module turns the *allocation policy* into a
selectable, sweepable strategy, mirroring the ``CoveragePatternFactory``
design in ``lib/sar_patterns.py``, so missions can compare allocation
algorithms the same way they already compare coverage patterns.

Strategies (registered names):

- ``greedy_auction`` (default): wraps the existing ``AuctionEngine`` unchanged.
  utility = priority / distance, nearest eligible drone wins. Keeps every
  existing auction test and sweep result reproducible.
- ``round_robin``: distance-agnostic rotation over the fleet. An honest
  no-coordination baseline (the allocation analogue of ``random_walk``).
- ``hungarian``: optimal assignment via ``scipy.optimize.linear_sum_assignment``.
  The mission_manager dispatches one candidate per tick, so for a single
  target this is provably identical to ``greedy_auction`` (the optimal
  assignment of 1 task is the min-cost drone). It becomes distinct only when
  several candidates are assigned jointly; see the ``assign`` batch method;
  exercising that needs ``max_concurrent_investigations > 1`` and a batch
  dispatch path in mission_manager (documented limitation).

All strategies share ``top_bids`` (used for witness-handover runner-up
selection); only the primary ``bid`` winner differs.
"""

from __future__ import annotations

import inspect
from typing import (
    Dict, Iterable, List, Optional, Protocol, Type, runtime_checkable,
)

from .auction import AuctionEngine, Bid

# Large sentinel for "give me every eligible bidder" via top_bids.
_ALL = 1_000_000


@runtime_checkable
class AllocationBidder(Protocol):
    """Structural type of a task-allocation strategy: picks the winning
    drone for a single target.

    The extension boundary is a ``typing.Protocol`` for
    consistency with the rest of ``lib/`` (BidderProtocol in auction.py,
    the bt.py callable protocols, etc.); ``AllocationStrategy`` below is
    the concrete shared base, not the type other modules refer to.
    """
    name: str

    def bid(self, target, priority: int,
            exclude: Optional[Iterable[str]] = None) -> Optional[str]: ...

    def top_bids(self, target, priority: int, n: int,
                 exclude: Optional[Iterable[str]] = None) -> List[Bid]: ...


@runtime_checkable
class BatchAllocationBidder(AllocationBidder, Protocol):
    """An ``AllocationBidder`` that also assigns several targets jointly,
    minimising total cost. ``mission_manager`` routes the batch INVESTIGATE
    drain through this; ``isinstance(strategy, BatchAllocationBidder)``
    replaces the former ``hasattr(strategy, 'assign')`` duck-typed guard.
    """

    def assign(self, targets: List[object], priority: int,
               exclude: Optional[Iterable[str]] = None
               ) -> List[Optional[str]]: ...


class AllocationStrategy:
    """Concrete shared base: holds an AuctionEngine for eligibility +
    utility scoring.

    Subclasses override only ``bid`` (the winner-selection policy).
    ``top_bids`` is shared: the witness/runner-up is always the
    next-nearest drone, regardless of the primary allocation policy.
    Not an ``abc.ABC``: the *type* callers refer to is the
    ``AllocationBidder`` Protocol above; this class exists only for code
    reuse across the three concrete strategies.
    """

    name: str = "abstract"
    description: str = ""

    def __init__(self, drones, rng):
        self._engine = AuctionEngine(drones, rng)

    def top_bids(self, target, priority: int, n: int,
                 exclude: Optional[Iterable[str]] = None) -> List[Bid]:
        """Up to ``n`` eligible bids, descending by utility (proximity)."""
        return self._engine.top_bids(target, priority, n, exclude)

    def bid(self, target, priority: int,
            exclude: Optional[Iterable[str]] = None) -> Optional[str]:
        """Return the winning drone's name, or None if none eligible.

        Overridden by every concrete strategy; the base raises so an
        un-overridden subclass fails loudly."""
        raise NotImplementedError

    def __repr__(self) -> str:
        # Log-friendly: the mission-startup log reports the active strategy.
        # `name` is a class attribute, so the repr is stable.
        return f'{self.__class__.__name__}(name={self.name!r})'


class GreedyAuctionStrategy(AllocationStrategy):
    """Default: the existing greedy single-item auction, unchanged."""

    name = "greedy_auction"
    description = ("Greedy single-item auction — nearest eligible drone "
                   "(utility = priority / distance). Gerkey & Mataric 2004.")

    def bid(self, target, priority, exclude=None):
        return self._engine.bid(target, priority, exclude)


class RoundRobinStrategy(AllocationStrategy):
    """Distance-agnostic rotation baseline.

    Cycles a cursor over the eligible drones (sorted by name for
    determinism). Ignores proximity entirely: a deliberate worst-case
    coordination baseline, so sweeps can quantify what the greedy/optimal
    policies actually buy.
    """

    name = "round_robin"
    description = ("Round-robin rotation over the fleet, distance-agnostic. "
                   "No-coordination baseline.")

    def __init__(self, drones, rng):
        super().__init__(drones, rng)
        self._cursor = 0

    def bid(self, target, priority, exclude=None):
        # Reuse the engine's eligibility filter; ignore its utility ordering.
        eligible = sorted(b.bidder
                          for b in self._engine.top_bids(
                              target, priority, _ALL, exclude))
        if not eligible:
            return None
        winner = eligible[self._cursor % len(eligible)]
        self._cursor += 1
        return winner


class HungarianStrategy(AllocationStrategy):
    """Optimal assignment via scipy ``linear_sum_assignment``.

    For a single target (the mission_manager's per-tick dispatch) the optimal
    assignment is the min-cost drone, which equals ``greedy_auction``'s
    nearest-drone pick; this is documented and expected. ``assign`` provides
    the genuine batch primitive; it minimises *total* assignment cost across M
    targets and N drones and is where this strategy diverges from greedy.
    """

    name = "hungarian"
    description = ("Optimal (Hungarian) assignment minimising total cost "
                   "via scipy.optimize.linear_sum_assignment.")

    def bid(self, target, priority, exclude=None):
        winners = self.assign([target], priority, exclude)
        return winners[0]

    def assign(self, targets: List[object], priority: int,
               exclude: Optional[Iterable[str]] = None) -> List[Optional[str]]:
        """Jointly assign ``targets`` to drones, minimising total cost.

        Returns a winner name (or None) per target, in input order. A drone
        is assigned at most one target. Cost is ``-utility`` (utility already
        encodes proximity); unassignable targets get None.
        """
        from scipy.optimize import linear_sum_assignment

        # Per-target eligible bids; collect the union of candidate drones.
        per_target = [
            {b.bidder: b.utility
             for b in self._engine.top_bids(t, priority, _ALL, exclude)}
            for t in targets
        ]
        drones = sorted({name for d in per_target for name in d})
        if not drones or not targets:
            return [None] * len(targets)

        # Cost matrix [target x drone]; missing pair = large finite penalty.
        big = 1.0e9
        cost = [[-per_target[i].get(dn, -big) for dn in drones]
                for i in range(len(targets))]
        rows, cols = linear_sum_assignment(cost)

        winners: List[Optional[str]] = [None] * len(targets)
        for r, c in zip(rows, cols):
            if cost[r][c] < big:  # skip penalty (ineligible) pairings
                winners[r] = drones[c]
        return winners


# ----------------------------------------------------------------- factory


class AllocationStrategyFactory:
    """Registry-based factory for AllocationStrategy.

    Adding a strategy: subclass AllocationStrategy, then call
    ``AllocationStrategyFactory.register(YourStrategy)`` at module import.
    Mirrors ``CoveragePatternFactory`` so the two registries are consistent.
    """

    _registry: Dict[str, Type[AllocationStrategy]] = {}

    @classmethod
    def register(cls, strategy_cls: Type[AllocationStrategy]) -> None:
        if not strategy_cls.name or strategy_cls.name == "abstract":
            raise ValueError("strategy must define a non-abstract name")
        cls._registry[strategy_cls.name] = strategy_cls

    @classmethod
    def create(cls, name: str, drones, rng,
               *, affect=None) -> AllocationBidder:
        """Construct the named strategy.

        ``affect`` is the L2/L3 ``AffectMonitor``. Only strategies whose
        ``__init__`` accepts an ``affect`` parameter (today:
        ``MotivationWorkspaceStrategy``) receive it; the rest ignore it
        by construction. This is the bridge the frustration-feedback loop
        needs: without it ``SafetyMotivation`` reads ``ctx.affect=None``
        and silently emits no inhibition.
        """
        if name not in cls._registry:
            raise ValueError(
                f"unknown allocation strategy '{name}'. "
                f"Available: {sorted(cls._registry.keys())}"
            )
        strategy_cls = cls._registry[name]
        extra = {}
        if 'affect' in inspect.signature(strategy_cls).parameters:
            extra['affect'] = affect
        return strategy_cls(drones, rng, **extra)

    @classmethod
    def list_names(cls) -> List[str]:
        return sorted(cls._registry.keys())

    @classmethod
    def describe(cls, name: str) -> str:
        return cls._registry[name].description if name in cls._registry else ""


class MotivationWorkspaceStrategy(AllocationStrategy):
    """Distributed-goal Unit-10 allocator: each drone has its own
    ``IntentionWorkspace`` reconciling competing motivations; the
    strategy collects per-drone investigation strengths and picks the
    maximum, with deterministic tie-break through the seeded RNG.

    Coexists with ``greedy_auction`` in the factory, selectable per
    scenario, so a sweep can compare emergent distributed-goal
    behaviour against the central auction baseline under the same
    seeded harness.

    Determinism: motivations are evaluated in fixed registration order;
    candidate drones are scored in eligibility order from the auction
    engine; ties are broken by sorted name + ``RngSource.choice``,
    mirroring ``AuctionEngine.best_bid``. In the symmetric case (only
    ``VictimMotivation`` contributes, ``CoverageMotivation`` /
    ``SafetyMotivation`` silent) the strategy's strength equals
    ``priority / distance`` (greedy's utility), so its choice
    matches ``greedy_auction``'s under the same RNG seed.

    Distance is recovered from the auction engine's utility
    (``utility = priority / max(distance, 1)``) so the strategy needs
    no engine introspection. ``AffectMonitor`` injection enables
    ``SafetyMotivation``'s frustration-driven inhibition; tests inject
    one directly, production wires it through the composition root.
    """

    name = 'motivation_workspace'
    description = (
        "Per-drone IntentionWorkspace reconciling distributed motivations "
        "(Unit 10): VictimMotivation + CoverageMotivation + "
        "SafetyMotivation (frustration-driven inhibition via AffectMonitor)."
    )

    def __init__(self, drones, rng, *,
                 affect=None, motivations=None):
        super().__init__(drones, rng)
        # Late imports break the lib.ports/lib.domain import cycle.
        from .domain.intention_workspace import IntentionWorkspace
        from .domain.motivations import DEFAULT_MOTIVATIONS, INVESTIGATE
        self._intention_key = INVESTIGATE
        self._rng = rng
        self._affect = affect
        self._motivations = (
            tuple(motivations) if motivations is not None else DEFAULT_MOTIVATIONS
        )
        # Per-drone workspaces; lazily expand for drones added mid-mission
        # via sector handover. The registry's ``candidates()`` is the
        # canonical iteration order the auction engine uses.
        self._workspaces: Dict[str, 'IntentionWorkspace'] = {}
        for b in self._engine._registry.candidates():   # noqa: SLF001
            self._workspaces[b.name] = IntentionWorkspace(
                drone_name=b.name,
                motivations=self._motivations,
                affect=self._affect,
            )

    def _workspace_for(self, drone_name: str):
        ws = self._workspaces.get(drone_name)
        if ws is None:
            from .domain.intention_workspace import IntentionWorkspace
            ws = IntentionWorkspace(
                drone_name=drone_name,
                motivations=self._motivations,
                affect=self._affect,
            )
            self._workspaces[drone_name] = ws
        return ws

    def bid(self, target, priority, exclude=None):
        from .ports.motivation import MotivationContext

        eligible: List[Bid] = self._engine.top_bids(
            target, priority, _ALL, exclude)
        if not eligible:
            return None
        tx, ty = float(target.x), float(target.y)
        # Score each eligible drone via its workspace.
        scored: List[tuple] = []   # (strength, name)
        for b in eligible:
            # Recover distance from utility (auction's monotonic encoding).
            dist = (float(priority) / b.utility) if b.utility > 0 else None
            ctx = MotivationContext(
                drone_name=b.bidder,
                target=(tx, ty),
                target_priority=int(priority),
                distance_to_target=dist,
                battery_ok=True,        # already eligibility-filtered
                is_down=False,
                affect=self._affect,
            )
            strength = self._workspace_for(b.bidder).strength_for(
                self._intention_key, ctx)
            scored.append((strength, b.bidder))
        # Highest strength wins; tie-break by sorted name + seeded RNG
        # choice, exactly as the auction engine breaks ties (lib/auction.py).
        scored.sort(key=lambda s: (-s[0], s[1]))
        top_strength = scored[0][0]
        ties = sorted(name for s, name in scored
                      if abs(s - top_strength) <= 1e-9)
        if len(ties) == 1:
            return ties[0]
        return self._rng.choice(ties)

    # Hook: the strategy's owner feeds path memory back in.
    def on_intention_succeeded(self, drone_name: str) -> None:
        """Notify the winning drone's workspace that its investigation
        succeeded; updates per-intention path memory. The
        mission_manager / saga owner is the natural caller; tests
        invoke this directly."""
        self._workspace_for(drone_name).record_success(self._intention_key)


for _strategy in (GreedyAuctionStrategy, RoundRobinStrategy,
                  HungarianStrategy, MotivationWorkspaceStrategy):
    AllocationStrategyFactory.register(_strategy)
