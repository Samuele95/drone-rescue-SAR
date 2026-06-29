"""Motivation: first-class organisation-layer goal-producer port.

The slides (Marcelletti, "Autonomous and Collaborative Robotics",
A.Y. 2025/26, Unit 10) place a hybrid behaviour-based architecture's
top layer as a set of distributed, asynchronous "motivations" that
manifest competing desires (to satisfy or inhibit intentions),
rather than a single central planner. Desires are then reconciled by
a coordination layer (the intention workspace) into intentions that
configure the behaviours below, never issuing motor commands
directly.

This port names a single motivation. The contract is deliberately
narrow: given a per-drone, per-target ``MotivationContext``, the
motivation proposes zero or more ``Desire``\\s. The reconciliation,
the path memory, and the activation of behaviours all live in the
``IntentionWorkspace`` consumer.

Distribution here is topological, not wall-clock: motivations
behave as independent processes (each one only reads the local
context, no shared mutable state, no central decision), which makes
them re-orderable, additively extensible, and trivially testable;
but they are evaluated in fixed sorted order each tick so the
reproducible evaluation harness stays deterministic.

3T boundary: ``LAYER_BOUNDARY = 'L3-organisation'``, the Unit-10
organisation layer that sits above the deliberative planner and
feeds the intention workspace.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Iterable, Optional, Protocol, Tuple

if TYPE_CHECKING:
    from .affect_monitor import AffectMonitor


LAYER_BOUNDARY = 'L3-organisation'   # 3T architecture annotation.


@dataclass(frozen=True)
class Desire:
    """One motivation's vote toward (or against) an intention.

    ``intention_key`` names the intention this desire pulls on
    (e.g. ``'investigate'``). ``strength`` is positive to satisfy /
    pull toward the intention and negative to inhibit it:
    Unit 10's explicit "inhibition is a first-class desire" property.
    ``source`` is the motivation's name, kept for diagnostics so a
    reconciled intention can be traced back to who voted for it.
    """

    intention_key: str
    strength: float
    source: str


@dataclass(frozen=True)
class MotivationContext:
    """Per-drone, per-target snapshot handed to each motivation.

    Frozen and small on purpose: motivations must remain pure functions
    of their context, so reordering them or running them in parallel
    cannot change the system's behaviour.

    ``distance_to_target`` is supplied by the strategy (recovered from
    the auction's utility so no engine introspection is needed) when
    the context concerns a victim target; for non-target contexts it
    is ``None``.
    """

    drone_name: str
    target: Optional[Tuple[float, float]] = None
    target_priority: int = 0
    distance_to_target: Optional[float] = None
    battery_ok: bool = True
    is_down: bool = False
    affect: Optional['AffectMonitor'] = None


class Motivation(Protocol):
    """A first-class organisation-layer process producing desires.

    Implementations are pure with respect to their ``MotivationContext``:
    they hold construction-time tuning parameters but no per-call
    state. The intention workspace evaluates them in registration order
    each tick.
    """

    name: str

    def propose(self, ctx: MotivationContext) -> Iterable[Desire]:
        """Yield the desires this motivation produces for ``ctx``.

        Return an empty iterable when the motivation has nothing to
        contribute (no stimulus, irrelevant target, gated by the
        drone's state). May yield more than one desire when a single
        motivation pulls on multiple intentions.
        """
        ...
