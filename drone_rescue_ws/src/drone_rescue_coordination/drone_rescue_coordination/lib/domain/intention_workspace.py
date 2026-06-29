"""IntentionWorkspace: per-drone coordination of competing desires.

Unit-10 places an intention workspace between the organisation
layer's motivations and the behavioural layer: it reconciles desires
into intentions, holds a per-intention path memory of which
behaviour configurations have worked, and consults the model-free
failure detector (``AffectMonitor``) when intentions stop
resolving.

One workspace per drone, the per-robot distribution Unit 10
mandates. The workspace is pure in the strict sense for the
allocation slice: ``reconcile`` is a deterministic function of
``(motivations, ctx, path memory)``. Determinism is the load-bearing
property for the seeded evaluation harness, so motivations are
iterated in fixed registration order and any tie-break is deferred to
the caller's ``RngSource``.

The workspace's only allowed actuation is via the ``BehaviourRegistry``
(set weights / enable / disable) and through its strength readout that
the ``MotivationWorkspaceStrategy`` consumes; it never issues a
motor command directly.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Mapping, Optional, Sequence

from drone_rescue_coordination.lib.ports.affect_monitor import AffectMonitor
from drone_rescue_coordination.lib.ports.motivation import (
    Motivation,
    MotivationContext,
)


# Fraction the per-intention path memory boosts strength per recorded
# success. Small and bounded by an upper cap: enough to bias toward
# proven performers, never enough to drown the proximity term.
_PATH_MEMORY_BOOST_PER_SUCCESS: float = 0.05
_PATH_MEMORY_CAP: float = 0.5    # max +50% boost regardless of streak


@dataclass
class IntentionWorkspace:
    """Per-drone reconciliation of motivations into intention strengths."""

    drone_name: str
    motivations: Sequence[Motivation]
    affect: Optional[AffectMonitor] = None
    _success: Dict[str, int] = field(default_factory=dict)

    def reconcile(self, ctx: MotivationContext) -> Mapping[str, float]:
        """Sum motivation desires into per-intention net strength.

        Iterates motivations in fixed registration order. Applies a
        bounded path-memory bonus per intention key based on this
        drone's past successes.
        """
        net: Dict[str, float] = {}
        for m in self.motivations:
            for d in m.propose(ctx):
                net[d.intention_key] = net.get(d.intention_key, 0.0) + d.strength
        if not self._success:
            return net
        for key in list(net.keys()):
            wins = self._success.get(key, 0)
            if wins <= 0 or net[key] <= 0.0:
                continue   # only boost positive intentions, never inhibitions
            boost = min(_PATH_MEMORY_BOOST_PER_SUCCESS * wins, _PATH_MEMORY_CAP)
            net[key] = net[key] * (1.0 + boost)
        return net

    def strength_for(self, intention_key: str, ctx: MotivationContext) -> float:
        """The reconciled net strength of ``intention_key`` for ``ctx``."""
        return self.reconcile(ctx).get(intention_key, 0.0)

    def record_success(self, intention_key: str) -> None:
        """Per-intention path memory: tally a successful exploitation.

        Called by the consumer when an intention this workspace voted
        for completed, e.g. the strategy notifies the winning drone's
        workspace after the saga confirms a victim. Failures are
        deliberately not tallied here; the AffectMonitor is the
        path along which "this isn't working" flows.
        """
        self._success[intention_key] = self._success.get(intention_key, 0) + 1

    def successes(self, intention_key: str) -> int:
        return self._success.get(intention_key, 0)
