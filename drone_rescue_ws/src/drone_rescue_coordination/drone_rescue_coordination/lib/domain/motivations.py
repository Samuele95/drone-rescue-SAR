"""Concrete motivations: the organisation-layer goal producers.

These are the slice of Unit-10 "motivations" the project ships as the
worked example: three independent, pure, additively-extensible
processes whose desires get reconciled by an ``IntentionWorkspace``.

- ``VictimMotivation``: positive desire to investigate a confirmed
  victim, scaled by ``priority / max(distance, 1)`` so the slice is
  strength-equivalent to the existing greedy-auction utility when no
  other motivation fires. This is the baseline that lets sweeps show
  the workspace strategy matches greedy in the symmetric case.
- ``CoverageMotivation``: mild inhibition of investigation,
  parameterised by ``coverage_pull``. Default zero (no inhibition);
  raising it biases the workspace toward continued scanning.
- ``SafetyMotivation``: inhibits investigation in proportion to the
  drone's frustration / stuck signal from ``AffectMonitor``. The
  Unit-10 "emotion-modulates-decision" link, expressed as a desire
  rather than a side channel.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from drone_rescue_coordination.lib.ports.motivation import (
    Desire,
    MotivationContext,
)


INVESTIGATE: str = 'investigate'


@dataclass(frozen=True)
class VictimMotivation:
    """Pull toward investigating a confirmed victim."""

    name: str = 'victim'

    def propose(self, ctx: MotivationContext) -> Iterable[Desire]:
        if ctx.target is None or ctx.target_priority <= 0:
            return ()
        if ctx.is_down or not ctx.battery_ok:
            return ()
        dist = ctx.distance_to_target
        if dist is None:
            return ()
        dist = max(dist, 1.0)
        strength = float(ctx.target_priority) / dist
        return (Desire(INVESTIGATE, strength, self.name),)


@dataclass(frozen=True)
class CoverageMotivation:
    """Mild bias toward staying on the scan rather than breaking off.

    Default ``coverage_pull=0.0`` is the no-bias setting so the slice
    is greedy-equivalent out of the box; sweeps can crank it up to
    show emergent specialisation (drones that have not yet broken off
    favour completing their pattern).
    """

    name: str = 'coverage'
    coverage_pull: float = 0.0

    def propose(self, ctx: MotivationContext) -> Iterable[Desire]:
        if ctx.target is None or self.coverage_pull <= 0.0:
            return ()
        return (Desire(INVESTIGATE, -self.coverage_pull, self.name),)


@dataclass(frozen=True)
class SafetyMotivation:
    """Inhibit investigation when the drone's affect monitor reports stress.

    Reads the drone's own ``investigate:<drone>`` frustration from the
    injected ``AffectMonitor``, so a drone whose recent investigations
    keep stalling will progressively self-de-prioritise, letting fresher
    teammates win. The 0..1 frustration is scaled by
    ``frustration_inhibition_scale`` to be commensurate with
    ``VictimMotivation``'s strength.
    """

    name: str = 'safety'
    frustration_inhibition_scale: float = 10.0

    def propose(self, ctx: MotivationContext) -> Iterable[Desire]:
        if ctx.target is None or ctx.affect is None:
            return ()
        f = ctx.affect.frustration(f'investigate:{ctx.drone_name}')
        if f <= 0.0 or self.frustration_inhibition_scale <= 0.0:
            return ()
        if math.isnan(f) or math.isinf(f):   # never inhibit on garbage input
            return ()
        return (Desire(INVESTIGATE, -f * self.frustration_inhibition_scale, self.name),)


DEFAULT_MOTIVATIONS = (
    VictimMotivation(),
    CoverageMotivation(),
    SafetyMotivation(),
)
