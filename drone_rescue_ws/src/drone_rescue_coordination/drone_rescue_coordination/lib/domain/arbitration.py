"""Concrete arbitration strategies for the motor-schema reactive layer.

``MotorSchemaArbitration`` is the production default: the weighted
vector sum + normalise that ``navigation.motor_schema_blend`` has
always performed (Arkin's motor schema, slides p. 89). It is written
to iterate the behaviour outputs in the registry's declared order, so
its result is bit-identical to the legacy fixed-arity blend for the
canonical five-behaviour set.

``SubsumptionArbitration`` is the demonstrator alternative (Brooks
1986, slides p. 90): instead of summing, the highest-priority
behaviour that produces a non-negligible command suppresses the rest.
It exists to prove the seam: both a motor-schema and a subsumption
controller can be presented as interchangeable
``ArbitrationStrategy`` implementations.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Tuple


@dataclass(frozen=True)
class MotorSchemaArbitration:
    """Weighted vector sum + normalise (Arkin motor schema).

    Equivalent to ``navigation.motor_schema_blend`` for the canonical
    behaviour set; summation follows ``outputs`` iteration order so
    the floating-point result matches the legacy positional blend.
    """

    def combine(
        self,
        outputs: Mapping[str, Tuple[float, float]],
        weights: Mapping[str, float],
    ) -> Tuple[float, float]:
        # Accumulate with ``sum()`` over generators in registry order,
        # exactly as the legacy ``navigation.motor_schema_blend`` does,
        # so the floating-point result is bit-identical (a manual +=
        # loop differs from sum() by ~1 ULP for some inputs).
        items = list(outputs.items())
        sx = sum(v[0] * weights.get(name, 0.0) for name, v in items)
        sy = sum(v[1] * weights.get(name, 0.0) for name, v in items)
        mag = math.hypot(sx, sy)
        if mag < 1e-9:
            return (0.0, 0.0)
        return (sx / mag, sy / mag)


@dataclass(frozen=True)
class SubsumptionArbitration:
    """Priority-ordered suppression (Brooks subsumption).

    Behaviours are tried in registry order (highest priority first);
    the first whose raw vector exceeds ``suppression_threshold`` wins
    and suppresses all lower layers, returning its normalised vector.
    Weights are accepted for interface conformance but unused: in
    subsumption, order is priority, not a blend coefficient.
    """

    suppression_threshold: float = 1e-6

    def combine(
        self,
        outputs: Mapping[str, Tuple[float, float]],
        weights: Mapping[str, float],
    ) -> Tuple[float, float]:
        for vx, vy in outputs.values():
            mag = math.hypot(vx, vy)
            if mag > self.suppression_threshold:
                return (vx / mag, vy / mag)
        return (0.0, 0.0)
