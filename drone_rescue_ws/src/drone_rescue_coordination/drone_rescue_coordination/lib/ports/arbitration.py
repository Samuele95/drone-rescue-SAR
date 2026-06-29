"""ArbitrationStrategy: swappable basis-behaviour combination rule.

Behaviour-based control (Brooks 1986; Arkin 1998; slides pp. 88-96)
admits several ways of combining a set of basis behaviours into one
command: the motor schema (Arkin) sums their vectors; subsumption
(Brooks) lets higher-priority behaviours suppress lower ones; voting
and learned combinations exist too. The motor-schema vector sum was
hardwired inside ``navigation.motor_schema_blend``, so demonstrating
subsumption meant editing the reactive core.

This Protocol names the combination step so it becomes a strategy.
``Surveyor`` holds an ``ArbitrationStrategy`` and calls ``combine``
on the per-behaviour vectors produced by its ``BehaviourRegistry``;
swapping motor-schema for subsumption (or a learned policy) is then
an injected object, not a code edit.

3T boundary: ``LAYER_BOUNDARY = 'L1'``.
"""

from __future__ import annotations

from typing import Mapping, Protocol, Tuple


LAYER_BOUNDARY = 'L1'   # 3T architecture annotation.


class ArbitrationStrategy(Protocol):
    """Combine named, weighted basis-behaviour vectors into one command.

    ``outputs`` maps each active behaviour's name to its raw 2-D
    vector; ``weights`` maps the same names to blend weights. The
    iteration order of ``outputs`` is the registry's declared order,
    which an order-sensitive strategy (e.g. subsumption priority) may
    rely on. The return is the post-combination command vector:
    unit-magnitude, or ``(0.0, 0.0)`` when the behaviours produce no
    net command.
    """

    def combine(
        self,
        outputs: Mapping[str, Tuple[float, float]],
        weights: Mapping[str, float],
    ) -> Tuple[float, float]:
        ...
