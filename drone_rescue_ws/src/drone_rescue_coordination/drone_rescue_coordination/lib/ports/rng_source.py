"""RngSource driven port.

A structural-typing surface for the seeded RNG the auction tie-break
consumes. Production callers pass ``random.Random(seed)``; tests can
substitute a deterministic stub without importing ``random``.
"""

from __future__ import annotations

from typing import Protocol, Sequence, TypeVar

T = TypeVar('T')


# 3T boundary annotation: seeded RNG abstraction consumed by L3
# (AuctionEngine tie-break) and L1 (sensor degradation noise). Pure
# cross-cutting infra.
LAYER_BOUNDARY = 'cross-cutting'


class RngSource(Protocol):
    """Subset of ``random.Random`` the auction actually uses.

    ``random.Random`` is a structural fit out of the box."""

    def choice(self, seq: Sequence[T]) -> T: ...

    def random(self) -> float: ...
