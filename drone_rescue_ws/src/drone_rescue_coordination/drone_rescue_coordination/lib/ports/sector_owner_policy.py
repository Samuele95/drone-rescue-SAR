"""SectorOwnerPolicy: L3 sector-ownership lookup port.

The deliberative planner (``Mission.plan``) gives the sector-owning
drone first refusal on a victim that falls inside its angular wedge,
only opening a cross-sector auction when the owner is unavailable.
This port names that lookup so the planner stays free of the
``lib/sector_geometry`` + mission-center plumbing the concrete
implementation needs.

Concrete production implementation: an adapter in ``mission_manager``
wrapping the existing ``_sector_owner_for`` helper. Tests inject a
fake returning a fixed owner (or ``None`` for the non-angular path).

3T boundary: ``LAYER_BOUNDARY = 'L3-internal'``; this is a
planner-internal policy, not a layer-crossing boundary, marked as
L3-owned.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Protocol

if TYPE_CHECKING:
    from ..domain.value_objects import Position


LAYER_BOUNDARY = 'L3-internal'   # 3T architecture annotation.


class SectorOwnerPolicy(Protocol):
    """Which drone owns the angular sector containing ``p``?

    Returns the owning drone's name, or ``None`` when the coverage
    pattern is non-angular (every wedge zero-width) so no drone has
    sector ownership, in which case the planner auctions directly.
    """

    def owner_for(self, p: 'Position') -> Optional[str]:
        ...
