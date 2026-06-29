"""PartitionKind: typed declaration of how a CoverageStrategy
partitions the search disk.

Replaces the stringly-typed `sector_type: str = 'full'` sentinel on
CoverageStrategy. Mission_manager branches on this to decide whether
to assign angular sector wedges to each drone.

Closed enum (Effective Java item 23): adding a new partition shape
becomes a new enum value, surfacing every consumer that needs updating
at static-check time instead of silently widening a string set.
"""

from __future__ import annotations

from enum import Enum


class PartitionKind(str, Enum):
    """How a CoverageStrategy partitions the disk across drones."""
    ANGULAR = 'angular'   # pie-wedge per drone (spiral_in / spiral_out)
    STRIP = 'strip'       # parallel strips (parallel_track)
    FULL = 'full'         # whole disk per drone (expanding_square / sector_search / random_walk)
