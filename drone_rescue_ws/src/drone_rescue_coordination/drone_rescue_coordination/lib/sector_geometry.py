"""Sector ownership for the angular-partition coverage strategies.

Pure math, no rclpy. Extracted from MissionManager so the bearing to
drone lookup can be unit-tested without a ROS context.

Used together with the ``CoverageStrategy.sector_type`` declaration in
``lib/sar_patterns.py``: the mission_manager only assigns angular
sector wedges when the strategy is ``sector_type == 'angular'``; for
other partition types every drone keeps ``sector_start_rad ==
sector_end_rad``, the ``sector_owner_for()`` early-exit guard fires,
and the auction falls back to nearest-drone.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional, Protocol, Tuple


class _SectorBidder(Protocol):
    """Structural type: anything with these three attributes works.

    Production: ``mission_manager.DroneRecord``. Tests: SimpleNamespace.
    """
    name: str
    sector_start_rad: float
    sector_end_rad: float


def sector_owner_for(
    point,
    drones: Iterable[_SectorBidder],
    mission_center: Tuple[float, float] = (0.0, 0.0),
) -> Optional[str]:
    """Return the drone whose angular wedge contains ``point``.

    ``point`` only needs ``.x`` and ``.y``. Returns ``None`` if no drone
    has a non-zero wedge (i.e. the strategy isn't angular, or
    ``_begin_scan`` hasn't run yet) or if ``point`` coincides with the
    mission centre (ambiguous bearing).

    Wedges are interpreted on ``[0, 2π)`` matching the assignment in
    ``mission_manager._begin_scan``; bearings in the southern half-plane
    are normalised by adding 2π.
    """
    cx, cy = mission_center
    dx = point.x - cx
    dy = point.y - cy
    if dx * dx + dy * dy < 1e-3:
        return None
    bearing = math.atan2(dy, dx)
    if bearing < 0.0:
        bearing += 2.0 * math.pi
    for d in drones:
        if d.sector_start_rad == d.sector_end_rad:
            continue
        # A wedge with start > end wraps across the
        # 0/2π seam (e.g. start=5.8, end=0.5 covers [5.8, 2π) and [0, 0.5)).
        # The naive `start <= bearing < end` check would silently miss
        # every point in such a wrap wedge. Branch on start vs end so
        # both layouts work.
        if d.sector_start_rad <= d.sector_end_rad:
            inside = d.sector_start_rad <= bearing < d.sector_end_rad
        else:
            inside = (bearing >= d.sector_start_rad
                      or bearing < d.sector_end_rad)
        if inside:
            return d.name
    return None
