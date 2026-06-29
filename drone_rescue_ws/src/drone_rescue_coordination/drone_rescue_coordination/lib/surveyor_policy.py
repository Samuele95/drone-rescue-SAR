"""Surveyor policy: pure-Python scatter table + stuck recovery thresholds.

Extracted from `surveyor.py:144-162, 220-231`
so the tactical-navigation parameters are unit-testable without
rclpy and adding a new scatter direction or tuning a stuck-recovery
threshold is a one-edit lib change.

Pure-Python algorithm core in lib/ with no rclpy dependency
(siblings: lib/auction.py, lib/sector_geometry.py, lib/pid.py).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


# Quadrant bias by drone index: drone1 NW, drone2 NE, drone3 SW,
# drone4 SE, then 5..8 fill in cardinal directions for larger fleets.
_SCATTER_TABLE: Tuple[Tuple[float, float], ...] = (
    (-1.0,  1.0),   # drone1 NW
    ( 1.0,  1.0),   # drone2 NE
    (-1.0, -1.0),   # drone3 SW
    ( 1.0, -1.0),   # drone4 SE
    (-1.0,  0.0),   # drone5 W
    ( 1.0,  0.0),   # drone6 E
    ( 0.0,  1.0),   # drone7 N
    ( 0.0, -1.0),   # drone8 S
)


def scatter_direction_for(drone_name: str) -> Tuple[float, float]:
    """Resolve the per-drone scatter direction unit vector.

    Parses the drone name's digit suffix (`drone1`, `drone2`, ...) and
    looks up the scatter table. Names without digits fall through to
    index 0 (NW). Index clamps to the table size so a 9th drone in a
    future fleet doesn't IndexError: it falls back to the last
    cardinal direction.
    """
    try:
        idx = int(''.join(c for c in drone_name if c.isdigit())) - 1
    except ValueError:
        idx = 0
    idx = max(0, min(idx, len(_SCATTER_TABLE) - 1))
    sx, sy = _SCATTER_TABLE[idx]
    magnitude = math.hypot(sx, sy) or 1.0
    return (sx / magnitude, sy / magnitude)


@dataclass(frozen=True)
class StuckRecoveryPolicy:
    """Stuck-detection thresholds + recovery escalation.

    `stuck_threshold_s`: seconds of no movement before recovery
    triggers. `stuck_max_retries`: how many escalations before
    abort. `stuck_altitude_increase_m`: vertical step per retry,
    so each recovery attempt climbs above the previous obstacle.
    """
    stuck_threshold_s: float = 30.0
    stuck_max_retries: int = 3
    stuck_altitude_increase_m: float = 2.0

    def escalated_altitude(self, base_altitude: float, retry_index: int) -> float:
        """Altitude to fly at on the `retry_index`-th attempt (0 is
        the first escalation)."""
        return base_altitude + (retry_index + 1) * self.stuck_altitude_increase_m
