"""Flight-plan feasibility: pure Python, no rclpy.

The platform had no flight-plan-feasibility concept anywhere: nothing checked
whether a drone's remaining battery endurance is enough to finish its scan plan
and still return home. This module supplies that check as a pure function over
data mission_manager already holds (remaining scan waypoints, a survey speed,
and the per-drone battery endurance from DroneHealth.battery_remaining_s).

A plan is feasible when the time to fly the remaining waypoints plus the return
leg, at the survey speed, fits inside the remaining endurance with a safety
reserve. The signed ``margin_s`` (endurance minus need) is the go/no-go headroom.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass(frozen=True)
class FlightPlanFeasibility:
    """Go/no-go verdict for one drone's remaining plan.

    ``margin_s`` is endurance minus (time-needed + reserve): positive means
    feasible with that many seconds of headroom; negative means infeasible by
    |margin_s|.
    """
    drone_name: str
    feasible: bool
    margin_s: float
    time_needed_s: float
    endurance_s: float


def remaining_plan_length(start_xy: Tuple[float, float], waypoints: Sequence) -> float:
    """Path length (m) from ``start_xy`` through the remaining ``waypoints``.

    Each waypoint exposes ``.x`` / ``.y`` (a geometry_msgs/Point or any
    duck-typed equivalent). Returns 0.0 for an empty remaining plan.
    """
    total = 0.0
    px, py = start_xy
    for wp in waypoints:
        total += math.hypot(wp.x - px, wp.y - py)
        px, py = wp.x, wp.y
    return total


def assess_feasibility(
    *,
    drone_name: str,
    remaining_plan_m: float,
    return_home_m: float,
    speed_mps: float,
    endurance_s: float,
    reserve_s: float = 0.0,
) -> FlightPlanFeasibility:
    """Assess whether ``endurance_s`` covers the remaining plan + return leg at
    ``speed_mps``, keeping ``reserve_s`` in hand."""
    time_needed_s = (remaining_plan_m + return_home_m) / max(speed_mps, 1e-6)
    margin_s = endurance_s - time_needed_s - reserve_s
    return FlightPlanFeasibility(
        drone_name=drone_name,
        feasible=margin_s >= 0.0,
        margin_s=margin_s,
        time_needed_s=time_needed_s,
        endurance_s=endurance_s,
    )
