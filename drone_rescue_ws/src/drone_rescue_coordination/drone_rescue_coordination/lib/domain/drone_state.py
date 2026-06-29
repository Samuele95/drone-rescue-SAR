"""DroneState: operational state enum for the drone controller.

Extracted from `drone_controller.py` so consumers in `lib/` and test
code can reference these states without pulling in the rclpy-bound
lifecycle node. Pure-Python; no rclpy.
"""

from __future__ import annotations

from enum import Enum


class DroneState(Enum):
    """Drone operational states (controller-side)."""
    IDLE = 0
    TAKEOFF = 1
    SURVEYING = 2     # Pheromone-based survey navigation
    RETURNING = 3
    LANDING = 4
    HOVER = 5
    NAVIGATING = 6
    EMERGENCY = 7
