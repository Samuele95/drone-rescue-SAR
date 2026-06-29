"""Fleet-level constants: pure-Python, no rclpy.

Canonical home for the fleet-size default that the coordination
LifecycleNodes consume in their ``declare_parameter`` defaults. The
same literal sat hand-typed in 14 sites; lifting it behind
``DEFAULT_DRONE_NAMES`` makes a fleet-size change a one-row edit.
Dashboard / viz / mission_recorder consume the equivalent constant
from ``drone_rescue_ui_common.constants``; this module exists so the
coordination package doesn't need a cross-package import to UI for
one literal.
"""

from __future__ import annotations

from typing import Tuple


DEFAULT_DRONE_NAMES: Tuple[str, ...] = ('drone1', 'drone2', 'drone3', 'drone4')


def default_drone_names_list() -> list:
    """Mutable list copy: ROS `declare_parameter` defaults reject
    tuples in some rclpy versions."""
    return list(DEFAULT_DRONE_NAMES)
