"""Concrete basis behaviours: first-class wrappers over ``compute_*``.

Each class implements the ``Behaviour`` Protocol
(``lib.ports.behaviour``) and delegates to the corresponding
module-level pure function in ``lib.domain.navigation``. The
``compute_*`` functions are UNCHANGED; these wrappers only adapt the
uniform ``compute(sensors)`` contract onto each function's specific
argument list, holding the construction-time configuration (meshes,
thresholds, geometry) as immutable state.

This is the GoF-Strategy realisation of the slides' "basis
behaviour" (pp. 99-100): a developer adding a sixth behaviour (e.g.
wind-correction, terrain-gradient) writes one class here and
registers it (``lib.domain.behaviour_registry``) without touching the
combination step.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence, Tuple

import numpy as np

from drone_rescue_coordination.lib import grid_utils
from drone_rescue_coordination.lib.domain import navigation
from drone_rescue_coordination.lib.domain.value_objects import Position
from drone_rescue_coordination.lib.ports.surveyor_port import SurveyorSensors


@dataclass(frozen=True)
class GridSpec:
    """Pheromone-grid geometry a grid-reading behaviour needs.

    Carries the origin / resolution / dimensions used to project a
    world position onto a grid cell. Frozen so a behaviour's spatial
    frame is fixed at construction.
    """

    origin_x: float
    origin_y: float
    cell_resolution: float
    grid_width: int
    grid_height: int

    def cell_of(self, pos: Position) -> Tuple[int, int]:
        """Clamp ``pos`` to an in-bounds (row, col) grid cell."""
        row, col = grid_utils.world_to_grid(
            pos.x, pos.y, self.origin_x, self.origin_y, self.cell_resolution,
        )
        row = max(0, min(self.grid_height - 1, row))
        col = max(0, min(self.grid_width - 1, col))
        return row, col


@dataclass(frozen=True)
class RepulsionBehaviour:
    """Behaviour-1: avoid visited areas (wraps ``compute_repulsion``)."""

    mesh: Mapping[str, np.ndarray]
    threshold: float
    grid: GridSpec
    name: str = 'B1-avoid-visited'

    def compute(self, sensors: SurveyorSensors) -> Tuple[float, float]:
        if sensors.current_position is None or sensors.pheromone_grid is None:
            return (0.0, 0.0)
        row, col = self.grid.cell_of(sensors.current_position)
        return navigation.compute_repulsion(
            sensors.pheromone_grid, row, col, self.mesh, self.threshold,
        )


@dataclass(frozen=True)
class AttractionBehaviour:
    """Behaviour-2: explore unvisited areas (wraps ``compute_attraction``)."""

    mesh: Mapping[str, np.ndarray]
    threshold: float
    grid: GridSpec
    name: str = 'B2-explore-unvisited'

    def compute(self, sensors: SurveyorSensors) -> Tuple[float, float]:
        if sensors.current_position is None or sensors.pheromone_grid is None:
            return (0.0, 0.0)
        row, col = self.grid.cell_of(sensors.current_position)
        return navigation.compute_attraction(
            sensors.pheromone_grid, row, col, self.mesh, self.threshold,
        )


@dataclass(frozen=True)
class CollisionAvoidanceBehaviour:
    """Behaviour-3: avoid peers (wraps ``compute_collision_avoidance``)."""

    collision_distance: float
    name: str = 'B3-avoid-peers'

    def compute(self, sensors: SurveyorSensors) -> Tuple[float, float]:
        if sensors.current_position is None:
            return (0.0, 0.0)
        return navigation.compute_collision_avoidance(
            sensors.current_position, sensors.peer_positions,
            self.collision_distance,
        )


@dataclass(frozen=True)
class BoundaryRepulsionBehaviour:
    """Behaviour-4: stay inside the disk (wraps ``compute_boundary_repulsion``)."""

    center_x: float
    center_y: float
    radius: float
    name: str = 'B4-stay-inside'

    def compute(self, sensors: SurveyorSensors) -> Tuple[float, float]:
        if sensors.current_position is None:
            return (0.0, 0.0)
        return navigation.compute_boundary_repulsion(
            sensors.current_position, self.center_x, self.center_y,
            self.radius,
        )


@dataclass(frozen=True)
class VictimAttractionBehaviour:
    """Behaviour-5: goal-seek victims (wraps ``compute_victim_attraction``)."""

    attraction_radius: float
    confirm_hover_radius: float
    ttl: float
    name: str = 'B5-goal-seek'

    def compute(self, sensors: SurveyorSensors) -> Tuple[float, float]:
        if sensors.current_position is None:
            return (0.0, 0.0)
        return navigation.compute_victim_attraction(
            sensors.current_position, sensors.hotspots,
            self.attraction_radius, self.confirm_hover_radius,
            sensors.now_sec, self.ttl,
        )


# Canonical declared order of the basis-behaviour set (slides p. 100):
# avoid-visited, explore-unvisited, avoid-peers, stay-inside, goal-seek.
# The registry preserves this order so the motor-schema vector sum is
# bit-identical to the legacy positional ``motor_schema_blend`` call.
BASIS_BEHAVIOUR_NAMES: Sequence[str] = (
    'B1-avoid-visited',
    'B2-explore-unvisited',
    'B3-avoid-peers',
    'B4-stay-inside',
    'B5-goal-seek',
)
