"""Surveyor: concrete SurveyorPort implementation.

Composes the pure-function ``lib.domain.navigation`` policy with the
existing ``StuckRecoveryPolicy`` + ``scatter_direction_for`` from
``lib.surveyor_policy``. Rclpy-free: the legacy ``surveyor.Surveyor``
LifecycleNode is the thin adapter that builds ``SurveyorSensors`` and
publishes ``SurveyorOutputs``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from drone_rescue_coordination.lib.domain import navigation
from drone_rescue_coordination.lib.domain.arbitration import (
    MotorSchemaArbitration,
)
from drone_rescue_coordination.lib.domain.behaviour_registry import (
    BehaviourRegistry, default_registry,
)
from drone_rescue_coordination.lib.domain.value_objects import Position
from drone_rescue_coordination.lib.ports.arbitration import ArbitrationStrategy
from drone_rescue_coordination.lib.ports.surveyor_port import (
    SurveyorOutputs, SurveyorPort, SurveyorSensors,
)
from drone_rescue_coordination.lib.surveyor_policy import StuckRecoveryPolicy


@dataclass(frozen=True)
class SurveyorWeights:
    """Per-tick blend weights, frozen so the policy is reproducible.

    Lifted out of the ``Surveyor`` constructor so tests can construct
    a policy with synthetic weights without touching ROS params.
    """
    repulsion: float = 1.0
    attraction: float = 1.0
    collision_avoidance: float = 1.5
    boundary_repulsion: float = 1.2
    victim_attraction: float = 1.4


@dataclass(frozen=True)
class SurveyorThresholds:
    """Thresholds + radii consumed by the policy. Frozen: tests
    construct a fresh instance with synthetic values."""
    pheromone_repel: float = 0.4
    unexplored_attract: float = 0.1
    collision_avoidance_m: float = 5.0
    victim_attraction_m: float = 25.0
    victim_confirm_hover_m: float = 6.0
    victim_hotspot_ttl_s: float = 30.0
    survey_step_m: float = 1.0
    survey_altitude_m: float = 25.0
    mission_center_x: float = 0.0
    mission_center_y: float = 0.0
    mission_radius_m: float = 100.0
    grid_origin_x: float = -100.0
    grid_origin_y: float = -100.0
    cell_resolution: float = 1.0
    grid_width: int = 200
    grid_height: int = 200


class Surveyor(SurveyorPort):
    """Concrete per-tick navigation reducer.

    State held: the radial mesh caches for the repulsion + attraction
    windows. Everything else flows through ``SurveyorSensors`` each
    tick. The class is otherwise stateless w.r.t. mission progress:
    stuck-detection bookkeeping lives in the LifecycleNode adapter
    so the policy is reusable across multiple drones in tests.
    """

    def __init__(
        self,
        weights: SurveyorWeights,
        thresholds: SurveyorThresholds,
        repulsion_radius: int = 5,
        attraction_radius: int = 20,
        stuck_policy: Optional[StuckRecoveryPolicy] = None,
        registry: Optional[BehaviourRegistry] = None,
        arbitration: Optional[ArbitrationStrategy] = None,
    ):
        self.weights = weights
        self.thresholds = thresholds
        self.stuck_policy = stuck_policy or StuckRecoveryPolicy()
        self._repulsion_mesh = navigation.build_radial_mesh(repulsion_radius)
        self._attraction_mesh = navigation.build_radial_mesh(attraction_radius)
        # Basis behaviours are first-class objects in an ordered,
        # weighted registry. tick() iterates the registry instead of
        # calling five hardcoded functions; default_registry() rebuilds
        # the canonical set from the weights/thresholds/meshes so an
        # injected registry can add, drop, or reweight behaviours.
        self._registry = registry or default_registry(
            weights, thresholds, self._repulsion_mesh, self._attraction_mesh,
        )
        # The combination rule is a swappable strategy.
        # MotorSchemaArbitration is the production default (the legacy
        # weighted vector sum); a SubsumptionArbitration can be
        # injected instead.
        self._arbitration = arbitration or MotorSchemaArbitration()

    # ------------------------------------------------------------- API
    def tick(self, sensors: SurveyorSensors) -> SurveyorOutputs:
        """Reduce per-tick sensors into outputs.

        Returns empty outputs when prerequisites are missing (no
        current position, no pheromone grid). Stuck recovery + RTB
        decisions are the LifecycleNode adapter's responsibility;
        this reducer just produces the next-step navigation target.
        """
        if sensors.current_position is None or sensors.pheromone_grid is None:
            return SurveyorOutputs()

        pos = sensors.current_position
        thr = self.thresholds

        # Evaluate each registered basis behaviour against the sensor
        # snapshot. Grid-reading behaviours project the world position
        # onto a cell themselves (GridSpec), so the row/col clamp the
        # legacy tick computed inline now lives in each behaviour.
        entries = self._registry.build_active()
        outputs = {
            e.behaviour.name: e.behaviour.compute(sensors) for e in entries
        }
        weights = {e.behaviour.name: e.weight for e in entries}

        # Combine the basis behaviours through the injected
        # ArbitrationStrategy. The default MotorSchemaArbitration
        # reproduces the slides' weighted vector sum (pp. 88-90)
        # bit-for-bit, iterating the registry's canonical order;
        # injecting SubsumptionArbitration swaps the combination rule
        # with no edit here.
        nav = self._arbitration.combine(outputs, weights)
        if nav == (0.0, 0.0):
            return SurveyorOutputs()

        target_x = pos.x + nav[0] * thr.survey_step_m
        target_y = pos.y + nav[1] * thr.survey_step_m

        # Clamp to grid bounds (matches legacy publish_target).
        min_x = thr.grid_origin_x
        max_x = thr.grid_origin_x + thr.grid_width * thr.cell_resolution
        min_y = thr.grid_origin_y
        max_y = thr.grid_origin_y + thr.grid_height * thr.cell_resolution
        target_x = max(min_x, min(max_x, target_x))
        target_y = max(min_y, min(max_y, target_y))

        return SurveyorOutputs(
            target_pose=Position(target_x, target_y, thr.survey_altitude_m),
        )
