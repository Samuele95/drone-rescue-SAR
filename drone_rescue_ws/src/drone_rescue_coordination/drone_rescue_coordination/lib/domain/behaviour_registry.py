"""BehaviourRegistry: ordered, weighted catalogue of basis behaviours.

The registry is the extension point of the motor-schema reactive
layer (Abstract Factory + Strategy patterns, open-closed). Each entry
pairs a ``Behaviour`` with its blend weight and an enabled flag.
``Surveyor.tick`` iterates ``build_active()`` instead of calling five
hardcoded functions, so a behaviour can be added (``register``),
removed (``enabled=False``), or reweighted without editing the tick
body or the combination step.

``default_registry`` builds the canonical five-behaviour set from the
existing ``SurveyorWeights`` / ``SurveyorThresholds`` value objects
and the prebuilt radial meshes, preserving the declared order so the
resulting vector sum is bit-identical to the legacy positional
``motor_schema_blend`` call.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Dict, List, Mapping

import numpy as np

from drone_rescue_coordination.lib.domain.behaviours import (
    AttractionBehaviour,
    BoundaryRepulsionBehaviour,
    CollisionAvoidanceBehaviour,
    GridSpec,
    RepulsionBehaviour,
    VictimAttractionBehaviour,
)
from drone_rescue_coordination.lib.ports.behaviour import Behaviour


@dataclass(frozen=True)
class BehaviourEntry:
    """A registered basis behaviour with its blend weight."""

    behaviour: Behaviour
    weight: float
    enabled: bool = True


class BehaviourRegistry:
    """Ordered, named catalogue of basis behaviours.

    Insertion order is preserved (it determines the vector-sum order).
    ``register`` overwrites an existing entry of the same name in
    place, keeping its position; this lets a caller reweight or
    disable a behaviour without disturbing the order.
    """

    def __init__(self) -> None:
        self._entries: Dict[str, BehaviourEntry] = {}

    def register(self, entry: BehaviourEntry) -> None:
        """Add or replace the entry keyed by ``entry.behaviour.name``."""
        self._entries[entry.behaviour.name] = entry

    def set_weight(self, name: str, weight: float) -> None:
        """Reweight a registered behaviour, preserving its position."""
        self._entries[name] = replace(self._entries[name], weight=weight)

    def set_enabled(self, name: str, enabled: bool) -> None:
        """Enable or disable a registered behaviour."""
        self._entries[name] = replace(self._entries[name], enabled=enabled)

    def build_active(self) -> List[BehaviourEntry]:
        """Return the enabled entries in registration order."""
        return [e for e in self._entries.values() if e.enabled]


def default_registry(
    weights,
    thresholds,
    repulsion_mesh: Mapping[str, np.ndarray],
    attraction_mesh: Mapping[str, np.ndarray],
) -> BehaviourRegistry:
    """Build the canonical five-behaviour registry.

    ``weights`` is a ``SurveyorWeights`` and ``thresholds`` a
    ``SurveyorThresholds`` (lib.domain.surveyor); they are typed
    loosely here to avoid an import cycle with that module.
    """
    grid = GridSpec(
        origin_x=thresholds.grid_origin_x,
        origin_y=thresholds.grid_origin_y,
        cell_resolution=thresholds.cell_resolution,
        grid_width=thresholds.grid_width,
        grid_height=thresholds.grid_height,
    )
    registry = BehaviourRegistry()
    registry.register(BehaviourEntry(
        RepulsionBehaviour(repulsion_mesh, thresholds.pheromone_repel, grid),
        weights.repulsion,
    ))
    registry.register(BehaviourEntry(
        AttractionBehaviour(
            attraction_mesh, thresholds.unexplored_attract, grid,
        ),
        weights.attraction,
    ))
    registry.register(BehaviourEntry(
        CollisionAvoidanceBehaviour(thresholds.collision_avoidance_m),
        weights.collision_avoidance,
    ))
    registry.register(BehaviourEntry(
        BoundaryRepulsionBehaviour(
            thresholds.mission_center_x, thresholds.mission_center_y,
            thresholds.mission_radius_m,
        ),
        weights.boundary_repulsion,
    ))
    registry.register(BehaviourEntry(
        VictimAttractionBehaviour(
            thresholds.victim_attraction_m, thresholds.victim_confirm_hover_m,
            thresholds.victim_hotspot_ttl_s,
        ),
        weights.victim_attraction,
    ))
    return registry
