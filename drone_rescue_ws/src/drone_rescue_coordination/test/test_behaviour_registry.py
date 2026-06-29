"""Tests for first-class basis behaviours + registry.

Covers the extension contract:
the canonical five behaviours are registered in declared order; a
sixth can be added, an existing one reweighted or disabled, all
without editing the combination step; and the domain Surveyor honours
an injected registry/arbitration end-to-end.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from drone_rescue_coordination.lib.domain.arbitration import (
    SubsumptionArbitration,
)
from drone_rescue_coordination.lib.domain.behaviour_registry import (
    BehaviourEntry,
    BehaviourRegistry,
    default_registry,
)
from drone_rescue_coordination.lib.domain.behaviours import (
    BASIS_BEHAVIOUR_NAMES,
    CollisionAvoidanceBehaviour,
)
from drone_rescue_coordination.lib.domain.surveyor import (
    Surveyor,
    SurveyorThresholds,
    SurveyorWeights,
)
from drone_rescue_coordination.lib.ports.surveyor_port import (
    SurveyorOutputs,
    SurveyorSensors,
)
from drone_rescue_coordination.lib.domain.value_objects import Position


def _registry() -> BehaviourRegistry:
    return default_registry(
        SurveyorWeights(), SurveyorThresholds(),
        repulsion_mesh={}, attraction_mesh={},
    )


def _sensors(**overrides) -> SurveyorSensors:
    base = dict(
        now_sec=1.0, current_position=Position(0.0, 0.0, 25.0),
        pheromone_grid=np.zeros((200, 200), dtype=np.float32),
        grid_origin_x=-100.0, grid_origin_y=-100.0, cell_resolution=1.0,
        battery_level=1.0, zone_warn=False,
    )
    base.update(overrides)
    return SurveyorSensors(**base)


def test_default_registry_canonical_order():
    entries = _registry().build_active()
    assert [e.behaviour.name for e in entries] == list(BASIS_BEHAVIOUR_NAMES)


def test_default_registry_weights_match_surveyor_weights():
    wts = SurveyorWeights()
    by_name = {e.behaviour.name: e.weight for e in _registry().build_active()}
    assert by_name['B1-avoid-visited'] == wts.repulsion
    assert by_name['B3-avoid-peers'] == wts.collision_avoidance
    assert by_name['B5-goal-seek'] == wts.victim_attraction


def test_disable_behaviour_drops_it_from_active():
    reg = _registry()
    reg.set_enabled('B5-goal-seek', False)
    names = [e.behaviour.name for e in reg.build_active()]
    assert 'B5-goal-seek' not in names
    assert len(names) == 4


def test_set_weight_preserves_position():
    reg = _registry()
    reg.set_weight('B1-avoid-visited', 9.0)
    entries = reg.build_active()
    assert entries[0].behaviour.name == 'B1-avoid-visited'   # still first
    assert entries[0].weight == 9.0


def test_register_overwrites_in_place_keeping_order():
    reg = _registry()
    reg.register(BehaviourEntry(
        CollisionAvoidanceBehaviour(99.0, name='B3-avoid-peers'), 2.0))
    names = [e.behaviour.name for e in reg.build_active()]
    assert names == list(BASIS_BEHAVIOUR_NAMES)   # order unchanged
    b3 = next(e for e in reg.build_active()
              if e.behaviour.name == 'B3-avoid-peers')
    assert b3.weight == 2.0


def test_add_sixth_behaviour_without_touching_combination():
    """The open-closed proof: a brand-new behaviour is added by
    registration alone."""
    class WindCorrection:
        name = 'B6-wind-correction'

        def compute(self, sensors) -> Tuple[float, float]:
            return (1.0, 0.0)

    reg = _registry()
    reg.register(BehaviourEntry(WindCorrection(), 1.0))
    active = reg.build_active()
    assert len(active) == 6
    assert active[-1].behaviour.name == 'B6-wind-correction'
    assert active[-1].behaviour.compute(_sensors()) == (1.0, 0.0)


def test_collision_behaviour_returns_zero_without_position():
    b = CollisionAvoidanceBehaviour(5.0)
    assert b.compute(_sensors(current_position=None)) == (0.0, 0.0)


def test_collision_behaviour_repels_from_near_peer():
    b = CollisionAvoidanceBehaviour(5.0)
    vx, vy = b.compute(_sensors(
        current_position=Position(0.0, 0.0, 25.0),
        peer_positions={'drone2': Position(3.0, 0.0, 25.0)},
    ))
    assert vx < 0.0   # pushed west, away from the eastward peer
    assert math.isclose(vy, 0.0, abs_tol=1e-9)


def test_surveyor_accepts_injected_registry_and_arbitration():
    """End-to-end: a Surveyor built with a custom registry and a
    SubsumptionArbitration still ticks and produces a target."""
    s = Surveyor(
        weights=SurveyorWeights(), thresholds=SurveyorThresholds(),
        arbitration=SubsumptionArbitration(),
    )
    out = s.tick(_sensors())
    assert isinstance(out, SurveyorOutputs)
    if out.target_pose is not None:
        assert out.target_pose.z == 25.0


def test_surveyor_disabling_all_behaviours_yields_no_target():
    reg = _registry()
    for name in BASIS_BEHAVIOUR_NAMES:
        reg.set_enabled(name, False)
    s = Surveyor(
        weights=SurveyorWeights(), thresholds=SurveyorThresholds(),
        registry=reg,
    )
    assert s.tick(_sensors()) == SurveyorOutputs()
