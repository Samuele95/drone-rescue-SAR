"""Regression test for the sloped-terrain scenario.

Elevation was inert because no scenario or launch ever set terrain_slope_x/y.
Those keys were added to the LAUNCH scope of PARAM_SCHEMA along with
config/scenarios/sloped_terrain.yaml. This test pins that the scenario loads
and forwards the gradient (and the matching world) into the ros2 launch args,
so a run on it actually tilts the terrain.
"""

from __future__ import annotations

from pathlib import Path

from drone_rescue_coordination.lib.domain.scenario_schema import (
    ParamScope, keys_for_scope,
)
from drone_rescue_mission_control.scenario_loader import load_scenario

# src/drone_rescue_mission_control/test/ -> src/ -> bringup scenarios.
_SCENARIO = (
    Path(__file__).resolve().parents[2]
    / 'drone_rescue_bringup' / 'config' / 'scenarios' / 'sloped_terrain.yaml'
)


def test_terrain_slope_is_a_launch_scope_param():
    """terrain_slope_x/y must be valid LAUNCH keys so the YAML validates and
    Mission Control's form exposes them."""
    launch_keys = keys_for_scope(ParamScope.LAUNCH)
    assert 'terrain_slope_x' in launch_keys
    assert 'terrain_slope_y' in launch_keys


def test_sloped_scenario_loads():
    s = load_scenario(_SCENARIO)
    assert s.name == 'Sloped Terrain'
    assert s.launch['terrain_slope_x'] == 0.04
    assert s.launch['terrain_slope_y'] == -0.02
    assert s.launch['world'] == 'earthquake_zone_sloped.sdf'


def test_sloped_scenario_forwards_slope_to_launch_args():
    """The gradient + world reach ros2 launch as key:=value overrides, so a run
    on this scenario tilts both the flight model and the Gazebo ground."""
    args = load_scenario(_SCENARIO).launch_args()
    assert args['terrain_slope_x'] == '0.04'
    assert args['terrain_slope_y'] == '-0.02'
    assert args['world'] == 'earthquake_zone_sloped.sdf'
