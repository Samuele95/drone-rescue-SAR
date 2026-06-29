"""Tests for the PARAM_SCHEMA single source of truth.

Asserts that the four hand-written sites now derive from
PARAM_SCHEMA, and that the schema covers every key the legacy
hand-written lists tracked.
"""

from __future__ import annotations

from drone_rescue_coordination.lib.domain.scenario_schema import (
    PARAM_SCHEMA, ParamScope,
    declare_args_for_scope, form_schema_for_scope,
    keys_for_scope, runtime_tweakable_for_scope,
)


def test_keys_for_scope_returns_named_params():
    launch = keys_for_scope(ParamScope.LAUNCH)
    mission = keys_for_scope(ParamScope.MISSION)
    detection = keys_for_scope(ParamScope.DETECTION)
    # Every scope contributes at least one parameter.
    assert 'num_drones' in launch
    assert 'mission_radius' in mission
    assert 'dbscan_eps_m' in detection
    # Disjoint scopes.
    assert launch & mission == set()
    assert mission & detection == set()


def test_runtime_tweakable_is_subset_of_keys():
    for scope in ParamScope:
        rt = runtime_tweakable_for_scope(scope)
        keys = keys_for_scope(scope)
        assert rt <= keys


def test_declare_args_returns_default_pairs():
    args = declare_args_for_scope(ParamScope.MISSION)
    assert args['mission_radius'] == 70.0
    assert args['survey_altitude'] == 25.0


def test_form_schema_matches_param_count():
    for scope in ParamScope:
        form = form_schema_for_scope(scope)
        keys = keys_for_scope(scope)
        assert set(form.keys()) == keys


def test_legacy_loader_derives_from_schema():
    """scenario_loader's _MISSION_KEYS etc. now derive from PARAM_SCHEMA;
    the test asserts the derivation is live."""
    from drone_rescue_mission_control import scenario_loader
    assert scenario_loader._MISSION_KEYS == keys_for_scope(ParamScope.MISSION)
    assert scenario_loader._DETECTION_KEYS == keys_for_scope(ParamScope.DETECTION)
    assert scenario_loader._LAUNCH_KEYS == keys_for_scope(ParamScope.LAUNCH)


def test_schema_covers_legacy_runtime_params_mission():
    """The legacy hand-rolled mission_manager._RUNTIME_PARAMS set
    (preserved here for comparison) is a subset of what PARAM_SCHEMA
    now reports, i.e. nothing was lost in the migration."""
    legacy = frozenset((
        'coverage_overlap',
        'investigate_confidence_floor',
        'max_concurrent_investigations',
        'reject_age_seconds',
        'mission_timeout_seconds',
        'investigate_hover_seconds',
        'confirm_hover_seconds',
        'task_status_timeout_s',
    ))
    assert legacy <= runtime_tweakable_for_scope(ParamScope.MISSION)


def test_schema_covers_legacy_runtime_params_detection():
    legacy = frozenset((
        'confidence_floor', 'dbscan_eps_m', 'confirmation_threshold',
        'cluster_window_seconds', 'min_distance_from_drones',
        'lidar_corroboration_boost', 'lidar_depth_tolerance_m',
        'min_confirm_observations', 'min_multi_witnesses',
        'min_sightings_per_witness',
    ))
    assert legacy <= runtime_tweakable_for_scope(ParamScope.DETECTION)
