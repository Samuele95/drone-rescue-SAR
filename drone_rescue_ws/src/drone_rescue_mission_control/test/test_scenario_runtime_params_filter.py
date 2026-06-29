"""Regression test for scenario_loader runtime_params() filter.

The bug: scenario_loader pushed every key in the YAML's ``mission:`` and
``detection:`` blocks to the live node via ``set_parameters``, including
keys flagged ``runtime_tweakable=False`` in PARAM_SCHEMA. Live nodes
correctly rejected those four launch-only keys (``mission_radius``,
``inner_radius``, ``survey_altitude``, ``camera_footprint_m``), but
Mission Control's status bar flashed "4 params failed to apply" on every
mission start, a UX defect that masked any *real* failure.

This test pins the filter: runtime_params() must drop launch-only keys
while keeping all genuinely runtime-tweakable ones.
"""

from __future__ import annotations

from drone_rescue_coordination.lib.domain.scenario_schema import (
    ParamScope, runtime_tweakable_for_scope,
)
from drone_rescue_mission_control.scenario_loader import Scenario


def _scenario(mission=None, detection=None) -> Scenario:
    """Minimal in-memory Scenario fixture: no YAML, no disk I/O."""
    from pathlib import Path
    return Scenario(
        path=Path('/dev/null'), name='test',
        mission=dict(mission or {}),
        detection=dict(detection or {}),
    )


# launch-only

def test_launch_only_mission_geometry_is_filtered_out():
    """The four launch-time-only mission params from the schema must
    NOT appear in runtime_params() even though they are valid YAML
    keys consumed by mission_recorder for metadata."""
    s = _scenario(mission={
        'mission_radius': 70.0,
        'inner_radius': 5.0,
        'survey_altitude': 25.0,
        'camera_footprint_m': 35.0,
    })
    pushed = s.runtime_params()
    pushed_names = [name for (_node, name, _v) in pushed]
    for launch_only in (
        'mission_radius',
        'inner_radius',
        'survey_altitude',
        'camera_footprint_m',
    ):
        assert launch_only not in pushed_names, (
            f'{launch_only!r} is launch-time only — must not be pushed '
            f'via set_parameters'
        )


def test_runtime_tweakable_mission_params_are_kept():
    """And the runtime-tweakable ones MUST still be pushed."""
    s = _scenario(mission={
        'coverage_overlap': 0.85,
        'investigate_confidence_floor': 0.90,
        'max_concurrent_investigations': 1,
        'reject_age_seconds': 60.0,
        'mission_timeout_seconds': 600.0,
    })
    pushed = s.runtime_params()
    pushed_names = {name for (_node, name, _v) in pushed}
    assert pushed_names == {
        'coverage_overlap',
        'investigate_confidence_floor',
        'max_concurrent_investigations',
        'reject_age_seconds',
        'mission_timeout_seconds',
    }
    # And all routed to mission_manager.
    for node, _name, _v in pushed:
        assert node == 'mission_manager'


def test_runtime_tweakable_detection_params_are_kept():
    """Same gate, detection_filter side."""
    s = _scenario(detection={
        'confidence_floor': 0.65,
        'dbscan_eps_m': 6.0,
        'confirmation_threshold': 0.80,
    })
    pushed = s.runtime_params()
    pushed_names = {name for (_node, name, _v) in pushed}
    assert pushed_names == {
        'confidence_floor',
        'dbscan_eps_m',
        'confirmation_threshold',
    }
    for node, _name, _v in pushed:
        assert node == 'detection_filter'


# mixed payload

def test_mixed_yaml_filters_launch_only_keeps_the_rest():
    """The canonical default-scenario shape: a mix of launch-only and
    runtime-tweakable mission keys. Filter drops the four launch-only,
    keeps the rest."""
    s = _scenario(
        mission={
            # launch-only: must be filtered
            'mission_radius': 70.0,
            'survey_altitude': 25.0,
            # runtime: must be kept
            'coverage_overlap': 0.85,
            'mission_timeout_seconds': 600.0,
        },
        detection={'confidence_floor': 0.65},
    )
    pushed = {(node, name) for (node, name, _v) in s.runtime_params()}
    assert pushed == {
        ('mission_manager', 'coverage_overlap'),
        ('mission_manager', 'mission_timeout_seconds'),
        ('detection_filter', 'confidence_floor'),
    }


# schema gate

def test_filter_keys_match_schema_truth():
    """Pin that the filter consults PARAM_SCHEMA: if a new param is
    added with runtime_tweakable=True in the schema, the loader must
    pick it up automatically. Schema is the single source of truth."""
    mission_runtime = runtime_tweakable_for_scope(ParamScope.MISSION)
    detection_runtime = runtime_tweakable_for_scope(ParamScope.DETECTION)
    s = _scenario(
        mission={name: 0.0 for name in mission_runtime},
        detection={name: 0.0 for name in detection_runtime},
    )
    pushed = {(node, name) for (node, name, _v) in s.runtime_params()}
    expected = (
        {('mission_manager', n) for n in mission_runtime}
        | {('detection_filter', n) for n in detection_runtime}
    )
    assert pushed == expected
