"""ParamSchema: single source of truth for the scenario param surface.

Today the same parameter taxonomy is hand-written in FOUR places that
must not drift:

1. ``scenario_loader._LAUNCH_KEYS / _MISSION_KEYS / _DETECTION_KEYS``
   frozensets (the YAML schema validation whitelist).
2. ``mission_manager`` and ``detection_filter`` ``declare_parameter()``
   calls (the runtime ROS parameter declarations).
3. ``mission_manager._RUNTIME_PARAMS`` / ``detection_filter._RUNTIME_PARAMS``
   frozensets (which params Mission Control may tweak after activation).
4. ``mission_control/widgets/setup_tab._LAUNCH_FIELD_TYPES /
   _MISSION_FIELD_TYPES / _DETECTION_FIELD_TYPES`` (the form schema).

A typo in any one is a silent bug: a YAML-only entry would fail
validation; a node-only entry would never be settable from Mission
Control; a missing _RUNTIME_PARAMS entry would silently reject a
runtime tweak.

This module collapses all four into one ``PARAM_SCHEMA`` data
structure. Each consumer derives its hand-rolled view from this
schema instead of maintaining its own list.

The schema lives here; consumers continue to use their existing
hand-rolled lists. The migration to "derive from PARAM_SCHEMA"
happens as the consumers are touched for other reasons.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, FrozenSet, List, Optional, Tuple

# Selectable-algorithm names are single-sourced from the registries; the
# coverage_pattern / allocation_strategy choice lists below derive from these
# so a newly-registered strategy needs no schema edit. (No import cycle: the
# domain package is fully loaded before this module body runs.)
from ..sar_patterns import CoveragePatternFactory
from ..allocation import AllocationStrategyFactory


class ParamScope(str, Enum):
    """Which block of the scenario YAML / which node owns this param."""
    LAUNCH = 'launch'        # Wired through demo.launch.py / multi_drone_simulation.launch.py
    MISSION = 'mission'      # mission_manager parameter
    DETECTION = 'detection'  # detection_filter parameter


@dataclass(frozen=True)
class ParamDef:
    """One scenario parameter: all the metadata the four hand-written
    lists need, in one place.

    ``form_kind``: hint for the Mission Control form widget. One of
        'int', 'float', ('choice', [v1, v2, ...]), 'str'.
    """
    name: str
    scope: ParamScope
    type: type                   # int / float / str / bool
    default: Any
    runtime_tweakable: bool      # True: consumed by node param-callback
    description: str
    form_kind: Tuple = ('float', 0.0, 1.0, 0.05)
    range: Optional[Tuple[Any, Any]] = None


# The canonical schema. Adding a new scenario parameter: one row here.
# All four legacy hand-written lists derive from this (see helpers below).
PARAM_SCHEMA: List[ParamDef] = [
    # -------- LAUNCH (top-of-launch arguments) --------
    ParamDef(
        name='num_drones', scope=ParamScope.LAUNCH, type=int,
        default=4, runtime_tweakable=False,
        description='Number of drones spawned in the fleet.',
        form_kind=('int', 1, 8),
    ),
    ParamDef(
        name='coverage_pattern', scope=ParamScope.LAUNCH, type=str,
        default='spiral_out', runtime_tweakable=False,
        description='SAR coverage strategy name (CoveragePatternFactory).',
        form_kind=('choice', CoveragePatternFactory.list_names()),
    ),
    ParamDef(
        name='allocation_strategy', scope=ParamScope.LAUNCH, type=str,
        default='greedy_auction', runtime_tweakable=False,
        description='Task-allocation strategy name (AllocationStrategyFactory).',
        form_kind=('choice', AllocationStrategyFactory.list_names()),
    ),
    ParamDef(
        name='seed', scope=ParamScope.LAUNCH, type=int,
        default=0, runtime_tweakable=False,
        description='Master RNG seed (V5-A).',
        form_kind=('int', 0, 2_000_000_000),
    ),
    ParamDef(
        name='world', scope=ParamScope.LAUNCH, type=str,
        default='', runtime_tweakable=False,
        description='Override Gazebo world SDF.',
        form_kind=('str',),
    ),
    ParamDef(
        name='no_fly_zones_yaml', scope=ParamScope.LAUNCH, type=str,
        default='', runtime_tweakable=False,
        description='Override no-fly-zone YAML config.',
        form_kind=('str',),
    ),
    # Terrain gradient (m per m) for the planar ElevationModel. Both 0
    # (default) = flat. Non-zero tilts the terrain so scan waypoints fly at a
    # constant AGL height (mission_manager) and the detector measures AGL
    # (victim_detector); the matching Gazebo world's ground plane must be
    # tilted to the same gradient (see scenarios/sloped_terrain.yaml +
    # worlds/earthquake_zone_sloped.sdf).
    ParamDef(
        name='terrain_slope_x', scope=ParamScope.LAUNCH, type=float,
        default=0.0, runtime_tweakable=False,
        description='Terrain elevation gradient along world +x (m/m).',
        form_kind=('float', -0.5, 0.5, 0.01),
    ),
    ParamDef(
        name='terrain_slope_y', scope=ParamScope.LAUNCH, type=float,
        default=0.0, runtime_tweakable=False,
        description='Terrain elevation gradient along world +y (m/m).',
        form_kind=('float', -0.5, 0.5, 0.01),
    ),

    # -------- MISSION (mission_manager) --------
    ParamDef(
        name='mission_radius', scope=ParamScope.MISSION, type=float,
        default=70.0, runtime_tweakable=False,
        description='Outer radius of the search disk (m).',
        form_kind=('float', 10.0, 200.0, 0.5),
    ),
    ParamDef(
        name='inner_radius', scope=ParamScope.MISSION, type=float,
        default=5.0, runtime_tweakable=False,
        description='Inner radius (donut hole around launch pad).',
        form_kind=('float', 0.0, 50.0, 0.5),
    ),
    ParamDef(
        name='survey_altitude', scope=ParamScope.MISSION, type=float,
        default=25.0, runtime_tweakable=False,
        description='Cruise altitude during scan (m).',
        form_kind=('float', 5.0, 80.0, 0.5),
    ),
    ParamDef(
        name='camera_footprint_m', scope=ParamScope.MISSION, type=float,
        default=35.0, runtime_tweakable=False,
        description='Effective ground footprint of the camera (m).',
        form_kind=('float', 5.0, 100.0, 0.5),
    ),
    ParamDef(
        name='coverage_overlap', scope=ParamScope.MISSION, type=float,
        default=0.85, runtime_tweakable=False,
        description='Overlap between adjacent scan tracks (0..1).',
        form_kind=('float', 0.0, 0.99, 0.05),
    ),
    ParamDef(
        name='investigate_confidence_floor', scope=ParamScope.MISSION,
        type=float, default=0.90, runtime_tweakable=True,
        description='Min candidate confidence to dispatch INVESTIGATE.',
        form_kind=('float', 0.0, 1.0, 0.05),
    ),
    ParamDef(
        name='max_concurrent_investigations', scope=ParamScope.MISSION,
        type=int, default=1, runtime_tweakable=True,
        description='Cap on simultaneous off-sector INVESTIGATE drones.',
        form_kind=('int', 1, 4),
    ),
    ParamDef(
        name='reject_age_seconds', scope=ParamScope.MISSION, type=float,
        default=60.0, runtime_tweakable=True,
        description='Candidate decay window before auto-REJECTED.',
        form_kind=('float', 5.0, 600.0, 5.0),
    ),
    ParamDef(
        name='mission_timeout_seconds', scope=ParamScope.MISSION,
        type=float, default=600.0, runtime_tweakable=True,
        description='Sim-time after which the mission terminates.',
        form_kind=('float', 60.0, 3600.0, 10.0),
    ),
    ParamDef(
        name='investigate_hover_seconds', scope=ParamScope.MISSION,
        type=float, default=4.0, runtime_tweakable=True,
        description='Dwell time at INVESTIGATE target.',
    ),
    ParamDef(
        name='confirm_hover_seconds', scope=ParamScope.MISSION, type=float,
        default=6.0, runtime_tweakable=True,
        description='Dwell time during CONFIRM orbit.',
    ),
    ParamDef(
        name='task_status_timeout_s', scope=ParamScope.MISSION, type=float,
        default=30.0, runtime_tweakable=True,
        description='Watchdog: silence threshold before TASK_TIMEOUT.',
    ),

    # -------- DETECTION (detection_filter) --------
    ParamDef(
        name='confidence_floor', scope=ParamScope.DETECTION, type=float,
        default=0.65, runtime_tweakable=True,
        description='Drop sightings below this confidence.',
        form_kind=('float', 0.0, 1.0, 0.05),
    ),
    ParamDef(
        name='dbscan_eps_m', scope=ParamScope.DETECTION, type=float,
        default=6.0, runtime_tweakable=True,
        description='DBSCAN neighbourhood radius (m).',
        form_kind=('float', 1.0, 20.0, 0.5),
    ),
    ParamDef(
        name='confirmation_threshold', scope=ParamScope.DETECTION,
        type=float, default=0.80, runtime_tweakable=True,
        description='Min fused confidence for auto-CONFIRMED.',
        form_kind=('float', 0.0, 1.0, 0.05),
    ),
    ParamDef(
        name='min_confirm_observations', scope=ParamScope.DETECTION,
        type=int, default=5, runtime_tweakable=True,
        description='Min cluster observation_count before auto-CONFIRMED.',
        form_kind=('int', 1, 30),
    ),
    ParamDef(
        name='min_multi_witnesses', scope=ParamScope.DETECTION, type=int,
        default=2, runtime_tweakable=True,
        description='Min distinct drones contributing ≥K sightings.',
        form_kind=('int', 1, 4),
    ),
    ParamDef(
        name='min_sightings_per_witness', scope=ParamScope.DETECTION,
        type=int, default=2, runtime_tweakable=True,
        description='K for the multi-witness gate.',
        form_kind=('int', 1, 10),
    ),
    ParamDef(
        name='cluster_window_seconds', scope=ParamScope.DETECTION, type=float,
        default=45.0, runtime_tweakable=True,
        description='Sliding cluster-window length (s).',
    ),
    ParamDef(
        name='min_distance_from_drones', scope=ParamScope.DETECTION,
        # Was 9.0. The 9 m self-filter was calibrated for 14 m altitude
        # (half of the 16 m footprint at 60 deg FOV); at current 25 m
        # altitude + 90 deg FOV the footprint radius is 25 m, so 9 m
        # rejects the central 36 % of every frame, including the area
        # where INVESTIGATE-hovering drones see victims. 2 m only
        # rejects sightings literally under the drone body.
        type=float, default=2.0, runtime_tweakable=True,
        description='Self-filter: reject sightings near any drone XY.',
    ),
    ParamDef(
        name='lidar_corroboration_boost', scope=ParamScope.DETECTION,
        type=float, default=0.15, runtime_tweakable=True,
        description='Confidence boost when LiDAR corroborates.',
    ),
    ParamDef(
        name='lidar_depth_tolerance_m', scope=ParamScope.DETECTION,
        type=float, default=4.0, runtime_tweakable=True,
        description='Tolerance window around expected ground range.',
    ),
    # Lifecycle promotion exposed two DetectionFilter params that lived
    # inline and were missing from the schema. Adding them here closes
    # the silent-accept/no-effect bug for dbscan_min_samples (Mission
    # Control could set it but the runtime callback ignored it).
    ParamDef(
        name='dbscan_min_samples', scope=ParamScope.DETECTION,
        type=int, default=2, runtime_tweakable=True,
        description='DBSCAN min_samples — minimum cluster size '
                    '(Ester convention: includes the point itself).',
        form_kind=('int', 1, 10),
    ),
    ParamDef(
        name='publish_rate_hz', scope=ParamScope.DETECTION,
        type=float, default=2.0, runtime_tweakable=False,
        description='Cluster-tick rate (Hz). NOT runtime-tweakable — '
                    'changing the timer period requires destroy_timer + '
                    'create_timer plumbing not currently in place.',
        form_kind=('float', 0.5, 5.0, 0.5),
    ),
]


# ============================================================ derivation helpers
#
# The four legacy hand-written lists become functions over PARAM_SCHEMA.
# Consumers can either (a) keep their hand-rolled lists and derive them
# at module load via these helpers, or (b) iterate PARAM_SCHEMA directly.

def keys_for_scope(scope: ParamScope) -> FrozenSet[str]:
    """Equivalent of scenario_loader._LAUNCH_KEYS / _MISSION_KEYS /
    _DETECTION_KEYS, the YAML-block whitelist."""
    return frozenset(p.name for p in PARAM_SCHEMA if p.scope == scope)


def runtime_tweakable_for_scope(scope: ParamScope) -> FrozenSet[str]:
    """Equivalent of mission_manager._RUNTIME_PARAMS /
    detection_filter._RUNTIME_PARAMS, params the node accepts as a
    runtime ``ros2 param set``."""
    return frozenset(
        p.name for p in PARAM_SCHEMA
        if p.scope == scope and p.runtime_tweakable
    )


def declare_args_for_scope(scope: ParamScope) -> Dict[str, Any]:
    """Convenience for ``node.declare_parameter`` loops:
    returns ``{name: default}`` for the given scope."""
    return {p.name: p.default for p in PARAM_SCHEMA if p.scope == scope}


def form_schema_for_scope(scope: ParamScope) -> Dict[str, Tuple]:
    """Equivalent of setup_tab._LAUNCH_FIELD_TYPES / _MISSION_FIELD_TYPES /
    _DETECTION_FIELD_TYPES, Mission Control's form-widget schema."""
    return {p.name: p.form_kind for p in PARAM_SCHEMA if p.scope == scope}
