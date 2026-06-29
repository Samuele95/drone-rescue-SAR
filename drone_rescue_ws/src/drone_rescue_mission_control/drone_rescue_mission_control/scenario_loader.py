"""Scenario YAML registry loader.

Each YAML in `<bringup_share>/config/scenarios/*.yaml` defines a scenario:
  - launch:           args wired into demo.launch.py
  - mission:          ros2 params for mission_manager (runtime-applied)
  - detection:        ros2 params for detection_filter (runtime-applied)
  - drone_overrides:  per-drone battery (or other) overrides
  - ground_truth_victims: list of {id, position} for TP/FP scoring

The loader validates structure with friendly error messages and resolves
each scenario into:
  * `launch_args`: dict[str, str] passed to `ros2 launch ... key:=value`
  * `runtime_params`: list of (node_name, param_name, value) tuples to be
                      applied via `ros2 param set` once activation completes

Mission Control consumes both. Per-drone battery overrides currently
require a launch-time path (battery_monitor reads its drain rate at
__init__ and doesn't have a callback yet); we surface them as
`per_drone_battery_overrides` so the launcher can pass them as launch
params if/when wired.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

# Optional import resolved once at module load, not on
# every default_scenarios_dir() call. `ament_index_python` is absent in
# non-ROS dev shells; the source-tree fallback covers that case.
try:
    from ament_index_python.packages import (
        get_package_share_directory as _get_pkg_share,
    )
except ImportError:                       # pragma: no cover (non-ROS shell)
    _get_pkg_share = None


# ---------- schema ----------------------------------------------------

# Top-level keys allowed in a scenario YAML. Unknown top-level keys raise
# `ScenarioValidationError` so a typo in a hand-written YAML doesn't
# silently get ignored.
_TOP_LEVEL_KEYS = frozenset((
    'name', 'description', 'seed',
    'launch', 'mission', 'detection',
    'drone_overrides',
    'ground_truth_victims',
))

# Per-block whitelists derived from the single source of truth in
# `lib/domain/scenario_schema.PARAM_SCHEMA`.
# Adding a new scenario parameter is now a one-row edit in
# PARAM_SCHEMA; this loader picks it up automatically.
from drone_rescue_coordination.lib.domain.scenario_schema import (
    ParamScope as _ParamScope,
    keys_for_scope as _keys_for_scope,
    runtime_tweakable_for_scope as _runtime_tweakable_for_scope,
)
_LAUNCH_KEYS = _keys_for_scope(_ParamScope.LAUNCH)
_MISSION_KEYS = _keys_for_scope(_ParamScope.MISSION)
_DETECTION_KEYS = _keys_for_scope(_ParamScope.DETECTION)
# Names that nodes actually accept via the runtime ``set_parameters``
# service. The YAML block lists every scenario-tunable value (mission
# geometry included, because it lands in mission_recorder's JSONL
# metadata snapshot); but only the subset below should be pushed to a
# live node. mission_radius / inner_radius / survey_altitude /
# camera_footprint_m are launch-time-only by design: changing them
# mid-mission desyncs the executor's waypoint queue. Filtering here
# stops Mission Control's status bar from flashing "N params failed to
# apply" on every start.
_MISSION_RUNTIME_KEYS = _runtime_tweakable_for_scope(_ParamScope.MISSION)
_DETECTION_RUNTIME_KEYS = _runtime_tweakable_for_scope(_ParamScope.DETECTION)


class ScenarioValidationError(ValueError):
    """Raised when a scenario YAML has unknown / malformed keys."""


@dataclass
class ScenarioVictim:
    id: int
    position: Tuple[float, float, float]


@dataclass
class Scenario:
    """Validated scenario, ready to drive a mission."""
    path: Path
    name: str
    description: str = ''
    seed: int = 0   # master RNG seed; per-scenario default in YAML.
    launch: Dict[str, object] = field(default_factory=dict)
    mission: Dict[str, object] = field(default_factory=dict)
    detection: Dict[str, object] = field(default_factory=dict)
    drone_overrides: Dict[str, Dict[str, object]] = field(default_factory=dict)
    ground_truth_victims: List[ScenarioVictim] = field(default_factory=list)

    # ----- launch-side resolution -------------------------------------
    def launch_args(self) -> Dict[str, str]:
        """Return the dict of `key:=value` overrides to pass to ros2 launch.

        Always includes `record_run=true`, `scenario_yaml=<path>`,
        `scenario_name=<name>`, and `seed=<n>` so the recorder picks up
        ground truth and reproducibility metadata.
        """
        out: Dict[str, str] = {
            'record_run': 'true',
            'scenario_yaml': str(self.path),
            'scenario_name': self.name,
            'seed': str(int(self.seed)),
        }
        for k, v in self.launch.items():
            out[k] = str(v)
        return out

    def runtime_params(self) -> List[Tuple[str, str, object]]:
        """Return [(node_name, param_name, value)] tuples to apply via
        `ros2 param set` after activation completes.

        Filters launch-time-only params out of the pushed set: a YAML
        may legitimately list e.g. ``mission_radius`` because
        mission_recorder consumes it for the JSONL header, but the live
        node rejects it as launch-only. Pushing it anyway makes Mission
        Control flash "param failed to apply" warnings on every mission
        start. Source of truth: ``ParamDef.runtime_tweakable`` in
        ``scenario_schema.PARAM_SCHEMA``.
        """
        out: List[Tuple[str, str, object]] = []
        for k, v in self.mission.items():
            if k in _MISSION_RUNTIME_KEYS:
                out.append(('mission_manager', k, v))
        for k, v in self.detection.items():
            if k in _DETECTION_RUNTIME_KEYS:
                out.append(('detection_filter', k, v))
        return out

    def per_drone_battery_overrides(self) -> Dict[str, Dict[str, object]]:
        """Battery overrides keyed by drone name (e.g. drone3 base_drain).
        Currently surfaced for launch-time wiring (battery_monitor has no
        runtime param callback)."""
        return self.drone_overrides


# ---------- loader ----------------------------------------------------

def _validate_block(block_name: str, block: dict, allowed: frozenset) -> None:
    if block is None:
        return
    if not isinstance(block, dict):
        raise ScenarioValidationError(
            f"'{block_name}' must be a mapping, got {type(block).__name__}"
        )
    unknown = set(block.keys()) - allowed
    if unknown:
        raise ScenarioValidationError(
            f"unknown keys in '{block_name}': {sorted(unknown)}; "
            f"allowed: {sorted(allowed)}"
        )


def load_scenario(path: Path | str) -> Scenario:
    """Load + validate one scenario YAML."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ScenarioValidationError(f'{path}: top level must be a mapping')

    unknown = set(raw.keys()) - _TOP_LEVEL_KEYS
    if unknown:
        raise ScenarioValidationError(
            f'{path}: unknown top-level keys {sorted(unknown)}; '
            f'allowed: {sorted(_TOP_LEVEL_KEYS)}'
        )

    name = str(raw.get('name') or path.stem)
    description = str(raw.get('description') or '')
    seed = int(raw.get('seed') or 0)
    launch = raw.get('launch') or {}
    mission = raw.get('mission') or {}
    detection = raw.get('detection') or {}
    drone_overrides = raw.get('drone_overrides') or {}
    ground_truth_raw = raw.get('ground_truth_victims') or []

    _validate_block('launch', launch, _LAUNCH_KEYS)
    _validate_block('mission', mission, _MISSION_KEYS)
    _validate_block('detection', detection, _DETECTION_KEYS)

    # drone_overrides: keys are drone names, values are dicts of
    # battery params; values aren't whitelisted yet (battery_monitor
    # accepts a few; YAML errors there will surface at launch time).
    if not isinstance(drone_overrides, dict):
        raise ScenarioValidationError(
            f'{path}: drone_overrides must be a mapping'
        )

    victims: List[ScenarioVictim] = []
    if not isinstance(ground_truth_raw, list):
        raise ScenarioValidationError(
            f'{path}: ground_truth_victims must be a list'
        )
    for i, v in enumerate(ground_truth_raw):
        if not isinstance(v, dict) or 'id' not in v or 'position' not in v:
            raise ScenarioValidationError(
                f'{path}: ground_truth_victims[{i}] must be '
                f'{{id, position: [x, y, z]}}'
            )
        pos = v['position']
        if not (isinstance(pos, list) and len(pos) == 3):
            raise ScenarioValidationError(
                f'{path}: ground_truth_victims[{i}].position must be [x,y,z]'
            )
        victims.append(ScenarioVictim(
            id=int(v['id']),
            position=(float(pos[0]), float(pos[1]), float(pos[2])),
        ))

    return Scenario(
        path=path, name=name, description=description, seed=seed,
        launch=dict(launch), mission=dict(mission),
        detection=dict(detection),
        drone_overrides={
            str(k): dict(v) for k, v in drone_overrides.items()
        },
        ground_truth_victims=victims,
    )


def save_scenario(
    path: Path | str,
    *,
    name: str,
    description: str = '',
    seed: int = 0,
    launch: Optional[Dict[str, object]] = None,
    mission: Optional[Dict[str, object]] = None,
    detection: Optional[Dict[str, object]] = None,
    ground_truth_victims: Optional[List[ScenarioVictim]] = None,
    drone_overrides: Optional[Dict[str, Dict[str, object]]] = None,
) -> Scenario:
    """Write a scenario YAML and return the reloaded, validated Scenario.

    Mission Control's Setup tab uses this for "Save As…": the operator's
    current form values are persisted as a new scenario. The written file
    is immediately round-tripped through ``load_scenario``; if it fails
    validation the partial file is removed and the error re-raised, so a
    Save never leaves an unloadable YAML on disk.
    """
    path = Path(path)
    body: Dict[str, object] = {'name': str(name)}
    if description:
        body['description'] = str(description)
    body['seed'] = int(seed)
    if launch:
        body['launch'] = dict(launch)
    if mission:
        body['mission'] = dict(mission)
    if detection:
        body['detection'] = dict(detection)
    if drone_overrides:
        body['drone_overrides'] = {
            str(k): dict(v) for k, v in drone_overrides.items()
        }
    if ground_truth_victims:
        body['ground_truth_victims'] = [
            {'id': int(v.id), 'position': [float(c) for c in v.position]}
            for v in ground_truth_victims
        ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(body, f, sort_keys=False, default_flow_style=False)
    try:
        return load_scenario(path)
    except (ScenarioValidationError, FileNotFoundError):
        try:
            path.unlink()
        except OSError:
            pass
        raise


def discover_scenarios(scenarios_dir: Path | str) -> List[Scenario]:
    """Load every *.yaml in the directory; skip files that fail to validate
    (logged for the caller to surface). Sort by name for stable UI."""
    scenarios_dir = Path(scenarios_dir)
    out: List[Scenario] = []
    if not scenarios_dir.is_dir():
        return out
    for f in sorted(scenarios_dir.glob('*.yaml')):
        try:
            out.append(load_scenario(f))
        except ScenarioValidationError:
            # The launcher is responsible for surfacing the error; here we
            # just skip the broken file so the registry doesn't fail
            # globally.
            continue
    out.sort(key=lambda s: s.name.lower())
    return out


def default_scenarios_dir() -> Path:
    """Return the install-tree scenarios directory if available, else the
    source-tree path. Mission Control prefers the installed copy so it
    runs in production-like layouts."""
    if _get_pkg_share is not None:
        try:
            share = Path(_get_pkg_share('drone_rescue_bringup'))
            return share / 'config' / 'scenarios'
        except Exception:
            # Package not built yet; fall through to the source layout.
            pass
    # Source-tree fallback, useful in dev shells where colcon hasn't
    # been re-run after adding a new YAML, or ament_index is unavailable.
    here = Path(__file__).resolve()
    ws_root = here.parents[3]
    return ws_root / 'src' / 'drone_rescue_bringup' / 'config' / 'scenarios'
