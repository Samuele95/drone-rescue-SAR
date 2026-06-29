"""No-fly-zone enforcement helpers: pure Python, no rclpy.

Before this module, no-fly zones were advisory-only: ``zone_manager`` detected
breaches and published warnings, but the only subscriber (``surveyor.py``) was a
dead node launched nowhere, the coverage planner built ``WorldModel`` with
``no_fly_zones=tuple()``, and no scan waypoint was ever filtered: nothing kept a
drone out of a zone.

These pure functions give a *live* node (``mission_manager``) the two pieces it
needs to enforce the zones it already loads:

* :func:`load_no_fly_zones` / :func:`precompute_states`: load the same YAML the
  ``zone_manager`` reads and precompute the vectorised geometry state.
* :func:`waypoint_blocked_by` / :func:`filter_waypoints`: drop scan waypoints
  that fall inside a zone (so the planner never sends a drone into one).
* :func:`drone_name_from_violation`: parse ``zone_manager``'s ``/zones/violation``
  ``String`` alert so the live subscriber can force the offending drone to RTH.

The geometry itself is delegated to ``no_fly_zone_geometry`` (the same code the
``zone_manager`` check loop uses), so enforcement and detection agree by
construction.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import yaml

from .no_fly_zone_geometry import distance_to_zone, precompute_zone_state
from .value_objects import NoFlyZone


def load_no_fly_zones(config_file: str) -> List[NoFlyZone]:
    """Load no-fly zones from a YAML config file.

    Mirrors ``zone_manager._load_zones`` construction so the live planner and
    the detector node see identical zones. A malformed individual zone is
    skipped (logged by the caller if desired) rather than failing the load.
    Returns an empty list when the file is missing or unreadable.
    """
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f) or {}
    except Exception:
        return []

    zones: List[NoFlyZone] = []
    for zone_cfg in config.get('no_fly_zones', []):
        try:
            zones.append(NoFlyZone(
                name=zone_cfg['name'],
                zone_type=zone_cfg['type'],
                priority=zone_cfg.get('priority', 'medium'),
                reason=zone_cfg.get('reason', ''),
                vertices=tuple(
                    (v[0], v[1]) for v in zone_cfg.get('vertices', [])
                ),
                center=tuple(zone_cfg['center']) if 'center' in zone_cfg else None,
                radius=zone_cfg.get('radius'),
                min_altitude=zone_cfg.get('min_altitude', 0.0),
                max_altitude=zone_cfg.get('max_altitude', 100.0),
                buffer_distance=zone_cfg.get('buffer_distance', 2.0),
            ))
        except (KeyError, ValueError, TypeError):
            continue
    return zones


def precompute_states(zones: List[NoFlyZone]) -> Dict[str, Dict]:
    """Precompute the vectorised geometry state keyed by zone name."""
    return {zone.name: precompute_zone_state(zone) for zone in zones}


def waypoint_blocked_by(
    x: float, y: float, z: float,
    zones: List[NoFlyZone],
    states: Optional[Dict[str, Dict]] = None,
) -> Optional[NoFlyZone]:
    """Return the first zone that forbids ``(x, y, z)``, or ``None``.

    A waypoint is blocked when it is inside the zone footprint (signed
    boundary distance ``<= 0``, buffer included) *and* within the zone's
    ``[min_altitude, max_altitude]`` band, matching ``zone_manager``'s
    violation test exactly.
    """
    states = states or {}
    for zone in zones:
        if not (zone.min_altitude <= z <= zone.max_altitude):
            continue
        if distance_to_zone((x, y), zone, states.get(zone.name)) <= 0.0:
            return zone
    return None


def filter_waypoints(
    waypoints,
    zones: List[NoFlyZone],
    states: Optional[Dict[str, Dict]] = None,
) -> Tuple[list, list]:
    """Split ``waypoints`` into ``(kept, removed)`` by zone membership.

    Each waypoint must expose ``.x`` / ``.y`` / ``.z`` (a ``geometry_msgs/Point``
    or any duck-typed equivalent). ``removed`` waypoints fall inside a zone;
    ``kept`` are everything else, order-preserved. When there are no zones the
    input is returned unchanged (fast path).
    """
    if not zones:
        return list(waypoints), []
    states = states if states is not None else precompute_states(zones)
    kept, removed = [], []
    for wp in waypoints:
        if waypoint_blocked_by(wp.x, wp.y, wp.z, zones, states) is None:
            kept.append(wp)
        else:
            removed.append(wp)
    return kept, removed


def drone_name_from_violation(text: str) -> Optional[str]:
    """Extract the drone name from a ``zone_manager`` violation alert.

    The alert format is ``"ZONE_VIOLATION: <drone> in <zone> (<reason>)"``
    (zone_manager.check_zones_callback). Returns the drone token, or ``None``
    when the text does not match the expected prefix/shape.
    """
    if not text:
        return None
    tokens = text.split()
    # tokens: ["ZONE_VIOLATION:", "<drone>", "in", "<zone>", ...]
    if len(tokens) >= 2 and tokens[0] == 'ZONE_VIOLATION:':
        return tokens[1]
    return None
