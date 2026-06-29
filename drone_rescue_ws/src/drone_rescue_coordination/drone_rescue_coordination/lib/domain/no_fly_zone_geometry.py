"""NoFlyZone geometry: pure-Python numpy policy.

Lifted out of ``zone_manager.ZoneManager`` so the distance / containment
math is unit-testable without instantiating a LifecycleNode. The
NoFlyZone VO lives in ``value_objects``. Same shape as ``lib.grid_utils``
and ``lib.pid``: pure-Python algorithm cores in ``lib/domain``; ROS
adapters consume them.

Function inventory:

- ``precompute_zone_state(zone)``: per-zone numpy cache (centre array
  plus radius for circles, vertices plus edge_starts/ends for polygons).
  Returns the dict shape the LifecycleNode-side ``_zone_np`` map
  originally stored.
- ``distance_to_zone(position, zone, state)``: signed distance to the
  zone boundary; negative inside, positive outside. Consumes the
  precomputed state.
- ``distance_to_polygon_edge_np`` / ``point_in_polygon_np``: vectorised
  primitives.
"""

from __future__ import annotations

import math
from typing import Dict, Mapping, Tuple

import numpy as np

from .value_objects import NoFlyZone, ZoneShape


def precompute_zone_state(zone: NoFlyZone) -> Dict:
    """Precompute the per-zone numpy state used by
    ``distance_to_zone``.

    Returns a dict with ``zone_type`` + ``valid`` (True / False) and
    type-specific cached arrays. Invalid configurations (a circle
    missing centre/radius, a polygon with <3 vertices) return
    ``valid=False`` and ``distance_to_zone`` returns ``+inf`` for them.
    """
    state: Dict = {'zone_type': zone.zone_type}
    if zone.zone_type == ZoneShape.CIRCLE:
        if zone.center is None or zone.radius is None:
            state['valid'] = False
        else:
            state['valid'] = True
            state['center'] = np.asarray(zone.center, dtype=np.float64)
            state['radius'] = float(zone.radius)
    elif zone.zone_type == ZoneShape.POLYGON:
        if len(zone.vertices) < 3:
            state['valid'] = False
        else:
            state['valid'] = True
            verts = np.asarray(zone.vertices, dtype=np.float64)
            state['vertices'] = verts                     # (V, 2)
            state['edge_starts'] = verts
            state['edge_ends'] = np.roll(verts, -1, axis=0)
    else:
        state['valid'] = False
    return state


def distance_to_zone(
    position: Tuple[float, float],
    zone: NoFlyZone,
    state: Mapping,
) -> float:
    """Signed distance from ``position`` to the zone boundary
    (negative inside, positive outside; +inf for invalid state).

    Caller looks up the precomputed state by zone name; this helper
    stays generic so tests can pass synthetic dicts directly.
    """
    if state is None or not state.get('valid'):
        return float('inf')
    if state['zone_type'] == ZoneShape.CIRCLE:
        dx = position[0] - state['center'][0]
        dy = position[1] - state['center'][1]
        dist_to_center = math.sqrt(dx * dx + dy * dy)
        return dist_to_center - (state['radius'] + zone.buffer_distance)
    if state['zone_type'] == ZoneShape.POLYGON:
        point = np.asarray(position, dtype=np.float64)
        edge_min = distance_to_polygon_edge_np(
            point, state['edge_starts'], state['edge_ends'],
        )
        inside = point_in_polygon_np(point, state['vertices'])
        if inside:
            return -edge_min
        return edge_min - zone.buffer_distance
    return float('inf')


def distance_to_polygon_edge_np(
    point: np.ndarray,
    edge_starts: np.ndarray,
    edge_ends: np.ndarray,
) -> float:
    """Vectorised point-to-segment over every polygon edge; returns
    the minimum distance to any edge."""
    seg = edge_ends - edge_starts                 # (V, 2)
    seg_len_sq = (seg * seg).sum(axis=1)          # (V,)
    pt_rel = point - edge_starts                  # (V, 2)
    t = np.where(
        seg_len_sq > 0,
        (pt_rel * seg).sum(axis=1) / np.maximum(seg_len_sq, 1e-12),
        0.0,
    )
    t = np.clip(t, 0.0, 1.0)
    proj = edge_starts + (t[:, None] * seg)        # (V, 2)
    d = np.linalg.norm(point - proj, axis=1)       # (V,)
    return float(d.min())


def point_in_polygon_np(point: np.ndarray, vertices: np.ndarray) -> bool:
    """Ray-casting point-in-polygon, vectorised over edges."""
    x, y = float(point[0]), float(point[1])
    xi = vertices[:, 0]
    yi = vertices[:, 1]
    xj = np.roll(xi, 1)
    yj = np.roll(yi, 1)
    cond1 = (yi > y) != (yj > y)
    with np.errstate(divide='ignore', invalid='ignore'):
        x_intersect = (
            (xj - xi) * (y - yi) / np.where(yj - yi != 0, yj - yi, 1e-12) + xi
        )
    cond2 = x < x_intersect
    crossings = int((cond1 & cond2).sum())
    return crossings % 2 == 1
