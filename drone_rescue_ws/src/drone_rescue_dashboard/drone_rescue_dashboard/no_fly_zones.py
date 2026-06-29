"""No-fly-zone YAML loading shared by the 2D and 3D mission scenes.

scene_view's inline YAML load + polygon-centroid computation was
duplicated by the 3D sand-table (scene3d_view). One loader, one
normalised zone shape, two consumers.
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml


def load_no_fly_zones(yaml_path: Optional[str]) -> List[Dict[str, Any]]:
    """Load and normalise the ``no_fly_zones`` list from ``yaml_path``.

    Returns only well-formed zones: circles carry ``center``
    ``[x, y]`` + ``radius`` (float); polygons carry ``vertices`` with
    at least 3 points. Malformed entries and unreadable files yield
    an empty/partial list rather than raising, so the scene comes
    up even if the overlay config is broken.
    """
    if not yaml_path or not os.path.isfile(yaml_path):
        return []
    try:
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f) or {}
    except Exception:
        return []
    zones: List[Dict[str, Any]] = []
    for z in (cfg or {}).get('no_fly_zones', []) or []:
        kind = z.get('type')
        if kind == 'circle':
            center = z.get('center', [0, 0])
            if not (isinstance(center, (list, tuple)) and len(center) >= 2):
                continue
            zones.append({
                'type': 'circle',
                'name': z.get('name', ''),
                'center': [float(center[0]), float(center[1])],
                'radius': float(z.get('radius', 1.0)),
            })
        elif kind == 'polygon':
            pts = z.get('vertices', []) or []
            if len(pts) < 3:
                continue
            zones.append({
                'type': 'polygon',
                'name': z.get('name', ''),
                'vertices': [[float(p[0]), float(p[1])] for p in pts],
            })
    return zones


def polygon_centroid(pts: Sequence[Sequence[float]]) -> Tuple[float, float]:
    """Vertex-average centroid: where the zone label is drawn."""
    n = len(pts)
    return (sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n)
