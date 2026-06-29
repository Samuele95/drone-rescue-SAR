"""Pure-Python grid + geometry helpers shared across mission_manager,
coverage_planner, detection_filter, drone_executor, and any unit tests.

Extracted from the original surveyor.py:632-642 and pheromone_server.py:84-91.
"""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np


def world_to_grid(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    cell_resolution: float,
) -> Tuple[int, int]:
    """World (m) → grid (row, col). Row index advances with +y, col with +x."""
    col = int((x - origin_x) / cell_resolution)
    row = int((y - origin_y) / cell_resolution)
    return row, col


def grid_to_world(
    row: int,
    col: int,
    origin_x: float,
    origin_y: float,
    cell_resolution: float,
) -> Tuple[float, float]:
    """Grid → world coordinates of the cell *centre*."""
    x = origin_x + (col + 0.5) * cell_resolution
    y = origin_y + (row + 0.5) * cell_resolution
    return x, y


def in_disk(x: float, y: float, cx: float, cy: float, radius: float) -> bool:
    """True if (x,y) lies inside the closed disk of given centre and radius."""
    dx = x - cx
    dy = y - cy
    return dx * dx + dy * dy <= radius * radius


def gaussian_kernel(radius_cells: int, sigma_cells: float, peak: float = 1.0) -> np.ndarray:
    """Normalized 2-D Gaussian stamp of half-extent radius_cells, scaled so its
    maximum value equals `peak`. Used by pheromone deposit and (optionally)
    detection-confidence smoothing.
    """
    rr, cc = np.ogrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
    k = np.exp(-(rr ** 2 + cc ** 2) / (2.0 * max(sigma_cells, 1e-6) ** 2))
    k *= peak / k.max()
    return k.astype(np.float32)


def angle_in_sector(
    px: float,
    py: float,
    cx: float,
    cy: float,
    sector_start_rad: float,
    sector_end_rad: float,
) -> bool:
    """True if the angle from (cx,cy) to (px,py) is within
    [sector_start_rad, sector_end_rad). Handles wrap-around."""
    ang = math.atan2(py - cy, px - cx)
    # Normalize angles to [0, 2π).
    two_pi = 2.0 * math.pi
    a = ang % two_pi
    s = sector_start_rad % two_pi
    e = sector_end_rad % two_pi
    if s <= e:
        return s <= a < e
    # Wrapped sector (e.g. start=350°, end=20°)
    return a >= s or a < e


def euclidean(p: Iterable[float], q: Iterable[float]) -> float:
    """L2 distance between two iterables of equal length."""
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p, q)))
