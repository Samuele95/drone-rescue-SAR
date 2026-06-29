"""NavigationPolicy: pure-Python surveyor motor-schema.

Lifted out of ``surveyor.Surveyor.compute_*`` so the per-tick navigation
reducer is rclpy-free and unit-testable. The five ``compute_*`` functions
are module-level pure functions.

The five ``compute_*`` functions are the "basis behaviors" (pp. 99-100);
their weighted vector sum is the motor-schema combination (pp. 88-90).
``motor_schema_blend()`` is the named summation; ``blend()`` is kept as a
back-compat alias. Each basis behaviour carries a ``# Behaviour-N: <name>``
header referencing the taxonomy.

Source material:

- Behaviour-Based Control, Think the Way You Act (pp. 88-90):
  "A set of distributed, interacting modules, called behaviors,
  that collectively achieve the desired system-level behavior."
- Basic Principles of Behavior-Based Systems (pp. 94-96):
  Stimuli / Process / Behavior / Action.
- Defining behaviors (p. 100): "Basis behaviors are a set of
  behaviors such that each is necessary ... The basis behavior set
  is sufficient for achieving the goals mandated for the
  controller."

Invariants:
- Nav vectors are unit-magnitude or (0.0, 0.0).
- Weights are non-negative.
- The radial mesh cache is constructor-injected (so it's testable).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence, Tuple

import numpy as np

from drone_rescue_coordination.lib.domain.value_objects import Position


# ------------------------------------------------------------- VictimHotspot
# Canonical home is value_objects.py; re-exported here so existing
# `lib.domain.navigation.VictimHotspot` imports keep working.
from .value_objects import VictimHotspot  # noqa: F401


# ------------------------------------------------------------- mesh helpers


def build_radial_mesh(radius: int) -> Mapping[str, np.ndarray]:
    """Precompute (dr, dc, dist, inv_dist) meshes for a radial window.

    Returns a mapping with float32 arrays of shape ``(2r+1, 2r+1)``.
    The centre cell has ``dist==0`` so consumers must mask it out.
    """
    coords = np.arange(-radius, radius + 1, dtype=np.float32)
    dr_mesh, dc_mesh = np.meshgrid(coords, coords, indexing='ij')
    dist = np.sqrt(dr_mesh ** 2 + dc_mesh ** 2)
    inv_dist = np.where(dist > 0, 1.0 / np.maximum(dist, 1e-9), 0.0)
    return {
        'radius': radius,
        'dr': dr_mesh,
        'dc': dc_mesh,
        'dist': dist,
        'inv_dist': inv_dist.astype(np.float32),
    }


def slice_window(
    center_row: int,
    center_col: int,
    radius: int,
    grid_height: int,
    grid_width: int,
) -> Tuple[int, int, int, int, int, int, int, int]:
    """Window bounds (row_lo, row_hi, col_lo, col_hi, mesh offsets)
    for the intersection of the radial window with the grid."""
    r = radius
    row_lo = max(0, center_row - r)
    row_hi = min(grid_height, center_row + r + 1)
    col_lo = max(0, center_col - r)
    col_hi = min(grid_width, center_col + r + 1)
    mr_lo = row_lo - (center_row - r)
    mr_hi = mr_lo + (row_hi - row_lo)
    mc_lo = col_lo - (center_col - r)
    mc_hi = mc_lo + (col_hi - col_lo)
    return row_lo, row_hi, col_lo, col_hi, mr_lo, mr_hi, mc_lo, mc_hi


# ------------------------------------------------------------- compute_*


def compute_repulsion(
    pheromone_grid: np.ndarray,
    center_row: int,
    center_col: int,
    mesh: Mapping[str, np.ndarray],
    threshold: float,
) -> Tuple[float, float]:
    """Behaviour-1 (basis): Avoid Visited Areas, repulsion from
    high-pheromone cells. "avoid" is a canonical basis behaviour
    (p. 99). Stimulus: pheromone-grid cell weights > threshold;
    action: unit-magnitude vector AWAY from the centre-of-mass of
    those cells, weighted by inverse distance.
    """
    r = int(mesh['radius'])
    grid_h, grid_w = pheromone_grid.shape
    rl, rh, cl, ch, mr_lo, mr_hi, mc_lo, mc_hi = slice_window(
        center_row, center_col, r, grid_h, grid_w,
    )
    if rl >= rh or cl >= ch:
        return (0.0, 0.0)
    window = pheromone_grid[rl:rh, cl:ch]
    dr_w = mesh['dr'][mr_lo:mr_hi, mc_lo:mc_hi]
    dc_w = mesh['dc'][mr_lo:mr_hi, mc_lo:mc_hi]
    inv_dist_w = mesh['inv_dist'][mr_lo:mr_hi, mc_lo:mc_hi]
    active = (window > threshold) & (inv_dist_w > 0)
    if not active.any():
        return (0.0, 0.0)
    strength = window * inv_dist_w * active
    rep_x = float(-(dc_w * strength).sum())
    rep_y = float(-(dr_w * strength).sum())
    return (rep_x, rep_y)


def compute_attraction(
    pheromone_grid: np.ndarray,
    center_row: int,
    center_col: int,
    mesh: Mapping[str, np.ndarray],
    threshold: float,
) -> Tuple[float, float]:
    """Behaviour-2 (basis): Explore Unvisited Areas, attraction
    toward the nearest unexplored (low-pheromone) cell. "explore" is
    a canonical basis behaviour (p. 99); the L1 surveyor's
    exploration mandate is realised as the dual of the avoid
    behaviour over the same stigmergic medium."""
    r = int(mesh['radius'])
    grid_h, grid_w = pheromone_grid.shape
    rl, rh, cl, ch, mr_lo, mr_hi, mc_lo, mc_hi = slice_window(
        center_row, center_col, r, grid_h, grid_w,
    )
    if rl >= rh or cl >= ch:
        return (0.0, 0.0)
    window = pheromone_grid[rl:rh, cl:ch]
    dist_w = mesh['dist'][mr_lo:mr_hi, mc_lo:mc_hi]
    dr_w = mesh['dr'][mr_lo:mr_hi, mc_lo:mc_hi]
    dc_w = mesh['dc'][mr_lo:mr_hi, mc_lo:mc_hi]
    active = (window < threshold) & (dist_w > 0)
    if not active.any():
        return (0.0, 0.0)
    masked_dist = np.where(active, dist_w, np.float32('inf'))
    flat_idx = int(np.argmin(masked_dist))
    best_distance = float(masked_dist.flat[flat_idx])
    if not np.isfinite(best_distance) or best_distance <= 0.0:
        return (0.0, 0.0)
    best_dx = float(dc_w.flat[flat_idx]) / best_distance
    best_dy = float(dr_w.flat[flat_idx]) / best_distance
    return (best_dx, best_dy)


def compute_collision_avoidance(
    self_pos: Position,
    peer_positions: Mapping[str, Position],
    collision_distance: float,
) -> Tuple[float, float]:
    """Behaviour-3 (basis): Avoid Other Robots, repulsion from
    peers inside ``collision_distance``. "avoid robot/avoid obstacle"
    are canonical basis behaviours for multi-robot coordination
    (p. 96); this is the peer-avoidance variant.

    Signature takes ``Position`` VOs instead of
    ``geometry_msgs.msg.Point``.
    """
    if not peer_positions:
        return (0.0, 0.0)
    coords = np.fromiter(
        (v for pos in peer_positions.values() for v in (pos.x, pos.y)),
        dtype=np.float64,
        count=2 * len(peer_positions),
    ).reshape(-1, 2)
    dx = self_pos.x - coords[:, 0]
    dy = self_pos.y - coords[:, 1]
    dist = np.hypot(dx, dy)
    mask = (dist < collision_distance) & (dist > 0.1)
    if not mask.any():
        return (0.0, 0.0)
    strength = (collision_distance - dist[mask]) / dist[mask]
    avoidance_x = float((dx[mask] * strength).sum())
    avoidance_y = float((dy[mask] * strength).sum())
    return (avoidance_x, avoidance_y)


def compute_boundary_repulsion(
    self_pos: Position,
    center_x: float,
    center_y: float,
    radius: float,
) -> Tuple[float, float]:
    """Behaviour-4 (basis): Stay Inside, inward repulsion as the
    drone approaches or exits the mission disk. Workspace containment
    is a primitive obstacle-avoidance behaviour (p. 27); here the
    obstacle is the disk boundary itself.

    Zero force inside ``0.5 * radius``; ramps linearly from 0 at 0.5R
    to 1 at R, capped at 5.0 outside R."""
    dx = center_x - self_pos.x
    dy = center_y - self_pos.y
    r = math.hypot(dx, dy)
    if r < 1e-3:
        return (0.0, 0.0)
    safe_radius = radius * 0.5
    if r < safe_radius:
        return (0.0, 0.0)
    ramp_span = max(radius - safe_radius, 1e-3)
    strength = (r - safe_radius) / ramp_span
    strength = min(strength, 5.0)
    return (dx / r * strength, dy / r * strength)


def compute_victim_attraction(
    self_pos: Position,
    hotspots: Sequence[VictimHotspot],
    attraction_radius: float,
    confirm_hover_radius: float,
    now_sec: float,
    ttl: float,
) -> Tuple[float, float]:
    """Behaviour-5 (basis): Goal-Seek (Victim), attraction toward
    the highest-priority active victim hotspot. "goal-seek" / "homing"
    are canonical basis behaviours (p. 99); the L1 surveyor's
    prosocial mandate is realised as a confidence- and
    confirmation-weighted goal-seek over hotspots within
    ``attraction_radius``.

    Caller is responsible for pruning expired hotspots; this function
    treats anything older than ``ttl`` as inactive and skips it.
    Returns ``(0.0, 0.0)`` when no active hotspot is within
    ``attraction_radius``, or when the drone is already inside
    ``confirm_hover_radius`` of the chosen hotspot.
    """
    if not hotspots:
        return (0.0, 0.0)
    best = None
    best_priority = 0.0
    best_dx = best_dy = 0.0
    best_dist = 0.0
    for hs in hotspots:
        if now_sec - hs.t_seen > ttl:
            continue
        dx = hs.x - self_pos.x
        dy = hs.y - self_pos.y
        dist = math.hypot(dx, dy)
        if dist > attraction_radius:
            continue
        priority = (
            (2.0 if hs.confirmed else 1.0)
            * max(hs.confidence, 0.05)
            / max(dist, 1.0)
        )
        if priority > best_priority:
            best_priority = priority
            best = hs
            best_dx, best_dy, best_dist = dx, dy, dist
    if best is None or best_dist < 1e-3:
        return (0.0, 0.0)
    if best_dist < confirm_hover_radius:
        return (0.0, 0.0)
    boost = 1.5 if best.confirmed else 1.0
    return (best_dx / best_dist * boost, best_dy / best_dist * boost)


@dataclass(frozen=True)
class MotorSchemaOutput:
    """Auditable per-tick result of the motor-schema combination.

    The motor-schema combination is the weighted vector sum of basis
    behaviours (p. 89); this VO is its first-class typed result so
    callers (logger, dashboard diagnostics) can inspect each component
    instead of only seeing the final unit vector.

    ``nav_vector`` is the post-blend unit vector (or ``(0, 0)`` if
    all components cancel). The per-behaviour fields carry the raw
    component vectors before weighting, useful when tuning weights.
    """
    nav_vector: Tuple[float, float]
    avoid_visited: Tuple[float, float]
    explore_unvisited: Tuple[float, float]
    avoid_peers: Tuple[float, float]
    stay_inside: Tuple[float, float]
    goal_seek_victim: Tuple[float, float]


def motor_schema_blend(
    *,
    repulsion: Tuple[float, float],
    attraction: Tuple[float, float],
    collision_avoidance: Tuple[float, float],
    boundary: Tuple[float, float],
    victim: Tuple[float, float],
    weights: Sequence[float],
) -> MotorSchemaOutput:
    """Motor-schema combination of the five basis behaviours.

    Behaviour-Based Control (pp. 88-90) defines the motor-schema
    combination as a weighted vector sum over a set of basis
    behaviours. This is the typed entry point: keyword-only args name
    each basis behaviour so the call-site reads as the definition.

    The five named components correspond to the five ``compute_*``
    basis behaviours in this module (in declared order). ``weights``
    is the parallel weight tuple; all weights must be non-negative.

    The returned ``MotorSchemaOutput`` carries both the final unit
    nav vector and the per-behaviour components for diagnostics.
    The positional ``blend()`` shim below preserves the legacy
    Sequence-positional signature for the existing call site in
    ``lib.domain.surveyor`` and the unit tests.

    Invariant: ``nav_vector`` is unit-magnitude or ``(0, 0)``.
    """
    if len(weights) != 5:
        raise ValueError(
            f'motor_schema_blend expects 5 weights '
            f'(repulsion/attraction/collision_avoidance/boundary/victim), '
            f'got {len(weights)}'
        )
    if any(w < 0 for w in weights):
        raise ValueError('all weights must be non-negative')

    components = (repulsion, attraction, collision_avoidance, boundary, victim)
    sx = sum(v[0] * w for v, w in zip(components, weights))
    sy = sum(v[1] * w for v, w in zip(components, weights))
    mag = math.hypot(sx, sy)
    nav: Tuple[float, float]
    nav = (0.0, 0.0) if mag < 1e-9 else (sx / mag, sy / mag)

    return MotorSchemaOutput(
        nav_vector=nav,
        avoid_visited=repulsion,
        explore_unvisited=attraction,
        avoid_peers=collision_avoidance,
        stay_inside=boundary,
        goal_seek_victim=victim,
    )


def blend(
    vectors: Sequence[Tuple[float, float]],
    weights: Sequence[float],
) -> Tuple[float, float]:
    """Weighted sum + normalise. Unit vector or (0, 0).

    .. deprecated::
        Use :func:`motor_schema_blend` instead. The positional
        ``vectors`` / ``weights`` signature predates the named
        basis-behaviour API and is retained only for back-compat.
        Scheduled for removal after the test suite migrates to
        :func:`motor_schema_blend`.

    Back-compat positional API. New call sites should use
    ``motor_schema_blend(repulsion=..., attraction=..., ...)`` which
    returns a typed ``MotorSchemaOutput`` and names each basis
    behaviour per the motor-schema definition (pp. 88-90).

    Invariant: nav vectors are unit-magnitude or (0, 0); weights are
    non-negative.
    """
    # Deprecation warning to flag any surviving callers; the
    # production call site in ``surveyor.compute_navigation_vector``
    # migrated to ``motor_schema_blend()``. Removal gated on the
    # test_motor_schema.py shim-coverage tests being updated.
    import warnings
    warnings.warn(
        'navigation.blend() is deprecated; use motor_schema_blend() '
        'with named basis-behaviour kwargs.',
        DeprecationWarning,
        stacklevel=2,
    )
    if len(vectors) != len(weights):
        raise ValueError(
            f'vectors ({len(vectors)}) and weights ({len(weights)}) '
            f'must be parallel sequences'
        )
    if any(w < 0 for w in weights):
        raise ValueError('all weights must be non-negative')
    sx = sum(v[0] * w for v, w in zip(vectors, weights))
    sy = sum(v[1] * w for v, w in zip(vectors, weights))
    mag = math.hypot(sx, sy)
    if mag < 1e-9:
        return (0.0, 0.0)
    return (sx / mag, sy / mag)
