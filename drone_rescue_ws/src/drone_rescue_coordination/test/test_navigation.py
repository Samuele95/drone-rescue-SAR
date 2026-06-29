"""Unit coverage for the lifted NavigationPolicy + Surveyor port.

Pure-Python; exercises the gradient algorithm without ``rclpy.init()``
or a Mock LifecycleNode.
"""

import math

import numpy as np

from drone_rescue_coordination.lib.domain import navigation
from drone_rescue_coordination.lib.domain.navigation import VictimHotspot
from drone_rescue_coordination.lib.domain.surveyor import (
    Surveyor, SurveyorThresholds, SurveyorWeights,
)
from drone_rescue_coordination.lib.domain.value_objects import Position
from drone_rescue_coordination.lib.ports.surveyor_port import (
    SurveyorOutputs, SurveyorSensors,
)


# mesh

def test_build_radial_mesh_centre_distance_zero():
    mesh = navigation.build_radial_mesh(3)
    assert mesh['radius'] == 3
    assert mesh['dr'].shape == (7, 7)
    assert mesh['dist'][3, 3] == 0.0
    # Symmetric: distance to (0,0) == distance to (6,6)
    assert mesh['dist'][0, 0] == mesh['dist'][6, 6]


def test_build_radial_mesh_inv_dist_zero_at_centre():
    mesh = navigation.build_radial_mesh(2)
    assert mesh['inv_dist'][2, 2] == 0.0
    assert mesh['inv_dist'][0, 2] > 0.0


def test_slice_window_clamps_at_edges():
    rl, rh, cl, ch, mr_lo, mr_hi, mc_lo, mc_hi = navigation.slice_window(
        center_row=0, center_col=0, radius=3,
        grid_height=10, grid_width=10,
    )
    assert (rl, rh) == (0, 4)
    assert (cl, ch) == (0, 4)
    # Mesh offsets: the 3 left/top mesh rows fall outside the grid.
    assert (mr_lo, mr_hi) == (3, 7)
    assert (mc_lo, mc_hi) == (3, 7)


# compute_repulsion

def test_compute_repulsion_empty_grid_returns_zero():
    grid = np.zeros((50, 50), dtype=np.float32)
    mesh = navigation.build_radial_mesh(5)
    out = navigation.compute_repulsion(grid, 25, 25, mesh, threshold=0.4)
    assert out == (0.0, 0.0)


def test_compute_repulsion_hotspot_above_threshold_pushes_away():
    """Place a high-pheromone cell to the east of the drone; expect
    a leftward (negative x) repulsion."""
    grid = np.zeros((50, 50), dtype=np.float32)
    grid[25, 28] = 1.0   # east of the drone at (25, 25)
    mesh = navigation.build_radial_mesh(5)
    out = navigation.compute_repulsion(grid, 25, 25, mesh, threshold=0.4)
    assert out[0] < 0.0  # repelled left
    assert abs(out[1]) < 1e-6  # no vertical component


# compute_attraction

def test_compute_attraction_to_nearest_unexplored():
    """Almost-full grid: only one cell to the north is unexplored."""
    grid = np.ones((50, 50), dtype=np.float32)
    grid[22, 25] = 0.0   # north of the drone (smaller row idx = -y? legacy convention)
    mesh = navigation.build_radial_mesh(5)
    out = navigation.compute_attraction(grid, 25, 25, mesh, threshold=0.1)
    # Unit-magnitude pull, direction toward the unexplored cell.
    mag = math.hypot(*out)
    assert abs(mag - 1.0) < 1e-6
    assert out[1] < 0.0  # row delta dr is negative (toward row 22 from 25)


def test_compute_attraction_returns_zero_when_no_unexplored():
    grid = np.ones((50, 50), dtype=np.float32)
    mesh = navigation.build_radial_mesh(5)
    out = navigation.compute_attraction(grid, 25, 25, mesh, threshold=0.1)
    assert out == (0.0, 0.0)


# compute_collision_avoidance

def test_collision_avoidance_repels_from_close_peer():
    self_pos = Position(0.0, 0.0, 25.0)
    peers = {'drone2': Position(3.0, 0.0, 25.0)}   # 3m east, well inside 5m
    out = navigation.compute_collision_avoidance(
        self_pos, peers, collision_distance=5.0,
    )
    assert out[0] < 0.0   # pushed west
    assert abs(out[1]) < 1e-6


def test_collision_avoidance_ignores_far_peers():
    self_pos = Position(0.0, 0.0, 25.0)
    peers = {'drone2': Position(100.0, 0.0, 25.0)}
    out = navigation.compute_collision_avoidance(
        self_pos, peers, collision_distance=5.0,
    )
    assert out == (0.0, 0.0)


def test_collision_avoidance_handles_empty_peer_map():
    self_pos = Position(0.0, 0.0, 25.0)
    out = navigation.compute_collision_avoidance(
        self_pos, {}, collision_distance=5.0,
    )
    assert out == (0.0, 0.0)


# compute_boundary_repulsion

def test_boundary_repulsion_zero_inside_safe_zone():
    out = navigation.compute_boundary_repulsion(
        Position(10.0, 0.0, 25.0), center_x=0.0, center_y=0.0, radius=100.0,
    )
    assert out == (0.0, 0.0)


def test_boundary_repulsion_pulls_inward_near_edge():
    # 80 m from centre, radius 100, past the 50 m safe radius.
    out = navigation.compute_boundary_repulsion(
        Position(80.0, 0.0, 25.0), center_x=0.0, center_y=0.0, radius=100.0,
    )
    assert out[0] < 0.0  # pulled back toward origin (west)
    assert abs(out[1]) < 1e-6


def test_boundary_repulsion_caps_outside():
    """Strength capped at 5.0 even far outside the boundary."""
    out = navigation.compute_boundary_repulsion(
        Position(1000.0, 0.0, 25.0), center_x=0.0, center_y=0.0, radius=100.0,
    )
    # mag = |strength| since (dx/r, dy/r) is unit; strength capped at 5.0.
    assert abs(math.hypot(*out) - 5.0) < 1e-6


# compute_victim_attraction

def test_victim_attraction_pulls_to_in_range_hotspot():
    hs = VictimHotspot(x=10.0, y=0.0, t_seen=0.0, confirmed=False, confidence=0.8)
    out = navigation.compute_victim_attraction(
        Position(0.0, 0.0, 25.0), [hs],
        attraction_radius=25.0, confirm_hover_radius=2.0,
        now_sec=1.0, ttl=30.0,
    )
    assert out[0] > 0.0   # pulled east toward the hotspot
    assert abs(out[1]) < 1e-6


def test_victim_attraction_skips_expired_hotspot():
    hs = VictimHotspot(x=10.0, y=0.0, t_seen=0.0, confirmed=False, confidence=0.8)
    out = navigation.compute_victim_attraction(
        Position(0.0, 0.0, 25.0), [hs],
        attraction_radius=25.0, confirm_hover_radius=2.0,
        now_sec=100.0, ttl=30.0,    # hs is 100 s old, ttl 30, expired
    )
    assert out == (0.0, 0.0)


def test_victim_attraction_skips_when_inside_hover_radius():
    hs = VictimHotspot(x=1.0, y=0.0, t_seen=0.0, confirmed=True, confidence=0.9)
    out = navigation.compute_victim_attraction(
        Position(0.0, 0.0, 25.0), [hs],
        attraction_radius=25.0, confirm_hover_radius=2.0,
        now_sec=1.0, ttl=30.0,
    )
    assert out == (0.0, 0.0)


def test_victim_attraction_confirmed_hotspot_gets_boost():
    """Confirmed hotspot pulls 1.5x harder than unconfirmed."""
    hs_c = VictimHotspot(10.0, 0.0, 0.0, confirmed=True, confidence=0.5)
    hs_u = VictimHotspot(10.0, 0.0, 0.0, confirmed=False, confidence=0.5)
    out_c = navigation.compute_victim_attraction(
        Position(0.0, 0.0, 25.0), [hs_c], 25.0, 2.0, 1.0, 30.0,
    )
    out_u = navigation.compute_victim_attraction(
        Position(0.0, 0.0, 25.0), [hs_u], 25.0, 2.0, 1.0, 30.0,
    )
    assert math.hypot(*out_c) > math.hypot(*out_u)


# blend
# `blend()` is deprecated in favour of motor_schema_blend; suppress the
# warning for these legacy-shim coverage tests. They remain in place for
# back-compat verification until the shim is removed.

import warnings as _warnings


def test_blend_pure_x_is_unit_east():
    with _warnings.catch_warnings():
        _warnings.simplefilter('ignore', DeprecationWarning)
        out = navigation.blend(
            vectors=[(1.0, 0.0), (0.0, 0.0)],
            weights=[1.0, 1.0],
        )
    assert out == (1.0, 0.0)


def test_blend_zero_when_inputs_cancel():
    with _warnings.catch_warnings():
        _warnings.simplefilter('ignore', DeprecationWarning)
        out = navigation.blend(
            vectors=[(1.0, 0.0), (-1.0, 0.0)],
            weights=[1.0, 1.0],
        )
    assert out == (0.0, 0.0)


def test_blend_rejects_negative_weights():
    with _warnings.catch_warnings():
        _warnings.simplefilter('ignore', DeprecationWarning)
        try:
            navigation.blend(vectors=[(1.0, 0.0)], weights=[-1.0])
        except ValueError:
            return
    assert False, 'blend must reject negative weights'


def test_blend_rejects_mismatched_lengths():
    with _warnings.catch_warnings():
        _warnings.simplefilter('ignore', DeprecationWarning)
        try:
            navigation.blend(vectors=[(1.0, 0.0)], weights=[1.0, 1.0])
        except ValueError:
            return
    assert False, 'blend must reject parallel-length mismatch'


# SurveyorPort

def test_surveyor_returns_empty_when_pose_missing():
    s = Surveyor(weights=SurveyorWeights(), thresholds=SurveyorThresholds())
    out = s.tick(SurveyorSensors(
        now_sec=1.0, current_position=None, pheromone_grid=None,
        grid_origin_x=-100.0, grid_origin_y=-100.0, cell_resolution=1.0,
        battery_level=1.0, zone_warn=False,
    ))
    assert out == SurveyorOutputs()


def test_surveyor_returns_empty_when_grid_missing():
    s = Surveyor(weights=SurveyorWeights(), thresholds=SurveyorThresholds())
    out = s.tick(SurveyorSensors(
        now_sec=1.0, current_position=Position(0.0, 0.0, 25.0),
        pheromone_grid=None,
        grid_origin_x=-100.0, grid_origin_y=-100.0, cell_resolution=1.0,
        battery_level=1.0, zone_warn=False,
    ))
    assert out == SurveyorOutputs()


def test_surveyor_produces_target_in_unexplored_direction():
    """End-to-end: an empty grid means everywhere is unexplored;
    boundary repulsion is zero at origin; output target sits at the
    survey altitude one step from current position."""
    grid = np.zeros((200, 200), dtype=np.float32)
    s = Surveyor(weights=SurveyorWeights(), thresholds=SurveyorThresholds())
    out = s.tick(SurveyorSensors(
        now_sec=1.0, current_position=Position(0.0, 0.0, 25.0),
        pheromone_grid=grid,
        grid_origin_x=-100.0, grid_origin_y=-100.0, cell_resolution=1.0,
        battery_level=1.0, zone_warn=False,
    ))
    # No active attractive cells in a flat-zero grid (everything is
    # below threshold but the only cell with dist > 0 in the attraction
    # window; the nearest such cell is the immediate neighbour, so
    # output should be non-empty).
    assert out.target_pose is not None
    assert out.target_pose.z == 25.0   # survey altitude held
