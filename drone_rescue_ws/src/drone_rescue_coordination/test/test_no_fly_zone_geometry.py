"""Unit coverage for the lifted NoFlyZone numpy geometry.

Pure-Python; exercises distance / containment without rclpy or
a LifecycleNode. Pairs with the NoFlyZone value-object move.
"""

import numpy as np

from drone_rescue_coordination.lib.domain.no_fly_zone_geometry import (
    distance_to_zone,
    distance_to_polygon_edge_np,
    point_in_polygon_np,
    precompute_zone_state,
)
from drone_rescue_coordination.lib.domain.value_objects import NoFlyZone


# precompute

def test_precompute_circle_valid():
    zone = NoFlyZone(name='z1', zone_type='circle', center=(10.0, 5.0), radius=3.0)
    state = precompute_zone_state(zone)
    assert state['valid'] is True
    assert state['zone_type'] == 'circle'
    assert state['radius'] == 3.0


def test_precompute_polygon_valid():
    zone = NoFlyZone(
        name='z2', zone_type='polygon',
        vertices=((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),
    )
    state = precompute_zone_state(zone)
    assert state['valid'] is True
    assert state['zone_type'] == 'polygon'
    assert state['vertices'].shape == (4, 2)
    assert state['edge_starts'].shape == (4, 2)
    assert state['edge_ends'].shape == (4, 2)


# distance_to_zone

def test_distance_to_zone_circle_outside_positive():
    zone = NoFlyZone(name='z', zone_type='circle', center=(0.0, 0.0), radius=5.0)
    state = precompute_zone_state(zone)
    d = distance_to_zone((10.0, 0.0), zone, state)
    # 10m from centre, 5m radius, 2m default buffer -> 10 - (5 + 2) = 3.0
    assert d == 3.0


def test_distance_to_zone_circle_inside_negative():
    zone = NoFlyZone(name='z', zone_type='circle', center=(0.0, 0.0), radius=10.0)
    state = precompute_zone_state(zone)
    d = distance_to_zone((1.0, 0.0), zone, state)
    # 1m from centre, 10m radius, 2m buffer -> 1 - 12 = -11 (deep inside)
    assert d == -11.0


def test_distance_to_zone_polygon_outside_returns_edge_minus_buffer():
    zone = NoFlyZone(
        name='z', zone_type='polygon',
        vertices=((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),
        buffer_distance=1.0,
    )
    state = precompute_zone_state(zone)
    # Point 5m east of the rightmost edge, inside altitude band.
    d = distance_to_zone((15.0, 5.0), zone, state)
    # Nearest edge distance is 5m; minus 1m buffer -> 4.0
    assert abs(d - 4.0) < 1e-9


def test_distance_to_zone_polygon_inside_returns_negative_edge_distance():
    zone = NoFlyZone(
        name='z', zone_type='polygon',
        vertices=((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),
    )
    state = precompute_zone_state(zone)
    d = distance_to_zone((5.0, 5.0), zone, state)
    # Inside polygon, nearest edge is 5m away -> -5.0
    assert abs(d - (-5.0)) < 1e-9


def test_distance_to_zone_invalid_returns_inf():
    zone = NoFlyZone(name='z', zone_type='circle', center=(0.0, 0.0), radius=1.0)
    invalid_state = {'zone_type': 'circle', 'valid': False}
    assert distance_to_zone((0.0, 0.0), zone, invalid_state) == float('inf')


def test_distance_to_zone_none_state_returns_inf():
    zone = NoFlyZone(name='z', zone_type='circle', center=(0.0, 0.0), radius=1.0)
    assert distance_to_zone((0.0, 0.0), zone, None) == float('inf')


# point_in_polygon

def test_point_in_polygon_inside_true():
    verts = np.array([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)])
    assert point_in_polygon_np(np.array([5.0, 5.0]), verts) is True


def test_point_in_polygon_outside_false():
    verts = np.array([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)])
    assert point_in_polygon_np(np.array([15.0, 5.0]), verts) is False


# edge distance

def test_distance_to_polygon_edge_minimum_returned():
    verts = np.array([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)])
    starts = verts
    ends = np.roll(verts, -1, axis=0)
    d = distance_to_polygon_edge_np(np.array([15.0, 5.0]), starts, ends)
    assert abs(d - 5.0) < 1e-9
