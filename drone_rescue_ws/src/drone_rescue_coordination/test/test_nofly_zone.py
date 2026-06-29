"""Tests for the frozen NoFlyZone value object.

Pure-Python; no rclpy.init() needed.
"""

from __future__ import annotations

import pytest

from drone_rescue_coordination.zone_manager import NoFlyZone


def test_valid_polygon_constructs():
    z = NoFlyZone(
        name='park',
        zone_type='polygon',
        vertices=((0, 0), (10, 0), (10, 10), (0, 10)),
    )
    assert z.name == 'park'
    assert len(z.vertices) == 4


def test_valid_circle_constructs():
    z = NoFlyZone(
        name='helipad',
        zone_type='circle',
        center=(5.0, 5.0),
        radius=3.0,
    )
    assert z.center == (5.0, 5.0)
    assert z.radius == 3.0


def test_polygon_with_too_few_vertices_raises():
    with pytest.raises(ValueError, match='polygon needs'):
        NoFlyZone(
            name='bad',
            zone_type='polygon',
            vertices=((0, 0), (1, 1)),
        )


def test_circle_missing_center_raises():
    with pytest.raises(ValueError, match='requires center'):
        NoFlyZone(
            name='bad',
            zone_type='circle',
            radius=5.0,
        )


def test_circle_missing_radius_raises():
    with pytest.raises(ValueError, match='requires center'):
        NoFlyZone(
            name='bad',
            zone_type='circle',
            center=(0, 0),
        )


def test_circle_negative_radius_raises():
    with pytest.raises(ValueError, match='radius must be'):
        NoFlyZone(
            name='bad',
            zone_type='circle',
            center=(0, 0),
            radius=-1.0,
        )


def test_unknown_zone_type_raises():
    with pytest.raises(ValueError, match='unknown zone_type'):
        NoFlyZone(
            name='bad',
            zone_type='triangle',
            vertices=((0, 0), (1, 0), (0, 1)),
        )


def test_frozen_assignment_blocked():
    """VO is frozen; mutation forbidden."""
    z = NoFlyZone(
        name='x',
        zone_type='circle',
        center=(0, 0),
        radius=5.0,
    )
    with pytest.raises(Exception):
        z.radius = 10.0   # type: ignore[misc]


def test_defaults_for_optional_fields():
    """Adding a new optional field shouldn't break existing callers.
    priority/reason/buffer_distance/altitudes all have defaults."""
    z = NoFlyZone(
        name='minimal',
        zone_type='circle',
        center=(0, 0),
        radius=1.0,
    )
    assert z.priority == 'medium'
    assert z.reason == ''
    assert z.buffer_distance == 2.0
    assert z.min_altitude == 0.0
    assert z.max_altitude == 100.0
