"""Unit tests for the pixel-to-ground projection (lib/projection.py).

These tests pin the geometry of victim localization. The original
``victim_detector._project_to_ground`` ignored the drone's yaw, so every
off-centre detection was rotated by the (unaccounted) heading and confirmed
victims landed tens of metres from ground truth: every recorded run scored
``true_positives = 0``. The yaw-rotation cases below are exactly what would
have caught that bug.

Pure-python pytest; no ROS dependency.
"""

from __future__ import annotations

import math

import pytest

from drone_rescue_coordination.lib.projection import (
    project_pixel_to_ground,
    yaw_from_quaternion,
)


# Camera matches drone_rescue_gazebo/.../quadrotor/model.sdf: 480x360, 90deg FOV.
_SHAPE = (360, 480, 3)
_FOV = math.radians(90.0)
_CENTRE = (240.0, 180.0)


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# -------------------------------------------------- centre-pixel invariance

@pytest.mark.parametrize('yaw_deg', [0.0, 37.0, 90.0, 180.0, 270.0, -45.0])
@pytest.mark.parametrize('drone_xy', [(0.0, 0.0), (44.0, -28.0), (-12.0, 16.0)])
def test_centre_pixel_projects_to_drone_xy(yaw_deg, drone_xy):
    """The image centre is nadir: it must project to the drone's own (x, y)
    regardless of heading or altitude."""
    gx, gy = project_pixel_to_ground(
        _CENTRE, _SHAPE, drone_xy[0], drone_xy[1], 25.0,
        math.radians(yaw_deg), _FOV,
    )
    assert _dist((gx, gy), drone_xy) < 1e-6


# -------------------------------------------------- yaw-0 reference geometry

def test_yaw0_right_edge():
    """At yaw 0, 25 m altitude, 90deg FOV: ground_width = 2*25*tan(45) = 50 m.
    The right-edge / vertical-centre pixel projects 25 m along world +x."""
    gx, gy = project_pixel_to_ground(
        (480.0, 180.0), _SHAPE, 0.0, 0.0, 25.0, 0.0, _FOV,
    )
    assert _dist((gx, gy), (25.0, 0.0)) < 1e-6


def test_yaw0_top_edge():
    """Image +y is down -> body -y. Top edge (cy=0) projects to world +y."""
    gx, gy = project_pixel_to_ground(
        (240.0, 0.0), _SHAPE, 0.0, 0.0, 25.0, 0.0, _FOV,
    )
    # ground_height = 50 * (360/480) = 37.5; offset_y = -0.5 -> +18.75 m.
    assert _dist((gx, gy), (0.0, 18.75)) < 1e-6


# -------------------------------------------------- yaw rotation (the bug)

@pytest.mark.parametrize('yaw_deg', [90.0, 180.0, 270.0, 45.0])
def test_yaw_rotates_offset_about_drone(yaw_deg):
    """A fixed off-centre pixel must, under yaw theta, project to the yaw-0
    point rotated by theta about the drone. Ignoring yaw (the original bug)
    leaves the point unrotated."""
    pixel = (480.0, 180.0)
    drone = (10.0, -5.0)
    alt = 25.0

    base = project_pixel_to_ground(pixel, _SHAPE, *drone, alt, 0.0, _FOV)
    base_vec = (base[0] - drone[0], base[1] - drone[1])  # yaw-0 displacement

    yaw = math.radians(yaw_deg)
    rotated = project_pixel_to_ground(pixel, _SHAPE, *drone, alt, yaw, _FOV)

    c, s = math.cos(yaw), math.sin(yaw)
    expected = (
        drone[0] + c * base_vec[0] - s * base_vec[1],
        drone[1] + s * base_vec[0] + c * base_vec[1],
    )
    assert _dist(rotated, expected) < 1e-6


def test_yaw90_sends_plus_x_offset_to_plus_y():
    """Concrete check: the right-edge pixel (yaw-0 -> +x) at yaw 90deg -> +y."""
    gx, gy = project_pixel_to_ground(
        (480.0, 180.0), _SHAPE, 0.0, 0.0, 25.0, math.radians(90.0), _FOV,
    )
    assert _dist((gx, gy), (0.0, 25.0)) < 1e-6


# -------------------------------------------------- altitude scaling

def test_altitude_scales_offset_linearly():
    """Doubling altitude doubles the ground footprint, hence the offset."""
    pixel = (480.0, 180.0)
    near = project_pixel_to_ground(pixel, _SHAPE, 0.0, 0.0, 10.0, 0.0, _FOV)
    far = project_pixel_to_ground(pixel, _SHAPE, 0.0, 0.0, 20.0, 0.0, _FOV)
    assert _dist(far, (0.0, 0.0)) == pytest.approx(2.0 * _dist(near, (0.0, 0.0)))


# -------------------------------------------------- yaw_from_quaternion

def test_yaw_from_quaternion_identity():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


@pytest.mark.parametrize('deg', [30.0, 90.0, 180.0, -90.0])
def test_yaw_from_quaternion_known_angles(deg):
    """A pure-yaw quaternion (z = sin(yaw/2), w = cos(yaw/2)) round-trips."""
    half = math.radians(deg) / 2.0
    got = yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half))
    # atan2 wraps to (-pi, pi]; compare on the circle.
    assert math.cos(got) == pytest.approx(math.cos(math.radians(deg)), abs=1e-9)
    assert math.sin(got) == pytest.approx(math.sin(math.radians(deg)), abs=1e-9)
