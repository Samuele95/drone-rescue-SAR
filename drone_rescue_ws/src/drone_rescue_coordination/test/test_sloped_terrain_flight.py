"""Sloped-terrain flight coupling tests.

Elevation was implemented but inert: terrain_slope_x/y defaulted to 0.0 and no
scenario or launch ever set them, so the planar ElevationModel never tilted and
flight was always flat. The gradient is plumbed through the scenario schema and
launch into BOTH mission_manager (AGL scan altitude) and victim_detector (AGL
detection gate), paired with worlds/earthquake_zone_sloped.sdf.

These tests pin the coupling: the two nodes build the same elevation from the
same params, the AGL flight altitude is constant over the slope, and detection
still works over the slope: at the disk edge the absolute z would
otherwise leave the band.

Pure pytest: no rclpy.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from drone_rescue_coordination.lib.domain.elevation import ElevationModel
from drone_rescue_coordination.victim_detector import VictimDetector


# The shipped sloped_terrain.yaml gradient (and the tilt of the ground plane in
# earthquake_zone_sloped.sdf: z = 0.04*x - 0.02*y).
_SLOPE_X, _SLOPE_Y = 0.04, -0.02
_SURVEY_ALT = 25.0


def _elevation():
    return ElevationModel.from_slopes(slope_x=_SLOPE_X, slope_y=_SLOPE_Y)


def test_mission_manager_and_detector_share_one_elevation():
    """Both nodes construct from the same params via from_slopes, so they agree
    at every sample point: the AGL flight altitude and the AGL detection gate
    reference the same terrain."""
    a = _elevation()
    b = _elevation()
    for x, y in [(0.0, 0.0), (55.0, 48.0), (-12.0, 16.0), (44.0, -28.0)]:
        assert a.elevation_at(x, y) == pytest.approx(b.elevation_at(x, y))


def test_agl_flight_altitude_is_constant_over_slope():
    """survey_altitude + elevation_at is the absolute waypoint z; subtracting
    the terrain recovers a constant AGL everywhere (mission_manager._begin_scan
    flies this, victim_detector subtracts it)."""
    elev = _elevation()
    for x, y in [(0.0, 0.0), (55.0, 48.0), (-30.0, 30.0)]:
        absolute_z = _SURVEY_ALT + elev.elevation_at(x, y)
        agl = absolute_z - elev.elevation_at(x, y)
        assert agl == pytest.approx(_SURVEY_ALT)


def test_detection_holds_over_slope_edge():
    """At the disk edge the terrain rises, so the absolute z exceeds the survey
    altitude; with the AGL gate the drone is still DetectionCapable. A naive
    absolute-z gate (band [3, 50]) would clip a steeper case: pin the AGL one."""
    elev = _elevation()
    ns = SimpleNamespace(min_height=3.0, max_height=50.0, _elevation=elev)
    # Edge point (55, 48): terrain = 0.04*55 - 0.02*48 = 2.2 - 0.96 = 1.24 m.
    x, y = 55.0, 48.0
    absolute_z = _SURVEY_ALT + elev.elevation_at(x, y)
    pose = SimpleNamespace(x=x, y=y, z=absolute_z, yaw=0.0)
    assert VictimDetector._within_detection_band(ns, pose) is True


def test_steep_slope_would_clip_without_agl():
    """Sanity that the AGL handling is load-bearing: a steeper terrain pushes a
    25 m-AGL drone's absolute z above the 50 m ceiling, yet AGL keeps it in
    band: exactly the terrain-disables-detection defect that was fixed."""
    steep = ElevationModel.from_slopes(slope_x=0.5, slope_y=0.0)  # 0.5 m/m
    ns = SimpleNamespace(min_height=3.0, max_height=50.0, _elevation=steep)
    x, y = 60.0, 0.0           # terrain = 30 m; absolute z = 25 + 30 = 55 m
    pose = SimpleNamespace(x=x, y=y, z=_SURVEY_ALT + steep.elevation_at(x, y),
                           yaw=0.0)
    assert pose.z > ns.max_height                      # naive absolute gate clips
    assert VictimDetector._within_detection_band(ns, pose) is True  # AGL holds
