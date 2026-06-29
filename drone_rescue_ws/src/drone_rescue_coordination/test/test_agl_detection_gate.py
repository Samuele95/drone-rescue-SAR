"""Regression tests for the AGL detection-altitude gate.

The bug it closes: ``victim_detector`` gated frame processing on the drone's
ABSOLUTE odometry z (``pos.z < min_height or pos.z > max_height``), while scan
waypoints are flown at constant above-ground-level height
(``survey_altitude + elevation_at(x, y)``, mission_manager.py:1205-1211). Over
any terrain with elevation > 0 the absolute z of an AGL-held drone exceeds the
``max_detection_height`` ceiling, so the camera silently disabled itself, a
direct contributor to ``true_positives = 0`` once terrain is non-flat.

These tests mirror SWRL S6's ``ex:drone_terrain_agl_example`` (model.html):
a drone at (40, 10) over planar terrain (base 2.0, slope_x 0.1, slope_y -0.05)
holding the correct 25 m AGL has absolute z = 5.5 + 25 = 30.5 m. The OLD gate
(absolute z vs [5, 25]) rejects it; the FIXED gate (AGL vs [5, 25]) accepts it.

Pure pytest; no rclpy.init(); we exercise the unbound method against a
duck-typed ``self`` (same style as test_detection_range_decay.py).
"""

from __future__ import annotations

from types import SimpleNamespace

from drone_rescue_coordination.lib.domain.elevation import ElevationModel
from drone_rescue_coordination.victim_detector import VictimDetector


def _node(*, min_h: float = 5.0, max_h: float = 25.0,
          elevation: ElevationModel = None):
    """Minimal duck-typed surface for ``_within_detection_band``."""
    return SimpleNamespace(
        min_height=min_h,
        max_height=max_h,
        _elevation=elevation if elevation is not None else ElevationModel.flat(),
    )


def _pose(x: float, y: float, z: float, yaw: float = 0.0):
    return SimpleNamespace(x=x, y=y, z=z, yaw=yaw)


# ------------------------------------------------- the terrain-disables defect

def test_agl_drone_over_terrain_stays_detection_capable():
    """The S6 counter-example: correct 25 m AGL over 5.5 m terrain (absolute
    30.5 m) must be IN-band once the gate measures AGL, not absolute z."""
    elev = ElevationModel.from_slopes(slope_x=0.1, slope_y=-0.05, base=2.0)
    ns = _node(elevation=elev)
    # elevation_at(40, 10) = 2.0 + 0.1*40 - 0.05*10 = 5.5; AGL = 30.5 - 5.5 = 25.
    assert VictimDetector._within_detection_band(ns, _pose(40.0, 10.0, 30.5)) is True


def test_absolute_z_above_ceiling_rejected_when_terrain_flat():
    """Sanity: on flat terrain AGL == absolute z, so the old behaviour holds:
    a drone genuinely above the ceiling is still rejected (no permissive gate)."""
    ns = _node(elevation=ElevationModel.flat())
    assert VictimDetector._within_detection_band(ns, _pose(0.0, 0.0, 30.5)) is False


# ------------------------------------------------------------- band semantics

def test_flat_terrain_band_inclusive_both_ends():
    """The band is inclusive [min, max] (old gate: pos.z < min OR pos.z > max
    -> skip). On flat terrain AGL == z, so the boundaries are in-band."""
    ns = _node(min_h=5.0, max_h=25.0, elevation=ElevationModel.flat())
    assert VictimDetector._within_detection_band(ns, _pose(0.0, 0.0, 5.0)) is True
    assert VictimDetector._within_detection_band(ns, _pose(0.0, 0.0, 25.0)) is True
    assert VictimDetector._within_detection_band(ns, _pose(0.0, 0.0, 4.99)) is False
    assert VictimDetector._within_detection_band(ns, _pose(0.0, 0.0, 25.01)) is False


def test_terrain_does_not_make_a_too_low_drone_pass():
    """A drone flying BELOW min AGL is rejected even over raised terrain: the
    fix is real AGL handling, not a blanket widening of the band."""
    elev = ElevationModel.from_slopes(slope_x=0.1, slope_y=-0.05, base=2.0)
    ns = _node(elevation=elev)
    # At (40,10) terrain = 5.5; a drone at absolute 9.0 holds AGL 3.5 < min 5.
    assert VictimDetector._within_detection_band(ns, _pose(40.0, 10.0, 9.0)) is False


def test_terrain_rescues_an_absolute_z_that_a_naive_gate_would_drop():
    """Different (x,y) on the slope: at (60, 0) terrain = 2.0 + 6.0 = 8.0; a
    drone holding 20 m AGL sits at absolute 28.0 m, over the [5,25] ceiling on
    a naive absolute gate, but in-band on AGL."""
    elev = ElevationModel.from_slopes(slope_x=0.1, slope_y=-0.05, base=2.0)
    ns = _node(elevation=elev)
    assert VictimDetector._within_detection_band(ns, _pose(60.0, 0.0, 28.0)) is True
