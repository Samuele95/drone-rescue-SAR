"""Unit tests for the terrain ElevationModel (lib/domain/elevation.py).

Pure-python pytest; no ROS node.
"""

from __future__ import annotations

import pytest

from drone_rescue_coordination.lib.domain.elevation import ElevationModel


# ------------------------------------------------------------ flat (default)

def test_flat_is_zero_everywhere():
    m = ElevationModel()  # default kind 'flat'
    for x, y in [(0.0, 0.0), (100.0, -40.0), (-70.0, 70.0)]:
        assert m.elevation_at(x, y) == 0.0


def test_flat_is_flat_property():
    assert ElevationModel.flat().is_flat is True
    assert ElevationModel().kind == 'flat'


# ------------------------------------------------------------ planar

def test_planar_gradient():
    m = ElevationModel('planar', base=2.0, slope_x=0.1, slope_y=-0.05)
    assert m.elevation_at(0.0, 0.0) == pytest.approx(2.0)
    assert m.elevation_at(10.0, 0.0) == pytest.approx(3.0)     # 2 + 0.1*10
    assert m.elevation_at(0.0, 20.0) == pytest.approx(1.0)     # 2 - 0.05*20
    assert m.elevation_at(10.0, 20.0) == pytest.approx(2.0)    # 2 + 1 - 1


def test_planar_with_zero_slopes_is_flat_valued():
    m = ElevationModel('planar', base=0.0, slope_x=0.0, slope_y=0.0)
    assert m.elevation_at(123.0, -456.0) == 0.0
    assert m.is_flat is True


# ------------------------------------------------------------ from_slopes

def test_from_slopes_zero_returns_flat():
    m = ElevationModel.from_slopes(0.0, 0.0)
    assert m.kind == 'flat'
    assert m.is_flat is True


def test_from_slopes_nonzero_is_planar():
    m = ElevationModel.from_slopes(0.2, 0.0)
    assert m.kind == 'planar'
    assert m.elevation_at(5.0, 99.0) == pytest.approx(1.0)  # 0.2 * 5


# ------------------------------------------------------------ validation

def test_unknown_kind_raises():
    with pytest.raises(ValueError, match='unknown elevation model kind'):
        ElevationModel('mountains')
