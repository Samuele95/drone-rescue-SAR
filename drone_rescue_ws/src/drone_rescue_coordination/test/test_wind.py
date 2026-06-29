"""Tests for the software wind model.

Previously wind never moved a drone: the Gazebo wind topic was unbridged, the
compensation gain defaulted to 0.0, and the controller only ever subtracted a
(zero) compensation, so no term perturbed the drone. ``lib/domain/wind`` adds the
missing disturbance; these tests pin "wind disturbs pose, compensation reduces
drift".

Pure pytest; no rclpy.
"""

from __future__ import annotations

import pytest

from drone_rescue_coordination.lib.domain.wind import (
    integrate_drift,
    wind_velocity_offset,
)


_WIND = (3.0, -2.0)   # world-frame wind vector, m/s


# ----------------------------------------------------------- disturbance term

def test_wind_disturbs_when_uncompensated():
    """With no compensation the full wind is added to the velocity command,
    so the drone drifts downwind (the behaviour that was entirely absent)."""
    vx, vy = wind_velocity_offset(_WIND, disturbance_gain=1.0,
                                  compensation_gain=0.0)
    assert (vx, vy) == pytest.approx(_WIND)


def test_full_compensation_holds_station():
    """Equal disturbance and compensation gains cancel: no net drift."""
    vx, vy = wind_velocity_offset(_WIND, disturbance_gain=1.0,
                                  compensation_gain=1.0)
    assert (vx, vy) == pytest.approx((0.0, 0.0))


def test_partial_compensation_reduces_drift():
    """A non-zero compensation strictly reduces the net wind magnitude."""
    import math
    full = wind_velocity_offset(_WIND, 1.0, 0.0)
    half = wind_velocity_offset(_WIND, 1.0, 0.5)
    mag = lambda v: math.hypot(*v)  # noqa: E731
    assert 0.0 < mag(half) < mag(full)


def test_zero_wind_is_no_op():
    """When it is calm the disturbance vanishes regardless of the gains."""
    assert wind_velocity_offset((0.0, 0.0), 1.0, 0.0) == (0.0, 0.0)


# ----------------------------------------------------------------- drift integ

def test_drift_accumulates_over_time():
    """Open-loop drift grows linearly with elapsed time under steady wind."""
    short = integrate_drift(_WIND, 1.0, 0.0, dt=0.1, steps=10)   # 1 s
    long = integrate_drift(_WIND, 1.0, 0.0, dt=0.1, steps=20)    # 2 s
    assert long[0] == pytest.approx(2.0 * short[0])
    assert long[1] == pytest.approx(2.0 * short[1])


def test_compensation_reduces_accumulated_drift():
    """The headline demo: over the same window, compensation reduces the
    distance the drone is blown off course."""
    import math
    drift_off = integrate_drift(_WIND, 1.0, 0.0, dt=0.1, steps=30)
    drift_on = integrate_drift(_WIND, 1.0, 0.8, dt=0.1, steps=30)
    assert math.hypot(*drift_on) < math.hypot(*drift_off)
