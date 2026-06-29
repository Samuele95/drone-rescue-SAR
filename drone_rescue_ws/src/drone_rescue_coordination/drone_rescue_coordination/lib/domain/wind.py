"""Software wind model: pure Python, no rclpy.

Wind never moved a drone in any shipped configuration: ``environment_monitor``
publishes the wind vector on ``/environment/weather`` (WeatherState) and
``/environment/wind`` (Vector3), but the Gazebo wind topic
(``/world/<world>/wind``) is unbridged so the physics engine applies no force,
``wind_compensation_gain`` defaulted to 0.0, and the controller's wind branch
only ever subtracted a (zero) compensation: no term made the wind perturb the
drone.

This module supplies the missing physics as a software model the
``drone_controller`` applies to its world-frame velocity command each tick:

    net_wind = (disturbance_gain - compensation_gain) * wind

* ``disturbance_gain``: how strongly the modelled wind pushes the drone.
* ``compensation_gain``: how strongly the controller holds station against it.

At ``compensation_gain == 0`` the drone drifts with the full wind; raising the
compensation reduces the drift; at equal gains the drone holds station. The
function is pure so both the unit test and the controller share one definition.
"""

from __future__ import annotations

from typing import Tuple


def wind_velocity_offset(
    wind: Tuple[float, float],
    disturbance_gain: float,
    compensation_gain: float,
) -> Tuple[float, float]:
    """World-frame velocity the wind adds to the commanded velocity.

    Returns ``((disturbance_gain - compensation_gain) * wx,
    (disturbance_gain - compensation_gain) * wy)``, the net wind perturbation
    after compensation. Positive net values push the drone downwind.
    """
    wx, wy = wind
    net = disturbance_gain - compensation_gain
    return net * wx, net * wy


def integrate_drift(
    wind: Tuple[float, float],
    disturbance_gain: float,
    compensation_gain: float,
    dt: float,
    steps: int,
) -> Tuple[float, float]:
    """Open-loop position drift from the net wind (no feedback).

    Drift over ``steps`` ticks of ``dt`` seconds with no station-keeping.
    A teaching/diagnostic helper that makes "wind moves the drone, compensation
    reduces the drift" a single comparable number; the live controller still
    runs its position PID on top of the disturbance.
    """
    vx, vy = wind_velocity_offset(wind, disturbance_gain, compensation_gain)
    return vx * dt * steps, vy * dt * steps
