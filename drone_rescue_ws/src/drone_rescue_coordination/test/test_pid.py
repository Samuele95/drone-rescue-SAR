"""Unit tests for lib/pid.PIDController.

Pure-Python; no rclpy.init() needed.
"""

from __future__ import annotations

from drone_rescue_coordination.lib.pid import PIDController


def test_cold_start_returns_pure_p():
    """First call has no prev_time: P term only, no derivative."""
    pid = PIDController(kp=2.0, ki=0.5, kd=1.0)
    assert pid.compute(error=3.0, current_time=0.0) == 6.0
    # State recorded for next call.
    assert pid.prev_time == 0.0
    assert pid.prev_error == 3.0


def test_dt_zero_returns_pure_p_no_state_advance():
    """Repeating the same timestamp must not zero-divide."""
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
    pid.compute(error=1.0, current_time=0.0)
    pid.compute(error=2.0, current_time=0.0)
    # Integral untouched; the second call short-circuits.
    assert pid.integral == 0.0


def test_integral_anti_windup_clamps_to_ten():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
    # Prime with cold start.
    pid.compute(error=1.0, current_time=0.0)
    # Drive a large error over many seconds.
    for t in range(1, 20):
        pid.compute(error=100.0, current_time=float(t))
    assert pid.integral == 10.0


def test_output_clip_min_max():
    """Clipping applies on the steady-state (post-cold-start) path.
    Cold-start returns the unclipped kp*error per the original
    contract, exercised by `test_cold_start_returns_pure_p`."""
    pid = PIDController(kp=10.0, ki=0.0, kd=0.0,
                       output_min=-1.0, output_max=1.0)
    # Warm up so prev_time is set.
    pid.compute(error=0.0, current_time=0.0)
    # Now steady-state clipping fires.
    assert pid.compute(error=5.0, current_time=1.0) == 1.0
    assert pid.compute(error=-5.0, current_time=2.0) == -1.0


def test_derivative_term_responds_to_change():
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0)
    pid.compute(error=0.0, current_time=0.0)
    # Error jumps from 0 to 1 over dt=1: derivative = 1.0.
    out = pid.compute(error=1.0, current_time=1.0)
    assert out == 1.0


def test_reset_clears_state():
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
    pid.compute(error=2.0, current_time=0.0)
    pid.compute(error=3.0, current_time=1.0)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.prev_error == 0.0
    assert pid.prev_time is None
    # Next call is cold-start P-only.
    assert pid.compute(error=4.0, current_time=10.0) == 4.0


def test_droneState_enum_exported():
    """DroneState is importable from lib/domain."""
    from drone_rescue_coordination.lib.domain.drone_state import DroneState
    assert DroneState.IDLE.value == 0
    assert DroneState.LANDING.value == 4
