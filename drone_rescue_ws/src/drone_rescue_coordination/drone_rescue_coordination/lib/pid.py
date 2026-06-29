"""PID controller: pure-Python, no rclpy.

Extracted from `drone_controller.PIDController` so the
math is unit-testable without spinning up a ROS node. The lifecycle
shell in `drone_controller.py` now imports this as a thin shim.

Pure-Python algorithm cores live in `lib/` modules with no rclpy
dependency, so they're unit-testable without rclpy.init(). Same pattern
as AuctionEngine and sector_geometry.
"""

from __future__ import annotations

from typing import Optional


class PIDController:
    """Simple PID controller for position/velocity control.

    State carried across `compute()` calls: integral accumulator,
    previous error, previous timestamp. Anti-windup clamps the
    integral term to ±10.0. Output is clipped to `output_min` /
    `output_max`.

    The first call (when `prev_time is None`) returns the pure-P
    response with no derivative kick or integral accumulation:
    cold-start safe.
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -float('inf'),
                 output_max: float = float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time: Optional[float] = None

    def compute(self, error: float, current_time: float) -> float:
        """Compute PID output given current error and timestamp.

        `current_time` is wallclock-seconds (any monotonic source).
        The loop is dt-aware so a variable control rate doesn't
        warp the integral / derivative terms.
        """
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return self.kp * error

        dt = current_time - self.prev_time
        if dt <= 0:
            return self.kp * error

        p_term = self.kp * error

        # Anti-windup clamp on the integral state itself.
        self.integral += error * dt
        self.integral = max(-10.0, min(10.0, self.integral))
        i_term = self.ki * self.integral

        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        self.prev_error = error
        self.prev_time = current_time

        output = p_term + i_term + d_term
        return max(self.output_min, min(self.output_max, output))

    def reset(self) -> None:
        """Clear integral state and reset cold-start flag."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
