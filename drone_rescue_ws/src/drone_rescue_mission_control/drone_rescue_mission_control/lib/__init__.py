"""Mission Control pure-Python helpers (no rclpy, no PyQt).

Houses the score / metric / finaliser helpers that previously
lived inside the `mission_recorder` LifecycleNode, so they're
unit-testable without `rclpy.init()`.
"""
