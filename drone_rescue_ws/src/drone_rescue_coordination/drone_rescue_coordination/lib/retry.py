"""Exponential-backoff retry helper: pure-Python core + thin rclpy adapter.

Extracted from `drone_controller.OdomRetryManager`.
The retry-state machine itself is pure Python; the cancellable
timer lives behind a small node-adapter so the helper is unit-
testable without `rclpy.init()`.

Same Hexagonal-port pattern the project already adopted for `Clock`
and `EventPort`.
"""

from __future__ import annotations

from typing import Optional, Protocol


class _RetryTimerHost(Protocol):
    """Minimal subset of `rclpy.node.Node` the adapter needs.

    Production callers pass the LifecycleNode itself; tests pass a
    SimpleNamespace with a `create_timer` mock.
    """

    def create_timer(self, period: float, callback): ...
    def get_logger(self): ...


class BackoffRetry:
    """Exponential-backoff retry state.

    `delay(n)` is `base_delay * 2**n` for the n-th retry; the
    `attempt(failure_type)` method advances the counter, fires the
    log line, schedules the timer, and returns True if a retry is
    still in budget. After `max_retries` exhausts, returns False.

    Pure logic: accepts a `_RetryTimerHost` so consumers wire the
    rclpy timer through the adapter; tests substitute a fake host.
    """

    def __init__(
        self,
        host: _RetryTimerHost,
        max_retries: int = 3,
        base_delay: float = 1.0,
    ):
        self.host = host
        self.max_retries = int(max_retries)
        self.base_delay = float(base_delay)
        self.retry_count: int = 0
        self.retry_timer = None
        self.in_retry: bool = False

    def attempt_retry(self, failure_type: str) -> bool:
        """Schedule the next retry. Returns False once exhausted."""
        if self.retry_count >= self.max_retries:
            self.host.get_logger().error(
                f'{failure_type}: Max retries ({self.max_retries}) '
                f'exceeded - escalating'
            )
            return False

        delay = self.base_delay * (2 ** self.retry_count)
        self.host.get_logger().warning(
            f'{failure_type}: Retry {self.retry_count + 1}/'
            f'{self.max_retries} after {delay:.1f}s'
        )
        self.in_retry = True
        self.retry_count += 1
        if self.retry_timer is not None:
            self.retry_timer.cancel()
        self.retry_timer = self.host.create_timer(
            delay, self._on_retry_timeout,
        )
        return True

    def _on_retry_timeout(self) -> None:
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
        self.in_retry = False

    def reset(self) -> None:
        """Clear retry state on successful data reception."""
        if self.retry_count > 0:
            self.host.get_logger().info(
                'Retry manager reset - data received'
            )
        self.retry_count = 0
        self.in_retry = False
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
