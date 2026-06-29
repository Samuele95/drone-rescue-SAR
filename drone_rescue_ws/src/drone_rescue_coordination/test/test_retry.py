"""Unit tests for lib/retry.BackoffRetry.

The retry state machine is pure-Python; tests use a fake host with a
mock `create_timer`.
"""

from __future__ import annotations

from drone_rescue_coordination.lib.retry import BackoffRetry


class _FakeLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.infos = []
    def error(self, msg):    self.errors.append(msg)
    def warning(self, msg):  self.warnings.append(msg)
    def info(self, msg):     self.infos.append(msg)


class _FakeTimer:
    def __init__(self):
        self.cancelled = False
    def cancel(self):
        self.cancelled = True


class _FakeHost:
    def __init__(self):
        self._logger = _FakeLogger()
        self.created_timers = []
    def create_timer(self, period, callback):
        t = _FakeTimer()
        self.created_timers.append((period, t))
        return t
    def get_logger(self):
        return self._logger


def test_first_retry_doubles_base_delay_each_time():
    host = _FakeHost()
    r = BackoffRetry(host, max_retries=3, base_delay=1.0)
    assert r.attempt_retry('odom_lost') is True
    # First retry uses base_delay × 2**0 = 1.0
    assert host.created_timers[-1][0] == 1.0
    assert r.attempt_retry('odom_lost') is True
    # Second retry: 1.0 × 2 = 2.0
    assert host.created_timers[-1][0] == 2.0
    assert r.attempt_retry('odom_lost') is True
    # Third: 4.0
    assert host.created_timers[-1][0] == 4.0


def test_attempt_retry_returns_false_after_exhaustion():
    host = _FakeHost()
    r = BackoffRetry(host, max_retries=2)
    assert r.attempt_retry('x') is True
    assert r.attempt_retry('x') is True
    assert r.attempt_retry('x') is False
    assert any('Max retries' in e for e in host.get_logger().errors)


def test_reset_clears_state_and_cancels_timer():
    host = _FakeHost()
    r = BackoffRetry(host, max_retries=3)
    r.attempt_retry('x')
    timer = r.retry_timer
    r.reset()
    assert r.retry_count == 0
    assert r.in_retry is False
    assert r.retry_timer is None
    assert timer.cancelled is True


def test_in_retry_flag_lifecycle():
    host = _FakeHost()
    r = BackoffRetry(host)
    assert r.in_retry is False
    r.attempt_retry('x')
    assert r.in_retry is True
    r._on_retry_timeout()
    assert r.in_retry is False


def test_reset_log_only_when_state_was_nonzero():
    host = _FakeHost()
    r = BackoffRetry(host)
    r.reset()
    # No retries yet → no INFO log.
    assert host.get_logger().infos == []
    r.attempt_retry('x')
    r.reset()
    # After a retry attempt → INFO log fires.
    assert any('reset' in m for m in host.get_logger().infos)
