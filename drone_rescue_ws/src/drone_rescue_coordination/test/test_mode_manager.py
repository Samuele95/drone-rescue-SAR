"""Unit coverage for the lifecycle_manager.ModeManager Clock-port
adoption.

The ``ModeManager`` previously used ``self.node.get_clock().now()``
directly which prevented sub-millisecond unit testing of the
30-second persistent-warning escalation. With ``clock_fn`` injected,
the tests below drive the clock from a FakeClock and assert the
NORMAL → DEGRADED transition fires when (and only when) the warning
window crosses the threshold.
"""

from types import SimpleNamespace

import diagnostic_msgs.msg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from drone_rescue_coordination.lifecycle_manager import (
    ModeManager, SystemModeEnum,
)


class _FakeClockFn:
    """Mutable float-second clock: tests step it forward explicitly."""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def _make_owner():
    """Stand-in for the LifecycleManager: only the ``get_logger``
    surface ModeManager touches is needed for transition_to."""
    logger = SimpleNamespace(warn=lambda msg: None)
    return SimpleNamespace(get_logger=lambda: logger)


def _diag_with_warning(hardware_id: str = 'drone1-controller') -> DiagnosticArray:
    arr = DiagnosticArray()
    s = DiagnosticStatus()
    s.level = diagnostic_msgs.msg.DiagnosticStatus.WARN
    s.hardware_id = hardware_id
    arr.status = [s]
    return arr


def _diag_with_error() -> DiagnosticArray:
    arr = DiagnosticArray()
    s = DiagnosticStatus()
    s.level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
    s.hardware_id = 'pheromone_server'
    arr.status = [s]
    return arr


def _diag_clear() -> DiagnosticArray:
    arr = DiagnosticArray()
    arr.status = []
    return arr


# 30-second persistent-warning

def test_persistent_warning_does_not_fire_under_threshold():
    clock = _FakeClockFn()
    mm = ModeManager(_make_owner(), clock_fn=clock)
    mm.update_from_diagnostics(_diag_with_warning())
    clock.advance(29.0)   # still under the 30s threshold
    mm.update_from_diagnostics(_diag_with_warning())
    assert mm.current_mode == SystemModeEnum.NORMAL


def test_persistent_warning_fires_at_threshold():
    clock = _FakeClockFn()
    mm = ModeManager(_make_owner(), clock_fn=clock)
    mm.update_from_diagnostics(_diag_with_warning())
    clock.advance(30.5)   # just past the 30s threshold
    mm.update_from_diagnostics(_diag_with_warning())
    assert mm.current_mode == SystemModeEnum.DEGRADED


def test_persistent_warning_resets_when_warnings_clear():
    """Once warnings clear, the warning_start_time resets, so the
    next warning starts a fresh 30s window."""
    clock = _FakeClockFn()
    mm = ModeManager(_make_owner(), clock_fn=clock)
    mm.update_from_diagnostics(_diag_with_warning())
    clock.advance(15.0)
    mm.update_from_diagnostics(_diag_clear())  # NORMAL stays; reset start_time
    clock.advance(1.0)
    mm.update_from_diagnostics(_diag_with_warning())   # fresh start
    clock.advance(10.0)   # 10s elapsed since the new start, well under 30s
    mm.update_from_diagnostics(_diag_with_warning())
    assert mm.current_mode == SystemModeEnum.NORMAL


def test_error_immediately_escalates_to_safe_from_degraded():
    clock = _FakeClockFn()
    mm = ModeManager(_make_owner(), clock_fn=clock)
    mm.current_mode = SystemModeEnum.DEGRADED   # pre-conditioned for the test
    mm.update_from_diagnostics(_diag_with_error())
    assert mm.current_mode == SystemModeEnum.SAFE


def test_get_time_in_mode_uses_clock_fn():
    clock = _FakeClockFn(100.0)
    mm = ModeManager(_make_owner(), clock_fn=clock)
    clock.advance(7.5)
    assert mm.get_time_in_mode() == 7.5


def test_default_clock_fn_falls_back_to_get_clock_now():
    """If clock_fn is not supplied, ModeManager uses the owner's
    ``get_clock().now().nanoseconds / 1e9`` chain. Smoke test only."""
    owner = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(warn=lambda m: None),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=42_000_000_000),
        ),
    )
    mm = ModeManager(owner)   # no clock_fn supplied
    assert mm.get_time_in_mode() == 0.0
