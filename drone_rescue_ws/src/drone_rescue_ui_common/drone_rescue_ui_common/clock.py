"""UiClock Protocol: driven port for wall-clock reads in UI widgets.

Symmetric counterpart of the coordination side's
``drone_rescue_coordination.lib.ports.clock.Clock``.
The dashboard's freshness gates (``StateTableWidget._refresh``,
``VictimsTableWidget._refresh``, ``DashboardSubscriberNode._on_peer``
/ ``_on_health``) currently call ``time.monotonic()`` directly; that
forces widget unit tests to ``monkeypatch.setattr(time, 'monotonic',
...)`` rather than inject a fake. Widgets depend on the Protocol,
not on ``time``.
"""

from __future__ import annotations

import time
from typing import Protocol


class UiClock(Protocol):
    """Wall-clock source for UI widgets. Single-method Protocol;
    matches the coordination-side ``Clock.now_sec`` shape but uses
    ``monotonic()`` as the canonical method name to match the
    existing ``time.monotonic()`` call sites in the dashboard."""

    def monotonic(self) -> float:
        """Return a monotonic wall-clock reading in seconds."""
        ...


class RealUiClock(UiClock):
    """Production adapter: delegates to ``time.monotonic``."""

    __slots__ = ()

    def monotonic(self) -> float:
        return time.monotonic()


class FakeUiClock(UiClock):
    """Test fake: returns a value set via ``set(t)``. Mirrors the
    coordination side's ``FakeClock`` so widget unit tests can
    advance the clock deterministically without touching the
    ``time`` module."""

    __slots__ = ('_t',)

    def __init__(self, t: float = 0.0):
        self._t = float(t)

    def monotonic(self) -> float:
        return self._t

    def set(self, t: float) -> None:
        self._t = float(t)

    def advance(self, dt: float) -> None:
        self._t += float(dt)
