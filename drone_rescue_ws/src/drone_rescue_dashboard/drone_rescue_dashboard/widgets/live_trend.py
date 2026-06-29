"""LiveTrendWidget: in-mission coverage & confirmed-victim trend.

The CoverageBanner shows the *current* coverage % and victim
counts; this widget shows their *history*, so the operator can see
whether coverage is accelerating or stalling and when confirmations
clustered in time.

Painted with QPainter (the dashboard already uses QPainter for the
Mission Scene) so the dashboard gains no matplotlib dependency.
``TrendBuffer`` (the sampling/reset logic) is pure and unit-tested.
"""

from __future__ import annotations

from collections import deque
from typing import List, Optional, Protocol, TYPE_CHECKING

if TYPE_CHECKING:                          # pragma: no cover
    from drone_rescue_ui_common.view_model import MissionViewModel

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QColor, QFont, QPainter, QPen
from python_qt_binding.QtWidgets import QWidget

# The six hex literals below moved onto palette tokens (the last
# palette-bypassing colours in the dashboard).
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P


class TrendBuffer:
    """Bounded time-history of mission coverage % and cumulative
    confirmed-victim count, sampled from the dashboard StateCache.

    Pure (no Qt) so the sampling and new-mission-reset logic can be
    unit-tested without a running dashboard.
    """

    def __init__(self, maxlen: int = 600):
        self._maxlen = maxlen
        # Bounded ring buffers: deque(maxlen) auto-evicts the oldest
        # sample in O(1). list.pop(0) was O(n), the same anti-pattern
        # already corrected in pheromone_server / active_tab /
        # StateCache.trails.
        self.times: deque = deque(maxlen=maxlen)       # elapsed sim seconds
        self.coverage: deque = deque(maxlen=maxlen)    # coverage %
        self.confirmed: deque = deque(maxlen=maxlen)   # cumulative confirmed

    def record(self, elapsed_s: float, coverage_pct: float,
               confirmed: int) -> None:
        """Append a sample. Sim time running backwards means a new
        mission started, so the stale trace is dropped. Samples whose
        elapsed time has not advanced are ignored, so a paused sim or a
        burst of identical messages doesn't pile points on one x."""
        elapsed_s = float(elapsed_s)
        if self.times and elapsed_s < self.times[-1]:
            self.clear()
        if self.times and elapsed_s <= self.times[-1]:
            return
        # deque(maxlen) evicts the oldest sample automatically.
        self.times.append(elapsed_s)
        self.coverage.append(float(coverage_pct))
        self.confirmed.append(int(confirmed))

    def clear(self) -> None:
        self.times.clear()
        self.coverage.clear()
        self.confirmed.clear()

    def __len__(self) -> int:
        return len(self.times)


class _HasMissionView(Protocol):
    """Minimal interface LiveTrendWidget consumes from the dashboard
    StateCache: a ``.view`` carrying the MissionViewModel projection
    (mirrors the OperatorView approach)."""
    view: 'MissionViewModel'


class LiveTrendWidget(QWidget):
    """Two stacked sparklines (coverage % and cumulative confirmed
    victims vs. elapsed time) sampled from ``state.view`` at 2 Hz."""

    def __init__(self, state: '_HasMissionView',
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._state = state
        self._buf = TrendBuffer()
        self.setMinimumHeight(150)
        self.setStyleSheet(f'background-color:{_P.bg_dark};')
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._sample)
        self._timer.start(500)   # 2 Hz, matches CoverageBanner

    # ------------------------------------------------------------ sampling
    def _sample(self) -> None:
        view = self._state.view
        cov = view.coverage
        # Single-source fold on the view model.
        n_confirmed = view.confirmed_victim_count
        self._buf.record(
            cov.elapsed_time_seconds, cov.percentage, n_confirmed,
        )
        self.update()

    # ------------------------------------------------------------ painting
    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        pad = 6
        panel_h = (h - 3 * pad) / 2.0
        self._paint_panel(
            p, pad, pad, w - 2 * pad, panel_h,
            'Coverage %', self._buf.times, self._buf.coverage,
            v_max=100.0, color=QColor(_P.ok), unit='%',
        )
        # Pass the deque straight through; _paint_panel does the single
        # snapshot+cast copy.
        confirmed = self._buf.confirmed
        self._paint_panel(
            p, pad, 2 * pad + panel_h, w - 2 * pad, panel_h,
            'Confirmed victims', self._buf.times, confirmed,
            v_max=float(max(confirmed)) if confirmed else 1.0,
            color=QColor(_P.info), unit='', as_int=True,
        )
        p.end()

    def _paint_panel(self, p: QPainter, x: float, y: float,
                     w: float, h: float, label: str,
                     times: List[float], values: List[float],
                     v_max: float, color: QColor, unit: str,
                     as_int: bool = False) -> None:
        # Frame.
        p.setPen(QPen(QColor(_P.stroke), 1))
        p.drawRect(int(x), int(y), int(w), int(h))

        font = QFont(); font.setPointSize(8)
        p.setFont(font)
        p.setPen(QPen(QColor(_P.text_muted), 1))
        p.drawText(int(x + 4), int(y + 12), label)

        if len(times) < 2:
            p.setPen(QPen(QColor(_P.text_muted), 1))
            p.drawText(int(x + 4), int(y + h / 2),
                       'waiting for /coverage/metrics …')
            return

        # Snapshot to lists: the buffers are deques (O(n) middle
        # indexing); the polyline loop below indexes by position. The
        # float cast lives here beside the snapshot so callers don't
        # pre-materialise a second copy.
        times = list(times)
        values = [float(v) for v in values]

        v_max = v_max if v_max > 0 else 1.0
        t0, t1 = times[0], times[-1]
        t_span = (t1 - t0) or 1.0
        plot_x, plot_w = x + 4, w - 8
        plot_y, plot_h = y + 16, h - 22

        def sx(t: float) -> float:
            return plot_x + (t - t0) / t_span * plot_w

        def sy(v: float) -> float:
            return plot_y + plot_h - (min(v, v_max) / v_max) * plot_h

        # Trend polyline.
        p.setPen(QPen(color, 2))
        for i in range(1, len(times)):
            p.drawLine(int(sx(times[i - 1])), int(sy(values[i - 1])),
                       int(sx(times[i])), int(sy(values[i])))

        # Latest value, top-right.
        latest = values[-1]
        text = (f'{int(latest)}{unit}' if as_int
                else f'{latest:.1f}{unit}')
        p.setPen(QPen(color, 1))
        p.drawText(int(x + w - 52), int(y + 12), text)
