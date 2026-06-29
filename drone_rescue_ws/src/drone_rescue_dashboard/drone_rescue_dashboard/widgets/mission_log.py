"""MissionLogWidget: color-coded scrolling event log.

Extracted from ``dashboard_app.py`` so the widget is unit-testable
against a synthetic LogBuffer.
"""

from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QTextBrowser, QWidget

# Single source of truth for severity colour/label
# (drone_rescue_ui_common.constants), and no runtime dependency on
# drone_rescue_msgs from a pure view widget (the type is for checkers only).
from drone_rescue_ui_common.constants import (
    SEVERITY_COLOR as _SEVERITY_COLOR,
    SEVERITY_LABEL as _SEVERITY_LABEL,
)
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P

if TYPE_CHECKING:
    from drone_rescue_msgs.msg import MissionEvent


class MissionLogWidget(QTextBrowser):
    """Color-coded scrolling event log. `filter_drone` filters events:
    None  → all events
    str   → events whose drone_name matches AND mission-wide events
            (drone_name=='') so the operator still sees SCANNING_STARTED,
            MISSION_COMPLETE etc. on the per-drone tabs.
    """

    def __init__(self, log, filter_drone: Optional[str] = None,
                 *, bridge=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._log = log
        self._filter_drone = filter_drone
        # Operator-set filters (toolbar in the log dock).
        # ``severity`` is an exact severity int or None=all;
        # ``search`` is a case-insensitive substring over the
        # formatted body.
        self._filter_severity: Optional[int] = None
        self._filter_search: str = ''
        # Track how many events we've already rendered using LogBuffer's
        # monotonic counter (deque.len plateaus once maxlen is hit).
        self._rendered = 0
        self.setOpenExternalLinks(False)
        self.setLineWrapMode(QTextBrowser.WidgetWidth)
        font = QFont('Monospace')
        font.setStyleHint(QFont.TypeWriter)
        self.setFont(font)
        # Bridge mode: append on events_changed only.
        if bridge is not None:
            bridge.events_changed.connect(self._refresh)
        else:
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._refresh)
            self._timer.start(300)   # 3 Hz

    def set_filters(self, *, severity: Optional[int] = None,
                    drone: Optional[str] = None, search: str = '') -> None:
        """Apply operator filters and rebuild the visible log
        from the buffered events (the deque keeps the last 400)."""
        self._filter_severity = severity
        self._filter_drone = drone
        self._filter_search = (search or '').strip().lower()
        self.clear()
        for ts, evt in list(self._log.events):
            if self._matches(evt):
                self.append(self._format(ts, evt))
        self._rendered = self._log.total_appended
        sb = self.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _matches(self, evt) -> bool:
        if self._filter_drone is not None:
            if evt.drone_name and evt.drone_name != self._filter_drone:
                return False
        if (self._filter_severity is not None
                and evt.severity != self._filter_severity):
            return False
        if self._filter_search:
            hay = (f'{evt.event_type} {evt.drone_name} '
                   f'{getattr(evt, "detail", "")}').lower()
            if self._filter_search not in hay:
                return False
        return True

    def _refresh(self) -> None:
        total = self._log.total_appended
        if total == self._rendered:
            return
        events = list(self._log.events)
        oldest_idx = total - len(events)
        for i, (ts, evt) in enumerate(events):
            abs_idx = oldest_idx + i
            if abs_idx < self._rendered:
                continue
            if not self._matches(evt):
                continue
            self.append(self._format(ts, evt))
        self._rendered = total
        sb = self.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _format(self, ts: str, evt: 'MissionEvent') -> str:
        # ts is the receive-time captured at LogBuffer.append, not the
        # render-tick time.
        color = _SEVERITY_COLOR.get(evt.severity, _P.text_muted)
        sev = _SEVERITY_LABEL.get(evt.severity, '??')
        body = evt.event_type
        if evt.drone_name:
            body += f' / {evt.drone_name}'
        if evt.victim_id:
            body += f' (victim #{evt.victim_id})'
        if evt.detail:
            body += f' — {evt.detail}'
        if evt.position.x or evt.position.y:
            body += f' @ ({evt.position.x:.1f}, {evt.position.y:.1f})'
        return (f'<span style="color:{color}">[{ts}] {sev:5s}</span> '
                f'<span style="color:{_P.text_body}">{body}</span>')
