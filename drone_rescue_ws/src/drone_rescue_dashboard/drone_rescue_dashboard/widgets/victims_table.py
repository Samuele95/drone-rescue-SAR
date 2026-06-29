"""VictimsTableWidget: per-victim status table.

Extracted from ``dashboard_app.py`` so the widget is unit-testable
in isolation. Consumes ``state.view.victims`` only.
"""

from __future__ import annotations

from typing import Dict, Optional

from python_qt_binding.QtCore import QTimer, Signal
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P
from python_qt_binding.QtWidgets import (
    QHeaderView, QTableWidget, QWidget,
)

from drone_rescue_ui_common.clock import RealUiClock, UiClock

from ._table_helpers import set_cell


class VictimsTableWidget(QTableWidget):
    """Per-victim status table: one row per detected candidate."""

    # Row click selects the victim in the inspector.
    victim_selected = Signal(int)

    COLUMNS = ['ID', 'Position', 'Confidence', 'Reporters', 'Status', 'Age']

    def __init__(self, state, *, clock: Optional[UiClock] = None,
                 bridge=None, parent: Optional[QWidget] = None):
        super().__init__(0, len(self.COLUMNS), parent)
        self._state = state
        # UiClock port; ``RealUiClock`` default.
        self._clock: UiClock = clock if clock is not None else RealUiClock()
        self.setHorizontalHeaderLabels(self.COLUMNS)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setSelectionMode(QTableWidget.NoSelection)
        # Bridge mode: re-render on view change + 1 Hz heartbeat for
        # the Age column; legacy mode: 2 Hz poll.
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        if bridge is not None:
            bridge.view_changed.connect(lambda _v: self._refresh())
            self._timer.start(1000)
        else:
            self._timer.start(500)
        self._first_seen: Dict[int, float] = {}

    def _refresh(self) -> None:
        now = self._clock.monotonic()
        victims = sorted(self._state.view.victims.items())
        self.setRowCount(len(victims))
        for row, (vid, vv) in enumerate(victims):
            if vid not in self._first_seen:
                self._first_seen[vid] = now
            age = now - self._first_seen[vid]
            self._set(row, 0, str(vid))
            self._set(row, 1, f'({vv.position[0]:.1f}, {vv.position[1]:.1f})')
            self._set(row, 2, f'{vv.confidence:.2f}',
                      _P.ok if vv.confidence >= 0.85 else _P.warn)
            reporters = list(vv.reporting_drones)
            self._set(row, 3, ', '.join(reporters) if reporters else '—')
            if vv.confirmed:
                self._set(row, 4, 'CONFIRMED', _P.ok)
            else:
                self._set(row, 4, 'candidate', _P.warn)
            age_color = _P.text_muted
            if age > 60:
                age_color = _P.error
            elif age > 30:
                age_color = _P.warn
            self._set(row, 5, f'{age:.0f}s', age_color)

    def _on_cell_clicked(self, row: int, _col: int) -> None:
        item = self.item(row, 0)
        if item is None:
            return
        try:
            self.victim_selected.emit(int(item.text()))
        except ValueError:
            pass

    def _set(self, row: int, col: int, text: str, color: str = '') -> None:
        set_cell(self, row, col, text, color)
