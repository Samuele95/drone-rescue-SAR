"""StateTableWidget: per-drone state table.

Extracted from ``dashboard_app.py`` so the widget is unit-testable
in isolation. Consumes ``state.view`` only.
"""

from __future__ import annotations

from typing import List, Optional

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QHeaderView, QTableWidget, QTableWidgetItem, QWidget,
)

from drone_rescue_ui_common.clock import RealUiClock, UiClock

from ._table_helpers import set_cell
from drone_rescue_ui_common.constants import DEFAULT_DRONE_NAMES, TASK_LABEL
# Status fold shared with the fleet rail.
from drone_rescue_ui_common.view_model import drone_status
# Semantic status colours from the single palette source
# (these literals diverged from the canonical tokens).
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P

_SEVERITY_TOKEN = {
    'ok': _P.ok, 'warn': _P.warn, 'error': _P.error, 'muted': _P.text_muted,
}

# The display label comes from the UI bounded context's canonical
# table, not the coordination domain's TaskType enum (removes a
# view-widget to coordination-package build dependency).
_TASK_LABEL = TASK_LABEL
_DEFAULT_DRONES = list(DEFAULT_DRONE_NAMES)


class StateTableWidget(QTableWidget):
    """Per-drone state table: reads ``state.view.drones`` only."""

    COLUMNS = ['Drone', 'Health', 'Battery', 'Altitude', 'Task',
               'Position', 'Anomaly']

    def __init__(self, state, drones: Optional[List[str]] = None,
                 *, clock: Optional[UiClock] = None, bridge=None,
                 parent: Optional[QWidget] = None):
        if drones is None:
            drones = _DEFAULT_DRONES
        super().__init__(len(drones), len(self.COLUMNS), parent)
        self._state = state
        self._drones = drones
        # UiClock port; ``RealUiClock`` default keeps existing call
        # sites unchanged.
        self._clock: UiClock = clock if clock is not None else RealUiClock()
        self.setHorizontalHeaderLabels(self.COLUMNS)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setSelectionMode(QTableWidget.NoSelection)
        for i, name in enumerate(drones):
            self.setItem(i, 0, QTableWidgetItem(name))
            for j in range(1, len(self.COLUMNS)):
                self.setItem(i, j, QTableWidgetItem('—'))
        # With a ViewModelBridge the table re-renders on view_changed
        # (<=33 ms after arrival) and keeps only a 1 Hz heartbeat so
        # staleness ages keep advancing for silent drones. Without a
        # bridge (legacy/tests) the 5 Hz poll remains.
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        if bridge is not None:
            bridge.view_changed.connect(lambda _v: self._refresh())
            self._timer.start(1000)
        else:
            self._timer.start(200)   # 5 Hz

    def _refresh(self) -> None:
        now = self._clock.monotonic()
        peer_stale_s = 4.0
        health_stale_s = 3.0

        view = self._state.view
        for row, name in enumerate(self._drones):
            drone = view.drones.get(name)
            self._set(row, 0, name)

            if drone is None:
                self._set(row, 1, '—')
                for col in range(2, 7):
                    self._set(row, col, '—')
                continue

            peer_age = view.peer_age(name, now)
            health_age = view.health_age(name, now)
            peer_alive = (drone.peer_last_seen > 0
                          and peer_age < peer_stale_s)
            health_alive = (drone.health_last_seen > 0
                            and health_age < health_stale_s)

            # Shared status fold (ui_common drone_status); the fleet
            # rail renders the same labels.
            status_label, severity = drone_status(view, name, now)
            self._set(row, 1, status_label, _SEVERITY_TOKEN[severity])

            stale_color = _P.text_muted
            bat = int(drone.battery * 100)
            if not peer_alive:
                self._set(row, 2, f'{bat}% (stale)', stale_color)
            else:
                self._set(row, 2, f'{bat}%',
                          _P.error if bat < 20 else _P.ok)
            self._set(row, 3, f'{drone.pose_z:.1f} m',
                      stale_color if not peer_alive else '')
            label = _TASK_LABEL.get(drone.task_type, str(drone.task_type))
            if drone.wp_total > 0:
                label += f' ({drone.wp_index}/{drone.wp_total})'
            if drone.busy_with_victim:
                label += f' [v{drone.busy_with_victim}]'
            if not peer_alive:
                label = f'{label} (stale)'
            self._set(row, 4, label,
                      stale_color if not peer_alive else '')
            self._set(row, 5,
                      f'({drone.pose_x:.1f}, {drone.pose_y:.1f})',
                      stale_color if not peer_alive else '')

            if drone.health_reason:
                self._set(row, 6, drone.health_reason,
                          stale_color if not health_alive else _P.warn)
            elif drone.health_last_seen > 0:
                self._set(row, 6, 'none',
                          stale_color if not health_alive else _P.text_muted)
            else:
                self._set(row, 6, '—')

    def _set(self, row: int, col: int, text: str, color: str = '') -> None:
        set_cell(self, row, col, text, color)
