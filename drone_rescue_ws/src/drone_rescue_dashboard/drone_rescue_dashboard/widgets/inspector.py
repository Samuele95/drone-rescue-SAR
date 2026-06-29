"""InspectorPanel: contextual detail pane.

Click a drone (fleet rail or mission scene) or a victim and this pane
shows its full state: status, battery, task + waypoint progress,
altitude, anomaly, health reason, and the drone's live camera thumb.
The ``actions`` layout hosts per-selection command buttons
(Return home / Investigate).
"""

from __future__ import annotations

from typing import Optional

from drone_rescue_ui_common.clock import RealUiClock, UiClock
from drone_rescue_ui_common.constants import DRONE_COLORS, TASK_LABEL
from drone_rescue_ui_common.motion import fade_in, set_value_animated
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P
from drone_rescue_ui_common.style import MONO_FAMILY
from drone_rescue_ui_common.view_model import drone_status

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QFont, QPixmap
from python_qt_binding.QtWidgets import (
    QFormLayout, QLabel, QProgressBar, QVBoxLayout, QWidget,
)

_SEVERITY_COLOR = {
    'ok': _P.ok, 'warn': _P.warn, 'error': _P.error, 'muted': _P.text_muted,
}


class InspectorPanel(QWidget):
    """Detail pane for the selected drone or victim."""

    def __init__(self, state, images, *, clock: Optional[UiClock] = None,
                 bridge=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._state = state
        self._images = images
        self._clock: UiClock = clock if clock is not None else RealUiClock()
        self._selected_drone: Optional[str] = None
        self._selected_victim: Optional[int] = None

        # Responsive pass: was setFixedWidth(240); the pane now flexes
        # (wordwrapped health text + camera thumb benefit from extra
        # width on big displays).
        self.setMinimumWidth(220)
        self.setMaximumWidth(380)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(6)

        header = QLabel('INSPECTOR')
        header.setStyleSheet(
            f'color: {_P.text_muted}; font-size: 9pt; font-weight: bold;'
            f' letter-spacing: 1px;'
        )
        lay.addWidget(header)

        self._title = QLabel('click a drone or victim')
        f = QFont()
        f.setPointSize(12)
        f.setBold(True)
        self._title.setFont(f)
        self._title.setWordWrap(True)   # overlap fix: wrap, don't clip
        lay.addWidget(self._title)

        self._status = QLabel('')
        lay.addWidget(self._status)

        self._battery = QProgressBar()
        self._battery.setRange(0, 100)
        self._battery.setFixedHeight(14)
        self._battery.setVisible(False)
        lay.addWidget(self._battery)

        self._form = QFormLayout()
        self._form.setLabelAlignment(Qt.AlignRight)
        self._form.setHorizontalSpacing(10)
        self._form.setVerticalSpacing(3)
        self._rows = {}
        for key in ('task', 'waypoint', 'altitude', 'position',
                    'anomaly', 'health', 'confidence', 'reporters'):
            label = QLabel(key)
            label.setStyleSheet(
                f'color: {_P.text_muted}; font-size: 9pt;'
            )
            value = QLabel('')
            value.setStyleSheet(f'font-family: {MONO_FAMILY};')
            value.setWordWrap(True)
            self._form.addRow(label, value)
            self._rows[key] = (label, value)
        lay.addLayout(self._form)

        self._camera = QLabel()
        self._camera.setMinimumHeight(130)
        self._camera.setAlignment(Qt.AlignCenter)
        self._camera.setStyleSheet(
            f'background: {_P.bg_deep}; color: {_P.text_muted};'
            f' border: 1px solid {_P.stroke};'
        )
        self._camera.setVisible(False)
        lay.addWidget(self._camera)

        lay.addStretch(1)
        # Return-home / Investigate buttons mount here.
        self.actions = QVBoxLayout()
        self.actions.setSpacing(6)
        lay.addLayout(self.actions)

        self._set_rows_visible(())

        if bridge is not None:
            bridge.view_changed.connect(lambda _v: self._refresh())
            bridge.frame_arrived.connect(self._on_frame)
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._refresh)
            self._timer.start(1000)   # staleness heartbeat
        else:
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._refresh)
            self._timer.start(500)

    # --------------------------------------------------------- selection
    def show_drone(self, name: str) -> None:
        changed = (self._selected_drone != name
                   or self._selected_victim is not None)
        self._selected_drone = name
        self._selected_victim = None
        self._refresh()
        if changed:
            fade_in(self)   # selection context switch, not a data tick

    def show_victim(self, cid: int) -> None:
        changed = (self._selected_victim != cid
                   or self._selected_drone is not None)
        self._selected_victim = cid
        self._selected_drone = None
        self._refresh()
        if changed:
            fade_in(self)

    @property
    def selected_drone(self) -> Optional[str]:
        return self._selected_drone

    # ------------------------------------------------------------ render
    def _set_rows_visible(self, keys) -> None:
        for key, (label, value) in self._rows.items():
            visible = key in keys
            label.setVisible(visible)
            value.setVisible(visible)

    def _on_frame(self, topic: str) -> None:
        if (self._selected_drone is not None
                and topic == f'/{self._selected_drone}/camera'):
            self._update_camera()

    def _update_camera(self) -> None:
        if self._selected_drone is None:
            return
        img = self._images.images.get(f'/{self._selected_drone}/camera')
        if img is None:
            self._camera.setText('no camera frame yet')
            return
        scaled = img.scaled(
            self._camera.width(), self._camera.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )
        self._camera.setPixmap(QPixmap.fromImage(scaled))

    def _refresh(self) -> None:
        view = self._state.view
        if self._selected_drone is not None:
            name = self._selected_drone
            identity = DRONE_COLORS.get(name, '#cccccc')
            self._title.setText(name)
            self._title.setStyleSheet(f'color: {identity};')
            label, severity = drone_status(
                view, name, self._clock.monotonic(),
            )
            self._status.setText(label)
            self._status.setStyleSheet(
                f'color: {_SEVERITY_COLOR[severity]}; font-weight: bold;'
            )
            d = view.drones.get(name)
            self._set_rows_visible(
                ('task', 'waypoint', 'altitude', 'position', 'anomaly',
                 'health'),
            )
            self._battery.setVisible(True)
            self._camera.setVisible(True)
            if d is not None:
                pct = int(d.battery * 100)
                set_value_animated(self._battery, pct)
                bar_color = _P.error if pct < 20 else (
                    _P.warn if pct < 40 else _P.ok
                )
                self._battery.setStyleSheet(
                    f'QProgressBar {{ background: {_P.bg_deep};'
                    f' border: 1px solid {_P.stroke};'
                    f' border-radius: 2px; color: {_P.text_body};'
                    f' font-size: 8pt; }}'
                    f'QProgressBar::chunk {{ background: {bar_color}; }}'
                )
                self._rows['task'][1].setText(
                    TASK_LABEL.get(d.task_type, str(d.task_type))
                    + (f' [v{d.busy_with_victim}]'
                       if d.busy_with_victim else '')
                )
                self._rows['waypoint'][1].setText(
                    f'{d.wp_index}/{d.wp_total}' if d.wp_total else '—'
                )
                self._rows['altitude'][1].setText(f'{d.pose_z:.1f} m')
                self._rows['position'][1].setText(
                    f'({d.pose_x:.1f}, {d.pose_y:.1f})'
                )
                self._rows['anomaly'][1].setText(f'{d.anomaly_score:.2f}')
                self._rows['health'][1].setText(d.health_reason or 'none')
            self._update_camera()
            return

        if self._selected_victim is not None:
            cid = self._selected_victim
            vv = view.victims.get(cid)
            self._title.setText(f'victim v{cid}')
            self._battery.setVisible(False)
            self._camera.setVisible(False)
            self._set_rows_visible(('position', 'confidence', 'reporters'))
            if vv is None:
                self._status.setText('unknown')
                self._status.setStyleSheet(f'color: {_P.text_muted};')
                return
            if vv.confirmed:
                self._title.setStyleSheet(f'color: {_P.ok};')
                self._status.setText('CONFIRMED')
                self._status.setStyleSheet(
                    f'color: {_P.ok}; font-weight: bold;'
                )
            else:
                self._title.setStyleSheet(f'color: {_P.warn};')
                self._status.setText('candidate')
                self._status.setStyleSheet(
                    f'color: {_P.warn}; font-weight: bold;'
                )
            self._rows['position'][1].setText(
                f'({vv.position[0]:.1f}, {vv.position[1]:.1f})'
            )
            self._rows['confidence'][1].setText(f'{vv.confidence:.2f}')
            reporters = ', '.join(vv.reporting_drones) or '—'
            self._rows['reporters'][1].setText(reporters)
