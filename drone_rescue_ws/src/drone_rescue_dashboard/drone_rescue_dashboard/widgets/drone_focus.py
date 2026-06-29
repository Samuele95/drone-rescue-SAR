"""DroneFocusWindow: dedicated per-drone window.

The inspector pane (~240 px) is the right size for a glance but too
small for actually *watching* one drone. Double-clicking a fleet-rail
card (or the inspector's Focus button) opens this top-level window:
the drone's down-camera feed at full size, the follow cam beside it,
and the complete telemetry readout: status, battery, task, waypoint
progress, altitude, position, anomaly, health.

Bridge-driven like every dashboard surface: telemetry re-renders on
``view_changed``; the two ImageTiles repaint per arriving frame. A
1 Hz heartbeat keeps staleness-derived status honest when a drone
goes silent. Read-only except for the optional Return-home button
(present when a command port was injected).
"""

from __future__ import annotations

from typing import Optional

from drone_rescue_ui_common.clock import RealUiClock, UiClock
from drone_rescue_ui_common.constants import DRONE_COLORS, TASK_LABEL
from drone_rescue_ui_common.motion import set_value_animated
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P
from drone_rescue_ui_common.style import MONO_FAMILY
from drone_rescue_ui_common.view_model import drone_status

from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QFormLayout, QHBoxLayout, QLabel, QProgressBar, QPushButton,
    QSplitter, QVBoxLayout, QWidget,
)

from .image_tile import ImageTile

_SEVERITY_COLOR = {
    'ok': _P.ok, 'warn': _P.warn, 'error': _P.error, 'muted': _P.text_muted,
}

_TELEMETRY_KEYS = (
    'task', 'waypoint', 'altitude', 'position', 'anomaly', 'health',
)


class DroneFocusWindow(QWidget):
    """Top-level focus view for one drone: big camera + telemetry."""

    closed = Signal(str)   # drone name; lets the owner drop its handle

    def __init__(self, name: str, state, images, *,
                 clock: Optional[UiClock] = None, bridge=None,
                 cmd_port=None, parent: Optional[QWidget] = None):
        # Qt.Window on a parented widget gives a real top-level window
        # that still dies with the dashboard instead of outliving it.
        super().__init__(parent, Qt.Window)
        self._name = name
        self._state = state
        self._clock: UiClock = clock if clock is not None else RealUiClock()
        self._cmd = cmd_port
        identity = DRONE_COLORS.get(name, '#cccccc')

        self.setWindowTitle(f'{name} — focus')
        self.resize(1100, 720)
        self.setStyleSheet(f'background: {_P.bg_dark};')

        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 8, 10, 8)
        outer.setSpacing(8)

        # ---------------------------------------------------- header
        header = QHBoxLayout()
        header.setSpacing(12)
        title = QLabel(name)
        f = QFont()
        f.setPointSize(16)
        f.setBold(True)
        title.setFont(f)
        title.setStyleSheet(f'color: {identity};')
        header.addWidget(title)

        self._status = QLabel('—')
        sf = QFont()
        sf.setPointSize(12)
        sf.setBold(True)
        self._status.setFont(sf)
        header.addWidget(self._status)
        header.addStretch(1)

        self._battery = QProgressBar()
        self._battery.setRange(0, 100)
        self._battery.setFormat('battery %p%')
        self._battery.setMinimumWidth(180)
        self._battery.setMaximumWidth(280)
        self._battery.setFixedHeight(18)
        header.addWidget(self._battery)

        if self._cmd is not None:
            rth = QPushButton('⌂ Return home')
            rth.setToolTip('Issue this drone an RTH task '
                           '(fly home + land)')
            rth.clicked.connect(
                lambda: self._cmd.request_return_home(self._name),
            )
            header.addWidget(rth)
        outer.addLayout(header)

        # ------------------------------------------- cameras + telemetry
        split = QSplitter(Qt.Horizontal)
        split.setHandleWidth(6)

        # The big screen: down camera dominates the window.
        self._down_cam = ImageTile(
            f'/{name}/camera', images,
            title=f'{name} ↓ down camera', bridge=bridge,
        )
        self._down_cam.setMinimumSize(480, 360)
        split.addWidget(self._down_cam)

        side = QWidget()
        side_lay = QVBoxLayout(side)
        side_lay.setContentsMargins(0, 0, 0, 0)
        side_lay.setSpacing(8)
        self._follow_cam = ImageTile(
            f'/{name}/follow_cam', images,
            title=f'{name} → follow camera', bridge=bridge,
        )
        side_lay.addWidget(self._follow_cam, stretch=1)

        form_box = QWidget()
        form_box.setStyleSheet(
            f'background: {_P.bg_raised};'
            f' border: 1px solid {_P.stroke}; border-radius: 6px;'
        )
        form = QFormLayout(form_box)
        form.setContentsMargins(12, 10, 12, 10)
        form.setLabelAlignment(Qt.AlignRight)
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(6)
        self._rows = {}
        for key in _TELEMETRY_KEYS:
            label = QLabel(key)
            label.setStyleSheet(
                f'color: {_P.text_muted}; font-size: 9pt;'
                f' background: transparent; border: none;'
            )
            value = QLabel('—')
            value.setStyleSheet(
                f'font-family: {MONO_FAMILY};'
                f' background: transparent; border: none;'
            )
            value.setWordWrap(True)
            form.addRow(label, value)
            self._rows[key] = value
        side_lay.addWidget(form_box)
        split.addWidget(side)

        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 1)
        split.setSizes([760, 320])
        outer.addWidget(split, stretch=1)

        # Bridge-driven refresh + 1 Hz staleness heartbeat (the same
        # pattern the inspector uses).
        if bridge is not None:
            bridge.view_changed.connect(lambda _v: self._refresh())
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(1000)

        self._refresh()

    # ---------------------------------------------------------- render
    def _refresh(self) -> None:
        view = self._state.view
        label, severity = drone_status(
            view, self._name, self._clock.monotonic(),
        )
        self._status.setText(label)
        self._status.setStyleSheet(
            f'color: {_SEVERITY_COLOR[severity]};'
        )
        d = view.drones.get(self._name)
        if d is None:
            return

        pct = int(d.battery * 100)
        set_value_animated(self._battery, pct)
        bar_color = _P.error if pct < 20 else (
            _P.warn if pct < 40 else _P.ok
        )
        self._battery.setStyleSheet(
            f'QProgressBar {{ background: {_P.bg_deep};'
            f' border: 1px solid {_P.stroke}; border-radius: 2px;'
            f' color: {_P.text_body};'
            f' font-family: {MONO_FAMILY}; font-size: 9pt; }}'
            f'QProgressBar::chunk {{ background: {bar_color}; }}'
        )

        task = TASK_LABEL.get(d.task_type, str(d.task_type))
        if d.busy_with_victim:
            task += f'  [victim #{d.busy_with_victim}]'
        self._rows['task'].setText(task)
        self._rows['waypoint'].setText(
            f'{d.wp_index} / {d.wp_total}' if d.wp_total > 0 else '—'
        )
        self._rows['altitude'].setText(f'{d.pose_z:.1f} m')
        self._rows['position'].setText(
            f'({d.pose_x:.1f}, {d.pose_y:.1f})'
        )
        self._rows['anomaly'].setText(f'{d.anomaly_score:.2f}')
        health = d.health_reason or 'nominal'
        if d.unrecoverable:
            health += '  (UNRECOVERABLE)'
        self._rows['health'].setText(health)

    def closeEvent(self, event) -> None:  # noqa: N802 (Qt override)
        self.closed.emit(self._name)
        super().closeEvent(event)
