"""FleetRail: persistent per-drone status cards.

The rail keeps the whole fleet visible regardless of which stage view
(2D / 3D / cameras) is active: one card per drone with its identity
colour, derived status (shared ``drone_status`` fold from ui_common),
battery bar, and current task chip. Clicking a card selects the drone
in the inspector.
"""

from __future__ import annotations

from typing import Dict, List, Optional

from drone_rescue_ui_common.clock import RealUiClock, UiClock
from drone_rescue_ui_common.constants import (
    DEFAULT_DRONE_NAMES, DRONE_COLORS, TASK_LABEL,
)
from drone_rescue_ui_common.motion import set_value_animated
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P
from drone_rescue_ui_common.view_model import drone_status

from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QColor, QFont, QPainter
from python_qt_binding.QtWidgets import (
    QFrame, QLabel, QProgressBar, QSizePolicy, QVBoxLayout, QWidget,
)

_SEVERITY_COLOR = {
    'ok': _P.ok, 'warn': _P.warn, 'error': _P.error, 'muted': _P.text_muted,
}


class DroneCard(QFrame):
    """One fleet-rail card. Pure view: reads a DroneViewState."""

    clicked = Signal(str)
    double_clicked = Signal(str)   # opens the drone focus window

    def __init__(self, name: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._name = name
        self._identity = QColor(DRONE_COLORS.get(name, '#cccccc'))
        self.setObjectName('droneCard')
        self.setFrameShape(QFrame.StyledPanel)
        self.setCursor(Qt.PointingHandCursor)
        # Hover + selected feedback ride QSS state selectors (static
        # restyle; motion stays in code because Qt QSS is not CSS).
        self.setStyleSheet(
            f'QFrame#droneCard {{ background: {_P.bg_raised};'
            f' border: 1px solid {_P.stroke}; border-radius: 6px; }}'
            f'QFrame#droneCard:hover {{ border-color: {_P.accent}; }}'
            f'QFrame#droneCard[selected="true"] {{'
            f' border-color: {_P.accent};'
            f' background: {_P.accent_soft}; }}'
        )
        self.setProperty('selected', False)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 6, 8, 6)
        lay.setSpacing(3)

        self._title = QLabel(name)
        f = QFont()
        f.setBold(True)
        self._title.setFont(f)
        lay.addWidget(self._title)

        self._status = QLabel('—')
        self._status.setStyleSheet(f'color: {_P.text_muted};')
        lay.addWidget(self._status)

        self._battery = QProgressBar()
        self._battery.setRange(0, 100)
        self._battery.setValue(0)
        self._battery.setTextVisible(True)
        self._battery.setFormat('%p%')
        self._battery.setFixedHeight(14)
        lay.addWidget(self._battery)

        self._task = QLabel('')
        self._task.setStyleSheet(
            f'color: {_P.text_muted}; font-size: 9pt;'
        )
        lay.addWidget(self._task)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    # ------------------------------------------------------------ paint
    def paintEvent(self, event) -> None:  # noqa: N802 (Qt override)
        super().paintEvent(event)
        # Identity colour stripe down the left edge.
        p = QPainter(self)
        p.fillRect(0, 4, 4, self.height() - 8, self._identity)
        p.end()

    def mousePressEvent(self, event) -> None:  # noqa: N802 (Qt override)
        self.clicked.emit(self._name)
        super().mousePressEvent(event)

    def mouseDoubleClickEvent(self, event) -> None:  # noqa: N802
        self.double_clicked.emit(self._name)
        super().mouseDoubleClickEvent(event)

    def set_selected(self, selected: bool) -> None:
        """Persistent accent ring on the card driving the inspector."""
        if bool(self.property('selected')) == selected:
            return
        self.setProperty('selected', selected)
        # Dynamic-property QSS needs an explicit repolish to re-match.
        self.style().unpolish(self)
        self.style().polish(self)

    # ----------------------------------------------------------- update
    def update_from(self, view, now: float) -> None:
        label, severity = drone_status(view, self._name, now)
        self._status.setText(label)
        self._status.setStyleSheet(
            f'color: {_SEVERITY_COLOR[severity]}; font-weight: bold;'
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
            f' color: {_P.text_body}; font-size: 8pt; }}'
            f'QProgressBar::chunk {{ background: {bar_color}; }}'
        )
        task = TASK_LABEL.get(d.task_type, str(d.task_type))
        if d.wp_total > 0:
            task += f'  {d.wp_index}/{d.wp_total}'
        if d.busy_with_victim:
            task += f'  [v{d.busy_with_victim}]'
        self._task.setText(task)


class FleetRail(QWidget):
    """Vertical stack of DroneCards; emits ``drone_selected`` on click
    and ``drone_focused`` on double-click (focus window)."""

    drone_selected = Signal(str)
    drone_focused = Signal(str)

    def __init__(self, state, drones: Optional[List[str]] = None,
                 *, clock: Optional[UiClock] = None, bridge=None,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._state = state
        self._clock: UiClock = clock if clock is not None else RealUiClock()
        names = drones if drones is not None else list(DEFAULT_DRONE_NAMES)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)
        header = QLabel('FLEET')
        header.setStyleSheet(
            f'color: {_P.text_muted}; font-size: 9pt; font-weight: bold;'
            f' letter-spacing: 1px;'
        )
        lay.addWidget(header)
        self._cards: Dict[str, DroneCard] = {}
        for name in names:
            card = DroneCard(name)
            card.clicked.connect(self._on_card_clicked)
            card.double_clicked.connect(self.drone_focused.emit)
            lay.addWidget(card)
            self._cards[name] = card
        lay.addStretch(1)
        # Responsive pass: was setFixedWidth(190), which
        # made the splitter handle dead. The rail now flexes between a
        # readable floor and a cap so the stage keeps the surplus.
        self.setMinimumWidth(170)
        self.setMaximumWidth(280)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

        # Bridge mode: refresh on view change + 1 Hz staleness
        # heartbeat. Legacy mode: 2 Hz poll.
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        if bridge is not None:
            bridge.view_changed.connect(lambda _v: self._refresh())
            self._timer.start(1000)
        else:
            self._timer.start(500)

    def _on_card_clicked(self, name: str) -> None:
        for card_name, card in self._cards.items():
            card.set_selected(card_name == name)
        self.drone_selected.emit(name)

    def _refresh(self) -> None:
        view = self._state.view
        now = self._clock.monotonic()
        for card in self._cards.values():
            card.update_from(view, now)
