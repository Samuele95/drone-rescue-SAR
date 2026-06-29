"""MissionBar: top-of-window mission status strip.

One glance answers "what phase is the mission in, how far along, and
what has it found": the MissionState phase strip, the mission clock,
coverage progress, and victim tallies. The right-hand ``actions``
layout hosts the operator command buttons.
"""

from __future__ import annotations

from typing import Dict, Optional

from drone_rescue_ui_common.motion import pulse_color, set_value_animated
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P
from drone_rescue_ui_common.style import MONO_FAMILY

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QHBoxLayout, QLabel, QProgressBar, QVBoxLayout, QWidget,
)

#: MissionState.status → operator label, in strip order.
PHASES = [
    (0, 'INIT'), (1, 'ARMING'), (2, 'DEPLOYING'), (3, 'SCANNING'),
    (4, 'INVESTIGATING'), (5, 'COMPLETE'),
]
ABORTED_STATUS = 6


class MissionBar(QWidget):
    """Phase strip + clock + coverage + victims, palette-styled."""

    def __init__(self, state, *, bridge=None,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._state = state
        self.setStyleSheet(
            f'background: {_P.bg_panel};'
            f' border-bottom: 1px solid {_P.stroke};'
        )
        # Overlap fix: the original single QHBoxLayout demanded
        # ~1100 px minimum, forcing a >1340 px window; on a
        # ~1280-logical-px display the WM clamps the window below its
        # minimum and Qt paints widgets over each other. Two compact
        # rows halve the width floor: phases+clock on top, metrics+
        # actions below.
        rows = QVBoxLayout(self)
        rows.setContentsMargins(12, 4, 12, 6)
        rows.setSpacing(2)
        top = QHBoxLayout()
        top.setSpacing(10)
        rows.addLayout(top)
        lay = QHBoxLayout()
        lay.setSpacing(14)
        rows.addLayout(lay)

        # Phase strip (top row).
        self._phase_labels: Dict[int, QLabel] = {}
        for status, name in PHASES:
            lbl = QLabel(name)
            lbl.setStyleSheet(
                f'color: {_P.text_muted}; font-size: 9pt;'
                f' letter-spacing: 1px;'
            )
            top.addWidget(lbl)
            self._phase_labels[status] = lbl
            if status != PHASES[-1][0]:
                sep = QLabel('▸')
                sep.setStyleSheet(f'color: {_P.stroke};')
                top.addWidget(sep)

        top.addStretch(1)

        # Mission clock: the one big number (top row, right edge).
        self._clock = QLabel('--:--')
        clock_font = QFont('DejaVu Sans Mono')
        clock_font.setPointSize(16)
        clock_font.setBold(True)
        self._clock.setFont(clock_font)
        self._clock.setStyleSheet(f'color: {_P.text_body};')
        top.addWidget(self._clock)

        # Coverage gauge.
        self._coverage = QProgressBar()
        self._coverage.setRange(0, 1000)   # 0.1 % resolution
        self._coverage.setFormat('coverage %.1f%%' % 0.0)
        # Responsive pass: gauge flexes 160..320 px instead of a hard
        # 220 so the bar degrades on narrow windows.
        self._coverage.setMinimumWidth(160)
        self._coverage.setMaximumWidth(320)
        self._coverage.setFixedHeight(18)
        self._coverage.setStyleSheet(
            f'QProgressBar {{ background: {_P.bg_deep};'
            f' border: 1px solid {_P.stroke}; border-radius: 2px;'
            f' color: {_P.text_body};'
            f' font-family: {MONO_FAMILY}; font-size: 9pt; }}'
            f'QProgressBar::chunk {{ background: {_P.accent}; }}'
        )
        lay.addWidget(self._coverage)

        # Victim tally. The stylesheet format doubles as the
        # pulse_color() template (confirmed-count bumps flash teal).
        self._victims_fmt = (
            'color: {color};' + f' font-family: {MONO_FAMILY};'
        )
        self._victims = QLabel('victims ◉ 0 / ○ 0')
        self._victims.setStyleSheet(
            self._victims_fmt.format(color=_P.text_body)
        )
        lay.addWidget(self._victims)
        self._last_confirmed = 0

        lay.addStretch(1)
        # Start / Recall buttons mount here.
        self.actions = QHBoxLayout()
        self.actions.setSpacing(8)
        lay.addLayout(self.actions)

        if bridge is not None:
            bridge.view_changed.connect(lambda _v: self._refresh())
        else:
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._refresh)
            self._timer.start(500)

    # ------------------------------------------------------------ render
    def _refresh(self) -> None:
        view = self._state.view
        mission = view.mission
        cov = view.coverage

        # Phase strip highlight.
        active = mission.status if mission.received else None
        for status, lbl in self._phase_labels.items():
            if mission.received and mission.status == ABORTED_STATUS:
                # ABORTED: dim everything; clock turns red below.
                lbl.setStyleSheet(
                    f'color: {_P.text_muted}; font-size: 9pt;'
                    f' letter-spacing: 1px;'
                )
            elif status == active:
                lbl.setStyleSheet(
                    f'color: {_P.accent}; font-size: 9pt; font-weight:'
                    f' bold; letter-spacing: 1px;'
                )
            else:
                lbl.setStyleSheet(
                    f'color: {_P.text_muted}; font-size: 9pt;'
                    f' letter-spacing: 1px;'
                )

        # Clock from coverage elapsed (sim time).
        secs = int(cov.elapsed_time_seconds)
        self._clock.setText(f'{secs // 60:02d}:{secs % 60:02d}')
        clock_color = (
            _P.error
            if mission.received and mission.status == ABORTED_STATUS
            else _P.text_body
        )
        self._clock.setStyleSheet(f'color: {clock_color};')

        set_value_animated(self._coverage, int(cov.percentage * 10))
        self._coverage.setFormat(f'coverage {cov.percentage:.1f}%')

        n_conf = view.confirmed_victim_count
        n_cand = len(view.victims)
        self._victims.setText(f'victims ◉ {n_conf} / ○ {n_cand}')
        if n_conf > self._last_confirmed:
            pulse_color(self._victims, _P.focus, _P.text_body,
                        stylesheet_fmt=self._victims_fmt)
        self._last_confirmed = n_conf
