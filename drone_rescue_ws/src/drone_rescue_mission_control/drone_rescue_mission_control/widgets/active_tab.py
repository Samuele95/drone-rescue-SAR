"""Active tab: live mission view inside Mission Control.

Two stacked panels:
  * a status banner (Idle / Spawning / Activating / Running / Stopping /
    Done) with a Stop button
  * a `QTextBrowser` tail of the launch subprocess's stdout (handy when
    something fails before activation completes)

The dashboard window is the operator live view; this tab is just so the
launcher can show progress without forcing the user to alt-tab. The
buffer is bounded.
"""

from __future__ import annotations

import logging
import re
from collections import deque
from typing import Optional

from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextBrowser,
)

# semantic colours from the single palette source.
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P
# typed mission-lifecycle FSM (replaces implicit string state).
from drone_rescue_ui_common.mission_lifecycle import (
    MissionLifecycleState, MissionPhase, VALID_TRANSITIONS,
)

_LOG = logging.getLogger(__name__)


# word-boundary matches so a substring like 'battery_warn_threshold'
# or 'no_error_recovery' is NOT mis-coloured.
_ERROR_RE = re.compile(r'\b(error|traceback|exception)\b', re.IGNORECASE)
_WARN_RE = re.compile(r'\bwarn(?:ing)?\b', re.IGNORECASE)


_STATE_COLORS = {
    'IDLE':       _P.text_muted,
    'SPAWNING':   _P.info,
    'ACTIVATING': _P.info,
    'RUNNING':    _P.ok,
    'STOPPING':   _P.warn,
    'DONE':       _P.ok,
    'ERROR':      _P.error,
}


class ActiveTab(QWidget):
    stopRequested = Signal()

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)

        # the operator-observed mission lifecycle as a typed FSM.
        # set_state() advances it; illegal transitions are logged (not
        # raised) so a lifecycle race never crashes the console.
        self._lifecycle = MissionLifecycleState()

        self._stdout_buffer: deque = deque(maxlen=2000)
        # render cursor uses a monotonic counter rather than len(deque),
        # which plateaus at maxlen once the buffer is full and would
        # silently freeze the live stdout tail. Mirrors the
        # LogBuffer.total_appended pattern dashboard_app already uses.
        self._rendered = 0
        self._total_appended = 0

        outer = QVBoxLayout(self)

        # Banner row
        row = QHBoxLayout()
        self._state_label = QLabel('Idle')
        font = QFont(); font.setPointSize(14); font.setBold(True)
        self._state_label.setFont(font)
        self._state_label.setStyleSheet(
            f'background:{_STATE_COLORS["IDLE"]}; color:white; '
            f'padding:6px 12px; border-radius:4px;'
        )
        row.addWidget(self._state_label)
        self._detail_label = QLabel('No mission running.')
        self._detail_label.setStyleSheet(f'color:{_P.text_body}; padding:6px;')
        row.addWidget(self._detail_label)
        row.addStretch(1)
        self._stop_btn = QPushButton('Stop')
        self._stop_btn.setStyleSheet(
            f'background:{_P.action_stop}; color:white; padding:6px 18px;'
        )
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self.stopRequested.emit)
        row.addWidget(self._stop_btn)
        outer.addLayout(row)

        # Stdout tail
        self._log = QTextBrowser()
        self._log.setLineWrapMode(QTextBrowser.NoWrap)
        f = QFont('Monospace'); f.setStyleHint(QFont.TypeWriter)
        self._log.setFont(f)
        self._log.setStyleSheet(
            f'background:{_P.bg_dark}; color:{_P.text_body};'
        )
        outer.addWidget(self._log, stretch=1)

        # Refresh tail at 5 Hz from the buffer.
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._flush_tail)
        self._timer.start(200)

    # ------------------------------------------------------------ public
    def set_state(self, state: str, detail: str = '') -> None:
        # advance the typed lifecycle FSM. Unknown strings leave it
        # untouched; illegal-but-known transitions are logged and forced
        # (raise_on_invalid=False) rather than crashing the console.
        try:
            phase = MissionPhase(state)
        except ValueError:
            phase = None
        if phase is not None:
            cur = self._lifecycle.phase
            if phase != cur and phase not in VALID_TRANSITIONS.get(cur, set()):
                _LOG.warning('UI mission lifecycle: illegal transition %s → %s',
                             cur.value, phase.value)
            self._lifecycle = self._lifecycle.transition(
                phase, detail, raise_on_invalid=False,
            )
        color = _STATE_COLORS.get(state, _P.text_muted)
        self._state_label.setText(state.title())
        self._state_label.setStyleSheet(
            f'background:{color}; color:white; padding:6px 12px; '
            f'border-radius:4px;'
        )
        self._detail_label.setText(detail or '')
        # A detail prefixed with ⚠ marks a degraded-but-running mission
        # (e.g. a runtime param that failed to apply). Render it amber and
        # bold so it is unmissable even while the state banner stays green.
        if detail.startswith('⚠'):
            self._detail_label.setStyleSheet(
                f'color:{_P.warn}; font-weight:bold; padding:6px;'
            )
        else:
            self._detail_label.setStyleSheet(f'color:{_P.text_body}; padding:6px;')
        # Stop button only meaningful while a mission is alive (derived from
        # the typed lifecycle; falls back to the string check for unknown
        # states that didn't resolve to a MissionPhase).
        self._stop_btn.setEnabled(
            self._lifecycle.is_active() if phase is not None
            else state in ('SPAWNING', 'ACTIVATING', 'RUNNING')
        )

    def append_stdout(self, line: str) -> None:
        self._stdout_buffer.append(line)
        self._total_appended += 1

    def clear_log(self) -> None:
        self._stdout_buffer.clear()
        self._rendered = 0
        self._total_appended = 0
        self._log.clear()

    # ------------------------------------------------------------ tail
    def _flush_tail(self) -> None:
        # cursor in monotonic-total space, not in current-buffer-len
        # space. Once the deque hits maxlen, the oldest items are evicted
        # and `_total_appended - len(buffer)` is the index of the oldest
        # survivor.
        if self._total_appended == self._rendered:
            return
        oldest_idx = self._total_appended - len(self._stdout_buffer)
        slice_start = max(0, self._rendered - oldest_idx)
        new_lines = list(self._stdout_buffer)[slice_start:]
        self._rendered = self._total_appended
        for line in new_lines:
            # colorize: ERROR red, WARN amber (palette tokens, word-boundary).
            if _ERROR_RE.search(line):
                color = _P.error
            elif _WARN_RE.search(line):
                color = _P.warn
            else:
                color = _P.text_body
            self._log.append(
                f'<span style="color:{color}">{self._escape(line)}</span>'
            )
        sb = self._log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _escape(self, s: str) -> str:
        return (s.replace('&', '&amp;')
                 .replace('<', '&lt;')
                 .replace('>', '&gt;'))
