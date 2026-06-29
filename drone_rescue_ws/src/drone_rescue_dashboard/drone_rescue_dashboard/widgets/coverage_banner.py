"""CoverageBanner: top-of-overview banner showing coverage %.

Extracted from ``dashboard_app.py`` so the widget is unit-testable
against a synthetic StateCache. Consumes ``state.view.coverage`` and
``state.view.victims`` exclusively.
"""

from __future__ import annotations

from typing import Optional

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QLabel, QWidget
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P


class CoverageBanner(QLabel):
    """Top-of-overview banner showing coverage % and victim counts.

    Victim counts are computed from ``state.view.victims`` (the
    candidate stream) directly, because the upstream
    ``/coverage/metrics.victims_found`` is fed by ``coverage_tracker``
    which only counts the legacy ``/victims/detected`` PoseStamped
    passthrough and misses auto-confirmed candidates from the
    detection_filter.
    """

    def __init__(self, state, *, bridge=None,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._state = state
        self.setMinimumHeight(40)
        self.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setPointSize(12)
        font.setBold(True)
        self.setFont(font)
        self.setStyleSheet(
            f'background-color: {_P.bg_panel}; color: {_P.text_body}; padding: 4px;'
        )
        self.setText('Mission: waiting for /coverage/metrics …')
        # Bridge mode: repaint on view change only.
        if bridge is not None:
            bridge.view_changed.connect(lambda _v: self._refresh())
        else:
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._refresh)
            self._timer.start(500)   # 2 Hz

    def _refresh(self) -> None:
        view = self._state.view
        cov = view.coverage
        n_candidates = len(view.victims)
        # Single-source fold on the view model.
        n_confirmed = view.confirmed_victim_count
        if cov.elapsed_time_seconds == 0.0 and cov.cells_visited == 0:
            self.setText(
                f'Mission: waiting for /coverage/metrics  •  '
                f'Confirmed: {n_confirmed} / Candidates: {n_candidates}'
            )
            return
        # Surface the scan-time ETA (0.0 = not yet estimable).
        eta = getattr(cov, 'estimated_time_remaining', 0.0)
        eta_str = f'  •  ETA: {eta:.0f}s' if eta > 0.0 else ''
        # Fleet flight-plan feasibility: mission_manager folds a
        # " NO-GO(margin)" marker into each drone's active_tasks_summary line
        # when its remaining plan + return no longer fits its battery endurance.
        feas_str = ''
        mission = getattr(view, 'mission', None)
        if mission is not None and getattr(mission, 'received', False):
            summ = getattr(mission, 'active_tasks_summary', ()) or ()
            no_go = sum(1 for s in summ if 'NO-GO' in s)
            if no_go > 0:
                feas_str = f'  •  ⚠ Plan: {no_go}/{len(summ)} NO-GO'
        self.setText(
            f'Coverage: {cov.percentage:.1f}%  •  '
            f'Cells: {cov.cells_visited}/{cov.total_cells}  •  '
            f'Victims (confirmed/candidates): {n_confirmed}/{n_candidates}  •  '
            f'Elapsed: {cov.elapsed_time_seconds:.0f}s{eta_str}{feas_str}'
        )
