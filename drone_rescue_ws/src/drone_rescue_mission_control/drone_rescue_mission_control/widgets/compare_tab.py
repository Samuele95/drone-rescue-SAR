"""Compare Runs tab: K-up overlay of recorded runs.

Reads which JSONLs the operator selected over in Past Runs (we expose the
list via the parent `mission_control_app`), builds a matplotlib Figure for
the chosen metric via `analytics.py`, and embeds it in the tab via
`FigureCanvasQTAgg`. Below the canvas, a small per-run summary table makes
the comparison numeric in addition to visual.

The metric dropdown drives which `analytics.make_*_figure` call we make.
We rebuild the figure on every refresh; these are small (<=10 runs, a few
hundred samples each), so cost is trivial and the code stays simple.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, List, Optional

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QTableWidget, QTableWidgetItem, QHeaderView, QSplitter,
)

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from .. import analytics
# figure list derives from the FigureRenderer registry; adding a 10th
# figure is one register() call in figures/registry.py (no widget edits
# needed).
from ..figures import renderers as _figure_renderers
# typed RunSummary + RunViewModel for the summary table; figure builders
# still receive raw dicts (via rs.to_dict()) until their consumers migrate.
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P
from drone_rescue_ui_common.run_view_model import RunViewModel


def _build_metrics_list():
    """Materialise the legacy `(label, builder_callable)` tuple list
    by iterating the registry. Adapter so the rest of this widget's
    code keeps the same shape; once the widget folds a typed
    RunViewModel, the adapter can retire."""
    return [(r.label, r.render) for r in _figure_renderers()]


_METRICS = _build_metrics_list()

_SUMMARY_COLS = [
    'run', 'scenario', 'pattern', 'duration_s', 'final_coverage_pct',
    'tp', 'fp', 'fn', 't_first_detection_s', 't_first_confirm_s',
    'drones_down', 'sector_reassignments',
]

# Return the formatted cell values directly from the RunRow in
# `_SUMMARY_COLS` order, instead of building a parallel string-keyed dict
# the consumer then re-walks with `.get`. The RunRow VO stays the single
# source; this carries only per-column presentation formatting (rounding /
# 0.0 coercion / field remaps like tp->true_positives).
def _row_cells(row, label: str) -> list:
    return [
        label,                                    # run
        row.scenario,                             # scenario
        row.pattern,                              # pattern
        round(row.duration_s, 1),                 # duration_s
        round(row.final_coverage_pct, 1),         # final_coverage_pct
        row.true_positives,                       # tp
        row.false_positives,                      # fp
        row.false_negatives,                      # fn
        (round(row.time_to_first_detection_s, 2)  # t_first_detection_s
         if row.time_to_first_detection_s else 0.0),
        (round(row.time_to_first_confirm_s, 2)    # t_first_confirm_s
         if row.time_to_first_confirm_s else 0.0),
        row.drones_down,                          # drones_down
        row.sector_reassignments,                 # sector_reassignments
    ]


class CompareTab(QWidget):
    """Overlay K runs and show a summary table."""

    def __init__(
        self,
        get_selected_paths: Callable[[], List[Path]],
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self._get_selected_paths = get_selected_paths
        self._canvas: Optional[FigureCanvas] = None

        outer = QVBoxLayout(self)

        # Top control row.
        ctrl = QHBoxLayout()
        ctrl.addWidget(QLabel('Metric:'))
        self._metric_combo = QComboBox()
        for label, _ in _METRICS:
            self._metric_combo.addItem(label)
        ctrl.addWidget(self._metric_combo)

        self._selection_label = QLabel('No runs selected.')
        self._selection_label.setStyleSheet(
            f'color: {_P.text_muted}; padding-left: 12px;')
        ctrl.addWidget(self._selection_label)

        ctrl.addStretch(1)
        self._refresh_btn = QPushButton('Refresh')
        ctrl.addWidget(self._refresh_btn)
        outer.addLayout(ctrl)

        # Splitter: figure on top, summary table on bottom.
        self._splitter = QSplitter(Qt.Vertical)
        outer.addWidget(self._splitter, stretch=1)

        # Placeholder canvas slot (replaced on every refresh).
        self._canvas_holder = QWidget()
        self._canvas_holder_lay = QVBoxLayout(self._canvas_holder)
        self._canvas_holder_lay.setContentsMargins(0, 0, 0, 0)
        self._splitter.addWidget(self._canvas_holder)

        # Summary table.
        self._table = QTableWidget(0, len(_SUMMARY_COLS))
        self._table.setHorizontalHeaderLabels(_SUMMARY_COLS)
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._splitter.addWidget(self._table)
        self._splitter.setStretchFactor(0, 4)
        self._splitter.setStretchFactor(1, 1)

        # Wiring
        self._metric_combo.currentIndexChanged.connect(self.refresh)
        self._refresh_btn.clicked.connect(self.refresh)

        self.refresh()

    # --------------------------------------------------------- public
    def refresh(self) -> None:
        paths = self._get_selected_paths()
        n = len(paths)
        self._selection_label.setText(
            f'{n} run{"s" if n != 1 else ""} selected'
        )

        # load typed RunSummary VOs and fold them through a RunViewModel
        # for the summary table. Figure builders consume typed RunSummary
        # directly; the .to_dict() round-trip is gone.
        summaries = []
        for p in paths:
            try:
                summaries.append(analytics.load_run_typed(p))
            except Exception:
                # Skip malformed files; fail soft so the rest still renders.
                pass

        # Rebuild the figure.
        idx = self._metric_combo.currentIndex()
        _label, builder = _METRICS[idx]
        fig = builder(summaries)
        self._replace_canvas(fig)

        # Refill the summary table: typed walk through RunViewModel.
        vm = RunViewModel()
        labels = [analytics.run_label(rs) for rs in summaries]
        for rs in summaries:
            vm = vm.apply(rs)
        self._table.setRowCount(len(vm.rows))
        for ri, (row, label) in enumerate(zip(vm.rows, labels)):
            cells = _row_cells(row, label)   # values in _SUMMARY_COLS order
            for ci, v in enumerate(cells):
                item = QTableWidgetItem('' if v is None else str(v))
                item.setTextAlignment(Qt.AlignLeft if ci < 3 else Qt.AlignRight)
                self._table.setItem(ri, ci, item)

    # --------------------------------------------------------- helpers
    def _replace_canvas(self, fig) -> None:
        # Drop the old canvas (matplotlib leaks if you don't release it).
        # Mirror sweep_tab's pattern: figure.clf() before deleteLater() so
        # the Axes/numpy arrays the Figure holds get released immediately,
        # not on the next GC. Without this every metric-dropdown change
        # accumulates a Figure-worth of state.
        if self._canvas is not None:
            try:
                self._canvas.figure.clf()
            except Exception:
                pass
            self._canvas_holder_lay.removeWidget(self._canvas)
            self._canvas.deleteLater()
            self._canvas = None
        self._canvas = FigureCanvas(fig)
        self._canvas_holder_lay.addWidget(self._canvas)
        self._canvas.draw_idle()
