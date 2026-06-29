"""Sweep Runs tab.

Loads a sweep directory (one written by `bench.py`), shows the manifest
header + an aggregated per-(pattern, scenario) table + the headline
boxplot figure side-by-side, and offers an "Export sweep PDF" button.

The tab has no per-mission lifecycle; it's purely a viewer over a
directory full of recorded JSONLs. Mission Control's other tabs handle
producing those JSONLs.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFileDialog,
    QTableWidget, QTableWidgetItem, QHeaderView, QSplitter, QMessageBox,
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from .. import analytics
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P
from .. import report
# typed sweep aggregation. The widget no longer walks `<metric>_mean` /
# `<metric>_ci95_lo` flat-dict keys; `aggregate_runs` returns
# SweepAggregateRow VOs whose `metrics` field is a typed
# `Mapping[str, SweepMetricCell]`.
from ..sweep.aggregator import aggregate_runs as _aggregate_runs


_TABLE_METRICS = (
    ('final_coverage_pct',       'cov %'),
    ('f1_score',                 'F1'),
    ('time_to_coverage_80pct_s', 't→80%'),
    ('energy_per_coverage_pct_J', 'J/cov%'),
    ('task_fairness_jain',       'Jain'),
    ('drones_down',              'down'),
)


class SweepTab(QWidget):
    """Multi-run aggregator. Operator picks a sweep dir; we render the
    aggregated table + boxplots + offer a sweep-PDF export.
    """

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._sweep_dir: Optional[Path] = None
        self._canvas: Optional[FigureCanvas] = None

        outer = QVBoxLayout(self)

        # Top control row.
        ctrl = QHBoxLayout()
        self._open_btn = QPushButton('Open sweep directory…')
        self._open_btn.clicked.connect(self._on_open)
        ctrl.addWidget(self._open_btn)

        self._info_label = QLabel('No sweep loaded.')
        self._info_label.setStyleSheet(f'color:{_P.text_muted}; padding-left:12px;')
        ctrl.addWidget(self._info_label)

        ctrl.addStretch(1)
        self._refresh_btn = QPushButton('Refresh')
        self._refresh_btn.clicked.connect(self._refresh)
        self._refresh_btn.setEnabled(False)
        ctrl.addWidget(self._refresh_btn)

        self._export_btn = QPushButton('Export sweep PDF…')
        self._export_btn.clicked.connect(self._on_export)
        self._export_btn.setEnabled(False)
        self._export_btn.setStyleSheet(
            f'background:{_P.action_export}; color:white; padding:6px 14px;'
        )
        ctrl.addWidget(self._export_btn)
        outer.addLayout(ctrl)

        # Splitter: aggregated table on the left, boxplot on the right.
        self._splitter = QSplitter(Qt.Horizontal)
        outer.addWidget(self._splitter, stretch=1)

        # Left: table.
        self._table = QTableWidget(0, 0)
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._splitter.addWidget(self._table)

        # Right: boxplot canvas holder.
        self._canvas_holder = QWidget()
        self._canvas_holder_lay = QVBoxLayout(self._canvas_holder)
        self._canvas_holder_lay.setContentsMargins(0, 0, 0, 0)
        self._splitter.addWidget(self._canvas_holder)
        self._splitter.setStretchFactor(0, 1)
        self._splitter.setStretchFactor(1, 1)

    # --------------------------------------------------------- open
    def _on_open(self) -> None:
        d = QFileDialog.getExistingDirectory(
            self, 'Open sweep directory',
            str(Path.cwd()),
        )
        if not d:
            return
        self._sweep_dir = Path(d)
        self._refresh()

    # --------------------------------------------------------- refresh
    def _refresh(self) -> None:
        if self._sweep_dir is None:
            return
        # manifest first (fast O(1) manifest.json read), then the full
        # sweep load needed for aggregation: two explicit ordered calls
        # instead of the prior full-then-manifest pair.
        manifest = (analytics.load_sweep(self._sweep_dir, manifest_only=True)
                    or [{}])[0]
        runs = analytics.load_sweep(self._sweep_dir)
        n_runs = len(runs)
        patterns = manifest.get('patterns', sorted({
            r.get('metadata', {}).get('pattern', '?') for r in runs
        }))
        scenarios = manifest.get('scenarios', sorted({
            r.get('metadata', {}).get('scenario', '?') for r in runs
        }))
        self._info_label.setText(
            f'{self._sweep_dir.name}: {n_runs} runs · '
            f'{len(patterns)} patterns × {len(scenarios)} scenarios'
        )
        self._refresh_btn.setEnabled(True)
        self._export_btn.setEnabled(n_runs > 0)

        # typed SweepAggregateRow VOs. The `as_dict` shim is still used to
        # feed the boxplot builder (its raw-dict contract doesn't change
        # yet).
        group_by = ('pattern', 'scenario')
        rows = _aggregate_runs(runs, group_by=group_by)

        # Build the table.
        cols = ['pattern', 'scenario', 'n']
        for _, short in _TABLE_METRICS:
            cols.extend([short, '95% CI'])
        self._table.setColumnCount(len(cols))
        self._table.setHorizontalHeaderLabels(cols)
        self._table.setRowCount(len(rows))
        for ri, agg_row in enumerate(rows):
            ci = 0
            pattern, scenario = (
                agg_row.group_keys[0] if len(agg_row.group_keys) > 0 else '?',
                agg_row.group_keys[1] if len(agg_row.group_keys) > 1 else '?',
            )
            self._table.setItem(ri, ci, QTableWidgetItem(str(pattern))); ci += 1
            self._table.setItem(ri, ci, QTableWidgetItem(str(scenario))); ci += 1
            self._table.setItem(ri, ci, QTableWidgetItem(str(agg_row.n_trials))); ci += 1
            for key, _ in _TABLE_METRICS:
                cell = agg_row.metrics.get(key)
                mean = cell.mean if cell is not None else None
                lo = cell.ci95_lo if cell is not None else None
                hi = cell.ci95_hi if cell is not None else None
                self._table.setItem(ri, ci, QTableWidgetItem(
                    self._fmt(mean))); ci += 1
                if lo is None or hi is None:
                    self._table.setItem(ri, ci, QTableWidgetItem('—'))
                else:
                    self._table.setItem(ri, ci, QTableWidgetItem(
                        f'[{self._fmt(lo)}, {self._fmt(hi)}]'))
                ci += 1

        # Build the boxplot.
        try:
            agg = [r.as_dict(group_by) for r in rows]
            fig = analytics.make_pattern_boxplots_figure(agg, raw_runs=runs)
        except Exception as e:
            QMessageBox.warning(self, 'Plot error',
                                f'Could not build boxplots: {e}')
            return
        self._replace_canvas(fig)

    # --------------------------------------------------------- export
    def _on_export(self) -> None:
        if self._sweep_dir is None:
            return
        out, _ = QFileDialog.getSaveFileName(
            self, 'Export sweep PDF',
            # Default to the host-mounted /reports so the PDF lands on the host.
            str(report.export_dir(self._sweep_dir) / 'report.pdf'),
            'PDF (*.pdf)',
        )
        if not out:
            return
        try:
            report.render_sweep_pdf(self._sweep_dir, Path(out))
        except Exception as e:
            QMessageBox.critical(self, 'Export failed',
                                 f'{type(e).__name__}: {e}')
            return
        QMessageBox.information(self, 'Export OK',
                                f'Wrote {out}')

    # --------------------------------------------------------- helpers
    @staticmethod
    def _fmt(v) -> str:
        if v is None:
            return '—'
        if isinstance(v, float):
            if abs(v) >= 1000:
                return f'{v:.0f}'
            if abs(v) >= 1:
                return f'{v:.2f}'
            return f'{v:.3f}'
        return str(v)

    def _replace_canvas(self, fig) -> None:
        if self._canvas is not None:
            # deleteLater() schedules Qt-side deletion only: the
            # matplotlib Figure attached to the canvas keeps every Axes'
            # numpy arrays alive until the GC eventually picks it up.
            # Across 10 Refresh clicks on a 30-run sweep that's hundreds
            # of un-collected arrays. fig.clf() is the canonical
            # matplotlib/Qt teardown and is idempotent, so it's safe even
            # if Qt has already started destroying the canvas.
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
