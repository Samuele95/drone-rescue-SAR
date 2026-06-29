"""Past Runs tab: a table over runs/*.json.

Reads every JSONL summary in the runs directory at startup and on focus,
shows the high-level metrics, and offers a JSON inspector dialog when the
operator double-clicks a row.

Selection is exposed via `selectedRunPaths()` so the Compare Runs tab can
read which runs the operator wants to overlay.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List, Optional

from python_qt_binding.QtCore import Qt, Signal
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTableWidget,
    QHeaderView, QDialog, QTextBrowser, QSizePolicy,
    QFileDialog, QMessageBox,
)

from .. import report   # PDF export
from ._table_helpers import set_cell


COLUMNS = [
    'Date', 'Scenario', 'Pattern', 'Confirmed/GT', 'FP', 'TP', 'FN',
    'Final cov%', 'Duration', 'Ended', 'File',
]


class PastRunsTab(QWidget):
    """Table over the runs directory."""

    selectionChanged = Signal()   # fires whenever the operator's check set changes

    def __init__(self, runs_dir: Path, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._runs_dir = runs_dir

        outer = QVBoxLayout(self)

        row = QHBoxLayout()
        self._info_label = QLabel(f'Reading from {runs_dir}')
        self._info_label.setStyleSheet(f'color:{_P.text_muted};')
        row.addWidget(self._info_label)
        row.addStretch(1)
        # PDF export. Enabled only when exactly one row is selected
        # (exporting for an arbitrary multi-selection is the Sweep Runs
        # tab's job).
        self._export_pdf_btn = QPushButton('Export PDF…')
        self._export_pdf_btn.clicked.connect(self._on_export_pdf)
        self._export_pdf_btn.setEnabled(False)
        self._export_pdf_btn.setStyleSheet(
            f'background:{_P.action_export}; color:white; padding:6px 14px;'
        )
        row.addWidget(self._export_pdf_btn)
        self._refresh_btn = QPushButton('Refresh')
        self._refresh_btn.clicked.connect(self.refresh)
        row.addWidget(self._refresh_btn)
        outer.addLayout(row)

        self._table = QTableWidget(0, len(COLUMNS))
        self._table.setHorizontalHeaderLabels(COLUMNS)
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._table.verticalHeader().setVisible(False)
        self._table.setSelectionBehavior(QTableWidget.SelectRows)
        self._table.setSelectionMode(QTableWidget.MultiSelection)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._table.itemSelectionChanged.connect(self.selectionChanged.emit)
        self._table.itemSelectionChanged.connect(self._update_export_btn)
        self._table.cellDoubleClicked.connect(self._open_inspector)
        outer.addWidget(self._table, stretch=1)

        self.refresh()

    # --------------------------------------------------------- refresh
    def refresh(self) -> None:
        if not self._runs_dir.is_dir():
            self._table.setRowCount(0)
            return
        files = sorted(self._runs_dir.glob('*.json'))
        self._table.setRowCount(len(files))
        # cache the typed RunSummary per path so the inspector
        # (_open_inspector) reuses it via .to_dict() instead of re-reading +
        # re-parsing the full JSONL on the UI thread at double-click time.
        # Cleared each refresh = cache invalidation.
        self._run_cache = {}
        # typed RunSummary access. The back-compat dict shim is no longer
        # the consumer here; we read `.metadata.scenario` etc. directly
        # off the VO.
        from .. import analytics
        for row, f in enumerate(files):
            try:
                rs = analytics.load_run_typed(f)
            except Exception:
                continue
            self._run_cache[str(f)] = rs
            meta = rs.metadata
            summ = rs.summary
            gt = len(meta.ground_truth_victims)
            tp = summ.true_positives
            fp = summ.false_positives
            fn = summ.false_negatives
            self._set(row, 0, meta.started_at)
            self._set(row, 1, meta.scenario)
            self._set(row, 2, meta.pattern)
            tp_color = _P.ok if tp >= max(1, gt - 1) else _P.warn
            self._set(row, 3, f'{tp}/{gt}', tp_color)
            self._set(row, 4, str(fp),
                      _P.error if fp > 0 else _P.text_muted)
            self._set(row, 5, str(tp), _P.ok)
            self._set(row, 6, str(fn),
                      _P.warn if fn > 0 else _P.text_muted)
            self._set(row, 7, f'{summ.final_coverage_pct:.1f}')
            self._set(row, 8, f'{meta.duration_s:.0f}s')
            ended = meta.ended_by
            ec = (_P.ok if ended == 'MISSION_COMPLETE'
                  else _P.warn if ended == 'MISSION_TIMEOUT'
                  else _P.text_muted)
            self._set(row, 9, ended, ec)
            self._set(row, 10, f.name)
            # Stash the path on every cell of the row for easy lookup later.
            for col in range(self._table.columnCount()):
                item = self._table.item(row, col)
                if item is not None:
                    item.setData(Qt.UserRole, str(f))

    def _set(self, row: int, col: int, text: str, color: str = '') -> None:
        set_cell(self._table, row, col, text, color)

    # --------------------------------------------------------- selection
    def selected_run_paths(self) -> List[Path]:
        rows = sorted({i.row() for i in self._table.selectedItems()})
        out: List[Path] = []
        for r in rows:
            item = self._table.item(r, 0)
            if item is not None:
                p = item.data(Qt.UserRole)
                if p:
                    out.append(Path(p))
        return out

    # --------------------------------------------------------- export
    def _update_export_btn(self) -> None:
        """Enable Export PDF only when exactly one row is selected."""
        self._export_pdf_btn.setEnabled(len(self.selected_run_paths()) == 1)

    def _on_export_pdf(self) -> None:
        paths = self.selected_run_paths()
        if len(paths) != 1:
            return
        src = paths[0]
        # Default to the host-mounted reports folder (/reports) so the PDF is
        # reachable from the host, not the drone-runs volume the run lives on.
        default_out = str(report.export_dir(src.parent) / src.with_suffix('.pdf').name)
        out, _ = QFileDialog.getSaveFileName(
            self, 'Export PDF', default_out, 'PDF (*.pdf)',
        )
        if not out:
            return
        try:
            report.render_run_pdf(src, Path(out))
        except Exception as e:
            QMessageBox.critical(self, 'Export failed',
                                 f'{type(e).__name__}: {e}')
            return
        hint = ('\n\nThis is the host-mounted reports folder — find it on your '
                'machine under ./reports (or $DRONE_REPORTS_DIR).'
                if str(out).startswith('/reports') else '')
        QMessageBox.information(self, 'Export OK', f'Wrote {out}{hint}')

    # --------------------------------------------------------- inspector
    def _open_inspector(self, row: int, _col: int) -> None:
        item = self._table.item(row, 0)
        if item is None:
            return
        path = Path(item.data(Qt.UserRole))
        # reuse the RunSummary cached in refresh (load_run is exactly
        # RunSummary.from_jsonl().to_dict(), so the cached VO's to_dict() is
        # equivalent without a second file read). Fall back to a fresh load
        # if the cache misses (e.g. row predates the last refresh).
        from .. import analytics
        cached = getattr(self, '_run_cache', {}).get(str(path))
        try:
            doc = cached.to_dict() if cached is not None else analytics.load_run(path)
        except Exception:
            return
        dlg = QDialog(self)
        dlg.setWindowTitle(path.name)
        dlg.resize(800, 600)
        lay = QVBoxLayout(dlg)
        body = QTextBrowser()
        body.setPlainText(json.dumps(doc, indent=2))
        lay.addWidget(body)
        dlg.exec_()
