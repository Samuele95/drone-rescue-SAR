"""PDF report generator.

Two entry points: `render_run_pdf(jsonl, out_pdf)` produces a 5-page
report for one run; `render_sweep_pdf(sweep_dir, out_pdf)` produces a
report for an entire bench sweep: cover page (manifest summary), per-
metric boxplots, statistical aggregation table.

Used by the Mission Control "Export PDF" button (Past Runs tab) and the
"Export sweep PDF" button (Sweep Runs tab), and exposed as a standalone
console script `report` for headless batch reporting:

    ros2 run drone_rescue_mission_control report \\
        --run runs/baseline/2026-05-11_091536__spiral_out__default.json \\
        --out /tmp/report.pdf

    ros2 run drone_rescue_mission_control report \\
        --sweep runs/v5_baseline --out runs/v5_baseline/report.pdf

Implementation: matplotlib for the embedded plots, reportlab for the
page layout. No scipy / pandas dependency.
"""

from __future__ import annotations

import argparse
import io
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import cm
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    Image as PlatyImage, PageBreak,
)

# report.py is a flow assembler over the figure/stat/sweep Protocols
# rather than depending wholesale on `analytics`. The `from . import
# analytics` import is kept for `run_label`, `summarise`, and
# `make_pattern_boxplots_figure`, which remain in analytics until a
# future split lifts them out.
from . import analytics
from .figures import renderers as _figure_renderers
from .stats import t_test as _t_test
from .sweep.aggregator import aggregate_runs as _aggregate_runs
from .sweep.aggregator import load_sweep as _load_sweep


# ---------- shared helpers --------------------------------------------

def export_dir(fallback: Path) -> Path:
    """Preferred directory for an exported PDF.

    Returns the host-mounted reports folder when one is available: the
    ``DRONE_REPORTS_DIR`` env var, else ``/reports`` (the compose bind mount),
    so the file is reachable from the host. Falls back to ``fallback`` (the
    run's own directory, which inside the container lives on the
    non-host-accessible ``drone-runs`` volume) when neither exists/is writable.
    """
    for cand in (os.environ.get('DRONE_REPORTS_DIR'), '/reports'):
        if not cand:
            continue
        p = Path(cand)
        if p.is_dir() and os.access(p, os.W_OK):
            return p
    return fallback


def _figure_to_image(fig, width_cm: float = 16.0,
                     height_cm: float = 11.0) -> PlatyImage:
    """Render a matplotlib Figure into a reportlab Image flowable.

    Uses an in-memory PNG so we don't litter /tmp. After serialisation
    the Figure's axes are cleared via ``fig.clf()`` to release the
    numpy arrays each Axes holds; without this, sweep PDFs (30 trials
    x 8 figures = 240 live Figures) can exhaust process memory before
    ``doc.build()`` returns. PlatyImage reads the BytesIO at
    construction (its `validate=True` codepath does a seekable read),
    so the buffer can be safely closed once we've handed it over.
    """
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=120, bbox_inches='tight')
    buf.seek(0)
    img = PlatyImage(buf, width=width_cm * cm, height=height_cm * cm)
    fig.clf()
    return img


def _section_title(text: str, styles) -> Paragraph:
    return Paragraph(text, styles['Heading2'])


def _key_value_table(rows: List[List[str]], col_widths=None) -> Table:
    if col_widths is None:
        col_widths = [5 * cm, 11 * cm]
    t = Table(rows, colWidths=col_widths)
    t.setStyle(TableStyle([
        ('FONT', (0, 0), (-1, -1), 'Helvetica', 9),
        ('FONT', (0, 0), (0, -1), 'Helvetica-Bold', 9),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
        ('TOPPADDING', (0, 0), (-1, -1), 3),
        ('LINEBELOW', (0, 0), (-1, -2), 0.25, colors.grey),
    ]))
    return t


def _fmt_num(v, default: str = '—') -> str:
    if v is None:
        return default
    if isinstance(v, float):
        if abs(v) >= 1000:
            return f'{v:.0f}'
        if abs(v) >= 1:
            return f'{v:.2f}'
        return f'{v:.3f}'
    return str(v)


# ---------- single-run report ------------------------------------------

def render_run_pdf(jsonl_path: Path | str, out_pdf_path: Path | str) -> None:
    """Render a 5-page PDF for one run JSONL."""
    jsonl_path = Path(jsonl_path)
    out_pdf_path = Path(out_pdf_path)
    out_pdf_path.parent.mkdir(parents=True, exist_ok=True)

    # Load typed RunSummary; renderers + cover-page builders consume
    # typed attribute access directly.
    run = analytics.load_run_typed(jsonl_path)
    meta = run.metadata
    summ = run.summary
    styles = getSampleStyleSheet()
    styles.add(ParagraphStyle(
        name='SmallGray', parent=styles['Normal'],
        fontSize=8, textColor=colors.grey,
    ))

    doc = SimpleDocTemplate(
        str(out_pdf_path), pagesize=A4,
        leftMargin=2 * cm, rightMargin=2 * cm,
        topMargin=2 * cm, bottomMargin=2 * cm,
    )
    flow: List[Any] = []

    # --- cover page ---------------------------------------------------
    flow.append(Paragraph(
        f"Drone-Rescue Mission Report", styles['Heading1']))
    flow.append(Paragraph(
        f"<b>{meta.scenario}</b> &nbsp; · &nbsp; "
        f"<b>{meta.pattern}</b> &nbsp; · &nbsp; "
        f"{meta.started_at[:19].replace('T', ' ')}",
        styles['Heading3']))
    flow.append(Spacer(1, 0.4 * cm))

    rows = [
        ['Started at', meta.started_at or '?'],
        ['Ended at', meta.ended_at or '?'],
        ['Duration', f"{_fmt_num(meta.duration_s)} s"],
        ['Ended by', meta.ended_by],
        ['Scenario', meta.scenario],
        ['Pattern', meta.pattern],
        ['Source JSONL', jsonl_path.name],
    ]
    flow.append(_key_value_table(rows))
    flow.append(Spacer(1, 0.6 * cm))

    flow.append(_section_title('Summary metrics', styles))
    summ_rows = [
        ['Final coverage %', _fmt_num(summ.final_coverage_pct)],
        ['Time to coverage 50 %', f"{_fmt_num(summ.time_to_coverage_50pct_s)} s"],
        ['Time to coverage 80 %', f"{_fmt_num(summ.time_to_coverage_80pct_s)} s"],
        ['Time to coverage 90 %', f"{_fmt_num(summ.time_to_coverage_90pct_s)} s"],
        ['Energy / cov%', f"{_fmt_num(summ.energy_per_coverage_pct_J)} J"],
        ['Jain task fairness', _fmt_num(summ.task_fairness_jain)],
        ['', ''],
        ['Candidates emitted', _fmt_num(summ.candidates_emitted)],
        ['Victims confirmed', _fmt_num(summ.victims_confirmed)],
        ['True positives', _fmt_num(summ.true_positives)],
        ['False positives', _fmt_num(summ.false_positives)],
        ['False negatives', _fmt_num(summ.false_negatives)],
        ['Precision', _fmt_num(summ.precision)],
        ['Recall', _fmt_num(summ.recall)],
        ['F1 score', _fmt_num(summ.f1_score)],
        ['Time-to-first-detection', f"{_fmt_num(summ.time_to_first_detection_s)} s"],
        ['Time-to-first-confirm', f"{_fmt_num(summ.time_to_first_confirm_s)} s"],
        ['', ''],
        ['Drones down', _fmt_num(summ.drones_down)],
        ['Sector reassignments', _fmt_num(summ.sector_reassignments)],
    ]
    flow.append(_key_value_table(summ_rows))
    flow.append(Spacer(1, 0.4 * cm))
    flow.append(Paragraph(
        f"Report generated by drone_rescue_mission_control.report (V5).",
        styles['SmallGray'],
    ))

    # --- one figure per page --------------------------------------------
    # Page list derives from the FigureRenderer registry; order matches
    # the legacy hardcoded list (pinned by
    # `figures/registry.py:_bootstrap`). Adding a 10th figure is one
    # `register()` call there, no edit here.
    from .figures import renderers as _figure_renderers
    for renderer in _figure_renderers():
        flow.append(PageBreak())
        flow.append(_section_title(renderer.label, styles))
        try:
            fig = renderer.render([run])
            flow.append(_figure_to_image(fig))
        except Exception as e:
            flow.append(Paragraph(
                f"<i>(could not render: {type(e).__name__}: {e})</i>",
                styles['SmallGray']))

    # --- event log appendix --------------------------------------------
    flow.append(PageBreak())
    flow.append(_section_title('Event log (first 50 events)', styles))
    events = run.events
    if not events:
        flow.append(Paragraph('<i>(no events recorded)</i>', styles['SmallGray']))
    else:
        ev_rows = [['t (s)', 'type', 'drone', 'victim', 'detail']]
        for e in events[:50]:
            ev_rows.append([
                _fmt_num(e.get('t')),
                e.get('type', '?'),
                e.get('drone', '') or '',
                str(e.get('victim_id') or ''),
                (e.get('detail') or '')[:60],
            ])
        t = Table(ev_rows, colWidths=[1.5 * cm, 4 * cm, 2 * cm, 1.5 * cm, 7 * cm])
        t.setStyle(TableStyle([
            ('FONT', (0, 0), (-1, -1), 'Helvetica', 7.5),
            ('FONT', (0, 0), (-1, 0), 'Helvetica-Bold', 7.5),
            ('LINEBELOW', (0, 0), (-1, 0), 0.5, colors.black),
            ('LINEBELOW', (0, 1), (-1, -1), 0.25, colors.lightgrey),
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ]))
        flow.append(t)
        if len(events) > 50:
            flow.append(Spacer(1, 0.3 * cm))
            flow.append(Paragraph(
                f"<i>… {len(events) - 50} more events truncated; "
                f"see source JSONL for the full log.</i>",
                styles['SmallGray']))

    doc.build(flow)


# ---------- whole-sweep report -----------------------------------------

def render_sweep_pdf(sweep_dir: Path | str, out_pdf_path: Path | str) -> None:
    """Render a multi-page PDF for an entire bench sweep dir."""
    sweep_dir = Path(sweep_dir)
    out_pdf_path = Path(out_pdf_path)
    out_pdf_path.parent.mkdir(parents=True, exist_ok=True)

    # Load via the sweep module directly; sweep is its own bounded
    # subdomain and report.py becomes the mediator that wires it to the
    # figure-rendering layer.
    runs = _load_sweep(sweep_dir)
    manifest = (_load_sweep(sweep_dir, manifest_only=True) or [{}])[0]
    # Consume typed SweepAggregateRow VOs directly; the `.as_dict()`
    # back-compat round-trip is gone now that _aggregation_tables reads
    # `row.metrics[key]`.
    agg = list(_aggregate_runs(runs))

    styles = getSampleStyleSheet()
    styles.add(ParagraphStyle(
        name='SmallGray', parent=styles['Normal'],
        fontSize=8, textColor=colors.grey,
    ))
    doc = SimpleDocTemplate(
        str(out_pdf_path), pagesize=A4,
        leftMargin=2 * cm, rightMargin=2 * cm,
        topMargin=2 * cm, bottomMargin=2 * cm,
    )
    flow: List[Any] = []

    # --- cover page ---------------------------------------------------
    flow.append(Paragraph(
        f"Drone-Rescue Sweep Report", styles['Heading1']))
    flow.append(Paragraph(
        f"<b>{sweep_dir.name}</b> &nbsp; · &nbsp; {len(runs)} runs",
        styles['Heading3']))
    flow.append(Spacer(1, 0.4 * cm))
    cover_rows = [
        ['Started at', manifest.get('started_at', '?')],
        ['Ended at', manifest.get('ended_at', '?')],
        ['Patterns', ', '.join(manifest.get('patterns', [])) or '?'],
        ['Scenarios', ', '.join(manifest.get('scenarios', [])) or '?'],
        ['Trials per combination', _fmt_num(manifest.get('trials'))],
        ['Seed-start', _fmt_num(manifest.get('seed_start'))],
        ['Git SHA at sweep time', manifest.get('git_sha', '?') or '?'],
        ['Source dir', str(sweep_dir)],
    ]
    flow.append(_key_value_table(cover_rows))
    flow.append(Spacer(1, 0.5 * cm))
    flow.append(Paragraph(
        f"Generated by drone_rescue_mission_control.report (V5).",
        styles['SmallGray'],
    ))

    # --- per-pattern boxplots -----------------------------------------
    flow.append(PageBreak())
    flow.append(_section_title('Per-pattern boxplots', styles))
    flow.append(Paragraph(
        'Boxes summarise per-trial values across all scenarios for each '
        'pattern. Whiskers extend to 1.5×IQR; means are shown as green '
        'triangles. The middle line is the median.',
        styles['Normal']))
    flow.append(Spacer(1, 0.3 * cm))
    try:
        fig = analytics.make_pattern_boxplots_figure(agg, raw_runs=runs)
        flow.append(_figure_to_image(fig, width_cm=16, height_cm=22))
    except Exception as e:
        flow.append(Paragraph(
            f"<i>(could not render boxplot: {type(e).__name__}: {e})</i>",
            styles['SmallGray']))

    # --- aggregated table page ----------------------------------------
    flow.append(PageBreak())
    flow.append(_section_title('Statistical aggregation', styles))
    flow.append(Paragraph(
        'Per-(pattern, scenario) means with bootstrap-95% confidence '
        'intervals. n = number of trials in the cell. Bootstrap uses '
        '10 000 resamples; CI is the percentile interval. Cite Efron 1979.',
        styles['Normal']))
    flow.append(Spacer(1, 0.3 * cm))
    flow.extend(_aggregation_tables(agg, styles))

    # --- pairwise t-tests page ----------------------------------------
    flow.append(PageBreak())
    flow.append(_section_title('Pairwise pattern comparison (Welch t-test)',
                               styles))
    flow.append(Paragraph(
        "Two-sided p-value for the null hypothesis that two patterns have "
        "equal means on the metric. p < 0.05 means the difference is "
        "unlikely to be sampling noise. Cite Welch 1947.",
        styles['Normal']))
    flow.append(Spacer(1, 0.3 * cm))
    flow.extend(_pairwise_ttest_tables(runs, styles))

    doc.build(flow)


def _aggregation_tables(rows: List[Any], styles) -> List[Any]:
    """Render the aggregate_sweep output as a key metrics table.

    Consumes typed SweepAggregateRow VOs. The `.as_dict()` back-compat
    shim that used to live in the caller is no longer needed; access
    ``row.metrics[key]`` directly.
    """
    if not rows:
        return [Paragraph('<i>(no rows)</i>', styles['SmallGray'])]
    metrics = [
        ('final_coverage_pct',      'cov %'),
        ('f1_score',                'F1'),
        ('time_to_coverage_80pct_s', 't→80%'),
        ('energy_per_coverage_pct_J', 'J/cov%'),
        ('task_fairness_jain',      'Jain'),
        ('drones_down',             'down'),
    ]
    header = ['pattern', 'scenario', 'n']
    for _, short in metrics:
        header.extend([short, '95% CI'])
    table_rows: List[List[Any]] = [header]
    for r in rows:
        pattern = str(r.group_keys[0]) if len(r.group_keys) > 0 else '?'
        scenario = str(r.group_keys[1]) if len(r.group_keys) > 1 else '?'
        row = [pattern, scenario, _fmt_num(r.n_trials)]
        for key, _ in metrics:
            cell = r.metrics.get(key)
            if cell is None:
                row.append('—')
                row.append('—')
                continue
            row.append(_fmt_num(cell.mean))
            if cell.ci95_lo is not None and cell.ci95_hi is not None:
                row.append(
                    f'[{_fmt_num(cell.ci95_lo)}, {_fmt_num(cell.ci95_hi)}]'
                )
            else:
                row.append('—')
        table_rows.append(row)
    n_cols = len(header)
    col_w = (16 * cm) / n_cols
    t = Table(table_rows, colWidths=[col_w] * n_cols)
    t.setStyle(TableStyle([
        ('FONT', (0, 0), (-1, -1), 'Helvetica', 7.5),
        ('FONT', (0, 0), (-1, 0), 'Helvetica-Bold', 7.5),
        ('LINEBELOW', (0, 0), (-1, 0), 0.5, colors.black),
        ('LINEBELOW', (0, 1), (-1, -1), 0.25, colors.lightgrey),
        ('ALIGN', (2, 1), (-1, -1), 'RIGHT'),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
    ]))
    return [t]


def _pairwise_ttest_tables(runs: List[Dict],
                           styles) -> List[Any]:
    """Tabulate Welch t-tests between every pair of patterns on a small
    set of headline metrics. Pooled across scenarios."""
    patterns = sorted({
        r.get('metadata', {}).get('pattern', '?') for r in runs
    })
    if len(patterns) < 2:
        return [Paragraph('<i>(need at least two patterns to compare)</i>',
                          styles['SmallGray'])]
    headline_metrics = [
        ('final_coverage_pct',  'coverage %'),
        ('f1_score',            'F1 score'),
        ('time_to_coverage_80pct_s', 'time→80% (s)'),
    ]
    blocks: List[Any] = []
    for metric, label in headline_metrics:
        # Build values list per pattern.
        vals = {p: [
            r.get('summary', {}).get(metric)
            for r in runs
            if r.get('metadata', {}).get('pattern') == p
        ] for p in patterns}
        # Filter Nones.
        for p in patterns:
            vals[p] = [v for v in vals[p] if v is not None]
        # Build matrix.
        header = [label] + patterns
        table_rows: List[List[Any]] = [header]
        for a in patterns:
            row: List[Any] = [a]
            for b in patterns:
                if a == b:
                    row.append('—')
                    continue
                if not vals[a] or not vals[b]:
                    row.append('n/a')
                    continue
                # Consume the StatTest port directly (Welch's t-test
                # lives in `lib/stats`).
                _, p_value = _t_test(vals[a], vals[b])
                # Mark significance levels.
                marker = ''
                if p_value < 0.001:
                    marker = ' ***'
                elif p_value < 0.01:
                    marker = ' **'
                elif p_value < 0.05:
                    marker = ' *'
                row.append(f'p={p_value:.3g}{marker}')
            table_rows.append(row)
        n_cols = len(header)
        col_w = (16 * cm) / n_cols
        t = Table(table_rows, colWidths=[col_w] * n_cols)
        t.setStyle(TableStyle([
            ('FONT', (0, 0), (-1, -1), 'Helvetica', 8),
            ('FONT', (0, 0), (-1, 0), 'Helvetica-Bold', 8),
            ('FONT', (0, 0), (0, -1), 'Helvetica-Bold', 8),
            ('LINEBELOW', (0, 0), (-1, 0), 0.5, colors.black),
            ('LINEBELOW', (0, 1), (-1, -1), 0.25, colors.lightgrey),
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ]))
        blocks.append(t)
        blocks.append(Spacer(1, 0.5 * cm))
    blocks.append(Paragraph(
        '<i>Significance markers: *** p&lt;0.001, ** p&lt;0.01, * p&lt;0.05.</i>',
        styles['SmallGray']))
    return blocks


# ---------- CLI -------------------------------------------------------

def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        prog='report',
        description='V5 PDF report generator. Pick exactly one of '
                    '--run or --sweep.',
    )
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--run', type=Path,
                   help='Path to a single mission_recorder JSONL.')
    g.add_argument('--sweep', type=Path,
                   help='Path to a bench sweep directory.')
    p.add_argument('--out', type=Path, required=True,
                   help='Output PDF path.')
    args = p.parse_args(argv)
    try:
        if args.run is not None:
            render_run_pdf(args.run, args.out)
            print(f'wrote {args.out}')
        else:
            render_sweep_pdf(args.sweep, args.out)
            print(f'wrote {args.out}')
        return 0
    except Exception as e:
        print(f'error: {type(e).__name__}: {e}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
