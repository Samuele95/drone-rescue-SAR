"""Pattern boxplots over an aggregated sweep: the headline poster figure.

The last ``make_*_figure`` body remaining in ``analytics.py`` after
the per-file figure extraction. Lives here with the other per-file
figure modules; ``analytics.make_pattern_boxplots_figure`` stays as
an import shim so report.py / sweep_tab.py call sites are unchanged.

NOT registered in ``registry``: the registry's renderers consume
per-run ``RunSummary`` sequences, while this figure consumes
``aggregate_sweep`` output rows (+ optional raw run dicts). It is a
sweep-level figure, invoked directly by its two callers.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from matplotlib.figure import Figure

from .common import empty


def render(
    rows: List[Dict[str, Any]],
    metrics: Tuple[str, ...] = (
        'final_coverage_pct', 'f1_score',
        'task_fairness_jain', 'energy_per_coverage_pct_J',
    ),
    raw_runs: Optional[List[Dict]] = None,
) -> Figure:
    """Boxplots over an aggregated sweep: the headline poster figure.

    `rows` is the output of `aggregate_sweep`. `raw_runs` (optional)
    lets us draw the underlying per-trial scatter on top of each box;
    when omitted, we fall back to drawing means with CI-error-bars
    (because the aggregated rows already have CI95 fields).
    """
    if not metrics:
        metrics = ('final_coverage_pct',)
    fig = Figure(figsize=(8, max(3.0, 2.5 * len(metrics))), tight_layout=True)
    if not rows:
        ax = fig.add_subplot(111)
        empty(ax, 'no runs to aggregate')
        return fig
    patterns = sorted({r.get('pattern', '?') for r in rows})
    for mi, metric in enumerate(metrics):
        ax = fig.add_subplot(len(metrics), 1, mi + 1)
        if raw_runs is not None:
            # True boxplot per pattern from raw values across all
            # scenarios. Order patterns alphabetically.
            data: List[List[float]] = []
            for p in patterns:
                vals = [
                    float(r.get('summary', {}).get(metric))
                    for r in raw_runs
                    if r.get('metadata', {}).get('pattern') == p
                    and r.get('summary', {}).get(metric) is not None
                ]
                data.append(vals)
            if any(len(d) for d in data):
                ax.boxplot(data, labels=patterns, showmeans=True)
            else:
                empty(ax, f'{metric}: no values')
        else:
            # Aggregated mean + CI plot, one bar per pattern. The
            # legacy fallback formula averaged absolute `lo`/`hi`
            # bounds across multiple `(pattern, scenario)` rows then
            # subtracted the mean of means, combining two different
            # averages (mean-of-CIs and mean-of-means) into an
            # interval that has no defensible CI semantics. The honest
            # fallback is to plot the means without CI bars and label
            # the figure accordingly; the caller can supply `raw_runs`
            # for a real bootstrap CI.
            xs = list(range(len(patterns)))
            means = []
            for p in patterns:
                vals = [r.get(f'{metric}_mean') for r in rows
                        if r.get('pattern') == p]
                vals = [v for v in vals if v is not None]
                means.append(sum(vals) / len(vals) if vals else 0.0)
            ax.bar(xs, means, capsize=0,
                   color='#3b82f6', alpha=0.7)
            ax.set_xticks(xs)
            ax.set_xticklabels(patterns)
            # Marker on the title clarifies that the rendered bars are
            # means-only when raw_runs isn't supplied.
            ax.text(0.99, 0.95, '(means, no CI — pass raw_runs)',
                    transform=ax.transAxes, ha='right', va='top',
                    fontsize=7, color='#475569')
        ax.set_ylabel(metric)
        ax.grid(True, axis='y', alpha=0.3)
    return fig
