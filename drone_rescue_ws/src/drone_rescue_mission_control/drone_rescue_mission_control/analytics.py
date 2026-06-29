"""Pure functions to load run JSONLs and build matplotlib Figures.

No Qt here; the Compare Runs widget consumes the Figure objects via
FigureCanvasQTAgg. Keeping the analytics layer Qt-free makes it
trivially testable from a notebook or shell.

Each `make_*_figure(...)` returns a Matplotlib `Figure` ready to embed.
The functions tolerate missing fields (older JSONLs from before a metric
existed) by falling back to empty plots with a "no data" annotation.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib
matplotlib.use('Agg')   # no display until embedded


def load_run(path: Path | str) -> Dict:
    """Load a run JSONL as the legacy raw-dict shape.

    Routes through RunSummary VO so the V4/V5 schema upgrade logic
    lives in one place (the persistence layer's
    `RunSummary.from_jsonl`) instead of being duplicated inline at
    every consumer. Returns `RunSummary.to_dict()` so existing
    string-keyed consumers (`r.get('summary', {}).get('precision')`)
    keep working unchanged. New code paths should use
    `RunRepository.load(handle)` directly for the typed VO.
    """
    from .persistence import RunHandle, RunSummary
    summary = RunSummary.from_jsonl(Path(path))
    # Tag the originating handle on the dict so downstream callers
    # that need the path (e.g. run_label) can reach it.
    out = summary.to_dict()
    return out


def load_run_typed(path: Path | str) -> 'RunSummary':
    """Typed VO loader. New consumers (RunViewModel widgets, future
    report builders) call this; the legacy `load_run` shim returns the
    dict shape until callers migrate."""
    from .persistence import RunSummary
    return RunSummary.from_jsonl(Path(path))


def run_label(run) -> str:
    """Short human-readable label for a run, used in legends.

    Accepts both ``RunSummary`` and raw-dict shapes (back-compat for
    any remaining dict call site)."""
    if hasattr(run, 'metadata') and hasattr(run.metadata, 'scenario'):
        meta = run.metadata
        when = meta.started_at[:19].replace('T', ' ')
        return f'{meta.scenario} / {meta.pattern}  [{when}]'
    meta = run.get('metadata', {})
    when = meta.get('started_at', '')[:19].replace('T', ' ')
    return f'{meta.get("scenario", "?")} / {meta.get("pattern", "?")}  [{when}]'


# make_*_figure builders moved into per-file modules under
# ``figures/``. The names below remain importable as back-compat
# re-exports so legacy ``from . import analytics; analytics.make_X_figure``
# callers keep working. New code should iterate
# ``figures.registry.renderers()`` instead.
from .figures.coverage import render as make_coverage_figure  # noqa: E402
from .figures.cumulative_confirmed import (  # noqa: E402
    render as make_cumulative_confirmed_figure,
)
from .figures.per_drone_battery import (  # noqa: E402
    render as make_per_drone_battery_figure,
)
from .figures.task_histogram import render as make_task_histogram_figure  # noqa: E402


# ---------- summary table for the Compare tab -----------------------

# Single source of truth for summary metric keys.
# `summarise()` and `SWEEP_METRICS_DEFAULT` both derive from this
# table, so adding a new metric is one row here.
#
# Each entry: (canonical_key_in_recorder_summary, display_column,
#              rounding_digits or None for raw, default_when_missing).
SUMMARY_METRICS: Tuple[Tuple[str, str, Optional[int], Any], ...] = (
    ('final_coverage_pct',        'final_coverage_pct',   1,    0.0),
    ('true_positives',            'tp',                   None, 0),
    ('false_positives',           'fp',                   None, 0),
    ('false_negatives',           'fn',                   None, 0),
    ('time_to_first_detection_s', 't_first_detection_s',  None, None),
    ('time_to_first_confirm_s',   't_first_confirm_s',    None, None),
    ('drones_down',               'drones_down',          None, 0),
    ('sector_reassignments',      'sector_reassignments', None, 0),
)


def summarise(runs: List[Dict]) -> List[Dict]:
    """Per-run flat dict for the table under the comparison plot.

    The metric portion derives from SUMMARY_METRICS so `summarise()`
    and `aggregate_sweep` agree on the canonical keys. Non-metric
    headers (run, scenario, pattern, duration_s) are presentation-only
    and stay inline.
    """
    out = []
    for r in runs:
        meta = r.get('metadata', {})
        s = r.get('summary', {})
        row = {
            'run': run_label(r),
            'scenario': meta.get('scenario', '?'),
            'pattern': meta.get('pattern', '?'),
            'duration_s': round(meta.get('duration_s', 0.0), 1),
        }
        for canonical_key, display, digits, default in SUMMARY_METRICS:
            value = s.get(canonical_key, default)
            if digits is not None and isinstance(value, (int, float)):
                value = round(value, digits)
            row[display] = value
        out.append(row)
    return out


# Additional figure builders.
#
# Each builder takes the same inputs as the existing make_*_figure
# functions (a list of run dicts), so they slot directly into the
# Compare Runs metric dropdown. They MUST return a Figure even when
# data is missing: empty plots with a "no data" annotation, never
# exceptions, so the GUI doesn't crash on a malformed JSONL.


from .figures.trajectory_heatmap import (  # noqa: E402
    render as make_trajectory_heatmap_figure,
)
from .figures.latency_cdf import render as make_latency_cdf_figure  # noqa: E402
from .figures.survival_curve import render as make_survival_curve_figure  # noqa: E402
from .figures.threshold_roc import render as make_threshold_roc_figure  # noqa: E402
from .figures.pattern_boxplots import (  # noqa: E402
    render as make_pattern_boxplots_figure,
)


# Sweep loading + statistical aggregation.

def load_sweep(runs_dir: Path | str,
               manifest_only: bool = False) -> List[Dict]:
    """Back-compat shim; delegates to
    ``drone_rescue_mission_control.sweep.aggregator.load_sweep``."""
    from .sweep.aggregator import load_sweep as _load_sweep
    return _load_sweep(runs_dir, manifest_only=manifest_only)


# SweepAggregator moved to `sweep/aggregator.py`.
# `SWEEP_METRICS_DEFAULT` and `aggregate_sweep` are re-exported here
# as back-compat shims so call sites that still
# `from . import analytics; analytics.aggregate_sweep(...)` keep
# working. New code should import from
# `drone_rescue_mission_control.sweep` directly.
from .sweep.aggregator import (
    SWEEP_METRICS_DEFAULT,
    aggregate_runs as _aggregate_runs,
)


def aggregate_sweep(
    runs: List[Dict],
    group_by: Tuple[str, ...] = ('pattern', 'allocation_strategy', 'scenario'),
    metrics: Tuple[str, ...] = SWEEP_METRICS_DEFAULT,
    n_bootstrap: int = 10_000,
    rng_seed: int = 0,
) -> List[Dict[str, Any]]:
    """Back-compat shim; delegates to
    ``drone_rescue_mission_control.sweep.aggregator.aggregate_runs``
    and renders the rows as the legacy `<metric>_mean` /
    `<metric>_ci95_lo` flat dict shape. New code should consume the
    typed `SweepAggregateRow` VOs directly."""
    rows = _aggregate_runs(
        runs, group_by=group_by, metrics=metrics,
        n_bootstrap=n_bootstrap, rng_seed=rng_seed,
    )
    out: List[Dict[str, Any]] = [r.as_dict(group_by) for r in rows]
    out.sort(key=lambda r: tuple(str(r.get(k, '')) for k in group_by))
    return out


#: Minimum samples per group below which Welch's t-test is essentially
#: powerless (df=2 means only |t|>4.3 yields p<0.05, so genuine differences
#: usually fail to reject). Callers can lower this if they know what
#: they're doing, but the report layer keeps the default.
# Welch t-test moved to `lib/stats/welch.py`. Module re-exports the
# symbols so existing call sites (`analytics.t_test`,
# `analytics.T_TEST_MIN_N`) keep working until the consumers
# (report.py mainly) cut over directly.
from .stats.welch import T_TEST_MIN_N, t_test
from .stats import welch as _welch
_student_t_two_sided_p = _welch._student_t_two_sided_p
_regularized_incomplete_beta = _welch._regularized_incomplete_beta
_betacf = _welch._betacf
