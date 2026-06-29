"""Sweep aggregation: mean / std / bootstrap CI per (group_key,
metric) cell.

Extracted from ``analytics.py``. Emits typed rows so consumers
(`report.py`, `sweep_tab.py`) stop reading `r.get(f'{metric}_mean')`
f-string-interpolated keys.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np


# re-exported here so consumers don't reach into analytics.py for the
# metric registry. SUMMARY_METRICS still lives in analytics for now since
# `summarise()` consumes it for the Compare-Runs table; this module just
# unions in its sweep extras.
def _summary_canonical_keys() -> Tuple[str, ...]:
    from .. import analytics  # late import to avoid cycle
    return tuple(canonical for canonical, _, _, _ in analytics.SUMMARY_METRICS)


_SWEEP_EXTRA_METRICS: Tuple[str, ...] = (
    'precision', 'recall', 'f1_score',
    'time_to_coverage_50pct_s', 'time_to_coverage_80pct_s',
    'time_to_coverage_90pct_s',
    'energy_per_coverage_pct_J', 'task_fairness_jain',
)


def _build_default_metrics() -> Tuple[str, ...]:
    return tuple(dict.fromkeys(
        list(_summary_canonical_keys()) + list(_SWEEP_EXTRA_METRICS)
    ))


# typed VO for per-cell aggregation results. Replaces the
# f-string-keyed dict (`row[f'{metric}_mean']`).
@dataclass(frozen=True)
class SweepMetricCell:
    """One (group_key, metric) aggregation cell."""
    mean: Optional[float]
    std: Optional[float]
    ci95_lo: Optional[float]
    ci95_hi: Optional[float]
    n: int


@dataclass(frozen=True)
class SweepAggregateRow:
    """One group in the sweep, typically `(pattern, scenario)`.

    `group_keys` is the ordered tuple of `group_by` values
    (`('spiral_out', 'rural_dense')` etc.). `metrics` is keyed by
    canonical metric name (e.g. `'final_coverage_pct'`).
    """
    group_keys: Tuple[Any, ...]
    n_trials: int
    metrics: Mapping[str, SweepMetricCell] = field(default_factory=dict)

    def as_dict(self, group_by: Sequence[str]) -> Dict[str, Any]:
        """Render as the legacy `aggregate_sweep`-shaped dict so the
        existing report-layer consumers (`<metric>_mean`,
        `<metric>_ci95_lo`, ...) keep working during the cutover.
        """
        out: Dict[str, Any] = {}
        for name, value in zip(group_by, self.group_keys):
            out[name] = value
        out['n_trials'] = self.n_trials
        for metric_name, cell in self.metrics.items():
            out[f'{metric_name}_mean'] = cell.mean
            out[f'{metric_name}_std'] = cell.std
            out[f'{metric_name}_ci95_lo'] = cell.ci95_lo
            out[f'{metric_name}_ci95_hi'] = cell.ci95_hi
            out[f'{metric_name}_n'] = cell.n
        return out


SWEEP_METRICS_DEFAULT: Tuple[str, ...] = _build_default_metrics()


def load_sweep(runs_dir: Path | str,
               manifest_only: bool = False) -> List[Dict]:
    """Load every run JSONL inside a sweep directory.

    Same contract as the legacy `analytics.load_sweep`; moved here
    to keep sweep concerns colocated.
    """
    from .. import analytics  # for load_run
    runs_dir = Path(runs_dir)
    if manifest_only:
        m = runs_dir / 'manifest.json'
        return [json.loads(m.read_text())] if m.is_file() else []
    out: List[Dict] = []
    for p in sorted(runs_dir.glob('*.json')):
        if p.name in ('manifest.json', 'report.json'):
            continue
        try:
            out.append(analytics.load_run(p))
        except Exception:
            continue
    return out


def aggregate_runs(
    runs: List[Dict],
    group_by: Tuple[str, ...] = ('pattern', 'allocation_strategy', 'scenario'),
    metrics: Tuple[str, ...] = SWEEP_METRICS_DEFAULT,
    n_bootstrap: int = 10_000,
    rng_seed: int = 0,
) -> List[SweepAggregateRow]:
    """Group runs by `group_by` keys (read from `metadata`) and
    compute mean / std / 95% percentile-bootstrap CI / n per metric.

    Returns a list of typed `SweepAggregateRow` VOs. Bootstrap is
    seeded so the aggregation is reproducible.
    """
    rng = np.random.default_rng(rng_seed)
    groups: Dict[Tuple[Any, ...], List[Dict]] = {}
    for r in runs:
        meta = r.get('metadata', {})
        key = tuple(meta.get(k, '?') for k in group_by)
        groups.setdefault(key, []).append(r)

    out: List[SweepAggregateRow] = []
    for key, group_runs in groups.items():
        cells: Dict[str, SweepMetricCell] = {}
        for m in metrics:
            values = [r.get('summary', {}).get(m) for r in group_runs]
            values = [float(v) for v in values if v is not None]
            if not values:
                cells[m] = SweepMetricCell(None, None, None, None, 0)
                continue
            arr = np.array(values, dtype=float)
            mean = float(arr.mean())
            std = float(arr.std(ddof=1)) if len(arr) > 1 else 0.0
            n = int(len(arr))
            if len(arr) >= 2:
                idxs = rng.integers(0, len(arr), size=(n_bootstrap, len(arr)))
                resampled_means = arr[idxs].mean(axis=1)
                lo, hi = np.percentile(resampled_means, [2.5, 97.5])
                ci_lo, ci_hi = float(lo), float(hi)
            else:
                ci_lo = ci_hi = float(arr[0])
            cells[m] = SweepMetricCell(mean, std, ci_lo, ci_hi, n)
        out.append(SweepAggregateRow(
            group_keys=key,
            n_trials=len(group_runs),
            metrics=cells,
        ))
    return out
