"""Per-drone task histogram.

Consumes typed ``RunSummary``."""

from __future__ import annotations

from typing import Sequence

import numpy as np
from matplotlib.figure import Figure

from drone_rescue_coordination.lib.domain.task_type import TaskType as _TaskType

from .common import empty, run_label
from .protocol import _FunctionRenderer


_TASK_NAMES = {int(t): t.label for t in _TaskType}
_TASK_COLORS = {
    0: '#3b82f6', 1: '#a855f7', 2: '#16a34a',
    3: '#f59e0b', 4: '#ef4444', 5: '#94a3b8',
}


def render(runs: Sequence) -> Figure:
    """Stacked bar of fraction-of-time spent per task type, per drone, per run."""
    fig = Figure(
        figsize=(8, max(2.5, 2.0 * max(len(runs), 1))),
        tight_layout=True,
    )
    if not runs:
        ax = fig.add_subplot(111)
        empty(ax, 'no runs selected')
        return fig
    for ri, r in enumerate(runs):
        ax = fig.add_subplot(len(runs), 1, ri + 1)
        per_drone_ts = r.time_series.per_drone
        drones = sorted(per_drone_ts.keys())
        if not drones:
            empty(ax, 'no per-drone data')
            continue
        per_drone = {}
        for dname in drones:
            series = per_drone_ts[dname].get('task') or []
            counts = {k: 0.0 for k in _TASK_NAMES.keys()}
            if len(series) >= 2:
                for j in range(len(series) - 1):
                    dt = series[j + 1][0] - series[j][0]
                    counts[int(series[j][1])] = (
                        counts.get(int(series[j][1]), 0.0) + dt
                    )
            total = sum(counts.values()) or 1.0
            per_drone[dname] = {k: v / total for k, v in counts.items()}
        task_keys = sorted(_TASK_NAMES.keys())
        data = np.array([
            [per_drone[d].get(tk, 0.0) for d in drones]
            for tk in task_keys
        ], dtype=float)
        baselines = np.vstack([
            np.zeros(len(drones)),
            np.cumsum(data, axis=0)[:-1],
        ])
        for i, tk in enumerate(task_keys):
            ax.bar(
                drones, data[i], bottom=baselines[i],
                label=_TASK_NAMES[tk], color=_TASK_COLORS[tk],
            )
        ax.set_ylabel('fraction of mission')
        ax.set_ylim(0, 1)
        ax.legend(loc='upper right', fontsize=7, ncol=6)
        ax.set_title(run_label(r))
    return fig


Renderer = _FunctionRenderer(
    'per_drone_task_histogram', 'Per-drone task histogram', render,
)
