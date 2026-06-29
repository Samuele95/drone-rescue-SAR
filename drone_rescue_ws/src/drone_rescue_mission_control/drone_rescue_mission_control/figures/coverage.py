"""Coverage % over time, one file per figure.

Consumes typed ``RunSummary``."""

from __future__ import annotations

from typing import Sequence

import numpy as np
from matplotlib.figure import Figure

from .common import RUN_COLORS, empty, run_label
from .protocol import _FunctionRenderer


def render(runs: Sequence) -> Figure:
    fig = Figure(figsize=(8, 4.5), tight_layout=True)
    ax = fig.add_subplot(111)
    if not runs:
        empty(ax, 'no runs selected')
        return fig
    for i, r in enumerate(runs):
        ts = r.time_series.coverage_pct
        if not ts:
            continue
        arr = np.asarray(ts)
        xs = arr[:, 0]
        ys = arr[:, 1]
        ax.plot(xs, ys, label=run_label(r),
                color=RUN_COLORS[i % len(RUN_COLORS)], linewidth=1.6)
    ax.set_xlabel('time since survey start (s)')
    ax.set_ylabel('coverage (%)')
    ax.set_ylim(0, 100)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize=8)
    ax.set_title('Coverage % over time')
    return fig


Renderer = _FunctionRenderer(
    'coverage_over_time', 'Coverage over time', render,
)
