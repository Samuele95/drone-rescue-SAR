"""Cumulative confirmed victims.

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
        ts = r.time_series.cumulative_confirmed
        if not ts:
            continue
        arr = np.asarray(ts)
        xs = arr[:, 0]
        ys = arr[:, 1]
        ax.step(xs, ys, where='post', label=run_label(r),
                color=RUN_COLORS[i % len(RUN_COLORS)], linewidth=1.6)
    ax.set_xlabel('time since survey start (s)')
    ax.set_ylabel('confirmed victims (cumulative)')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize=8)
    ax.set_title('Cumulative confirmed victims')
    return fig


Renderer = _FunctionRenderer(
    'cumulative_confirmed', 'Cumulative confirmed victims', render,
)
