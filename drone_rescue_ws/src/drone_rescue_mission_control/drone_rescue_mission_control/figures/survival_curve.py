"""Victim survival curve (Kaplan-Meier-style).

Consumes typed ``RunSummary``."""

from __future__ import annotations

from typing import Sequence

from matplotlib.figure import Figure

from .common import RUN_COLORS, empty, run_label
from .protocol import _FunctionRenderer


def render(runs: Sequence) -> Figure:
    fig = Figure(figsize=(8, 4.5), tight_layout=True)
    ax = fig.add_subplot(111)
    if not runs:
        empty(ax, 'no runs selected')
        return fig
    plotted_any = False
    for i, r in enumerate(runs):
        ts = r.time_series.cumulative_confirmed
        gt_n = len(r.metadata.ground_truth_victims)
        if gt_n == 0 or not ts:
            continue
        xs = [float(t) for t, _ in ts]
        ys = [max(0.0, 1.0 - float(c) / gt_n) for _, c in ts]
        if xs[0] > 0:
            xs = [0.0] + xs
            ys = [1.0] + ys
        ax.step(
            xs, ys, where='post', label=run_label(r),
            color=RUN_COLORS[i % len(RUN_COLORS)], linewidth=1.6,
        )
        plotted_any = True
    if not plotted_any:
        empty(ax, 'no survival data')
        return fig
    ax.set_xlabel('time since survey start (s)')
    ax.set_ylabel('victims still unfound (fraction)')
    ax.set_ylim(-0.02, 1.05)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title('Victim discovery survival curve (Kaplan-Meier)')
    return fig


Renderer = _FunctionRenderer(
    'victim_survival_curve', 'Victim survival curve', render,
)
