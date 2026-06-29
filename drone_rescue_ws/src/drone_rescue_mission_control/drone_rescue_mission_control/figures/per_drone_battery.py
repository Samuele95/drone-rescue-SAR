"""Per-drone battery.

Consumes typed ``RunSummary`` (per_drone inner records remain
dict-shaped; see ``TimeSeries.per_drone`` forward-compat note in
run_summary.py)."""

from __future__ import annotations

from typing import Sequence

import numpy as np
from matplotlib.figure import Figure

from .common import RUN_COLORS, empty, run_label
from .protocol import _FunctionRenderer


def render(runs: Sequence) -> Figure:
    fig = Figure(
        figsize=(8, max(2.5, 2.4 * max(len(runs), 1))),
        tight_layout=True,
    )
    if not runs:
        ax = fig.add_subplot(111)
        empty(ax, 'no runs selected')
        return fig
    for ri, r in enumerate(runs):
        ax = fig.add_subplot(len(runs), 1, ri + 1)
        per_drone = r.time_series.per_drone
        drones = sorted(per_drone.keys())
        for di, dname in enumerate(drones):
            series = per_drone[dname].get('battery') or []
            if not series:
                continue
            arr = np.asarray(series)
            xs = arr[:, 0]
            ys = arr[:, 1] * 100   # 0..1 to %
            ax.plot(xs, ys, label=dname,
                    color=RUN_COLORS[di % len(RUN_COLORS)], linewidth=1.4)
        ax.set_ylabel('battery %')
        ax.set_ylim(0, 100)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='lower left', fontsize=7, ncol=4)
        ax.set_title(run_label(r))
    fig.axes[-1].set_xlabel('time since survey start (s)')
    return fig


Renderer = _FunctionRenderer(
    'per_drone_battery', 'Per-drone battery', render,
)
