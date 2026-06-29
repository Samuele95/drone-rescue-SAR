"""Detection latency CDF.

Consumes typed ``RunSummary``."""

from __future__ import annotations

from typing import Sequence

from matplotlib.figure import Figure

from .common import RUN_COLORS, empty, run_label
from .protocol import _FunctionRenderer


def render(runs: Sequence) -> Figure:
    """Empirical CDF of per-victim detection latency, one curve per run."""
    fig = Figure(figsize=(8, 4.5), tight_layout=True)
    ax = fig.add_subplot(111)
    if not runs:
        empty(ax, 'no runs selected')
        return fig
    plotted_any = False
    for i, r in enumerate(runs):
        latencies = r.summary.detection_latency_per_victim_s
        if not latencies:
            continue
        values = sorted(float(x) for x in latencies)
        # Prepend the first value at y=0 and draw point markers so the curve is
        # a visible step even with a single victim (otherwise step() renders
        # one invisible dot at the right edge and the chart looks empty).
        xs = [values[0]] + values
        ys = [0.0] + [(j + 1) / len(values) for j in range(len(values))]
        ax.step(
            xs, ys, where='post', marker='o', markersize=4,
            label=run_label(r),
            color=RUN_COLORS[i % len(RUN_COLORS)], linewidth=1.6,
        )
        plotted_any = True
    if not plotted_any:
        empty(
            ax,
            'no detection_latency_per_victim_s in any run\n'
            '(V4 JSONL? requires V5 recorder)',
        )
        return fig
    ax.set_xlabel('per-victim detection latency (s)')
    ax.set_ylabel('cumulative fraction of victims')
    ax.set_ylim(0, 1.05)
    ax.set_xlim(left=0)   # anchor at t=0 so a single-victim point has context
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize=8)
    ax.set_title('Detection latency CDF')
    return fig


Renderer = _FunctionRenderer(
    'detection_latency_cdf', 'Detection latency CDF', render,
)
