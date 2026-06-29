"""Trajectory heatmap.

Consumes typed ``RunSummary``."""

from __future__ import annotations

from typing import List, Sequence

from matplotlib.figure import Figure

from .common import empty, run_label
from .protocol import _FunctionRenderer


def render(runs: Sequence) -> Figure:
    """2D histogram of cumulative drone positions, one subplot per run."""
    fig = Figure(
        figsize=(8, max(3.0, 4.0 * max(len(runs), 1))),
        tight_layout=True,
    )
    if not runs:
        ax = fig.add_subplot(111)
        empty(ax, 'no runs selected')
        return fig
    from matplotlib.colors import LogNorm
    for ri, r in enumerate(runs):
        ax = fig.add_subplot(len(runs), 1, ri + 1, aspect='equal')
        per_drone = r.time_series.per_drone
        xs: List[float] = []
        ys: List[float] = []
        for v in per_drone.values():
            for sample in v.get('position') or []:
                if len(sample) >= 3:
                    xs.append(float(sample[1]))
                    ys.append(float(sample[2]))
        if not xs:
            empty(ax, 'no position data (V4 JSONL?)')
            continue
        # Bound from BOTH the flown positions and the ground-truth victims, so
        # outlying samples (drones reach ~70 m) are not clipped out of the
        # fixed histogram range.
        bound = 10.0
        for gt in r.metadata.ground_truth_victims:
            pos = gt.get('position') or [0, 0, 0]
            bound = max(bound, abs(float(pos[0])) + 10, abs(float(pos[1])) + 10)
        bound = max(bound,
                    max(abs(x) for x in xs) + 5,
                    max(abs(y) for y in ys) + 5)
        # Log colour scale + drop empty cells (cmin=1): the drones hover at the
        # origin (one cell with a huge visit count), which on a linear scale
        # saturates the colormap and blacks out the whole sparse flight path.
        # LogNorm spreads the 1..N counts so the actual trajectories show.
        h = ax.hist2d(
            xs, ys, bins=60,
            range=[[-bound, bound], [-bound, bound]],
            cmap='magma', cmin=1, norm=LogNorm(),
        )
        for gt in r.metadata.ground_truth_victims:
            pos = gt.get('position') or [0, 0, 0]
            ax.plot(
                float(pos[0]), float(pos[1]), marker='*',
                color='#22d3ee', markersize=10, markeredgecolor='black',
            )
        ax.set_xlim(-bound, bound)
        ax.set_ylim(-bound, bound)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title(run_label(r))
        fig.colorbar(h[3], ax=ax, fraction=0.046, pad=0.04, label='visits')
    return fig


Renderer = _FunctionRenderer(
    'trajectory_heatmap', 'Trajectory heatmap', render,
)
