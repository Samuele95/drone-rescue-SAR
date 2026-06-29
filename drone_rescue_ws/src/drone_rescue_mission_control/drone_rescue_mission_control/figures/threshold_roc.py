"""Detection threshold ROC sweep.

Consumes typed ``RunSummary``."""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np
from matplotlib.figure import Figure

from .common import RUN_COLORS, empty, run_label
from .protocol import _FunctionRenderer


def render(runs: Sequence) -> Figure:
    """ROC-like curve: trades off true-positive rate vs false-positive
    count as the confirmation threshold varies. Built from the
    candidate-event log + the run's ground truth.
    """
    fig = Figure(figsize=(8, 4.5), tight_layout=True)
    ax = fig.add_subplot(111)
    if not runs:
        empty(ax, 'no runs selected')
        return fig
    plotted_any = False
    for i, r in enumerate(runs):
        events = r.events
        gt_pts = [
            (float(g['position'][0]), float(g['position'][1]))
            for g in r.metadata.ground_truth_victims
        ]
        if not gt_pts:
            continue
        cands: List[Tuple[float, float, float]] = []
        for e in events:
            if e.get('type') != 'CANDIDATE_DETECTED':
                continue
            conf_val = e.get('confidence')
            if conf_val is None or float(conf_val) <= 0.0:
                detail = (e.get('detail') or '')
                conf = None
                for tok in detail.split(','):
                    tok = tok.strip()
                    if tok.startswith('conf='):
                        try:
                            conf = float(tok.split('=', 1)[1])
                        except ValueError:
                            pass
            else:
                conf = float(conf_val)
            pos = e.get('position') or [0, 0, 0]
            if conf is not None and len(pos) >= 2:
                cands.append((conf, float(pos[0]), float(pos[1])))
        if not cands:
            continue
        cands.sort(reverse=True)
        radius2 = 8.0 ** 2
        tps: List[int] = []
        fps: List[int] = []
        bound: set = set()
        tp = 0
        fp = 0
        gt_np = np.asarray(gt_pts, dtype=np.float64) if gt_pts else None
        for conf, cx, cy in cands:
            best_g = None
            best_d = float('inf')
            if gt_np is not None and gt_np.size > 0:
                d_sq = ((gt_np - np.array([cx, cy])) ** 2).sum(axis=1)
                if bound:
                    d_sq = d_sq.copy()
                    for j_bound in bound:
                        if 0 <= j_bound < d_sq.shape[0]:
                            d_sq[j_bound] = np.inf
                j_min = int(np.argmin(d_sq))
                d_min = float(d_sq[j_min])
                if np.isfinite(d_min):
                    best_g = j_min
                    best_d = d_min
            if best_g is not None and best_d <= radius2:
                bound.add(best_g)
                tp += 1
            else:
                fp += 1
            tps.append(tp)
            fps.append(fp)
        n_gt = len(gt_pts)
        tpr = [t / max(n_gt, 1) for t in tps]
        ax.step(
            fps, tpr, where='post', label=run_label(r),
            color=RUN_COLORS[i % len(RUN_COLORS)], linewidth=1.6,
        )
        plotted_any = True
    if not plotted_any:
        empty(ax, 'no candidate events with confidence')
        return fig
    ax.set_xlabel('cumulative false positives')
    ax.set_ylabel('true-positive rate (TP / |GT|)')
    ax.set_ylim(-0.02, 1.05)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize=8)
    ax.set_title('Detection threshold ROC sweep')
    return fig


Renderer = _FunctionRenderer(
    'detection_threshold_roc', 'Detection threshold ROC', render,
)
