"""Shared helpers for the per-file figure modules.

Lifted out of ``analytics.py`` so each ``figures/<name>.py`` module
is self-contained. ``run_label`` consumes the typed ``RunSummary``.
"""

from __future__ import annotations

from typing import List


# Stable colour cycle so the Nth run gets the same colour across plots.
RUN_COLORS: List[str] = [
    '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
    '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf',
]


def empty(ax, text: str) -> None:
    """Stamp an empty axes with an explanatory message, used when
    the input runs have no data for the figure's axis."""
    ax.text(0.5, 0.5, text, ha='center', va='center',
            transform=ax.transAxes, color='#64748b')
    ax.set_xticks([])
    ax.set_yticks([])


def run_label(run) -> str:
    """Short human-readable label for a run, used in legends.

    ``run`` is a ``RunSummary`` VO."""
    meta = run.metadata
    when = meta.started_at[:19].replace('T', ' ')
    return f'{meta.scenario} / {meta.pattern}  [{when}]'
