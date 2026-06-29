"""FigureRenderer registry.

Module-level registry that the consumers iterate. Order is
preserved so `report.py`'s PDF page order matches the legacy
hardcoded list. Adding a 10th figure: add the import + one
`_register(_FunctionRenderer(...))` call below.

The registry pre-populates itself at import time with the 9 figure
builders that were previously hardcoded in `report.py` and
`compare_tab.py`. Each one delegates to the corresponding
`analytics.make_*_figure` function until the renderers move out of
analytics altogether.
"""

from __future__ import annotations

from typing import Dict, List, Optional

from .protocol import FigureRenderer, _FunctionRenderer


# Ordered registry. List-insertion order = display order.
_REGISTRY: List[FigureRenderer] = []
_BY_NAME: Dict[str, FigureRenderer] = {}


def register(renderer: FigureRenderer) -> None:
    """Append a renderer to the ordered registry. The renderer's
    `name` must be unique within the process."""
    if renderer.name in _BY_NAME:
        raise ValueError(
            f'Figure renderer {renderer.name!r} already registered'
        )
    _REGISTRY.append(renderer)
    _BY_NAME[renderer.name] = renderer


def renderers() -> List[FigureRenderer]:
    """Return the ordered list of registered renderers. Defensive
    copy so callers can iterate without holding the registry's list
    by reference."""
    return list(_REGISTRY)


def get_renderer(name: str) -> Optional[FigureRenderer]:
    return _BY_NAME.get(name)


# Bootstrap: import each per-file figure module so its module-level
# ``Renderer = _FunctionRenderer(...)`` export is constructed. Order
# matches the legacy `report.py` page sequence; the PDF page order is
# pinned by this list. Each builder lives in its own
# ``figures/<name>.py``. Adding a 9th figure: new module + one
# ``register()`` line below.

def _bootstrap() -> None:
    from . import (
        coverage, cumulative_confirmed, per_drone_battery,
        task_histogram, trajectory_heatmap, latency_cdf,
        survival_curve, threshold_roc,
    )
    for module in (
        coverage, cumulative_confirmed, per_drone_battery,
        task_histogram, trajectory_heatmap, latency_cdf,
        survival_curve, threshold_roc,
    ):
        register(module.Renderer)


_bootstrap()
