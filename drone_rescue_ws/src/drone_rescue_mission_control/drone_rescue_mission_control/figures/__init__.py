"""FigureRenderer registry: single source of truth for the
Mission Control figure builders.

Replaces the hardcoded `(label, builder)` lists that duplicated
themselves across `report.py` and `compare_tab.py`. Each figure
registers itself with a `name` (stable id), a `label`
(operator-facing string), and a `render(runs) -> Figure` callable.
The two consumers iterate `registry.renderers()` instead of
maintaining parallel lists.
"""

from .protocol import FigureRenderer
from .registry import register, renderers, get_renderer

__all__ = [
    'FigureRenderer',
    'register',
    'renderers',
    'get_renderer',
]
