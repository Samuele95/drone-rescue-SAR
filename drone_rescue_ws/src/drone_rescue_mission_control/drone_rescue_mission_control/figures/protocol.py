"""FigureRenderer Protocol.

The driver port for rendering a list of runs into a matplotlib
`Figure`. Implementations register with the module-level registry in
`drone_rescue_mission_control.figures.registry`. Both `report.py`
(which paginates a PDF) and `compare_tab.py` (which populates a
dropdown) iterate the registry instead of maintaining parallel
hardcoded lists.

The Protocol contract is tightened to ``Sequence[RunSummary]``.
``_FunctionRenderer`` accepts both raw dicts and ``RunSummary`` for
one cutover commit; new code paths pass ``RunSummary`` directly. The
internal renderer functions consume typed attributes (e.g.
``run.time_series.coverage_pct``) instead of ``r.get('time_series',
{}).get('coverage_pct')`` raw-dict walks.
"""

from __future__ import annotations

from typing import Any, Callable, Protocol, Sequence

# Matplotlib Figure is the return type; typed as Any to avoid making
# this Protocol pull matplotlib at import time. Implementations
# explicitly return a `matplotlib.figure.Figure`.


class FigureRenderer(Protocol):
    """A registered figure builder.

    Attributes:
        name: Stable identifier (used as registry key). Lowercase
              snake_case is conventional.
        label: Operator-facing display string. Same vocabulary the
              legacy hardcoded list used so PDF/dropdown labels stay
              identical post-cutover.

    Method:
        render(runs): take a sequence of ``RunSummary`` VOs and return
              a matplotlib Figure. ``_FunctionRenderer`` normalises
              raw dicts to ``RunSummary`` via ``RunSummary.from_dict``
              for back-compat with the one or two callers still
              passing dict views; new consumers pass ``RunSummary``
              directly.
    """

    name: str
    label: str

    def render(self, runs: Sequence['RunSummary']) -> Any: ...


class _FunctionRenderer:
    """Adapter: wraps a pure ``(runs: Sequence[RunSummary]) -> Figure``
    function as a FigureRenderer.

    Accepts both ``RunSummary`` and dict inputs to keep the one
    cutover commit safe; raw dicts are upgraded via
    ``RunSummary.from_dict``.
    """

    __slots__ = ('name', 'label', '_fn')

    def __init__(self, name: str, label: str, fn: Callable):
        self.name = name
        self.label = label
        self._fn = fn

    def render(self, runs: Sequence[Any]) -> Any:
        from ..persistence.run_summary import RunSummary
        typed = []
        for r in runs:
            if isinstance(r, RunSummary):
                typed.append(r)
            elif isinstance(r, dict):
                typed.append(RunSummary.from_dict(r))
            else:
                # Defensive: pass through unknown shapes. The renderer
                # body will fail with AttributeError if the input
                # genuinely doesn't satisfy the contract.
                typed.append(r)
        return self._fn(typed)
