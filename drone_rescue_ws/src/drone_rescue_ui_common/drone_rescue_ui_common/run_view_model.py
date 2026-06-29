"""RunViewModel: the post-hoc analogue of MissionViewModel.

Completes the bounded-context symmetry the project committed to: the
operator UI is one bounded context with two surfaces, live (dashboard)
and post-hoc (Mission Control). The live surface folds
peer/health/event streams through `MissionViewModel.apply_*`; the
post-hoc surface folds a typed `RunSummary` through
`RunViewModel.apply`.

Today's scope: the projection the four Mission Control widgets
need, a `RunRow` per loaded run (the Past Runs table shape) and
the typed metrics dict the Compare / Sweep widgets consume. The
reducer is pure: `RunViewModel().apply(run_summary) -> RunViewModel`
returns a new VO; the legacy raw-dict consumers can migrate
one widget at a time.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Mapping, Tuple


@dataclass(frozen=True)
class RunRow:
    """One row in the Past Runs / Compare Runs tables. Matches the
    `_SUMMARY_COLS` layout the legacy widgets read out of raw dicts."""
    run_label: str
    scenario: str
    pattern: str
    duration_s: float = 0.0
    final_coverage_pct: float = 0.0
    true_positives: int = 0
    false_positives: int = 0
    false_negatives: int = 0
    victims_confirmed: int = 0
    time_to_first_detection_s: float = 0.0
    time_to_first_confirm_s: float = 0.0
    drones_down: int = 0
    sector_reassignments: int = 0
    energy_per_coverage_pct_J: float = 0.0
    started_at: str = ''


@dataclass(frozen=True)
class RunViewModel:
    """Operator post-hoc projection. Folded via `apply(run_summary)`.

    Stores both the latest `RunRow` and an `Any`-typed `summary_dict`
    field for the metric figures that haven't yet migrated off the
    legacy dict shape. As future improvements migrate those
    consumers, the `summary_dict` field can retire.
    """
    rows: Tuple[RunRow, ...] = ()

    def apply(self, run: Any) -> 'RunViewModel':
        """Fold a `RunSummary` into the view model.

        Accepts either a `RunSummary` VO or the legacy dict shape
        (so widgets can migrate one-by-one). Field extraction goes
        through attribute access first, falls back to dict get.
        """
        def _attr(obj: Any, name: str, default: Any) -> Any:
            if hasattr(obj, name):
                return getattr(obj, name)
            if isinstance(obj, dict):
                return obj.get(name, default)
            return default

        if hasattr(run, 'metadata') and hasattr(run, 'metrics'):
            meta = run.metadata
            metrics = run.metrics
            row = RunRow(
                run_label=_attr(meta, 'run_label', ''),
                scenario=_attr(meta, 'scenario', '?'),
                pattern=_attr(meta, 'pattern', '?'),
                duration_s=float(_attr(meta, 'duration_s', 0.0) or 0.0),
                started_at=_attr(meta, 'started_at', ''),
                final_coverage_pct=float(_attr(metrics, 'final_coverage_pct', 0.0) or 0.0),
                true_positives=int(_attr(metrics, 'true_positives', 0) or 0),
                false_positives=int(_attr(metrics, 'false_positives', 0) or 0),
                false_negatives=int(_attr(metrics, 'false_negatives', 0) or 0),
                victims_confirmed=int(_attr(metrics, 'victims_confirmed', 0) or 0),
                time_to_first_detection_s=float(_attr(metrics, 'time_to_first_detection_s', 0.0) or 0.0),
                time_to_first_confirm_s=float(_attr(metrics, 'time_to_first_confirm_s', 0.0) or 0.0),
                drones_down=int(_attr(metrics, 'drones_down', 0) or 0),
                sector_reassignments=int(_attr(metrics, 'sector_reassignments', 0) or 0),
                energy_per_coverage_pct_J=float(_attr(metrics, 'energy_per_coverage_pct_J', 0.0) or 0.0),
            )
        else:
            # Legacy raw-dict shape.
            meta = run.get('metadata', {}) if isinstance(run, dict) else {}
            summ = run.get('summary', {}) if isinstance(run, dict) else {}
            row = RunRow(
                run_label='',
                scenario=meta.get('scenario', '?'),
                pattern=meta.get('pattern', '?'),
                duration_s=float(meta.get('duration_s', 0.0) or 0.0),
                started_at=meta.get('started_at', ''),
                final_coverage_pct=float(summ.get('final_coverage_pct', 0.0) or 0.0),
                true_positives=int(summ.get('true_positives', 0) or 0),
                false_positives=int(summ.get('false_positives', 0) or 0),
                false_negatives=int(summ.get('false_negatives', 0) or 0),
                victims_confirmed=int(summ.get('victims_confirmed', 0) or 0),
                time_to_first_detection_s=float(summ.get('time_to_first_detection_s', 0.0) or 0.0),
                time_to_first_confirm_s=float(summ.get('time_to_first_confirm_s', 0.0) or 0.0),
                drones_down=int(summ.get('drones_down', 0) or 0),
                sector_reassignments=int(summ.get('sector_reassignments', 0) or 0),
                energy_per_coverage_pct_J=float(summ.get('energy_per_coverage_pct_J', 0.0) or 0.0),
            )

        return replace(self, rows=self.rows + (row,))

    def with_rows(self, rows: Tuple[RunRow, ...]) -> 'RunViewModel':
        """Replace the rows tuple wholesale, used when reloading
        the Past Runs table from a list."""
        return replace(self, rows=tuple(rows))
