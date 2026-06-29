"""Sweep loading + aggregation subdomain.

Extracted from ``analytics.py``. Sweeps are a distinct Mission Control
sub-concern (one sweep is N trials over a pattern-by-scenario matrix);
the bootstrap aggregation produces typed rows the report layer pages
through. Keeping the math here means analytics.py stops being the
god-module.
"""

from .aggregator import (
    SWEEP_METRICS_DEFAULT,
    SweepAggregateRow,
    SweepMetricCell,
    aggregate_runs,
    load_sweep,
)

__all__ = [
    'SWEEP_METRICS_DEFAULT',
    'SweepAggregateRow',
    'SweepMetricCell',
    'aggregate_runs',
    'load_sweep',
]
