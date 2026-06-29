"""Persistence layer: RunSummary VO and RunRepository port.

- ``RunSummary`` is the typed shadow of one mission_recorder JSONL.
  Knows its schema_version; ``RunSummary.from_jsonl(path)`` upgrades
  V4 to V5 fields silently so consumers (analytics, report, GUI widgets)
  read typed attributes instead of indexing raw dicts by string keys.
- ``RunRepository`` is the Protocol for "where do runs live?". The
  default ``JsonlRunRepository`` reads/writes one file per run on the
  filesystem; a test fake or an in-memory fixture can implement the
  same Protocol.

This package is pure-Python (no rclpy, no Qt). Consumed today by
``analytics.load_run`` (transitionally), and target consumed by
``report.py``, ``widgets/past_runs_tab.py``, ``widgets/compare_tab.py``,
``widgets/sweep_tab.py``, and ``mission_recorder._finalize`` once the
migration completes.
"""

from .run_summary import (
    RunSummary, RunMetadata, RunMetrics, TimeSeries, RunHandle,
    SCHEMA_VERSION_LATEST, MIN_SUPPORTED_SCHEMA,
)
from .run_repository import RunRepository, JsonlRunRepository
from .scenario_repository import (
    InMemoryScenarioRepository, ScenarioRepository, YamlScenarioRepository,
)

__all__ = [
    'RunSummary', 'RunMetadata', 'RunMetrics', 'TimeSeries', 'RunHandle',
    'SCHEMA_VERSION_LATEST', 'MIN_SUPPORTED_SCHEMA',
    'RunRepository', 'JsonlRunRepository',
    'ScenarioRepository', 'YamlScenarioRepository',
    'InMemoryScenarioRepository',
]
