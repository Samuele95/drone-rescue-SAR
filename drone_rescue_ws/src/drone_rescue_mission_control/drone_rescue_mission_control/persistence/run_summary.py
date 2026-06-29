"""RunSummary value object: typed shadow of one mission_recorder JSONL.

Replaces the implicit dict-by-string-key reads scattered through
``analytics.py``, ``report.py``, and ``widgets/*``. Schema versioning
lifts the ``if 'detection_latency_per_victim_s' in summary else ...``
branches into the loader so consumers see the typed attribute either
way (None when the V4 JSONL didn't carry the field).

Schema versions:
- 4 = V4 (the first JSONLs the recorder wrote: no precision/recall/F1,
      no time_to_coverage_*, no energy_per_*, no Jain fairness, no
      per-victim latency, no per-drone position).
- 5 = V5 (current: all the later metrics, plus per-drone position).

The loader detects the version by field presence (no explicit version
field in the legacy JSONL) and constructs a uniform RunSummary; the
``schema_version`` attribute is set so consumers can branch on it
where the difference is semantically meaningful.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


SCHEMA_VERSION_LATEST = 5
MIN_SUPPORTED_SCHEMA = 4


@dataclass(frozen=True)
class RunHandle:
    """Locator for a run: the canonical thing the GUI passes around
    to identify "this row in the past-runs table"."""
    path: Path
    name: str   # filename basename, used as the human-friendly label

    @staticmethod
    def from_path(path: Path) -> 'RunHandle':
        return RunHandle(path=path, name=path.name)


@dataclass(frozen=True)
class RunMetadata:
    """Per-run metadata block: scenario, pattern, timing, ground truth."""
    started_at: str = ''
    ended_at: str = ''
    duration_s: float = 0.0
    ended_by: str = 'UNKNOWN'
    scenario: str = '?'
    pattern: str = '?'
    ground_truth_victims: Tuple[Dict[str, Any], ...] = ()
    params_snapshot: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RunMetrics:
    """Per-run summary metrics. V5 fields are Optional so V4 JSONLs
    map to ``None`` rather than raising; consumers branch on the
    attribute being None where the V4 vs V5 distinction matters."""
    candidates_emitted: int = 0
    victims_confirmed: int = 0
    victims_rejected: int = 0
    true_positives: int = 0
    false_positives: int = 0
    false_negatives: int = 0
    matched_pairs: Tuple[Dict[str, Any], ...] = ()
    unmatched_confirmed_ids: Tuple[int, ...] = ()
    unmatched_ground_truth_ids: Tuple[int, ...] = ()
    time_to_first_detection_s: Optional[float] = None
    time_to_first_confirm_s: Optional[float] = None
    drones_down: int = 0
    drone_down_events: Tuple[Dict[str, Any], ...] = ()
    sector_reassignments: int = 0
    final_coverage_pct: float = 0.0
    # V5 additions (None on V4 JSONLs)
    precision: Optional[float] = None
    recall: Optional[float] = None
    f1_score: Optional[float] = None
    time_to_coverage_50pct_s: Optional[float] = None
    time_to_coverage_80pct_s: Optional[float] = None
    time_to_coverage_90pct_s: Optional[float] = None
    energy_per_coverage_pct_J: Optional[float] = None
    task_fairness_jain: Optional[float] = None
    detection_latency_per_victim_s: Optional[Tuple[float, ...]] = None


@dataclass(frozen=True)
class TimeSeries:
    """Time-series block. Per-drone series are nested in ``per_drone``:
    a frozen mapping from drone_name to a frozen-mapping inner record."""
    coverage_pct: Tuple[Tuple[float, float], ...] = ()
    cumulative_confirmed: Tuple[Tuple[float, int], ...] = ()
    candidates_count: Tuple[Tuple[float, int], ...] = ()
    per_drone: Dict[str, Dict[str, Any]] = field(default_factory=dict)


@dataclass(frozen=True)
class RunSummary:
    """Typed shadow of one mission_recorder JSONL.

    Construct via ``RunSummary.from_jsonl(path)`` (production path) or
    ``RunSummary.from_dict(d)`` (in-memory / test). ``to_dict()``
    round-trips back to the wire-format dict so existing
    ``mission_recorder._finalize`` can keep emitting via ``json.dump``
    until that consumer is migrated too.
    """
    schema_version: int
    metadata: RunMetadata
    summary: RunMetrics
    time_series: TimeSeries
    events: Tuple[Dict[str, Any], ...]   # typed MissionEvent ADT lands later

    # ------------------------------------------------------------ loaders

    @staticmethod
    def from_jsonl(path: Path) -> 'RunSummary':
        """Load + validate one JSONL. Auto-detects V4 vs V5 by field
        presence and silently up-grades V4 fields to None where V5
        introduced new ones."""
        d = json.loads(Path(path).read_text())
        return RunSummary.from_dict(d)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'RunSummary':
        meta_raw = d.get('metadata', {})
        summ_raw = d.get('summary', {})
        ts_raw = d.get('time_series', {})

        # Detect schema version: V5 introduced precision/recall/F1; if
        # any is present, we're V5+. Otherwise V4. (No explicit
        # version field in the legacy JSONLs.)
        if 'precision' in summ_raw or 'f1_score' in summ_raw:
            schema_version = 5
        else:
            schema_version = 4

        if schema_version < MIN_SUPPORTED_SCHEMA:
            raise ValueError(
                f'unsupported run schema_version={schema_version}; '
                f'MIN_SUPPORTED_SCHEMA={MIN_SUPPORTED_SCHEMA}'
            )

        metadata = RunMetadata(
            started_at=str(meta_raw.get('started_at', '')),
            ended_at=str(meta_raw.get('ended_at', '')),
            duration_s=float(meta_raw.get('duration_s', 0.0)),
            ended_by=str(meta_raw.get('ended_by', 'UNKNOWN')),
            scenario=str(meta_raw.get('scenario', '?')),
            pattern=str(meta_raw.get('pattern', '?')),
            ground_truth_victims=tuple(meta_raw.get('ground_truth_victims') or ()),
            params_snapshot=dict(meta_raw.get('params_snapshot') or {}),
        )

        # Per-drone TimeSeries inner records: keep as raw dicts for now
        # (the per-drone fields evolve independently and we don't want
        # to lose forward-compat for keys we haven't typed yet).
        per_drone: Dict[str, Dict[str, Any]] = {}
        for k, v in ts_raw.items():
            if k.startswith('drone'):
                per_drone[k] = dict(v) if isinstance(v, dict) else {}
        time_series = TimeSeries(
            coverage_pct=tuple(tuple(p) for p in (ts_raw.get('coverage_pct') or ())),
            cumulative_confirmed=tuple(tuple(p) for p in (ts_raw.get('cumulative_confirmed') or ())),
            candidates_count=tuple(tuple(p) for p in (ts_raw.get('candidates_count') or ())),
            per_drone=per_drone,
        )

        # V5 metric fields default to None on V4; tuple(None) coercion
        # is handled per-field for the latency list.
        latency_raw = summ_raw.get('detection_latency_per_victim_s')
        if latency_raw is not None:
            latency = tuple(float(x) for x in latency_raw)
        else:
            latency = None

        summary = RunMetrics(
            candidates_emitted=int(summ_raw.get('candidates_emitted', 0)),
            victims_confirmed=int(summ_raw.get('victims_confirmed', 0)),
            victims_rejected=int(summ_raw.get('victims_rejected', 0)),
            true_positives=int(summ_raw.get('true_positives', 0)),
            false_positives=int(summ_raw.get('false_positives', 0)),
            false_negatives=int(summ_raw.get('false_negatives', 0)),
            matched_pairs=tuple(summ_raw.get('matched_pairs') or ()),
            unmatched_confirmed_ids=tuple(summ_raw.get('unmatched_confirmed_ids') or ()),
            unmatched_ground_truth_ids=tuple(summ_raw.get('unmatched_ground_truth_ids') or ()),
            time_to_first_detection_s=summ_raw.get('time_to_first_detection_s'),
            time_to_first_confirm_s=summ_raw.get('time_to_first_confirm_s'),
            drones_down=int(summ_raw.get('drones_down', 0)),
            drone_down_events=tuple(summ_raw.get('drone_down_events') or ()),
            sector_reassignments=int(summ_raw.get('sector_reassignments', 0)),
            final_coverage_pct=float(summ_raw.get('final_coverage_pct', 0.0)),
            # V5+ fields:
            precision=summ_raw.get('precision'),
            recall=summ_raw.get('recall'),
            f1_score=summ_raw.get('f1_score'),
            time_to_coverage_50pct_s=summ_raw.get('time_to_coverage_50pct_s'),
            time_to_coverage_80pct_s=summ_raw.get('time_to_coverage_80pct_s'),
            time_to_coverage_90pct_s=summ_raw.get('time_to_coverage_90pct_s'),
            energy_per_coverage_pct_J=summ_raw.get('energy_per_coverage_pct_J'),
            task_fairness_jain=summ_raw.get('task_fairness_jain'),
            detection_latency_per_victim_s=latency,
        )

        events = tuple(d.get('events') or ())

        return RunSummary(
            schema_version=schema_version,
            metadata=metadata,
            summary=summary,
            time_series=time_series,
            events=events,
        )

    # ------------------------------------------------------------ to-wire

    def to_dict(self) -> Dict[str, Any]:
        """Round-trip back to the JSONL wire format. Preserves
        ``mission_recorder``'s existing on-disk schema."""
        out: Dict[str, Any] = {
            'metadata': {
                'started_at': self.metadata.started_at,
                'ended_at': self.metadata.ended_at,
                'duration_s': self.metadata.duration_s,
                'ended_by': self.metadata.ended_by,
                'scenario': self.metadata.scenario,
                'pattern': self.metadata.pattern,
                'params_snapshot': dict(self.metadata.params_snapshot),
                'ground_truth_victims': list(self.metadata.ground_truth_victims),
            },
            'summary': {
                'candidates_emitted': self.summary.candidates_emitted,
                'victims_confirmed': self.summary.victims_confirmed,
                'victims_rejected': self.summary.victims_rejected,
                'true_positives': self.summary.true_positives,
                'false_positives': self.summary.false_positives,
                'false_negatives': self.summary.false_negatives,
                'matched_pairs': list(self.summary.matched_pairs),
                'unmatched_confirmed_ids': list(self.summary.unmatched_confirmed_ids),
                'unmatched_ground_truth_ids': list(self.summary.unmatched_ground_truth_ids),
                'time_to_first_detection_s': self.summary.time_to_first_detection_s,
                'time_to_first_confirm_s': self.summary.time_to_first_confirm_s,
                'drones_down': self.summary.drones_down,
                'drone_down_events': list(self.summary.drone_down_events),
                'sector_reassignments': self.summary.sector_reassignments,
                'final_coverage_pct': self.summary.final_coverage_pct,
            },
            'time_series': {
                'coverage_pct': [list(p) for p in self.time_series.coverage_pct],
                'cumulative_confirmed': [list(p) for p in self.time_series.cumulative_confirmed],
                'candidates_count': [list(p) for p in self.time_series.candidates_count],
                **{k: dict(v) for k, v in self.time_series.per_drone.items()},
            },
            'events': list(self.events),
        }
        # V5 metric fields, only emitted when not None.
        for fname in (
            'precision', 'recall', 'f1_score',
            'time_to_coverage_50pct_s', 'time_to_coverage_80pct_s',
            'time_to_coverage_90pct_s',
            'energy_per_coverage_pct_J', 'task_fairness_jain',
        ):
            v = getattr(self.summary, fname)
            if v is not None:
                out['summary'][fname] = v
        if self.summary.detection_latency_per_victim_s is not None:
            out['summary']['detection_latency_per_victim_s'] = list(
                self.summary.detection_latency_per_victim_s
            )
        return out
