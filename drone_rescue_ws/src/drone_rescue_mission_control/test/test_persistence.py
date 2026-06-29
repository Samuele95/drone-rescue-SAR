"""Unit tests for persistence layer.

Exercises RunSummary.from_jsonl + JsonlRunRepository against the
committed baseline JSONL and a synthetic in-memory dict for V4-shape
schema coverage.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from drone_rescue_mission_control.persistence import (
    RunSummary, RunMetadata, RunMetrics, TimeSeries,
    RunHandle, RunRepository, JsonlRunRepository,
    SCHEMA_VERSION_LATEST, MIN_SUPPORTED_SCHEMA,
)


_REPO_ROOT = Path(__file__).resolve().parents[4]
_BASELINE_JSONL = (
    _REPO_ROOT / 'drone_rescue_ws' / 'runs' / 'baseline'
    / '2026-05-11_091536__spiral_out__default.json'
)


# from_jsonl on real V5 JSONL

@pytest.mark.skipif(not _BASELINE_JSONL.is_file(),
                    reason='baseline JSONL not present')
def test_load_baseline_jsonl():
    """The committed baseline JSONL predates the V5 metric extensions:
    it carries TP/FP but no precision/F1, so schema detection
    correctly flags it as V4. The loader silently up-grades V5
    fields to None."""
    rs = RunSummary.from_jsonl(_BASELINE_JSONL)
    assert rs.schema_version == 4
    assert rs.metadata.pattern == 'spiral_out'
    assert rs.metadata.scenario == 'default'
    # V4-shape: classifier counts present, V5 derived metrics None
    assert rs.summary.true_positives is not None
    assert rs.summary.precision is None
    assert rs.summary.f1_score is None
    # Frozen dataclass; mutation forbidden
    with pytest.raises(Exception):
        rs.metadata = RunMetadata()   # type: ignore[misc]


# V4 schema upgrade

def test_v4_dict_upgrades_with_v5_fields_as_none():
    """A V4-era dict (no precision/f1_score keys) should load with
    schema_version=4 and the V5 fields as None, no crash."""
    v4_dict = {
        'metadata': {
            'started_at': '2026-05-08T10:00:00+00:00',
            'ended_at': '2026-05-08T10:09:50+00:00',
            'duration_s': 590.0,
            'ended_by': 'MISSION_COMPLETE',
            'scenario': 'default',
            'pattern': 'spiral_out',
            'ground_truth_victims': [{'id': 1, 'position': [44, 38, 0]}],
        },
        'summary': {
            'candidates_emitted': 5,
            'victims_confirmed': 3,
            'true_positives': 3,
            'false_positives': 0,
            'false_negatives': 2,
            'final_coverage_pct': 87.0,
            'drones_down': 0,
        },
        'time_series': {
            'coverage_pct': [[0, 0.0], [10, 5.5]],
            'cumulative_confirmed': [[0, 0]],
            'candidates_count': [],
            'drone1': {'battery': [], 'task': []},
        },
        'events': [],
    }
    rs = RunSummary.from_dict(v4_dict)
    assert rs.schema_version == 4
    assert rs.summary.true_positives == 3
    assert rs.summary.precision is None
    assert rs.summary.f1_score is None
    assert rs.summary.detection_latency_per_victim_s is None
    assert rs.metadata.pattern == 'spiral_out'


def test_v5_dict_loads_all_fields():
    v5_dict = {
        'metadata': {'pattern': 'random_walk', 'scenario': 'cluster',
                     'duration_s': 600.0, 'ended_by': 'MISSION_TIMEOUT',
                     'started_at': '', 'ended_at': '',
                     'ground_truth_victims': [],
                     'params_snapshot': {'seed': 42}},
        'summary': {'precision': 0.8, 'recall': 0.6, 'f1_score': 0.686,
                    'true_positives': 4, 'false_positives': 1, 'false_negatives': 3,
                    'final_coverage_pct': 76.5,
                    'time_to_coverage_50pct_s': 120.0,
                    'energy_per_coverage_pct_J': 0.012,
                    'task_fairness_jain': 0.94,
                    'detection_latency_per_victim_s': [12.5, 18.0, 25.0, 9.0],
                    'drones_down': 0},
        'time_series': {'coverage_pct': [], 'cumulative_confirmed': [],
                        'candidates_count': []},
        'events': [],
    }
    rs = RunSummary.from_dict(v5_dict)
    assert rs.schema_version == 5
    assert rs.summary.precision == pytest.approx(0.8)
    assert rs.summary.f1_score == pytest.approx(0.686)
    assert rs.summary.detection_latency_per_victim_s == (12.5, 18.0, 25.0, 9.0)
    assert rs.metadata.params_snapshot['seed'] == 42


# to_dict round-trip

def test_round_trip_preserves_v5_fields():
    """from_dict → to_dict → from_dict produces an equivalent RunSummary."""
    v5_dict = {
        'metadata': {'pattern': 'spiral_out', 'scenario': 'default',
                     'duration_s': 100.0, 'ended_by': 'OPERATOR_STOP',
                     'started_at': '2026-05-12T10:00:00Z', 'ended_at': '',
                     'ground_truth_victims': [], 'params_snapshot': {}},
        'summary': {'true_positives': 2, 'false_positives': 1, 'false_negatives': 0,
                    'final_coverage_pct': 50.0, 'precision': 0.667,
                    'drones_down': 0},
        'time_series': {'coverage_pct': [[0, 0]], 'cumulative_confirmed': [],
                        'candidates_count': []},
        'events': [],
    }
    rs1 = RunSummary.from_dict(v5_dict)
    out = rs1.to_dict()
    rs2 = RunSummary.from_dict(out)
    assert rs1.schema_version == rs2.schema_version
    assert rs1.summary.precision == rs2.summary.precision
    assert rs1.summary.true_positives == rs2.summary.true_positives


# JsonlRunRepository

def test_repo_list_skips_non_run_files(tmp_path):
    (tmp_path / 'manifest.json').write_text('{}')
    (tmp_path / '2026-05-12_120000__spiral_out__default.json').write_text(
        json.dumps({
            'metadata': {'pattern': 'spiral_out', 'scenario': 'default',
                         'duration_s': 1.0, 'ended_by': 'X',
                         'started_at': '', 'ended_at': '',
                         'ground_truth_victims': [], 'params_snapshot': {}},
            'summary': {'final_coverage_pct': 0.0},
            'time_series': {'coverage_pct': [], 'cumulative_confirmed': [],
                            'candidates_count': []},
            'events': [],
        })
    )
    repo = JsonlRunRepository(tmp_path)
    handles = repo.list()
    assert len(handles) == 1
    assert handles[0].name.endswith('__default.json')


def test_repo_save_then_load_round_trip(tmp_path):
    rs = RunSummary.from_dict({
        'metadata': {'pattern': 'random_walk', 'scenario': 'sparse',
                     'duration_s': 100.0, 'ended_by': 'MISSION_COMPLETE',
                     'started_at': '', 'ended_at': '',
                     'ground_truth_victims': [], 'params_snapshot': {}},
        'summary': {'true_positives': 1, 'final_coverage_pct': 42.0,
                    'precision': 1.0, 'recall': 0.5, 'f1_score': 0.667,
                    'drones_down': 0},
        'time_series': {'coverage_pct': [], 'cumulative_confirmed': [],
                        'candidates_count': []},
        'events': [],
    })
    repo = JsonlRunRepository(tmp_path)
    written = repo.save(rs, tmp_path)
    assert written.is_file()
    handles = repo.list()
    assert len(handles) == 1
    loaded = repo.load(handles[0])
    assert loaded.summary.f1_score == pytest.approx(0.667)
    assert loaded.metadata.pattern == 'random_walk'


def test_repo_iter_summaries_skips_malformed(tmp_path):
    """A malformed JSONL is silently skipped (mission-recorder analytics
    path historically tolerated mixed-era / corrupt files)."""
    (tmp_path / 'malformed.json').write_text('not-valid-json')
    (tmp_path / '2026-05-12_x__spiral_out__default.json').write_text(
        json.dumps({
            'metadata': {'pattern': 'spiral_out', 'scenario': 'default',
                         'duration_s': 1.0, 'ended_by': 'X',
                         'started_at': '', 'ended_at': '',
                         'ground_truth_victims': [], 'params_snapshot': {}},
            'summary': {'final_coverage_pct': 0.0},
            'time_series': {'coverage_pct': [], 'cumulative_confirmed': [],
                            'candidates_count': []},
            'events': [],
        })
    )
    repo = JsonlRunRepository(tmp_path)
    summaries = list(repo.iter_summaries())
    assert len(summaries) == 1   # malformed dropped


def test_repo_list_empty_dir():
    """Non-existent directory → empty list, not error."""
    repo = JsonlRunRepository(Path('/tmp/does-not-exist-12345'))
    assert repo.list() == []


# schema-version constants

def test_schema_constants_make_sense():
    assert SCHEMA_VERSION_LATEST == 5
    assert MIN_SUPPORTED_SCHEMA == 4
    assert MIN_SUPPORTED_SCHEMA <= SCHEMA_VERSION_LATEST


def test_unsupported_schema_raises():
    """A future schema_version=3 dict should fail-loud."""
    # A fake "v3" dict signals V3 by adding a field, but since detection
    # is by V5 field presence, V3 just looks like V4. The
    # MIN_SUPPORTED_SCHEMA guard fires when from_dict is asked for an
    # explicitly older schema, simulated here by monkey-patching the
    # constant locally.
    import drone_rescue_mission_control.persistence.run_summary as rs_mod
    orig = rs_mod.MIN_SUPPORTED_SCHEMA
    try:
        rs_mod.MIN_SUPPORTED_SCHEMA = 5
        with pytest.raises(ValueError):
            RunSummary.from_dict({
                'metadata': {'pattern': 'x', 'scenario': 'y'},
                'summary': {'final_coverage_pct': 0.0},   # no V5 fields → schema=4
                'time_series': {}, 'events': [],
            })
    finally:
        rs_mod.MIN_SUPPORTED_SCHEMA = orig
