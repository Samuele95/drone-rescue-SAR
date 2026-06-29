"""Tests for the RunViewModel projection (architect F7).

Pure-Python; no Qt or rclpy.
"""

from __future__ import annotations

from drone_rescue_ui_common.run_view_model import RunRow, RunViewModel


def test_empty_view_model_starts_with_no_rows():
    vm = RunViewModel()
    assert vm.rows == ()


def test_apply_legacy_raw_dict_extracts_row():
    raw = {
        'metadata': {
            'scenario': 'rural_dense',
            'pattern': 'spiral_out',
            'duration_s': 612.5,
            'started_at': '2026-05-13T08:30:00Z',
        },
        'summary': {
            'final_coverage_pct': 88.4,
            'true_positives': 7,
            'false_positives': 0,
            'false_negatives': 1,
            'victims_confirmed': 7,
        },
    }
    vm = RunViewModel().apply(raw)
    assert len(vm.rows) == 1
    row = vm.rows[0]
    assert row.scenario == 'rural_dense'
    assert row.pattern == 'spiral_out'
    assert row.final_coverage_pct == 88.4
    assert row.true_positives == 7


def test_apply_returns_new_view_model_not_mutates_old():
    """RunViewModel.apply is pure; old VM unchanged."""
    vm0 = RunViewModel()
    vm1 = vm0.apply({'metadata': {}, 'summary': {}})
    assert vm0.rows == ()
    assert len(vm1.rows) == 1


def test_with_rows_replaces_wholesale():
    vm = RunViewModel(
        rows=(RunRow(run_label='', scenario='a', pattern='x'),)
    )
    new_rows = (
        RunRow(run_label='', scenario='b', pattern='y'),
        RunRow(run_label='', scenario='c', pattern='z'),
    )
    vm2 = vm.with_rows(new_rows)
    assert len(vm2.rows) == 2
    assert vm2.rows[0].scenario == 'b'


def test_apply_handles_missing_fields():
    """Defaults kick in when fields are missing, backward-compat
    for legacy JSONLs."""
    vm = RunViewModel().apply({'metadata': {'scenario': 'x'}, 'summary': {}})
    row = vm.rows[0]
    assert row.scenario == 'x'
    assert row.pattern == '?'
    assert row.duration_s == 0.0
    assert row.victims_confirmed == 0
