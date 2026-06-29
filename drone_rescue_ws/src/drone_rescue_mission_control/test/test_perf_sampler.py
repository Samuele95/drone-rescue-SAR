"""Tests for the runtime performance sampler.

No run recorded a resource metric. PerfSampler records REAL RTF / CPU / RSS
during a mission; summarize_perf folds the samples into the run JSON's
performance block, with null envelopes when unmeasured (never fabricated).
"""

from __future__ import annotations

import pytest

from drone_rescue_mission_control.perf_sampler import (
    PerfSampler,
    summarize_perf,
)


def test_summarize_empty_is_all_null():
    s = summarize_perf([], [], [])
    assert s['rtf'] == {'mean': None, 'min': None, 'max': None}
    assert s['system_cpu_percent']['mean'] is None
    assert s['node_tree_rss_mb']['max'] is None
    assert s['sample_count'] == 0


def test_summarize_envelope_math():
    s = summarize_perf([0.2, 0.3, 0.4], [10.0, 30.0], [100.0])
    assert s['rtf']['mean'] == pytest.approx(0.3)
    assert s['rtf']['min'] == pytest.approx(0.2)
    assert s['rtf']['max'] == pytest.approx(0.4)
    assert s['system_cpu_percent']['mean'] == pytest.approx(20.0)
    assert s['node_tree_rss_mb'] == {'mean': 100.0, 'min': 100.0, 'max': 100.0}
    assert s['sample_count'] == 3


def test_rtf_needs_two_samples():
    # node_hints that match nothing keeps RSS at 0 and avoids env coupling.
    sampler = PerfSampler(node_hints=('__no_such_process__',))
    sampler.sample(0.0, 0.0)
    assert sampler.summary()['sample_count'] == 0      # first call seeds only
    sampler.sample(2.0, 1.0)                            # +2 sim over +1 wall
    summ = sampler.summary()
    assert summ['sample_count'] == 1
    assert summ['rtf']['mean'] == pytest.approx(2.0)


def test_rtf_skips_nonincreasing_wall_time():
    sampler = PerfSampler(node_hints=('__no_such_process__',))
    sampler.sample(0.0, 5.0)
    sampler.sample(1.0, 5.0)   # wall didn't advance -> no rtf point
    assert sampler.summary()['sample_count'] == 0


def test_summary_note_disclaims_fabrication():
    assert 'never fabricated' in summarize_perf([], [], [])['note']
