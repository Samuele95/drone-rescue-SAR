"""Regression tests for the pre-survey candidate gate.

The bug it closes: ``mission_manager._on_candidate`` only checked
``self._is_active`` and ``_stage not in (COMPLETE, ABORTED)``; it
did NOT check whether the deliberative coverage plan had been issued.
So if ``/survey/start`` never fired (e.g. the /clock-bridge race
suppressed readiness_coordinator), candidates flowing in still got
auctioned out as INVESTIGATEs, degenerating the system into a pure-
reactive victim chase with no spiral coverage. Trajectories from such
a run contradict the thesis's 3T claim (deliberative L3 contributes
nothing) and the Unit-10 organisation-layer claim (CoverageMotivation
has no plan to express).

These tests pin (a) candidates arriving in INIT/ARMING/DEPLOYING are
dropped with a tombstone-and-log, (b) the same cluster_id re-arriving
after SCANNING begins is processed normally, (c) the tombstone set
is cleared by ``_begin_scan`` so post-SCANNING re-arrivals enter the
saga.

Pure pytest; no rclpy.init(); we exercise the unbound method.
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import List

from drone_rescue_coordination.lib.domain.state_machines import (
    MissionStage,
)
from drone_rescue_coordination.mission_manager import MissionManager


# fixtures

class _FakeLogger:
    """Records every (level, message) call so tests can assert."""

    def __init__(self) -> None:
        self.warnings: List[str] = []
        self.infos: List[str] = []

    def warning(self, msg: str) -> None:
        self.warnings.append(msg)

    def info(self, msg: str) -> None:
        self.infos.append(msg)


def _candidate(cid: int, *, x: float = 1.0, y: float = 2.0,
               conf: float = 0.95, confirmed: bool = False):
    """Minimal VictimCandidate stand-in, duck-typed so the test
    doesn't depend on the generated msg class being importable."""
    return SimpleNamespace(
        candidate_id=cid,
        position=SimpleNamespace(x=x, y=y, z=0.0),
        confidence=conf,
        confirmed=confirmed,
        reporting_drones=['drone1'],
    )


def _node(stage: MissionStage, *, is_active: bool = True):
    """Construct the minimal SimpleNamespace surface that
    ``_on_candidate`` reads. We bind the unbound method against this
    duck-typed object so no rclpy.init() is required."""
    logger = _FakeLogger()
    ns = SimpleNamespace(
        _is_active=is_active,
        _stage=stage,
        _victims={},
        _pre_survey_dropped_cids=set(),
        get_logger=lambda: logger,
        # The downstream branches reference these but should NEVER
        # execute in the gated cases. Booby-trap so any test that
        # accidentally falls through fails loudly.
        _time=SimpleNamespace(
            now_sec=lambda: _fail('_time consulted in gated path'),
        ),
        _allocation_strategy=None,
        _emit_event=lambda *a, **kw: _fail(
            '_emit_event invoked in gated path'
        ),
        _dispatch_investigate=lambda r: _fail(
            '_dispatch_investigate invoked in gated path'
        ),
        _drain_investigate_batch=lambda: _fail(
            '_drain_investigate_batch invoked in gated path'
        ),
    )
    return ns, logger


def _fail(why: str):
    raise AssertionError(why)


# gated cases

def test_candidate_in_init_is_dropped_and_logged_once():
    """A candidate arriving while stage=INIT must be dropped, logged
    exactly once, and tombstoned by cluster_id."""
    ns, logger = _node(MissionStage.INIT)
    MissionManager._on_candidate(ns, _candidate(42))
    # No victim record created (saga did not advance).
    assert ns._victims == {}
    # Tombstone set (so a re-arrival doesn't re-log).
    assert ns._pre_survey_dropped_cids == {42}
    # Exactly one warning emitted, naming the stage and the cid.
    assert len(logger.warnings) == 1
    assert 'CANDIDATE #42' in logger.warnings[0]
    assert 'INIT' in logger.warnings[0]


def test_candidate_in_arming_is_dropped():
    """ARMING is also pre-survey; same gate applies."""
    ns, logger = _node(MissionStage.ARMING)
    MissionManager._on_candidate(ns, _candidate(7))
    assert ns._victims == {}
    assert 7 in ns._pre_survey_dropped_cids
    assert len(logger.warnings) == 1
    assert 'ARMING' in logger.warnings[0]


def test_candidate_in_deploying_is_dropped():
    """DEPLOYING, the exact stage we observed the bug in, must
    drop candidates until /survey/start triggers _begin_scan."""
    ns, logger = _node(MissionStage.DEPLOYING)
    MissionManager._on_candidate(ns, _candidate(9))
    assert ns._victims == {}
    assert 9 in ns._pre_survey_dropped_cids
    assert 'DEPLOYING' in logger.warnings[0]


def test_repeated_drops_log_only_once_per_cluster():
    """detection_filter republishes the same cluster at publish_rate_hz
    cadence. The drop log must fire exactly once per cluster_id, not
    on every re-publish, or the runtime log floods."""
    ns, logger = _node(MissionStage.DEPLOYING)
    for _ in range(20):
        MissionManager._on_candidate(ns, _candidate(99))
    # 20 re-publishes, 1 log line.
    assert len(logger.warnings) == 1
    assert ns._pre_survey_dropped_cids == {99}


def test_distinct_clusters_each_log_once():
    """Multiple distinct cluster_ids each produce one drop line."""
    ns, logger = _node(MissionStage.DEPLOYING)
    for cid in (1, 2, 3, 1, 2):    # 1 and 2 each repeat
        MissionManager._on_candidate(ns, _candidate(cid))
    assert ns._pre_survey_dropped_cids == {1, 2, 3}
    assert len(logger.warnings) == 3   # one per distinct cid


def test_inactive_node_does_not_engage_the_gate():
    """The pre-existing ``_is_active`` check must still short-circuit
    before the new stage gate (otherwise a non-active node would
    silently tombstone its first pre-survey clusters and re-engage
    weirdly later)."""
    ns, logger = _node(MissionStage.DEPLOYING, is_active=False)
    MissionManager._on_candidate(ns, _candidate(5))
    # No tombstone; gate didn't even run.
    assert ns._pre_survey_dropped_cids == set()
    assert logger.warnings == []


def test_terminal_stages_still_short_circuit_before_gate():
    """COMPLETE / ABORTED are terminal; the gate should never run
    against them. The pre-existing terminal-stage check takes
    priority; assert it still does."""
    for stage in (MissionStage.COMPLETE, MissionStage.ABORTED):
        ns, logger = _node(stage)
        MissionManager._on_candidate(ns, _candidate(11))
        assert ns._pre_survey_dropped_cids == set(), (
            f'gate should not run in terminal stage {stage.name}'
        )
        assert logger.warnings == []


# post-survey passes

def test_candidate_in_scanning_passes_the_gate():
    """Once SCANNING begins, candidates must follow the saga path.
    We arrange a node whose downstream dispatcher captures the call;
    asserting the gate didn't drop is equivalent to asserting
    _dispatch_investigate ran."""
    dispatched: List[int] = []
    logger = _FakeLogger()
    ns = SimpleNamespace(
        _is_active=True,
        _stage=MissionStage.SCANNING,
        _victims={},
        _pre_survey_dropped_cids=set(),
        get_logger=lambda: logger,
        _time=SimpleNamespace(now_sec=lambda: 123.0),
        _allocation_strategy=SimpleNamespace(),
        _emit_event=lambda *a, **kw: None,
        _dispatch_investigate=lambda r: dispatched.append(r.candidate_id),
    )
    MissionManager._on_candidate(ns, _candidate(77))
    # Reached the saga; dispatcher fired.
    assert dispatched == [77]
    # And got a victim record.
    assert 77 in ns._victims
    # Gate did NOT tombstone.
    assert ns._pre_survey_dropped_cids == set()
    assert logger.warnings == []


def test_candidate_in_investigating_passes_the_gate():
    """INVESTIGATING is also post-survey; candidates must pass."""
    dispatched: List[int] = []
    logger = _FakeLogger()
    ns = SimpleNamespace(
        _is_active=True,
        _stage=MissionStage.INVESTIGATING,
        _victims={},
        _pre_survey_dropped_cids=set(),
        get_logger=lambda: logger,
        _time=SimpleNamespace(now_sec=lambda: 123.0),
        _allocation_strategy=SimpleNamespace(),
        _emit_event=lambda *a, **kw: None,
        _dispatch_investigate=lambda r: dispatched.append(r.candidate_id),
    )
    MissionManager._on_candidate(ns, _candidate(88))
    assert dispatched == [88]
