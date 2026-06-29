"""Tests for the MotivationWorkspaceStrategy allocator.

Pure-domain pytest. Covers: registration, basic bid pick, symmetric-case
equivalence with ``greedy_auction`` (the central design claim:
distributed-goal is greedy-equivalent in the default symmetric case so
sweeps can isolate when divergence comes from the new motivations),
safety-motivation divergence under injected frustration, determinism
under a fixed seed, and path memory bias.
"""

from __future__ import annotations

import random
from types import SimpleNamespace

from drone_rescue_coordination.lib.allocation import (
    AllocationStrategyFactory,
    GreedyAuctionStrategy,
    MotivationWorkspaceStrategy,
)
from drone_rescue_coordination.lib.domain.affect import ExploitationTracker
from drone_rescue_coordination.lib.domain.motivations import (
    CoverageMotivation,
    SafetyMotivation,
    VictimMotivation,
)
from drone_rescue_coordination.lib.ports.affect_monitor import ExploitationSample


# fixtures

def _drone(name, x, y):
    return SimpleNamespace(
        name=name,
        pose=SimpleNamespace(x=x, y=y),
        battery_ok=True,
        is_down=False,
        current_task_type=0,
        busy_with_victim=None,
    )


def _fleet():
    """Four drones at the cardinal points, 10 m from origin, same as
    the existing ``test_allocation.py`` so behaviour is comparable."""
    return {
        'drone1': _drone('drone1', 10.0, 0.0),
        'drone2': _drone('drone2', 0.0, 10.0),
        'drone3': _drone('drone3', -10.0, 0.0),
        'drone4': _drone('drone4', 0.0, -10.0),
    }


def _target(x, y):
    return SimpleNamespace(x=x, y=y)


# registration

def test_motivation_workspace_strategy_is_registered_by_name():
    strat = AllocationStrategyFactory.create(
        'motivation_workspace', _fleet(), random.Random(42))
    assert isinstance(strat, MotivationWorkspaceStrategy)
    assert strat.name == 'motivation_workspace'


# basic pick

def test_strategy_picks_nearest_in_default_symmetric_case():
    """With only VictimMotivation contributing (the default), the
    strength is ``priority / distance``, so the choice equals
    ``greedy_auction``'s nearest pick."""
    strat = MotivationWorkspaceStrategy(_fleet(), random.Random(42))
    assert strat.bid(_target(20.0, 0.0), priority=2) == 'drone1'   # nearest east


def test_strategy_returns_none_when_no_eligible_drones():
    fleet = _fleet()
    for d in fleet.values():
        d.is_down = True
    strat = MotivationWorkspaceStrategy(fleet, random.Random(42))
    assert strat.bid(_target(0.0, 0.0), priority=1) is None


# greedy-equivalence

def test_symmetric_case_matches_greedy_auction_under_same_seed():
    """The central claim: in the default motivation set
    (CoverageMotivation.coverage_pull=0, SafetyMotivation silent
    because no AffectMonitor is observing), the strategy must match
    ``greedy_auction`` task-for-task under the same RNG seed, so
    introducing the workspace cannot regress the existing sweeps."""
    targets = [
        _target(20.0, 0.0),
        _target(0.0, 20.0),
        _target(-15.0, 5.0),
        _target(5.0, -15.0),
    ]
    greedy = GreedyAuctionStrategy(_fleet(), random.Random(7))
    workspace = MotivationWorkspaceStrategy(_fleet(), random.Random(7))
    g = [greedy.bid(t, priority=2) for t in targets]
    w = [workspace.bid(t, priority=2) for t in targets]
    assert g == w


# safety-inhibition divergence

def test_safety_inhibition_makes_a_frustrated_drone_lose_its_normal_pick():
    """With an AffectMonitor injected and one drone heavily frustrated
    on ``investigate:droneN``, the next-strongest drone should win even
    if the frustrated one is geometrically closest, proving the new
    motivation actually changes allocations."""
    affect = ExploitationTracker(stuck_threshold_s=10.0)
    affect.observe(ExploitationSample('investigate:drone1', 0.0, False))
    affect.observe(ExploitationSample('investigate:drone1', 30.0, False))   # max frustration
    strat = MotivationWorkspaceStrategy(
        _fleet(), random.Random(42), affect=affect,
        # Use a strong inhibition so the closest drone clearly loses.
        motivations=(VictimMotivation(), CoverageMotivation(),
                     SafetyMotivation(frustration_inhibition_scale=10.0)),
    )
    # Target 20m east -> drone1 is the geometric winner.
    winner = strat.bid(_target(20.0, 0.0), priority=2)
    assert winner != 'drone1'        # frustration eliminated the closest pick


# determinism

def test_strategy_is_deterministic_under_a_fixed_seed():
    """Same seed -> identical winner sequence. Determinism is the gate
    for the seeded evaluation harness."""
    targets = [_target(20.0, 0.0), _target(0.0, 20.0), _target(20.0, 20.0)]
    runs = [
        [MotivationWorkspaceStrategy(_fleet(), random.Random(123))
            .bid(t, priority=2) for t in targets]
        for _ in range(3)
    ]
    assert runs[0] == runs[1] == runs[2]


def test_strategy_tie_break_uses_seeded_rng_like_greedy():
    """A perfectly symmetric target equidistant from all four drones
    creates a 4-way tie; under the same seed both strategies must pick
    the same drone (the seeded ``choice`` over sorted names)."""
    target = _target(0.0, 0.0)   # equidistant
    greedy = GreedyAuctionStrategy(_fleet(), random.Random(99))
    workspace = MotivationWorkspaceStrategy(_fleet(), random.Random(99))
    assert greedy.bid(target, priority=2) == workspace.bid(target, priority=2)


# path memory

def test_path_memory_can_bias_the_choice_in_a_close_tie():
    """A small initial proximity gap can be overcome by repeated past
    successes, demonstrating the per-intention path memory the
    workspace records."""
    fleet = _fleet()
    # Bring drone2 almost as close as drone1: 11m east vs 10m east-ish.
    fleet['drone1'].pose = SimpleNamespace(x=10.0, y=0.0)
    fleet['drone2'].pose = SimpleNamespace(x=11.0, y=0.0)
    target = _target(20.0, 0.0)
    strat = MotivationWorkspaceStrategy(fleet, random.Random(42))
    # distance(drone1, target) = 10 ; distance(drone2, target) = 9.
    # drone2 is closer -> drone2 wins by default.
    assert strat.bid(target, priority=2) == 'drone2'
    # Now bias drone1 with repeated successes: 50 hits -> +50% boost.
    for _ in range(50):
        strat.on_intention_succeeded('drone1')
    # drone1 strength = 0.2 * 1.5 = 0.30 ; drone2 strength = 2/9 ~= 0.222
    # drone1 now wins.
    assert strat.bid(target, priority=2) == 'drone1'
