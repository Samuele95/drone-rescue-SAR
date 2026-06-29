"""Unit tests for sar_patterns.py: strategy geometry, factory invariants,
RandomWalkPattern reproducibility.

No ROS dependency. Validates the algorithmic contracts that
mission_manager assumes about every CoverageStrategy.
"""

from __future__ import annotations

import math
import random

import pytest

from drone_rescue_coordination.lib.sar_patterns import (
    CoveragePatternFactory,
    PlannerInput,
    RandomWalkPattern,
    Disk,
)


def test_factory_lists_all_six_v5_patterns():
    names = set(CoveragePatternFactory.list_names())
    expected = {'spiral_out', 'spiral_in', 'expanding_square',
                'parallel_track', 'sector_search', 'random_walk'}
    assert expected.issubset(names)


def test_factory_create_returns_strategy_with_matching_name():
    for name in CoveragePatternFactory.list_names():
        s = CoveragePatternFactory.create(name)
        assert s.name == name


def test_factory_create_rejects_unknown():
    with pytest.raises(ValueError):
        CoveragePatternFactory.create('definitely_not_a_pattern')


@pytest.fixture
def planner():
    return PlannerInput(
        mission_center=(0.0, 0.0), radius=70.0, inner_radius=5.0,
        n_drones=4, footprint_m=35.0, overlap=0.85, seed=0,
    )


@pytest.mark.parametrize('pattern_name', [
    'spiral_out', 'spiral_in', 'expanding_square',
    'parallel_track', 'sector_search', 'random_walk',
])
def test_pattern_produces_one_waypoint_list_per_drone(planner, pattern_name):
    s = CoveragePatternFactory.create(pattern_name)
    out = s.plan(planner)
    assert len(out) == planner.n_drones


@pytest.mark.parametrize('pattern_name', [
    'spiral_out', 'spiral_in', 'expanding_square',
    'parallel_track', 'sector_search', 'random_walk',
])
def test_pattern_produces_at_least_one_waypoint_per_drone(planner, pattern_name):
    s = CoveragePatternFactory.create(pattern_name)
    out = s.plan(planner)
    for drone_wps in out:
        assert len(drone_wps) >= 1


def test_spiral_out_radius_monotonic(planner):
    """spiral_out per-drone arcs should grow outward: sqrt(x²+y²) should
    end up > start for the bulk of the trajectory."""
    s = CoveragePatternFactory.create('spiral_out')
    out = s.plan(planner)
    for drone_wps in out:
        first_r = math.hypot(*drone_wps[0])
        last_r = math.hypot(*drone_wps[-1])
        assert last_r > first_r, (
            f'expected outward growth: first_r={first_r}, last_r={last_r}'
        )


def test_n_drones_below_one_raises():
    p = PlannerInput(mission_center=(0, 0), radius=10, inner_radius=0,
                     n_drones=0, footprint_m=2)
    s = CoveragePatternFactory.create('spiral_out')
    with pytest.raises(ValueError):
        s.plan(p)


def test_random_walk_uniform_inside_disk():
    """All RandomWalkPattern waypoints must lie inside the disk."""
    rng = random.Random(42)
    pat = RandomWalkPattern(n_waypoints=200, rng=rng)
    region = Disk(cx=0.0, cy=0.0, outer_radius=70.0)
    wps = pat.generate_waypoints(region, footprint_width_m=35.0)
    for x, y in wps:
        assert math.hypot(x, y) <= 70.0 + 1e-6


def test_random_walk_reproducible_under_same_seed():
    rng_a = random.Random(123)
    rng_b = random.Random(123)
    pat_a = RandomWalkPattern(n_waypoints=50, rng=rng_a)
    pat_b = RandomWalkPattern(n_waypoints=50, rng=rng_b)
    region = Disk(cx=0.0, cy=0.0, outer_radius=50.0)
    a = pat_a.generate_waypoints(region, footprint_width_m=10.0)
    b = pat_b.generate_waypoints(region, footprint_width_m=10.0)
    assert a == b


def test_random_walk_diverges_under_different_seed():
    rng_a = random.Random(123)
    rng_b = random.Random(456)
    pat_a = RandomWalkPattern(n_waypoints=50, rng=rng_a)
    pat_b = RandomWalkPattern(n_waypoints=50, rng=rng_b)
    region = Disk(cx=0.0, cy=0.0, outer_radius=50.0)
    a = pat_a.generate_waypoints(region, footprint_width_m=10.0)
    b = pat_b.generate_waypoints(region, footprint_width_m=10.0)
    assert a != b


def test_random_walk_strategy_is_seeded(planner):
    """Two RandomWalkStrategy.plan() calls with the same PlannerInput
    seed produce identical waypoint lists across all drones."""
    s_a = CoveragePatternFactory.create('random_walk')
    s_b = CoveragePatternFactory.create('random_walk')
    a = s_a.plan(planner)
    b = s_b.plan(planner)
    assert a == b


def test_random_walk_strategy_per_drone_streams_differ(planner):
    """Within one plan(), drone i and drone j must not get the SAME
    waypoint sequence; that would defeat the purpose of an N-drone
    swarm."""
    s = CoveragePatternFactory.create('random_walk')
    out = s.plan(planner)
    for i in range(len(out)):
        for j in range(i + 1, len(out)):
            assert out[i] != out[j], (
                f'drone {i} and drone {j} got identical random-walk lists'
            )


def test_random_walk_invalid_inputs():
    pat = RandomWalkPattern(n_waypoints=0)
    with pytest.raises(ValueError):
        pat.generate_waypoints(Disk(0, 0, 10.0), footprint_width_m=5.0)
    pat2 = RandomWalkPattern(n_waypoints=10)
    with pytest.raises(ValueError):
        pat2.generate_waypoints(Disk(0, 0, 10.0), footprint_width_m=0.0)
