"""Batch INVESTIGATE dispatch.

``mission_manager`` now routes batch-capable allocation strategies
(``hungarian``) through ``HungarianStrategy.assign()`` instead of
dispatching one candidate per tick. This test pins the property that
makes that routing worthwhile: the joint Hungarian assignment beats
iterated greedy whenever several candidates compete for the same drones.

Pure-python pytest; no ROS node; it exercises ``lib/allocation.py``
exactly as the mission_manager batch path does.
"""

from __future__ import annotations

import math
import random
from types import SimpleNamespace

from drone_rescue_coordination.lib.allocation import (
    GreedyAuctionStrategy,
    HungarianStrategy,
)


def _drone(name, x, y):
    return SimpleNamespace(
        name=name, pose=SimpleNamespace(x=x, y=y),
        battery_ok=True, is_down=False,
        current_task_type=0, busy_with_victim=None,
    )


def _t(x, y):
    return SimpleNamespace(x=x, y=y)


def _rng():
    return random.Random(42)


# Two drones, two targets, all on the x-axis. Both targets sit nearer
# drone_b, so iterated greedy grabs a target with drone_b first and
# strands drone_a on the long leg. The joint optimum does the opposite.
def _fleet():
    return {
        'drone_a': _drone('drone_a', 0.0, 0.0),
        'drone_b': _drone('drone_b', 10.0, 0.0),
    }


_T1 = (6.0, 0.0)   # drone_a 6 m, drone_b 4 m
_T2 = (9.0, 0.0)   # drone_a 9 m, drone_b 1 m


def _total_distance(fleet, assignment, targets):
    tot = 0.0
    for drone_name, (tx, ty) in zip(assignment, targets):
        d = fleet[drone_name]
        tot += math.hypot(d.pose.x - tx, d.pose.y - ty)
    return tot


def test_iterated_greedy_is_suboptimal():
    """Greedy assigns target-by-target: T1 goes to the nearest free
    drone (drone_b), then T2 takes whoever is left (drone_a)."""
    greedy = GreedyAuctionStrategy(_fleet(), _rng())
    w1 = greedy.bid(_t(*_T1), priority=2)
    w2 = greedy.bid(_t(*_T2), priority=2, exclude={w1})
    assert [w1, w2] == ['drone_b', 'drone_a']


def test_hungarian_batch_finds_joint_optimum():
    """assign() minimises TOTAL cost: drone_a takes the near target,
    drone_b the far one, the opposite of greedy, and cheaper overall."""
    hungarian = HungarianStrategy(_fleet(), _rng())
    winners = hungarian.assign([_t(*_T1), _t(*_T2)], priority=2)
    assert winners == ['drone_a', 'drone_b']


def test_batch_assignment_beats_greedy_total_distance():
    """Batch path goal: lower total transit."""
    fleet, targets = _fleet(), [_T1, _T2]

    greedy = GreedyAuctionStrategy(_fleet(), _rng())
    g1 = greedy.bid(_t(*_T1), priority=2)
    g2 = greedy.bid(_t(*_T2), priority=2, exclude={g1})
    greedy_total = _total_distance(fleet, [g1, g2], targets)

    hungarian = HungarianStrategy(_fleet(), _rng())
    batch = hungarian.assign([_t(*_T1), _t(*_T2)], priority=2)
    batch_total = _total_distance(fleet, batch, targets)

    assert batch_total < greedy_total


def test_assign_honours_exclude():
    """_drain_investigate_batch hands already-committed sector owners to
    assign() via exclude=; assign() must never re-pick them."""
    hungarian = HungarianStrategy(_fleet(), _rng())
    winners = hungarian.assign([_t(*_T1)], priority=2, exclude={'drone_b'})
    assert winners == ['drone_a']
