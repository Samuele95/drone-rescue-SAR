"""Regression tests for the D1->D2 wiring.

The bug: AllocationStrategyFactory.create dropped its ``affect`` kwarg,
so MotivationWorkspaceStrategy ran with ``_affect=None`` in every
production run and SafetyMotivation was silently inert. The parallel
hole was on the saga side: Mission.on_task_completed never called
``on_intention_succeeded`` on the allocation strategy, so the
per-intention path memory accumulated no entries.

These tests pin both halves of the fix down so the bug cannot
regress.
"""

from __future__ import annotations

import random
from types import SimpleNamespace

from drone_rescue_coordination.lib.allocation import (
    AllocationStrategyFactory,
    MotivationWorkspaceStrategy,
)
from drone_rescue_coordination.lib.auction import TaskType
from drone_rescue_coordination.lib.domain.affect import ExploitationTracker
from drone_rescue_coordination.lib.domain.entities import Drone, Victim
from drone_rescue_coordination.lib.domain.mission import Mission
from drone_rescue_coordination.lib.domain.value_objects import (
    OutgoingTask,
    Position,
)


# factory side


def _fleet():
    """Same 4-drone cardinal fixture the other allocation tests use."""
    def _drone(name, x, y):
        return SimpleNamespace(
            name=name,
            pose=SimpleNamespace(x=x, y=y),
            battery_ok=True,
            is_down=False,
            current_task_type=0,
            busy_with_victim=None,
        )
    return {
        'drone1': _drone('drone1', 10.0, 0.0),
        'drone2': _drone('drone2', 0.0, 10.0),
        'drone3': _drone('drone3', -10.0, 0.0),
        'drone4': _drone('drone4', 0.0, -10.0),
    }


def test_factory_threads_affect_into_motivation_workspace_strategy():
    """The headline regression lock: an AffectMonitor passed to
    ``create(...)`` reaches MotivationWorkspaceStrategy._affect,
    the D1->D2 feedback link the bug had severed."""
    monitor = ExploitationTracker(stuck_threshold_s=10.0)
    strat = AllocationStrategyFactory.create(
        'motivation_workspace', _fleet(), random.Random(7),
        affect=monitor,
    )
    assert isinstance(strat, MotivationWorkspaceStrategy)
    assert strat._affect is monitor


def test_factory_drops_affect_for_strategies_that_do_not_accept_it():
    """The other strategies don't accept ``affect``: the factory
    introspects each strategy's signature and only forwards the kwarg
    where it fits. Passing ``affect=`` to a 'hungarian' build must NOT
    raise TypeError."""
    monitor = ExploitationTracker()
    # Should not raise: hungarian's __init__ doesn't accept affect.
    AllocationStrategyFactory.create(
        'hungarian', _fleet(), random.Random(0), affect=monitor)
    AllocationStrategyFactory.create(
        'greedy_auction', _fleet(), random.Random(0), affect=monitor)
    AllocationStrategyFactory.create(
        'round_robin', _fleet(), random.Random(0), affect=monitor)


def test_factory_default_affect_is_none_backcompat():
    """Existing callers of ``create(name, drones, rng)`` (no kwarg) must
    keep working: ``affect=None`` is the default."""
    strat = AllocationStrategyFactory.create(
        'motivation_workspace', _fleet(), random.Random(0))
    assert strat._affect is None


# saga side


class _RecordingStrategy:
    """Minimal stub of the on_intention_succeeded surface."""

    def __init__(self):
        self.succeeded_drones = []

    def on_intention_succeeded(self, drone_name: str) -> None:
        self.succeeded_drones.append(drone_name)


def _mission_with_one_drone_confirming(drone_name='droneZ', victim_id=42):
    """Build a Mission set up so a CONFIRM-completed task on
    ``drone_name`` will resolve victim ``victim_id``: the minimal
    state the on_task_completed branch needs."""
    m = Mission()
    d = Drone(name=drone_name)
    d.pose = Position(0.0, 0.0, 10.0)
    d.busy_with_victim = victim_id
    d.scan_waypoints = ()
    d.scan_cursor = 0
    m.drones[drone_name] = d
    v = Victim(
        candidate_id=victim_id,
        position=Position(5.0, 5.0, 0.0),
        confidence=0.95,
    )
    m.victims[victim_id] = v
    return m


def test_on_task_completed_confirm_invokes_on_intention_succeeded():
    """The other half of the wiring: when a CONFIRM task completes
    and the allocation strategy supports the hook, the saga notifies
    it with the confirming drone's name. Without this the
    MotivationWorkspaceStrategy's per-intention path memory stays
    empty in production."""
    m = _mission_with_one_drone_confirming()
    m._allocation_strategy = _RecordingStrategy()

    completed = OutgoingTask(
        drone_name='droneZ',
        task_type=int(TaskType.CONFIRM),
        waypoints=(),
        target=(5.0, 5.0, 0.0),
        victim_id=42,
        priority=2,
        hover_seconds=0.0,
    )
    m.on_task_completed(world=None, completed_task=completed)
    assert m._allocation_strategy.succeeded_drones == ['droneZ']


def test_on_task_completed_confirm_is_silent_when_strategy_lacks_hook():
    """The hook is hasattr-guarded so non-motivation strategies do not
    raise. Critically, this also means existing greedy_auction sweeps
    don't break."""
    m = _mission_with_one_drone_confirming()
    m._allocation_strategy = object()   # no on_intention_succeeded

    completed = OutgoingTask(
        drone_name='droneZ',
        task_type=int(TaskType.CONFIRM),
        waypoints=(),
        target=(5.0, 5.0, 0.0),
        victim_id=42,
        priority=2,
        hover_seconds=0.0,
    )
    m.on_task_completed(world=None, completed_task=completed)
    # No exception, no recording happens; guard holds.


def test_on_task_completed_non_confirm_does_not_invoke_hook():
    """The hook fires only on CONFIRM completion: INVESTIGATE/SCAN
    completion must not record a success."""
    m = _mission_with_one_drone_confirming()
    m._allocation_strategy = _RecordingStrategy()

    # SCAN completion should not fire the hook.
    completed = OutgoingTask(
        drone_name='droneZ',
        task_type=int(TaskType.SCAN),
        waypoints=(),
        target=None,
        victim_id=0,
        priority=1,
        hover_seconds=0.0,
    )
    m.on_task_completed(world=None, completed_task=completed)
    assert m._allocation_strategy.succeeded_drones == []
