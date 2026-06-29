"""Unit coverage for the concretised Executor.

The Executor is pure-Python; these tests exercise its tick()
orchestration via an InMemoryTree (a fake BT root) + a fake
``state`` object without rclpy. The legacy BT actions live in
``drone_executor.py`` and are exercised through the full integration
flow; this file just pins the architectural seam.
"""

from dataclasses import dataclass
from typing import Optional

from drone_rescue_coordination.lib import bt
from drone_rescue_coordination.lib.domain.behaviour_actions import (
    BehaviouralOutput,
)
from drone_rescue_coordination.lib.domain.executor import (
    Executor, ExecutorOutputs, ExecutorSensors,
)
from drone_rescue_coordination.lib.domain.value_objects import Position


@dataclass
class _FakeState:
    """Stand-in for ExecCtx: only the fields the bridge reads (pure
    input; the BT output is the tree's return value, not state)."""
    now_sec: float = 0.0
    current_pose: Optional[Position] = None
    last_task_id: int = 0


class _FakeTree:
    """BT root stand-in: records the tick and returns a seeded
    ``(Status, BehaviouralOutput)`` (output channel)."""

    def __init__(self, output=None, status=bt.Status.RUNNING, on_tick=None):
        self.ticks = 0
        self._output = output
        self._status = status
        self._on_tick = on_tick

    def tick(self, state):
        self.ticks += 1
        if self._on_tick is not None:
            self._on_tick(state)
        return self._status, self._output


def _update(state, sensors):
    state.now_sec = sensors.now_sec
    state.current_pose = sensors.current_pose


def _read(state, status, output) -> ExecutorOutputs:
    out = output or BehaviouralOutput()
    return ExecutorOutputs(
        target_pose=out.target_pose,
        land_command=out.land_command,
        completed_task_id=state.last_task_id if out.task_completed else None,
    )


# ---------------------------------------------------------------- tick

def test_tick_writes_sensors_into_state_before_running_tree():
    state = _FakeState()
    tree = _FakeTree(on_tick=lambda s: _assert_state_seen(s))
    e = Executor(
        state=state, tree=tree,
        update_state_from_sensors=_update,
        read_outputs_from_state=_read,
    )
    out = e.tick(ExecutorSensors(
        now_sec=42.0,
        current_pose=Position(1.0, 2.0, 25.0),
    ))
    assert tree.ticks == 1
    assert out.target_pose is None  # no out_target seeded
    assert state.now_sec == 42.0
    assert state.current_pose == Position(1.0, 2.0, 25.0)


def _assert_state_seen(state):
    """Asserts the update hook ran before the tree.tick(), i.e.
    the state already reflects the sensor frame by the time BT sees it.
    """
    assert state.now_sec == 42.0
    assert state.current_pose == Position(1.0, 2.0, 25.0)


def test_tick_reads_outputs_from_tree_return():
    """The tree RETURNS its BehaviouralOutput; the read hook maps
    it to ExecutorOutputs post-tick, no state mutation involved."""
    state = _FakeState(last_task_id=9)
    tree = _FakeTree(
        status=bt.Status.SUCCESS,
        output=BehaviouralOutput(
            target_pose=Position(10.0, 20.0, 30.0),
            land_command=True, task_completed=True,
        ),
    )
    e = Executor(
        state=state, tree=tree,
        update_state_from_sensors=_update,
        read_outputs_from_state=_read,
    )
    out = e.tick(ExecutorSensors(now_sec=1.0))
    assert out.target_pose == Position(10.0, 20.0, 30.0)
    assert out.land_command is True
    assert out.completed_task_id == 9   # resolved from state.last_task_id


def test_tick_isolates_per_tick_outputs():
    """Two ticks, second sensor frame; ensure the bridge is called
    fresh, not relying on stale state."""
    state = _FakeState()
    tree = _FakeTree()
    e = Executor(
        state=state, tree=tree,
        update_state_from_sensors=_update,
        read_outputs_from_state=_read,
    )
    e.tick(ExecutorSensors(now_sec=1.0,
                           current_pose=Position(0.0, 0.0, 0.0)))
    out = e.tick(ExecutorSensors(now_sec=5.0,
                                 current_pose=Position(7.0, 0.0, 0.0)))
    assert state.now_sec == 5.0
    assert state.current_pose == Position(7.0, 0.0, 0.0)
    assert tree.ticks == 2


def test_state_property_returns_underlying_state():
    state = _FakeState()
    e = Executor(
        state=state, tree=_FakeTree(),
        update_state_from_sensors=_update,
        read_outputs_from_state=_read,
    )
    assert e.state is state


def test_executor_sensors_default_to_idle_drone():
    """Sanity-check the frozen VO defaults: the BT's IDLE branch
    should fire when given a minimal sensors frame (now_sec only)."""
    s = ExecutorSensors(now_sec=0.0)
    assert s.current_task_type == 5   # IDLE
    assert s.is_down is False
    assert s.battery_ok is True


# ----------------------------------------------------- output channel

def test_bt_threads_leaf_output_up_through_selector_and_switch():
    """The behavioural output from the leaf that determined the
    result bubbles up through Switch and Selector unchanged."""
    sentinel = BehaviouralOutput(status_detail='leaf-ran')
    leaf = bt.Action(lambda ctx: (bt.Status.RUNNING, sentinel), name='leaf')
    sw = bt.Switch(key_fn=lambda ctx: 'k', branches={'k': leaf}, name='sw')
    root = bt.Selector([sw], name='root')
    status, out = root.tick(object())
    assert status == bt.Status.RUNNING
    assert out is sentinel


def test_bt_sequence_carries_action_output_past_condition():
    """A Condition contributes no output (None); the Action's output is
    what the Sequence returns on SUCCESS."""
    out_val = BehaviouralOutput(land_command=True)
    seq = bt.Sequence([
        bt.Condition(lambda ctx: True, name='cond'),
        bt.Action(lambda ctx: (bt.Status.SUCCESS, out_val), name='act'),
    ], name='seq', memory=False)
    status, out = seq.tick(object())
    assert status == bt.Status.SUCCESS
    assert out is out_val


def test_bt_condition_failure_yields_none_output():
    seq = bt.Sequence([
        bt.Condition(lambda ctx: False, name='cond'),
        bt.Action(lambda ctx: (bt.Status.SUCCESS,
                               BehaviouralOutput(land_command=True)), name='act'),
    ], name='seq', memory=False)
    status, out = seq.tick(object())
    assert status == bt.Status.FAILURE
    assert out is None    # the failing condition's output, action never ran
