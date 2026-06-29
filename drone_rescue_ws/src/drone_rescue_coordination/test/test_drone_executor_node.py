"""Smoke tests for the DroneExecutor scaffolding."""

from __future__ import annotations

import os

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

from drone_rescue_coordination import drone_executor_node
from drone_rescue_coordination.lib.domain.executor import (
    ExecutorOutputs, ExecutorSensors,
)
from drone_rescue_coordination.lib.domain.value_objects import Position
from drone_rescue_coordination.lib.ros_adapter.translators_executor import (
    position_from_odom, sensors_from_inputs, target_pose_msg_from,
)


def test_node_module_imports_cleanly():
    assert hasattr(drone_executor_node, 'main')


def test_use_legacy_drone_executor_flag_defaults_to_zero():
    """The BT cutover is complete, so the legacy flag now defaults OFF
    (USE_LEGACY_DRONE_EXECUTOR=0)."""
    if 'USE_LEGACY_DRONE_EXECUTOR' in os.environ:
        del os.environ['USE_LEGACY_DRONE_EXECUTOR']
    import importlib
    importlib.reload(drone_executor_node)
    assert drone_executor_node.USE_LEGACY_DRONE_EXECUTOR is False


def test_drone_executor_implements_behavioural_layer():
    """DroneExecutor realizes the L1 BehaviouralLayer port: it exposes
    the dispatch_task + cancel_task contract (checked at the class level;
    instantiating the LifecycleNode needs rclpy.init)."""
    from drone_rescue_coordination.drone_executor import DroneExecutor
    for method in ('dispatch_task', 'cancel_task'):
        assert callable(getattr(DroneExecutor, method, None)), method


def test_position_from_odom():
    msg = Odometry()
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 2.0
    msg.pose.pose.position.z = 3.0
    assert position_from_odom(msg) == Position(1.0, 2.0, 3.0)


def test_sensors_from_inputs_builds_VO():
    target = Point()
    target.x, target.y, target.z = 10.0, 20.0, 0.5
    sensors = sensors_from_inputs(
        now_sec=42.0, target_msg=target,
        current_task_type=1, current_task_id=7,
    )
    assert isinstance(sensors, ExecutorSensors)
    assert sensors.now_sec == 42.0
    assert sensors.target == Position(10.0, 20.0, 0.5)
    assert sensors.current_task_type == 1
    assert sensors.current_task_id == 7


def test_target_pose_msg_from_outputs_none_when_no_target():
    assert target_pose_msg_from(ExecutorOutputs()) is None


def test_target_pose_msg_from_outputs_round_trips():
    outputs = ExecutorOutputs(target_pose=Position(1.0, 2.0, 3.0))
    msg = target_pose_msg_from(outputs)
    assert (msg.x, msg.y, msg.z) == (1.0, 2.0, 3.0)


# act_investigate
# act_investigate flies the multi-view orbit the deliberative layer
# (L3) stamped on the task (investigate_radius / dwell_s /
# investigate_angles), falling back to its own BehaviouralContextMutable config only when
# those wire fields are unset (legacy publishers / replayed bags).

import math   # noqa: E402

from drone_rescue_coordination.drone_executor import (   # noqa: E402
    BehaviouralContextMutable, act_investigate,
)
from drone_rescue_coordination.lib import bt   # noqa: E402
from drone_rescue_msgs.msg import TaskAssignment   # noqa: E402


def _investigate_task(*, radius, dwell, angles, tx=10.0, ty=20.0):
    t = TaskAssignment()
    t.task_type = TaskAssignment.INVESTIGATE
    t.target_point.x, t.target_point.y, t.target_point.z = tx, ty, 0.0
    t.investigate_radius = float(radius)
    t.dwell_s = float(dwell)
    t.investigate_angles = [float(a) for a in angles]
    return t


def test_act_investigate_uses_planned_angles_and_radius():
    """The angle queue + viewpoint radius come from the task, not the
    executor's 4-cardinal / 5 m defaults. act_* return
    ``(Status, BehaviouralOutput)`` rather than mutating ctx.out_*."""
    ctx = BehaviouralContextMutable(drone_name='drone1')
    ctx.current_task = _investigate_task(radius=6.0, dwell=1.0,
                                         angles=[0.0, math.pi])
    _status, out = act_investigate(ctx)   # first tick seeds the queue
    assert len(ctx.cursor.investigate_angles) == 2   # planned, not 4 cardinals
    # angle[0]=0 → viewpoint at target.x + 6.0 along +x.
    assert abs(out.target_pose.x - 16.0) < 1e-4
    assert abs(out.target_pose.y - 20.0) < 1e-4


def test_act_investigate_completes_after_planned_dwell_count():
    """A 2-angle plan completes after 2 dwells (proves the count is
    plan-driven, not hard-wired to 4)."""
    ctx = BehaviouralContextMutable(drone_name='drone1')
    ctx.position_tolerance_m = 2.0
    ctx.current_task = _investigate_task(radius=6.0, dwell=1.0,
                                         angles=[0.0, math.pi])
    # Viewpoint 0 = (16, 20): arrive, dwell, pop.
    ctx.current_pose = Position(16.0, 20.0, 10.0); ctx.now_sec = 0.0
    assert act_investigate(ctx)[0] == bt.Status.RUNNING        # start dwell
    ctx.now_sec = 1.5
    assert act_investigate(ctx)[0] == bt.Status.RUNNING        # dwell done → pop
    assert len(ctx.cursor.investigate_angles) == 1
    # Viewpoint 1 (angle π) = (4, 20): arrive, dwell, pop → SUCCESS.
    ctx.current_pose = Position(4.0, 20.0, 10.0)
    assert act_investigate(ctx)[0] == bt.Status.RUNNING        # start dwell
    ctx.now_sec = 3.0
    status, out = act_investigate(ctx)
    assert status == bt.Status.SUCCESS
    assert out.task_completed is True
    assert out.status_detail == 'multi-view investigate done'


def test_act_investigate_falls_back_to_executor_config_when_unset():
    """Empty / 0.0 wire fields (legacy task / replayed bag) → the executor
    flies its own 4 cardinals at its own investigate_radius_m."""
    ctx = BehaviouralContextMutable(drone_name='drone1')
    ctx.investigate_radius_m = 7.0
    ctx.current_task = _investigate_task(radius=0.0, dwell=0.0, angles=[])
    _status, out = act_investigate(ctx)
    assert len(ctx.cursor.investigate_angles) == 4          # 4 cardinals
    assert abs(out.target_pose.x - 17.0) < 1e-4   # 10 + 7.0


# act output + bridge
# act_* return (Status, BehaviouralOutput); BehaviouralContextMutable carries no output
# slots; the bridge resolves the completed task id from ctx.

from drone_rescue_coordination.drone_executor import (   # noqa: E402
    act_land, act_idle, _outputs_from_tick,
)
from drone_rescue_coordination.lib.domain.behaviour_actions import (   # noqa: E402
    BehaviouralOutput,
)


def test_act_land_returns_completion_output():
    status, out = act_land(BehaviouralContextMutable(drone_name='drone1'))
    assert status == bt.Status.SUCCESS
    assert out.land_command is True
    assert out.task_completed is True


def test_act_idle_returns_hold_or_none():
    # No pose → nothing to command.
    status, out = act_idle(BehaviouralContextMutable(drone_name='drone1'))
    assert status == bt.Status.RUNNING and out is None
    # With a pose → hold-position target, not a completion.
    ctx = BehaviouralContextMutable(drone_name='drone1')
    ctx.current_pose = Position(3.0, 4.0, 10.0)
    status, out = act_idle(ctx)
    assert status == bt.Status.RUNNING
    assert out.target_pose is not None and out.task_completed is False


def test_outputs_from_tick_maps_output_and_resolves_task_id():
    ctx = BehaviouralContextMutable(drone_name='drone1')
    ctx.last_task_id = 42
    done = _outputs_from_tick(
        ctx, bt.Status.SUCCESS,
        BehaviouralOutput(task_completed=True, status_detail='done'),
    )
    assert done.completed_task_id == 42      # resolved from ctx.last_task_id
    assert done.status_detail == 'done'
    running = _outputs_from_tick(
        ctx, bt.Status.RUNNING,
        BehaviouralOutput(target_pose=Position(1.0, 2.0, 3.0)),
    )
    assert running.completed_task_id is None   # no completion this tick
    assert running.target_pose == Position(1.0, 2.0, 3.0)
    # A tick that commanded nothing (None output) is mapped safely.
    empty = _outputs_from_tick(ctx, bt.Status.RUNNING, None)
    assert empty.target_pose is None and empty.completed_task_id is None
