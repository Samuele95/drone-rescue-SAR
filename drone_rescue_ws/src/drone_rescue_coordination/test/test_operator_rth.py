"""Tests for operator-commanded return-home.

The dashboard's Return-home / Recall-fleet buttons publish
``/mission/operator_rth`` (String: drone name, or '*' for the fleet);
``MissionManager._on_operator_rth`` flips the drone's ``battery_ok``
exclusion gate and issues an RTH TaskAssignment. The executor's BT
then flies home + lands and stops streaming survey targets. The
original implementation published one home setpoint straight to
``/<drone>/survey_target`` and was overwritten by the executor's
per-tick stream; these tests pin the task-system path instead.

Bare-instance pattern (no rclpy.init) per test_operator_goal.
"""

from __future__ import annotations

from types import SimpleNamespace

from drone_rescue_coordination.mission_manager import MissionManager

from drone_rescue_msgs.msg import TaskAssignment
from std_msgs.msg import String


def _drone(name='drone', task_type=TaskAssignment.SCAN_WAYPOINTS, *,
           down=False, battery_ok=True):
    # name + empty scan state so _force_rth's sector handoff (which now runs on
    # every RTH) is a clean no-op here: nothing assigned, nothing to hand off.
    return SimpleNamespace(
        name=name, is_down=down, battery_ok=battery_ok,
        current_task_type=task_type, scan_waypoints=[], scan_cursor=0,
    )


def _bare_mm(drones, active=True):
    mm = object.__new__(MissionManager)
    mm._is_active = active
    mm._drones = drones
    mm._issued = []
    mm._events = []
    mm._issue_task = (
        lambda name, ttype, **kw: mm._issued.append((name, ttype))
    )
    mm._emit_event = lambda *a, **k: mm._events.append((a, k))
    mm.get_logger = lambda: SimpleNamespace(
        info=lambda *a, **k: None, warning=lambda *a, **k: None,
    )
    return mm


def _msg(data):
    m = String()
    m.data = data
    return m


def test_single_drone_rth_issues_task_and_excludes_from_auction():
    drones = {'drone1': _drone('drone1'), 'drone2': _drone('drone2')}
    mm = _bare_mm(drones)
    mm._on_operator_rth(_msg('drone1'))
    assert mm._issued == [('drone1', TaskAssignment.RTH)]
    # battery_ok is the system-wide "no further tasks" gate.
    assert drones['drone1'].battery_ok is False
    assert drones['drone2'].battery_ok is True
    assert any(a[0] == 'OPERATOR_RTH' for a, k in mm._events)


def test_star_recalls_whole_fleet():
    drones = {f'drone{i}': _drone(f'drone{i}') for i in (1, 2, 3)}
    mm = _bare_mm(drones)
    mm._on_operator_rth(_msg('*'))
    assert sorted(n for n, t in mm._issued) == ['drone1', 'drone2', 'drone3']
    assert all(t == TaskAssignment.RTH for _n, t in mm._issued)
    assert all(not d.battery_ok for d in drones.values())


def test_down_and_already_homeward_drones_skipped():
    drones = {
        'down': _drone('down', down=True),
        'homeward': _drone('homeward', task_type=TaskAssignment.RTH),
        'flying': _drone('flying'),
    }
    mm = _bare_mm(drones)
    mm._on_operator_rth(_msg('*'))
    assert mm._issued == [('flying', TaskAssignment.RTH)]


def test_unknown_drone_is_noop():
    mm = _bare_mm({'drone1': _drone('drone1')})
    mm._on_operator_rth(_msg('nonexistent'))
    assert mm._issued == []
    assert mm._events == []


def test_ignored_while_inactive():
    drones = {'drone1': _drone()}
    mm = _bare_mm(drones, active=False)
    mm._on_operator_rth(_msg('drone1'))
    assert mm._issued == []
    assert drones['drone1'].battery_ok is True
