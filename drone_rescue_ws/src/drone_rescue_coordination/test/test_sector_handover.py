"""Contiguity-preserving sector handover on drone loss.

When a drone is marked DOWN, ``MissionManager._reassign_sector`` hands its
unfinished scan waypoints (as one contiguous ordered run) to the single
angularly-nearest surviving sector owner, instead of scattering them
round-robin across the whole fleet.

These tests exercise the handover logic without spinning up a ROS node:
the manager is allocated via ``object.__new__`` and the node-side
collaborators (``_issue_task``, ``_emit_event``, ``get_logger``) are
stubbed. The new geometry helpers are pure and tested directly.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from drone_rescue_coordination.mission_manager import DroneRecord, MissionManager
from drone_rescue_coordination.lib.domain.mission import Mission
from drone_rescue_msgs.msg import TaskAssignment

# The sector-handover geometry
# (``_wedge_midpoint`` / ``_nearest_survivor`` / ``_absorb_wedge``) and the
# reassignment decision moved into ``Mission.handle_drone_lost`` (+ its
# ``_nearest_survivor`` / ``_absorb_wedge`` helpers). Those are covered
# directly in ``test_mission_saga.py``. What this file now uniquely
# verifies is the L2 ADAPTER SHIM: that ``MissionManager._reassign_sector``
# correctly sync→delegates to the aggregate, mirrors the result back onto
# the legacy ``DroneRecord``s, issues the preempting SCAN task, and emits
# the SECTOR_REASSIGNED event.


def _drone(name, *, x=0.0, y=0.0, wedge=(0.0, 0.0),
           task_type=TaskAssignment.IDLE, is_down=False,
           scan_waypoints=None, scan_cursor=0):
    # Real DroneRecord: exercises the append_scan_tail /
    # clear_scan_state methods that _reassign_sector now routes through.
    return DroneRecord(
        name=name,
        pose=SimpleNamespace(x=x, y=y, z=10.0),
        is_down=is_down,
        sector_start_rad=wedge[0],
        sector_end_rad=wedge[1],
        current_task_type=task_type,
        scan_waypoints=list(scan_waypoints or []),
        scan_cursor=scan_cursor,
    )


def _wp(x, y):
    return SimpleNamespace(x=float(x), y=float(y), z=10.0)


def _bare_manager(drones):
    """A MissionManager shell (no __init__, no ROS) wired with just
    what the ``_reassign_sector`` shim touches: a real
    ``Mission`` aggregate (the shim delegates the geometry to it), the
    legacy record dicts, a stub clock + point factory, and recorders
    for the issued tasks / emitted events."""
    mm = object.__new__(MissionManager)
    mm._drones = {d.name: d for d in drones}
    mm._victims = {}
    mm._mission = Mission()
    mm._stage = None
    mm._time = SimpleNamespace(now_sec=lambda: 0.0)
    mm._point = lambda x, y, z: SimpleNamespace(
        x=float(x), y=float(y), z=float(z))
    mm.issued = []
    mm.events = []
    mm._issue_task = lambda name, tt, **kw: mm.issued.append((name, tt, kw))
    mm._emit_event = lambda kind, **kw: mm.events.append((kind, kw))
    mm.get_logger = lambda: SimpleNamespace(
        error=lambda *a, **k: None, info=lambda *a, **k: None,
        warning=lambda *a, **k: None,
    )
    return mm


# Shim integration. The underlying geometry (angularly-nearest
# survivor, wedge absorption, contiguous handover) is unit-tested
# directly on the aggregate in test_mission_saga.py.

def test_reassign_hands_contiguous_run_to_single_survivor():
    orphans = [_wp(i, i) for i in range(6)]
    dead = _drone('drone2', wedge=(3.0, 4.0),
                  scan_waypoints=orphans, scan_cursor=2)   # wp2..wp5 orphan
    near = _drone('drone1', wedge=(2.0, 3.0),
                  task_type=TaskAssignment.SCAN_WAYPOINTS,
                  scan_waypoints=[_wp(-1, -1), _wp(-2, -2)], scan_cursor=1)
    far = _drone('drone3', wedge=(4.5, 6.0),
                 task_type=TaskAssignment.SCAN_WAYPOINTS,
                 scan_waypoints=[_wp(-9, -9)], scan_cursor=0)
    mm = _bare_manager([dead, near, far])

    mm._reassign_sector(dead)

    # Whole 4-wp orphan run appended contiguously to the one nearest
    # survivor (drone1), mirrored back onto its DroneRecord; the far
    # survivor is untouched.
    assert len(near.scan_waypoints) == 6
    assert [(p.x, p.y) for p in near.scan_waypoints[2:]] == \
        [(2.0, 2.0), (3.0, 3.0), (4.0, 4.0), (5.0, 5.0)]
    assert len(far.scan_waypoints) == 1
    # Dead drone drained.
    assert dead.scan_waypoints == []
    assert dead.scan_cursor == 0
    # Receiver wedge widened to absorb the dead sector (mirrored back).
    assert near.sector_end_rad == pytest.approx(4.0)
    # Mid-SCAN receiver was eagerly re-dispatched with the extended tail.
    assert len(mm.issued) == 1
    name, tt, kw = mm.issued[0]
    assert name == 'drone1' and tt == TaskAssignment.SCAN_WAYPOINTS
    assert len(kw['waypoints']) == 5          # scan_waypoints[1:]
    # One SECTOR_REASSIGNED event, naming the single receiving drone.
    assert len(mm.events) == 1
    kind, kw = mm.events[0]
    assert kind == 'SECTOR_REASSIGNED'
    assert 'drone1' in kw['detail']


def test_force_rth_reassigns_returning_drones_sector():
    """Regression: a drone recalled home (operator recall / zone breach /
    battery RTH all route through _force_rth or the sibling _on_battery) must
    hand its remaining sector to a healthy peer, the same takeover the DOWN
    path gets. Previously RTH orphaned the sector (only is_down triggered
    reassignment), so the returning drone's area was never re-covered."""
    orphans = [_wp(i, i) for i in range(6)]
    leaving = _drone('drone2', wedge=(3.0, 4.0),
                     task_type=TaskAssignment.SCAN_WAYPOINTS,
                     scan_waypoints=orphans, scan_cursor=2)   # wp2..wp5 orphan
    survivor = _drone('drone1', wedge=(2.0, 3.0),
                      task_type=TaskAssignment.SCAN_WAYPOINTS,
                      scan_waypoints=[_wp(-1, -1), _wp(-2, -2)], scan_cursor=1)
    mm = _bare_manager([leaving, survivor])

    ok = mm._force_rth('drone2', event_name='OPERATOR_RTH',
                       detail='operator recall', severity=2)

    assert ok is True
    assert leaving.battery_ok is False           # marked unavailable
    # Orphan run handed to the survivor; leaving drone drained.
    assert leaving.scan_waypoints == []
    assert len(survivor.scan_waypoints) == 6
    issued = [(n, t) for n, t, _ in mm.issued]
    assert ('drone2', TaskAssignment.RTH) in issued       # the drone flies home
    assert ('drone1', TaskAssignment.SCAN_WAYPOINTS) in issued  # peer takes over
    kinds = [k for k, _ in mm.events]
    assert 'SECTOR_REASSIGNED' in kinds          # takeover signalled
    assert 'OPERATOR_RTH' in kinds               # departure signalled


def test_battery_low_rth_reassigns_sector():
    """The battery-low path (_on_battery, a *separate* call site from
    _force_rth) must also hand the returning drone's sector to a healthy peer.
    Regression: only DOWN reassigned; a battery-RTH drone orphaned its sector."""
    orphans = [_wp(i, i) for i in range(6)]
    leaving = _drone('drone2', wedge=(3.0, 4.0),
                     task_type=TaskAssignment.SCAN_WAYPOINTS,
                     scan_waypoints=orphans, scan_cursor=2)
    survivor = _drone('drone1', wedge=(2.0, 3.0),
                      task_type=TaskAssignment.SCAN_WAYPOINTS,
                      scan_waypoints=[_wp(-1, -1), _wp(-2, -2)], scan_cursor=1)
    mm = _bare_manager([leaving, survivor])

    # battery_low fires: was battery_ok=True, msg.data=True → transition to RTH.
    mm._on_battery(SimpleNamespace(data=True), 'drone2')

    assert leaving.battery_ok is False
    assert leaving.scan_waypoints == []          # drained
    assert len(survivor.scan_waypoints) == 6     # absorbed the orphan run
    kinds = [k for k, _ in mm.events]
    assert 'SECTOR_REASSIGNED' in kinds          # takeover
    assert 'BATTERY_RTH' in kinds                # departure signalled


def test_returning_drone_is_not_chosen_as_reassignment_receiver():
    """A peer that is itself returning home (battery_ok=False) must not be
    handed an orphaned sector, it would orphan it again."""
    orphans = [_wp(i, i) for i in range(4)]
    dead = _drone('drone2', wedge=(3.0, 4.0),
                  scan_waypoints=orphans, scan_cursor=0)
    returning = _drone('drone1', wedge=(2.0, 3.0),
                       task_type=TaskAssignment.SCAN_WAYPOINTS,
                       scan_waypoints=[_wp(-1, -1)], scan_cursor=0)
    returning.battery_ok = False                  # already flying home
    healthy = _drone('drone3', wedge=(4.5, 6.0),
                     task_type=TaskAssignment.SCAN_WAYPOINTS,
                     scan_waypoints=[_wp(-9, -9)], scan_cursor=0)
    mm = _bare_manager([dead, returning, healthy])

    mm._reassign_sector(dead)

    # The healthy drone receives the tail; the returning one is skipped.
    assert len(healthy.scan_waypoints) == 5
    assert len(returning.scan_waypoints) == 1


def test_reassign_noop_when_dead_had_no_orphans():
    dead = _drone('drone2', wedge=(3.0, 4.0),
                  scan_waypoints=[_wp(0, 0)], scan_cursor=1)   # all done
    near = _drone('drone1', wedge=(2.0, 3.0))
    mm = _bare_manager([dead, near])
    mm._reassign_sector(dead)
    assert mm.issued == []
    assert mm.events == []
