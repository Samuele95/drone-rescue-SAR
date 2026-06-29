"""Unit tests for MissionPortAdapter.

Verifies the rclpy-free adapter wires the pure ``Mission`` aggregate to
the ``MissionPort`` contract the L2 ROS node + translator expect:
- supplies the Clock's now_sec to the timestamp-less MissionPort
  callbacks;
- routes on_health → on_drone_health (+ on_battery_low when battery
  not-ok);
- builds coverage via the provider for on_survey_start;
- forwards tick / state_snapshot.

No rclpy. The FakeAllocation / FakeSectorOwner / _drone helpers mirror
test_mission_saga.py.
"""
from __future__ import annotations

import math
from types import SimpleNamespace

from drone_rescue_coordination.lib.domain.entities import Drone, Victim
from drone_rescue_coordination.lib.domain.incoming import (
    IncomingCandidate, IncomingHealth, IncomingTaskStatus,
)
from drone_rescue_coordination.lib.domain.mission import Mission
from drone_rescue_coordination.lib.domain.state_machines import (
    MissionStage, VictimStage,
)
from drone_rescue_coordination.lib.domain.task_type import TaskType
from drone_rescue_coordination.lib.domain.value_objects import (
    Bid, MissionStateSnapshot, Position, ScanPlan, SectorWedge,
)
from drone_rescue_coordination.lib.mission_port_adapter import (
    MissionPortAdapter,
)
from drone_rescue_coordination.lib.ports.clock import FakeClock


class _FakeAlloc:
    name = 'fake'

    def __init__(self, order):
        self.order = list(order)

    def bid(self, target, priority, exclude=None):
        exclude = exclude or set()
        return next((n for n in self.order if n not in exclude), None)

    def top_bids(self, target, priority, n, exclude=None):
        exclude = exclude or set()
        return [Bid(bidder=p, utility=1.0, target_x=0.0, target_y=0.0)
                for p in self.order if p not in exclude][:n]


def _mission(order=()):
    m = Mission()
    m._allocation_strategy = _FakeAlloc(order)
    m._sector_owner_policy = SimpleNamespace(owner_for=lambda p: None)
    m.investigate_confidence_floor = 0.9
    m.max_concurrent_investigations = 4
    return m


def _adapter(m, *, t=100.0, provider=None, alt=25.0, elev=lambda x, y: 0.0):
    return MissionPortAdapter(
        m, FakeClock(t), coverage_plan_provider=provider,
        elevation_at=elev, survey_altitude=alt,
    )


# Protocol conformance

def test_adapter_satisfies_mission_port_shape():
    """All seven MissionPort methods present + callable."""
    a = _adapter(_mission())
    for name in ('on_candidate', 'on_task_status', 'on_health',
                 'on_battery_low', 'on_survey_start', 'tick',
                 'state_snapshot'):
        assert callable(getattr(a, name))


# clock injection

def test_on_candidate_supplies_clock_now_sec():
    m = _mission(order=['d1'])
    m.register_drone(Drone(name='d1', current_task_type=TaskType.IDLE,
                           pose=Position(0, 0, 25)))
    a = _adapter(m, t=42.0)
    inc = IncomingCandidate(candidate_id=1, position=Position(10, 10, 0),
                            confidence=0.95)

    a.on_candidate(inc)

    # the victim's last_update_sec came from the clock, not a param
    assert m.victims[1].last_update_sec == 42.0


def test_on_task_status_routes_completed_with_clock():
    m = _mission()
    d = Drone(name='d1', current_task_type=TaskType.SCAN,
              pose=Position(0, 0, 25))
    d.current_task_id = 7
    d.scan_waypoints = [Position(5, 5, 25)]
    m.register_drone(d)
    a = _adapter(m, t=55.0)

    out = a.on_task_status(IncomingTaskStatus('d1', 7, 2))  # COMPLETED

    assert d.scan_cursor == 1                 # SCAN-complete → end
    assert out[0].task_type == int(TaskType.IDLE)


# on_health fan-out

def test_on_health_routes_unrecoverable_to_drone_health():
    m = _mission()
    dead = Drone(name='dead', current_task_type=TaskType.SCAN)
    dead.scan_waypoints = [Position(50, 50, 25)]
    receiver = Drone(name='rx', current_task_type=TaskType.SCAN,
                     pose=Position(48, 48, 25))
    receiver.scan_waypoints = [Position(40, 40, 25)]
    m.register_drone(dead)
    m.register_drone(receiver)
    a = _adapter(m)

    out = a.on_health(IncomingHealth(
        drone_name='dead', anomaly_score=1.0, is_down=True,
        battery_ok=True))

    assert m.drones['dead'].is_down is True
    assert out[0].drone_name == 'rx'          # reassigned, preempted SCAN


def test_on_health_also_fires_battery_low_when_not_ok():
    m = _mission()
    d = Drone(name='d1', current_task_type=TaskType.SCAN,
              pose=Position(0, 0, 25))
    m.register_drone(d)
    a = _adapter(m)

    out = a.on_health(IncomingHealth(
        drone_name='d1', anomaly_score=0.0, is_down=False,
        battery_ok=False))

    assert d.battery_ok is False
    assert any(t.task_type == int(TaskType.RTH) for t in out)


def test_on_battery_low_forwards():
    m = _mission()
    d = Drone(name='d1', current_task_type=TaskType.SCAN,
              pose=Position(0, 0, 25))
    m.register_drone(d)
    a = _adapter(m)

    out = a.on_battery_low('d1')

    assert d.battery_ok is False
    assert out[0].task_type == int(TaskType.RTH)


# survey start + coverage

def test_on_survey_start_records_start_and_assigns_coverage():
    m = _mission()
    m.stage = MissionStage.DEPLOYING
    m.register_drone(Drone(name='d1', current_task_type=TaskType.IDLE))
    plan = ScanPlan(waypoints=((0.0, 0.0), (10.0, 0.0)),
                    wedge=SectorWedge(0.0, math.pi / 2))
    coverage = SimpleNamespace(per_drone=[plan])
    a = _adapter(m, provider=lambda: coverage, alt=25.0,
                 elev=lambda x, y: 5.0)

    out = a.on_survey_start(now_sec=200.0)

    assert m.mission_start_sec == 200.0
    assert len(out) == 1
    assert out[0].task_type == int(TaskType.SCAN)
    assert m.drones['d1'].scan_waypoints[0].z == 30.0   # 25 + 5
    assert m.stage == MissionStage.SCANNING


def test_on_survey_start_without_provider_is_noop_dispatch():
    m = _mission()
    a = _adapter(m, provider=None)
    out = a.on_survey_start(now_sec=10.0)
    assert m.mission_start_sec == 10.0    # start still recorded
    assert out == ()                      # but no coverage assigned


# tick + snapshot

def test_tick_forwards_to_mission():
    m = _mission()
    m.stage = MissionStage.SCANNING
    m.mission_start_sec = 0.0
    m.register_drone(Drone(name='d1', current_task_type=TaskType.IDLE,
                           pose=Position(0, 0, 25)))
    a = _adapter(m)
    # idle + no pending → tick drives mission completion without raising
    out = a.tick(now_sec=50.0)
    assert isinstance(out, tuple)


def test_state_snapshot_returns_mission_snapshot():
    m = _mission()
    m.victims[1] = Victim(candidate_id=1, position=Position(0, 0, 0),
                          confidence=0.9, stage=VictimStage.CONFIRMED)
    m.ensure_sub_mission(m.victims[1]).stage = VictimStage.CONFIRMED
    a = _adapter(m)

    snap = a.state_snapshot()

    assert isinstance(snap, MissionStateSnapshot)
    assert snap.victims_confirmed == 1
