"""Saga-lift unit tests: pure-Python, no rclpy.

Exercises the saga logic AFTER it is lifted into the `Mission`
aggregate (the L3 deliberative planner).

These tests construct `Mission` directly with `Drone` / `Victim` /
`VictimSubMission` entities and assert on the returned `OutgoingTask`
sequences plus the mutated aggregate state: no ROS, no
`_sync_to_mission`.
"""
from __future__ import annotations

import math

import pytest

from drone_rescue_coordination.lib.domain.entities import Drone, Victim
from drone_rescue_coordination.lib.domain.mission import Mission
from drone_rescue_coordination.lib.domain.state_machines import VictimStage
from drone_rescue_coordination.lib.domain.task_type import TaskType
from drone_rescue_coordination.lib.domain.value_objects import (
    Bid, OutgoingTask, Position, SectorWedge,
)


# fakes

class FakeAllocation:
    """Stub AllocationBidder/BatchAllocationBidder for planner tests.

    ``bid`` returns the first name in ``order`` not excluded; ``top_bids``
    returns the next-after-winner; ``assign`` round-robins ``order`` over
    the targets (excluding ``exclude``).
    """
    name = 'fake'

    def __init__(self, order):
        self.order = list(order)

    def bid(self, target, priority, exclude=None):
        exclude = exclude or set()
        for n in self.order:
            if n not in exclude:
                return n
        return None

    def top_bids(self, target, priority, n, exclude=None):
        exclude = exclude or set()
        picks = [name for name in self.order if name not in exclude][:n]
        return [Bid(bidder=p, utility=1.0, target_x=0.0, target_y=0.0)
                for p in picks]

    def assign(self, targets, priority, exclude=None):
        exclude = set(exclude or set())
        out = []
        for _ in targets:
            winner = next((n for n in self.order if n not in exclude), None)
            out.append(winner)
            if winner is not None:
                exclude.add(winner)
        return out


class FakeSectorOwner:
    """Stub SectorOwnerPolicy: returns a fixed owner (or None)."""
    def __init__(self, owner=None):
        self._owner = owner

    def owner_for(self, p):
        return self._owner


def _drone(name, *, pose=None, wedge=None, task=TaskType.SCAN,
           waypoints=None, cursor=0, is_down=False, busy=None):
    d = Drone(name=name, pose=pose, current_task_type=task,
              is_down=is_down, busy_with_victim=busy, scan_cursor=cursor)
    d.sector_wedge = wedge
    if waypoints:
        d.scan_waypoints = list(waypoints)
    return d


# handle_drone_lost

def test_handle_drone_lost_no_survivors_returns_empty():
    m = Mission()
    m.register_drone(_drone('drone1', waypoints=[Position(5, 5, 25)]))
    # the only other drone is also down
    m.register_drone(_drone('drone2', is_down=True))
    out = m.handle_drone_lost('drone1', now_sec=10.0)
    assert out == ()


def test_handle_drone_lost_no_remaining_waypoints_returns_empty():
    m = Mission()
    m.register_drone(_drone('drone1', waypoints=[]))
    m.register_drone(_drone('drone2', pose=Position(0, 0, 25)))
    out = m.handle_drone_lost('drone1', now_sec=10.0)
    assert out == ()


def test_handle_drone_lost_appends_orphan_run_to_nearest_survivor_by_pose():
    """No wedges (non-angular pattern) → nearest-by-pose receives the run."""
    m = Mission()
    m.register_drone(_drone(
        'dead', waypoints=[Position(50, 50, 25), Position(60, 60, 25)],
        cursor=0,
    ))
    near = _drone('near', pose=Position(48, 48, 25), task=TaskType.IDLE)
    far = _drone('far', pose=Position(-90, -90, 25), task=TaskType.IDLE)
    m.register_drone(near)
    m.register_drone(far)

    m.handle_drone_lost('dead', now_sec=10.0)

    # near is closer to the orphan run's first waypoint (50,50)
    assert [(p.x, p.y) for p in near.scan_waypoints] == [(50, 50), (60, 60)]
    assert far.scan_waypoints == []
    # dead drone drained
    assert m.drones['dead'].scan_waypoints == []
    assert m.drones['dead'].scan_cursor == 0


def test_handle_drone_lost_picks_angularly_nearest_wedge_survivor():
    """With wedges, the angularly-closest survivor receives the run."""
    m = Mission()
    # dead wedge centred at ~0 rad
    m.register_drone(_drone(
        'dead', wedge=SectorWedge(0.0, math.pi / 2),
        waypoints=[Position(10, 0, 25)],
    ))
    # adjacent survivor wedge just past dead's end
    adjacent = _drone('adjacent', wedge=SectorWedge(math.pi / 2, math.pi),
                      task=TaskType.IDLE, pose=Position(0, 0, 25))
    # opposite survivor wedge across the disk
    opposite = _drone('opposite', wedge=SectorWedge(math.pi, 3 * math.pi / 2),
                      task=TaskType.IDLE, pose=Position(0, 0, 25))
    m.register_drone(adjacent)
    m.register_drone(opposite)

    m.handle_drone_lost('dead', now_sec=10.0)

    assert len(adjacent.scan_waypoints) == 1
    assert opposite.scan_waypoints == []


def test_handle_drone_lost_widens_receiver_wedge():
    m = Mission()
    m.register_drone(_drone(
        'dead', wedge=SectorWedge(0.0, math.pi / 2),
        waypoints=[Position(10, 0, 25)],
    ))
    receiver = _drone('receiver', wedge=SectorWedge(math.pi / 2, math.pi),
                      task=TaskType.IDLE, pose=Position(0, 0, 25))
    m.register_drone(receiver)

    m.handle_drone_lost('dead', now_sec=10.0)

    # wedge widened to cover the absorbed area (absorb() returns a new
    # wedge spanning both)
    assert receiver.sector_wedge != SectorWedge(math.pi / 2, math.pi)


def test_handle_drone_lost_preempts_receiver_mid_scan():
    """Receiver mid-SCAN → a preempting SCAN OutgoingTask is returned."""
    m = Mission()
    m.register_drone(_drone(
        'dead', waypoints=[Position(50, 50, 25)], cursor=0,
    ))
    receiver = _drone('receiver', pose=Position(48, 48, 25),
                      task=TaskType.SCAN,
                      waypoints=[Position(40, 40, 25)], cursor=0)
    m.register_drone(receiver)

    out = m.handle_drone_lost('dead', now_sec=10.0)

    assert len(out) == 1
    task = out[0]
    assert task.drone_name == 'receiver'
    assert task.task_type == int(TaskType.SCAN)
    # the preempting task carries the receiver's remaining tail
    # (its own waypoint + the appended orphan)
    assert len(task.waypoints) == 2


def test_handle_drone_lost_skips_preempt_when_receiver_busy_with_victim():
    """Receiver mid-INVESTIGATE (not SCAN) → no preempting task; the
    tail is still appended for later SCAN re-dispatch."""
    m = Mission()
    m.register_drone(_drone(
        'dead', waypoints=[Position(50, 50, 25)], cursor=0,
    ))
    receiver = _drone('receiver', pose=Position(48, 48, 25),
                      task=TaskType.INVESTIGATE, busy=7)
    m.register_drone(receiver)

    out = m.handle_drone_lost('dead', now_sec=10.0)

    assert out == ()
    # tail still appended
    assert len(receiver.scan_waypoints) == 1


# plan / plan_for

def _mission_with_planner(*, order, owner=None, floor=0.9, cap=1):
    m = Mission()
    m._allocation_strategy = FakeAllocation(order)
    m._sector_owner_policy = FakeSectorOwner(owner)
    m.investigate_confidence_floor = floor
    m.max_concurrent_investigations = cap
    return m


def _victim(vid, *, conf=0.95, stage=VictimStage.DETECTED, x=10.0, y=10.0):
    return Victim(candidate_id=vid, position=Position(x, y, 0.0),
                  confidence=conf, stage=stage)


def test_plan_for_dispatches_to_sector_owner_when_available():
    m = _mission_with_planner(order=['auctioned'], owner='owner1')
    m.register_drone(_drone('owner1', task=TaskType.IDLE,
                            pose=Position(0, 0, 25)))
    v = _victim(1)
    m.victims[1] = v

    out = m.plan_for(v, world=None)

    assert len(out) == 1
    assert out[0].drone_name == 'owner1'      # owner wins, not the auction
    assert out[0].task_type == int(TaskType.INVESTIGATE)
    assert v.stage == VictimStage.INVESTIGATING
    assert m.drones['owner1'].busy_with_victim == 1


def test_plan_for_stamps_multi_view_orbit_on_investigate_task():
    """The INVESTIGATE OutgoingTask the planner emits carries the
    multi-view orbit config (radius / per-angle dwell / angle set) so the
    L1 executor flies the plan the deliberative layer chose."""
    m = _mission_with_planner(order=['auctioned'], owner='owner1')
    m.investigate_radius_m = 6.5
    m.investigate_dwell_s = 3.0
    m.register_drone(_drone('owner1', task=TaskType.IDLE,
                            pose=Position(0, 0, 25)))
    v = _victim(1)
    m.victims[1] = v

    out = m.plan_for(v, world=None)

    assert out[0].investigate_radius == 6.5
    assert out[0].dwell_s == 3.0
    assert out[0].investigate_angles == m.investigate_angles
    assert len(out[0].investigate_angles) == 4   # default 4 cardinals


def test_plan_for_skips_when_confidence_below_floor():
    m = _mission_with_planner(order=['d1'], floor=0.9)
    m.register_drone(_drone('d1', task=TaskType.IDLE))
    v = _victim(1, conf=0.5)
    m.victims[1] = v
    assert m.plan_for(v, world=None) == ()
    assert v.stage == VictimStage.DETECTED


def test_plan_for_skips_when_concurrency_cap_reached():
    m = _mission_with_planner(order=['d1'], cap=1)
    # one drone already busy → dispatched count == cap
    m.register_drone(_drone('busy', task=TaskType.INVESTIGATE, busy=99))
    m.register_drone(_drone('d1', task=TaskType.IDLE))
    v = _victim(1)
    m.victims[1] = v
    assert m.plan_for(v, world=None) == ()


def test_plan_for_falls_back_to_auction_when_owner_unavailable():
    # owner exists but is down → auction picks 'peer'
    m = _mission_with_planner(order=['peer'], owner='owner1')
    m.register_drone(_drone('owner1', task=TaskType.IDLE, is_down=True))
    m.register_drone(_drone('peer', task=TaskType.IDLE,
                            pose=Position(0, 0, 25)))
    v = _victim(1)
    m.victims[1] = v

    out = m.plan_for(v, world=None)

    assert len(out) == 1
    assert out[0].drone_name == 'peer'


def test_plan_for_caches_witness_runner_up_on_sub_mission():
    """The auction runner-up is cached on VictimSubMission."""
    m = _mission_with_planner(order=['winner', 'runner_up'])
    m.register_drone(_drone('winner', task=TaskType.IDLE,
                            pose=Position(0, 0, 25)))
    m.register_drone(_drone('runner_up', task=TaskType.IDLE,
                            pose=Position(1, 1, 25)))
    v = _victim(1)
    m.victims[1] = v

    m.plan_for(v, world=None)

    sub = m.sub_missions[1]
    assert sub.witness_drone == 'runner_up'    # excludes the winner


def test_plan_for_returns_empty_when_no_drone_available():
    m = _mission_with_planner(order=[])   # auction yields nobody
    v = _victim(1)
    m.victims[1] = v
    assert m.plan_for(v, world=None) == ()
    assert v.stage == VictimStage.DETECTED


def test_plan_batch_pass_a_sector_owner_direct_dispatch():
    """Batch plan(): a candidate whose owner is free goes straight to
    that owner (Pass A), not into the joint assignment."""
    m = _mission_with_planner(order=['pooled'], owner='owner1', cap=4)
    m.register_drone(_drone('owner1', task=TaskType.IDLE,
                            pose=Position(0, 0, 25)))
    m.victims[1] = _victim(1)

    out = m.plan(world=None)

    assert len(out) == 1
    assert out[0].drone_name == 'owner1'


def test_plan_batch_pass_b_hungarian_pooled_assign():
    """Batch plan(): owner-unavailable candidates enter the joint
    assign() pass and are distributed across the order."""
    m = _mission_with_planner(order=['a', 'b'], owner=None, cap=4)
    m.register_drone(_drone('a', task=TaskType.IDLE, pose=Position(0, 0, 25)))
    m.register_drone(_drone('b', task=TaskType.IDLE, pose=Position(1, 1, 25)))
    m.victims[1] = _victim(1, x=10, y=10)
    m.victims[2] = _victim(2, x=20, y=20)

    out = m.plan(world=None)

    assert len(out) == 2
    assert {t.drone_name for t in out} == {'a', 'b'}


def test_plan_respects_concurrency_budget_in_batch():
    m = _mission_with_planner(order=['a', 'b', 'c'], owner=None, cap=1)
    m.register_drone(_drone('a', task=TaskType.IDLE, pose=Position(0, 0, 25)))
    m.register_drone(_drone('b', task=TaskType.IDLE, pose=Position(1, 1, 25)))
    m.victims[1] = _victim(1)
    m.victims[2] = _victim(2)

    out = m.plan(world=None)

    assert len(out) == 1   # cap=1 → only one dispatch


def test_concurrency_invariant_after_plan():
    """Invariant: after a batch plan(), dispatched_investigate_count
    equals the number of drones with a busy_with_victim slot set."""
    m = _mission_with_planner(order=['a', 'b'], owner=None, cap=4)
    m.register_drone(_drone('a', task=TaskType.IDLE, pose=Position(0, 0, 25)))
    m.register_drone(_drone('b', task=TaskType.IDLE, pose=Position(1, 1, 25)))
    m.victims[1] = _victim(1)
    m.victims[2] = _victim(2)

    m.plan(world=None)

    busy = sum(1 for d in m.drones.values()
               if d.busy_with_victim is not None)
    assert m.dispatched_investigate_count() == busy == 2


# on_task_completed

def _completed(drone_name, task_type, *, victim_id=0):
    return OutgoingTask(
        drone_name=drone_name, task_type=int(task_type),
        waypoints=(), target=None, victim_id=victim_id,
        priority=0, hover_seconds=0.0,
    )


def test_on_completed_investigate_dispatches_confirm_to_witness():
    """INVESTIGATE done → CONFIRM dispatched to the cached witness, a
    DIFFERENT drone; busy slot handed off. The investigator is also
    re-engaged with a follow-on task (IDLE here: no scan waypoints
    were registered; the SCAN-resume case is pinned by a separate
    test below)."""
    m = _mission_with_planner(order=['fresh'])
    inv = _drone('investigator', task=TaskType.INVESTIGATE, busy=1,
                 pose=Position(10, 10, 25))
    wit = _drone('witness', task=TaskType.IDLE, pose=Position(11, 11, 25))
    m.register_drone(inv)
    m.register_drone(wit)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v
    sub = m.ensure_sub_mission(v)
    sub.witness_drone = 'witness'

    out = m.on_task_completed(None, _completed('investigator',
                                               TaskType.INVESTIGATE,
                                               victim_id=1))

    # The saga now returns BOTH the witness's CONFIRM and the
    # investigator's follow-on. The follow-on is order-independent;
    # assert by drone_name rather than positional index.
    assert len(out) == 2
    by_drone = {t.drone_name: t for t in out}
    assert 'witness' in by_drone
    assert by_drone['witness'].task_type == int(TaskType.CONFIRM)
    assert 'investigator' in by_drone
    # No waypoints registered for this drone → follow-on is IDLE
    # (the SCAN-resume branch is exercised by the dedicated
    # regression test below).
    assert by_drone['investigator'].task_type == int(TaskType.IDLE)
    # busy slot handed off
    assert inv.busy_with_victim is None
    assert wit.busy_with_victim == 1


def test_on_completed_investigate_falls_back_when_witness_unavailable():
    """Cached witness now busy → fresh top-bid (excluding investigator)."""
    m = _mission_with_planner(order=['peer'])
    inv = _drone('investigator', task=TaskType.INVESTIGATE, busy=1,
                 pose=Position(10, 10, 25))
    stale = _drone('stale', task=TaskType.INVESTIGATE, busy=9)  # busy
    peer = _drone('peer', task=TaskType.IDLE, pose=Position(12, 12, 25))
    for d in (inv, stale, peer):
        m.register_drone(d)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v
    m.ensure_sub_mission(v).witness_drone = 'stale'

    out = m.on_task_completed(None, _completed('investigator',
                                               TaskType.INVESTIGATE,
                                               victim_id=1))

    assert out[0].drone_name == 'peer'


def test_on_completed_investigate_reuses_investigator_when_alone():
    """No other drone available → CONFIRM reuses the investigator."""
    m = _mission_with_planner(order=[])   # auction yields nobody else
    inv = _drone('investigator', task=TaskType.INVESTIGATE, busy=1,
                 pose=Position(10, 10, 25))
    m.register_drone(inv)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v
    m.ensure_sub_mission(v).witness_drone = None

    out = m.on_task_completed(None, _completed('investigator',
                                               TaskType.INVESTIGATE,
                                               victim_id=1))

    assert out[0].drone_name == 'investigator'
    assert out[0].task_type == int(TaskType.CONFIRM)


def test_investigator_gets_scan_resume_after_investigate():
    """Regression: investigator with unvisited waypoints must receive a
    SCAN tail alongside the witness's CONFIRM. Without this, the
    investigator hovers at the investigation point forever and the
    greedy auction keeps re-picking it for nearby candidates (the
    live-mission "drone2 stuck investigating" loop)."""
    m = _mission_with_planner(order=['fresh'])
    # Investigator has scan waypoints registered; sector NOT exhausted.
    inv = _drone(
        'investigator', task=TaskType.INVESTIGATE, busy=1,
        pose=Position(10, 10, 25),
        waypoints=[Position(11, 11, 25), Position(20, 20, 25),
                   Position(50, 50, 25)],
        cursor=0,
    )
    wit = _drone('witness', task=TaskType.IDLE, pose=Position(11, 11, 25))
    m.register_drone(inv)
    m.register_drone(wit)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v
    m.ensure_sub_mission(v).witness_drone = 'witness'

    out = m.on_task_completed(None, _completed('investigator',
                                               TaskType.INVESTIGATE,
                                               victim_id=1))

    by_drone = {t.drone_name: t for t in out}
    assert by_drone['witness'].task_type == int(TaskType.CONFIRM)
    # Investigator gets a SCAN; must NOT be left hovering.
    follow_on = by_drone['investigator']
    assert follow_on.task_type == int(TaskType.SCAN)
    # And the cursor was advanced to the NEAREST unvisited waypoint
    # from the investigation point (11,11), not blindly to cursor=0.
    assert inv.scan_cursor == 0   # (11,11) is closest already at idx 0
    assert len(follow_on.waypoints) == 3
    # First waypoint of the dispatched tail is the nearest one.
    assert follow_on.waypoints[0] == (11.0, 11.0, 25.0)


def test_investigator_gets_idle_when_sector_exhausted():
    """If the investigator's sector is exhausted, the follow-on is IDLE
    rather than silently nothing. Closes the "scan_cursor at end +
    INVESTIGATE done → returns ()" latent gap that would leave the
    drone hovering once its sector finished mid-mission."""
    m = _mission_with_planner(order=['fresh'])
    inv = _drone(
        'investigator', task=TaskType.INVESTIGATE, busy=1,
        pose=Position(10, 10, 25),
        waypoints=[Position(0, 0, 25)], cursor=1,   # exhausted
    )
    wit = _drone('witness', task=TaskType.IDLE, pose=Position(11, 11, 25))
    m.register_drone(inv)
    m.register_drone(wit)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v
    m.ensure_sub_mission(v).witness_drone = 'witness'

    out = m.on_task_completed(None, _completed('investigator',
                                               TaskType.INVESTIGATE,
                                               victim_id=1))

    by_drone = {t.drone_name: t for t in out}
    assert by_drone['witness'].task_type == int(TaskType.CONFIRM)
    assert by_drone['investigator'].task_type == int(TaskType.IDLE)


def test_investigator_alone_returns_only_confirm_no_parallel_scan():
    """Degenerate case: no other drone is eligible, so the witness IS
    the investigator and the CONFIRM is handed back to the same drone.
    In that case the saga must NOT also dispatch a parallel SCAN/IDLE
    to that drone (it is now busy with the CONFIRM). This pins that the
    dual-task return is gated on confirmer != investigator."""
    m = _mission_with_planner(order=[])   # no witness available
    inv = _drone(
        'investigator', task=TaskType.INVESTIGATE, busy=1,
        pose=Position(10, 10, 25),
        waypoints=[Position(20, 20, 25), Position(30, 30, 25)],
        cursor=0,
    )
    m.register_drone(inv)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v
    m.ensure_sub_mission(v).witness_drone = None

    out = m.on_task_completed(None, _completed('investigator',
                                               TaskType.INVESTIGATE,
                                               victim_id=1))

    # Exactly one task: the CONFIRM, addressed to the investigator.
    assert len(out) == 1
    assert out[0].drone_name == 'investigator'
    assert out[0].task_type == int(TaskType.CONFIRM)


def test_confirm_with_exhausted_sector_dispatches_idle():
    """The CONFIRM branch had the same latent gap as INVESTIGATE:
    scan_cursor at end after CONFIRM completion silently returned
    ``()`` instead of dispatching IDLE. This test pins the symmetric
    fix."""
    m = _mission_with_planner(order=[])
    d = _drone(
        'confirmer', task=TaskType.CONFIRM, busy=1,
        pose=Position(40, 40, 25),
        waypoints=[Position(0, 0, 25)], cursor=1,   # exhausted
    )
    m.register_drone(d)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v

    out = m.on_task_completed(None, _completed('confirmer',
                                               TaskType.CONFIRM,
                                               victim_id=1))

    assert len(out) == 1
    assert out[0].task_type == int(TaskType.IDLE)
    assert v.stage == VictimStage.CONFIRMED


def test_on_completed_confirm_marks_victim_confirmed_and_frees_drone():
    m = _mission_with_planner(order=[])
    d = _drone('confirmer', task=TaskType.CONFIRM, busy=1,
               pose=Position(10, 10, 25))
    m.register_drone(d)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v

    m.on_task_completed(None, _completed('confirmer', TaskType.CONFIRM,
                                         victim_id=1))

    assert v.stage == VictimStage.CONFIRMED
    assert d.busy_with_victim is None


def test_on_completed_scan_advances_cursor_to_nearest_after_confirm():
    """After CONFIRM, resume SCAN at the nearest unvisited waypoint."""
    m = _mission_with_planner(order=[])
    d = _drone('d1', task=TaskType.CONFIRM, busy=1,
               pose=Position(40, 40, 25),
               waypoints=[Position(0, 0, 25), Position(41, 41, 25),
                          Position(90, 90, 25)], cursor=0)
    m.register_drone(d)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v

    out = m.on_task_completed(None, _completed('d1', TaskType.CONFIRM,
                                               victim_id=1))

    # nearest unvisited to (40,40) is index 1 (41,41) → cursor advances
    assert d.scan_cursor == 1
    assert out[0].task_type == int(TaskType.SCAN)
    assert len(out[0].waypoints) == 2   # from index 1


def test_on_completed_scan_dispatches_remaining_tail():
    m = _mission_with_planner(order=[])
    d = _drone('d1', task=TaskType.SCAN, pose=Position(0, 0, 25),
               waypoints=[Position(0, 0, 25), Position(5, 5, 25)], cursor=0)
    m.register_drone(d)

    out = m.on_task_completed(None, _completed('d1', TaskType.SCAN))

    # SCAN-completed sets cursor to end → no remaining → IDLE
    assert d.scan_cursor == 2
    assert len(out) == 1
    assert out[0].task_type == int(TaskType.IDLE)


def test_on_completed_scan_dispatches_idle_when_exhausted():
    m = _mission_with_planner(order=[])
    d = _drone('d1', task=TaskType.SCAN, pose=Position(0, 0, 25),
               waypoints=[Position(0, 0, 25)], cursor=1)
    m.register_drone(d)

    out = m.on_task_completed(None, _completed('d1', TaskType.SCAN))

    assert out[0].task_type == int(TaskType.IDLE)


# replan (failure)

def test_replan_after_investigate_failure_reverts_victim_to_detected():
    m = _mission_with_planner(order=[])
    d = _drone('d1', task=TaskType.INVESTIGATE, busy=1,
               pose=Position(10, 10, 25))
    m.register_drone(d)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v
    sub = m.ensure_sub_mission(v)
    sub.stage = VictimStage.INVESTIGATING

    out = m.replan(None, _completed('d1', TaskType.INVESTIGATE, victim_id=1))

    assert v.stage == VictimStage.DETECTED
    assert v.assigned_drone is None
    assert d.busy_with_victim is None
    assert out == ()        # no remaining scan waypoints


def test_replan_after_failure_resumes_scan_when_waypoints_remain():
    m = _mission_with_planner(order=[])
    d = _drone('d1', task=TaskType.INVESTIGATE, busy=1,
               pose=Position(10, 10, 25),
               waypoints=[Position(5, 5, 25), Position(6, 6, 25)], cursor=0)
    m.register_drone(d)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v
    m.ensure_sub_mission(v).stage = VictimStage.INVESTIGATING

    out = m.replan(None, _completed('d1', TaskType.INVESTIGATE, victim_id=1))

    assert len(out) == 1
    assert out[0].task_type == int(TaskType.SCAN)
    assert len(out[0].waypoints) == 2


# tick

class _EmitRecorder:
    def __init__(self):
        self.events = []

    def __call__(self, event_type, **kwargs):
        self.events.append((event_type, kwargs))

    def types(self):
        return [e[0] for e in self.events]


def _ticking_mission(*, batch=False, order=(), reject_age=60.0,
                     timeout=600.0, watchdog=30.0, start_sec=0.0):
    from drone_rescue_coordination.lib.domain.state_machines import (
        MissionStage,
    )
    m = _mission_with_planner(order=order, owner=None, cap=4)
    # A ticking mission is past survey-start; SCANNING is the stage the
    # completion transitions (MISSION_COMPLETE / MISSION_TIMEOUT) are
    # legal from.
    m.stage = MissionStage.SCANNING
    m._strategy_is_batch = batch
    m.reject_age_seconds = reject_age
    m.mission_timeout_seconds = timeout
    m.task_status_timeout_seconds = watchdog
    m.mission_start_sec = start_sec
    rec = _EmitRecorder()
    m._emit_event = rec
    return m, rec


def test_tick_decays_stale_candidate_to_rejected():
    m, rec = _ticking_mission(reject_age=30.0, start_sec=0.0)
    v = _victim(1, stage=VictimStage.DETECTED)
    v.last_update_sec = 0.0
    m.victims[1] = v

    m.tick(now_sec=100.0)   # 100 > 30 reject age

    assert v.stage == VictimStage.REJECTED
    assert 'CANDIDATE_REJECTED' in rec.types()


def test_tick_does_not_decay_fresh_candidate():
    m, rec = _ticking_mission(reject_age=30.0, start_sec=0.0)
    v = _victim(1, stage=VictimStage.DETECTED)
    v.last_update_sec = 95.0
    m.victims[1] = v

    m.tick(now_sec=100.0)   # 5s < 30 reject age

    assert v.stage == VictimStage.DETECTED
    assert 'CANDIDATE_REJECTED' not in rec.types()


def test_tick_emits_task_timeout_on_silence():
    from drone_rescue_coordination.lib.domain.value_objects import WatchdogClock
    m, rec = _ticking_mission(watchdog=30.0, start_sec=0.0)
    d = _drone('d1', task=TaskType.SCAN, pose=Position(0, 0, 25))
    d.clock = WatchdogClock(last_status_t=0.0, task_dispatched_t=0.0)
    m.register_drone(d)

    m.tick(now_sec=100.0)   # silent 100s > 30s watchdog

    assert 'TASK_TIMEOUT' in rec.types()
    # clock reset so it won't re-fire immediately
    assert d.clock.last_status_t == 100.0


def test_tick_mission_complete_when_all_idle_and_no_pending():
    m, rec = _ticking_mission(start_sec=0.0)
    m.register_drone(_drone('d1', task=TaskType.IDLE, pose=Position(0, 0, 25)))
    # a confirmed victim (not pending), no busy drones
    m.victims[1] = _victim(1, stage=VictimStage.CONFIRMED)
    m.ensure_sub_mission(m.victims[1]).stage = VictimStage.CONFIRMED

    m.tick(now_sec=50.0)

    assert 'MISSION_COMPLETE' in rec.types()
    assert m._published_complete is True


def test_tick_mission_timeout_when_elapsed_exceeds_budget():
    m, rec = _ticking_mission(timeout=600.0, start_sec=0.0)
    # a still-pending victim + a busy drone → not "complete", so the
    # timeout branch fires once elapsed passes the budget
    m.register_drone(_drone('d1', task=TaskType.INVESTIGATE, busy=1,
                            pose=Position(0, 0, 25)))
    m.victims[1] = _victim(1, stage=VictimStage.INVESTIGATING)

    m.tick(now_sec=700.0)   # 700 > 600 timeout

    assert 'MISSION_TIMEOUT' in rec.types()
    assert m._published_complete is True


# recovery callbacks

def test_on_drone_health_marks_down_and_reassigns():
    m = _mission_with_planner(order=[], owner=None)
    dead = _drone('dead', waypoints=[Position(50, 50, 25)], cursor=0)
    receiver = _drone('receiver', task=TaskType.SCAN,
                      pose=Position(48, 48, 25),
                      waypoints=[Position(40, 40, 25)], cursor=0)
    m.register_drone(dead)
    m.register_drone(receiver)

    out = m.on_drone_health('dead', unrecoverable=True, now_sec=10.0)

    assert dead.is_down is True
    # receiver mid-SCAN → preempting SCAN task returned
    assert len(out) == 1
    assert out[0].drone_name == 'receiver'
    assert len(receiver.scan_waypoints) == 2   # own + orphan


def test_on_drone_health_idempotent_for_already_down():
    m = _mission_with_planner(order=[], owner=None)
    d = _drone('d1', is_down=True, waypoints=[Position(5, 5, 25)])
    m.register_drone(d)
    assert m.on_drone_health('d1', unrecoverable=True, now_sec=10.0) == ()


def test_on_drone_health_noop_when_recoverable():
    m = _mission_with_planner(order=[], owner=None)
    d = _drone('d1', waypoints=[Position(5, 5, 25)])
    m.register_drone(d)
    assert m.on_drone_health('d1', unrecoverable=False, now_sec=10.0) == ()
    assert d.is_down is False


def test_on_battery_low_marks_not_ok_and_returns_rth():
    m = _mission_with_planner(order=[], owner=None)
    d = _drone('d1', task=TaskType.SCAN, pose=Position(10, 10, 25))
    m.register_drone(d)

    out = m.on_battery_low('d1')

    assert d.battery_ok is False
    assert len(out) == 1
    assert out[0].drone_name == 'd1'
    assert out[0].task_type == int(TaskType.RTH)


def test_on_battery_low_idempotent_when_already_not_ok():
    m = _mission_with_planner(order=[], owner=None)
    d = _drone('d1')
    d.battery_ok = False
    m.register_drone(d)
    assert m.on_battery_low('d1') == ()


# on_candidate / on_task_status

def _incoming_candidate(cid, *, conf=0.95, confirmed=False, x=10.0, y=10.0):
    from drone_rescue_coordination.lib.domain.incoming import IncomingCandidate
    return IncomingCandidate(
        candidate_id=cid, position=Position(x, y, 0.0),
        confidence=conf, confirmed=confirmed,
    )


def _incoming_status(drone_name, task_id, status, detail=''):
    from drone_rescue_coordination.lib.domain.incoming import IncomingTaskStatus
    return IncomingTaskStatus(
        drone_name=drone_name, task_id=task_id, status=status, detail=detail,
    )


def test_on_candidate_registers_new_victim_and_emits():
    m, rec = _ticking_mission(order=['d1'])
    m.register_drone(_drone('d1', task=TaskType.IDLE, pose=Position(0, 0, 25)))

    m.on_candidate(_incoming_candidate(1, conf=0.95), now_sec=10.0)

    assert 1 in m.victims
    assert 'CANDIDATE_DETECTED' in rec.types()
    # DETECTED + drone available → INVESTIGATE dispatched
    assert 'INVESTIGATE_DISPATCHED' in rec.types()


def test_on_candidate_auto_confirms_when_confirmed_upstream():
    m, rec = _ticking_mission(order=['d1'])
    m.register_drone(_drone('d1', task=TaskType.IDLE, pose=Position(0, 0, 25)))

    out = m.on_candidate(_incoming_candidate(1, confirmed=True), now_sec=10.0)

    assert m.victims[1].stage == VictimStage.CONFIRMED
    assert m.sub_missions[1].stage == VictimStage.CONFIRMED
    assert 'VICTIM_CONFIRMED' in rec.types()
    assert out == ()        # no INVESTIGATE when auto-confirmed


def test_on_candidate_updates_existing_victim():
    m, rec = _ticking_mission(order=[])
    v = _victim(1, conf=0.5, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v
    m.ensure_sub_mission(v)

    m.on_candidate(_incoming_candidate(1, conf=0.8, x=99, y=99), now_sec=20.0)

    assert m.victims[1].confidence == 0.8
    assert m.victims[1].position.x == 99
    assert m.victims[1].last_update_sec == 20.0
    # already INVESTIGATING → not re-dispatched
    assert 'INVESTIGATE_DISPATCHED' not in rec.types()


def test_on_task_status_completed_routes_to_on_task_completed():
    m, rec = _ticking_mission(order=[])
    d = _drone('d1', task=TaskType.SCAN, pose=Position(0, 0, 25),
               waypoints=[Position(5, 5, 25)], cursor=0)
    d.current_task_id = 7
    m.register_drone(d)

    out = m.on_task_status(_incoming_status('d1', 7, 2), now_sec=30.0)  # COMPLETED

    # SCAN completed → cursor to end → IDLE follow-up
    assert d.scan_cursor == 1
    assert out[0].task_type == int(TaskType.IDLE)


def test_on_task_status_failed_routes_to_replan():
    m, rec = _ticking_mission(order=[])
    d = _drone('d1', task=TaskType.INVESTIGATE, busy=1,
               pose=Position(10, 10, 25),
               waypoints=[Position(5, 5, 25)], cursor=0)
    d.current_task_id = 9
    m.register_drone(d)
    v = _victim(1, stage=VictimStage.INVESTIGATING)
    m.victims[1] = v
    m.ensure_sub_mission(v).stage = VictimStage.INVESTIGATING

    out = m.on_task_status(_incoming_status('d1', 9, 3), now_sec=40.0)  # FAILED

    assert v.stage == VictimStage.DETECTED      # compensated
    assert out[0].task_type == int(TaskType.SCAN)   # resume scan


def test_on_task_status_ignores_stale_task_id():
    m, rec = _ticking_mission(order=[])
    d = _drone('d1', task=TaskType.SCAN, pose=Position(0, 0, 25))
    d.current_task_id = 5
    m.register_drone(d)

    # status for a different (preempted) task_id → ignored
    out = m.on_task_status(_incoming_status('d1', 999, 2), now_sec=50.0)
    assert out == ()


def test_on_task_status_in_progress_advances_scan_cursor():
    from drone_rescue_coordination.lib.domain.value_objects import WatchdogClock
    m, rec = _ticking_mission(order=[])
    d = _drone('d1', task=TaskType.SCAN, pose=Position(0, 0, 25),
               waypoints=[Position(i, i, 25) for i in range(10)], cursor=0)
    d.current_task_id = 3
    d.clock = WatchdogClock(last_dispatch_offset=2)
    m.register_drone(d)

    m.on_task_status(_incoming_status('d1', 3, 1, detail='wp=4'),
                     now_sec=60.0)  # IN_PROGRESS

    # absolute cursor = dispatch_offset (2) + local idx (4) = 6
    assert d.scan_cursor == 6


# begin_scan (coverage assignment)

def test_begin_scan_assigns_plans_and_returns_scan_tasks():
    from types import SimpleNamespace
    from drone_rescue_coordination.lib.domain.state_machines import MissionStage
    from drone_rescue_coordination.lib.domain.value_objects import ScanPlan

    m = _mission_with_planner(order=[])
    m.stage = MissionStage.DEPLOYING   # SURVEY_STARTED is legal from here
    m.register_drone(_drone('d1', task=TaskType.IDLE))
    m.register_drone(_drone('d2', task=TaskType.IDLE))
    plan1 = ScanPlan(waypoints=((0.0, 0.0), (10.0, 0.0)),
                     wedge=SectorWedge(0.0, math.pi / 2))
    plan2 = ScanPlan(waypoints=((0.0, 10.0),), wedge=None)
    coverage = SimpleNamespace(per_drone=[plan1, plan2])

    out = m.begin_scan(coverage, elevation_at=lambda x, y: 5.0,
                       survey_altitude=25.0)

    # two SCAN tasks, one per drone
    assert len(out) == 2
    assert all(t.task_type == int(TaskType.SCAN) for t in out)
    # d1 got plan1's 2 waypoints with z = 25 + 5 = 30
    assert len(m.drones['d1'].scan_waypoints) == 2
    assert m.drones['d1'].scan_waypoints[0].z == 30.0
    # d1 got the angular wedge; d2 got None (non-angular)
    assert m.drones['d1'].sector_wedge == SectorWedge(0.0, math.pi / 2)
    assert m.drones['d2'].sector_wedge is None
    assert m.sectors_total == 2
    # mission transitioned to SCANNING
    assert m.stage == MissionStage.SCANNING


def test_begin_scan_skips_unknown_drone_in_order():
    from types import SimpleNamespace
    from drone_rescue_coordination.lib.domain.state_machines import MissionStage
    from drone_rescue_coordination.lib.domain.value_objects import ScanPlan

    m = _mission_with_planner(order=[])
    m.stage = MissionStage.DEPLOYING
    m.register_drone(_drone('d1', task=TaskType.IDLE))
    coverage = SimpleNamespace(
        per_drone=[ScanPlan(waypoints=((1.0, 1.0),), wedge=None)])

    out = m.begin_scan(coverage, elevation_at=lambda x, y: 0.0,
                       survey_altitude=25.0, drone_order=['ghost'])

    # 'ghost' not registered → skipped, no tasks
    assert out == ()
