"""Tests for the operator-injected investigate goal.

The dashboard publishes ``/mission/operator_goal`` (PointStamped) on
a right-click in the mission scene; ``MissionManager._on_operator_goal``
mints a synthetic high-confidence VictimCandidate (cid >= 9000,
reporter 'operator') and delegates to the existing ``_on_candidate``
path, so the goal rides the same auction -> INVESTIGATE -> CONFIRM saga
as any detection. Pre-survey stage gating therefore applies unchanged.

Bare-instance pattern (no rclpy.init) per test_mission_manager_node.
"""

from __future__ import annotations

from types import SimpleNamespace

from drone_rescue_coordination.mission_manager import (
    MissionManager, OPERATOR_GOAL_CID_BASE,
)

from geometry_msgs.msg import PointStamped


def _bare_mm(active=True):
    mm = object.__new__(MissionManager)
    mm._is_active = active
    mm._next_operator_cid = OPERATOR_GOAL_CID_BASE
    mm._candidates = []
    mm._events = []
    mm._on_candidate = lambda msg: mm._candidates.append(msg)
    mm._emit_event = lambda *a, **k: mm._events.append((a, k))
    mm.get_logger = lambda: SimpleNamespace(
        info=lambda *a, **k: None, warning=lambda *a, **k: None,
    )
    return mm


def _goal(x=12.5, y=-3.0):
    msg = PointStamped()
    msg.header.frame_id = 'world'
    msg.point.x = float(x)
    msg.point.y = float(y)
    return msg


def test_operator_goal_mints_synthetic_candidate():
    mm = _bare_mm()
    mm._on_operator_goal(_goal(12.5, -3.0))
    assert len(mm._candidates) == 1
    cand = mm._candidates[0]
    assert cand.candidate_id == OPERATOR_GOAL_CID_BASE
    assert cand.position.x == 12.5 and cand.position.y == -3.0
    assert cand.confidence > 0.9
    assert not cand.confirmed          # operator goal still needs CONFIRM
    assert list(cand.reporting_drones) == ['operator']


def test_operator_goal_logs_operator_event():
    mm = _bare_mm()
    mm._on_operator_goal(_goal())
    assert any(a[0] == 'OPERATOR_GOAL' for a, k in mm._events)


def test_operator_goal_cids_increment_and_never_collide_with_clusters():
    mm = _bare_mm()
    mm._on_operator_goal(_goal(1, 1))
    mm._on_operator_goal(_goal(2, 2))
    cids = [c.candidate_id for c in mm._candidates]
    assert cids == [OPERATOR_GOAL_CID_BASE, OPERATOR_GOAL_CID_BASE + 1]


def test_operator_goal_ignored_while_inactive():
    mm = _bare_mm(active=False)
    mm._on_operator_goal(_goal())
    assert mm._candidates == []
    assert mm._events == []
