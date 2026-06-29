"""Regression tests for the saga-confirmed publication channel.

The bug: mission_manager confirmed victims via the cross-drone CONFIRM
saga and recorded the result internally, but never broadcast that fact
on any ROS topic. The visualizer consumed only ``VictimCandidate.confirmed``
(detection_filter's multi-view fusion flag), which sector-scanning
rarely sets, so saga-confirmed victims stayed orange in RViz forever.

These tests pin (a) ``_publish_saga_confirmed`` writes a UInt32 carrying
the cluster_id when a publisher is wired, (b) it is a quiet no-op when
called pre-configure (test setups that skip lifecycle), and
(c) ``_emit_completion_events`` invokes the helper exactly when a
CONFIRM task completes.

Pure pytest: no rclpy.init(); we exercise the unbound method.
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import List

from std_msgs.msg import UInt32

from drone_rescue_coordination.mission_manager import MissionManager


class _FakePublisher:
    """Stand-in for an rclpy Publisher. Records every publish call."""

    def __init__(self) -> None:
        self.calls: List[UInt32] = []

    def publish(self, msg) -> None:
        self.calls.append(msg)


def test_publish_saga_confirmed_writes_cluster_id():
    """Happy path: publisher wired, helper writes the cluster_id."""
    helper = MissionManager._publish_saga_confirmed
    pub = _FakePublisher()
    ns = SimpleNamespace(_victims_confirmed_pub=pub)
    helper(ns, 42)
    assert len(pub.calls) == 1
    msg = pub.calls[0]
    assert isinstance(msg, UInt32)
    assert msg.data == 42


def test_publish_saga_confirmed_handles_int_like_values():
    """Cluster IDs in mission_manager flow as ``int(msg.candidate_id)``,
    but the helper must also tolerate np.uint32 / np.int64 inputs since
    detection_filter sometimes hands those through."""
    helper = MissionManager._publish_saga_confirmed
    pub = _FakePublisher()
    ns = SimpleNamespace(_victims_confirmed_pub=pub)
    helper(ns, True)         # bool is an int subclass
    helper(ns, 0)             # zero is a valid cluster_id
    assert [m.data for m in pub.calls] == [1, 0]


def test_publish_saga_confirmed_is_noop_without_publisher():
    """A test setup that constructs MissionManager without going
    through on_configure will have ``_victims_confirmed_pub = None``;
    the helper must NOT raise in that case (lazy-publisher pattern)."""
    helper = MissionManager._publish_saga_confirmed
    ns = SimpleNamespace(_victims_confirmed_pub=None)
    helper(ns, 7)   # must not raise; no side effects to assert


def test_emit_completion_events_publishes_on_confirm():
    """When a CONFIRM task completes, _emit_completion_events must
    invoke _publish_saga_confirmed with the victim's cluster_id. This
    is the load-bearing wiring between the saga finish line and the
    visualizer channel."""
    from drone_rescue_msgs.msg import TaskAssignment

    pub = _FakePublisher()
    emitted_events: list = []
    helper = MissionManager._emit_completion_events
    ns = SimpleNamespace(
        _victims_confirmed_pub=pub,
        _victims={99: SimpleNamespace(
            assigned_drone='drone2',
            position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
        )},
        _emit_event=lambda *a, **kw: emitted_events.append((a, kw)),
        _publish_saga_confirmed=lambda cid: pub.publish(
            UInt32(data=int(cid))
        ),
    )
    # CONFIRM task just completed for victim 99; no follow-on tasks.
    helper(ns, TaskAssignment.CONFIRM, 99, ())
    assert len(pub.calls) == 1
    assert pub.calls[0].data == 99
    # And the VICTIM_CONFIRMED event was still emitted (we didn't
    # replace one channel with the other).
    assert any(
        args[0] == 'VICTIM_CONFIRMED'
        for (args, _kw) in emitted_events
    )


def test_emit_completion_events_skips_publish_when_not_confirm():
    """Skip the publish when the completed task wasn't CONFIRM.

    A SCAN completion (or any non-CONFIRM ttype) must not write
    anything to the saga-confirmed channel.
    """
    from drone_rescue_msgs.msg import TaskAssignment

    pub = _FakePublisher()
    helper = MissionManager._emit_completion_events
    ns = SimpleNamespace(
        _victims_confirmed_pub=pub,
        _victims={},
        _emit_event=lambda *a, **kw: None,
        _publish_saga_confirmed=lambda cid: pub.publish(
            UInt32(data=int(cid))
        ),
    )
    helper(ns, TaskAssignment.SCAN_WAYPOINTS, None, ())
    assert pub.calls == []


def test_emit_completion_events_skips_publish_when_victim_missing():
    """Skip the publish when the victim record is gone.

    If the saga records the busy_with_victim slot but the victim
    record was purged (e.g. by a rejection saga), the helper must
    not publish a stale ID nor crash.
    """
    from drone_rescue_msgs.msg import TaskAssignment

    pub = _FakePublisher()
    helper = MissionManager._emit_completion_events
    ns = SimpleNamespace(
        _victims_confirmed_pub=pub,
        _victims={},   # victim 99 has been forgotten
        _emit_event=lambda *a, **kw: None,
        _publish_saga_confirmed=lambda cid: pub.publish(
            UInt32(data=int(cid))
        ),
    )
    helper(ns, TaskAssignment.CONFIRM, 99, ())
    assert pub.calls == []
