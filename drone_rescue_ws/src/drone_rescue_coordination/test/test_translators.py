"""Round-trip tests for the ROS-msg <-> domain-VO translators.

Every public field of each wire message is covered. These tests are
not pure-Python (they import ROS msg types) but they don't need
rclpy.init(), so they run in <100 ms.
"""

from __future__ import annotations

from geometry_msgs.msg import Point
from drone_rescue_msgs.msg import (
    VictimCandidate, TaskStatus, DroneHealth, TaskAssignment,
)

from drone_rescue_coordination.lib.domain.value_objects import (
    OutgoingTask, Position,
)
from drone_rescue_coordination.lib.ros_adapter.translators import (
    from_ros_candidate, from_ros_health, from_ros_task_status,
    point_from_position, position_from_point, to_ros_task_assignment,
)


def test_point_position_round_trip():
    p = Point()
    p.x, p.y, p.z = 1.5, -2.25, 3.0
    pos = position_from_point(p)
    assert pos == Position(1.5, -2.25, 3.0)
    p2 = point_from_position(pos)
    assert (p2.x, p2.y, p2.z) == (1.5, -2.25, 3.0)


def test_from_ros_candidate_full_fields():
    msg = VictimCandidate()
    msg.candidate_id = 42
    msg.position.x = 10.0
    msg.position.y = 20.0
    msg.position.z = 0.5
    msg.confidence = 0.82
    msg.observation_count = 5
    msg.reporting_drones = ['drone1', 'drone2']
    msg.confirmed = True
    inc = from_ros_candidate(msg)
    assert inc.candidate_id == 42
    assert inc.position == Position(10.0, 20.0, 0.5)
    assert inc.confidence == 0.82
    assert inc.observation_count == 5
    assert inc.reporting_drones == ('drone1', 'drone2')
    assert inc.confirmed is True


def test_from_ros_task_status_full_fields():
    msg = TaskStatus()
    msg.drone_name = 'drone3'
    msg.task_id = 7
    msg.status = 2
    msg.detail = 'done'
    inc = from_ros_task_status(msg)
    assert inc.drone_name == 'drone3'
    assert inc.task_id == 7
    assert inc.status == 2
    assert inc.detail == 'done'


def test_from_ros_health_full_fields():
    msg = DroneHealth()
    msg.drone_name = 'drone4'
    msg.anomaly_score = 0.6
    msg.reason = 'imu_spike'
    msg.unrecoverable = True
    msg.battery_remaining_s = 30.0
    inc = from_ros_health(msg, battery_rth_threshold_s=60.0)
    assert inc.drone_name == 'drone4'
    assert inc.anomaly_score == 0.6
    assert inc.is_down is True
    assert inc.reason == 'imu_spike'
    assert inc.battery_remaining_s == 30.0
    assert inc.battery_ok is False    # 30 < 60 → not OK


def test_from_ros_health_default_threshold_means_ok():
    msg = DroneHealth()
    msg.drone_name = 'drone5'
    msg.battery_remaining_s = 1.0
    inc = from_ros_health(msg)        # threshold=0 → battery_ok=True always
    assert inc.battery_ok is True


def test_to_ros_task_assignment_round_trip():
    t = OutgoingTask(
        drone_name='drone1',
        task_type=1,
        waypoints=((1.0, 2.0, 0.0), (3.0, 4.0, 0.0)),
        target=(10.0, 20.0, 5.0),
        victim_id=42,
        priority=2,
        hover_seconds=4.0,
        confirm_orbit_radius=4.0,
    )
    msg = to_ros_task_assignment(t)
    assert msg.drone_name == 'drone1'
    assert msg.task_type == 1
    assert msg.victim_id == 42
    assert msg.priority == 2
    assert msg.hover_seconds == 4.0
    assert msg.confirm_orbit_radius == 4.0
    assert (msg.target_point.x, msg.target_point.y, msg.target_point.z) == (10.0, 20.0, 5.0)
    assert len(msg.waypoints) == 2
    assert (msg.waypoints[0].x, msg.waypoints[0].y) == (1.0, 2.0)


def test_to_ros_task_assignment_carries_investigate_fields():
    """The multi-view INVESTIGATE plan (radius / dwell / angle set)
    the L3 layer stamped on the OutgoingTask survives onto the wire."""
    import math
    angles = (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)
    t = OutgoingTask(
        drone_name='drone2', task_type=1, waypoints=(),
        target=(10.0, 20.0, 5.0), victim_id=7, priority=2,
        hover_seconds=4.0, investigate_radius=6.5, dwell_s=3.0,
        investigate_angles=angles,
    )
    msg = to_ros_task_assignment(t)
    assert msg.investigate_radius == 6.5
    assert msg.dwell_s == 3.0
    assert [round(a, 5) for a in msg.investigate_angles] == \
        [round(a, 5) for a in angles]


def test_to_ros_task_assignment_investigate_fields_default_unset():
    """A non-INVESTIGATE task leaves the fields at the 0.0 / empty
    sentinels so the executor falls back to its own config."""
    t = OutgoingTask(
        drone_name='drone1', task_type=0,
        waypoints=((1.0, 2.0, 0.0),), target=None,
        victim_id=0, priority=1, hover_seconds=0.0,
    )
    msg = to_ros_task_assignment(t)
    assert msg.investigate_radius == 0.0
    assert msg.dwell_s == 0.0
    assert list(msg.investigate_angles) == []


# outbound converters

def test_to_ros_drone_peer_state_carries_fields():
    """Outbound converter is the symmetric half of the anti-corruption
    layer."""
    from drone_rescue_coordination.lib.ros_adapter.translators import (
        to_ros_drone_peer_state,
    )
    msg = to_ros_drone_peer_state(
        drone_name='drone3',
        pose=Position(5.0, 10.0, 25.0),
        battery=0.72,
        task_type=1,
        current_task_id=7,
        busy_with_victim=42,
        is_down=False,
        wp_index=12,
        wp_total=85,
    )
    assert msg.drone_name == 'drone3'
    assert msg.pose.position.x == 5.0
    assert msg.pose.position.y == 10.0
    assert msg.pose.position.z == 25.0
    assert msg.battery == 0.72
    assert msg.task_type == 1
    assert msg.current_task_id == 7
    assert msg.busy_with_victim == 42
    assert msg.is_down is False
    assert msg.wp_index == 12
    assert msg.wp_total == 85


def test_to_ros_task_status_carries_fields():
    from drone_rescue_coordination.lib.ros_adapter.translators import (
        to_ros_task_status,
    )
    msg = to_ros_task_status(
        drone_name='drone2',
        task_id=99,
        status=2,
        detail='done',
    )
    assert msg.drone_name == 'drone2'
    assert msg.task_id == 99
    assert msg.status == 2
    assert msg.detail == 'done'
