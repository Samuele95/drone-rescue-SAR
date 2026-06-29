"""ROS-message <-> domain-VO translators.

The anti-corruption layer of the hexagonal carve-out. `lib/domain/`
and `lib/ports/` never import `drone_rescue_msgs.msg.*` or
`geometry_msgs.msg.*`. The adapter (this module) is the only
translator. `mission_manager_node.py` calls into these functions
when wiring subscriptions to port callbacks.

Tests for the translators live in the coordination package's test/
tree and round-trip every public field of each wire message through
a translator and back.
"""

from __future__ import annotations

import math

from drone_rescue_msgs.msg import (
    DronePeerState, DroneHealth, TaskAssignment, TaskStatus, VictimCandidate,
)
from geometry_msgs.msg import Point

from ..domain.incoming import (
    IncomingCandidate, IncomingHealth, IncomingTaskStatus,
)
from ..domain.value_objects import OutgoingTask, Position


def position_from_point(p: Point) -> Position:
    """Lossless `geometry_msgs.msg.Point` -> `Position`."""
    return Position(x=float(p.x), y=float(p.y), z=float(p.z))


def point_from_position(p: Position) -> Point:
    """Inverse: used by the adapter when populating an outgoing ROS msg."""
    out = Point()
    out.x = float(p.x)
    out.y = float(p.y)
    out.z = float(p.z)
    return out


def from_ros_candidate(msg: VictimCandidate) -> IncomingCandidate:
    return IncomingCandidate(
        candidate_id=int(msg.candidate_id),
        position=position_from_point(msg.position),
        confidence=float(getattr(msg, 'confidence', 0.0)),
        observation_count=int(getattr(msg, 'observation_count', 0)),
        reporting_drones=tuple(str(s) for s in getattr(msg, 'reporting_drones', ())),
        confirmed=bool(getattr(msg, 'confirmed', False)),
    )


def from_ros_task_status(msg: TaskStatus) -> IncomingTaskStatus:
    return IncomingTaskStatus(
        drone_name=str(msg.drone_name),
        task_id=int(msg.task_id),
        status=int(msg.status),
        detail=str(getattr(msg, 'detail', '')),
    )


def from_ros_health(
    msg: DroneHealth,
    *,
    battery_rth_threshold_s: float = 0.0,
) -> IncomingHealth:
    """Translate health snapshot. `battery_ok` is true when
    `battery_remaining_s` is unknown (NaN) or >= the RTH threshold; the
    adapter passes the threshold from the mission_manager parameter so
    the domain doesn't have to know it."""
    rem = float(getattr(msg, 'battery_remaining_s', float('nan')))
    if battery_rth_threshold_s <= 0.0 or math.isnan(rem):
        battery_ok = True
    else:
        battery_ok = rem >= battery_rth_threshold_s
    return IncomingHealth(
        drone_name=str(msg.drone_name),
        anomaly_score=float(getattr(msg, 'anomaly_score', 0.0)),
        is_down=bool(getattr(msg, 'unrecoverable', False)),
        reason=str(getattr(msg, 'reason', '')),
        battery_remaining_s=rem,
        battery_ok=battery_ok,
    )


def to_ros_task_assignment(t: OutgoingTask) -> TaskAssignment:
    """`OutgoingTask` -> `TaskAssignment` wire message.

    Drives the adapter when the Mission aggregate's tick returns a
    sequence of outgoing tasks.
    """
    msg = TaskAssignment()
    msg.drone_name = t.drone_name
    msg.task_id = 0
    msg.task_type = int(t.task_type)
    msg.victim_id = int(t.victim_id)
    msg.priority = int(t.priority)
    msg.hover_seconds = float(t.hover_seconds)
    msg.confirm_orbit_radius = float(t.confirm_orbit_radius)
    # multi-view INVESTIGATE plan (empty/0.0 means executor defaults).
    msg.investigate_radius = float(t.investigate_radius)
    msg.dwell_s = float(t.dwell_s)
    msg.investigate_angles = [float(a) for a in t.investigate_angles]
    msg.waypoints = [
        _point_from_xyz(x, y, z) for (x, y, z) in t.waypoints
    ]
    if t.target is not None:
        msg.target_point = _point_from_xyz(*t.target)
    return msg


def _point_from_xyz(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


# Outgoing converters. The inbound side (from_ros_*) is dense; outbound
# has been sparse beyond `to_ros_task_assignment`. These helpers make
# the anti-corruption layer fully symmetric so the Mission aggregate /
# Executor aggregate can stay rclpy-free while the adapter takes the
# domain VO and produces the wire-format ROS message.

def to_ros_drone_peer_state(
    *,
    drone_name: str,
    pose: Position,
    battery: float,
    task_type: int,
    current_task_id: int = 0,
    busy_with_victim: int = 0,
    is_down: bool = False,
    wp_index: int = 0,
    wp_total: int = 0,
) -> DronePeerState:
    """Build a `DronePeerState` from typed domain primitives.

    The legacy drone_executor builds this inline; once the domain
    Executor aggregate lands it returns the typed fields and the
    adapter converts at the boundary.
    """
    msg = DronePeerState()
    msg.drone_name = drone_name
    msg.pose.position.x = float(pose.x)
    msg.pose.position.y = float(pose.y)
    msg.pose.position.z = float(pose.z)
    msg.battery = float(battery)
    msg.task_type = int(task_type)
    msg.current_task_id = int(current_task_id)
    msg.busy_with_victim = int(busy_with_victim)
    msg.is_down = bool(is_down)
    msg.wp_index = int(wp_index)
    msg.wp_total = int(wp_total)
    return msg


def to_ros_task_status(
    *,
    drone_name: str,
    task_id: int,
    status: int,
    detail: str = '',
) -> TaskStatus:
    """Build a `TaskStatus` from typed primitives.

    Status code mirrors the legacy enum (ACCEPTED=0, IN_PROGRESS=1,
    COMPLETED=2, FAILED=3, PREEMPTED=4)."""
    msg = TaskStatus()
    msg.drone_name = drone_name
    msg.task_id = int(task_id)
    msg.status = int(status)
    msg.detail = str(detail)
    return msg
