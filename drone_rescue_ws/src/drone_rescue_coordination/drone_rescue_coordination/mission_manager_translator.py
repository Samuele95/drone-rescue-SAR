"""Per-ROS-callback translation layer for the new MissionManager node.

The deconstruction's anti-corruption layer.
Translates incoming ROS messages into domain VOs, calls into the
`MissionPort` (the live `Mission` aggregate), and converts the
returned `Sequence[OutgoingTask]` back into `TaskAssignment` messages
ready to publish.

Pure-function module: no rclpy state lives here; the node owns
publishers and subscribers and delegates the per-callback work to
these functions. Each function takes (port, ros_msg) and returns
`Sequence[TaskAssignment]` so the caller decides how to publish.

This module is consumed by `mission_manager_node.MissionManagerNode`
when the `USE_LEGACY_MISSION_MANAGER` feature flag is off. Until the
saga migration completes, the new node calls the legacy
`MissionManager` directly and these functions exist as the typed
seams the follow-up commits will switch over to.
"""

from __future__ import annotations

from typing import Sequence

from drone_rescue_msgs.msg import (
    DroneHealth as RosDroneHealth,
    TaskAssignment as RosTaskAssignment,
    TaskStatus as RosTaskStatus,
    VictimCandidate as RosVictimCandidate,
)

from drone_rescue_coordination.lib.ports.mission_port import MissionPort
from drone_rescue_coordination.lib.ros_adapter.translators import (
    from_ros_candidate, from_ros_health, from_ros_task_status,
    to_ros_task_assignment,
)


def handle_candidate(
    port: MissionPort, msg: RosVictimCandidate,
) -> Sequence[RosTaskAssignment]:
    """ROS msg → domain VO → port → ROS msgs out."""
    inc = from_ros_candidate(msg)
    return [to_ros_task_assignment(t) for t in port.on_candidate(inc)]


def handle_task_status(
    port: MissionPort, msg: RosTaskStatus,
) -> Sequence[RosTaskAssignment]:
    inc = from_ros_task_status(msg)
    return [to_ros_task_assignment(t) for t in port.on_task_status(inc)]


def handle_health(
    port: MissionPort, msg: RosDroneHealth,
    *,
    battery_rth_threshold_s: float = 0.0,
) -> Sequence[RosTaskAssignment]:
    inc = from_ros_health(msg, battery_rth_threshold_s=battery_rth_threshold_s)
    return [to_ros_task_assignment(t) for t in port.on_health(inc)]


def handle_tick(
    port: MissionPort, now_sec: float,
) -> Sequence[RosTaskAssignment]:
    return [to_ros_task_assignment(t) for t in port.tick(now_sec)]
