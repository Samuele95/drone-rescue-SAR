"""Sensor / output translators for the DroneExecutor.

Scaffolding only. The legacy LifecycleNode is still the runtime path
until the BT tree migrates onto ``lib/domain/executor.Executor``.
These translators define the seam the follow-up commits will switch
the publisher / subscriber sides through.
"""

from __future__ import annotations

from typing import Optional, Tuple

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

from ..domain.executor import ExecutorOutputs, ExecutorSensors
from ..domain.value_objects import Position
from .translators import point_from_position, position_from_point


def position_from_odom(msg: Odometry) -> Position:
    """Extract the drone pose from an Odometry message."""
    return position_from_point(msg.pose.pose.position)


def sensors_from_inputs(
    *,
    now_sec: float,
    pose_msg: Optional[Odometry] = None,
    lidar_min_range_m: float = float('inf'),
    current_task_type: int = 5,
    current_task_id: int = 0,
    target_msg: Optional[Point] = None,
    waypoints_msgs: Tuple[Point, ...] = (),
    is_down: bool = False,
    battery_ok: bool = True,
) -> ExecutorSensors:
    """Build an ``ExecutorSensors`` VO from raw ROS messages.

    All ROS-message inputs are optional so the test harness can
    construct partial sensor frames; the production composition
    root passes the full set on every tick.
    """
    return ExecutorSensors(
        now_sec=now_sec,
        current_pose=position_from_odom(pose_msg) if pose_msg is not None else None,
        lidar_min_range_m=float(lidar_min_range_m),
        current_task_type=int(current_task_type),
        current_task_id=int(current_task_id),
        target=position_from_point(target_msg) if target_msg is not None else None,
        waypoints=tuple(position_from_point(w) for w in waypoints_msgs),
        is_down=is_down,
        battery_ok=battery_ok,
    )


def target_pose_msg_from(outputs: ExecutorOutputs) -> Optional[Point]:
    """Inverse: outputs.target_pose -> ``geometry_msgs.Point``. Returns
    None when the tick produced no target update."""
    if outputs.target_pose is None:
        return None
    return point_from_position(outputs.target_pose)
