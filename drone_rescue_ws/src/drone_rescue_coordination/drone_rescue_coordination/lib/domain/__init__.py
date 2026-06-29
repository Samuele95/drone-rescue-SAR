"""Domain layer: pure-Python entities, value objects, aggregates.

No rclpy. No PyQt. No Gazebo. No matplotlib. Anything here must run
in a unit test without ROS context.

This package is the centre of the SAR domain model. The corresponding
ports live in `lib/ports/`; the existing algorithm helpers (auction,
sector_geometry, sar_patterns, detection_cluster, bt) stay in
`lib/*.py` for now, already pure-Python and consumed by 10+ sites.

Public surface:
    Drone               (entity)
    Victim              (entity)
    SectorWedge         (frozen value object)
    ScanPlan            (frozen value object)
    Bid                 (frozen value object)
    OutgoingTask        (frozen value object) what Mission emits to its adapter
    MissionStage        (state machine enum, kept compatible with old IntEnum)
    VictimStage         (state machine enum, kept compatible)
    Mission             (aggregate root)
    VictimSubMission    (aggregate root for the saga)
    MissionStateMachine (typed transition table)
    VictimStateMachine  (typed transition table)
"""

from .entities import Drone, Victim
from .incoming import IncomingCandidate, IncomingHealth, IncomingTaskStatus
from .partition_kind import PartitionKind
from .task_type import TaskType
from .value_objects import (
    Position, WatchdogClock,
    SectorWedge, ScanPlan, Bid, OutgoingTask, MissionStateSnapshot,
)
from .state_machines import (
    MissionStage, VictimStage,
    MissionStateMachine, VictimStateMachine,
    TransitionEvent, IllegalTransition,
)
from .victim_sub_mission import VictimSubMission
from .mission import Mission
from .events import (
    MissionEventVariant,
    CandidateDetected, InvestigateDispatched, ConfirmDispatched,
    VictimConfirmed, CandidateRejected, BatteryRTH,
    DroneDown, DroneDamageReport, SectorReassigned, TaskTimeout,
    MissionComplete, MissionTimeout, ScanningStarted, UnknownEvent,
    build_variant,
)
# The from_ros_msg / to_ros_msg back-compat re-exports were removed:
# they pulled `drone_rescue_msgs` (a ROS wire-format type, via
# lib/ros_adapter/event_codec) through the domain package's public
# surface, breaching the rclpy-free invariant. The codec functions'
# canonical home is lib/ros_adapter/event_codec; import them directly
# from there.
#
# NO `..ros_adapter` / rclpy / drone_rescue_msgs imports in this
# module: lib/domain/ must import cleanly in a pure-Python test.

__all__ = [
    'Drone', 'Victim',
    'Position', 'WatchdogClock',
    'IncomingCandidate', 'IncomingHealth', 'IncomingTaskStatus',
    'PartitionKind',
    'TaskType',
    'SectorWedge', 'ScanPlan', 'Bid', 'OutgoingTask', 'MissionStateSnapshot',
    'MissionStage', 'VictimStage',
    'MissionStateMachine', 'VictimStateMachine',
    'TransitionEvent', 'IllegalTransition',
    'VictimSubMission',
    'Mission',
    # Typed MissionEvent ADT
    'MissionEventVariant',
    'CandidateDetected', 'InvestigateDispatched', 'ConfirmDispatched',
    'VictimConfirmed', 'CandidateRejected', 'BatteryRTH',
    'DroneDown', 'DroneDamageReport', 'SectorReassigned', 'TaskTimeout',
    'MissionComplete', 'MissionTimeout', 'ScanningStarted', 'UnknownEvent',
    'build_variant',
]
