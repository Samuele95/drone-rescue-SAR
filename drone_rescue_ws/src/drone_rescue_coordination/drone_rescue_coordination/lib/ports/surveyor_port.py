"""SurveyorPort: driver port for the per-tick surveyor reducer.

Mirrors the ExecutorPort shape. Pure-Python, no rclpy imports: the
legacy ``surveyor.Surveyor`` LifecycleNode becomes the thin ROS
adapter that builds ``SurveyorSensors`` from incoming messages, calls
``tick()``, and turns ``SurveyorOutputs`` into outgoing publishes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Optional, Protocol, Tuple

import numpy as np

from drone_rescue_coordination.lib.domain.navigation import VictimHotspot
from drone_rescue_coordination.lib.domain.value_objects import Position


# 3T boundary annotation: per-tick reducer port consumed by the L1
# Surveyor LifecycleNode. The motor-schema vector blend in
# `lib/domain/navigation.py` runs behind this Protocol; the
# LifecycleNode is the L1 adapter.
LAYER_BOUNDARY = 'L1-driven'


@dataclass(frozen=True)
class SurveyorSensors:
    """Per-tick sensor bundle handed to ``SurveyorPort.tick``.

    Frozen VO: the LifecycleNode constructs one per tick from the
    ROS callbacks it maintains.
    """
    now_sec: float
    current_position: Optional[Position]
    pheromone_grid: Optional[np.ndarray]
    grid_origin_x: float
    grid_origin_y: float
    cell_resolution: float
    battery_level: float
    zone_warn: bool
    peer_positions: Mapping[str, Position] = field(default_factory=dict)
    hotspots: Tuple[VictimHotspot, ...] = ()
    stuck_seconds: float = 0.0


@dataclass(frozen=True)
class SurveyorOutputs:
    """Per-tick output bundle returned by ``SurveyorPort.tick``.

    Frozen VO: the LifecycleNode adapter translates each field
    into the corresponding ROS publish (PoseStamped, PointStamped,
    Bool, etc.). Optional fields with ``None`` mean "no publish
    this tick".
    """
    target_pose: Optional[Position] = None
    deposit_at: Optional[Position] = None
    return_to_base: bool = False
    stuck_recovery_target: Optional[Position] = None


class SurveyorPort(Protocol):
    """Per-tick navigation reducer.

    Implementations:
    - ``lib/domain/surveyor.Surveyor``: concrete production policy.
    - ``InMemorySurveyorFake`` (tests): records inputs / replays outputs.

    The Protocol stays free of ROS-message types so substitution in
    tests does not require ``rclpy.init()``.
    """

    def tick(self, sensors: SurveyorSensors) -> SurveyorOutputs:
        ...
