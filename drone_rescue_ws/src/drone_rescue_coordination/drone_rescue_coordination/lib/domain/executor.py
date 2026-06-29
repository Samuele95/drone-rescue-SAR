"""DroneExecutor aggregate.

``ExecutorPort`` + ``ExecutorSensors`` + ``ExecutorOutputs`` are a
Protocol + frozen VOs. This module adds a concrete ``Executor`` that
implements ``ExecutorPort.tick(sensors)`` by orchestrating the
BT-based per-tick update.

The BT tree itself lives in ``lib.bt`` (already rclpy-free). The
per-tick state is the legacy ``ExecCtx`` in ``drone_executor.py``,
re-exported here as ``ExecutorState`` so the imports tell the
intended layering: drone_executor.py is the ROS adapter, this
module is the domain aggregate.

The Executor takes a ``state``, a BT ``tree`` root, and two callable
hooks (``update_state_from_sensors``, ``read_outputs_from_state``)
that bridge between the typed domain VOs and the LifecycleNode's
state. The hooks are pure-Python and unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol, Tuple

from .task_type import TaskType
from .value_objects import Position


@dataclass(frozen=True)
class ExecutorSensors:
    """Per-tick sensor inputs. Translated from
    ROS Odometry / LaserScan / TaskAssignment / DroneHealth by
    ``lib/ros_adapter/translators_executor.py``."""
    now_sec: float
    current_pose: Optional[Position] = None
    lidar_min_range_m: float = float('inf')
    current_task_type: TaskType = TaskType.IDLE
    current_task_id: int = 0
    target: Optional[Position] = None
    waypoints: Tuple[Position, ...] = ()
    is_down: bool = False
    battery_ok: bool = True


@dataclass(frozen=True)
class ExecutorOutputs:
    """What a single tick produces. The composition root translates
    each non-None field into the corresponding ROS publish."""
    target_pose: Optional[Position] = None
    land_command: bool = False
    status_detail: str = ''
    completed_task_id: Optional[int] = None
    failed_task_id: Optional[int] = None
    damage_reason: Optional[str] = None


class ExecutorPort(Protocol):
    """Driver port: the composition root calls this per tick."""

    def tick(self, sensors: ExecutorSensors) -> ExecutorOutputs: ...


# ExecutorState is the canonical name for the per-tick mutable state
# ExecCtx holds. ``drone_executor.ExecCtx`` is kept as a back-compat
# alias so the act_* functions continue to work with their current
# signatures.
class ExecutorState:
    """Placeholder type; runtime is ``drone_executor.ExecCtx``.

    The typed name is declared in the domain layer so references read
    naturally; the body lives in the LifecycleNode file because it
    carries ROS-typed publish-side fields the domain doesn't model yet.
    """


class Executor:
    """Concrete ExecutorPort implementation.

    Orchestrates the per-tick BT update. Two callable hooks bridge
    between typed ``ExecutorSensors`` / ``ExecutorOutputs`` VOs and
    the LifecycleNode-owned mutable state. The Executor stays
    rclpy-free; the LifecycleNode owns the ROS-typed state.

    Construction:
    - ``state``: the mutable per-tick state object (e.g. ExecCtx).
    - ``tree``: the BT root built by ``lib.bt`` (already pure-Python).
    - ``update_state_from_sensors``: writes ExecutorSensors fields
      into the state in place. Called at the top of each tick.
    - ``read_outputs_from_state``: builds ExecutorOutputs from the
      tick result, the state plus the ``(status, output)`` the BT
      returned (the behavioural output is the tree's return value,
      not a side effect on the state). The LifecycleNode then translates
      the ``Position`` target_pose into a ROS PoseStamped publish.
    """

    def __init__(
        self,
        state: Any,
        tree: Any,
        *,
        update_state_from_sensors: Callable[[Any, ExecutorSensors], None],
        read_outputs_from_state: Callable[[Any, Any, Optional[Any]], ExecutorOutputs],
    ):
        self._state = state
        self._tree = tree
        self._update = update_state_from_sensors
        self._read = read_outputs_from_state

    def tick(self, sensors: ExecutorSensors) -> ExecutorOutputs:
        self._update(self._state, sensors)
        status, output = self._tree.tick(self._state)
        return self._read(self._state, status, output)

    @property
    def state(self) -> Any:
        """Exposed read access: the LifecycleNode's existing
        callbacks reach into the state directly during the migration."""
        return self._state
