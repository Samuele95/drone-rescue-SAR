"""BehaviouralLayer: Layer-1/Layer-2 boundary port (3T architecture).

The slides (Marcelletti, "Autonomous and Collaborative Robotics",
A.Y. 2025/26, pp. 33, 37-38, 90, 94-96) place L1 close to sensors and
actuators with linear time/complexity and stimulus->response
semantics; L2 translates planning-layer task assignments into L1
behaviour invocations and monitors completion / exceptions.

This Protocol names the L2->L1 dispatch boundary. The concrete
production implementation is ``DroneExecutor``: its BT actions consume
the sensor-only ``BehaviouralContext`` input and return
``BehaviouralOutput`` values (``lib.domain.behaviour_actions``), and
the node implements this Protocol's ``dispatch_task`` /
``cancel_task``. On this deployment L1 is a separate ROS node, so the
realized L2->L1 payload is the ``TaskAssignment`` wire message rather
than the pure-domain ``OutgoingTask``: the topic is the boundary.

3T boundary: ``LAYER_BOUNDARY = 'L1-L2'``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from ..domain.value_objects import OutgoingTask


LAYER_BOUNDARY = 'L1-L2'   # 3T architecture annotation.


class BehaviouralLayer(Protocol):
    """L2 → L1 dispatch interface.

    The executive layer hands a task to this Protocol; the L1
    implementation (BT + PID + Surveyor) executes it. The Protocol is
    intentionally narrow: the LifecycleNode owns the publishers, the
    BT owns the tick loop, this Protocol just names the contract.
    """

    def dispatch_task(self, task: 'OutgoingTask') -> None:
        """Begin executing ``task`` on this drone's behavioural stack.

        Idempotent w.r.t. ``task.task_id``: re-dispatching the same
        task is a no-op (the BT keeps ticking on it). Dispatching a
        new task implicitly cancels any previous one.
        """
        ...

    def cancel_task(self, task_id: int) -> None:
        """Stop executing the named task. Drone returns to IDLE."""
        ...
