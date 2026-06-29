"""DeliberativePlanner: Layer-3 boundary port (3T architecture).

The slides (Marcelletti, "Autonomous and Collaborative Robotics",
A.Y. 2025/26, pp. 33, 42-44, 85-86) decompose hybrid robot control
into three layers:

    Layer 3, Task-Planning (Deliberative): long-range activity
              selection, HTN/planner+scheduler, replans on change.
    Layer 2, Executive: translates plans to behaviour invocations,
              handles exceptions, sequences lifecycle.
    Layer 1, Behavioural: close to sensors/actuators, linear
              time/complexity, condition->action.

This Protocol names the L3->L2 boundary so the deliberative planner
has a type-level existence separate from the LifecycleNode that
adapts it to ROS. The concrete implementation will be
``lib.domain.Mission`` once the saga lift completes.

Anti-corruption invariant: no ``drone_rescue_msgs.msg.*`` and no
``geometry_msgs.msg.*`` imports. The L2 adapter translates inbound ROS
messages into domain VOs before calling the planner.

3T boundary: ``LAYER_BOUNDARY = 'L2-L3'``; the executive layer drives
a deliberative planner instance.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, Sequence

if TYPE_CHECKING:
    # Forward-referenced so this Protocol module imports cleanly even
    # before the WorldModel VO lands.
    from ..domain.world_model import WorldModel
    from ..domain.value_objects import OutgoingTask


LAYER_BOUNDARY = 'L2-L3'   # 3T architecture annotation.


class DeliberativePlanner(Protocol):
    """Pure-Python entry point into the SAR deliberative planning layer.

    The executive layer (L2, the ``mission_manager_node.py`` adapter)
    consults this planner once per planning tick by passing a
    ``WorldModel`` snapshot; the planner returns a sequence of
    ``OutgoingTask`` records the adapter publishes as ROS
    ``TaskAssignment`` messages.

    Stateless w.r.t. individual calls in principle (mission state is
    carried in the ``WorldModel`` snapshot), but the concrete
    ``Mission`` implementation owns the per-victim saga state, so an
    opaque ``MissionPlanner`` impl may mutate internal aggregates
    between calls.
    """

    def plan(self, world: 'WorldModel') -> Sequence['OutgoingTask']:
        """Given the current world-model snapshot, return the next set
        of task assignments for the executive to dispatch.

        Called per planning tick (typically 2 Hz).  An empty return
        means "no new dispatches this tick".
        """
        ...

    def replan(
        self,
        world: 'WorldModel',
        failed_task: 'OutgoingTask',
    ) -> Sequence['OutgoingTask']:
        """Called by the executive when a dispatched task fails.

        Returns the compensating assignment(s): typically an RTH for
        the failed drone plus a re-dispatch of the victim sub-mission
        to a survivor. Mirrors the slides' "replanning when situation
        changes" Type-2 integration (slide p. 44).
        """
        ...

    def on_task_completed(
        self,
        world: 'WorldModel',
        completed_task: 'OutgoingTask',
    ) -> Sequence['OutgoingTask']:
        """Called by the executive when a dispatched task SUCCEEDS.

        The planner advances the relevant saga step and returns the
        follow-on assignment(s): CONFIRM after INVESTIGATE (cross-drone
        witness handoff), SCAN resume after CONFIRM, or IDLE when a
        sector is exhausted. Mirrors the slides' "monitors the
        progress" executive->planning feedback (slide p. 44).
        """
        ...
