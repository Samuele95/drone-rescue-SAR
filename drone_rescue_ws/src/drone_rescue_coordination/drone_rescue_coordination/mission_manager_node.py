"""Mission Manager: composition root.

The legacy ``mission_manager.MissionManager``
(1267 LOC) is being deconstructed. This module is the new composition
root: a thin LifecycleNode that wires the Mission aggregate (from
``lib/domain/``) to the ROS adapter layer (translators in
``lib/ros_adapter/``) and exposes the same `/mission/...` topics the
operator UI expects.

Feature-flagged via ``USE_LEGACY_MISSION_MANAGER`` (env var, default
``"1"``). While the flag is set, ``main()`` defers to the legacy
``MissionManager`` so the cutover is reversible. Today the new node
itself is scaffolding: the saga still lives in the legacy class.

Once the flag flips, this node owns:

- subscription wiring via ``TopicFactory``;
- the `Mission` aggregate constructed with `Clock` + `RngSource` +
  `BidderRegistry` + `EventPort`;
- per-callback delegation through ``mission_manager_translator``
  (the anti-corruption layer that turns ROS msgs into domain VOs
  and the returned ``OutgoingTask`` records into ``TaskAssignment``
  messages);
- the 1 Hz tick that drives `Mission.tick(now_sec)`.
"""

from __future__ import annotations

import os

import rclpy


# Default to the legacy path. Flip to "0" to exercise the new
# composition root (only useful once the saga migration lands).
USE_LEGACY_MISSION_MANAGER = os.environ.get(
    'USE_LEGACY_MISSION_MANAGER', '1',
) != '0'


def _legacy_main(args=None) -> None:
    """Delegate to the legacy LifecycleNode. Preserves all current
    behaviour while the saga migration is staged."""
    from drone_rescue_coordination.mission_manager import main as legacy
    legacy(args=args)


def _new_main(args=None) -> None:
    """Run the composition root.

    The saga is now fully lifted
    into ``lib.domain.Mission`` (the L3 deliberative planner), and the
    pure-Python wiring is complete + tested:

    - ``Mission`` implements the full ``MissionPort`` shape via
      ``lib.mission_port_adapter.MissionPortAdapter`` (45 saga tests +
      10 adapter tests, all rclpy-free).
    - the per-callback anti-corruption translation already exists in
      ``mission_manager_translator`` (handle_candidate / handle_health
      / handle_task_status / handle_tick → MissionPort → TaskAssignment).

    Making this the production runtime is the ROS-gated, irreversible
    step (it must be validated by the ``multi_drone_simulation.launch.py``
    smoke launch, which cannot run in the rclpy-free CI sandbox):

      1. Build the composition: construct ``Mission`` wired with its
         ``AllocationStrategy`` + ``SectorOwnerPolicy`` + ``EventPort``
         + ``_emit_event`` callable + tick/dispatch config; wrap it in a
         ``MissionPortAdapter`` with a ``RosClock`` + a coverage-plan
         provider (PlannerInput → ``CoverageStrategy.plan_v2``) +
         ``ElevationModel.elevation_at``.
      2. Create subscriptions / publishers / tick timer (mirror the
         legacy ``MissionManager.on_configure`` ROS setup) and route
         each callback through ``mission_manager_translator``.
      3. Retire the legacy ``DroneRecord`` / ``VictimRecord`` mirror +
         ``_sync_to_mission`` and flip ``USE_LEGACY_MISSION_MANAGER=0``.

    Until that ROS-validated cutover lands, fall back to the legacy
    ``MissionManager``, which is itself now fully Mission-backed (every
    saga decision routes through the aggregate via the back-compat shims),
    so default-path behaviour is already the lifted architecture; only
    the redundant record mirror remains.
    """
    rclpy.logging.get_logger('mission_manager_node').warn(
        'USE_LEGACY_MISSION_MANAGER=0: the saga is fully lifted into '
        'Mission (+ MissionPortAdapter), but the production node cutover '
        '(subscription wiring + record-mirror retirement) must be '
        'validated by the multi_drone_simulation smoke launch. Falling '
        'back to the legacy (Mission-backed) MissionManager.'
    )
    _legacy_main(args=args)


def main(args=None) -> None:
    if USE_LEGACY_MISSION_MANAGER:
        _legacy_main(args=args)
    else:
        _new_main(args=args)
