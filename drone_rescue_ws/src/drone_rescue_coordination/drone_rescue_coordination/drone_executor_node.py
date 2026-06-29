"""Drone Executor: composition root.

The ``drone_executor.DroneExecutor`` node was
deconstructed so the BT actions consume sensor-derived
input and RETURN a ``BehaviouralOutput`` (no shared output slots), the
per-tick state is the pure-input ``BehaviouralContextMutable``, ROS
messages are built only at the publish edge, and ``DroneExecutor``
implements the L1 ``BehaviouralLayer`` port.

That cutover is COMPLETE, so
``USE_LEGACY_DRONE_EXECUTOR`` now defaults to ``'0'`` and the "new"
path runs the (finished) ``DroneExecutor`` directly. The two paths
converge on the same node; the flag survives only as an escape hatch.
"""

from __future__ import annotations

import os


USE_LEGACY_DRONE_EXECUTOR = os.environ.get(
    'USE_LEGACY_DRONE_EXECUTOR', '0',
) != '0'


def _legacy_main(args=None) -> None:
    from drone_rescue_coordination.drone_executor import main as legacy
    legacy(args=args)


def _new_main(args=None) -> None:
    # The cutover target IS the DroneExecutor in
    # drone_executor.py (BT output channel + BehaviouralLayer already
    # live there), so the "new" path runs it directly.
    from drone_rescue_coordination.drone_executor import main as executor_main
    executor_main(args=args)


def main(args=None) -> None:
    if USE_LEGACY_DRONE_EXECUTOR:
        _legacy_main(args=args)
    else:
        _new_main(args=args)
