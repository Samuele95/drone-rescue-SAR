"""3T Architecture: Executive Layer (L2).

Lifecycle subdomain: pure-Python helpers extracted from
lifecycle_manager.py.

This package implements the Executive Layer (Layer 2) of the 3T
architecture (slides pp. 33, 38, 85-86):

    > "Interface between behavioural and planning layers, translates
    > high-level plans into low-level invocations also taking care of
    > monitoring and handling exceptions."

It translates task assignments (from the deliberative planning layer,
``lib/domain/mission.Mission``) into behavioural invocations (to
``drone_executor`` / ``drone_controller`` / ``surveyor``), and handles
exceptions (via ``RecoveryPolicy``) and lifecycle sequencing (via
``StartupSequencer``).

Component overview:

  * ``orchestrator.py``: ordered node-list builder + transition
    sequencer parametrised by a change-state caller.
  * ``watchdog.py``: ``NodeKind`` IntEnum, ``classify_node``, and
    ``HeartbeatTracker``. Drives the L2 anomaly-detection that
    escalates to the planner via ``ExecutiveSupervisor``.
  * ``startup_sequencer.py``: staged lifecycle activation orchestrator
    consuming ``ChangeStateClient``.
  * ``recovery_policy.py``: exception-handling policy; fulfils part
    of the ``ExecutiveSupervisor`` Protocol contract.

Sibling L2 implementations outside this package:

  * ``readiness_coordinator.py``: pre-mission admission gate (L2).
  * ``lifecycle_manager.py``: top-level L2 supervisor LifecycleNode.
  * ``lib/domain/system_mode_machine.py``: typed SystemMode FSM
    consulted by the supervisor.
  * ``lib/ports/{change_state_client,recovery_dispatcher,
    executive_supervisor}.py``: L2 boundary Protocols.
"""

from .orchestrator import build_node_list
from .watchdog import HeartbeatTracker, NodeKind, classify_node

__all__ = [
    'build_node_list',
    'HeartbeatTracker',
    'NodeKind',
    'classify_node',
]
