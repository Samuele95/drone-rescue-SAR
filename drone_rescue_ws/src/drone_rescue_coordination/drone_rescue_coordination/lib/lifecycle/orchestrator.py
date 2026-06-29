"""Lifecycle orchestrator helpers: pure Python, no rclpy.

Extracts the pure-Python pieces of
`lifecycle_manager.LifecycleManager`'s startup/shutdown sequencing.
The legacy 1000-LOC LifecycleNode fused three subdomains; this is
the orchestration slice.

Today this module only exposes the `build_node_list` pure function
(the canonical ordered list of nodes the lifecycle manager drives).
The actual startup/shutdown sequencer remains in the LifecycleNode
because each transition step is a rclpy service-client call that
isn't trivial to abstract behind a callable in a way that wins more
than it costs. A `ChangeStateClient` Protocol + `StartupSequencer`
class is the natural next step once a second orchestrator (e.g. for
the Mission Control bench harness) needs the same logic.
"""

from __future__ import annotations

from typing import List, Sequence


def build_node_list(drone_names: Sequence[str]) -> List[str]:
    """Return the ordered list of nodes the LifecycleManager drives.

    Order matters: `pheromone_server` first (other nodes subscribe
    to it), then per-drone controllers (must be active before any
    executor publishes targets), then `mission_manager` (sets up
    pubs/subs but emits nothing until /survey/start), then per-drone
    executors (subscribe to /<drone>/task from mission_manager).
    """
    nodes: List[str] = ['pheromone_server']
    for drone in drone_names:
        nodes.append(f'{drone}_controller')
    nodes.append('mission_manager')
    for drone in drone_names:
        nodes.append(f'{drone}_executor')
    return nodes
