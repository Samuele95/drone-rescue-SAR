"""QoS profile factories for operator-facing topics.

Today peer_state's QoS appears reconstructed (RELIABLE, TRANSIENT_LOCAL,
depth=1) in 3 places; mission_events' QoS appears reconstructed
(RELIABLE, VOLATILE, depth=50) in 2 places. This module provides
named factories so a profile change happens in one place.

Importing this module triggers `import rclpy.qos`, fine for a UI
package (both consumer packages are rclpy-aware), but pure-Python
domain modules should not import this.
"""

from __future__ import annotations

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def peer_state_qos(depth: int = 1) -> QoSProfile:
    """QoS for ``/<drone>/peer_state``: RELIABLE, TRANSIENT_LOCAL.

    A late-joining dashboard or mission_recorder catches the latest
    peer-state on subscribe, which is exactly what the per-drone
    state widget wants. Producers must match (drone_executor sets
    its peer_state publisher to the same profile).
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=depth,
    )


def mission_events_qos(depth: int = 50) -> QoSProfile:
    """QoS for ``/mission/events``: RELIABLE, VOLATILE, depth=50.

    Late subscribers don't replay the full history (TRANSIENT_LOCAL
    would be too aggressive) but the publisher's queue is deep enough
    that brief subscriber stalls don't drop events.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        depth=depth,
    )


def sensor_qos(depth: int = 10) -> QoSProfile:
    """QoS for sensor streams: BEST_EFFORT, VOLATILE.

    Use for camera/LiDAR/IMU bridges; appropriate when stale data is
    less harmful than blocking the publisher waiting for ACK.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        depth=depth,
    )


def transient_local_reliable_qos(depth: int = 1) -> QoSProfile:
    """Generic late-joiner-friendly QoS for one-shot trigger topics
    (``/survey/start``) and other latched signals."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=depth,
    )
