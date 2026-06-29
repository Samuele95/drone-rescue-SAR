"""TopicFactory: centralise per-drone topic naming + QoS profiles.

At least 9 first-party nodes today contain
``for d in drone_names: create_subscription(... f'/{d}/<topic>', ..., qos)``
with the topic name as a string-literal f-string and the QoS profile
hand-rolled per call site. The QoS profiles for the same topic
(notably ``peer_state``: RELIABLE, TRANSIENT_LOCAL, depth=1) appear
identically reconstructed in 3+ places.

This factory provides:

- ``TOPIC_REGISTRY``: a single source of truth for every per-drone
  topic name template and the QoS profile category it should use.
- ``TopicFactory(node, drone_names)``: convenience methods that wrap
  ``node.create_publisher`` / ``node.create_subscription`` with the
  right name and QoS, hiding the lambda-late-binding workaround that
  9 nodes hand-roll today.

Adding a new per-drone topic now: one line in ``TOPIC_REGISTRY``;
existing factory methods generalise via ``per_drone_sub(topic_name,
msg_type, callback)``.

Scaffolding: the existing 9 nodes continue to work unchanged. New
code paths (and progressive migrations of legacy nodes when they're
touched for other reasons) consume this factory.
"""

from __future__ import annotations

from enum import Enum
from typing import Any, Callable, Dict, List

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class QosName(str, Enum):
    """Named QoS profiles. The factory looks up the actual profile
    from this name, so the four call sites that today hand-roll the
    same RELIABLE+TRANSIENT_LOCAL+depth=1 profile collapse to one
    string."""
    PEER_STATE = 'peer_state'           # RELIABLE, TRANSIENT_LOCAL, depth=1
    TASK = 'task'                        # RELIABLE, TRANSIENT_LOCAL, depth=10 (late executor still gets task)
    CMD = 'cmd'                          # RELIABLE, VOLATILE, depth=10
    SENSOR = 'sensor'                    # BEST_EFFORT, VOLATILE, depth=10
    # Hot sensor stream where stale frames are useless (camera images,
    # odom at >5 Hz). depth=1 drops backlog.
    SENSOR_HOT = 'sensor_hot'            # BEST_EFFORT, VOLATILE, depth=1
    SENSOR_RELIABLE = 'sensor_reliable'  # RELIABLE, VOLATILE, depth=20
                                          # Use when downstream wants
                                          # ordered, lossless delivery
                                          # (trajectory analysis, etc.)
    EVENTS = 'events'                    # RELIABLE, VOLATILE, depth=50
    HEALTH = 'health'                    # RELIABLE, VOLATILE, depth=10
    LATCHED_TRIGGER = 'latched_trigger'  # RELIABLE, TRANSIENT_LOCAL, depth=1


# Single source of truth for QoS profiles.
def _build_qos_table() -> Dict[QosName, QoSProfile]:
    return {
        QosName.PEER_STATE: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        ),
        QosName.TASK: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        ),
        QosName.CMD: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        ),
        QosName.SENSOR: QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        ),
        QosName.SENSOR_HOT: QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        ),
        QosName.SENSOR_RELIABLE: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=20,
        ),
        QosName.EVENTS: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=50,
        ),
        QosName.HEALTH: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        ),
        QosName.LATCHED_TRIGGER: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        ),
    }


# Per-drone topic registry. Format: short_name -> (template, default QoS).
# Templates use ``{drone}`` as the placeholder. New per-drone topics add
# a row here; consumers iterate the registry instead of writing their
# own lambda loops.
TOPIC_REGISTRY: Dict[str, Dict[str, Any]] = {
    'task':              {'template': '/{drone}/task',              'qos': QosName.TASK},
    'task_status':       {'template': '/{drone}/task_status',       'qos': QosName.CMD},
    'peer_state':        {'template': '/{drone}/peer_state',        'qos': QosName.PEER_STATE},
    'health':            {'template': '/{drone}/health',            'qos': QosName.HEALTH},
    'odom':              {'template': '/{drone}/odom',              'qos': QosName.SENSOR},
    # zone_manager uses default QoS (depth=10, RELIABLE+VOLATILE).
    # Registry's `CMD` profile matches exactly.
    'zone_warning':      {'template': '/{drone}/zone_warning',      'qos': QosName.CMD},
    'camera':            {'template': '/{drone}/camera',            'qos': QosName.SENSOR},
    'follow_cam':        {'template': '/{drone}/follow_cam',        'qos': QosName.SENSOR},
    'scan':              {'template': '/{drone}/scan',              'qos': QosName.SENSOR},
    'survey_target':     {'template': '/{drone}/survey_target',     'qos': QosName.CMD},
    'land':              {'template': '/{drone}/land',              'qos': QosName.CMD},
    'enable':            {'template': '/{drone}/enable',            'qos': QosName.CMD},
    'cmd_vel':           {'template': '/{drone}/cmd_vel',           'qos': QosName.CMD},
    'battery_low':       {'template': '/{drone}/battery_low',       'qos': QosName.CMD},
    'battery_level':     {'template': '/{drone}/battery_level',     'qos': QosName.SENSOR},
    'detections_raw':    {'template': '/{drone}/detections_raw',    'qos': QosName.EVENTS},
    'status':            {'template': '/{drone}/status',            'qos': QosName.CMD},
    'pheromone_deposit': {'template': '/pheromone/deposit',         'qos': QosName.CMD},
}


def topic_name(short_name: str, drone: str) -> str:
    """Resolve a registry entry's template against a drone name.

    Example: ``topic_name('peer_state', 'drone1') -> '/drone1/peer_state'``.
    """
    if short_name not in TOPIC_REGISTRY:
        raise KeyError(
            f'unknown topic short-name {short_name!r}; '
            f'add to lib/ros_adapter/topic_factory.TOPIC_REGISTRY'
        )
    return TOPIC_REGISTRY[short_name]['template'].format(drone=drone)


class TopicFactory:
    """Wraps ``rclpy.Node`` per-drone publisher/subscription creation.

    The node owns the actual lifecycle (configure/activate/deactivate);
    this factory is just a thin convenience that consumes the registry.

    Usage::

        factory = TopicFactory(node, drone_names=['drone1', 'drone2'])
        # Subscribe to peer_state on every drone, with the correct
        # QoS, with the per-drone closure baked in:
        for sub in factory.per_drone_subs(
                'peer_state', DronePeerState, on_peer):
            self._subs.append(sub)
        # Or per-drone publisher (for task dispatch):
        self._task_pubs = factory.per_drone_pubs('task', TaskAssignment)
    """

    def __init__(self, node: Any, drone_names: List[str]):
        self._node = node
        self._drone_names = list(drone_names)
        self._qos_table = _build_qos_table()

    # ----------------------------------------------------------- per-drone
    def per_drone_pubs(
        self, short_name: str, msg_type: Any,
    ) -> Dict[str, Any]:
        """Publishers, one per drone, indexed by drone_name.

        QoS picked from the registry entry. Returns
        ``{drone_name: Publisher}`` so callers can do
        ``self._task_pubs[d.name].publish(msg)`` without re-resolving
        the topic name at every dispatch.
        """
        spec = TOPIC_REGISTRY[short_name]
        qos = self._qos_table[spec['qos']]
        out: Dict[str, Any] = {}
        for drone in self._drone_names:
            out[drone] = self._node.create_publisher(
                msg_type,
                spec['template'].format(drone=drone),
                qos,
            )
        return out

    def per_drone_subs(
        self,
        short_name: str,
        msg_type: Any,
        callback: Callable[[Any, str], None],
        qos_override: 'QosName | None' = None,
    ) -> List[Any]:
        """Subscriptions, one per drone, with the per-drone late-binding
        closure baked in.

        ``callback`` receives ``(msg, drone_name)``; the lambda
        late-binding gymnastics 9 nodes today repeat
        (``lambda msg, name=d: ...``) are handled here once.

        ``qos_override`` lets a consumer pick a non-default QoS for a
        topic without bypassing the factory (extend QosName, don't
        bypass).
        """
        spec = TOPIC_REGISTRY[short_name]
        qos = self._qos_table[qos_override if qos_override else spec['qos']]
        out: List[Any] = []
        for drone in self._drone_names:

            def _make_cb(d: str):
                def _cb(msg: Any) -> None:
                    callback(msg, d)
                return _cb

            sub = self._node.create_subscription(
                msg_type,
                spec['template'].format(drone=drone),
                _make_cb(drone),
                qos,
            )
            out.append(sub)
        return out

    # ----------------------------------------------------------- single-instance
    def make_pub(
        self, topic: str, msg_type: Any, qos_name: QosName,
    ) -> Any:
        """Convenience for non-per-drone topics like
        ``/mission/events`` or ``/coverage/metrics``."""
        return self._node.create_publisher(
            msg_type, topic, self._qos_table[qos_name],
        )

    def make_sub(
        self, topic: str, msg_type: Any,
        callback: Callable[[Any], None], qos_name: QosName,
    ) -> Any:
        return self._node.create_subscription(
            msg_type, topic, callback, self._qos_table[qos_name],
        )

    # ----------------------------------------------------------- introspection
    def all_per_drone_topics(self) -> List[str]:
        """For diagnostics: every topic name this fleet uses."""
        out: List[str] = []
        for short, spec in TOPIC_REGISTRY.items():
            for drone in self._drone_names:
                out.append(spec['template'].format(drone=drone))
        return out
