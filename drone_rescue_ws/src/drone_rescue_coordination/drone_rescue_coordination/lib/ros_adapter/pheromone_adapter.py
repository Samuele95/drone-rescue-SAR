"""ROS adapter for ``StigmergyPort``: pheromone-server bridge.

Wraps the existing ``PheromoneMap`` ROS topic subscription / the
per-drone deposit publish into a typed ``StigmergyPort`` instance.
The Surveyor LifecycleNode can hold a reference to this adapter
and call ``port.get_grid()`` / ``port.deposit(p)`` instead of
managing the subscription details itself.

Anti-corruption invariant: domain code consumes ``StigmergyPort``,
never ``PheromoneMap`` directly.

The adapter does NOT replace the existing Surveyor pheromone
subscription; `surveyor.py` is left untouched. Adoption is a
separate runtime cutover (consistent with the "Protocol first,
runtime cutover later" pattern used elsewhere). Tests construct
the adapter directly via ``RosPheromoneAdapter.from_node(node)``
once they have an rclpy context.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

import numpy as np

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.publisher import Publisher
    from rclpy.subscription import Subscription

    from drone_rescue_coordination.lib.domain.value_objects import Position


class RosPheromoneAdapter:
    """Concrete ``StigmergyPort`` implementation backed by ROS topics.

    Subscribes to ``/pheromone/map`` for the decayed grid;
    publishes ``PointStamped`` deposit messages on
    ``/<drone>/pheromone_deposit`` (the existing topic the
    pheromone_server collects from).

    Construction is two-step because the rclpy publisher / subscriber
    need an active context. The typical pattern is:

        adapter = RosPheromoneAdapter(drone_name='drone1')
        adapter.bind(node)         # creates the subscriber + publisher

    or the all-in-one form ``adapter = RosPheromoneAdapter.bound(node, 'drone1')``.
    """

    def __init__(self, drone_name: str):
        self._drone_name = drone_name
        self._grid: Optional[np.ndarray] = None
        self._origin: tuple[float, float] = (0.0, 0.0)
        self._resolution: float = 0.5
        self._sub: Optional['Subscription'] = None
        self._pub: Optional['Publisher'] = None

    # StigmergyPort interface
    def get_grid(self) -> Optional[np.ndarray]:
        return self._grid

    def deposit(
        self, position: 'Position', strength: float = 1.0,
    ) -> None:
        # Local imports, only valid in a live ROS context. The adapter
        # is hexagonal-edge code so this is the right place to import
        # the ROS message types.
        if self._pub is None:
            # Test path or pre-bind call: no-op rather than raising;
            # the test fake (InMemoryStigmergyGrid) records deposits
            # explicitly.
            return
        from geometry_msgs.msg import PointStamped
        msg = PointStamped()
        msg.header.frame_id = 'world'
        msg.point.x = float(position.x)
        msg.point.y = float(position.y)
        msg.point.z = float(strength)   # convention: server reads z as weight
        self._pub.publish(msg)

    def grid_origin(self) -> tuple[float, float]:
        return self._origin

    def cell_resolution(self) -> float:
        return self._resolution

    # ROS plumbing
    def bind(self, node: 'Node') -> None:
        """Wire subscriptions + publishers using the host node.

        Idempotent: second call replaces the previous handles.
        """
        from drone_rescue_msgs.msg import PheromoneMap
        from geometry_msgs.msg import PointStamped
        from rclpy.qos import (
            DurabilityPolicy, QoSProfile, ReliabilityPolicy,
        )

        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_deposit = QoSProfile(depth=10)
        self._sub = node.create_subscription(
            PheromoneMap, '/pheromone/map',
            self._on_map, qos_map,
        )
        self._pub = node.create_publisher(
            PointStamped,
            f'/{self._drone_name}/pheromone_deposit',
            qos_deposit,
        )

    @classmethod
    def bound(cls, node: 'Node', drone_name: str) -> 'RosPheromoneAdapter':
        a = cls(drone_name)
        a.bind(node)
        return a

    # subscription callback
    def _on_map(self, msg) -> None:
        """Cache the latest decayed grid for ``get_grid()`` callers."""
        try:
            arr = np.asarray(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width,
            )
        except (ValueError, AttributeError):
            return
        self._grid = arr
        self._origin = (float(msg.origin_x), float(msg.origin_y))
        self._resolution = float(msg.resolution)
