"""Production `Clock` adapter: wraps a `rclpy.node.Node`.

The Mission aggregate constructs a RosClock at the composition root
and passes it in. The aggregate itself imports no rclpy.
"""

from __future__ import annotations


class RosClock:
    """Reads `node.get_clock().now().nanoseconds / 1e9`."""

    def __init__(self, node):
        self._node = node

    def now_sec(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9
