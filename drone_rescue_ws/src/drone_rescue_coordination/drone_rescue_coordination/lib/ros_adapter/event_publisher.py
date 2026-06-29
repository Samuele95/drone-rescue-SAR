"""RosEventPublisherAdapter: production EventPort impl.

Lifted out of ``lib/ports/event_port.py``. Ports
declare contracts; adapters live with adapters. Mirrors the
existing ``lib/ports/recovery_dispatcher.py`` (Protocol) vs
``lib/ros_adapter/recovery_dispatcher.py`` (Adapter) split.
"""

from __future__ import annotations

from ..domain.events import MissionEventVariant
from .event_codec import to_ros_msg


class RosEventPublisherAdapter:
    """Production adapter: wraps an rclpy Publisher and emits ROS
    MissionEvent messages on ``/mission/events``.

    The adapter sits at the hexagonal boundary: the Mission aggregate
    knows nothing about the publisher; this class hides the wire
    format conversion via ``to_ros_msg`` (lifted to
    ``lib/ros_adapter/event_codec.py``).

    Construction takes a publisher object (anything with a ``publish``
    method) so this class itself remains rclpy-agnostic for testing.
    The real wiring (creating the rclpy publisher with the right QoS)
    is done by the consuming ROS node.
    """

    def __init__(self, publisher) -> None:
        # ``publisher`` is typed as Any so this class can be unit-tested
        # with a SimpleNamespace(publish=lambda m: emitted.append(m)).
        self._pub = publisher

    def emit(self, event: MissionEventVariant) -> None:
        if self._pub is None:
            return
        self._pub.publish(to_ros_msg(event))
