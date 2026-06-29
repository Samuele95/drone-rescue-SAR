"""ROS MissionEvent <-> MissionEventVariant codec.

Lifted out of ``lib/domain/events.py`` so the
domain module stays rclpy-free (the project's hexagonal invariant
documented in ``lib/domain/value_objects.py:14-21``). The variant
dataclasses + ``build_variant`` factory stay in the domain; the
ROS-message <-> variant converters live here in ``ros_adapter``
alongside ``translators.py``, ``translators_executor.py``,
``recovery_dispatcher.py``, ``ros_clock.py``.
"""

from __future__ import annotations

from typing import Tuple

from drone_rescue_msgs.msg import MissionEvent as _RosMissionEvent

from ..domain.events import (
    CandidateDetected,
    CandidateRejected,
    ConfirmDispatched,
    InvestigateDispatched,
    MissionEventVariant,
    UnknownEvent,
    VictimConfirmed,
    _DECODER,
    _VARIANT_TO_TYPE_STR,
)


def from_ros_msg(msg: _RosMissionEvent) -> MissionEventVariant:
    """Decode a ROS MissionEvent into the typed ADT.

    Variants that don't carry the corresponding fields silently drop
    them (e.g., MissionComplete ignores `confidence`). UnknownEvent
    is the forward-compat shield: anything not in the closed enum.
    """
    base_kwargs = dict(
        severity=int(msg.severity),
        raw_detail=str(msg.detail),
        drone_name=str(msg.drone_name),
    )
    et = str(msg.event_type)
    cls = _DECODER.get(et)
    if cls is None:
        return UnknownEvent(event_type=et, **base_kwargs)
    if cls is CandidateDetected:
        # Parse `reporters=[...]` out of the legacy detail string for
        # back-compat; new code can use the typed reporters tuple.
        reporters = _parse_reporters(msg.detail)
        return CandidateDetected(
            **base_kwargs,
            victim_id=int(msg.victim_id),
            position=(float(msg.position.x), float(msg.position.y),
                      float(msg.position.z)),
            confidence=float(getattr(msg, 'confidence', 0.0)),
            reporters=reporters,
        )
    if cls in (InvestigateDispatched, ConfirmDispatched, VictimConfirmed):
        return cls(
            **base_kwargs,
            victim_id=int(msg.victim_id),
            position=(float(msg.position.x), float(msg.position.y),
                      float(msg.position.z)),
        )
    if cls is CandidateRejected:
        return CandidateRejected(
            **base_kwargs,
            victim_id=int(msg.victim_id),
        )
    return cls(**base_kwargs)


def to_ros_msg(variant: MissionEventVariant) -> _RosMissionEvent:
    """Re-encode a typed variant back to the wire format. Used by
    the JsonlEventReplay adapter and tests."""
    msg = _RosMissionEvent()
    if isinstance(variant, UnknownEvent):
        msg.event_type = variant.event_type
    else:
        msg.event_type = _VARIANT_TO_TYPE_STR.get(type(variant), 'UNKNOWN')
    msg.severity = int(variant.severity)
    msg.detail = str(variant.raw_detail)
    msg.drone_name = str(variant.drone_name)
    if isinstance(variant, (CandidateDetected, InvestigateDispatched,
                             ConfirmDispatched, VictimConfirmed)):
        msg.victim_id = int(variant.victim_id)
        msg.position.x = float(variant.position[0])
        msg.position.y = float(variant.position[1])
        msg.position.z = float(variant.position[2])
    if isinstance(variant, CandidateDetected):
        msg.confidence = float(variant.confidence)
    if isinstance(variant, CandidateRejected):
        msg.victim_id = int(variant.victim_id)
    return msg


def _parse_reporters(detail: str) -> Tuple[str, ...]:
    """Best-effort extraction of ``reporters=['drone1', 'drone2']``
    from a legacy detail string. Returns an empty tuple when not
    present; new code paths populate the structured field at the
    publisher side."""
    if 'reporters=' not in detail:
        return ()
    try:
        bracket = detail[detail.index('reporters=') + len('reporters='):]
        if not bracket.startswith('['):
            return ()
        end = bracket.index(']')
        inner = bracket[1:end]
        parts = [p.strip().strip("'\"") for p in inner.split(',')]
        return tuple(p for p in parts if p)
    except Exception:
        return ()
