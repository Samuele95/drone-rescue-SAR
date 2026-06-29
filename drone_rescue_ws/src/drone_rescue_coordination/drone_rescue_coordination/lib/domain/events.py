"""Typed MissionEvent ADT: closed sum-type over /mission/events.

The wire format (``drone_rescue_msgs.msg.MissionEvent``) carries
``event_type: string`` plus generic fields (drone_name, detail,
victim_id, position, severity, confidence). Internal consumers do
``if msg.event_type == 'VICTIM_CONFIRMED': ...`` at multiple sites,
stringly-typed, with the field-shape per event type implicit.

This module provides a typed sum-type so consumers can pattern-match
and the publisher can't silently drop events with a typo.

Wire format unchanged. The typed ADT is purely an internal
projection; ``to_ros_msg(e)`` round-trips back if you need to
re-publish (e.g., from a JSONL replayer).

``from_ros_msg`` / ``to_ros_msg`` live in
``lib/ros_adapter/event_codec.py`` so this module does not
import ``drone_rescue_msgs``. The variant dataclasses, the
``MissionEventVariant`` Union, and ``build_variant`` (a pure-Python
factory keyed on event_type strings) stay here, all rclpy-free.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import List, Optional, Tuple, Union


# variants

@dataclass(frozen=True)
class _BaseEvent:
    """Common fields every variant carries (sourced from
    ``MissionEvent``'s shared header)."""
    severity: int = 0
    raw_detail: str = ''
    drone_name: str = ''


@dataclass(frozen=True)
class CandidateDetected(_BaseEvent):
    victim_id: int = 0
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    confidence: float = 0.0
    reporters: Tuple[str, ...] = ()


@dataclass(frozen=True)
class InvestigateDispatched(_BaseEvent):
    victim_id: int = 0
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ConfirmDispatched(_BaseEvent):
    victim_id: int = 0
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class VictimConfirmed(_BaseEvent):
    victim_id: int = 0
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class CandidateRejected(_BaseEvent):
    victim_id: int = 0


@dataclass(frozen=True)
class BatteryRTH(_BaseEvent):
    pass


@dataclass(frozen=True)
class DroneDown(_BaseEvent):
    """Drone has gone unrecoverable. ``raw_detail`` carries the
    DroneHealth-derived reason string."""
    pass


@dataclass(frozen=True)
class DroneDamageReport(_BaseEvent):
    """Executor self-reported damage."""
    pass


@dataclass(frozen=True)
class SectorReassigned(_BaseEvent):
    pass


@dataclass(frozen=True)
class TaskTimeout(_BaseEvent):
    pass


@dataclass(frozen=True)
class MissionComplete(_BaseEvent):
    pass


@dataclass(frozen=True)
class MissionTimeout(_BaseEvent):
    pass


@dataclass(frozen=True)
class ScanningStarted(_BaseEvent):
    pass


@dataclass(frozen=True)
class UnknownEvent(_BaseEvent):
    """Forward-compat shield: an event_type the closed enum doesn't
    recognise. Carries the full raw event_type string so future
    operator code can match on it after a wire-format addition."""
    event_type: str = ''


MissionEventVariant = Union[
    CandidateDetected, InvestigateDispatched, ConfirmDispatched,
    VictimConfirmed, CandidateRejected, BatteryRTH,
    DroneDown, DroneDamageReport, SectorReassigned, TaskTimeout,
    MissionComplete, MissionTimeout, ScanningStarted, UnknownEvent,
]


# dispatch tables
# event_type string -> variant constructor. Exposed for the codec
# module in ros_adapter; build_variant below also consumes it.
_DECODER = {
    'CANDIDATE_DETECTED':       CandidateDetected,
    'INVESTIGATE_DISPATCHED':   InvestigateDispatched,
    'CONFIRM_DISPATCHED':       ConfirmDispatched,
    'VICTIM_CONFIRMED':         VictimConfirmed,
    'CANDIDATE_REJECTED':       CandidateRejected,
    'BATTERY_RTH':              BatteryRTH,
    'DRONE_DOWN':               DroneDown,
    'DRONE_DAMAGE_REPORT':      DroneDamageReport,
    'SECTOR_REASSIGNED':        SectorReassigned,
    'TASK_TIMEOUT':             TaskTimeout,
    'MISSION_COMPLETE':         MissionComplete,
    'MISSION_TIMEOUT':          MissionTimeout,
    'SCANNING_STARTED':         ScanningStarted,
}

# Inverse for to_ros_msg (lives in ros_adapter/event_codec.py).
_VARIANT_TO_TYPE_STR = {v: k for k, v in _DECODER.items()}

# The closed set of recognised MissionEvent.event_type strings, derived
# from the single dispatch table above (not re-listed). MissionEvent.event_type
# stays a string ON THE WIRE (the .msg comment documents it as a loose,
# forward-compatible enum, and re-typing it would break unmodified subscribers),
# but emitters and consumers should type their event_type against THIS set
# rather than scattering string literals. ``build_variant`` already maps any
# string outside it to ``UnknownEvent``; ``is_known_event_type`` lets callers
# check membership without reaching into the private decoder.
KNOWN_EVENT_TYPES = frozenset(_DECODER)


def is_known_event_type(event_type: str) -> bool:
    """True iff ``event_type`` is one of the closed sum-type's recognised
    strings (i.e. ``build_variant`` will return a typed variant, not
    ``UnknownEvent``)."""
    return event_type in _DECODER


def build_variant(event_type: str, **fields) -> 'MissionEventVariant':
    """Construct a typed variant from an event_type string + keyword
    fields. Single dispatch table for the legacy stringly-typed emitter
    API. Unknown event_type strings produce UnknownEvent. Kwargs that
    don't match a variant's declared dataclass fields are silently
    dropped (so a caller passing `confidence=` to MissionComplete
    won't raise).

    The (position) tuple may be passed as a tuple or as anything with
    `.x / .y / .z` attributes (e.g. ``geometry_msgs.msg.Point``); the
    helper normalises to the (x, y, z) tuple shape the variants store.
    """
    position = fields.pop('position', None)
    if position is not None and not isinstance(position, tuple):
        position = (
            float(getattr(position, 'x', 0.0)),
            float(getattr(position, 'y', 0.0)),
            float(getattr(position, 'z', 0.0)),
        )
        fields['position'] = position
    elif position is not None:
        fields['position'] = position

    cls = _DECODER.get(event_type)
    if cls is None:
        accepted = {f.name for f in dataclasses.fields(UnknownEvent)}
        return UnknownEvent(
            event_type=event_type,
            **{k: v for k, v in fields.items() if k in accepted},
        )
    accepted = {f.name for f in dataclasses.fields(cls)}
    return cls(**{k: v for k, v in fields.items() if k in accepted})
