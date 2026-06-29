"""Frozen value objects for the SAR domain."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

from .state_machines import MissionStage


# Default multi-view INVESTIGATE orbit: 4 cardinal viewpoints. Single
# source for the planner default (Mission / VictimSubMission) and the
# executor's legacy fallback.
DEFAULT_INVESTIGATE_ANGLES: Tuple[float, ...] = (
    0.0, math.pi / 2, math.pi, 3 * math.pi / 2,
)


@dataclass(frozen=True)
class Position:
    """Pure-Python 3D position. The anti-corruption layer's reason
    for being: `lib/domain/` and `lib/ports/` never import
    `geometry_msgs.msg.Point`. Translation lives in
    `lib/ros_adapter/translators.py`.

    Field access (`.x`, `.y`, `.z`) is duck-compatible with
    `geometry_msgs.msg.Point` so legacy call sites that already
    read `pose.x` keep working when migrated."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass(frozen=True)
class WatchdogClock:
    """Per-drone silence-detector timestamps.

    Bundles ``last_status_t`` and ``task_dispatched_t`` (the two values
    whose max defines drone silence) with the running
    ``last_dispatch_offset`` bookkeeping. ``silence(now)`` is the method
    the watchdog tick calls.

    Frozen: write sites use ``dataclasses.replace`` to advance the
    fields; the per-Hz watchdog allocation cost is negligible.
    """
    last_status_t: float = 0.0
    task_dispatched_t: float = 0.0
    last_dispatch_offset: int = 0

    def silence(self, now_sec: float) -> float:
        """Seconds since the last status or task dispatch (later of the two)."""
        return now_sec - max(self.last_status_t, self.task_dispatched_t)


@dataclass(frozen=True)
class SectorWedge:
    """Angular wedge of the mission disk owned by a single drone.

    Bearings on `[0, 2π)`. ``start_rad > end_rad`` indicates a wedge that
    wraps the seam (e.g. start=5.8, end=0.5 covers `[5.8, 2π) ∪ [0, 0.5)`).
    """
    start_rad: float
    end_rad: float

    def contains(self, bearing: float) -> bool:
        """Inclusive on start, exclusive on end; handles wrap-around."""
        if self.start_rad <= self.end_rad:
            return self.start_rad <= bearing < self.end_rad
        return bearing >= self.start_rad or bearing < self.end_rad

    def midpoint(self) -> float:
        """Angular centre of the wedge on ``[0, 2π)``, handling wrap.

        The dual of ``contains()``, used to compare wedge proximity
        when a survivor must absorb a lost drone's sector.
        """
        two_pi = 2.0 * math.pi
        width = (self.end_rad - self.start_rad) % two_pi
        return (self.start_rad + width / 2.0) % two_pi

    def absorb(self, other: 'SectorWedge') -> 'SectorWedge':
        """Return a new wedge extending ``self`` to also cover the
        adjacent ``other`` wedge.

        The shared boundary is whichever of ``self``'s edges sits
        angularly nearer ``other``'s midpoint; that edge is pushed out
        to ``other``'s matching edge. Immutable: the receiver is not
        mutated; a fresh ``SectorWedge`` is returned (EJ-17).
        """
        other_mid = other.midpoint()

        def gap(a: float, b: float) -> float:
            return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

        if gap(self.end_rad, other_mid) <= gap(self.start_rad, other_mid):
            return SectorWedge(self.start_rad, other.end_rad)
        return SectorWedge(other.start_rad, self.end_rad)


@dataclass(frozen=True)
class ScanPlan:
    """Per-drone coverage plan emitted by a CoverageStrategy.

    ``waypoints`` is a tuple of ``(x, y)`` pairs in world frame; the
    cursor lives on the mutable Drone, not on this value. ``wedge`` is
    None when the strategy doesn't partition angularly (parallel_track,
    expanding_square, sector_search, random_walk; see
    `CoverageStrategy.sector_type`).

    Shape note: waypoints here are 2-tuples, the z
    coordinate is NOT stored. Survey altitude is injected by the ROS
    adapter at dispatch time (``mission_manager._begin_scan`` adds
    ``survey_altitude + elevation_at(x, y)``). Contrast
    ``OutgoingTask.waypoints``, which carries full ``(x, y, z)`` triples.
    """
    waypoints: Tuple[Tuple[float, float], ...]
    wedge: Optional[SectorWedge] = None

    @property
    def length(self) -> int:
        return len(self.waypoints)


@dataclass(frozen=True)
class Bid:
    """Result of an auction round.

    Returned by ``AuctionEngine.bid()``. The legacy auction returns
    ``Optional[str]``; those call sites pass through a back-compat shim
    that maps ``Bid(...).bidder``.
    """
    bidder: str
    utility: float
    target_x: float
    target_y: float


@dataclass(frozen=True)
class OutgoingTask:
    """Typed shadow of a ``TaskAssignment`` to be published.

    What the `Mission` aggregate emits to its ROS adapter.
    The adapter translates this to a ``drone_rescue_msgs.TaskAssignment``
    message on the wire. Until that deconstruction is complete this
    class is a stub used by the new code paths only; legacy
    ``_issue_task`` continues to bypass it.

    ``task_type`` uses the same int values as
    ``TaskAssignment.task_type`` (mirrored by ``TaskType`` IntEnum in
    ``lib/auction.py``; the assert in mission_manager keeps them
    aligned).

    Shape note: ``waypoints`` here are full ``(x, y, z)``
    triples; z is ``survey_altitude + terrain_elevation_at(x, y)``.
    Contrast ``ScanPlan.waypoints``, which stores only ``(x, y)`` and
    leaves z to be injected at dispatch.
    """
    drone_name: str
    task_type: int
    waypoints: Tuple[Tuple[float, float, float], ...]
    target: Optional[Tuple[float, float, float]]
    victim_id: int
    priority: int
    hover_seconds: float
    confirm_orbit_radius: float = 0.0
    # The multi-view INVESTIGATE plan the L3 deliberative layer commits
    # per candidate (orbit radius, per-angle dwell, and the angle set).
    # The executor falls back to its own config when these are 0.0 /
    # empty, so old bags replay unchanged. Mirrors the
    # ``confirm_orbit_radius`` precedent.
    investigate_radius: float = 0.0
    dwell_s: float = 0.0
    investigate_angles: Tuple[float, ...] = ()


class ZoneShape(str, Enum):
    """Closed set of no-fly-zone geometries.

    ``str``-Enum so YAML round-trips unchanged: ``ZoneShape('polygon')``
    yields ``ZoneShape.POLYGON`` and ``ZoneShape.POLYGON == 'polygon'``
    is True, so loaders that pass raw strings and consumers that compare
    against string literals both keep working. Mirrors the
    ``PartitionKind`` precedent.
    """
    POLYGON = 'polygon'
    CIRCLE = 'circle'


class ZonePriority(str, Enum):
    """Closed set of no-fly-zone priorities (``str``-Enum, as ``ZoneShape``)."""
    CRITICAL = 'critical'
    HIGH = 'high'
    MEDIUM = 'medium'
    LOW = 'low'


@dataclass(frozen=True)
class NoFlyZone:
    """Represents a no-fly zone.

    Frozen VO with construction-time invariant validation. Two valid
    shapes (polygon >=3 vertices, or circle with center+radius+positive
    radius) are checked in ``__post_init__``; a malformed zone raises at
    load time instead of silently returning ``float('inf')`` from
    distance checks. ``zone_manager.NoFlyZone`` re-exports it for the
    in-flight migration.

    ``zone_type`` / ``priority`` are closed ``str``-Enums (``ZoneShape``
    / ``ZonePriority``); a raw string is accepted at construction and
    coerced, so existing YAML loaders are unaffected, but an out-of-set
    value now fails fast.
    """
    name: str
    zone_type: ZoneShape          # coerced from str in __post_init__
    priority: ZonePriority = ZonePriority.MEDIUM
    reason: str = ''
    vertices: Tuple[Tuple[float, float], ...] = field(default_factory=tuple)
    center: Optional[Tuple[float, float]] = None
    radius: Optional[float] = None
    min_altitude: float = 0.0
    max_altitude: float = 100.0
    buffer_distance: float = 2.0

    def __post_init__(self):
        # Coerce raw strings (the YAML-loader path) to the enums, and
        # fail fast on out-of-set values. Frozen dataclass → object.__setattr__.
        try:
            object.__setattr__(self, 'zone_type', ZoneShape(self.zone_type))
        except ValueError:
            raise ValueError(
                f"NoFlyZone {self.name!r}: unknown zone_type "
                f"{self.zone_type!r}; expected 'polygon' or 'circle'"
            )
        try:
            object.__setattr__(
                self, 'priority', ZonePriority(self.priority))
        except ValueError:
            raise ValueError(
                f"NoFlyZone {self.name!r}: unknown priority "
                f"{self.priority!r}; expected critical/high/medium/low"
            )
        if self.zone_type == ZoneShape.POLYGON:
            if len(self.vertices) < 3:
                raise ValueError(
                    f"NoFlyZone {self.name!r}: polygon needs >=3 vertices, "
                    f"got {len(self.vertices)}"
                )
        elif self.zone_type == ZoneShape.CIRCLE:
            if self.center is None or self.radius is None:
                raise ValueError(
                    f"NoFlyZone {self.name!r}: circle requires center "
                    f"and radius"
                )
            if self.radius <= 0:
                raise ValueError(
                    f"NoFlyZone {self.name!r}: radius must be > 0, "
                    f"got {self.radius}"
                )


@dataclass(frozen=True)
class VictimHotspot:
    """One pheromone-driven victim hotspot; frozen VO.

    A frozen dataclass to enforce ``dataclasses.replace`` merges (FP
    discipline). Lives in ``lib.domain.value_objects`` alongside the
    other frozen VOs. The navigation policy reads it as
    ``Sequence[VictimHotspot]``.
    """
    x: float
    y: float
    t_seen: float
    confirmed: bool
    confidence: float


@dataclass(frozen=True)
class MissionStateSnapshot:
    """Read-only snapshot of the mission aggregate for the adapter to
    publish on operator topics (e.g. /mission/state).

    The adapter never reads the live aggregate; it requests a snapshot
    on the tick and serialises that. Decouples the publish rate from
    the aggregate's mutation cadence.
    """
    stage: MissionStage
    sectors_total: int
    sectors_completed: int
    victims_found: int
    victims_confirmed: int
    active_tasks_summary: Tuple[str, ...]
