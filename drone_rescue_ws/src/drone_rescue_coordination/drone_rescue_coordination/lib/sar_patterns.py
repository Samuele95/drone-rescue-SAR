"""Search-and-rescue coverage patterns.

Design (GoF):

  * Strategy: `CoveragePattern` is the abstract Strategy. Each concrete
    subclass implements `generate_waypoints(region, footprint_m)` for a
    different search pattern. The mission manager treats them
    interchangeably.

  * Template Method: `CoverageStrategy` (the higher-level orchestrator)
    defines the skeleton "partition the search area, then run pattern in each
    region" and lets subclasses override the partitioning step. This is what
    converts ONE pattern into N drone-specific waypoint lists.

  * Registry / Factory: `CoveragePatternFactory.create(name)` returns a
    fully configured `CoverageStrategy` for the named pattern. Adding a new
    pattern is one entry in the registry plus a subclass, no caller changes.

Patterns implemented:

  * spiral_out / spiral_in : Concentric arcs over an annular sector.
        IAMSAR Vol II section 4.4 / Choset 1998 cellular decomposition adapted
        to a polar region. spiral_out builds coverage from the launch pad
        outward (Expanding-Square principle): reasonable when the prior on
        victim location is uniform, and resilient to early drone failures.

  * expanding_square : IAMSAR Vol II section 4.7 Expanding Square Search.
        Square spiral outward from the search datum. Standard pattern when
        the Position Last Seen (PLS) has high confidence (lost-person
        searches per Lost Person Behavior, R. Koester).

  * parallel_track : IAMSAR Vol II section 4.4 Parallel Track / classic lawnmower.
        Parallel sweeps across a rectangular bounding box. Statistically
        optimal when the prior on victim location is uniform across the box
        and partitioning by strip is cheap.

  * sector_search : IAMSAR Vol II section 4.6 Sector Search.
        Three or six radial legs from a datum. Used when a single point has
        very high confidence (point-source emergency).

References:
  * IAMSAR Manual Vol II "Mission Co-ordination", IMO/ICAO 2022.
  * Koester, "Lost Person Behavior", dbS Productions 2008.
  * Choset, "Coverage Path Planning: The Boustrophedon Cellular
    Decomposition", FSR 1998.
  * Gamma et al., "Design Patterns: Elements of Reusable OO Software",
    Addison-Wesley 1994. Strategy, Factory, Template Method.
"""

from __future__ import annotations

import math
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Type

from .domain.partition_kind import PartitionKind


Waypoint = Tuple[float, float]


# region types


@dataclass
class AnnularSector:
    """An angular slice of an annulus centred at (cx, cy)."""
    cx: float
    cy: float
    inner_radius: float
    outer_radius: float
    start_rad: float
    end_rad: float


@dataclass
class RectStrip:
    """An axis-aligned rectangular strip used by ParallelTrackPattern."""
    cx: float
    cy: float
    width: float
    height: float
    yaw: float = 0.0


@dataclass
class Disk:
    """A full disk centred at (cx, cy). Used by patterns that don't subdivide
    angularly (Expanding Square, Sector Search); the disk's outer_radius
    bounds the pattern's outward growth."""
    cx: float
    cy: float
    outer_radius: float


# base


class CoveragePattern(ABC):
    """Strategy interface: produces an ordered waypoint list for a region."""

    @abstractmethod
    def generate_waypoints(self, region, footprint_width_m: float) -> List[Waypoint]:
        ...


# arcs


@dataclass
class ConcentricArcsPattern(CoveragePattern):
    """Concentric-arc lawnmower over an annular sector.

    Tracks are arcs at constant radius; adjacent tracks are stepped radially
    by `footprint * (1 - overlap)` and zig-zag in opposite angular directions.
    `outward=True` produces a spiral-out (start at inner_radius, expand);
    `outward=False` is the inverse (perimeter-first).
    """

    overlap: float = 0.3
    yaw_offset: float = 0.0
    outward: bool = True
    # Cap the angular gap between consecutive points on an arc. The
    # footprint-driven density alone collapses to 2 points (the two wedge
    # boundaries) whenever the arc is shorter than one footprint, which is
    # every inner-radius arc for a wide sector. Adjacent wedges share a
    # boundary radial, so with 2-point arcs every drone flies only the shared
    # sector edges (e.g. the cardinal axes for a 4-way split): neighbours
    # converge on the shared edge and the disk interior is never swept. A
    # ~30deg cap forces interior points so each drone sweeps its own wedge.
    max_angular_step_rad: float = math.pi / 6.0   # 30 degrees

    def generate_waypoints(self, region: AnnularSector, footprint_width_m: float) -> List[Waypoint]:
        if footprint_width_m <= 0.0:
            raise ValueError("footprint_width_m must be > 0")
        if region.outer_radius <= region.inner_radius:
            raise ValueError("region outer_radius must exceed inner_radius")

        track_spacing = (1.0 - self.overlap) * footprint_width_m
        radial_extent = region.outer_radius - region.inner_radius
        n_tracks = max(2, int(math.ceil(radial_extent / track_spacing)) + 1)
        track_spacing = radial_extent / (n_tracks - 1)

        sector_extent = (region.end_rad - region.start_rad) % (2.0 * math.pi)
        if sector_extent < 1e-3:
            sector_extent = 2.0 * math.pi

        waypoints: List[Waypoint] = []
        for i in range(n_tracks):
            r = (region.inner_radius + i * track_spacing) if self.outward \
                else (region.outer_radius - i * track_spacing)
            # Coverage density from the footprint, floored by an angular cap so
            # the arc is actually swept (interior points) rather than collapsing
            # to its two boundary radials; see max_angular_step_rad.
            n_footprint = int(math.ceil(r * sector_extent / footprint_width_m)) + 1
            n_angular = int(math.ceil(sector_extent / self.max_angular_step_rad)) + 1
            n_along = max(2, n_footprint, n_angular)
            thetas = [
                region.start_rad + j * sector_extent / (n_along - 1) + self.yaw_offset
                for j in range(n_along)
            ]
            if i % 2 == 1:
                thetas = list(reversed(thetas))
            for theta in thetas:
                waypoints.append((region.cx + r * math.cos(theta),
                                  region.cy + r * math.sin(theta)))
        return waypoints


# expanding square


@dataclass
class ExpandingSquarePattern(CoveragePattern):
    """IAMSAR Vol II section 4.7 Expanding Square Search."""

    overlap: float = 0.3
    yaw_offset: float = 0.0

    def generate_waypoints(self, region: Disk, footprint_width_m: float) -> List[Waypoint]:
        if footprint_width_m <= 0:
            raise ValueError("footprint_width_m must be > 0")
        leg_step = (1.0 - self.overlap) * footprint_width_m
        base = self.yaw_offset
        heads = [
            (math.cos(base + 0.0), math.sin(base + 0.0)),
            (math.cos(base + math.pi / 2), math.sin(base + math.pi / 2)),
            (math.cos(base + math.pi), math.sin(base + math.pi)),
            (math.cos(base + 3 * math.pi / 2), math.sin(base + 3 * math.pi / 2)),
        ]
        waypoints: List[Waypoint] = [(region.cx, region.cy)]
        x, y = region.cx, region.cy
        leg_len = leg_step
        i = 0
        while True:
            hx, hy = heads[i % 4]
            x += hx * leg_len
            y += hy * leg_len
            if math.hypot(x - region.cx, y - region.cy) > region.outer_radius:
                break
            waypoints.append((x, y))
            i += 1
            if i % 2 == 0:
                leg_len += leg_step
        return waypoints


# parallel track


@dataclass
class ParallelTrackPattern(CoveragePattern):
    """IAMSAR Vol II section 4.4 Parallel-Track lawnmower over a rectangle."""

    overlap: float = 0.3

    def generate_waypoints(self, region: RectStrip, footprint_width_m: float) -> List[Waypoint]:
        if footprint_width_m <= 0:
            raise ValueError("footprint_width_m must be > 0")
        track_spacing = (1.0 - self.overlap) * footprint_width_m
        n_tracks = max(2, int(math.ceil(region.height / track_spacing)) + 1)
        track_spacing = region.height / (n_tracks - 1)

        cy_local = math.cos(region.yaw)
        sy_local = math.sin(region.yaw)
        waypoints: List[Waypoint] = []
        for i in range(n_tracks):
            y_local = -region.height / 2 + i * track_spacing
            xs_local = [-region.width / 2, region.width / 2]
            if i % 2 == 1:
                xs_local = list(reversed(xs_local))
            for x_local in xs_local:
                wx = region.cx + cy_local * x_local - sy_local * y_local
                wy = region.cy + sy_local * x_local + cy_local * y_local
                waypoints.append((wx, wy))
        return waypoints


# creeping line


@dataclass
class CreepingLinePattern(CoveragePattern):
    """IAMSAR Vol II section 4.5 Creeping-Line search.

    The 90deg complement of ParallelTrackPattern: sweep legs run along the
    strip's *short* axis and advance along its *long* axis. Mirrors
    ParallelTrackPattern exactly with the local x / y roles swapped, so the
    complete-coverage guarantee carries over.
    """

    overlap: float = 0.3

    def generate_waypoints(self, region: RectStrip, footprint_width_m: float) -> List[Waypoint]:
        if footprint_width_m <= 0:
            raise ValueError("footprint_width_m must be > 0")
        track_spacing = (1.0 - self.overlap) * footprint_width_m
        n_tracks = max(2, int(math.ceil(region.width / track_spacing)) + 1)
        track_spacing = region.width / (n_tracks - 1)

        cy_local = math.cos(region.yaw)
        sy_local = math.sin(region.yaw)
        waypoints: List[Waypoint] = []
        for i in range(n_tracks):
            x_local = -region.width / 2 + i * track_spacing
            ys_local = [-region.height / 2, region.height / 2]
            if i % 2 == 1:
                ys_local = list(reversed(ys_local))
            for y_local in ys_local:
                wx = region.cx + cy_local * x_local - sy_local * y_local
                wy = region.cy + sy_local * x_local + cy_local * y_local
                waypoints.append((wx, wy))
        return waypoints


# random walk
#
# Pearson, "The Problem of the Random Walk", Nature 72 (1865), 1905.
# Used here as an honest baseline: any structured search pattern (spiral,
# parallel-track, expanding-square) is expected to outperform uniform
# random sampling on coverage and detection latency, providing a
# no-coordination control to measure against.


@dataclass
class RandomWalkPattern(CoveragePattern):
    """Uniform random sampling inside a disk.

    Generates `n_waypoints` waypoints by drawing each independently from a
    uniform-on-disk distribution: r = R*sqrt(U1), theta = 2*pi*U2. RNG is
    seeded explicitly via `rng` for reproducibility: same seed gives an
    identical waypoint list, run after run.
    """

    n_waypoints: int = 50
    rng: Optional[random.Random] = None

    def generate_waypoints(self, region: Disk, footprint_width_m: float) -> List[Waypoint]:
        if footprint_width_m <= 0:
            raise ValueError("footprint_width_m must be > 0")
        if self.n_waypoints < 1:
            raise ValueError("n_waypoints must be >= 1")
        rng = self.rng if self.rng is not None else random.Random()
        out: List[Waypoint] = []
        for _ in range(self.n_waypoints):
            # Uniform-on-disk requires the sqrt; without it samples would
            # cluster near the centre.
            r = region.outer_radius * math.sqrt(rng.random())
            theta = 2.0 * math.pi * rng.random()
            out.append((region.cx + r * math.cos(theta),
                        region.cy + r * math.sin(theta)))
        return out


# sector


@dataclass
class SectorSearchPattern(CoveragePattern):
    """IAMSAR Vol II section 4.6 Sector Search: radial legs from a datum."""

    n_legs: int = 6
    yaw_offset: float = 0.0

    def generate_waypoints(self, region: Disk, footprint_width_m: float) -> List[Waypoint]:
        cx, cy, r = region.cx, region.cy, region.outer_radius
        waypoints: List[Waypoint] = [(cx, cy)]
        for i in range(self.n_legs):
            theta = self.yaw_offset + i * (2.0 * math.pi / self.n_legs)
            waypoints.append((cx + r * math.cos(theta), cy + r * math.sin(theta)))
            waypoints.append((cx, cy))
        return waypoints


# partitioning


def partition_disk_into_sectors(
    cx: float, cy: float, radius: float, n_sectors: int,
    inner_radius: float = 0.0, base_yaw_rad: float = 0.0,
) -> List[AnnularSector]:
    """Split a disk into N equal angular sectors, listed CCW from base_yaw_rad."""
    if n_sectors < 1:
        raise ValueError("n_sectors must be >= 1")
    span = 2.0 * math.pi / n_sectors
    return [
        AnnularSector(
            cx=cx, cy=cy,
            inner_radius=inner_radius, outer_radius=radius,
            start_rad=base_yaw_rad + i * span,
            end_rad=base_yaw_rad + (i + 1) * span,
        )
        for i in range(n_sectors)
    ]


def partition_disk_into_strips(
    cx: float, cy: float, radius: float, n_strips: int,
) -> List[RectStrip]:
    """Split the disk's bounding square into N horizontal strips of equal area."""
    if n_strips < 1:
        raise ValueError("n_strips must be >= 1")
    full_h = 2.0 * radius
    strip_h = full_h / n_strips
    return [
        RectStrip(
            cx=cx, cy=cy + (i + 0.5) * strip_h - radius,
            width=2.0 * radius, height=strip_h, yaw=0.0,
        )
        for i in range(n_strips)
    ]


# strategy


@dataclass(frozen=True)
class CoveragePlan:
    """Typed return type for ``CoverageStrategy.plan_v2()``.

    Carries per-drone ``ScanPlan``s (waypoints + optional sector
    wedge) in one frozen value so the caller can iterate
    ``for drone, plan in zip(drones, coverage_plan.per_drone): ...``
    and pass each plan to ``Drone.set_plan(plan)`` without re-deriving
    the wedge from a 'sector_type' string sentinel branch in the
    caller.

    ``per_drone`` is a tuple of ``ScanPlan`` (from ``lib.domain``).
    Frozen so the planner's output is safely shared across consumers.
    """
    per_drone: tuple   # tuple[ScanPlan, ...], typed via TYPE_CHECKING import


@dataclass
class PlannerInput:
    """Configuration object handed to a CoverageStrategy at planning time.

    `seed` is used by stochastic strategies (RandomWalkStrategy) to make
    waypoint generation deterministic across runs. Deterministic strategies
    ignore it. Default 0 keeps existing tests/behaviour unchanged.
    """
    mission_center: Tuple[float, float]
    radius: float
    inner_radius: float
    n_drones: int
    footprint_m: float
    overlap: float = 0.3
    seed: int = 0

    def validate(self) -> None:
        """Eagerly validate every PlannerInput field at the strategy
        boundary. Each error message names the launch / scenario-YAML
        field that the caller should fix, not the internal pattern
        variable, so a missing scenario YAML key surfaces as a clear
        node-startup error instead of a mid-mission ValueError deep in
        the geometry code.
        """
        if self.n_drones < 1:
            raise ValueError(
                f"PlannerInput.n_drones must be >= 1 "
                f"(launch arg `num_drones`), got {self.n_drones}"
            )
        if self.footprint_m <= 0.0:
            raise ValueError(
                f"PlannerInput.footprint_m must be > 0 "
                f"(scenario `mission.camera_footprint_m`), "
                f"got {self.footprint_m}"
            )
        if self.radius <= 0.0:
            raise ValueError(
                f"PlannerInput.radius must be > 0 "
                f"(scenario `mission.mission_radius`), got {self.radius}"
            )
        if self.inner_radius < 0.0:
            raise ValueError(
                f"PlannerInput.inner_radius must be >= 0 "
                f"(scenario `mission.inner_radius`), got {self.inner_radius}"
            )
        if self.inner_radius >= self.radius:
            raise ValueError(
                f"PlannerInput.inner_radius ({self.inner_radius}) must "
                f"be < radius ({self.radius}) — check scenario "
                f"`mission.inner_radius` vs `mission.mission_radius`"
            )
        if not (0.0 <= self.overlap < 1.0):
            raise ValueError(
                f"PlannerInput.overlap must be in [0.0, 1.0) "
                f"(scenario `mission.coverage_overlap`), got {self.overlap}"
            )


class CoverageStrategy(ABC):
    """Template Method: orchestrates "partition the search area, then run a
    pattern per drone." Subclasses override `_partition_and_run` to implement
    the right partitioning for their pattern.

    Public entry: `plan(input) -> List[List[Waypoint]]`, one ordered waypoint
    list per drone.

    `sector_type` declares how the strategy partitions the disk so the
    mission_manager's auction can decide whether sector-ownership gating
    applies. Values:
      * 'angular': disk split into N angular wedges, one per drone
                    (spiral_out / spiral_in). The auction prefers the
                    wedge owner of a victim's bearing.
      * 'strip':   disk split into N horizontal strips (parallel_track).
                    Angular wedges don't apply; auction falls back to
                    nearest-drone.
      * 'full':    every drone receives the full disk (expanding_square,
                    sector_search) or independent random sampling
                    (random_walk). No partition; auction is unfiltered.
    """

    name: str = "abstract"
    description: str = ""
    # Typed declaration of how this strategy partitions
    # the disk. Replaces the stringly-typed `sector_type: str = 'full'`
    # sentinel. Subclasses override to PartitionKind.ANGULAR or
    # PartitionKind.STRIP. mission_manager branches on this to assign
    # angular sector wedges only when partition_kind is ANGULAR.
    partition_kind: PartitionKind = PartitionKind.FULL

    def plan(self, p: PlannerInput) -> List[List[Waypoint]]:
        # Full eager validation at the strategy boundary.
        # Every field-named error message points at the launch arg /
        # scenario YAML key the caller should fix.
        p.validate()
        return self._partition_and_run(p)

    def plan_v2(self, p: PlannerInput) -> 'CoveragePlan':
        """Typed return: `CoveragePlan(per_drone:
        tuple[ScanPlan, ...])` carrying waypoints AND the optional
        sector wedge for each drone in one frozen value.

        Default impl wraps ``plan()`` and assigns wedges based on
        ``sector_type``: 'angular' fills wedges from
        ``partition_disk_into_sectors``; 'strip' / 'full' leave wedges
        as None. Subclasses can override for finer control.

        ``plan()`` continues to exist for back-compat; mission_manager's
        ``_begin_scan`` still calls it. The migration to ``plan_v2()``
        happens as part of the mission_manager deconstruction.
        """
        per_drone_lists = self._partition_and_run(p)
        # Materialise wedges only when this strategy partitions angularly.
        wedges: List[Optional[object]] = [None] * len(per_drone_lists)
        if self.partition_kind == PartitionKind.ANGULAR:
            sectors = partition_disk_into_sectors(
                p.mission_center[0], p.mission_center[1],
                p.radius, p.n_drones, inner_radius=p.inner_radius,
            )
            from .domain import SectorWedge   # local import to avoid cycle
            wedges = [SectorWedge(start_rad=s.start_rad,
                                   end_rad=s.end_rad)
                      for s in sectors]
        from .domain import ScanPlan
        per_drone_plans = tuple(
            ScanPlan(
                waypoints=tuple(tuple(wp) for wp in wps),
                wedge=w,
            )
            for wps, w in zip(per_drone_lists, wedges)
        )
        return CoveragePlan(per_drone=per_drone_plans)

    @abstractmethod
    def _partition_and_run(self, p: PlannerInput) -> List[List[Waypoint]]:
        ...


class SpiralOutStrategy(CoverageStrategy):
    name = "spiral_out"
    partition_kind = PartitionKind.ANGULAR
    description = (
        "Concentric arcs from the launch pad outward, each drone sweeping "
        "its own 90° sector. Builds coverage gradually around the origin — "
        "early failure leaves the highest-density region covered."
    )

    def _partition_and_run(self, p: PlannerInput) -> List[List[Waypoint]]:
        sectors = partition_disk_into_sectors(
            *p.mission_center, p.radius, p.n_drones, inner_radius=p.inner_radius,
        )
        pat = ConcentricArcsPattern(overlap=p.overlap, outward=True)
        return [pat.generate_waypoints(s, p.footprint_m) for s in sectors]


class SpiralInStrategy(CoverageStrategy):
    name = "spiral_in"
    partition_kind = PartitionKind.ANGULAR
    description = (
        "Concentric arcs from the perimeter inward, each drone sweeping its "
        "own 90° sector. Useful when victims are expected to be near the "
        "boundary (e.g., people fleeing toward a known exit)."
    )

    def _partition_and_run(self, p: PlannerInput) -> List[List[Waypoint]]:
        sectors = partition_disk_into_sectors(
            *p.mission_center, p.radius, p.n_drones, inner_radius=p.inner_radius,
        )
        pat = ConcentricArcsPattern(overlap=p.overlap, outward=False)
        return [pat.generate_waypoints(s, p.footprint_m) for s in sectors]


class ExpandingSquareStrategy(CoverageStrategy):
    name = "expanding_square"
    description = (
        "IAMSAR §4.7 Expanding Square Search. Each drone spirals outward "
        "from the disk centre at a different cardinal heading, covering "
        "the highest-density region first."
    )

    def _partition_and_run(self, p: PlannerInput) -> List[List[Waypoint]]:
        disk = Disk(cx=p.mission_center[0], cy=p.mission_center[1],
                    outer_radius=p.radius)
        out: List[List[Waypoint]] = []
        for i in range(p.n_drones):
            yaw_offset = i * (2.0 * math.pi / p.n_drones)
            pat = ExpandingSquarePattern(overlap=p.overlap, yaw_offset=yaw_offset)
            out.append(pat.generate_waypoints(disk, p.footprint_m))
        return out


class ParallelTrackStrategy(CoverageStrategy):
    name = "parallel_track"
    partition_kind = PartitionKind.STRIP
    description = (
        "IAMSAR §4.4 classic lawnmower across the disk's bounding square. "
        "The disk is split into N horizontal strips; each drone sweeps "
        "parallel east-west tracks across its strip. Statistically optimal "
        "when the prior on victim location is uniform."
    )

    def _partition_and_run(self, p: PlannerInput) -> List[List[Waypoint]]:
        strips = partition_disk_into_strips(
            *p.mission_center, p.radius, p.n_drones,
        )
        pat = ParallelTrackPattern(overlap=p.overlap)
        return [pat.generate_waypoints(s, p.footprint_m) for s in strips]


class CreepingLineStrategy(CoverageStrategy):
    name = "creeping_line"
    partition_kind = PartitionKind.STRIP
    description = (
        "IAMSAR §4.5 creeping-line search — like parallel_track but the "
        "sweep legs run perpendicular to each strip's major axis. Complete "
        "coverage; complements parallel_track for elongated search areas."
    )

    def _partition_and_run(self, p: PlannerInput) -> List[List[Waypoint]]:
        strips = partition_disk_into_strips(
            *p.mission_center, p.radius, p.n_drones,
        )
        pat = CreepingLinePattern(overlap=p.overlap)
        return [pat.generate_waypoints(s, p.footprint_m) for s in strips]


class RandomWalkStrategy(CoverageStrategy):
    name = "random_walk"
    description = (
        "Baseline: uniform random sampling inside the search disk, one "
        "independent waypoint list per drone, seeded for reproducibility. "
        "Used as a no-coordination control against which structured "
        "patterns (spiral_out, expanding_square, parallel_track, "
        "sector_search) are measured. Pearson 1905."
    )

    def _partition_and_run(self, p: PlannerInput) -> List[List[Waypoint]]:
        # Per-drone deterministic streams: drone i gets seed = parent_seed + i.
        # This way two drones never produce the same sequence even when the
        # parent seed is small (e.g. 0), and reseeding the same parent
        # reproduces the exact set of N drone trajectories.
        disk = Disk(cx=p.mission_center[0], cy=p.mission_center[1],
                    outer_radius=p.radius)
        # Heuristic: match the order-of-magnitude waypoint count of the
        # structured strategies so coverage budget is comparable. For the
        # default scenario (R=70 m, footprint=35 m, overlap=0.85) ConcentricArcs
        # produces ~45 wps/drone; this gives 50.
        n_per_drone = max(20, int(
            math.pi * (p.radius ** 2) /
            ((1.0 - p.overlap) * (p.footprint_m ** 2) + 1e-6)
        ))
        out: List[List[Waypoint]] = []
        for i in range(p.n_drones):
            rng = random.Random(p.seed + i + 1)   # +1 so seed=0 still varies per drone
            pat = RandomWalkPattern(n_waypoints=n_per_drone, rng=rng)
            out.append(pat.generate_waypoints(disk, p.footprint_m))
        return out


class SectorSearchStrategy(CoverageStrategy):
    name = "sector_search"
    description = (
        "IAMSAR §4.6 Sector Search. Each drone flies radial legs from the "
        "disk centre out to the perimeter and back. Used when the search "
        "datum has very high confidence — denses, fast, repeated coverage."
    )

    def _partition_and_run(self, p: PlannerInput) -> List[List[Waypoint]]:
        disk = Disk(cx=p.mission_center[0], cy=p.mission_center[1],
                    outer_radius=p.radius)
        out: List[List[Waypoint]] = []
        legs_per_drone = max(2, 6 // max(p.n_drones, 1))
        for i in range(p.n_drones):
            yaw_offset = i * (2.0 * math.pi / p.n_drones)
            pat = SectorSearchPattern(n_legs=legs_per_drone, yaw_offset=yaw_offset)
            out.append(pat.generate_waypoints(disk, p.footprint_m))
        return out


# factory


class CoveragePatternFactory:
    """Registry-based factory for CoverageStrategy.

    Adding a new pattern: subclass CoverageStrategy, then call
    `CoveragePatternFactory.register(YourStrategy)` once at module import.
    """

    _registry: Dict[str, Type[CoverageStrategy]] = {}

    @classmethod
    def register(cls, strategy_cls: Type[CoverageStrategy]) -> None:
        if not strategy_cls.name or strategy_cls.name == "abstract":
            raise ValueError("strategy must define a non-abstract name")
        cls._registry[strategy_cls.name] = strategy_cls

    @classmethod
    def create(cls, name: str) -> CoverageStrategy:
        if name not in cls._registry:
            raise ValueError(
                f"unknown coverage pattern '{name}'. "
                f"Available: {sorted(cls._registry.keys())}"
            )
        return cls._registry[name]()

    @classmethod
    def list_names(cls) -> List[str]:
        return sorted(cls._registry.keys())

    @classmethod
    def describe(cls, name: str) -> str:
        return cls._registry[name].description if name in cls._registry else ""


# Register all built-in strategies once.
for _strategy in (SpiralOutStrategy, SpiralInStrategy,
                  ExpandingSquareStrategy, ParallelTrackStrategy,
                  CreepingLineStrategy,
                  SectorSearchStrategy, RandomWalkStrategy):
    CoveragePatternFactory.register(_strategy)

