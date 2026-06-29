"""Lightweight terrain-elevation model: pure Python, no rclpy.

The search area was previously assumed perfectly flat: scan waypoints were
flown at a fixed altitude above the world datum, so over sloped terrain the
camera's above-ground height (and therefore its ground footprint and the
coverage track spacing) would drift.

``ElevationModel`` gives ``mission_manager`` the terrain height under any
(x, y), so each scan waypoint can be flown at a constant above-ground-level
height (``survey_altitude + elevation_at(x, y)``). The footprint then stays
consistent and the existing coverage patterns need no change.

The default model is ``flat`` (0.0 everywhere): the altitude offset is +0 and
the simulation behaves as before. A ``planar`` model adds a constant gradient;
richer models (loaded grids, analytic hills) can subclass or extend
``elevation_at`` without touching callers.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ElevationModel:
    """Terrain elevation (metres, relative to the world datum) under the
    search disk.

    A frozen value object: the fields are set at construction and never
    mutated, and the model has no identity. ``elevation_at`` is a pure
    function of the immutable parameters.

    Args:
        kind: ``'flat'`` (default, 0 everywhere) or ``'planar'``.
        base: constant offset added to every sample (planar only).
        slope_x, slope_y: elevation gradient per metre along world x / y
            (planar only). Both 0 makes ``planar`` equivalent to ``flat``.
    """

    VALID_KINDS = ('flat', 'planar')

    kind: str = 'flat'
    base: float = 0.0
    slope_x: float = 0.0
    slope_y: float = 0.0

    def __post_init__(self):
        if self.kind not in self.VALID_KINDS:
            raise ValueError(
                f"unknown elevation model kind '{self.kind}'. "
                f"Available: {list(self.VALID_KINDS)}"
            )
        # Coerce numeric inputs to float (frozen VO, so object.__setattr__).
        object.__setattr__(self, 'base', float(self.base))
        object.__setattr__(self, 'slope_x', float(self.slope_x))
        object.__setattr__(self, 'slope_y', float(self.slope_y))

    def elevation_at(self, x: float, y: float) -> float:
        """Terrain height at world ``(x, y)`` in metres."""
        if self.kind == 'flat':
            return 0.0
        # planar
        return self.base + self.slope_x * x + self.slope_y * y

    @property
    def is_flat(self) -> bool:
        """True when the model is identically zero (the no-op default)."""
        return self.kind == 'flat' or (
            self.base == 0.0 and self.slope_x == 0.0 and self.slope_y == 0.0
        )

    @classmethod
    def flat(cls) -> 'ElevationModel':
        """The default no-op model: 0 elevation everywhere."""
        return cls('flat')

    @classmethod
    def from_slopes(cls, slope_x: float, slope_y: float,
                    base: float = 0.0) -> 'ElevationModel':
        """Build a planar model from x/y gradients (the form mission_manager
        constructs from its ``terrain_slope_x`` / ``terrain_slope_y`` params).
        Returns the flat model when both slopes and the base are zero."""
        if slope_x == 0.0 and slope_y == 0.0 and base == 0.0:
            return cls.flat()
        return cls('planar', base=base, slope_x=slope_x, slope_y=slope_y)
