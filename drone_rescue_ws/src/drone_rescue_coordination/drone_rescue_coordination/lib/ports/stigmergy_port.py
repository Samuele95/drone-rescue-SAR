"""StigmergyPort: explicit boundary for the pheromone medium.

The slides (Marcelletti, "Autonomous and Collaborative Robotics",
A.Y. 2025/26) frame swarm coordination via environment as a
first-class coordination mechanism:

> Slides pp. 123-126, Swarm Robotics: "Desired collective behavior
>   emerges from the interaction between the robots and the
>   interaction of robots with the environment."
>
> Slides pp. 88-90, Behaviour-Based Control: "No centralized
>   representation or control, individual behaviours can manage
>   data independently."

The pheromone grid arrives at ``surveyor.py`` as a raw
``Optional[np.ndarray]`` carried inside ``SurveyorSensors`` (the
ROS callback ``_on_pheromone_map`` fills it). This module names
that boundary so:

1. The Surveyor's L1 navigation policy can be tested with an
   in-memory stigmergy grid, not via a topic round-trip.
2. The pheromone-server implementation can be swapped (e.g. for a
   virtual potential field) without changing surveyor code.
3. The stigmergic coordination model is visible in the type
   system: a reader of ``lib/ports/`` sees that this project
   uses stigmergy as a named coordination mechanism, matching
   the slides' taxonomy.

The Surveyor LifecycleNode continues to receive the grid via the
ROS topic subscription; the adapter (``lib/ros_adapter/
pheromone_adapter.py``) wraps that subscription as a
``StigmergyPort`` instance.

3T boundary: ``LAYER_BOUNDARY = 'L1-stigmergy'``; the stigmergic
medium is L1 infrastructure consumed by L1 behaviours (the Surveyor
motor-schema blend).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Protocol

if TYPE_CHECKING:
    import numpy as np
    from ..domain.value_objects import Position


LAYER_BOUNDARY = 'L1-stigmergy'   # 3T architecture annotation.


class StigmergyPort(Protocol):
    """Driven port for the pheromone / stigmergic medium.

    The Surveyor's motor-schema blend (``lib.domain.navigation``)
    reads the grid via ``get_grid()``: the field returned is the
    same shape the existing ``SurveyorSensors.pheromone_grid``
    carries (``Optional[np.ndarray]`` of float32 cell weights).

    ``deposit()`` is the write-side: drones broadcast their
    presence so other drones can read it as repulsion. This is
    done by the per-drone ``pheromone_publisher`` publishing
    ``PointStamped`` messages onto the same topic; this Protocol
    abstracts that out of the surveyor's logic.

    ``cell_resolution()`` + ``grid_origin()`` expose the metric
    metadata so callers can index the grid without knowing the ROS
    message layout.
    """

    def get_grid(self) -> Optional['np.ndarray']:
        """Latest decayed pheromone-cell grid.

        ``None`` until the first ``PheromoneMap`` message arrives.
        Returned arrays are read-only by convention: the surveyor
        must not mutate them in place (decay/deposit happens on the
        server side).
        """
        ...

    def deposit(
        self, position: 'Position', strength: float = 1.0,
    ) -> None:
        """Deposit pheromone at ``position``.

        Idempotent w.r.t. the position: the server's decay model
        handles staleness. Strength is multiplied by the server's
        deposit-weight parameter.
        """
        ...

    def grid_origin(self) -> tuple[float, float]:
        """World-frame XY of the grid's (0, 0) cell."""
        ...

    def cell_resolution(self) -> float:
        """Metric size of a single grid cell (metres)."""
        ...


class InMemoryStigmergyGrid:
    """In-memory ``StigmergyPort`` for unit tests.

    Records deposits in a list and returns the last grid set via
    ``set_grid()``. Pure-Python; rclpy-free.
    """

    def __init__(
        self,
        *,
        grid: Optional['np.ndarray'] = None,
        origin: tuple[float, float] = (0.0, 0.0),
        resolution: float = 0.5,
    ):
        self._grid = grid
        self._origin = origin
        self._resolution = resolution
        self.deposits: list[tuple['Position', float]] = []

    def get_grid(self) -> Optional['np.ndarray']:
        return self._grid

    def deposit(
        self, position: 'Position', strength: float = 1.0,
    ) -> None:
        self.deposits.append((position, strength))

    def grid_origin(self) -> tuple[float, float]:
        return self._origin

    def cell_resolution(self) -> float:
        return self._resolution

    # Test-only helpers
    def set_grid(self, grid: Optional['np.ndarray']) -> None:
        self._grid = grid
