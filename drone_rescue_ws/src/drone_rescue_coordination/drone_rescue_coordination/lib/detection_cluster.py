"""DBSCAN clustering + Bayesian fusion of victim sightings.

DBSCAN is implemented inline (no scikit-learn dependency) to keep the package
deps minimal. For the small N expected here (a few hundred sightings over a
30 s window), a quadratic neighbour search is fine.

References:
  * Ester, Kriegel, Sander, Xu. "A Density-Based Algorithm for Discovering
    Clusters in Large Spatial Databases with Noise." KDD 1996.
  * Pearl. "Probabilistic Reasoning in Intelligent Systems." 1988
    (Bayesian independent-evidence fusion).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Tuple


@dataclass
class Sighting:
    """A single raw detection reported by one drone at one moment in time."""
    x: float
    y: float
    confidence: float           # in (0, 1]
    drone_name: str
    t_seen: float               # ROS time, seconds
    detection_type: int = 0     # mirrors VictimDetection.detection_type


@dataclass
class Cluster:
    sightings: List[Sighting] = field(default_factory=list)

    @property
    def position(self) -> Tuple[float, float]:
        """Confidence-weighted centroid."""
        if not self.sightings:
            return (0.0, 0.0)
        wsum = sum(max(s.confidence, 1e-3) for s in self.sightings)
        x = sum(s.x * max(s.confidence, 1e-3) for s in self.sightings) / wsum
        y = sum(s.y * max(s.confidence, 1e-3) for s in self.sightings) / wsum
        return (x, y)

    @property
    def fused_confidence(self) -> float:
        """Bayesian fusion under independence: P = 1 - prod(1 - p_i)."""
        prod = 1.0
        for s in self.sightings:
            prod *= max(0.0, 1.0 - min(1.0, s.confidence))
        return 1.0 - prod

    @property
    def distinct_drones(self) -> List[str]:
        return sorted({s.drone_name for s in self.sightings})

    @property
    def observation_count(self) -> int:
        return len(self.sightings)

    def witnesses_with_at_least(self, k: int) -> int:
        """Number of distinct drones contributing at least `k` sightings.

        Used as a stricter confirmation gate than `len(distinct_drones)`:
        a real victim, once INVESTIGATEd by a hovering drone, accumulates
        many sightings from that drone, but that single drone is one
        viewpoint and one detector instance. Requiring multiple drones
        each to hover/observe at least k times forces the cluster to
        survive at least one CONFIRM dispatch by a different drone, which
        is what filters transient HSV pops on world textures.
        """
        if k <= 0:
            return len({s.drone_name for s in self.sightings})
        counts: dict = {}
        for s in self.sightings:
            counts[s.drone_name] = counts.get(s.drone_name, 0) + 1
        return sum(1 for c in counts.values() if c >= k)


def dbscan(
    points: List[Sighting],
    eps: float,
    min_samples: int,
) -> List[Cluster]:
    """DBSCAN clustering of `points` by their (x, y) coordinates.

    Returns a list of Cluster objects, one per dense region. Noise points
    (insufficient density) are dropped: they cannot become candidates.

    `min_samples` follows the original Ester, Kriegel, Sander, Xu (KDD
    1996) convention: a point is a CORE point iff it has at least
    ``min_samples`` points within its eps-neighbourhood **including
    itself**. So `min_samples=2` means "the point plus at least one
    other neighbour"; `min_samples=3` means "the point plus at least
    two others". Internally we model this by counting
    ``len(neighbours(i)) + 1`` (the +1 is the point itself, since
    ``neighbours()`` returns strictly other indices) and comparing to
    ``min_samples``.
    """
    n = len(points)
    if n == 0:
        return []

    UNVISITED, VISITED, NOISE = 0, 1, 2
    label = [UNVISITED] * n
    cluster_id = [-1] * n

    # Replace the O(N) linear neighbour scan with a
    # fixed-cell grid hash. Bucket side = eps so any point within eps
    # of point i must lie in cell (cx_i, cy_i) or one of its 8 neighbours
    # (the standard 3x3 cell stencil). Build cost is O(N); each
    # neighbours() call drops to O(k) where k is the average bucket
    # population. For the swarm-size operating point (4 drones x 2 Hz
    # x 60 s window = 480 sightings spread over ~70 m radius) this is
    # ~5 points/cell on average. Total DBSCAN drops from O(N^2) to
    # roughly O(N*k) ~ O(N).
    eps_sq = eps * eps
    grid: Dict[Tuple[int, int], List[int]] = {}
    for j, p in enumerate(points):
        key = (int(p.x // eps), int(p.y // eps))
        grid.setdefault(key, []).append(j)

    def neighbours(i: int) -> List[int]:
        xi, yi = points[i].x, points[i].y
        cx, cy = int(xi // eps), int(yi // eps)
        out: List[int] = []
        for dcx in (-1, 0, 1):
            for dcy in (-1, 0, 1):
                bucket = grid.get((cx + dcx, cy + dcy))
                if bucket is None:
                    continue
                for j in bucket:
                    if j == i:
                        continue
                    dx = points[j].x - xi
                    dy = points[j].y - yi
                    if dx * dx + dy * dy <= eps_sq:
                        out.append(j)
        return out

    cid = 0
    for i in range(n):
        if label[i] != UNVISITED:
            continue
        label[i] = VISITED
        nb = neighbours(i)
        if len(nb) + 1 < min_samples:
            label[i] = NOISE
            continue
        cluster_id[i] = cid
        seeds = list(nb)
        k = 0
        while k < len(seeds):
            j = seeds[k]
            if label[j] == UNVISITED:
                label[j] = VISITED
                nb_j = neighbours(j)
                if len(nb_j) + 1 >= min_samples:
                    for m in nb_j:
                        if m not in seeds:
                            seeds.append(m)
            if cluster_id[j] == -1:
                cluster_id[j] = cid
            k += 1
        cid += 1

    clusters: List[Cluster] = [Cluster() for _ in range(cid)]
    for i in range(n):
        if cluster_id[i] != -1:
            clusters[cluster_id[i]].sightings.append(points[i])
    return clusters


def bayesian_fuse(probs: Iterable[float]) -> float:
    """Independent-evidence fusion: P = 1 - prod(1 - p)."""
    prod = 1.0
    for p in probs:
        prod *= max(0.0, 1.0 - min(1.0, p))
    return 1.0 - prod
