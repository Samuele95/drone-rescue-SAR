"""Unit tests for detection_cluster.py: DBSCAN + Bayesian fusion + the
multi-witness gate.

Exercises the algorithmic core of the detection-filter chain. No
ROS dependency; pure-python pytest.
"""

from __future__ import annotations

import math

import pytest

from drone_rescue_coordination.lib.detection_cluster import (
    Sighting,
    Cluster,
    dbscan,
    bayesian_fuse,
)


# DBSCAN

def _s(x: float, y: float, drone: str = 'd1', conf: float = 0.9, t: float = 0.0):
    return Sighting(x=x, y=y, confidence=conf, drone_name=drone,
                    t_seen=t, detection_type=0)


def test_dbscan_empty():
    assert dbscan([], eps=5.0, min_samples=2) == []


def test_dbscan_single_cluster():
    pts = [_s(0, 0), _s(1, 0), _s(0, 1), _s(1, 1)]
    clusters = dbscan(pts, eps=2.0, min_samples=2)
    assert len(clusters) == 1
    assert len(clusters[0].sightings) == 4


def test_dbscan_noise_filtered():
    """Two well-separated singletons should be noise (no clusters)."""
    pts = [_s(0, 0), _s(50, 50)]
    clusters = dbscan(pts, eps=5.0, min_samples=2)
    assert clusters == []


def test_dbscan_eps_boundary_inclusive():
    """eps is the inclusive distance threshold."""
    pts = [_s(0, 0), _s(5.0, 0)]
    assert len(dbscan(pts, eps=5.0, min_samples=2)) == 1
    # Move one tick past eps → noise.
    pts = [_s(0, 0), _s(5.001, 0)]
    assert dbscan(pts, eps=5.0, min_samples=2) == []


def test_dbscan_min_samples_gate():
    """Cluster of 2 should fire at min_samples=2 but not at 3."""
    pts = [_s(0, 0), _s(1, 1)]
    assert len(dbscan(pts, eps=3.0, min_samples=2)) == 1
    assert dbscan(pts, eps=3.0, min_samples=3) == []


def test_dbscan_two_separated_clusters():
    a = [_s(0, 0), _s(1, 0), _s(0, 1)]
    b = [_s(50, 50), _s(51, 50), _s(50, 51)]
    clusters = dbscan(a + b, eps=2.0, min_samples=2)
    assert len(clusters) == 2
    sizes = sorted(len(c.sightings) for c in clusters)
    assert sizes == [3, 3]


# Cluster properties

def test_cluster_centroid_confidence_weighted():
    """Higher-confidence points should pull the centroid toward them."""
    c = Cluster(sightings=[
        _s(0, 0, conf=0.1),
        _s(10, 0, conf=0.9),
    ])
    cx, cy = c.position
    # Weighted: (0*0.1 + 10*0.9) / (0.1+0.9) = 9.0
    assert cx == pytest.approx(9.0, rel=1e-3)
    assert cy == pytest.approx(0.0, abs=1e-9)


def test_cluster_distinct_drones():
    c = Cluster(sightings=[
        _s(0, 0, drone='d1'),
        _s(0, 1, drone='d1'),
        _s(0, 2, drone='d2'),
    ])
    assert c.distinct_drones == ['d1', 'd2']


def test_cluster_observation_count():
    c = Cluster(sightings=[_s(0, 0), _s(1, 0), _s(0, 1)])
    assert c.observation_count == 3


# Multi-witness gate

def test_witnesses_with_at_least_zero_returns_distinct():
    """k=0 degenerates to "count distinct drones, regardless of sightings"."""
    c = Cluster(sightings=[_s(0, 0, drone='d1'), _s(0, 0, drone='d2')])
    assert c.witnesses_with_at_least(0) == 2


def test_witnesses_filters_singleton_drones():
    """A drone that contributed only 1 sighting fails k>=2."""
    c = Cluster(sightings=[
        _s(0, 0, drone='d1'),
        _s(0, 0, drone='d1'),
        _s(0, 0, drone='d2'),
    ])
    assert c.witnesses_with_at_least(2) == 1   # only d1 has >=2


def test_witnesses_two_observers_both_qualify():
    c = Cluster(sightings=[
        _s(0, 0, drone='d1'), _s(0, 0, drone='d1'),
        _s(0, 0, drone='d2'), _s(0, 0, drone='d2'),
    ])
    assert c.witnesses_with_at_least(2) == 2


# Bayesian fusion

def test_bayesian_fuse_independence_under_repetition():
    """Two independent 0.5 sightings → 1 - (0.5)*(0.5) = 0.75."""
    assert bayesian_fuse([0.5, 0.5]) == pytest.approx(0.75, rel=1e-9)


def test_bayesian_fuse_monotonic_in_evidence():
    """Adding evidence never decreases confidence."""
    a = bayesian_fuse([0.7])
    b = bayesian_fuse([0.7, 0.5])
    c = bayesian_fuse([0.7, 0.5, 0.3])
    assert a < b < c


def test_bayesian_fuse_saturates_at_one():
    """A single confidence-1 observation saturates the fusion."""
    assert bayesian_fuse([1.0]) == pytest.approx(1.0)
    assert bayesian_fuse([1.0, 0.1, 0.2]) == pytest.approx(1.0)


def test_bayesian_fuse_empty_returns_zero():
    """No evidence → no confidence."""
    assert bayesian_fuse([]) == 0.0


def test_cluster_fused_confidence_matches_bayesian_fuse():
    """Cluster.fused_confidence is bayesian_fuse over its sightings'
    confidence values; check the contract."""
    confs = [0.4, 0.6, 0.8]
    c = Cluster(sightings=[_s(0, 0, conf=v) for v in confs])
    assert c.fused_confidence == pytest.approx(bayesian_fuse(confs), rel=1e-9)
