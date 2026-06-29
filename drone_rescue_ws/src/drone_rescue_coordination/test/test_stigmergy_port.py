"""Unit tests for lib/ports/stigmergy_port.py.

Pure-Python; no rclpy. Verifies that the InMemoryStigmergyGrid is
a structural ``StigmergyPort`` implementation and that the deposit
recording works for unit-test consumers.
"""
from __future__ import annotations

import numpy as np
import pytest

from drone_rescue_coordination.lib.domain.value_objects import Position
from drone_rescue_coordination.lib.ports.stigmergy_port import (
    InMemoryStigmergyGrid,
    StigmergyPort,
)


def test_in_memory_starts_empty():
    g = InMemoryStigmergyGrid()
    assert g.get_grid() is None
    assert g.deposits == []
    assert g.grid_origin() == (0.0, 0.0)
    assert g.cell_resolution() == 0.5


def test_in_memory_constructor_accepts_grid():
    arr = np.zeros((8, 8), dtype=np.float32)
    g = InMemoryStigmergyGrid(
        grid=arr, origin=(-10.0, -10.0), resolution=2.5,
    )
    assert g.get_grid() is arr
    assert g.grid_origin() == (-10.0, -10.0)
    assert g.cell_resolution() == 2.5


def test_in_memory_records_deposit():
    g = InMemoryStigmergyGrid()
    g.deposit(Position(5.0, 5.0, 25.0), strength=0.8)
    g.deposit(Position(10.0, -3.0, 25.0))
    assert len(g.deposits) == 2
    (p1, s1), (p2, s2) = g.deposits
    assert (p1.x, p1.y) == (5.0, 5.0)
    assert s1 == 0.8
    assert (p2.x, p2.y) == (10.0, -3.0)
    assert s2 == 1.0   # default


def test_in_memory_set_grid_swaps():
    g = InMemoryStigmergyGrid()
    arr = np.ones((4, 4), dtype=np.float32)
    g.set_grid(arr)
    assert g.get_grid() is arr


def test_protocol_runtime_structural_match():
    """``StigmergyPort`` is a typing.Protocol; isinstance via
    runtime_checkable is opt-in. We assert duck-shape instead:
    InMemoryStigmergyGrid has all four required methods."""
    g = InMemoryStigmergyGrid()
    assert hasattr(g, 'get_grid')
    assert hasattr(g, 'deposit')
    assert hasattr(g, 'grid_origin')
    assert hasattr(g, 'cell_resolution')


def test_port_module_exports_layer_boundary():
    """StigmergyPort module carries a 3T layer annotation so the
    future CI check can read it."""
    from drone_rescue_coordination.lib.ports import stigmergy_port
    assert stigmergy_port.LAYER_BOUNDARY == 'L1-stigmergy'
