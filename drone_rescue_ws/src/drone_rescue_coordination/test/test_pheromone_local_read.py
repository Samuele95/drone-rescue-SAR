"""Regression test for PheromoneServer.get_local_pheromones.

get_local_pheromones read the shared grid WITHOUT holding _grid_lock, while the
decay/deposit timer mutates self.grid in place under the lock; an unlocked read
could observe a half-applied update. The read now takes _grid_lock like every
other grid access; this test pins that the window contents are still correct and
that the lock can be re-acquired afterwards (i.e. the read releases it).
"""

from __future__ import annotations

import pytest
import rclpy

from drone_rescue_coordination.pheromone_server import PheromoneServer


@pytest.fixture(scope='module', autouse=True)
def _rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def server():
    node = PheromoneServer()
    yield node
    node.destroy_node()


def test_local_window_size_matches_radius(server):
    assert len(server.get_local_pheromones(0.0, 0.0, radius=1)) == 9   # 3x3
    assert len(server.get_local_pheromones(0.0, 0.0, radius=2)) == 25  # 5x5


def test_read_releases_the_lock(server):
    """A second call (and any other locked grid op) must still succeed, i.e.
    get_local_pheromones does not leave _grid_lock held."""
    server.get_local_pheromones(0.0, 0.0, radius=1)
    # If the lock had leaked, this would deadlock; it returns promptly.
    again = server.get_local_pheromones(5.0, -5.0, radius=1)
    assert len(again) == 9
    assert not server._grid_lock.locked()


def test_local_read_reflects_grid_contents(server):
    """A value written to the grid is observed by the locked read; behaviour
    preserved under the lock."""
    row, col = server.world_to_grid(0.0, 0.0)
    server.grid[row, col] = 0.7
    cells = server.get_local_pheromones(0.0, 0.0, radius=1)
    assert (row, col, pytest.approx(0.7)) in [
        (r, c, pytest.approx(v)) for r, c, v in cells
    ]
