"""Unit tests for lib/ports/executive_supervisor.py.

Pure-Python; no rclpy. Verifies the InMemoryExecutiveCapture test
fake and the structural-Protocol match.
"""
from __future__ import annotations

from drone_rescue_coordination.lib.domain.system_mode_machine import (
    SystemMode,
)
from drone_rescue_coordination.lib.ports.executive_supervisor import (
    ExecutiveSupervisor,
    InMemoryExecutiveCapture,
    LAYER_BOUNDARY,
)


def test_layer_boundary_annotation():
    """Protocol module carries 3T annotation."""
    assert LAYER_BOUNDARY == 'L2-L3'


def test_in_memory_default_mode_is_normal():
    cap = InMemoryExecutiveCapture()
    assert cap.current_mode() == SystemMode.NORMAL


def test_in_memory_records_drone_lost():
    cap = InMemoryExecutiveCapture()
    cap.on_drone_lost('drone1')
    cap.on_drone_lost('drone3')
    assert cap.drone_lost == ['drone1', 'drone3']


def test_in_memory_records_task_failed():
    cap = InMemoryExecutiveCapture()
    cap.on_task_failed(42, 'timeout')
    cap.on_task_failed(43, 'battery_low')
    assert cap.task_failed == [(42, 'timeout'), (43, 'battery_low')]


def test_in_memory_records_task_completed():
    cap = InMemoryExecutiveCapture()
    cap.on_task_completed(1)
    cap.on_task_completed(2)
    assert cap.task_completed == [1, 2]


def test_in_memory_mode_override():
    cap = InMemoryExecutiveCapture(mode=SystemMode.SAFE)
    assert cap.current_mode() == SystemMode.SAFE
    cap.set_mode(SystemMode.DEGRADED)
    assert cap.current_mode() == SystemMode.DEGRADED


def test_in_memory_matches_protocol_shape():
    """Duck-typing check: every required method exists."""
    cap = InMemoryExecutiveCapture()
    assert callable(cap.on_drone_lost)
    assert callable(cap.on_task_failed)
    assert callable(cap.on_task_completed)
    assert callable(cap.current_mode)


def test_executive_supervisor_protocol_is_a_protocol():
    """Sanity: the Protocol exists and is importable."""
    assert ExecutiveSupervisor is not None
