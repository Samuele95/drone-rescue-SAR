"""Tests for the ScenarioRepository port."""

from __future__ import annotations

from pathlib import Path

from drone_rescue_mission_control.persistence import (
    InMemoryScenarioRepository, ScenarioRepository,
)
from drone_rescue_mission_control.scenario_loader import Scenario


def _mk(name: str, path: str = '/tmp/s.yaml') -> Scenario:
    return Scenario(path=Path(path), name=name)


def test_in_memory_list_returns_provided_scenarios():
    s1 = _mk('a', '/tmp/a.yaml')
    s2 = _mk('b', '/tmp/b.yaml')
    repo = InMemoryScenarioRepository([s1, s2])
    items = repo.list()
    assert {s.name for s in items} == {'a', 'b'}


def test_in_memory_load_by_name():
    s1 = _mk('alpha', '/tmp/a.yaml')
    repo = InMemoryScenarioRepository([s1])
    assert repo.load('alpha') is s1
    assert repo.load('missing') is None


def test_in_memory_by_path():
    s1 = _mk('alpha', '/tmp/a.yaml')
    repo = InMemoryScenarioRepository([s1])
    assert repo.by_path(Path('/tmp/a.yaml')) is s1


def test_in_memory_by_path_raises_for_missing():
    repo = InMemoryScenarioRepository([])
    try:
        repo.by_path(Path('/missing'))
        assert False, 'expected FileNotFoundError'
    except FileNotFoundError:
        pass


def test_protocol_structural_typing():
    """InMemoryScenarioRepository must satisfy ScenarioRepository
    structurally; Mission Control / bench can substitute it
    without inheritance ceremonies."""
    repo: ScenarioRepository = InMemoryScenarioRepository([])
    # ScenarioRepository is a Protocol; the assignment is the test.
    assert hasattr(repo, 'list')
    assert hasattr(repo, 'load')
    assert hasattr(repo, 'by_path')
