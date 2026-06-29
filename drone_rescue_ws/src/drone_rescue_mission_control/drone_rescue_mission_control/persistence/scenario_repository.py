"""ScenarioRepository: driven port for "where do scenarios live?".

Mirrors ``RunRepository``. Three production sites
(``scenario_loader.discover_scenarios``,
``mission_recorder._load_scenario_params``, ``bench._scenario_path``)
hand-roll YAML loading with subtly different error handling. This
Protocol, and the production ``YamlScenarioRepository`` adapter,
collapse them to one persistence boundary.

Tests can implement the Protocol with ``InMemoryScenarioRepository``
or a dict of scenarios so the bench / Mission Control logic runs
without YAML files on disk.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Protocol, Sequence

from ..scenario_loader import Scenario, default_scenarios_dir, load_scenario


class ScenarioRepository(Protocol):
    """Where do scenarios live? Driven port.

    `list()` returns every valid scenario discoverable.
    `load(name)` resolves a scenario by its `name` field.
    `by_path(path)` loads a scenario at a specific path (the
    mission_recorder reads ``scenario_yaml`` as a path string).
    """

    def list(self) -> Sequence[Scenario]: ...

    def load(self, name: str) -> Optional[Scenario]: ...

    def by_path(self, path: Path) -> Scenario: ...


class YamlScenarioRepository:
    """Production adapter: reads YAML scenarios from a directory tree.

    Defaults to ``default_scenarios_dir()`` (the bringup package's
    install-tree ``scenarios/`` directory). Caches discovery results
    per instance; create a fresh repository between unrelated calls
    if you need to pick up filesystem additions.
    """

    def __init__(self, scenarios_dir: Optional[Path] = None) -> None:
        self._scenarios_dir = (
            Path(scenarios_dir) if scenarios_dir is not None
            else default_scenarios_dir()
        )
        self._cache: Optional[List[Scenario]] = None

    @property
    def scenarios_dir(self) -> Path:
        return self._scenarios_dir

    def list(self) -> Sequence[Scenario]:
        if self._cache is None:
            from ..scenario_loader import discover_scenarios
            self._cache = list(discover_scenarios(self._scenarios_dir))
        return tuple(self._cache)

    def load(self, name: str) -> Optional[Scenario]:
        for s in self.list():
            if s.name == name:
                return s
        return None

    def by_path(self, path: Path) -> Scenario:
        return load_scenario(Path(path))


class InMemoryScenarioRepository:
    """Test double: wraps a list/dict of scenarios. No filesystem."""

    def __init__(self, scenarios: Sequence[Scenario]) -> None:
        self._scenarios: List[Scenario] = list(scenarios)
        self._by_name: Dict[str, Scenario] = {s.name: s for s in scenarios}
        self._by_path: Dict[Path, Scenario] = {
            s.path: s for s in scenarios if s.path is not None
        }

    @property
    def scenarios_dir(self) -> Path:
        return Path('/in-memory')

    def list(self) -> Sequence[Scenario]:
        return tuple(self._scenarios)

    def load(self, name: str) -> Optional[Scenario]:
        return self._by_name.get(name)

    def by_path(self, path: Path) -> Scenario:
        key = Path(path)
        if key not in self._by_path:
            raise FileNotFoundError(key)
        return self._by_path[key]
