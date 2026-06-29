"""ScenarioRepository: driven port for scenario lookup.

The composition root used to construct the scenario repository by importing
``drone_rescue_mission_control.persistence.YamlScenarioRepository`` directly,
a boundary inversion, because mission_control depends on coordination, not the
other way round. coordination must not reach up into the launcher/analytics
package.

This port names the contract coordination's ``CompositionRoot`` needs. The
mission_control layer (``mission_recorder``) implements it
(``YamlScenarioRepository``) and INJECTS the adapter via
``CompositionRoot.for_node(..., scenario_repo=...)``; coordination never imports
mission_control. Anything structurally matching this Protocol qualifies, so no
import edge is created in either direction.
"""

from __future__ import annotations

from typing import Optional, Protocol, Sequence, runtime_checkable


@runtime_checkable
class ScenarioRepository(Protocol):
    """Read-side lookup of scenarios by name or path.

    Mirrors the surface mission_control's ``YamlScenarioRepository`` already
    exposes: ``list`` returns every discoverable scenario, ``load`` resolves one
    by its ``name`` field, ``by_path`` loads the scenario at a given path. The
    return type is left as ``object`` here because the concrete ``Scenario`` VO
    lives in the mission_control layer; coordination only passes the repository
    through to the injecting layer, it never inspects the results.
    """

    def list(self) -> Sequence[object]: ...

    def load(self, name: str) -> Optional[object]: ...

    def by_path(self, path) -> object: ...
