"""ParameterDeclarer: drive ``declare_parameter`` from PARAM_SCHEMA.

Closes the last drift site in the PARAM_SCHEMA cutover.
``lib.domain.scenario_schema.PARAM_SCHEMA`` is the single source of
truth for runtime-tweakability + YAML-block whitelisting; this module
makes it the single source for the declaration itself too.

Each LifecycleNode replaces its hand-rolled ``self.declare_parameter``
block with one call to ``declare_for_scope(self, ParamScope.MISSION)``
(or ``DETECTION``, etc.). Adding a new param becomes a one-row
PARAM_SCHEMA edit.

Pure-Python: uses ``node.declare_parameter`` via duck-typing, so no
rclpy import is required at module load.
"""

from __future__ import annotations

from typing import Any, FrozenSet, Mapping, Optional

from drone_rescue_coordination.lib.domain.scenario_schema import (
    PARAM_SCHEMA, ParamScope,
)


def declared_names_for_scope(scope: ParamScope) -> FrozenSet[str]:
    """The set of parameter names the declarer would emit for this
    scope. Useful for the post-on_configure regression test that
    pins schema-vs-node drift."""
    return frozenset(p.name for p in PARAM_SCHEMA if p.scope == scope)


def declare_for_scope(
    node,
    scope: ParamScope,
    defaults_override: Optional[Mapping[str, Any]] = None,
) -> None:
    """Iterate PARAM_SCHEMA filtered by ``scope`` and call
    ``node.declare_parameter(name, default)`` for each.

    ``defaults_override``: per-name override map. Used during the
    cutover to preserve legacy runtime defaults that diverge from
    the schema default. Once the divergent rows are reconciled in
    PARAM_SCHEMA, this argument falls into disuse.

    The node argument is duck-typed: anything exposing
    ``declare_parameter(name, default)`` works. Pass a real
    ``LifecycleNode`` in production; pass a fake in tests.
    """
    overrides = defaults_override or {}
    for param in PARAM_SCHEMA:
        if param.scope is not scope:
            continue
        default = overrides.get(param.name, param.default)
        node.declare_parameter(param.name, default)
