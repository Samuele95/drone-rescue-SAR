"""Unit coverage for the lifted ParameterDeclarer.

The post-on_configure declared-name set MUST match PARAM_SCHEMA per
scope. The schema is the single source of truth. A regression where
someone hand-rolls a new ``declare_parameter`` in a node without
adding the PARAM_SCHEMA row is what this pins.
"""

from drone_rescue_coordination.lib.domain.scenario_schema import (
    PARAM_SCHEMA, ParamScope,
)
from drone_rescue_coordination.lib.ros_adapter.parameter_declarer import (
    declared_names_for_scope, declare_for_scope,
)


class _FakeNode:
    """Duck-typed stand-in for LifecycleNode: only
    ``declare_parameter(name, default)`` is needed."""

    def __init__(self):
        self.declared = []

    def declare_parameter(self, name, default):
        self.declared.append((name, default))


def test_declared_names_for_scope_matches_schema_filter():
    expected = frozenset(
        p.name for p in PARAM_SCHEMA if p.scope is ParamScope.MISSION
    )
    assert declared_names_for_scope(ParamScope.MISSION) == expected


def test_declare_for_scope_calls_declare_parameter_per_row():
    node = _FakeNode()
    declare_for_scope(node, ParamScope.LAUNCH)
    names = {n for n, _ in node.declared}
    assert names == declared_names_for_scope(ParamScope.LAUNCH)


def test_declare_for_scope_uses_schema_default_when_no_override():
    node = _FakeNode()
    declare_for_scope(node, ParamScope.DETECTION)
    pair_dict = dict(node.declared)
    # confidence_floor's schema default is 0.65.
    assert pair_dict['confidence_floor'] == 0.65


def test_declare_for_scope_overrides_default_when_supplied():
    node = _FakeNode()
    declare_for_scope(
        node, ParamScope.DETECTION,
        defaults_override={'confidence_floor': 0.4},
    )
    pair_dict = dict(node.declared)
    assert pair_dict['confidence_floor'] == 0.4


def test_declare_for_scope_ignores_unrelated_scope_overrides():
    """An override for a name in a different scope is a no-op for
    this scope. Prevents a typo'd override from silently breaking
    cross-scope assumptions."""
    node = _FakeNode()
    declare_for_scope(
        node, ParamScope.LAUNCH,
        defaults_override={'confidence_floor': 9.99},
    )
    names = {n for n, _ in node.declared}
    assert 'confidence_floor' not in names


def test_schema_partition_does_not_overlap():
    """PARAM_SCHEMA rows are partitioned by scope: no name appears
    under two scopes."""
    by_name = {}
    for p in PARAM_SCHEMA:
        by_name.setdefault(p.name, set()).add(p.scope)
    duplicates = {n: s for n, s in by_name.items() if len(s) > 1}
    assert not duplicates, f'name spans multiple scopes: {duplicates}'
