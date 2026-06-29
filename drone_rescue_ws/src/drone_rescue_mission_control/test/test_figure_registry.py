"""Tests for the FigureRenderer registry.

The registry pre-populates with the 8 legacy per-run figure builders.
This test pins the registered name set + the order so a future
silent-drop of a registration shows up in CI rather than as a
missing PDF page.
"""

from __future__ import annotations

from drone_rescue_mission_control.figures import (
    FigureRenderer,
    get_renderer,
    renderers,
)


_EXPECTED_NAMES_IN_ORDER = (
    'coverage_over_time',
    'cumulative_confirmed',
    'per_drone_battery',
    'per_drone_task_histogram',
    'trajectory_heatmap',
    'detection_latency_cdf',
    'victim_survival_curve',
    'detection_threshold_roc',
)


def test_registry_has_exactly_the_8_legacy_renderers():
    names = tuple(r.name for r in renderers())
    assert names == _EXPECTED_NAMES_IN_ORDER


def test_registry_labels_are_non_empty():
    for r in renderers():
        assert r.label, f'Renderer {r.name!r} has empty label'


def test_renderers_satisfy_protocol_shape():
    for r in renderers():
        assert hasattr(r, 'name')
        assert hasattr(r, 'label')
        assert hasattr(r, 'render')


def test_get_renderer_returns_registered_instance():
    r = get_renderer('coverage_over_time')
    assert r is not None
    assert r.name == 'coverage_over_time'


def test_get_renderer_unknown_returns_none():
    assert get_renderer('does_not_exist') is None


def test_renderers_defensive_copy():
    """The list returned by renderers() shouldn't share identity with
    the registry's internal list."""
    a = renderers()
    a.append(None)
    b = renderers()
    assert b[-1] is not None
