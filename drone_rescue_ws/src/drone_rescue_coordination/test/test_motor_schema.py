"""Unit tests for motor_schema_blend + MotorSchemaOutput.

The basis-behaviour named API that wraps the legacy positional
``blend()``. Pure-Python; no rclpy.
"""
from __future__ import annotations

import math

import pytest

from drone_rescue_coordination.lib.domain import navigation
from drone_rescue_coordination.lib.domain.navigation import (
    MotorSchemaOutput,
    motor_schema_blend,
)


def test_motor_schema_blend_returns_named_components():
    out = motor_schema_blend(
        repulsion=(1.0, 0.0),
        attraction=(0.0, 1.0),
        collision_avoidance=(0.0, 0.0),
        boundary=(0.0, 0.0),
        victim=(0.0, 0.0),
        weights=(1.0, 1.0, 0.0, 0.0, 0.0),
    )
    assert isinstance(out, MotorSchemaOutput)
    # Repulsion + attraction = (1, 1) -> unit-normalised to (sqrt(2)/2, sqrt(2)/2).
    assert math.isclose(out.nav_vector[0], math.sqrt(2) / 2, abs_tol=1e-9)
    assert math.isclose(out.nav_vector[1], math.sqrt(2) / 2, abs_tol=1e-9)
    # Component fields preserved verbatim.
    assert out.avoid_visited == (1.0, 0.0)
    assert out.explore_unvisited == (0.0, 1.0)
    assert out.avoid_peers == (0.0, 0.0)
    assert out.stay_inside == (0.0, 0.0)
    assert out.goal_seek_victim == (0.0, 0.0)


def test_motor_schema_blend_zero_weights_zero_vector():
    out = motor_schema_blend(
        repulsion=(1.0, 0.0),
        attraction=(0.0, 1.0),
        collision_avoidance=(0.0, 0.0),
        boundary=(0.0, 0.0),
        victim=(0.0, 0.0),
        weights=(0.0, 0.0, 0.0, 0.0, 0.0),
    )
    assert out.nav_vector == (0.0, 0.0)


def test_motor_schema_blend_canceling_components_zero_vector():
    """Opposing repulsion/attraction with equal weights: blend cancels."""
    out = motor_schema_blend(
        repulsion=(1.0, 0.0),
        attraction=(-1.0, 0.0),
        collision_avoidance=(0.0, 0.0),
        boundary=(0.0, 0.0),
        victim=(0.0, 0.0),
        weights=(1.0, 1.0, 0.0, 0.0, 0.0),
    )
    assert out.nav_vector == (0.0, 0.0)


def test_motor_schema_blend_wrong_weight_count_raises():
    with pytest.raises(ValueError, match='5 weights'):
        motor_schema_blend(
            repulsion=(0, 0), attraction=(0, 0),
            collision_avoidance=(0, 0), boundary=(0, 0), victim=(0, 0),
            weights=(1.0, 1.0, 1.0),
        )


def test_motor_schema_blend_negative_weight_raises():
    with pytest.raises(ValueError, match='non-negative'):
        motor_schema_blend(
            repulsion=(0, 0), attraction=(0, 0),
            collision_avoidance=(0, 0), boundary=(0, 0), victim=(0, 0),
            weights=(1.0, -0.1, 1.0, 1.0, 1.0),
        )


def test_motor_schema_blend_pure_victim_pulls_toward_victim():
    """When only the victim component fires, the result is the victim
    unit direction; confirms each basis behaviour can drive the
    output in isolation."""
    out = motor_schema_blend(
        repulsion=(0.0, 0.0),
        attraction=(0.0, 0.0),
        collision_avoidance=(0.0, 0.0),
        boundary=(0.0, 0.0),
        victim=(0.7071, 0.7071),
        weights=(1.0, 1.0, 1.0, 1.0, 1.0),
    )
    assert math.isclose(out.nav_vector[0], math.sqrt(2) / 2, abs_tol=1e-3)
    assert math.isclose(out.nav_vector[1], math.sqrt(2) / 2, abs_tol=1e-3)


def test_motor_schema_output_is_frozen():
    out = motor_schema_blend(
        repulsion=(0, 0), attraction=(0, 0),
        collision_avoidance=(0, 0), boundary=(0, 0), victim=(0, 0),
        weights=(0, 0, 0, 0, 0),
    )
    with pytest.raises(Exception):
        out.nav_vector = (1.0, 1.0)   # type: ignore[misc]


# `blend()` now emits DeprecationWarning; silence the warning inside
# these shim-coverage tests so the suite stays clean. Production callers
# no longer use blend().
@pytest.mark.filterwarnings('ignore::DeprecationWarning')
def test_legacy_blend_still_works_unchanged():
    """The positional `blend()` shim preserves the legacy API exactly."""
    nav = navigation.blend(
        vectors=((1.0, 0.0), (0.0, 1.0)),
        weights=(1.0, 1.0),
    )
    assert math.isclose(nav[0], math.sqrt(2) / 2, abs_tol=1e-9)
    assert math.isclose(nav[1], math.sqrt(2) / 2, abs_tol=1e-9)


@pytest.mark.filterwarnings('ignore::DeprecationWarning')
def test_legacy_blend_negative_weight_raises():
    with pytest.raises(ValueError, match='non-negative'):
        navigation.blend(vectors=[(1.0, 0.0)], weights=[-1.0])


@pytest.mark.filterwarnings('ignore::DeprecationWarning')
def test_legacy_blend_mismatched_lengths_raises():
    with pytest.raises(ValueError, match='parallel sequences'):
        navigation.blend(vectors=[(1.0, 0.0)], weights=[1.0, 1.0])


def test_blend_emits_deprecation_warning():
    """Every call to blend() emits a DeprecationWarning so reviewers
    see the migration target at use-site."""
    with pytest.warns(DeprecationWarning, match='motor_schema_blend'):
        navigation.blend(
            vectors=((1.0, 0.0),),
            weights=(1.0,),
        )
