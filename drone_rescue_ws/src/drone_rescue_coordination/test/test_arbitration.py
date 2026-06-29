"""Tests for the ArbitrationStrategy seam.

The critical guarantee here is bit-identity: ``MotorSchemaArbitration``
must reproduce the legacy ``navigation.motor_schema_blend`` vector sum
exactly, so the refactor cannot have silently changed the
deployed navigation behaviour. The remaining tests pin the
``SubsumptionArbitration`` semantics that prove the seam is a genuine
strategy, not a rename.
"""

from __future__ import annotations

import math

from drone_rescue_coordination.lib.domain import navigation
from drone_rescue_coordination.lib.domain.arbitration import (
    MotorSchemaArbitration,
    SubsumptionArbitration,
)
from drone_rescue_coordination.lib.domain.behaviours import (
    BASIS_BEHAVIOUR_NAMES,
)


# Several representative component sets: a generic mix, an asymmetric
# set, and one that cancels to zero.
_CASES = [
    (
        ((1.0, 0.0), (0.0, 2.0), (-0.5, 0.3), (0.0, 0.0), (0.7, -1.1)),
        (1.0, 1.0, 1.5, 1.2, 1.4),
    ),
    (
        ((0.3, 0.9), (-0.2, 0.1), (0.0, 0.0), (2.0, -0.4), (0.05, 0.05)),
        (0.8, 1.3, 0.0, 2.0, 0.5),
    ),
    (
        ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0), (0.0, 0.0)),
        (1.0, 1.0, 1.0, 1.0, 1.0),
    ),
]


def _named(components):
    """Map a 5-tuple of vectors onto the canonical behaviour names,
    preserving declared order (so the dict iteration order matches the
    legacy positional blend)."""
    return {name: vec for name, vec in zip(BASIS_BEHAVIOUR_NAMES, components)}


def test_motor_schema_arbitration_matches_legacy_blend_bit_for_bit():
    """MotorSchemaArbitration.combine == motor_schema_blend.nav_vector,
    exactly (not approximately): the regression lock for the
    refactor."""
    arb = MotorSchemaArbitration()
    for components, weights in _CASES:
        outputs = _named(components)
        weight_map = dict(zip(BASIS_BEHAVIOUR_NAMES, weights))
        got = arb.combine(outputs, weight_map)
        legacy = navigation.motor_schema_blend(
            repulsion=components[0],
            attraction=components[1],
            collision_avoidance=components[2],
            boundary=components[3],
            victim=components[4],
            weights=weights,
        ).nav_vector
        assert got == legacy   # exact float equality, by construction


def test_motor_schema_arbitration_output_is_unit_or_zero():
    arb = MotorSchemaArbitration()
    for components, weights in _CASES:
        got = arb.combine(_named(components), dict(
            zip(BASIS_BEHAVIOUR_NAMES, weights)))
        mag = math.hypot(*got)
        assert math.isclose(mag, 1.0, abs_tol=1e-9) or got == (0.0, 0.0)


def test_motor_schema_cancelling_components_give_zero():
    arb = MotorSchemaArbitration()
    outputs = {'a': (1.0, 0.0), 'b': (-1.0, 0.0)}
    assert arb.combine(outputs, {'a': 1.0, 'b': 1.0}) == (0.0, 0.0)


def test_subsumption_highest_priority_nonzero_wins():
    """First (highest-priority) behaviour with a non-negligible vector
    suppresses the rest; result is its normalised direction."""
    arb = SubsumptionArbitration()
    outputs = {
        'B1-avoid-visited': (0.0, 0.0),     # silent, suppressed layer
        'B2-explore-unvisited': (3.0, 4.0),  # first speaker -> wins
        'B5-goal-seek': (0.0, 9.0),          # lower priority, ignored
    }
    got = arb.combine(outputs, {})
    assert math.isclose(got[0], 0.6, abs_tol=1e-9)
    assert math.isclose(got[1], 0.8, abs_tol=1e-9)


def test_subsumption_skips_below_threshold():
    arb = SubsumptionArbitration(suppression_threshold=0.5)
    outputs = {'a': (0.1, 0.0), 'b': (0.0, 2.0)}   # 'a' below threshold
    got = arb.combine(outputs, {})
    assert math.isclose(got[0], 0.0, abs_tol=1e-9)
    assert math.isclose(got[1], 1.0, abs_tol=1e-9)


def test_subsumption_all_silent_returns_zero():
    arb = SubsumptionArbitration()
    assert arb.combine({'a': (0.0, 0.0), 'b': (0.0, 0.0)}, {}) == (0.0, 0.0)


def test_motor_schema_and_subsumption_differ_on_same_inputs():
    """Same behaviour outputs, different strategy -> different command:
    the seam is a real choice, not cosmetic."""
    outputs = {'B1-avoid-visited': (1.0, 0.0), 'B5-goal-seek': (0.0, 2.0)}
    weights = {'B1-avoid-visited': 1.0, 'B5-goal-seek': 1.0}
    ms = MotorSchemaArbitration().combine(outputs, weights)
    sub = SubsumptionArbitration().combine(outputs, weights)
    assert ms != sub
