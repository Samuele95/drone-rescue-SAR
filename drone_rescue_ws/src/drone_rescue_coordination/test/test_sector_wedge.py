"""Unit tests for SectorWedge.midpoint() and .absorb().

Pure-Python pytest; no ROS node. These methods own the wedge geometry
that the mission_manager drone-loss handover delegates to.
"""

from __future__ import annotations

import math

import pytest

from drone_rescue_coordination.lib.domain.value_objects import SectorWedge


def test_midpoint_of_normal_wedge():
    assert SectorWedge(0.0, math.pi / 2).midpoint() == pytest.approx(
        math.pi / 4)


def test_midpoint_of_wide_wedge():
    assert SectorWedge(math.pi / 2, 3 * math.pi / 2).midpoint() == pytest.approx(
        math.pi)


def test_midpoint_of_wraparound_wedge():
    # [7π/4, π/4) straddles the 0/2π seam: width is π/2, centre is 0.
    mid = SectorWedge(7 * math.pi / 4, math.pi / 4).midpoint()
    assert mid == pytest.approx(0.0) or mid == pytest.approx(2 * math.pi)


def test_absorb_extends_end_edge():
    # receiver wedge sits "before" the other: its end edge is the
    # shared boundary and is pushed out to the other's end.
    receiver = SectorWedge(2.0, 3.0)
    merged = receiver.absorb(SectorWedge(3.0, 4.0))
    assert (merged.start_rad, merged.end_rad) == pytest.approx((2.0, 4.0))


def test_absorb_extends_start_edge():
    # receiver wedge sits "after" the other: its start edge is shared.
    receiver = SectorWedge(3.0, 4.0)
    merged = receiver.absorb(SectorWedge(2.0, 3.0))
    assert (merged.start_rad, merged.end_rad) == pytest.approx((2.0, 4.0))


def test_absorb_returns_new_wedge_receiver_unchanged():
    receiver = SectorWedge(2.0, 3.0)
    merged = receiver.absorb(SectorWedge(3.0, 4.0))
    assert merged is not receiver
    assert (receiver.start_rad, receiver.end_rad) == (2.0, 3.0)


def test_absorbed_wedge_contains_both_inputs():
    receiver = SectorWedge(2.0, 3.0)
    other = SectorWedge(3.0, 4.0)
    merged = receiver.absorb(other)
    assert merged.contains(2.5)   # from receiver
    assert merged.contains(3.5)   # from other
