"""Unit coverage for the lifted ReadinessPolicy.

The policy is pure Python: these tests verify the readiness rule
without rclpy.init(): the message-count floor, the freshness
window, and the default policy values.
"""

from drone_rescue_coordination.lib.readiness import (
    DroneReadinessState, ReadinessPolicy,
)


def test_drone_below_count_floor_is_not_ready():
    p = ReadinessPolicy(min_odom_count=10, odom_timeout=2.0)
    d = DroneReadinessState(odom_count=5, last_odom_time=1.0)
    assert p.is_drone_ready(d, current_time=1.5) is False


def test_drone_at_count_floor_with_fresh_odom_is_ready():
    p = ReadinessPolicy(min_odom_count=10, odom_timeout=2.0)
    d = DroneReadinessState(odom_count=10, last_odom_time=1.0)
    assert p.is_drone_ready(d, current_time=1.5) is True


def test_drone_above_count_floor_but_stale_odom_is_not_ready():
    p = ReadinessPolicy(min_odom_count=10, odom_timeout=2.0)
    d = DroneReadinessState(odom_count=100, last_odom_time=1.0)
    # current_time - last_odom_time = 5.0 > odom_timeout(2.0)
    assert p.is_drone_ready(d, current_time=6.0) is False


def test_drone_with_never_seen_odom_passes_freshness_window():
    """If last_odom_time is 0.0 (never updated), the freshness
    window check is skipped: only the count floor matters. Matches
    pre-extraction behaviour of `if drone.last_odom_time > 0:`."""
    p = ReadinessPolicy(min_odom_count=10, odom_timeout=2.0)
    d = DroneReadinessState(odom_count=10, last_odom_time=0.0)
    assert p.is_drone_ready(d, current_time=999.0) is True


def test_policy_is_frozen():
    p = ReadinessPolicy()
    try:
        p.min_odom_count = 99
    except Exception:
        return
    assert False, 'ReadinessPolicy must be frozen'


def test_policy_defaults_match_legacy():
    p = ReadinessPolicy()
    assert p.min_odom_count == 10
    assert p.odom_timeout == 2.0
    assert p.min_ready_duration == 5.0
