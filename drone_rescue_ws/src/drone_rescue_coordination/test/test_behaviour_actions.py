"""Unit tests for lib/domain/behaviour_actions.py.

Pure-Python; no rclpy. The behaviour primitives are the future shape
of the BT actions in drone_executor.py once the cutover lands; this
suite locks in the contract today.
"""
from __future__ import annotations

import math

import pytest

from drone_rescue_coordination.lib.domain.behaviour_actions import (
    BehaviouralContext,
    BehaviouralOutput,
    DiskGeometry,
    emergency_rth,
    fly_toward,
    graceful_down_hold,
    hold_position,
    land,
    orbit_at,
    _clip_to_disk,
    _deflect_for_obstacle,
    _reactive_altitude,
)
from drone_rescue_coordination.lib.domain.value_objects import Position


GEO = DiskGeometry(
    center_x=0.0, center_y=0.0, radius=85.0,
    survey_altitude=25.0, escape_altitude=40.0,
    position_tolerance_m=1.5,
)


def test_fly_toward_no_pose_returns_target():
    ctx = BehaviouralContext()
    out = fly_toward(ctx, Position(20.0, 5.0, 25.0), GEO)
    assert out.target_pose is not None
    # No pose -> goes straight to the target (after disk clip / altitude logic).
    assert out.target_pose.x == 20.0
    assert out.target_pose.y == 5.0
    assert out.target_pose.z == 25.0
    assert out.land_command is False


def test_fly_toward_close_target_caps_at_target():
    """Within position_tolerance_m -> publish target directly."""
    ctx = BehaviouralContext(current_pose=Position(10.0, 5.0, 25.0))
    target = Position(10.5, 5.5, 25.0)
    out = fly_toward(ctx, target, GEO)
    assert out.target_pose.x == 10.5
    assert out.target_pose.y == 5.5


def test_fly_toward_uses_lookahead_step():
    """Far target -> step ahead by survey_speed × tick_period × 4."""
    ctx = BehaviouralContext(current_pose=Position(0.0, 0.0, 25.0))
    target = Position(100.0, 0.0, 25.0)
    out = fly_toward(ctx, target, GEO, survey_speed=2.5, tick_rate_hz=5.0)
    # 2.5 * (1/5) * 4 = 2.0 m lookahead step.
    assert math.isclose(out.target_pose.x, 2.0, abs_tol=0.01)
    assert math.isclose(out.target_pose.y, 0.0, abs_tol=0.01)


def test_clip_to_disk_inside_returns_unchanged():
    x, y = _clip_to_disk(10.0, 5.0, GEO)
    assert (x, y) == (10.0, 5.0)


def test_clip_to_disk_outside_pulls_to_boundary():
    x, y = _clip_to_disk(200.0, 0.0, GEO)
    assert math.isclose(math.hypot(x, y), 85.0, abs_tol=0.01)


def test_reactive_altitude_zone_warn_climbs():
    ctx = BehaviouralContext(zone_warn=True)
    z = _reactive_altitude(25.0, ctx, GEO)
    assert z == 40.0   # escape altitude wins


def test_reactive_altitude_lidar_danger_climbs():
    ctx = BehaviouralContext(lidar_danger=True)
    z = _reactive_altitude(25.0, ctx, GEO)
    assert z == 40.0


def test_reactive_altitude_no_warning_keeps_base():
    ctx = BehaviouralContext()
    z = _reactive_altitude(25.0, ctx, GEO)
    assert z == 25.0


def test_hold_position_emits_current_xy():
    ctx = BehaviouralContext(current_pose=Position(15.0, 10.0, 25.0))
    out = hold_position(ctx, GEO)
    assert out.target_pose is not None
    assert out.target_pose.x == 15.0
    assert out.target_pose.y == 10.0


def test_hold_position_no_pose_no_output():
    ctx = BehaviouralContext()
    out = hold_position(ctx, GEO)
    assert out.target_pose is None


def test_orbit_at_phase_zero_target_plus_radius():
    ctx = BehaviouralContext(current_pose=Position(0.0, 0.0, 25.0))
    out = orbit_at(ctx, Position(50.0, 50.0, 25.0), GEO,
                   phase_rad=0.0, radius_m=4.0)
    # Phase 0 -> target.x + 4, target.y
    assert math.isclose(out.target_pose.x, 54.0, abs_tol=0.01)
    assert math.isclose(out.target_pose.y, 50.0, abs_tol=0.01)


def test_orbit_at_phase_quarter_target_plus_y_radius():
    ctx = BehaviouralContext(current_pose=Position(0.0, 0.0, 25.0))
    out = orbit_at(ctx, Position(50.0, 50.0, 25.0), GEO,
                   phase_rad=math.pi / 2, radius_m=4.0)
    # Phase π/2 -> target.x, target.y + 4
    assert math.isclose(out.target_pose.x, 50.0, abs_tol=0.01)
    assert math.isclose(out.target_pose.y, 54.0, abs_tol=0.01)


def test_emergency_rth_targets_origin():
    ctx = BehaviouralContext(current_pose=Position(50.0, 30.0, 25.0))
    out = emergency_rth(ctx, GEO)
    assert out.target_pose is not None
    assert out.target_pose.x == 0.0
    assert out.target_pose.y == 0.0
    assert 'emergency' in out.status_detail


def test_land_sets_land_command_no_target():
    ctx = BehaviouralContext(current_pose=Position(0.0, 0.0, 1.0))
    out = land(ctx)
    assert out.land_command is True
    assert out.target_pose is None


def test_graceful_down_hold_airborne_descends_one_meter():
    ctx = BehaviouralContext(
        current_pose=Position(10.0, 5.0, 20.0),
        current_z=20.0,
        is_down=True,
    )
    out = graceful_down_hold(ctx, GEO)
    assert out.target_pose is not None
    assert out.target_pose.z == 19.0


def test_graceful_down_hold_near_ground_holds_low():
    ctx = BehaviouralContext(
        current_pose=Position(10.0, 5.0, 1.0),
        current_z=1.0,
        is_down=True,
    )
    out = graceful_down_hold(ctx, GEO)
    assert out.target_pose.z == 0.5


def test_graceful_down_hold_no_pose_silent():
    ctx = BehaviouralContext(is_down=True)
    out = graceful_down_hold(ctx, GEO)
    assert out.target_pose is None
    assert out.land_command is False


def test_lidar_forward_block_deflects_target():
    """lidar_front_blocked + pose -> perpendicular deflection ±π/2."""
    ctx = BehaviouralContext(
        current_pose=Position(10.0, 10.0, 25.0),
        lidar_front_blocked=True,
        lidar_block_bearing=0.0,    # obstacle directly ahead (+x)
    )
    out = fly_toward(ctx, Position(50.0, 10.0, 25.0), GEO)
    # Deflection bearing = π/2 -> deflect by +y. Target should be at
    # current.x + 0, current.y + 6 (the 6.0 m deflection in
    # _deflect_for_obstacle), before disk-clip and lookahead.
    # The step then pushes that 2 m further.
    assert out.target_pose is not None
    # y should be substantially > 10 (deflected up)
    assert out.target_pose.y > 12.0


def test_lidar_deflection_rotates_body_bearing_into_world_by_yaw():
    """The LiDAR block bearing is body-frame; the deflected setpoint is
    world-frame. A crab-walking drone holds an arbitrary fixed yaw, so the
    bearing must be rotated by current_yaw before projecting, otherwise the
    target is shoved in a yaw-offset (wrong) world direction (the docker
    "jittery sideways shove"). Regression for that frame bug.
    """
    # Obstacle dead-ahead in BODY frame (+x). Drone yawed +90° -> the obstacle
    # is actually in world +y, so the perpendicular avoidance is world -x.
    ctx = BehaviouralContext(
        current_pose=Position(0.0, 0.0, 25.0),
        lidar_front_blocked=True,
        lidar_block_bearing=0.0,
        current_yaw=math.pi / 2.0,
    )
    x, y = _deflect_for_obstacle(0.0, 0.0, ctx)
    assert x == pytest.approx(-6.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)

    # yaw 0 -> reduces exactly to the original body==world behaviour (+y).
    ctx0 = BehaviouralContext(
        current_pose=Position(0.0, 0.0, 25.0),
        lidar_front_blocked=True,
        lidar_block_bearing=0.0,
        current_yaw=0.0,
    )
    assert _deflect_for_obstacle(0.0, 0.0, ctx0) == pytest.approx((0.0, 6.0), abs=1e-6)


def test_behavioural_output_default_empty():
    out = BehaviouralOutput()
    assert out.target_pose is None
    assert out.land_command is False
    assert out.status_detail == ''


def test_behavioural_context_is_frozen():
    ctx = BehaviouralContext()
    with pytest.raises(Exception):   # FrozenInstanceError subclass of Exception
        ctx.current_z = 5.0          # type: ignore[misc]


def test_disk_geometry_is_frozen():
    g = DiskGeometry()
    with pytest.raises(Exception):
        g.radius = 100.0             # type: ignore[misc]
