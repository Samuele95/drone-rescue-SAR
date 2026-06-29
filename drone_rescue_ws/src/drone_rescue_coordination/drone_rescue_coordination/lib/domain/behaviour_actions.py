"""L1 behavioural primitives: pure functions, 3T Layer 1.

The slides (Marcelletti, pp. 37, 90, 94-96) describe behaviours as
condition->action rules taking sensor stimuli and producing actuator
commands. This module implements that shape as pure Python functions
returning typed ``BehaviouralOutput`` VOs: no ROS message types, no
ExecCtx mutation. The ``act_*`` functions in ``drone_executor.py`` are
intended to migrate onto these primitives.

The legacy BT runtime continues to use the ExecCtx-mutating ``act_*``
functions; this module sits alongside them as the target shape and is
unit-testable in isolation.

L1/L2 split: ``BehaviouralContext`` carries only sensor-derived state
(pose, lidar, battery, zone-warn, the disk/escape parameters).
Plan-progress state (``investigate_angles``, ``orbit_phase``,
``scan_index``) is L2 saga state and belongs in ``VictimSubMission``
or on a per-task L2 cursor.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .value_objects import Position


@dataclass(frozen=True)
class BehaviouralContext:
    """L1 sensor-derived context: slides p. 90 inputs.

    Contains *only* fields a behaviour-based layer reads (slides
    "stimuli"). No plan-progress state. The L2 executive (BT or
    BehaviouralLayer adapter) is responsible for advancing per-task
    cursors and reading them back into the BT logic.

    Field correspondence with the legacy mutable ``ExecCtx`` (in
    ``drone_executor.py``):

    ====================  ===================================
    BehaviouralContext    ExecCtx (legacy)
    ====================  ===================================
    current_pose          current_pose
    current_z             current_z
    lidar_min_range       lidar_min_range
    lidar_front_blocked   lidar_front_blocked
    lidar_block_bearing   lidar_block_bearing
    lidar_danger          lidar_danger
    battery_low           battery_low
    battery_level         battery_level
    zone_warn             zone_warn
    is_down               is_down
    now_sec               now_sec
    ====================  ===================================

    Parameters carried separately as ``DiskGeometry`` to keep the
    context narrowly stimuli + state.
    """
    current_pose: Optional[Position] = None
    current_z: float = 0.0
    # World-frame heading of the airframe (rad, 0 = +x, ccw), from odom.
    # The LiDAR scan bearings are body-frame; the drones hold an arbitrary
    # fixed yaw (no yaw-toward-travel control), so a body-frame bearing must
    # be rotated by this to recover a world-frame avoidance direction.
    current_yaw: float = 0.0
    lidar_min_range: float = float('inf')
    lidar_front_blocked: bool = False
    lidar_block_bearing: float = 0.0
    lidar_danger: bool = False
    battery_low: bool = False
    battery_level: float = 1.0
    zone_warn: bool = False
    is_down: bool = False
    down_reason: str = ''
    now_sec: float = 0.0


@dataclass(frozen=True)
class DiskGeometry:
    """Mission-disk parameters: the boundary clip used by L1 actions.

    Slides p. 100 (basis behaviors) treat parameters as separate from
    the per-tick stimulus context. The disk geometry rarely changes
    (it is mission-level configuration), so it is passed alongside
    the per-tick ``BehaviouralContext`` rather than embedded in it.
    """
    center_x: float = 0.0
    center_y: float = 0.0
    radius: float = 85.0
    survey_altitude: float = 25.0
    escape_altitude: float = 55.0
    position_tolerance_m: float = 1.5


@dataclass(frozen=True)
class BehaviouralOutput:
    """L1 output: the slides' "action" in stimulus->action.

    Decoupled from any ROS message type. The L2 publish edge
    (``DroneExecutor._publish_outputs``) translates ``target_pose``
    to ``geometry_msgs.msg.PoseStamped`` and ``land_command`` to
    ``std_msgs.msg.Bool`` once at the LifecycleNode boundary,
    mirroring the anti-corruption translator pattern.

    A single L1 tick produces at most one positional command
    (``target_pose``), optionally with a ``land_command`` set, plus
    a status detail string for diagnostics.

    ``task_completed`` is the one task-lifecycle
    signal a behaviour leaf raises (the BT actions only ever report
    COMPLETED; FAILURE is carried by the ``bt.Status`` return, and the
    deliberative layer handles timeouts). When True the L2 edge maps it
    to a ``TaskStatus.COMPLETED`` (with ``status_detail`` as the detail,
    and the task id supplied by the executor). This keeps the BT output
    a single pure-domain value so ``ExecCtx`` carries no ROS-typed
    output slots.
    """
    target_pose: Optional[Position] = None
    land_command: bool = False
    status_detail: str = ''
    task_completed: bool = False


# ---------------------------------------------------------------- helpers
def _clip_to_disk(
    x: float, y: float, geometry: DiskGeometry,
) -> tuple[float, float]:
    """Hard-clip a target XY to the mission disk boundary."""
    dx = x - geometry.center_x
    dy = y - geometry.center_y
    r = math.hypot(dx, dy)
    if r <= geometry.radius:
        return x, y
    scale = geometry.radius / r
    return geometry.center_x + dx * scale, geometry.center_y + dy * scale


def _reactive_altitude(
    base_z: float,
    ctx: BehaviouralContext,
    geometry: DiskGeometry,
) -> float:
    """Climb to escape altitude on zone-warn or LiDAR-danger.

    Hysteretic on the LiDAR side: ``lidar_danger`` is a separate
    field from ``lidar_min_range`` so the executive layer's
    threshold logic doesn't oscillate (see ExecCtx hysteresis).
    """
    needs_escape = ctx.zone_warn or ctx.lidar_danger
    return max(base_z, geometry.escape_altitude) if needs_escape else base_z


def _deflect_for_obstacle(
    x: float, y: float, ctx: BehaviouralContext,
) -> tuple[float, float]:
    """Lateral deflection on LiDAR forward-cone obstacle.

    When ``lidar_front_blocked`` fires, project a perpendicular
    avoidance vector off the obstacle bearing. Returns the deflected
    XY; pose-less ctx returns the input unchanged.

    ``lidar_block_bearing`` is a *body-frame* bearing (0 = airframe +x).
    The setpoint we deflect is *world-frame*, and the drones hold an
    arbitrary fixed yaw, so we rotate the bearing into the world frame by
    ``current_yaw`` before projecting. (Reduces to the original body==world
    formula exactly at yaw 0.) Omitting this rotation deflected the target
    in a yaw-offset direction, the "jittery sideways shove" near obstacles.
    """
    if not ctx.lidar_front_blocked or ctx.current_pose is None:
        return x, y
    world_bearing = ctx.lidar_block_bearing + ctx.current_yaw
    deflect = world_bearing + math.pi / 2.0
    dx = 6.0 * math.cos(deflect)
    dy = 6.0 * math.sin(deflect)
    return ctx.current_pose.x + dx, ctx.current_pose.y + dy


def _build_target(
    x: float,
    y: float,
    z: Optional[float],
    ctx: BehaviouralContext,
    geometry: DiskGeometry,
) -> Position:
    """The pure-domain analogue of ``drone_executor._make_target``.

    Disk clip → forward-obstacle deflection → reactive altitude.
    Returns a frozen ``Position`` instead of a ``PoseStamped``.
    """
    dx, dy = _deflect_for_obstacle(x, y, ctx)
    cx, cy = _clip_to_disk(dx, dy, geometry)
    base_z = z if z is not None else geometry.survey_altitude
    target_z = _reactive_altitude(base_z, ctx, geometry)
    return Position(cx, cy, target_z)


# ---------------------------------------------------------------- primitives
def fly_toward(
    ctx: BehaviouralContext,
    target: Position,
    geometry: DiskGeometry,
    *,
    survey_speed: float = 2.5,
    tick_rate_hz: float = 5.0,
) -> BehaviouralOutput:
    """Step the setpoint toward ``target`` at ``survey_speed``.

    Drops a setpoint at most a couple-of-seconds-out (×4 lookahead)
    so the L1 PID doesn't dither. If close enough (within
    ``position_tolerance_m``), publishes ``target`` directly.
    Mirrors the lookahead logic from ``act_scan_waypoints`` /
    ``act_investigate`` in ``drone_executor.py``, but as a pure
    function returning a ``BehaviouralOutput``.
    """
    if ctx.current_pose is None:
        pose = _build_target(target.x, target.y, target.z, ctx, geometry)
        return BehaviouralOutput(target_pose=pose)

    dx = target.x - ctx.current_pose.x
    dy = target.y - ctx.current_pose.y
    dist = math.hypot(dx, dy)
    step_m = survey_speed * (1.0 / max(tick_rate_hz, 1e-3)) * 4.0
    if dist <= geometry.position_tolerance_m or dist <= step_m:
        sx, sy = target.x, target.y
    else:
        sx = ctx.current_pose.x + (dx / dist) * step_m
        sy = ctx.current_pose.y + (dy / dist) * step_m

    pose = _build_target(sx, sy, target.z, ctx, geometry)
    return BehaviouralOutput(target_pose=pose)


def hold_position(
    ctx: BehaviouralContext,
    geometry: DiskGeometry,
) -> BehaviouralOutput:
    """Idle hold at current XY, ``survey_altitude`` Z."""
    if ctx.current_pose is None:
        return BehaviouralOutput()
    pose = _build_target(
        ctx.current_pose.x, ctx.current_pose.y, None, ctx, geometry,
    )
    return BehaviouralOutput(target_pose=pose)


def orbit_at(
    ctx: BehaviouralContext,
    target_xy: Position,
    geometry: DiskGeometry,
    *,
    phase_rad: float,
    radius_m: float = 4.0,
) -> BehaviouralOutput:
    """One orbit step; caller advances ``phase_rad``.

    ``phase_rad`` is an L2 cursor: it is *passed in*, not stored
    on the context. The L2 executive owns phase tracking and decides
    when the orbit completes; this L1 primitive just emits the
    setpoint for the requested phase.
    """
    cx = target_xy.x + radius_m * math.cos(phase_rad)
    cy = target_xy.y + radius_m * math.sin(phase_rad)
    pose = _build_target(cx, cy, target_xy.z, ctx, geometry)
    return BehaviouralOutput(target_pose=pose)


def emergency_rth(
    ctx: BehaviouralContext,
    geometry: DiskGeometry,
) -> BehaviouralOutput:
    """Fly to home (0, 0) at survey altitude. L2 will issue LAND on
    arrival via ``land()``."""
    pose = _build_target(0.0, 0.0, None, ctx, geometry)
    return BehaviouralOutput(target_pose=pose, status_detail='emergency-rth')


def land(ctx: BehaviouralContext) -> BehaviouralOutput:
    """Hard land command; bypasses positional control."""
    return BehaviouralOutput(land_command=True, status_detail='land')


def graceful_down_hold(
    ctx: BehaviouralContext,
    geometry: DiskGeometry,
) -> BehaviouralOutput:
    """Hard-fail descent; drone is ``is_down``.

    Step the Z setpoint down 1 m/tick while airborne; once near the
    ground, hold position at 0.5 m so the drone is visible to a human
    operator. Mirrors ``act_down_hold`` in ``drone_executor.py``.
    """
    if ctx.current_pose is None:
        return BehaviouralOutput()
    if ctx.current_z > 1.5:
        target_z = max(0.5, ctx.current_z - 1.0)
    else:
        target_z = 0.5
    pose = _build_target(
        ctx.current_pose.x, ctx.current_pose.y, target_z, ctx, geometry,
    )
    return BehaviouralOutput(target_pose=pose)
