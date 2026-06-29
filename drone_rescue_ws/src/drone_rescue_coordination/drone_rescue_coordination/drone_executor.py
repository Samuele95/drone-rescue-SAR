"""Drone Executor: per-drone behavior-tree task runner.

3T Architecture: Behavioural Layer (L1), BT task runner with documented
L2 leakage (Marcelletti slides pp. 37, 42-44).

L1-hosted BT that executes per-drone behaviour. The top-level
``Switch(task_type)`` is technically L2 (executive translation), with
the L1 leaves being the ``act_*`` reactive primitives. See the
``BehaviouralLayer`` Protocol and ``lib/domain/behaviour_actions.py``
for the target shape post-cutover.

Replaces surveyor.py. Receives TaskAssignment from mission_manager, drives the
existing drone_controller via /<drone>/survey_target waypoints (or /<drone>/land
for emergencies), and publishes TaskStatus feedback.

The behavior tree is hand-rolled (lib/bt.py); no py_trees / py_trees_ros dep.

Tree shape:

    root: Selector
    ├── EmergencyTree:  Sequence(InEmergency? → IssueRTH)
    └── TaskTree:       Switch(task_type)
                          SCAN_WAYPOINTS → WaypointSequencer
                          INVESTIGATE    → FlyAndHover
                          CONFIRM        → OrbitAndHover
                          RTH            → FlyAndLand
                          LAND           → IssueLand
                          IDLE           → HoldPosition

Per-drone Lifecycle node, managed by lifecycle_manager. Activates after the
controller is up so it can immediately listen for tasks.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from drone_rescue_coordination.lib.domain.fleet import default_drone_names_list
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import diagnostic_updater
import diagnostic_msgs.msg

from std_msgs.msg import Bool, Float32, Header   # Float32 was previously deferred-imported inside on_configure
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from drone_rescue_msgs.msg import (
    TaskAssignment, TaskStatus, DronePeerState, DroneHealth, MissionEvent,
)

from drone_rescue_coordination.lib import bt
from drone_rescue_coordination.lib.domain.executor import (
    Executor as _ExecutorImpl,
    ExecutorOutputs,
    ExecutorSensors,
)
# L1 geometry primitives. `_make_target` delegates its disk-clip /
# lidar-deflect / reactive-altitude math to these tested pure helpers
# (the single source of the motor-schema geometry), instead of
# duplicating it inline.
from drone_rescue_coordination.lib.domain import behaviour_actions as _ba
from drone_rescue_coordination.lib.domain.value_objects import (
    DEFAULT_INVESTIGATE_ANGLES, Position,
)
from drone_rescue_coordination.lib.ros_adapter.translators import (
    point_from_position, position_from_point,
)
from drone_rescue_coordination.lib.composition import (
    bind_composition, resolve_clock,
)


_TASK_TYPE_NAMES = {
    TaskAssignment.SCAN_WAYPOINTS: 'SCAN_WAYPOINTS',
    TaskAssignment.INVESTIGATE: 'INVESTIGATE',
    TaskAssignment.CONFIRM: 'CONFIRM',
    TaskAssignment.RTH: 'RTH',
    TaskAssignment.LAND: 'LAND',
    TaskAssignment.IDLE: 'IDLE',
}


@dataclass
class BTCursor:
    """Per-task L2 plan-progress cursor.

    Groups the mutable per-task progress state that the BT actions
    advance over the life of a single task, distinct from the L1
    sensor-derived stimulus fields on ``BehaviouralContextMutable``. Reset wholesale on
    each new task dispatch via ``reset_for_new_task()``. Per the slides'
    3T taxonomy this is executive-layer (L2) state; the
    investigate-specific fields are slated to move onto the per-victim
    ``VictimSubMission``.
    """
    scan_index: int = 0
    orbit_phase: float = 0.0           # 0..2pi, CONFIRM orbit
    investigate_done_at: float = 0.0   # ROS time
    # Multi-view INVESTIGATE: cardinal angles still to visit + per-angle
    # hover clock.
    investigate_angles: list = field(default_factory=list)
    investigate_dwell_until: float = 0.0

    def reset_for_new_task(self) -> None:
        """Reset per-task progress so cursors restart on a new dispatch."""
        self.scan_index = 0
        self.orbit_phase = 0.0
        self.investigate_done_at = 0.0
        self.investigate_angles = []
        self.investigate_dwell_until = 0.0


@dataclass
class BehaviouralContextMutable:
    """Mutable BT execution context: 3T Layer 1 behavioural context
    (with documented L2 leakage).

    Labelled per the slides' 3T taxonomy (Marcelletti, pp. 33, 37, 90).
    The fields below are partitioned into L1 (sensor-derived stimuli,
    the slides' "Behaviors - Inputs", p. 90) and L2 (saga /
    plan-progress state, the slides' "Executive Layer", p. 38).

    Target shape: see ``lib.domain.behaviour_actions.BehaviouralContext``
    for the frozen, sensor-only L1 context the BT actions will consume
    after the cutover. Until then this mutable dataclass remains; the
    L2-leakage fields below are marked for migration to
    ``VictimSubMission`` (per-task saga aggregate) or to per-task L2
    cursors once the saga lift lands.

    ``current_pose`` is the domain ``Position`` VO rather than
    ``geometry_msgs.msg.Point``. Field access (``.x/.y/.z``) is
    duck-compatible so the act_* functions work unchanged; the
    ROS-message boundary is translated at the odom callback and the
    peer-state publish.
    """
    # L1: identity (always present; not strictly "stimulus")
    drone_name: str

    # L1: sensor-derived stimuli (slides p. 90 "Behaviors - Inputs").
    # Migrates to BehaviouralContext (lib/domain/behaviour_actions.py).
    current_pose: Optional[Position] = None
    current_z: float = 0.0
    # World-frame airframe heading (rad) from odom; needed to rotate the
    # body-frame LiDAR block bearing into the world frame for deflection.
    current_yaw: float = 0.0

    # L1: disk geometry (mission parameters, slides p. 100 "basis
    # behavior parameters"; passed alongside stimulus context).
    survey_altitude: float = 10.0
    survey_speed: float = 2.5
    tick_rate_hz: float = 5.0
    position_tolerance_m: float = 1.5
    # Hard clip on every published setpoint, last line of defence against PID
    # overshoot when the drone is asked to fly to a waypoint at the boundary.
    mission_center_x: float = 0.0
    mission_center_y: float = 0.0
    mission_radius: float = 85.0
    # Reactive obstacle avoidance: when zone_warn fires (drone in or near a
    # no-fly zone, which in this scenario contains buildings/structures), the
    # executor publishes setpoints at this higher altitude until it clears the
    # zone. Acts as a software analog of a downward-facing rangefinder.
    escape_altitude_m: float = 55.0

    # L1: latched flags from sensor pipelines (still stimuli).
    # Latched battery_low (must persist, battery doesn't recover) and
    # transient zone_warning (resets each callback). in_emergency is the OR.
    battery_low: bool = False
    zone_warn: bool = False

    # LiDAR-driven reactive obstacle avoidance (added because the YAML
    # no-fly zones don't cover the world's organic obstacles like trees,
    # walls, debris piles). `lidar_min_range` is the smallest distance
    # returned by the most recent scan; `lidar_danger` is hysteresis
    # over `lidar_min_range`: true once the drone gets close to an
    # obstacle, only resets when there's plenty of clearance again.
    # `lidar_front_blocked` indicates an obstacle in the *direction of
    # travel* specifically, used to deflect the lateral target, not just
    # climb.
    lidar_min_range: float = 1e9
    lidar_danger: bool = False
    lidar_front_blocked: bool = False
    # Heading of the closest forward-cone obstacle (radians, 0=+x, ccw),
    # so the executor can pick a perpendicular deflection direction.
    lidar_block_bearing: float = 0.0
    # Hysteresis thresholds: set danger when below `enter_m`, clear it
    # only when ALL of `clear_m` have passed (avoids oscillation between
    # climb and descend at the threshold).
    lidar_danger_enter_m: float = 5.0
    lidar_danger_clear_m: float = 8.0

    # Continuous battery 0..1 from /<drone>/battery_level, broadcast to peers
    # for the dashboard and used as a soft input to the auction by sibling
    # awareness logic.
    battery_level: float = 1.0

    # L1/L2 boundary: anomaly flags (sensor-detected, but their
    # interpretation drives executive-layer recovery dispatch).
    # Hard-failure flag from drone_health_monitor. When True the executor
    # stops all task work and parks the drone in a hold pose; siblings see it
    # via peer_state and the mission_manager reassigns its sector.
    is_down: bool = False
    down_reason: str = ''

    # L2 LEAKAGE: plan-progress / saga state.
    # Slides p. 38 places these in the Executive Layer, not L1.
    # Migration target: VictimSubMission (per-task saga aggregate)
    # owned by the Mission aggregate post saga lift. Until that lands,
    # this state lives here because the BT actions need to read it via
    # ctx.* and the saga aggregate has no presence on the LifecycleNode
    # runtime path.
    current_task: Optional[TaskAssignment] = None
    last_task_id: int = 0
    # The per-task progress cursor (scan_index / orbit_phase /
    # investigate angle queue + dwell clock) is grouped into ``BTCursor``
    # and reset wholesale on each new task. The BT actions read it via
    # ``ctx.cursor.*``. ``investigate_radius_m`` / ``investigate_dwell_s``
    # below stay flat: they are CONFIG (set once), not per-task progress.
    cursor: BTCursor = field(default_factory=BTCursor)
    # act_investigate reads these per tick; config, not cursor state.
    investigate_radius_m: float = 5.0
    investigate_dwell_s: float = 2.0

    # BehaviouralContextMutable is pure INPUT. The BT actions do not
    # write output slots here; each ``act_*`` RETURNS a
    # ``BehaviouralOutput`` (target / land / completion), threaded up
    # through ``lib.bt`` and translated to ROS once at the publish edge
    # (``_publish_outputs``).
    now_sec: float = 0.0


# The per-tick L1 state ``BehaviouralContextMutable`` is the mutable
# counterpart of the frozen
# ``lib.domain.behaviour_actions.BehaviouralContext`` (input view).
# ``ExecCtx`` stays as a one-release deprecated alias so any out-of-tree
# reference keeps importing.
ExecCtx = BehaviouralContextMutable   # deprecated alias, remove next cycle


# ----------------------------------------------------------------- BT actions
def _behavioural_view(ctx: BehaviouralContextMutable) -> '_ba.BehaviouralContext':
    """Project the mutable BehaviouralContextMutable into the frozen L1
    ``BehaviouralContext`` the pure geometry helpers read (sensor-derived
    stimulus fields only)."""
    return _ba.BehaviouralContext(
        current_pose=ctx.current_pose,
        current_z=ctx.current_z,
        current_yaw=ctx.current_yaw,
        lidar_min_range=ctx.lidar_min_range,
        lidar_front_blocked=ctx.lidar_front_blocked,
        lidar_block_bearing=ctx.lidar_block_bearing,
        lidar_danger=ctx.lidar_danger,
        battery_low=ctx.battery_low,
        battery_level=ctx.battery_level,
        zone_warn=ctx.zone_warn,
        is_down=ctx.is_down,
        down_reason=ctx.down_reason,
        now_sec=ctx.now_sec,
    )


def _disk_geometry(ctx: BehaviouralContextMutable) -> '_ba.DiskGeometry':
    """Return the mission-disk geometry the helpers clip and climb against."""
    return _ba.DiskGeometry(
        center_x=ctx.mission_center_x,
        center_y=ctx.mission_center_y,
        radius=ctx.mission_radius,
        survey_altitude=ctx.survey_altitude,
        escape_altitude=ctx.escape_altitude_m,
        position_tolerance_m=ctx.position_tolerance_m,
    )


def _make_target(
    ctx: BehaviouralContextMutable, x: float, y: float,
    z: Optional[float] = None,
) -> Position:
    """Apply the tested pure geometry helpers in
    ``lib.domain.behaviour_actions``. Returns a pure-domain ``Position``
    (the L1 setpoint); the ROS ``PoseStamped`` is built once at the
    publish edge (``_publish_outputs``), so the BT actions and
    ``BehaviouralContextMutable`` stay free of ROS message types.

    Behaviour-preserving: the helpers are invoked in this function's
    historical order (clip-to-disk, then forward-obstacle deflection,
    then reactive altitude), which is NOT the order in
    ``behaviour_actions._build_target`` (deflect then clip). The legacy
    order leaves a LiDAR-deflected point un-re-clipped (the deflection
    overrides the clip), so we replicate it exactly by calling the
    helpers directly rather than ``_build_target``. The disk-clip,
    deflection and reactive-altitude maths are single-sourced and
    unit-tested in ``test_behaviour_actions.py``.
    """
    view = _behavioural_view(ctx)
    geo = _disk_geometry(ctx)
    cx, cy = _ba._clip_to_disk(x, y, geo)
    cx, cy = _ba._deflect_for_obstacle(cx, cy, view)
    base_z = z if z is not None else ctx.survey_altitude
    target_z = _ba._reactive_altitude(base_z, view, geo)
    return Position(float(cx), float(cy), float(target_z))


# ---- DOWN (hard fail) ----------------------------------------------------
def cond_is_down(ctx: BehaviouralContextMutable) -> bool:
    """drone_health_monitor said unrecoverable=True. Stop everything; controlled
    descend / hold until a human picks the drone up. mission_manager will
    have reassigned this drone's sector to its peers."""
    return ctx.is_down


def act_down_hold(ctx: BehaviouralContextMutable) -> bt.TickResult:
    if ctx.current_pose is None:
        return bt.Status.RUNNING, None
    # Descend gracefully if airborne; otherwise hold position. Always RUNNING
    # (never SUCCESS) so the Selector keeps choosing this branch and the
    # task_tree below never executes.
    if ctx.current_z > 1.5:
        # Step the z setpoint down 1 m / tick at our tick rate. Reuses
        # _make_target to also benefit from disk clip + reactive zone climb.
        target_z = max(0.5, ctx.current_z - 1.0)
        tgt = _make_target(
            ctx, ctx.current_pose.x, ctx.current_pose.y, z=target_z,
        )
    else:
        # On the ground: hold current XY at low altitude (no LAND command,
        # to leave the drone visible in place for the human, not release the
        # safety lock).
        tgt = _make_target(
            ctx, ctx.current_pose.x, ctx.current_pose.y, z=0.5,
        )
    return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)


# ---- emergency
def cond_in_emergency(ctx: BehaviouralContextMutable) -> bool:
    # Only battery-low triggers emergency RTH. Zone warnings used to trigger
    # this too, but they fire on transient buffer brushes during normal scan
    # and would suspend the mission for no real benefit; the boundary clip
    # in _make_target already keeps drones inside the mission disk, and the
    # no-fly zones in this scenario are interior obstacles the drones can fly
    # over (cameras still see victims).
    return ctx.battery_low


def act_emergency_rth(ctx: BehaviouralContextMutable) -> bt.TickResult:
    if ctx.current_pose is None:
        return bt.Status.RUNNING, None
    # Send to (0, 0) at survey_altitude. mission_manager will issue LAND when
    # the drone is over its safe pad.
    tgt = _make_target(ctx, 0.0, 0.0)
    # keep flying RTH every tick until task changes
    return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)


# ---- SCAN_WAYPOINTS
# Publish a *stepped* setpoint, advancing only `survey_speed * tick_period` m
# along the line from current pose to next waypoint each tick. Same trick the
# original surveyor used to keep the controller's PID happy: it never sees a
# setpoint more than ~2-3 m away, so I-term doesn't wind up and the drone
# tracks the waypoint smoothly instead of overshooting.
def act_scan_waypoints(ctx: BehaviouralContextMutable) -> bt.TickResult:
    t = ctx.current_task
    if t is None or not t.waypoints:
        return bt.Status.SUCCESS, None
    if ctx.cursor.scan_index >= len(t.waypoints):
        return bt.Status.SUCCESS, _ba.BehaviouralOutput(
            task_completed=True, status_detail='all waypoints reached',
        )
    target = t.waypoints[ctx.cursor.scan_index]
    if ctx.current_pose is None:
        tgt = _make_target(ctx, target.x, target.y, target.z or ctx.survey_altitude)
        return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)

    # Step toward the active waypoint at survey_speed * tick_period, capped at
    # the actual distance to the waypoint.
    dx = target.x - ctx.current_pose.x
    dy = target.y - ctx.current_pose.y
    dist = math.hypot(dx, dy)
    step_m = ctx.survey_speed * (1.0 / max(ctx.tick_rate_hz, 1e-3)) * 4.0
    # x4 lookahead so PID gets a setpoint a couple of seconds out, not literally
    # the next dt; avoids dithering when drone's velocity briefly exceeds the
    # tick increment.
    if dist <= ctx.position_tolerance_m:
        # Reached: advance cursor; publish current waypoint as target this tick.
        ctx.cursor.scan_index += 1
        tgt = _make_target(ctx, target.x, target.y, target.z or ctx.survey_altitude)
        return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)
    if dist > step_m:
        sx = ctx.current_pose.x + (dx / dist) * step_m
        sy = ctx.current_pose.y + (dy / dist) * step_m
    else:
        sx, sy = target.x, target.y
    tgt = _make_target(ctx, sx, sy, target.z or ctx.survey_altitude)
    return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)


# ---- INVESTIGATE  (multi-view: orbit candidate at 4 cardinal angles)
def act_investigate(ctx: BehaviouralContextMutable) -> bt.TickResult:
    """Fly the multi-view orbit the deliberative layer committed for this
    candidate: visit each angle the L3 planner stamped on the task
    (``investigate_angles``, defaulting to the 4 cardinals) at the planned
    ``investigate_radius``, dwelling ``dwell_s`` at each so victim_detector
    gets a fresh frame from each angle. Completes after the last dwell.
    (Radius / dwell / angle-set flow from the planner; the executor's own
    params are the fallback for legacy / replayed tasks.)

    The distinct viewpoints are what the detection_filter's DBSCAN +
    Bayesian fusion needs to cluster confidently: one drone hovering at one
    spot can produce only marginally varied frames; orbiting yields genuinely
    independent observations from each angle.
    """
    t = ctx.current_task
    if t is None:
        return bt.Status.FAILURE, None
    target = t.target_point

    # The multi-view orbit is the plan the deliberative layer (L3)
    # committed for this candidate, carried on the task. Prefer it; fall
    # back to this executor's own config when the wire fields are absent
    # (legacy publishers / replayed bags without the new TaskAssignment
    # fields). 0.0 / empty are the "unset" sentinels.
    radius = t.investigate_radius if t.investigate_radius > 0.0 else ctx.investigate_radius_m
    dwell_s = t.dwell_s if t.dwell_s > 0.0 else ctx.investigate_dwell_s

    # Lazy-initialise the angle queue on first tick of this task.
    if not ctx.cursor.investigate_angles:
        ctx.cursor.investigate_angles = (
            list(t.investigate_angles) if len(t.investigate_angles)
            # 4 cardinal offsets around the candidate (legacy default).
            else list(DEFAULT_INVESTIGATE_ANGLES)
        )
        ctx.cursor.investigate_dwell_until = 0.0

    # Compute current viewpoint from first remaining angle.
    theta = ctx.cursor.investigate_angles[0]
    vx = target.x + radius * math.cos(theta)
    vy = target.y + radius * math.sin(theta)
    tgt = _make_target(ctx, vx, vy)

    if ctx.current_pose is None:
        return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)

    # Have we arrived at the current viewpoint?
    arrived = (
        (ctx.current_pose.x - vx) ** 2 + (ctx.current_pose.y - vy) ** 2
    ) <= ctx.position_tolerance_m ** 2
    if not arrived:
        ctx.cursor.investigate_dwell_until = 0.0
        return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)

    # Start dwell clock on arrival.
    if ctx.cursor.investigate_dwell_until == 0.0:
        ctx.cursor.investigate_dwell_until = ctx.now_sec + dwell_s

    if ctx.now_sec < ctx.cursor.investigate_dwell_until:
        return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)

    # Dwell complete: pop this angle, move to the next.
    ctx.cursor.investigate_angles.pop(0)
    ctx.cursor.investigate_dwell_until = 0.0
    if not ctx.cursor.investigate_angles:
        # Full multi-view orbit done.
        return bt.Status.SUCCESS, _ba.BehaviouralOutput(
            target_pose=tgt, task_completed=True,
            status_detail='multi-view investigate done',
        )
    return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)


# ---- CONFIRM  (orbit at radius)
def act_confirm(ctx: BehaviouralContextMutable) -> bt.TickResult:
    t = ctx.current_task
    if t is None:
        return bt.Status.FAILURE, None
    target = t.target_point
    # Per-task orbit radius: confirm_orbit_radius is plumbed through
    # TaskAssignment. 0.0 (or absent on a legacy message) means "use the
    # executor's default", keeping backward compatibility with bag
    # replays from before the message field was added.
    radius = float(t.confirm_orbit_radius) or 4.0
    # Phase advances ~6deg per tick at 5Hz, full orbit in ~12s
    ctx.cursor.orbit_phase += 0.10
    if ctx.cursor.orbit_phase > 2.0 * math.pi + 0.2:
        ctx.cursor.orbit_phase = 0.0
        return bt.Status.SUCCESS, _ba.BehaviouralOutput(
            task_completed=True, status_detail='orbit complete',
        )
    cx = target.x + radius * math.cos(ctx.cursor.orbit_phase)
    cy = target.y + radius * math.sin(ctx.cursor.orbit_phase)
    tgt = _make_target(ctx, cx, cy)
    return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)


# ---- RTH and LAND
def act_rth(ctx: BehaviouralContextMutable) -> bt.TickResult:
    if ctx.current_pose is None:
        return bt.Status.RUNNING, None
    # Within 3 m of the launch pad (the origin), then land.
    if ctx.current_pose.x ** 2 + ctx.current_pose.y ** 2 <= 3.0 ** 2:
        return bt.Status.SUCCESS, _ba.BehaviouralOutput(
            land_command=True, task_completed=True, status_detail='RTH+LAND',
        )
    tgt = _make_target(ctx, 0.0, 0.0)
    return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)


def act_land(ctx: BehaviouralContextMutable) -> bt.TickResult:
    return bt.Status.SUCCESS, _ba.BehaviouralOutput(
        land_command=True, task_completed=True, status_detail='land',
    )


def act_idle(ctx: BehaviouralContextMutable) -> bt.TickResult:
    if ctx.current_pose is not None:
        tgt = _make_target(ctx, ctx.current_pose.x, ctx.current_pose.y)
        return bt.Status.RUNNING, _ba.BehaviouralOutput(target_pose=tgt)
    return bt.Status.RUNNING, None


# Bridge between the typed ExecutorSensors/Outputs VOs and the
# BehaviouralContextMutable mutable state. Pure-Python; unit-testable.
def _apply_sensors_to_ctx(ctx: BehaviouralContextMutable, sensors: ExecutorSensors) -> None:
    """Project an ExecutorSensors frame into BehaviouralContextMutable: only the fields
    the BT actions read. Position-typed; rclpy-free."""
    ctx.now_sec = sensors.now_sec
    if sensors.current_pose is not None:
        ctx.current_pose = sensors.current_pose
        ctx.current_z = sensors.current_pose.z
    ctx.lidar_min_range = sensors.lidar_min_range_m
    if sensors.is_down:
        ctx.is_down = True


def _outputs_from_tick(
    ctx: BehaviouralContextMutable, status: bt.Status, output: Optional['_ba.BehaviouralOutput'],
) -> ExecutorOutputs:
    """Map the BT's per-tick result into typed ``ExecutorOutputs``.

    The publish edge consumes them. The behavioural ``output`` (target /
    land / completion) is the BT's *return value*, not a side effect on
    ``ctx``; the task id behind ``task_completed`` is resolved here from
    ``ctx.last_task_id``."""
    out = output or _ba.BehaviouralOutput()
    return ExecutorOutputs(
        target_pose=out.target_pose,
        land_command=bool(out.land_command),
        status_detail=out.status_detail,
        completed_task_id=ctx.last_task_id if out.task_completed else None,
        failed_task_id=None,
        damage_reason=ctx.down_reason if ctx.is_down else None,
    )


def _build_tree() -> bt.BTNode:
    # memory=False so safety conditions are re-checked every tick. With
    # memory=True a transient brush would latch us into RTH forever because
    # Sequence skips ahead to the running Action and never re-evaluates the
    # Condition. Reactivity beats efficiency for safety branches.
    down_tree = bt.Sequence([
        bt.Condition(cond_is_down, name='is_down?'),
        bt.Action(act_down_hold, name='down_hold'),
    ], name='down_tree', memory=False)

    emergency = bt.Sequence([
        bt.Condition(cond_in_emergency, name='in_emergency?'),
        bt.Action(act_emergency_rth, name='emergency_rth'),
    ], name='emergency_tree', memory=False)

    task_branches = {
        TaskAssignment.SCAN_WAYPOINTS: bt.Action(act_scan_waypoints, name='scan'),
        TaskAssignment.INVESTIGATE: bt.Action(act_investigate, name='investigate'),
        TaskAssignment.CONFIRM: bt.Action(act_confirm, name='confirm'),
        TaskAssignment.RTH: bt.Action(act_rth, name='rth'),
        TaskAssignment.LAND: bt.Action(act_land, name='land'),
        TaskAssignment.IDLE: bt.Action(act_idle, name='idle'),
    }
    # No None guard in key_fn: `_tick()` already returns early when
    # `current_task is None` (see drone_executor._tick), so the BT never
    # gets ticked with a null task. The `default` branch handles unknown
    # / out-of-enum task_type values (forward-compat shield), not the
    # None case.
    task_tree = bt.Switch(
        key_fn=lambda c: c.current_task.task_type,
        branches=task_branches,
        default=bt.Action(act_idle, name='idle_default'),
        name='task_tree',
    )

    # Down outranks Emergency outranks Task. A drone marked DOWN ignores
    # everything else; its sector has been reassigned by mission_manager.
    return bt.Selector([down_tree, emergency, task_tree], name='root')


# ----------------------------------------------------------------- node
class DroneExecutor(LifecycleNode):
    # composition kwarg accepted via __init__; falls back to lazy adapter
    # construction when None.
    def __init__(self, *, composition=None):
        super().__init__('drone_executor')
        self._composition = composition

        self.declare_parameter('drone_name', 'drone1')
        self.declare_parameter('tick_rate_hz', 5.0)
        self.declare_parameter('survey_altitude', 10.0)
        self.declare_parameter('survey_speed', 2.5)
        self.declare_parameter('position_tolerance_m', 1.5)
        self.declare_parameter('min_takeoff_altitude_m', 5.0)
        self.declare_parameter('mission_center_x', 0.0)
        self.declare_parameter('mission_center_y', 0.0)
        self.declare_parameter('mission_radius', 85.0)
        self.declare_parameter('escape_altitude_m', 55.0)
        # Sibling drones, used to set up peer_state subscriptions. The
        # executor strips itself out of the list on configure so the awareness
        # map only contains other drones.
        self.declare_parameter('peer_drone_names',
                               default_drone_names_list())
        self.declare_parameter('peer_broadcast_hz', 2.0)
        self.declare_parameter('investigate_dwell_s', 2.0)
        self.declare_parameter('investigate_radius_m', 5.0)

        self.drone_name = str(self.get_parameter('drone_name').value)
        self.tick_rate_hz = float(self.get_parameter('tick_rate_hz').value)
        self.survey_altitude = float(self.get_parameter('survey_altitude').value)
        self.survey_speed = float(self.get_parameter('survey_speed').value)
        self.position_tolerance_m = float(self.get_parameter('position_tolerance_m').value)
        self.min_takeoff_altitude_m = float(self.get_parameter('min_takeoff_altitude_m').value)
        self.peer_broadcast_hz = float(self.get_parameter('peer_broadcast_hz').value)
        self.investigate_dwell_s = float(self.get_parameter('investigate_dwell_s').value)
        self.investigate_radius_m = float(self.get_parameter('investigate_radius_m').value)
        self.peer_drone_names = [
            str(n) for n in self.get_parameter('peer_drone_names').value
            if str(n) != self.drone_name
        ]

        self.ctx = BehaviouralContextMutable(
            drone_name=self.drone_name,
            survey_altitude=self.survey_altitude,
            survey_speed=self.survey_speed,
            tick_rate_hz=self.tick_rate_hz,
            position_tolerance_m=self.position_tolerance_m,
            mission_center_x=float(self.get_parameter('mission_center_x').value),
            mission_center_y=float(self.get_parameter('mission_center_y').value),
            mission_radius=float(self.get_parameter('mission_radius').value),
            escape_altitude_m=float(self.get_parameter('escape_altitude_m').value),
            investigate_radius_m=self.investigate_radius_m,
            investigate_dwell_s=self.investigate_dwell_s,
        )
        self.tree = _build_tree()
        # Concrete ExecutorPort implementation. The LifecycleNode keeps
        # driving the legacy `_tick` path during
        # USE_LEGACY_DRONE_EXECUTOR=1; this Executor is the typed entry
        # point that tests + the future cutover commit consume.
        self._executor_impl = _ExecutorImpl(
            state=self.ctx,
            tree=self.tree,
            update_state_from_sensors=_apply_sensors_to_ctx,
            read_outputs_from_state=_outputs_from_tick,
        )
        self._is_active = False

        self._target_pub = None
        self._land_pub = None
        self._status_pub = None
        self._task_sub = None
        self._odom_sub = None
        self._battery_sub = None
        self._zone_sub = None
        self._tick_timer = None
        # Own peer-state broadcast (consumed by the dashboard's fleet view).
        # The sibling-subscription read side was write-only (stored in
        # self._peers, never read for any decision), so it is removed; only the
        # outbound broadcast remains. Re-introduce a peer registry here if/when
        # peer-aware avoidance is actually wired.
        self._peer_state_pub = None
        self._peer_broadcast_timer = None
        # Cache scan-bearing numpy array; populated on first scan once the
        # scanner geometry is known.
        self._scan_bearings_cache: Optional[np.ndarray] = None
        # Continuous battery + per-drone health (Stage B4).
        self._battery_level_sub = None
        self._health_sub = None
        self._mission_event_pub = None
        self._damage_reported = False

        self.get_logger().info(f'drone_executor[{self.drone_name}] initialized')

    # ------------------------------------------------------------ lifecycle
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=10,
        )

        # We publish /survey_target. The controller's `survey_target_callback`
        # accepts these in the SURVEYING state, which is exactly the state the
        # controller ends up in after the readiness_coordinator triggers the
        # standard /survey/start to TAKEOFF to SURVEYING transition. This avoids
        # needing any drone_controller changes.
        self._target_pub = self.create_publisher(
            PoseStamped, f'/{self.drone_name}/survey_target', cmd_qos,
        )
        self._land_pub = self.create_publisher(
            Bool, f'/{self.drone_name}/land', cmd_qos,
        )
        self._status_pub = self.create_publisher(
            TaskStatus, f'/{self.drone_name}/task_status', cmd_qos,
        )
        # Deposit a pheromone every tick so coverage_tracker keeps its metric
        # meaningful: it now reads "where the executors have flown" rather
        # than "where the surveyor was."
        self._deposit_pub = self.create_publisher(
            PointStamped, '/pheromone/deposit', cmd_qos,
        )

        # Match mission_manager's TRANSIENT_LOCAL durability so we can pick up
        # the latest task even if we activate (or re-spawn) after dispatch.
        task_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        self._task_sub = self.create_subscription(
            TaskAssignment, f'/{self.drone_name}/task',
            self._on_task, task_qos,
        )
        self._odom_sub = self.create_subscription(
            Odometry, f'/{self.drone_name}/odom',
            self._on_odom, sensor_qos,
        )
        self._battery_sub = self.create_subscription(
            Bool, f'/{self.drone_name}/battery_low',
            self._on_battery_low, 10,
        )
        self._zone_sub = self.create_subscription(
            Bool, f'/{self.drone_name}/zone_warning',
            self._on_zone_warning, 10,
        )
        # LiDAR-driven obstacle awareness (added because the YAML no-fly
        # zones don't cover organic obstacles like buildings outside those
        # zones). Subscribes to the bridged /<drone>/scan and translates
        # each scan into BehaviouralContextMutable.lidar_danger / lidar_front_blocked /
        # lidar_block_bearing so _make_target can climb and deflect.
        self._scan_sub = self.create_subscription(
            LaserScan, f'/{self.drone_name}/scan',
            self._on_scan,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE, depth=1),
        )
        # Continuous battery for peer broadcast + health (vs. Bool
        # battery_low). Float32 is imported at module top so missing
        # dependencies fail at node load, not at first lifecycle configure.
        self._battery_level_sub = self.create_subscription(
            Float32, f'/{self.drone_name}/battery_level',
            self._on_battery_level, 10,
        )
        # Own health stream: when unrecoverable=True the executor enters DOWN
        # and publishes a one-shot DRONE_DAMAGE_REPORT mission event.
        self._health_sub = self.create_subscription(
            DroneHealth, f'/{self.drone_name}/health',
            self._on_health, 10,
        )

        # Peer-state broadcast: own publisher (TRANSIENT_LOCAL so a late or
        # restarted subscriber gets the most recent snapshot) + one
        # subscription per sibling. Together this gives every drone +
        # supervisor a single-hop awareness map without going through
        # mission_manager.
        peer_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._peer_state_pub = self.create_publisher(
            DronePeerState, f'/{self.drone_name}/peer_state', peer_qos,
        )
        # No sibling peer_state subscriptions: the stored snapshots were
        # never read. The dashboard still consumes this drone's broadcast above.
        # Consume composition.event_port when available; inline publisher
        # build is the back-compat path for tests with composition=None.
        if (self._composition is not None
                and self._composition.event_port is not None):
            self._event_port = self._composition.event_port
            self._mission_event_pub = None
        else:
            self._mission_event_pub = self.create_publisher(
                MissionEvent, '/mission/events',
                QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE, depth=50),
            )
            from drone_rescue_coordination.lib.ros_adapter.event_publisher import (
                RosEventPublisherAdapter,
            )
            self._event_port = RosEventPublisherAdapter(self._mission_event_pub)
        # Clock port resolved via the ``resolve_clock`` helper.
        self._time = resolve_clock(self, self._composition)

        self._tick_timer = self.create_timer(1.0 / self.tick_rate_hz, self._tick)
        self._peer_broadcast_timer = self.create_timer(
            1.0 / max(self.peer_broadcast_hz, 0.1),
            self._broadcast_peer_state,
        )

        # Diagnostics: lifecycle_manager's watchdog kills any executor that
        # goes 5s without a heartbeat. Hardware ID `<drone>-executor` matches
        # the watchdog's monitored_nodes list.
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID(f'{self.drone_name}-executor')
        self.updater.add('Executor State', self._diag_state)
        self.updater.add('Task Tracking', self._diag_task)

        self.get_logger().info(f'drone_executor[{self.drone_name}] configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = True
        self.get_logger().info(f'drone_executor[{self.drone_name}] activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------ subs
    def _on_odom(self, msg: Odometry) -> None:
        # Point to Position at the boundary; the domain VO is what the BT
        # actions consume from here on.
        self.ctx.current_pose = position_from_point(msg.pose.pose.position)
        self.ctx.current_z = msg.pose.pose.position.z
        # Yaw (about world +z) for the body-to-world LiDAR-bearing rotation in
        # _deflect_for_obstacle. Drones hold an arbitrary fixed yaw, so the
        # body-frame scan bearing alone is not a world-frame direction.
        q = msg.pose.pose.orientation
        self.ctx.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _on_battery_low(self, msg: Bool) -> None:
        # Battery only goes one way (low to critical); latch true forever.
        if msg.data:
            self.ctx.battery_low = True

    def _on_zone_warning(self, msg: Bool) -> None:
        # Track current state: drone moves OUT of buffer, emergency clears,
        # task tree resumes. Without this drone2 used to latch a transient
        # buffer brush at takeoff and never rescan.
        self.ctx.zone_warn = bool(msg.data)

    def _on_scan(self, msg: LaserScan) -> None:
        """Reactive obstacle avoidance from on-board LiDAR.

        Scans on this model are 360deg GPU lidar with 16 vertical beams.
        Strategy:
          * Find the global min range (drives `lidar_danger` hysteresis).
          * Find the min range in the FRONT cone (+/-45deg around the drone's
            +x axis, which is the direction it's commanded to fly via
            world-frame setpoints, close enough). If the front-cone min
            is below the danger threshold, set `lidar_front_blocked` and
            record the bearing of the closest hit so _make_target can
            deflect perpendicular to it.
        """
        # numpy mask drops the per-scan list of (i, r) tuples and the two
        # Python passes (global min + front-cone min). Bearings are cached
        # on first scan since angle_min / angle_increment are
        # scan-invariant on this drone.
        if not msg.ranges:
            return
        arr = np.asarray(msg.ranges, dtype=np.float32)
        mask = (arr > msg.range_min) & (arr < msg.range_max)
        if not mask.any():
            self.ctx.lidar_min_range = float('inf')
            self.ctx.lidar_front_blocked = False
            if self.ctx.lidar_danger:
                self.ctx.lidar_danger = False
            return
        valid_r = arr[mask]
        r_min = float(valid_r.min())
        self.ctx.lidar_min_range = r_min

        # Hysteresis on the global danger flag.
        if r_min < self.ctx.lidar_danger_enter_m:
            self.ctx.lidar_danger = True
        elif r_min > self.ctx.lidar_danger_clear_m:
            self.ctx.lidar_danger = False

        # Front-cone min (+/-45deg), computed in numpy. The bearings are
        # constant for a given scanner geometry; cache on first hit.
        front_half = math.pi / 4    # 45deg
        bearings = self._scan_bearings_cache
        if bearings is None or bearings.shape[0] != arr.shape[0]:
            n = arr.shape[0]
            idx = np.arange(n, dtype=np.float32)
            bearings = (msg.angle_min + idx * msg.angle_increment).astype(np.float32)
            # Wrap to [-pi, pi].
            bearings = (bearings + math.pi) % (2 * math.pi) - math.pi
            self._scan_bearings_cache = bearings
        front_mask = mask & (np.abs(bearings) <= front_half)
        if front_mask.any():
            front_r = arr[front_mask]
            front_min_idx_in_front = int(front_r.argmin())
            front_min_r = float(front_r[front_min_idx_in_front])
            front_min_bear = float(bearings[front_mask][front_min_idx_in_front])
        else:
            front_min_r = float('inf')
            front_min_bear = 0.0
        self.ctx.lidar_front_blocked = (
            front_min_r < self.ctx.lidar_danger_enter_m
        )
        self.ctx.lidar_block_bearing = front_min_bear

    def _on_battery_level(self, msg) -> None:
        # Float32 in [0, 1]; clamp to be safe.
        self.ctx.battery_level = max(0.0, min(1.0, float(msg.data)))

    def _on_health(self, msg: DroneHealth) -> None:
        # Only the executor's OWN health drives DOWN state. Sibling health is
        # handled by mission_manager (which owns sector reassignment).
        if msg.unrecoverable and not self.ctx.is_down:
            self.ctx.is_down = True
            self.ctx.down_reason = msg.reason
            self.get_logger().error(
                f'{self.drone_name}: marking DOWN ({msg.reason}); '
                f'broadcasting DRONE_DAMAGE_REPORT'
            )
            if not self._damage_reported:
                self._damage_reported = True
                self._publish_damage_report(msg.reason)

    def _publish_damage_report(self, reason: str) -> None:
        if getattr(self, '_event_port', None) is None:
            return
        from drone_rescue_coordination.lib.domain.events import DroneDamageReport
        self._event_port.emit(DroneDamageReport(
            severity=MissionEvent.SEVERITY_ERROR,
            raw_detail=reason,
            drone_name=self.drone_name,
        ))

    def _broadcast_peer_state(self) -> None:
        if not self._is_active or self._peer_state_pub is None:
            return
        msg = DronePeerState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.drone_name = self.drone_name
        if self.ctx.current_pose is not None:
            # Position to Point for the ROS message.
            msg.pose.position = point_from_position(self.ctx.current_pose)
            msg.pose.orientation.w = 1.0
        msg.battery = float(self.ctx.battery_level)
        if self.ctx.current_task is not None:
            msg.task_type = int(self.ctx.current_task.task_type)
            msg.current_task_id = int(self.ctx.current_task.task_id)
            msg.busy_with_victim = int(self.ctx.current_task.victim_id)
            msg.wp_index = int(self.ctx.cursor.scan_index)
            msg.wp_total = int(len(self.ctx.current_task.waypoints))
        else:
            msg.task_type = TaskAssignment.IDLE
        msg.is_down = bool(self.ctx.is_down)
        self._peer_state_pub.publish(msg)

    # ---------------------------------------------- BehaviouralLayer (L1)
    # DroneExecutor is the production realization of the L1
    # ``BehaviouralLayer`` port (lib/ports/behavioural_layer.py). On this
    # deployment L1 is a separate ROS node, so the L2-to-L1 payload arrives
    # as a ``TaskAssignment`` wire message rather than the Protocol's
    # pure-domain ``OutgoingTask``: the wire IS the boundary.
    # ``dispatch_task`` accepts what the wire delivers (and the BT
    # consumes); ``_on_task`` is the thin subscription callback that hands
    # the latched message to it.
    def dispatch_task(self, task: TaskAssignment) -> None:
        """Begin executing ``task`` on this drone's behavioural stack
        (BehaviouralLayer port). Preempts any task with a different id,
        resets the per-task cursor + BT memory, acknowledges receipt.
        Idempotent w.r.t. ``task.task_id``: re-dispatching the same task
        keeps the BT ticking on it."""
        if self.ctx.current_task is not None and self.ctx.current_task.task_id != task.task_id:
            # node-level status (not a BT-action output) publishes directly.
            self._publish_task_status(TaskStatus.PREEMPTED, 'new task arrived')
        self.ctx.current_task = task
        self.ctx.last_task_id = task.task_id
        self.ctx.cursor.reset_for_new_task()
        self.tree.reset()
        self.get_logger().info(
            f'received task #{task.task_id} '
            f'type={_TASK_TYPE_NAMES.get(task.task_type, task.task_type)} '
            f'waypoints={len(task.waypoints)}'
        )
        # Acknowledge receipt (node-level status, publish directly).
        self._publish_task_status(TaskStatus.ACCEPTED, '')

    def cancel_task(self, task_id: int) -> None:
        """Stop executing ``task_id``; the drone falls back to IDLE
        (BehaviouralLayer port). No-op if a different task is active."""
        if self.ctx.current_task is None or self.ctx.current_task.task_id != task_id:
            return
        self.ctx.current_task = None
        self.ctx.cursor.reset_for_new_task()
        self.tree.reset()

    def _on_task(self, msg: TaskAssignment) -> None:
        # ROS subscription callback to BehaviouralLayer dispatch.
        self.dispatch_task(msg)

    # ------------------------------------------------------------ tick
    def _tick(self) -> None:
        if not self._is_active:
            return
        # Wait for takeoff before starting any task. Without this we'd be
        # publishing target_pose while the controller is still in TAKEOFF
        # state and ignoring it.
        if self.ctx.current_z < self.min_takeoff_altitude_m:
            return
        if self.ctx.current_task is None:
            return
        self.ctx.now_sec = self._time.now_sec()

        # The per-tick BT runs through the typed Executor:
        # `_apply_sensors_to_ctx` projects the ExecutorSensors VO onto ctx
        # (pure input), `tree.tick(ctx)` returns its `(status,
        # BehaviouralOutput)`, and `_outputs_from_tick` maps that into the
        # `ExecutorOutputs` we publish below. No output slots on ctx to
        # clear: a tick that commands nothing returns an empty output, so
        # the previous setpoint is naturally not re-published (the "ghost
        # command" guard, structural rather than a clear-step).
        sensors = ExecutorSensors(
            now_sec=self.ctx.now_sec,
            current_pose=self.ctx.current_pose,
            lidar_min_range_m=self.ctx.lidar_min_range,
            current_task_type=(
                int(self.ctx.current_task.task_type)
                if self.ctx.current_task is not None else 5
            ),
            current_task_id=int(self.ctx.last_task_id),
            is_down=bool(self.ctx.is_down),
            battery_ok=not self.ctx.battery_low,
        )
        executor_outputs = self._executor_impl.tick(sensors)
        # Debug: every ~5s log cursor + pose to show what the BT decided.
        self._tick_count = getattr(self, '_tick_count', 0) + 1
        if self._tick_count % 25 == 0:
            t = self.ctx.current_task
            wp_total = len(t.waypoints) if t else 0
            cur_pose_str = (f'({self.ctx.current_pose.x:.1f},{self.ctx.current_pose.y:.1f})'
                            if self.ctx.current_pose else 'None')
            tp = executor_outputs.target_pose
            tgt_str = (f'({tp.x:.1f},{tp.y:.1f})' if tp is not None else 'None')
            wp_str = ('-' if t is None or self.ctx.cursor.scan_index >= wp_total
                      else f'({t.waypoints[self.ctx.cursor.scan_index].x:.1f},'
                           f'{t.waypoints[self.ctx.cursor.scan_index].y:.1f})')
            self.get_logger().info(
                f'tick #{self._tick_count} pose={cur_pose_str} '
                f'cursor={self.ctx.cursor.scan_index}/{wp_total} '
                f'wp={wp_str} out={tgt_str} '
                f'bat_low={self.ctx.battery_low} zone={self.ctx.zone_warn}'
            )
        self._publish_outputs(executor_outputs)
        self._deposit_current_position()

    def _deposit_current_position(self) -> None:
        if self.ctx.current_pose is None:
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.point.x = self.ctx.current_pose.x
        msg.point.y = self.ctx.current_pose.y
        msg.point.z = self.ctx.current_pose.z
        self._deposit_pub.publish(msg)

    def _publish_task_status(self, status: int, detail: str = '') -> None:
        """Publish a node-level TaskStatus immediately.

        Covers ACCEPTED / PREEMPTED / IN_PROGRESS. These are NOT
        BT-action outputs; the act_* completion status flows through
        ``ExecutorOutputs.completed_task_id`` instead."""
        s = TaskStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.drone_name = self.drone_name
        s.task_id = self.ctx.last_task_id
        s.status = status
        s.detail = detail
        self._status_pub.publish(s)

    def _publish_outputs(self, outputs: ExecutorOutputs) -> None:
        """Translate the typed ``ExecutorOutputs`` the BT produced this
        tick into ROS publishes. The ``Position`` to ``PoseStamped``
        projection lives here at the LifecycleNode boundary; the BT
        actions never touch a ROS message type."""
        if outputs.target_pose is not None:
            m = PoseStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'world'
            m.pose.position.x = float(outputs.target_pose.x)
            m.pose.position.y = float(outputs.target_pose.y)
            m.pose.position.z = float(outputs.target_pose.z)
            m.pose.orientation.w = 1.0
            self._target_pub.publish(m)
        if outputs.land_command:
            self._land_pub.publish(Bool(data=True))
        if outputs.completed_task_id is not None:
            self._publish_task_status(
                TaskStatus.COMPLETED, outputs.status_detail,
            )
        # Periodic IN_PROGRESS for SCAN tasks, gives mission_manager the
        # current waypoint index so on reassignment it can dispatch only
        # the un-visited tail to survivors instead of restarting them at
        # waypoint 0 (which is the inner ring, the "everyone converges to
        # center" symptom). Throttled to 1 Hz to avoid flooding the topic.
        now = self._time.now_sec()
        last = getattr(self, '_last_progress_t', 0.0)
        if (
            self.ctx.current_task is not None
            and self.ctx.current_task.task_type == TaskAssignment.SCAN_WAYPOINTS
            and (now - last) >= 1.0
        ):
            self._publish_task_status(
                TaskStatus.IN_PROGRESS, f'wp={self.ctx.cursor.scan_index}',
            )
            self._last_progress_t = now

    # ----------------------------------------------------------- diagnostics
    def _diag_state(self, stat):
        if not self._is_active:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE,
                         'Executor inactive')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                         f'active, z={self.ctx.current_z:.1f}m')
        stat.add('Active', str(self._is_active))
        stat.add('Altitude', f'{self.ctx.current_z:.2f}m')
        stat.add('Battery Low', str(self.ctx.battery_low))
        stat.add('Zone Warn', str(self.ctx.zone_warn))
        return stat

    def _diag_task(self, stat):
        t = self.ctx.current_task
        if t is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'no active task')
        else:
            n_wp = len(t.waypoints)
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                         f'task #{t.task_id} type={t.task_type} '
                         f'wp {self.ctx.cursor.scan_index}/{n_wp}')
            stat.add('Task ID', str(t.task_id))
            stat.add('Task Type', str(t.task_type))
            stat.add('Waypoints', str(n_wp))
            stat.add('Cursor', str(self.ctx.cursor.scan_index))
        return stat


def main(args=None):
    rclpy.init(args=args)
    from rclpy.executors import MultiThreadedExecutor
    node = bind_composition(DroneExecutor())
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
