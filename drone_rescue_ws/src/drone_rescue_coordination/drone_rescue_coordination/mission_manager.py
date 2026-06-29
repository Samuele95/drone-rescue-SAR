"""Mission Manager: central orchestrator for the multi-drone SAR mission.

3T Architecture: Deliberative Layer (L3) host / Executive (L2) adapter.

Per Marcelletti slides (pp. 33, 42-44, 85-86): this LifecycleNode hosts
the project's Layer 3 deliberative planner (the ``Mission`` aggregate
plus the auction engine) and simultaneously acts as the Layer 2
executive adapter that translates ROS callbacks into planner inputs and
the planner's ``OutgoingTask`` records into ROS publishes. Until the
saga lift completes (see internal design notes),
``mission_manager.py`` plays both layer roles; post-cutover it shrinks
to the L2 adapter and the L3 logic moves entirely to
``lib/domain/mission.Mission``.

Patterns applied:
  * Mediator: arbitrates between detection_filter (detections) and
              drone_executors (actions). Removes N×M coupling.
  * Saga: per-victim sub-mission INVESTIGATE → CONFIRM, with a
          compensating RTH if the drone fails or runs low on battery.
  * Repository: VictimRegistry encapsulates victim storage; trivially
                swappable for a database later.
  * State machine: explicit MissionFSM enum drives top-level behavior.
  * Auction: single-item assignment auction (utility =
             priority / max(distance, 1.0)). Battery readiness is
             modelled as a binary admission gate (DroneRecord.
             battery_ok): low-battery drones are excluded from the
             auction entirely, not down-weighted continuously.
             See `_auction()` for the citation (Gerkey & Mataric,
             IJRR 2004) and the seeded tie-break details.

Lifecycle node: gets configured/activated by the existing lifecycle_manager.
"""

from __future__ import annotations

import random
import time
from dataclasses import dataclass, field, replace
from enum import IntEnum
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import (
    ParameterDescriptor, ParameterType, SetParametersResult,
)

from std_msgs.msg import Bool, Header, String, UInt32
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry

from drone_rescue_msgs.msg import (
    TaskAssignment,
    TaskStatus,
    VictimCandidate,
    MissionState,
    CoverageMetrics,
    DroneHealth,
    MissionEvent,
)

from drone_rescue_coordination.lib.auction import AuctionEngine, TaskType
from drone_rescue_coordination.lib.allocation import (
    AllocationStrategyFactory, BatchAllocationBidder,
)
from drone_rescue_coordination.lib.domain.affect import ExploitationTracker
from drone_rescue_coordination.lib.domain.drone_state import DroneState
from drone_rescue_coordination.lib.domain.elevation import ElevationModel
from drone_rescue_coordination.lib.domain.feasibility import (
    assess_feasibility, remaining_plan_length,
)
from drone_rescue_coordination.lib.domain.entities import Drone, Victim
from drone_rescue_coordination.lib.domain.fleet import default_drone_names_list
from drone_rescue_coordination.lib.domain.mission import Mission as _Mission
from drone_rescue_coordination.lib.domain.no_fly_zone_filter import (
    drone_name_from_violation, filter_waypoints, load_no_fly_zones,
    precompute_states,
)
from drone_rescue_coordination.lib.domain.scenario_schema import ParamScope
from drone_rescue_coordination.lib.domain.task_type import task_type_label
from drone_rescue_coordination.lib.domain.value_objects import (
    Position, WatchdogClock, SectorWedge, OutgoingTask,
)
from drone_rescue_coordination.lib.domain.victim_sub_mission import (
    VictimSubMission,
)
from drone_rescue_coordination.lib.ros_adapter.parameter_declarer import (
    declare_for_scope,
)
from drone_rescue_coordination.lib.composition import (
    bind_composition, resolve_clock,
)
# Import-time guard: keep the domain TaskType enum in sync with the ROS
# message's task_type ints. Either side can be edited freely as long as
# the values stay aligned; a divergence makes the package fail to import
# (loud, immediate failure) rather than producing silent auction
# misbehaviour at runtime.
# Guard ALL six values, not just INVESTIGATE/CONFIRM. SCAN
# is the project's hottest dispatch path and its names diverge across the
# boundary (domain TaskType.SCAN ↔ ROS TaskAssignment.SCAN_WAYPOINTS), so
# it most needs the explicit value check.
_TASK_TYPE_ROS_VALUES = {
    TaskType.SCAN:        TaskAssignment.SCAN_WAYPOINTS,
    TaskType.INVESTIGATE: TaskAssignment.INVESTIGATE,
    TaskType.CONFIRM:     TaskAssignment.CONFIRM,
    TaskType.RTH:         TaskAssignment.RTH,
    TaskType.LAND:        TaskAssignment.LAND,
    TaskType.IDLE:        TaskAssignment.IDLE,
}
for _tt, _ros_val in _TASK_TYPE_ROS_VALUES.items():
    assert _tt == _ros_val, (
        f'TaskType.{_tt.name} ({int(_tt)}) drifted from its '
        f'TaskAssignment ROS constant ({_ros_val})'
    )
from drone_rescue_coordination.lib.sector_geometry import (
    sector_owner_for as _sector_owner_for_helper,
)
from drone_rescue_coordination.lib.sar_patterns import (
    CoveragePatternFactory,
    PlannerInput,
)


# domain
# MissionStage / VictimStage are the typed enums
# from lib/domain/state_machines.py (the single source of truth). The
# legacy IntEnums lived here; they're re-exported so the existing
# `from drone_rescue_coordination.mission_manager import MissionStage`
# import path keeps working.
from drone_rescue_coordination.lib.domain.state_machines import (
    IllegalTransition,
    MissionStage,
    MissionStateMachine,
    TransitionEvent,
    VictimStage,
    VictimStateMachine,
)


@dataclass
class VictimRecord:
    candidate_id: int
    position: Point
    confidence: float
    stage: VictimStage = VictimStage.DETECTED
    assigned_drone: Optional[str] = None
    last_update_sec: float = 0.0
    # Cached runner-up from the INVESTIGATE
    # auction. `_on_task_completed` consumes this for the CONFIRM
    # witness handover instead of running a second auction with
    # `exclude={d.name}`. None when only one drone was eligible.
    witness_drone: Optional[str] = None

    def __repr__(self) -> str:
        # The default dataclass __repr__ exposes geometry_msgs.Point's
        # binary wire-format representation, which renders as gibberish
        # in mission logs and exception tracebacks. Provide a
        # readable repr so log lines are useful at debug time.
        return (
            f'Victim(#{self.candidate_id} stage={self.stage.name} '
            f'conf={self.confidence:.2f} '
            f'pos=({self.position.x:.1f}, {self.position.y:.1f}) '
            f'assigned={self.assigned_drone or "—"})'
        )


@dataclass
class DroneRecord:
    name: str
    pose: Optional[Point] = None
    battery_ok: bool = True
    current_task_id: int = 0
    current_task_type: int = TaskAssignment.IDLE
    busy_with_victim: Optional[int] = None
    # The list of scan waypoints the drone is sweeping, plus the index of
    # the next one to issue if the executor finishes its current task. We
    # re-issue waypoints from `scan_cursor` onward when resuming after an
    # INVESTIGATE preemption.
    scan_waypoints: List[Point] = field(default_factory=list)
    scan_cursor: int = 0
    # drone_health_monitor → unrecoverable=True trips this
    # flag. mission_manager treats `is_down` drones as unavailable for the
    # auction and partitions their remaining waypoints across survivors.
    is_down: bool = False
    # Flight-controller operational state, when known. The auction excludes a
    # drone in EMERGENCY / LANDING / RETURNING even with a healthy battery
    # (auction._eligible_bids reads this via getattr). Defaults to None
    # (unknown ⇒ eligible); set where mission_manager commits a drone to an
    # RTH / landing so the health gate is not bypassed.
    drone_state: Optional[DroneState] = None
    # Capability weight feeding the auction utility. 1.0 = baseline; a
    # heterogeneous fleet sets per-drone values via the drone_capabilities param
    # so a faster / better-sensor drone is preferred for tasks.
    capability: float = 1.0
    # The watchdog timestamps and SCAN-dispatch
    # offset bookkeeping live on a frozen `WatchdogClock` VO. Advance via
    # `d.clock = replace(d.clock, last_status_t=now)`. Read silence via
    # `d.clock.silence(now)`.
    clock: WatchdogClock = field(default_factory=WatchdogClock)
    # SAR discipline: which angular sector this drone owns.
    # `sector_start_rad`/`sector_end_rad` define the wedge from the mission
    # centre. A candidate whose bearing falls inside this wedge is
    # auctioned ONLY to this drone (no opportunistic cross-sector poaching).
    # Cross-sector handover happens only as a fallback when the owner is
    # unavailable.
    sector_start_rad: float = 0.0
    sector_end_rad: float = 0.0

    def __repr__(self) -> str:
        # See VictimRecord.__repr__ for the same motivation.
        # Pull the label from the canonical
        # TaskType.label registry (single source of truth);
        # zero allocation per call (was a fresh 6-entry dict).
        pose_str = (
            f'({self.pose.x:.1f}, {self.pose.y:.1f})'
            if self.pose is not None else '—'
        )
        task_name = task_type_label(self.current_task_type)
        flags = []
        if self.is_down:
            flags.append('DOWN')
        if not self.battery_ok:
            flags.append('BAT_LOW')
        if self.busy_with_victim is not None:
            flags.append(f'busy=#{self.busy_with_victim}')
        flag_str = (' [' + ' '.join(flags) + ']') if flags else ''
        return (
            f'Drone({self.name} task={task_name} pose={pose_str} '
            f'cursor={self.scan_cursor}/{len(self.scan_waypoints)}'
            f'{flag_str})'
        )

    # Explicit scan-waypoint mutation surface. The drone-loss
    # handover (`_reassign_sector`) used bare field writes; routing them
    # through named methods keeps the mutation contract auditable and
    # mockable, and centralises the "what does a drained drone look like"
    # invariant in one place.
    def append_scan_tail(self, extra: List[Point]) -> None:
        """Append an ordered run of waypoints to this drone's scan plan."""
        self.scan_waypoints = list(self.scan_waypoints) + list(extra)

    def clear_scan_state(self) -> None:
        """Drain all scan bookkeeping, used when this drone is marked DOWN
        and its waypoints have been handed to a survivor."""
        self.scan_waypoints = []
        self.scan_cursor = 0
        self.busy_with_victim = None


# Adapter implementing the SectorOwnerPolicy port by
# delegating to the LifecycleNode's existing _sector_owner_for helper.
# Keeps the sector-geometry + mission-center plumbing in the L2 adapter
# while the L3 Mission planner sees only the typed port.
class _MissionSectorOwnerAdapter:
    def __init__(self, manager: 'MissionManager'):
        self._manager = manager

    def owner_for(self, p):
        return self._manager._sector_owner_for(p)


# node

# Operator-injected investigate goals (dashboard
# right-click on the mission scene) mint synthetic candidate ids from
# this base, far above detection_filter's cluster id range, so the
# two id spaces can never collide.
OPERATOR_GOAL_CID_BASE = 9000


class MissionManager(LifecycleNode):
    def __init__(self, strategy_factory=None, rng=None, *, composition=None,
                 allocation_factory=None):
        """Construct the lifecycle node.

        ``strategy_factory`` is an optional callable
        ``(name: str) -> CoverageStrategy``. By default we use
        ``CoveragePatternFactory.create``; tests can inject a
        ``lambda name: MockStrategy()`` to exercise the auction /
        sector / saga logic without depending on the registered
        strategies.

        ``rng`` is an optional pre-seeded ``random.Random`` used as
        the auction tie-break source. When omitted, the lifecycle reads
        the ``seed`` parameter and constructs ``random.Random(seed + 7919)``.
        Tests can pass ``rng=random.Random(42)``
        to get a stable tie-break sequence without reaching into
        ``self._auction_engine._rng`` post-construction.

        Strategy construction is deferred to ``on_configure`` so that
        a node instantiated for testing doesn't pay the strategy
        construction cost up-front and so that tests can override the
        factory before the lifecycle activates.
        """
        super().__init__('mission_manager')
        # CompositionRoot injection (back-compat
        # default: None → on_configure lazy-constructs adapters as
        # before).
        self._composition = composition
        self._strategy_factory = (
            strategy_factory or CoveragePatternFactory.create
        )
        # Selectable task-allocation policy. Mirrors strategy_factory; default
        # is AllocationStrategyFactory.create. Tests can inject
        # ``lambda name, drones, rng: MockStrategy()``.
        self._allocation_factory = (
            allocation_factory or AllocationStrategyFactory.create
        )
        self._injected_rng = rng    # consumed in __init__ below

        # Schema-registered MISSION params come from
        # PARAM_SCHEMA via declare_for_scope. Per-name overrides
        # preserve the legacy runtime defaults until those values are
        # reconciled into PARAM_SCHEMA itself. Non-schema params
        # (drone_names, mission_center_x/y, confirm_orbit_radius) and
        # the LAUNCH-scope seed continue to be declared inline.
        declare_for_scope(
            self, ParamScope.MISSION,
            defaults_override={
                'mission_radius': 85.0,
                'survey_altitude': 10.0,
                'camera_footprint_m': 11.5,
                'coverage_overlap': 0.3,
                'inner_radius': 15.0,
            },
        )
        self.declare_parameter('drone_names', default_drone_names_list())
        self.declare_parameter('mission_center_x', 0.0)
        self.declare_parameter('mission_center_y', 0.0)
        # Terrain elevation gradient (m per m). Both 0.0 (default) = flat
        # terrain, identical to the legacy behaviour. Non-zero makes scan
        # waypoints fly at a constant height above ground level; see
        # lib/domain/elevation.py and _begin_scan.
        self.declare_parameter('terrain_slope_x', 0.0)
        self.declare_parameter('terrain_slope_y', 0.0)
        # Selectable coverage strategy. See CoveragePatternFactory for the
        # registered names; default 'spiral_out' = concentric arcs from the
        # launch pad outward.
        self.declare_parameter('coverage_pattern', 'spiral_out')
        # Selectable task-allocation strategy. See AllocationStrategyFactory
        # for the registered names; default 'greedy_auction' = the existing
        # nearest-drone greedy auction.
        self.declare_parameter('allocation_strategy', 'greedy_auction')
        self.declare_parameter('confirm_orbit_radius', 4.0)
        # The multi-view INVESTIGATE plan the deliberative layer (L3)
        # commits and stamps onto each INVESTIGATE task. Defaults match the
        # drone_executor's own investigate_radius_m / investigate_dwell_s so
        # behaviour is unchanged; a scenario YAML can now vary the plan from
        # the planner side (the executor params remain the bag-replay
        # fallback). Mirrors the confirm_orbit_radius precedent.
        self.declare_parameter('investigate_radius_m', 5.0)
        self.declare_parameter('investigate_dwell_s', 2.0)
        # Master RNG seed for reproducible runs. Forwarded to the
        # CoverageStrategy (random_walk uses it) and to the auction
        # tie-break Random instance below. environment_monitor and
        # sensor_degradation read their own `seed` params (offset
        # internally for stream independence).
        self.declare_parameter('seed', 0)

        # State
        self._stage: MissionStage = MissionStage.INIT
        self._mission_start_sec: Optional[float] = None
        self._victims: Dict[int, VictimRecord] = {}
        self._drones: Dict[str, DroneRecord] = {}
        # No-fly-zone breach debounce: count consecutive in-zone samples per
        # drone so a transient boundary brush (the drone climbs over / routes
        # around and continues) does NOT force-RTH the whole mission; only a
        # drone genuinely STUCK inside a zone aborts. (last_t resets the streak
        # when samples stop arriving, i.e. the drone left the zone.)
        self._zone_breach_streak: Dict[str, int] = {}
        self._zone_breach_last_t: Dict[str, float] = {}
        # Typed Mission aggregate. Populated via ``_sync_to_mission`` once
        # per tick; readers consume ``self._mission.confirmed_count()`` etc.
        # instead of walking the legacy dicts inline. Imports hoisted to
        # module top.
        self._mission = _Mission()
        self._next_task_id: int = 1
        self._sectors_total: int = 0
        self._published_complete: bool = False
        # Track cluster_ids dropped
        # by the pre-survey gate so we log each one exactly once,
        # not on every re-publish (the same cid can re-arrive at the
        # publish_rate_hz cadence). Cleared in ``_begin_scan`` so
        # candidates that survive into SCANNING are still processed.
        self._pre_survey_dropped_cids: set = set()
        # Next synthetic cid for operator goals.
        self._next_operator_cid: int = OPERATOR_GOAL_CID_BASE

        # Topic placeholders, created in on_configure
        self._task_pubs: Dict[str, Optional[any]] = {}
        self._candidate_sub = None
        self._task_status_subs: List = []
        self._battery_subs: List = []
        self._odom_subs: List = []
        self._health_subs: List = []
        self._survey_start_sub = None
        self._mission_state_pub = None
        self._mission_event_pub = None
        # Saga-confirmation signal for the
        # visualizer (the multi-view ``VictimCandidate.confirmed``
        # field does NOT fire when a single drone scans a sector,
        # so saga-confirmed victims would stay orange in RViz without
        # this independent channel). Carries the cluster_id (=
        # ``VictimCandidate.candidate_id``) of victims whose CONFIRM
        # task succeeded. Wired in on_configure.
        self._victims_confirmed_pub = None
        # Events route through an EventPort
        # (driven port). Defaults to None until on_configure wires the
        # RosEventPublisherAdapter; tests can override at construction.
        self._event_port = None
        self._tick_timer = None
        self._is_active: bool = False
        # /survey/start race fix: set when a latched
        # /survey/start is delivered to our subscription while we are
        # still configuring (before on_activate). on_activate honours it.
        self._survey_start_pending: bool = False

        # Read params now (immutable after init)
        self.drone_names: List[str] = list(self.get_parameter('drone_names').value)
        self.mission_center = (
            float(self.get_parameter('mission_center_x').value),
            float(self.get_parameter('mission_center_y').value),
        )
        self.mission_radius = float(self.get_parameter('mission_radius').value)
        self.survey_altitude = float(self.get_parameter('survey_altitude').value)
        self.camera_footprint_m = float(self.get_parameter('camera_footprint_m').value)
        self.coverage_overlap = float(self.get_parameter('coverage_overlap').value)
        self.coverage_pattern_name = str(self.get_parameter('coverage_pattern').value)
        self.allocation_strategy_name = str(
            self.get_parameter('allocation_strategy').value)
        self.inner_radius = float(self.get_parameter('inner_radius').value)
        # Flight-plan feasibility: survey cruise speed and the endurance
        # reserve kept in hand when judging whether a drone's remaining scan
        # plan + return leg fits inside its battery endurance.
        self.declare_parameter('survey_speed_mps', 3.0)
        self.declare_parameter('feasibility_reserve_s', 30.0)
        self._survey_speed_mps = float(
            self.get_parameter('survey_speed_mps').value)
        self._feasibility_reserve_s = float(
            self.get_parameter('feasibility_reserve_s').value)
        # Per-drone capability weights aligned with drone_names; empty
        # (default) ⇒ a homogeneous fleet (every capability 1.0). A length
        # mismatch is ignored (logged) so a misconfigured fleet stays uniform.
        self.declare_parameter(
            'drone_capabilities', [],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )
        caps = list(self.get_parameter('drone_capabilities').value or [])
        if caps and len(caps) != len(self.drone_names):
            self.get_logger().warn(
                f'drone_capabilities has {len(caps)} entries but there are '
                f'{len(self.drone_names)} drones — ignoring (fleet stays uniform)'
            )
            caps = []
        self._drone_capabilities = {
            name: float(caps[i]) if caps else 1.0
            for i, name in enumerate(self.drone_names)
        }
        # Per-drone battery endurance (s) from DroneHealth.battery_remaining_s,
        # and a latch so a FLIGHT_PLAN_INFEASIBLE event fires only on the flip.
        self._battery_remaining_s: Dict[str, float] = {}
        self._infeasible_latch: Dict[str, bool] = {}
        # Terrain model: flat by default (no-op). _begin_scan offsets each
        # scan waypoint's altitude by elevation_at(x, y) so the drone holds a
        # constant above-ground-level height over sloped terrain.
        self._elevation = ElevationModel.from_slopes(
            slope_x=float(self.get_parameter('terrain_slope_x').value),
            slope_y=float(self.get_parameter('terrain_slope_y').value),
        )
        # No-fly zones: the live planner enforces the same zones the
        # zone_manager detects: scan waypoints inside a zone are filtered in
        # _begin_scan, and a /zones/violation breach forces RTH. Loaded from
        # the no_fly_zones_yaml param, else the bringup default location.
        self.declare_parameter('no_fly_zones_yaml', '')
        self._no_fly_zones = self._load_no_fly_zones(
            str(self.get_parameter('no_fly_zones_yaml').value)
        )
        self._zone_states = precompute_states(self._no_fly_zones)
        if self._no_fly_zones:
            self.get_logger().info(
                f'No-fly-zone enforcement active: {len(self._no_fly_zones)} '
                f'zone(s) loaded'
            )
        # Strategy construction is deferred to on_configure
        # so tests can inject a strategy_factory and so that __init__ has
        # no side effects beyond reading params. The pattern-name typo
        # check still fires at startup: on_configure aborts the lifecycle
        # transition if the name is unknown, so the operator gets the
        # same eager-validation guarantee as before.
        self._coverage_strategy = None
        self.investigate_hover_s = float(self.get_parameter('investigate_hover_seconds').value)
        self.task_status_timeout_s = float(self.get_parameter('task_status_timeout_s').value)
        self.investigate_confidence_floor = float(
            self.get_parameter('investigate_confidence_floor').value
        )
        self.max_concurrent_investigations = int(
            self.get_parameter('max_concurrent_investigations').value
        )
        self.confirm_hover_s = float(self.get_parameter('confirm_hover_seconds').value)
        self.confirm_orbit_r = float(self.get_parameter('confirm_orbit_radius').value)
        self.investigate_radius_m = float(self.get_parameter('investigate_radius_m').value)
        self.investigate_dwell_s = float(self.get_parameter('investigate_dwell_s').value)
        self.reject_age_s = float(self.get_parameter('reject_age_seconds').value)
        self.mission_timeout_s = float(self.get_parameter('mission_timeout_seconds').value)
        self.seed = int(self.get_parameter('seed').value)
        # Tie-break RNG for the auction. Without this, two drones equidistant
        # from a victim get resolved by dict insertion order, fine in practice
        # but not reproducible across runs (DDS discovery order varies). With a
        # seeded Random instance, two runs with the same seed pick the same
        # winner. Offset 7919 (a prime) keeps this stream uncorrelated from the
        # coverage planner's stream.
        # Prefer an injected RNG (test path) over the
        # seeded one (production path). When `rng=` is passed at
        # construction we use it verbatim; otherwise we fall back to
        # the seed-derived stream. Either way the AuctionEngine holds
        # a stable reference for the lifetime of this node, tests no
        # longer need to mutate `_auction_engine._rng` after the fact.
        self._auction_rng = (
            self._injected_rng
            if self._injected_rng is not None
            else random.Random(self.seed + 7919)
        )
        # Extracted auction logic. self._drones is shared
        # by reference: AuctionEngine sees mutations in real time.
        self._auction_engine = AuctionEngine(self._drones, self._auction_rng)
        # Selectable allocation strategy (greedy_auction / round_robin /
        # hungarian). Built here (not deferred to on_configure) because
        # _auction() is exercised by unit tests against a never-activated
        # node. An unknown name leaves the strategy None; on_configure then
        # aborts the lifecycle cleanly (same eager-validation guarantee as
        # coverage_pattern). self._drones is shared by reference.
        # Instantiate the affect monitor eagerly so the motivation_workspace
        # allocation strategy gets a non-None reference and the
        # D1->D2 frustration-feedback loop is live. Other strategies
        # ignore the kwarg via the factory's signature introspection.
        # The same instance is exposed on ``composition.affect_monitor``
        # in on_configure for any downstream consumer.
        self._affect_monitor = ExploitationTracker()
        try:
            self._allocation_strategy = self._allocation_factory(
                self.allocation_strategy_name, self._drones,
                self._auction_rng, affect=self._affect_monitor)
            self._allocation_error = None
        except ValueError as e:
            self._allocation_strategy = None
            self._allocation_error = str(e)
        except TypeError:
            # Legacy injected factories whose signature doesn't accept
            # ``affect=``. Fall back to the
            # affect-free call so existing test injectors keep working.
            try:
                self._allocation_strategy = self._allocation_factory(
                    self.allocation_strategy_name, self._drones,
                    self._auction_rng)
                self._allocation_error = None
            except ValueError as e:
                self._allocation_strategy = None
                self._allocation_error = str(e)

        for d in self.drone_names:
            self._drones[d] = DroneRecord(
                name=d, capability=self._drone_capabilities.get(d, 1.0))
            self._task_pubs[d] = None

        self.get_logger().info(
            f'mission_manager initialized: {len(self.drone_names)} drones, '
            f'disk r={self.mission_radius}m'
        )

    # runtime params
    # Whitelist of parameters Mission Control may tweak after activation.
    # Disk geometry (mission_radius / inner_radius / coverage_pattern /
    # camera_footprint_m) is excluded because _begin_scan has already run
    # and changing these now would put the executor's scan_waypoints out
    # of sync with mission_manager's auction logic.
    # Derived from `lib/domain/scenario_schema`
    # so a new runtime-tweakable param is one row in PARAM_SCHEMA.
    from drone_rescue_coordination.lib.domain.scenario_schema import (
        ParamScope as _ParamScope,
        runtime_tweakable_for_scope as _runtime_tweakable_for_scope,
    )
    _RUNTIME_PARAMS = _runtime_tweakable_for_scope(_ParamScope.MISSION)
    del _ParamScope, _runtime_tweakable_for_scope

    # Plan/geometry params: read once when the survey plan is built in
    # _begin_scan. They are NOT live-tweakable mid-mission (re-planning would
    # desync the executor's waypoint queues), but Mission Control applies an
    # operator's edits at activation (BEFORE the survey starts) so we accept
    # them then and let _begin_scan pick up the new values. Changing them once
    # scanning has begun is rejected with a restart hint.
    _PLAN_PARAMS = (
        'mission_radius', 'inner_radius', 'survey_altitude',
        'camera_footprint_m', 'coverage_overlap',
    )

    def _on_runtime_params(self, params) -> SetParametersResult:
        """Validate and apply incoming parameter changes from Mission Control.

        Runtime-tweakable knobs apply live. Plan/geometry knobs apply if the
        survey has not started yet (the GUI sets them at activation); once
        scanning, they are rejected so a late change can't desync the planner
        from the executor's queues. Truly launch-only knobs are always refused.
        """
        survey_started = self._mission_start_sec is not None
        for p in params:
            if p.name in self._RUNTIME_PARAMS:
                continue
            if p.name in self._PLAN_PARAMS:
                if survey_started:
                    return SetParametersResult(
                        successful=False,
                        reason=(f"'{p.name}' is a plan parameter — the survey "
                                "is already running; restart to change it"),
                    )
                continue   # pre-survey: accept; _begin_scan will use it
            if hasattr(self, p.name) or self.has_parameter(p.name):
                return SetParametersResult(
                    successful=False,
                    reason=(f"'{p.name}' is launch-time only — restart the "
                            "mission to change it"),
                )
        # All names accepted; refresh the cached values atomically.
        for p in params:
            value = p.value
            if p.name == 'mission_radius':
                self.mission_radius = float(value)
            elif p.name == 'inner_radius':
                self.inner_radius = float(value)
            elif p.name == 'survey_altitude':
                self.survey_altitude = float(value)
            elif p.name == 'camera_footprint_m':
                self.camera_footprint_m = float(value)
            elif p.name == 'coverage_overlap':
                self.coverage_overlap = float(value)
            elif p.name == 'investigate_confidence_floor':
                self.investigate_confidence_floor = float(value)
            elif p.name == 'max_concurrent_investigations':
                self.max_concurrent_investigations = int(value)
            elif p.name == 'reject_age_seconds':
                self.reject_age_s = float(value)
            elif p.name == 'mission_timeout_seconds':
                self.mission_timeout_s = float(value)
            elif p.name == 'investigate_hover_seconds':
                self.investigate_hover_s = float(value)
            elif p.name == 'confirm_hover_seconds':
                self.confirm_hover_s = float(value)
            elif p.name == 'task_status_timeout_s':
                self.task_status_timeout_s = float(value)
            self.get_logger().info(
                f'runtime param updated: {p.name} = {value}'
            )
        return SetParametersResult(successful=True)

    # lifecycle
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring mission_manager...')

        # Construct the coverage strategy here (rather
        # than in __init__) so tests can inject a strategy_factory
        # and so that __init__ remains a side-effect-free constructor.
        # An unknown pattern name aborts the lifecycle transition,
        # which preserves the eager-validation behaviour.
        try:
            self._coverage_strategy = self._strategy_factory(
                self.coverage_pattern_name
            )
        except ValueError as e:
            self.get_logger().error(
                f'unknown coverage_pattern={self.coverage_pattern_name!r}: {e}'
            )
            return TransitionCallbackReturn.FAILURE

        # Expose the affect
        # monitor on the composition root so downstream consumers
        # (future ExecutiveSupervisor wiring; the live D1 observe()
        # path) can read it without reaching into mission_manager.
        if (self._composition is not None
                and self._composition.affect_monitor is None):
            self._composition.affect_monitor = self._affect_monitor

        # Eager-validate the allocation strategy (built in __init__). An
        # unknown allocation_strategy name aborts the lifecycle here, same
        # as an unknown coverage_pattern.
        if self._allocation_strategy is None:
            self.get_logger().error(
                f'unknown allocation_strategy='
                f'{self.allocation_strategy_name!r}: {self._allocation_error}'
            )
            return TransitionCallbackReturn.FAILURE

        # Register the runtime tweak callback before any params can arrive.
        self.add_on_set_parameters_callback(self._on_runtime_params)

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # TopicFactory adoption. The legacy `task_qos`
        # (RELIABLE,TRANSIENT_LOCAL,depth=10) maps to `QosName.TASK` in
        # the registry; per-drone task publishers come from the factory
        # so adding a new per-drone topic is a one-row registry edit.
        from drone_rescue_coordination.lib.ros_adapter.topic_factory import (
            QosName, TopicFactory,
        )
        self._topic_factory = TopicFactory(self, self.drone_names)
        self._task_pubs = self._topic_factory.per_drone_pubs(
            'task', TaskAssignment,
        )

        self._mission_state_pub = self.create_publisher(
            MissionState, '/mission/state',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       depth=1),
        )
        # Operator-facing event stream: drives the dashboard mission log.
        # RELIABLE + depth=50 so a late dashboard subscriber catches the last
        # 50 events on connect (TRANSIENT_LOCAL is too aggressive, we don't
        # want to replay everything on every reconnect).
        # Consume composition.event_port when
        # available; fall back to the inline production-publisher
        # construction so tests that pass composition=None / sparse
        # compositions still emit. The CompositionRoot's event_port
        # already carries the canonical QoS (RELIABLE/VOLATILE depth=50).
        if (self._composition is not None
                and self._composition.event_port is not None):
            self._event_port = self._composition.event_port
            self._mission_event_pub = None
        else:
            self._mission_event_pub = self.create_publisher(
                MissionEvent, '/mission/events',
                QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE,
                           depth=50),
            )
            from drone_rescue_coordination.lib.ros_adapter.event_publisher import (
                RosEventPublisherAdapter,
            )
            self._event_port = RosEventPublisherAdapter(self._mission_event_pub)
        # Saga-confirmation channel. Each
        # message carries the cluster_id (``VictimCandidate.candidate_id``)
        # of a victim whose CONFIRM task just succeeded. TRANSIENT_LOCAL
        # with depth=64 so a victim_visualizer that joins or restarts
        # mid-mission picks up the history rather than only the next
        # confirmation; depth 64 comfortably exceeds the per-mission
        # victim count we plan for in the disk geometry.
        self._victims_confirmed_pub = self.create_publisher(
            UInt32, '/victims/saga_confirmed',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       depth=64),
        )
        # Clock port resolved via the ``resolve_clock`` helper.
        self._time = resolve_clock(self, self._composition)

        # Subscriptions
        self._candidate_sub = self.create_subscription(
            VictimCandidate, '/victims/candidates',
            self._on_candidate, 10,
        )
        # Per-drone TaskStatus feedback channels: drone_executor
        # publishes on /<drone>/task_status. Via
        # TopicFactory.make_sub (`_on_task_status` doesn't take a
        # drone_name kwarg, so per_drone_subs's late-binding signature
        # doesn't fit; make_sub per drone is the right shape).
        self._task_status_subs: List = [
            self._topic_factory.make_sub(
                f'/{d}/task_status', TaskStatus,
                self._on_task_status, QosName.CMD,
            )
            for d in self.drone_names
        ]
        # /survey/start: TRANSIENT_LOCAL so we pick up the readiness_coordinator's
        # latched message even though that publish happens before our
        # on_configure subscription is created. depth=1, only the latest
        # signal matters.
        self._survey_start_sub = self.create_subscription(
            Bool, '/survey/start',
            self._on_survey_start,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )
        # Operator investigate-here goals from the
        # dashboard mission scene.
        self._operator_goal_sub = self.create_subscription(
            PointStamped, '/mission/operator_goal',
            self._on_operator_goal, 10,
        )
        # Operator recall. The dashboard's
        # original Return-home/Recall buttons published a single home
        # setpoint straight to /<drone>/survey_target, which the
        # executor's per-tick survey stream overwrote within one tick,
        # the drone twitched and resumed. Recall must flow through the
        # task system: an RTH TaskAssignment changes the executor's BT
        # branch (it stops streaming survey targets and flies home),
        # and the battery_ok gate keeps the drone out of subsequent
        # auctions. msg.data = drone name, or '*' for the whole fleet.
        self._operator_rth_sub = self.create_subscription(
            String, '/mission/operator_rth',
            self._on_operator_rth, 10,
        )
        # The live subscriber to zone_manager's no-fly-zone breach
        # alert (previously only the dead surveyor.py listened). A breach
        # forces the offending drone to RTH (_on_zone_violation).
        self._zone_violation_sub = self.create_subscription(
            String, '/zones/violation',
            self._on_zone_violation, 10,
        )
        # Per-drone battery / odom / health subs via
        # TopicFactory.per_drone_subs. The callback signature
        # `(msg, drone_name)` matches the legacy lambda late-binding
        # pattern; the factory bakes the closure once.
        # `odom` uses the SENSOR QoS (BEST_EFFORT,VOLATILE,depth=10),
        # which matches the legacy hand-rolled profile exactly.
        self._battery_subs = self._topic_factory.per_drone_subs(
            'battery_low', Bool, self._on_battery,
        )
        self._odom_subs = self._topic_factory.per_drone_subs(
            'odom', Odometry, self._on_odom,
        )
        # Per-drone health stream from
        # drone_health_monitor. On unrecoverable=True we mark the
        # drone DOWN, drain its task, and partition its remaining
        # scan_waypoints across survivors.
        self._health_subs = self._topic_factory.per_drone_subs(
            'health', DroneHealth, self._on_health,
        )

        # Tick at 1 Hz
        self._tick_timer = self.create_timer(1.0, self._tick)

        self._set_stage(MissionStateMachine.transition(
            self._stage, TransitionEvent.LIFECYCLE_CONFIGURED,
        ))
        self.get_logger().info('mission_manager configured (ARMING)')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = True
        self._set_stage(MissionStateMachine.transition(
            self._stage, TransitionEvent.LIFECYCLE_ACTIVATED,
        ))
        self.get_logger().info('mission_manager activated (DEPLOYING — waiting for /survey/start)')
        # /survey/start race fix: if the latched
        # /survey/start was delivered while we were still configuring
        # (readiness fired before activation), honour it now that we
        # are active and in DEPLOYING.
        if self._survey_start_pending:
            self._survey_start_pending = False
            self.get_logger().info(
                'Deferred /survey/start (received during configure) — '
                'starting survey now'
            )
            self._begin_survey()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # callbacks
    def _on_odom(self, msg: Odometry, drone_name: str) -> None:
        if drone_name in self._drones:
            self._drones[drone_name].pose = msg.pose.pose.position

    def _on_battery(self, msg: Bool, drone_name: str) -> None:
        if drone_name not in self._drones:
            return
        was = self._drones[drone_name].battery_ok
        self._drones[drone_name].battery_ok = not bool(msg.data)
        if was and msg.data:
            self.get_logger().warning(
                f'{drone_name}: battery_low — sending RTH'
            )
            # Record the operational state so the auction's health gate
            # excludes this drone independently of the battery_ok
            # latch it shares with the dispatch path.
            self._drones[drone_name].drone_state = DroneState.RETURNING
            self._issue_task(drone_name, TaskAssignment.RTH,
                             waypoints=[], target=None,
                             victim_id=0, priority=3, hover_s=0.0)
            # Hand the returning drone's remaining sector to a healthy peer so
            # its area is still covered. Previously only the DOWN path did this;
            # a battery-RTH drone orphaned its sector (it kept owning waypoints
            # it would never fly). battery_ok is now False, so _reassign_sector
            # excludes it as a receiver.
            self._reassign_sector(self._drones[drone_name])
            self._emit_event(
                'BATTERY_RTH', drone_name=drone_name,
                detail='battery_low fired — RTH dispatched, sector handed off',
                severity=MissionEvent.SEVERITY_WARN,
            )

    def _on_health(self, msg: DroneHealth, drone_name: str) -> None:
        """drone_health_monitor → unrecoverable=True ⇒ mark drone DOWN, drain
        its task, partition remaining waypoints across surviving drones, emit
        DRONE_DOWN. Reactivation never happens, a human must remove the
        drone from the world to clear the flag (matches real ops)."""
        if drone_name not in self._drones:
            return
        d = self._drones[drone_name]
        # Capture battery endurance for the flight-plan feasibility
        # check (NaN when the monitor can't fit a curve yet → leave unknown).
        rem = float(getattr(msg, 'battery_remaining_s', float('nan')))
        if rem == rem and rem >= 0.0:   # not NaN
            self._battery_remaining_s[drone_name] = rem
        if msg.unrecoverable and not d.is_down:
            d.is_down = True
            self.get_logger().error(
                f'{drone_name}: marking DOWN (reason={msg.reason}); '
                f'reassigning {len(d.scan_waypoints) - d.scan_cursor} '
                f'remaining waypoints to surviving peers'
            )
            self._reassign_sector(d)
            self._emit_event(
                'DRONE_DOWN', drone_name=drone_name,
                detail=f'unrecoverable: {msg.reason}',
                severity=MissionEvent.SEVERITY_ERROR,
                position=d.pose,
            )

    def _issue_task_from_outgoing(self, t: OutgoingTask) -> None:
        """Anti-corruption translator: dispatch a
        domain ``OutgoingTask`` (returned by a lifted ``Mission``
        method) through the legacy ROS ``_issue_task`` path. Converts
        the VO's (x, y, z) tuples back into ``geometry_msgs.msg.Point``.
        """
        waypoints = [self._point(*wp) for wp in t.waypoints]
        target = self._point(*t.target) if t.target is not None else None
        self._issue_task(
            t.drone_name, t.task_type,
            waypoints=waypoints, target=target,
            victim_id=t.victim_id, priority=t.priority,
            hover_s=t.hover_seconds,
            confirm_orbit_radius=t.confirm_orbit_radius,
            investigate_radius=t.investigate_radius,
            dwell_s=t.dwell_s,
            investigate_angles=t.investigate_angles,
        )

    def _mirror_state_from_mission(self) -> None:
        """Transitional reverse-mirror (typed → legacy).

        A lifted ``Mission`` method mutates the ``Drone`` / ``Victim`` /
        ``VictimSubMission`` entities + the mission stage. Until the saga
        lift deletes the legacy ``DroneRecord`` / ``VictimRecord`` mirror,
        copy those mutations back onto the records so the still-legacy
        callbacks stay consistent.
        """
        # Drones: scan state, wedge, busy slot.
        for name, d in self._mission.drones.items():
            rec = self._drones.get(name)
            if rec is None:
                continue
            rec.scan_waypoints = [
                self._point(p.x, p.y, p.z) for p in d.scan_waypoints
            ]
            rec.scan_cursor = int(d.scan_cursor)
            rec.busy_with_victim = d.busy_with_victim
            if d.sector_wedge is not None:
                rec.sector_start_rad = d.sector_wedge.start_rad
                rec.sector_end_rad = d.sector_wedge.end_rad
            else:
                rec.sector_start_rad = 0.0
                rec.sector_end_rad = 0.0
        # Victims: stage, assigned drone, cached CONFIRM witness (the
        # witness lives on the sub-mission; mirror it back to
        # VictimRecord.witness_drone for the still-legacy CONFIRM path).
        for vid, v in self._mission.victims.items():
            rec = self._victims.get(vid)
            if rec is None:
                continue
            rec.stage = int(v.stage)
            rec.assigned_drone = v.assigned_drone
            sub = self._mission.sub_missions.get(vid)
            if sub is not None:
                rec.witness_drone = sub.witness_drone
        # Mission stage (INVESTIGATE_DISPATCHED transition in plan()).
        self._set_stage(self._mission.stage)

    def _wire_mission_planner(self) -> None:
        """Set the Mission aggregate's planning
        collaborators + live config from the L2 adapter's state, so
        ``Mission.plan`` / ``plan_for`` use the same strategy + sector
        policy + runtime-tweakable thresholds the legacy path used.
        Called at the head of each dispatch shim (config can change at
        runtime via ``_on_runtime_params``).
        """
        self._mission._allocation_strategy = self._allocation_strategy
        self._mission._sector_owner_policy = _MissionSectorOwnerAdapter(self)
        self._mission.investigate_confidence_floor = (
            self.investigate_confidence_floor
        )
        self._mission.max_concurrent_investigations = (
            self.max_concurrent_investigations
        )
        self._mission.investigate_hover_seconds = self.investigate_hover_s
        self._mission.confirm_hover_seconds = self.confirm_hover_s
        self._mission.confirm_orbit_radius = self.confirm_orbit_r
        # Push the multi-view INVESTIGATE plan config; the angle set
        # keeps the Mission default (4 cardinals) unless re-assigned.
        self._mission.investigate_radius_m = self.investigate_radius_m
        self._mission.investigate_dwell_s = self.investigate_dwell_s

    def _reassign_sector(self, dead: DroneRecord) -> None:
        """Multi-robot recovery: thin L2 shim over
        ``Mission.handle_drone_lost`` (saga lift).

        The decision logic (nearest survivor, contiguous handover,
        wedge absorption, mid-SCAN preemption) now lives in the
        ``Mission`` aggregate. This shim: sync legacy→typed, call the
        lifted method, reverse-mirror the mutated drones back onto the
        legacy records, issue any returned preempting task, and emit
        the SECTOR_REASSIGNED telemetry event.
        """
        # Nothing assigned to this drone (e.g. it was IDLE / investigating, or
        # a full-fleet recall) → nothing to hand off. Check this BEFORE the
        # survivor scan so a no-orphan drone doesn't log a spurious
        # "no survivors" error during a fleet RTH.
        remaining_count = len(dead.scan_waypoints[dead.scan_cursor:])
        if remaining_count == 0:
            return
        survivors = [
            d for d in self._drones.values()
            if not d.is_down and d.battery_ok and d.name != dead.name
        ]
        if not survivors:
            self.get_logger().error(
                f'{dead.name}: NO available survivors — cannot reassign '
                f'{remaining_count} waypoints'
            )
            return

        self._sync_to_mission()
        tasks = self._mission.handle_drone_lost(
            dead.name, self._time.now_sec(),
        )
        self._mirror_state_from_mission()

        receiver_name = tasks[0].drone_name if tasks else '(deferred SCAN)'
        for t in tasks:
            self._issue_task_from_outgoing(t)

        self._emit_event(
            'SECTOR_REASSIGNED', drone_name=dead.name,
            detail=(f'{remaining_count} wps → {receiver_name} '
                    f'(contiguous handover'
                    f'{", preempted in-flight" if tasks else ""})'),
            severity=MissionEvent.SEVERITY_WARN,
        )

    def _on_survey_start(self, msg: Bool) -> None:
        if not msg.data:
            return
        if not self._is_active:
            # /survey/start race fix: the readiness
            # coordinator publishes /survey/start TRANSIENT_LOCAL
            # (latched). If it fires before this node finishes lifecycle
            # activation, the latched sample is delivered to our
            # subscription the instant it is created in on_configure,
            # while _is_active is still False. Don't drop it, record it
            # and run the survey once we activate (on_activate). Without
            # this, a late-activating mission_manager silently never
            # scans (observed in the dockerised sim where activation lags
            # readiness by ~1s).
            self._survey_start_pending = True
            return
        self._begin_survey()

    def _begin_survey(self) -> None:
        """Start the coverage scan if the mission is in a startable
        stage. Shared by the live ``/survey/start`` callback and the
        deferred-start path in ``on_activate``."""
        if self._stage not in (MissionStage.DEPLOYING, MissionStage.ARMING):
            return
        self._mission_start_sec = self._time.now_sec()
        self._begin_scan()

    def _on_operator_goal(self, msg: PointStamped) -> None:
        """Operator-injected investigate goal.

        Mints a synthetic high-confidence ``VictimCandidate`` (cid from
        the reserved >= OPERATOR_GOAL_CID_BASE range, reporter
        'operator') and delegates to ``_on_candidate`` so the goal
        rides the exact auction → INVESTIGATE → CONFIRM saga path a
        detection would. The pre-survey stage gate inside
        ``_on_candidate`` therefore applies to operator goals too,
        a goal clicked before /survey/start is dropped with the same
        diagnostic warning.
        """
        if not self._is_active:
            return
        cid = self._next_operator_cid
        self._next_operator_cid += 1
        synthetic = VictimCandidate()
        synthetic.candidate_id = cid
        synthetic.position.x = float(msg.point.x)
        synthetic.position.y = float(msg.point.y)
        synthetic.position.z = 0.0
        synthetic.confidence = 0.99
        synthetic.confirmed = False
        synthetic.reporting_drones = ['operator']
        self._emit_event(
            'OPERATOR_GOAL',
            detail=f'operator requested investigate at '
                   f'({msg.point.x:.1f}, {msg.point.y:.1f})',
            victim_id=cid, position=synthetic.position,
            severity=MissionEvent.SEVERITY_INFO,
            confidence=0.99,
        )
        self._on_candidate(synthetic)

    def _on_operator_rth(self, msg: String) -> None:
        """Operator-commanded return-home.

        Mirrors ``_on_battery``'s RTH dispatch: flip ``battery_ok``
        (deliberate reuse, it is the system's existing "dispatch no
        further tasks to this drone" gate, enforced by the auction,
        the sector-owner check, and the CONFIRM-witness check alike)
        and issue an RTH task, which the executor's BT turns into
        fly-home + land while ceasing its survey-target stream.
        ``msg.data``: a drone name, or ``'*'``/empty for fleet recall.
        Idempotent: drones already DOWN or already homeward are
        skipped.
        """
        if not self._is_active:
            return
        name = msg.data.strip()
        targets = list(self._drones) if name in ('', '*') else [name]
        for drone_name in targets:
            self._force_rth(
                drone_name,
                event_name='OPERATOR_RTH',
                detail='operator recall — RTH dispatched',
                severity=MissionEvent.SEVERITY_WARN,
            )

    def _force_rth(self, drone_name: str, *, event_name: str, detail: str,
                   severity: int) -> bool:
        """Commit a drone to return-to-home and exclude it from tasking.

        The single dispatch path shared by operator recall, battery-low RTH
        and no-fly-zone breach: flip ``battery_ok`` (the system-wide
        "no further tasks" gate enforced by the auction / sector-owner /
        witness checks), record ``drone_state = RETURNING`` for the
        health gate, issue an RTH task and emit ``event_name``. Idempotent,
        a drone already DOWN or already homeward is skipped. Returns True
        iff an RTH was dispatched."""
        d = self._drones.get(drone_name)
        if d is None or d.is_down:
            return False
        if d.current_task_type == TaskAssignment.RTH:
            return False   # already going home
        d.battery_ok = False
        d.drone_state = DroneState.RETURNING
        self._issue_task(drone_name, TaskAssignment.RTH,
                         waypoints=[], target=None,
                         victim_id=0, priority=3, hover_s=0.0)
        # Hand the recalled drone's remaining sector to a healthy peer (operator
        # recall / zone breach RTH), same takeover the DOWN path gets. battery_ok
        # is now False, so _reassign_sector won't pick this drone as the receiver.
        self._reassign_sector(d)
        self._emit_event(
            event_name, drone_name=drone_name,
            detail=detail, severity=severity,
        )
        return True

    def _load_no_fly_zones(self, config_file: str):
        """Load no-fly zones from the given path, else the bringup default.

        Mirrors zone_manager's default-location fallback so the planner and
        the detector node enforce the same zones. Returns an empty list when
        nothing loads (zones not enforced, the prior behaviour)."""
        if config_file:
            return load_no_fly_zones(config_file)
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('drone_rescue_bringup')
            return load_no_fly_zones(f'{pkg_dir}/config/no_fly_zones.yaml')
        except Exception as e:
            self.get_logger().warn(f'No-fly-zone default config unavailable: {e}')
            return []

    def _filter_scan_waypoints(self, drone_name: str, wps):
        """Drop scan waypoints that fall inside a no-fly zone."""
        if not self._no_fly_zones:
            return wps
        kept, removed = filter_waypoints(wps, self._no_fly_zones,
                                         self._zone_states)
        if removed:
            self.get_logger().warn(
                f'{drone_name}: dropped {len(removed)} scan waypoint(s) '
                f'inside no-fly zone(s)'
            )
        return kept

    def _on_zone_violation(self, msg: String) -> None:
        """React to a ``zone_manager`` no-fly-zone breach.

        ``zone_manager`` publishes ``/zones/violation`` while a drone is
        actually *inside* a zone. Previously the FIRST sample force-RTH'd the
        drone, so a one-tick boundary brush during normal scanning permanently
        aborted its mission (e.g. drone2's spiral skims the buffered
        gas_leak_zone edge very early and never reached any victim). But the
        executor already climbs to escape altitude and the planner already
        drops in-zone waypoints, so a transient brush should be *avoided*, not
        fatal. Only force RTH when a drone is SUSTAINEDLY inside a zone (stuck,
        i.e. avoidance failed). The alert text carries the drone name."""
        if not self._is_active:
            return
        drone_name = drone_name_from_violation(msg.data)
        if drone_name is None:
            return

        # Debounce: accumulate consecutive in-zone samples; reset the streak
        # if the drone has been out long enough that samples stopped arriving.
        now = self._time.now_sec()
        if now - self._zone_breach_last_t.get(drone_name, -1e9) > 1.0:
            self._zone_breach_streak[drone_name] = 0
        self._zone_breach_last_t[drone_name] = now
        streak = self._zone_breach_streak.get(drone_name, 0) + 1
        self._zone_breach_streak[drone_name] = streak

        # ~3 s of continuous in-zone samples (zone_manager checks at ~10 Hz)
        # before we treat it as "stuck" and abort.
        if streak < 30:
            self.get_logger().warn(
                f'{drone_name}: no-fly-zone brush ({streak}) — avoiding, '
                f'not aborting ({msg.data})'
            )
            return

        if self._force_rth(
            drone_name,
            event_name='ZONE_VIOLATION_RTH',
            detail=f'sustained no-fly-zone breach — RTH dispatched ({msg.data})',
            severity=MissionEvent.SEVERITY_ERROR,
        ):
            self._zone_breach_streak[drone_name] = 0
            self.get_logger().error(
                f'{drone_name}: stuck in no-fly-zone — forcing RTH'
            )

    def _on_candidate(self, msg: VictimCandidate) -> None:
        if not self._is_active:
            return
        if self._stage in (MissionStage.COMPLETE, MissionStage.ABORTED):
            return
        # The saga must NOT react to
        # candidates until the deliberative coverage plan has been
        # issued. Without this gate, a missed /survey/start (e.g. the
        # /clock-bridge race that suppresses readiness_coordinator)
        # silently degenerates the system into a pure-reactive
        # victim-chasing swarm with no spiral coverage, bypassing
        # the L3 deliberative layer entirely and producing
        # trajectories that contradict the thesis's 3T claim.
        # Pre-survey candidates are dropped (logged once per cid for
        # diagnostic visibility); a candidate that re-arrives after
        # SCANNING begins follows the normal path below.
        if self._stage in (
            MissionStage.INIT,
            MissionStage.ARMING,
            MissionStage.DEPLOYING,
        ):
            cid = int(msg.candidate_id)
            if cid not in self._pre_survey_dropped_cids:
                self.get_logger().warning(
                    f'CANDIDATE #{cid} dropped: stage={self._stage.name}, '
                    f'deliberative coverage plan not yet issued '
                    f'(awaiting /survey/start)'
                )
                # Tombstone the cid so the same candidate doesn't
                # spam-log on every re-publish (detection_filter
                # republishes at publish_rate_hz). The tombstone is
                # cleared by ``_begin_scan`` so a re-arrival post-
                # survey-start still enters the saga normally.
                self._pre_survey_dropped_cids.add(cid)
            return
        cid = int(msg.candidate_id)
        record = self._victims.get(cid)
        if record is None:
            record = VictimRecord(
                candidate_id=cid, position=msg.position,
                confidence=float(msg.confidence),
                last_update_sec=self._time.now_sec(),
            )
            self._victims[cid] = record
            self.get_logger().info(
                f'CANDIDATE #{cid} at ({msg.position.x:.1f}, {msg.position.y:.1f}) '
                f'conf={msg.confidence:.2f} confirmed={msg.confirmed}'
            )
            self._emit_event(
                'CANDIDATE_DETECTED',
                detail=f'conf={msg.confidence:.2f}, '
                       f'reporters={list(msg.reporting_drones)}',
                victim_id=cid, position=msg.position,
                severity=MissionEvent.SEVERITY_INFO,
                confidence=float(msg.confidence),
            )
        else:
            # Update: Bayesian fusion already done upstream
            record.position = msg.position
            record.confidence = float(msg.confidence)
            record.last_update_sec = self._time.now_sec()

        # If already confirmed by detection_filter, skip extra CONFIRM step.
        if msg.confirmed and record.stage == VictimStage.DETECTED:
            record.stage = VictimStateMachine.transition(
                record.stage, TransitionEvent.CONFIRMED,
            )
            self.get_logger().info(f'  → auto-CONFIRMED by detection_filter')
            self._emit_event(
                'VICTIM_CONFIRMED',
                detail='auto-confirmed by detection_filter',
                victim_id=cid, position=msg.position,
                severity=MissionEvent.SEVERITY_INFO,
            )
            # Same saga-confirmation channel.
            # Redundant with VictimCandidate.confirmed in this branch
            # (the visualizer already knows), but emitting from both
            # confirmation paths keeps the publisher's invariant
            # uniform: every confirmed victim has its cluster_id on the
            # topic exactly once.
            self._publish_saga_confirmed(cid)
            return

        # Try to dispatch an INVESTIGATE (auction). Strategies that expose a
        # batch assign() (hungarian) jointly assign every queued candidate;
        # greedy / round-robin keep the unchanged per-candidate path.
        if record.stage == VictimStage.DETECTED:
            if isinstance(self._allocation_strategy, BatchAllocationBidder):
                self._drain_investigate_batch()
            else:
                self._dispatch_investigate(record)

    def _on_task_status(self, msg: TaskStatus) -> None:
        if msg.drone_name not in self._drones:
            return
        d = self._drones[msg.drone_name]
        # Bump watchdog clock on ANY status from this drone: proof of life,
        # regardless of which task we hear about.
        d.clock = replace(d.clock, last_status_t=self._time.now_sec())

        if msg.task_id != d.current_task_id:
            # Stale status from a preempted task, ignore.
            return

        # IN_PROGRESS for SCAN carries the executor's current waypoint
        # index in detail="wp=N". Update scan_cursor so a later
        # _reassign_sector dispatches only the un-visited tail instead of
        # restarting the survivor at waypoint 0.
        if (msg.status == TaskStatus.IN_PROGRESS
                and d.current_task_type == TaskAssignment.SCAN_WAYPOINTS
                and msg.detail.startswith('wp=')):
            try:
                idx = int(msg.detail.split('=', 1)[1])
                # Cursor is offset within the FULL scan_waypoints list, but the
                # executor's local index counts from the start of its current
                # dispatch. We track dispatch_offset so we can reconstruct the
                # absolute position.
                dispatched_from = d.clock.last_dispatch_offset
                d.scan_cursor = max(d.scan_cursor, dispatched_from + idx)
            except ValueError:
                pass
            return

        if msg.status == TaskStatus.COMPLETED:
            self.get_logger().info(
                f'{msg.drone_name} COMPLETED task {msg.task_id} '
                f'({task_type_label(d.current_task_type)})'
            )
            self._on_task_completed(d)
        elif msg.status == TaskStatus.FAILED:
            self.get_logger().warning(
                f'{msg.drone_name} FAILED task {msg.task_id}: {msg.detail}'
            )
            self._on_task_failed(d)
        # ACCEPTED / IN_PROGRESS / PREEMPTED, no action

    # saga steps
    def _begin_scan(self) -> None:
        """Plan coverage via the configured strategy and dispatch one
        SCAN_WAYPOINTS task per drone.

        The CoverageStrategy is responsible for partitioning the search area:
        concentric-arc strategies split angularly, parallel-track splits
        into horizontal strips, and so on. The mission manager hands it
        the configuration and consumes the per-drone waypoint lists.
        """
        # Clear the pre-survey
        # tombstone set; cluster_ids that re-arrive after SCANNING
        # begins should follow the normal saga path, not stay
        # silently rejected.
        self._pre_survey_dropped_cids.clear()
        planner_input = PlannerInput(
            mission_center=self.mission_center,
            radius=self.mission_radius,
            inner_radius=self.inner_radius,
            n_drones=len(self.drone_names),
            footprint_m=self.camera_footprint_m,
            overlap=self.coverage_overlap,
            seed=self.seed,
        )
        # Consume the typed CoveragePlan + ScanPlan
        # returned by ``plan_v2``. Each ScanPlan carries its waypoints
        # tuple + optional SectorWedge; the inline angular-wedge
        # derivation collapses into reading ``plan.wedge`` per drone.
        coverage_plan = self._coverage_strategy.plan_v2(planner_input)
        per_drone_plans = coverage_plan.per_drone
        self._sectors_total = len(per_drone_plans)
        self.get_logger().info(
            f'Coverage strategy: {self._coverage_strategy.name} '
            f'({len(per_drone_plans)} regions, '
            f'avg {sum(p.length for p in per_drone_plans) // max(len(per_drone_plans), 1)} '
            f'waypoints/drone)'
        )

        for drone_name, plan in zip(self.drone_names, per_drone_plans):
            wps = [
                # survey_altitude is above-ground-level; add terrain height
                # so the drone holds constant AGL over slopes (flat model -> +0).
                self._point(
                    x, y,
                    self.survey_altitude + self._elevation.elevation_at(x, y),
                )
                for x, y in plan.waypoints
            ]
            # Drop any waypoint that falls inside a no-fly zone so the
            # planner never sends a drone into one (was: empty-tuple WorldModel,
            # nothing filtered).
            wps = self._filter_scan_waypoints(drone_name, wps)
            self._drones[drone_name].scan_waypoints = wps
            self._drones[drone_name].scan_cursor = 0
            # Wedge carrier: the typed ScanPlan.wedge replaces the
            # inline ``partition_kind == ANGULAR`` derivation. For
            # strategies that don't partition angularly, wedge is None
            # and the (start_rad, end_rad) pair stays at (0, 0) so
            # ``_sector_owner_for()`` returns None for every bearing.
            wedge = plan.wedge
            if wedge is not None:
                self._drones[drone_name].sector_start_rad = wedge.start_rad
                self._drones[drone_name].sector_end_rad = wedge.end_rad
            else:
                self._drones[drone_name].sector_start_rad = 0.0
                self._drones[drone_name].sector_end_rad = 0.0
            self._issue_task(drone_name, TaskAssignment.SCAN_WAYPOINTS,
                             waypoints=wps, target=None,
                             victim_id=0, priority=1, hover_s=0.0)

        self._set_stage(MissionStateMachine.transition(
            self._stage, TransitionEvent.SURVEY_STARTED,
        ))
        # Keep the typed
        # Mission aggregate's stage mirror in sync with the L2 adapter.
        # Without this, ``_sync_to_mission`` later reads
        # ``self._mission.stage`` (still INIT, the aggregate's
        # ``begin_scan`` was bypassed by the adapter's own path) and
        # clobbers ``self._stage`` back to INIT, which then trips the
        # pre-survey gate and starts rejecting every candidate
        # post-SCANNING. The fix is a one-line forward-sync; the
        # broader unification of L2 ``_begin_scan`` and L3
        # ``Mission.begin_scan`` is a separate refactor.
        self._mission.stage = self._stage
        self.get_logger().info(
            f'SCANNING: {self._sectors_total} sectors assigned, '
            f'~{len(self._drones[self.drone_names[0]].scan_waypoints)} waypoints/drone'
        )
        self._emit_event(
            'SCANNING_STARTED',
            detail=f'{self._coverage_strategy.name} pattern, '
                   f'{self._sectors_total} sectors, '
                   f'{len(self.drone_names)} drones',
            severity=MissionEvent.SEVERITY_INFO,
        )

    def _dispatch_investigate(self, victim: VictimRecord) -> None:
        """SAR-disciplined auction.

        Real SAR procedure: each search asset OWNS a sector and stays in it.
        Cross-sector help only happens by explicit handover when the owner is
        unavailable. We enforce this here:

          1. Confidence floor: ignore low-conf candidates (they stay
             DETECTED and may be re-attempted later if the cluster grows).
          2. Concurrency cap: at most N drones may leave their arcs at a
             time; the rest keep scanning.
          3. Sector ownership: auction the sector-owning drone first.
             Only fall back to a foreign drone if the owner is DOWN, on
             low-battery RTH, or already busy with a different victim.

        This stops the "drone1 leaves NE to investigate a candidate in SW"
        pathology the operator was seeing.
        """
        # Thin L2 shim over ``Mission.plan_for`` (the
        # single-victim dispatch). The confidence floor, concurrency
        # cap, sector-owner first refusal, and auction fallback all
        # live in the aggregate now. Sync legacy→typed, dispatch, then
        # reverse-mirror + publish + emit telemetry.
        self._wire_mission_planner()
        self._sync_to_mission()
        v = self._mission.victims.get(victim.candidate_id)
        if v is None:
            return
        world = self._mission.snapshot_world(self._time.now_sec())
        tasks = self._mission.plan_for(v, world)
        self._mirror_state_from_mission()
        self._dispatch_outgoing_investigates(tasks)

    def _drain_investigate_batch(self) -> None:
        """Joint INVESTIGATE dispatch: thin L2 shim over
        ``Mission.plan`` (the batch pass).

        The two-pass discipline (sector-owner first
        refusal, then joint cost-minimising assignment of the
        owner-unavailable pool) lives in the aggregate. Only invoked
        for batch-capable strategies (the ``_tick`` ``isinstance``
        guard), matching the legacy behaviour exactly.
        """
        self._wire_mission_planner()
        self._sync_to_mission()
        world = self._mission.snapshot_world(self._time.now_sec())
        tasks = self._mission.plan(world)
        self._mirror_state_from_mission()
        self._dispatch_outgoing_investigates(tasks)

    def _dispatch_outgoing_investigates(self, tasks) -> None:
        """Publish each INVESTIGATE OutgoingTask the
        planner returned + emit its INVESTIGATE_DISPATCHED telemetry
        event. (Event emission stays in the L2 adapter until the
        EventPort is wired onto the aggregate.)
        """
        for t in tasks:
            self._issue_task_from_outgoing(t)
            self._emit_event(
                'INVESTIGATE_DISPATCHED', drone_name=t.drone_name,
                detail=f'auctioned to {t.drone_name}',
                victim_id=t.victim_id,
                position=self._point(*t.target) if t.target else None,
                severity=MissionEvent.SEVERITY_INFO,
            )

    def _on_task_completed(self, d: DroneRecord) -> None:
        """Thin L2 shim over
        ``Mission.on_task_completed``. The 3-branch completion logic
        (cross-drone CONFIRM handoff, victim confirmation, scan-resume
        / IDLE) lives in the aggregate. Sync legacy→typed, run the
        planner step, reverse-mirror, publish, emit telemetry.
        """
        # Capture the pre-call task type + victim before the aggregate
        # mutates the busy slot, the telemetry reconstruction needs them.
        ttype = d.current_task_type
        victim_id_before = d.busy_with_victim
        completed = OutgoingTask(
            drone_name=d.name, task_type=int(ttype),
            waypoints=(), target=None,
            victim_id=victim_id_before or 0, priority=0, hover_seconds=0.0,
        )
        self._wire_mission_planner()
        self._sync_to_mission()
        world = self._mission.snapshot_world(self._time.now_sec())
        tasks = self._mission.on_task_completed(world, completed)
        self._mirror_state_from_mission()
        for t in tasks:
            self._issue_task_from_outgoing(t)
        self._emit_completion_events(ttype, victim_id_before, tasks)

    def _emit_completion_events(self, ttype, victim_id_before, tasks) -> None:
        """Reconstruct the legacy completion telemetry
        from the pre-call task type + the planner's returned tasks.
        (Event emission stays in the L2 adapter.)
        """
        # VICTIM_CONFIRMED, a CONFIRM task just completed.
        if ttype == TaskAssignment.CONFIRM and victim_id_before is not None:
            v = self._victims.get(victim_id_before)
            if v is not None:
                self._emit_event(
                    'VICTIM_CONFIRMED', drone_name=v.assigned_drone or '',
                    detail='after CONFIRM orbit',
                    victim_id=victim_id_before, position=v.position,
                    severity=MissionEvent.SEVERITY_INFO,
                )
                # Publish to the saga channel so
                # the visualizer can paint the sphere green even when
                # ``VictimCandidate.confirmed`` never flipped (the
                # multi-view fusion gate requires ≥2 distinct reporting
                # drones, which sector-scanning rarely satisfies).
                self._publish_saga_confirmed(victim_id_before)
        for t in tasks:
            if t.task_type == TaskAssignment.CONFIRM:
                self._emit_event(
                    'CONFIRM_DISPATCHED', drone_name=t.drone_name,
                    detail=f'orbit-confirm by {t.drone_name}',
                    victim_id=t.victim_id,
                    position=self._point(*t.target) if t.target else None,
                    severity=MissionEvent.SEVERITY_INFO,
                )
            elif t.task_type == TaskAssignment.IDLE:
                self._emit_event(
                    'DRONE_SECTOR_COMPLETE', drone_name=t.drone_name,
                    detail='all assigned waypoints visited',
                    severity=MissionEvent.SEVERITY_INFO,
                )

    def _set_stage(self, new_stage: MissionStage) -> None:
        """Single mutation point for
        ``self._stage``. Emits a ``STAGE_TRANSITION`` event whenever the
        stage actually changes, so mission_recorder can journal the
        deliberative-layer timeline (INIT → ARMING → DEPLOYING →
        SCANNING → INVESTIGATING → COMPLETE/ABORTED) to JSONL. A run
        whose JSONL has no SCANNING-stage entry is then immediately
        recognisable in post-run analysis as one where the deliberative
        layer never engaged.

        Idempotent: re-assigning the same stage is a quiet no-op (the
        mirror-from-Mission path in ``_sync_to_mission`` calls this on
        every tick).
        """
        if new_stage == self._stage:
            return
        old_stage = self._stage
        self._stage = new_stage
        # The event_port is only wired in on_configure; emit only
        # after activation AND only when the port object exists. The
        # double ``getattr`` guard handles two test-setup patterns:
        # ``_bare_mm`` (object.__new__ with a stripped attribute set
        # that doesn't include _event_port) and ``SimpleNamespace``
        # binding (test_pre_survey_candidate_gate, no _is_active set).
        # The stage mutation itself is the contract; the event is
        # the observability layer on top, it must not be load-
        # bearing for the lifecycle transitions to work.
        if (getattr(self, '_is_active', False)
                and getattr(self, '_event_port', None) is not None):
            self._emit_event(
                'STAGE_TRANSITION',
                detail=f'{old_stage.name} → {new_stage.name}',
                severity=MissionEvent.SEVERITY_INFO,
            )

    def _publish_saga_confirmed(self, candidate_id: int) -> None:
        """Emit the cluster_id of a victim whose
        confirmation is settled (either by saga CONFIRM completion or by
        upstream multi-view auto-confirm). Visualizers subscribe to
        ``/victims/saga_confirmed`` and OR this signal with
        ``VictimCandidate.confirmed`` when deciding marker colour. The
        publisher is created lazily in on_configure; in unit-test setups
        that bypass lifecycle, ``self._victims_confirmed_pub`` may be
        None, in which case this is a no-op.
        """
        pub = self._victims_confirmed_pub
        if pub is None:
            return
        pub.publish(UInt32(data=int(candidate_id)))

    def _on_task_failed(self, d: DroneRecord) -> None:
        """Thin L2 shim over ``Mission.replan``. The
        saga compensation (free victim → DETECTED, resume SCAN) lives
        in the aggregate. Sync legacy→typed, replan, reverse-mirror,
        publish.
        """
        failed = OutgoingTask(
            drone_name=d.name, task_type=int(d.current_task_type),
            waypoints=(), target=None,
            victim_id=d.busy_with_victim or 0, priority=0, hover_seconds=0.0,
        )
        self._sync_to_mission()
        world = self._mission.snapshot_world(self._time.now_sec())
        tasks = self._mission.replan(world, failed)
        self._mirror_state_from_mission()
        for t in tasks:
            self._issue_task_from_outgoing(t)

    # sector ownership
    def _sector_owner_for(self, p: Point) -> Optional[str]:
        """Thin delegation to lib.sector_geometry.

        Kept as a method here so the ROS callbacks (`_on_candidate`
        etc.) read naturally; the math itself is pure-Python and
        independently testable in `lib/sector_geometry.py`.
        """
        return _sector_owner_for_helper(
            p, self._drones.values(), mission_center=self.mission_center,
        )

    # _still_eligible_witness lifted into
    # Mission._still_eligible_witness (the CONFIRM-handoff logic is now
    # in the aggregate). The legacy method was removed here.

    # auction
    def _auction(
        self, target: Point, priority: int,
        exclude: Optional[set] = None,
    ) -> Optional[str]:
        """Delegate to the selected ``AllocationStrategy`` (default
        ``greedy_auction``, the Gerkey & Matarić IJRR 2004 single-item
        utility auction). The strategy is chosen via the
        ``allocation_strategy`` parameter; see ``lib/allocation.py``.
        """
        return self._allocation_strategy.bid(target, priority, exclude)

    # task issue
    def _issue_task(self, drone_name: str, task_type: int,
                    waypoints: List[Point], target: Optional[Point],
                    victim_id: int, priority: int, hover_s: float,
                    confirm_orbit_radius: float = 0.0,
                    investigate_radius: float = 0.0, dwell_s: float = 0.0,
                    investigate_angles: Sequence[float] = ()) -> None:
        d = self._drones[drone_name]
        tid = self._next_task_id
        self._next_task_id += 1

        msg = TaskAssignment()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.drone_name = drone_name
        msg.task_id = tid
        msg.task_type = task_type
        msg.waypoints = list(waypoints) if waypoints else []
        msg.target_point = target if target is not None else Point()
        msg.victim_id = victim_id
        msg.priority = priority
        msg.hover_seconds = float(hover_s)
        msg.confirm_orbit_radius = float(confirm_orbit_radius)
        # Multi-view INVESTIGATE plan (empty/0.0 ⇒ executor defaults).
        msg.investigate_radius = float(investigate_radius)
        msg.dwell_s = float(dwell_s)
        msg.investigate_angles = [float(a) for a in investigate_angles]

        pub = self._task_pubs.get(drone_name)
        if pub is not None:
            pub.publish(msg)
        d.current_task_id = tid
        d.current_task_type = task_type
        now_sec = self._time.now_sec()
        # Reset watchdog clock, task just dispatched, treat as proof of life.
        # Remember where in scan_waypoints this dispatch starts so we can
        # convert the executor's local IN_PROGRESS index back to an absolute
        # cursor in scan_waypoints.
        new_offset = (
            d.scan_cursor if task_type == TaskAssignment.SCAN_WAYPOINTS
            else d.clock.last_dispatch_offset
        )
        d.clock = replace(
            d.clock,
            last_status_t=now_sec,
            task_dispatched_t=now_sec,
            last_dispatch_offset=new_offset,
        )
        self.get_logger().info(
            f'  → {drone_name}: task #{tid} {task_type_label(task_type)} '
            f'(victim={victim_id}, prio={priority}, wps={len(msg.waypoints)})'
        )

    # helpers
    def _point(self, x: float, y: float, z: float) -> Point:
        p = Point(); p.x = x; p.y = y; p.z = z
        return p

    # tick
    def _tick(self) -> None:
        """Thin L2 shim over ``Mission.tick``. The
        4-sub-task loop (candidate decay, re-attempt dispatch, task
        watchdog, mission completion) lives in the aggregate. Wire the
        planner + emit callable + tick config + mission_start, sync,
        run the tick, reverse-mirror, publish dispatches + state.
        """
        if not self._is_active:
            return
        self._wire_mission_planner()
        self._mission.reject_age_seconds = self.reject_age_s
        self._mission.task_status_timeout_seconds = self.task_status_timeout_s
        self._mission.mission_timeout_seconds = self.mission_timeout_s
        self._mission.mission_start_sec = self._mission_start_sec
        self._mission._published_complete = self._published_complete
        self._mission._strategy_is_batch = isinstance(
            self._allocation_strategy, BatchAllocationBidder,
        )
        self._mission._emit_event = self._emit_event
        self._sync_to_mission()

        tasks = self._mission.tick(self._time.now_sec())

        self._mirror_state_from_mission()
        self._published_complete = self._mission._published_complete
        for t in tasks:
            self._issue_task_from_outgoing(t)

        # 4. Publish global state
        self._publish_state()

    def _sync_to_mission(self) -> None:
        """One-direction mirror of legacy
        ``_drones`` / ``_victims`` dicts into the typed Mission
        aggregate (and its parallel sub_missions). Called once per
        tick before the typed read queries fire. Single-direction
        mirror (legacy → typed) keeps the sync surface tiny; the
        eventual saga lift will reverse the
        direction and retire the legacy records.
        """
        # Drone/Victim/Position/VictimSubMission
        # imports hoisted to module top so ImportError fails at module
        # load, not first _sync_to_mission call.
        # Drones: copy the runtime-relevant fields onto Drone entity.
        for name, rec in self._drones.items():
            pose = (
                Position(rec.pose.x, rec.pose.y, rec.pose.z)
                if rec.pose is not None else None
            )
            d = self._mission.drones.get(name)
            if d is None:
                d = Drone(name=name)
                self._mission.drones[name] = d
            d.pose = pose
            d.battery_ok = bool(rec.battery_ok)
            d.is_down = bool(rec.is_down)
            d.current_task_id = int(rec.current_task_id)
            d.current_task_type = int(rec.current_task_type)
            d.busy_with_victim = rec.busy_with_victim
            d.clock = rec.clock
            # Mirror the saga-relevant scan + wedge
            # state so Mission.handle_drone_lost can operate on
            # the typed entity. Translate Point→Position.
            d.scan_waypoints = [
                Position(p.x, p.y, p.z) for p in rec.scan_waypoints
            ]
            d.scan_cursor = int(rec.scan_cursor)
            if rec.sector_start_rad != rec.sector_end_rad:
                d.sector_wedge = SectorWedge(
                    rec.sector_start_rad, rec.sector_end_rad,
                )
            else:
                d.sector_wedge = None

        # Victims: copy stage/position; sub-mission stage tracks the
        # victim stage for the typed Mission.confirmed_count() query.
        for vid, rec in self._victims.items():
            pos = Position(rec.position.x, rec.position.y, rec.position.z)
            v = self._mission.victims.get(vid)
            if v is None:
                v = Victim(candidate_id=vid, position=pos, confidence=rec.confidence)
                self._mission.victims[vid] = v
            v.position = pos
            v.confidence = float(rec.confidence)
            v.stage = int(rec.stage)
            # Mirror age tracking for Mission.tick.
            v.last_update_sec = float(rec.last_update_sec)
            sm = self._mission.sub_missions.get(vid)
            if sm is None:
                sm = VictimSubMission(victim=v)
                self._mission.sub_missions[vid] = sm
            sm.victim = v
            sm.stage = int(rec.stage)
            # Mirror the runner-up witness cache
            # onto the saga aggregate so Mission.on_task_completed
            # reads it from there.
            sm.witness_drone = rec.witness_drone

    def _victims_confirmed_count(self) -> int:
        """Route through Mission.confirmed_count()."""
        self._sync_to_mission()
        return self._mission.confirmed_count()

    def _emit_event(self, event_type: str, drone_name: str = '',
                    detail: str = '', victim_id: int = 0,
                    position: Optional[Point] = None,
                    severity: int = 0,
                    confidence: float = 0.0) -> None:
        """Facade: routes legacy stringly-typed kwargs through the
        EventPort. The 18 call sites keep their
        existing signature; the wire-format build moves into the
        adapter. Tests substitute an InMemoryEventCapture for
        ``self._event_port`` and assert on the typed variants.

        Conversion of the 18 call sites to typed variant constructors
        (``self._event_port.emit(VictimConfirmed(...))``) is mechanical
        follow-up; the architectural port is in place from this
        change.
        """
        if self._event_port is None:
            return
        variant = _build_event_variant(
            event_type=event_type, drone_name=drone_name,
            detail=detail, victim_id=victim_id,
            position=position, severity=severity,
            confidence=confidence,
        )
        self._event_port.emit(variant)

    def _assess_drone_feasibility(self, d):
        """Flight-plan feasibility for drone ``d``, or None when not
        assessable: no remaining scan plan, no pose, or battery endurance not
        yet known from the health stream."""
        endurance = self._battery_remaining_s.get(d.name)
        if endurance is None or d.pose is None or not d.scan_waypoints:
            return None
        remaining = d.scan_waypoints[d.scan_cursor:]
        if not remaining:
            return None
        plan_m = remaining_plan_length((d.pose.x, d.pose.y), remaining)
        cx, cy = self.mission_center
        home_m = ((d.pose.x - cx) ** 2 + (d.pose.y - cy) ** 2) ** 0.5
        return assess_feasibility(
            drone_name=d.name, remaining_plan_m=plan_m, return_home_m=home_m,
            speed_mps=self._survey_speed_mps, endurance_s=endurance,
            reserve_s=self._feasibility_reserve_s,
        )

    def _publish_state(self) -> None:
        if self._mission_state_pub is None:
            return
        msg = MissionState()
        # Message stamp is a ROS transport
        # concern: it must use the rclpy clock so the bag-replay
        # timestamps and the sim-time/wall-clock domain match the
        # other publishers on the topic. The ``_time`` port is the
        # elapsed-second arithmetic clock (WatchdogClock, timeouts);
        # those two clocks intentionally diverge.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = int(self._stage)
        msg.sectors_total = self._sectors_total
        # We don't yet measure per-sector progress directly; use #drones whose
        # cursor reached end of waypoints as a proxy.
        msg.sectors_completed = sum(
            1 for d in self._drones.values()
            if d.scan_waypoints and d.scan_cursor >= len(d.scan_waypoints)
        )
        msg.victims_found = sum(
            1 for v in self._victims.values()
            if v.stage in (VictimStage.DETECTED, VictimStage.INVESTIGATING, VictimStage.CONFIRMED)
        )
        msg.victims_confirmed = self._victims_confirmed_count()
        summary = []
        for d in self._drones.values():
            label = task_type_label(d.current_task_type)
            if d.scan_waypoints:
                label += f'({d.scan_cursor}/{len(d.scan_waypoints)})'
            if d.busy_with_victim is not None:
                label += f'[v{d.busy_with_victim}]'
            # Flight-plan feasibility go/no-go + margin, folded into the
            # per-drone summary so the operator sees when a drone can no longer
            # finish its plan and return home on remaining battery.
            feas = self._assess_drone_feasibility(d)
            if feas is not None:
                label += (' NO-GO' if not feas.feasible else ' GO')
                label += f'({feas.margin_s:+.0f}s)'
            self._note_feasibility_flip(d.name, feas)
            summary.append(f'{d.name}:{label}')
        msg.active_tasks_summary = summary
        self._mission_state_pub.publish(msg)

    def _note_feasibility_flip(self, name: str, feas) -> None:
        """Emit a one-shot warning when a drone's plan flips to infeasible.
        Latched so the event fires on the transition, not every tick."""
        if feas is None:
            return
        was = self._infeasible_latch.get(name, False)
        now = not feas.feasible
        if now and not was:
            self._emit_event(
                'FLIGHT_PLAN_INFEASIBLE', drone_name=name,
                detail=(f'remaining plan + return exceeds endurance by '
                        f'{-feas.margin_s:.0f}s'),
                severity=MissionEvent.SEVERITY_WARN,
            )
        self._infeasible_latch[name] = now


def _build_event_variant(event_type: str, drone_name: str = '',
                         detail: str = '', victim_id: int = 0,
                         position: Optional[Point] = None,
                         severity: int = 0,
                         confidence: float = 0.0):
    """Thin shim over `lib.domain.events.build_variant`.

    The legacy 13-branch if/elif chain duplicated `events._DECODER`
    and risked drifting from the canonical sum-type. `build_variant`
    is the dispatch table; kwargs that don't fit a variant's fields
    are silently dropped (so passing `confidence=` to MissionComplete
    no longer raises).
    """
    from drone_rescue_coordination.lib.domain.events import build_variant
    return build_variant(
        event_type,
        severity=severity,
        raw_detail=str(detail),
        drone_name=str(drone_name),
        victim_id=victim_id,
        position=position,
        confidence=float(confidence),
    )


def main(args=None):
    rclpy.init(args=args)
    from rclpy.executors import MultiThreadedExecutor
    # Single-point-of-construction. Tests still
    # build MissionManager(composition=...) explicitly.
    node = bind_composition(MissionManager())
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
