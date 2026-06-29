#!/usr/bin/env python3
"""
Lifecycle Manager Node: 3T Architecture, Executive Layer (L2).

Top-level executive-layer supervisor per the slides' 3T taxonomy
(Marcelletti, "Autonomous and Collaborative Robotics", A.Y. 2025/26):

    Slides p. 38, Executive Layer: "Interface between behavioural
    and planning layers, translates high-level plans into low-level
    invocations also taking care of monitoring and handling
    exceptions."

Responsibilities (each an executive-layer concern):
- Startup / shutdown sequencing of L1 nodes (via
  ``lib/lifecycle/startup_sequencer.py``).
- Heartbeat monitoring + recovery policy dispatch (via
  ``lib/lifecycle/watchdog.py`` + ``lib/lifecycle/recovery_policy.py``).
- System-mode FSM (NORMAL / DEGRADED / SAFE) via
  ``lib/domain/system_mode_machine.py``.
- ``ExecutiveSupervisor`` Protocol fulfilment (escalation to L3
  planner, see ``lib/ports/executive_supervisor.py``).

Orchestrates lifecycle transitions for multi-drone survey system.
Ensures proper startup order:
1. pheromone_server (shared resource)
2. All drone controllers (parallel)
3. All surveyors (parallel, after controllers)

Shutdown reverses this order.
"""

import signal
import time
from enum import Enum, IntEnum, auto
from drone_rescue_coordination.lib.domain.fleet import default_drone_names_list
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray
import diagnostic_updater
import diagnostic_msgs.msg
from drone_rescue_msgs.msg import SystemMode
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


class SystemModeEnum(Enum):
    """Python enum for system operational modes."""
    NORMAL = 0
    DEGRADED = 1
    SAFE = 2


class ModeManager:
    """
    Manages system operational mode state machine.

    Transitions:
    - NORMAL -> DEGRADED: Persistent warnings (>30s) OR multiple drone warnings (2+)
    - DEGRADED -> SAFE: Any ERROR status
    - DEGRADED -> NORMAL: All warnings clear (auto-recovery)
    """

    def __init__(self, lifecycle_manager, *, clock_fn=None):
        """``clock_fn``: optional ``Callable[[], float]`` returning the
        current monotonic seconds. The HeartbeatTracker pattern extended
        to the mode-transition clock so the 30-second persistent-warning
        logic is unit-testable with a FakeClock. Default: read from
        the LifecycleManager's rclpy clock."""
        self.node = lifecycle_manager
        self._clock_fn = clock_fn or (
            lambda: lifecycle_manager.get_clock().now().nanoseconds / 1e9
        )
        self.current_mode = SystemModeEnum.NORMAL
        self.mode_start_time = self._clock_fn()
        self.last_transition_reason = "System initialized"

        # Warning tracking for persistent detection
        self.warning_start_time = None
        self.persistent_warning_threshold = 30.0  # seconds
        self.drones_with_warnings = set()

    def get_time_in_mode(self) -> float:
        """Calculate time spent in current mode (seconds)."""
        return self._clock_fn() - self.mode_start_time

    def transition_to(self, new_mode: SystemModeEnum, reason: str):
        """Execute mode transition with logging."""
        if self.current_mode != new_mode:
            old_mode = self.current_mode.name
            self.current_mode = new_mode
            self.mode_start_time = self._clock_fn()
            self.last_transition_reason = reason

            self.node.get_logger().warn(
                f'SYSTEM MODE TRANSITION: {old_mode} -> {new_mode.name}. '
                f'Reason: {reason}'
            )

    def update_from_diagnostics(self, diag_array: DiagnosticArray):
        """
        Update system mode based on aggregated diagnostics.

        Split into two stages:
         (1) classify the diagnostic array into a `ModeTrigger`
             (the ROS-tied, mutable-history stage; uses
             `self.warning_start_time` + `self.drones_with_warnings`);
         (2) look up the (current_mode, trigger) → new_mode in the
             pure `SystemModeMachine` table.

        The transition rules live in
        `lib/domain/system_mode_machine.py` and are enumerable.
        `SystemModeMachine.can_transition` no-ops silently when
        the table doesn't have a rule (the legacy `if/elif` did this
        implicitly via the missing `else` branch).
        """
        from drone_rescue_coordination.lib.domain.system_mode_machine import (
            ModeTrigger,
            SystemMode,
            SystemModeMachine,
        )

        # now is a float seconds value from the Clock port,
        # not an rclpy Time.
        now = self._clock_fn()
        trigger, reason = self._classify_diagnostics(diag_array, now)
        if trigger is None:
            return

        current = SystemMode(self.current_mode.value)
        if not SystemModeMachine.can_transition(current, trigger):
            return
        new_mode = SystemModeMachine.transition(current, trigger)
        self.transition_to(SystemModeEnum(new_mode.value), reason)

    def _classify_diagnostics(self, diag_array: DiagnosticArray, now):
        """Diagnostic-shape analysis lives here; the
        FSM table doesn't know about DiagnosticArray. Returns
        `(ModeTrigger, reason)` or `(None, None)` if no trigger fires.

        Drone classification routes through
        ``classify_node`` instead of inline ``split('-')[0]`` +
        ``startswith('drone')`` string parsing. Any future NodeKind
        added to ``lib/lifecycle/watchdog`` propagates here
        automatically.
        """
        from drone_rescue_coordination.lib.domain.system_mode_machine import (
            ModeTrigger,
        )
        from drone_rescue_coordination.lib.lifecycle.watchdog import (
            NodeKind, classify_node,
        )

        has_error = False
        has_warn = False
        drones_with_warnings_now = set()

        for status in diag_array.status:
            if status.level == diagnostic_msgs.msg.DiagnosticStatus.ERROR:
                has_error = True
            elif status.level == diagnostic_msgs.msg.DiagnosticStatus.WARN:
                has_warn = True
                kind = classify_node(status.hardware_id)
                # Per-drone tally counts controllers + executors/surveyors
                # (the per-drone NodeKinds); pheromone-server warnings are
                # fleet-wide and handled by the persistent-warning path.
                if kind in (NodeKind.CONTROLLER, NodeKind.EXECUTOR_OR_SURVEYOR):
                    drones_with_warnings_now.add(status.hardware_id)

        # Update drone warning tracking (mutable; needs `self`).
        if has_warn:
            if self.warning_start_time is None:
                self.warning_start_time = now
            self.drones_with_warnings = drones_with_warnings_now
        else:
            self.warning_start_time = None
            self.drones_with_warnings.clear()

        if has_error:
            return (ModeTrigger.ERROR, 'ERROR status detected in diagnostics')

        if has_warn and self.warning_start_time is not None:
            # both `now` and `warning_start_time` are floats;
            # plain subtraction yields seconds.
            warning_duration = now - self.warning_start_time
            if warning_duration >= self.persistent_warning_threshold:
                return (
                    ModeTrigger.PERSISTENT_WARN,
                    f'Persistent warnings for {warning_duration:.1f}s '
                    f'(threshold: {self.persistent_warning_threshold}s)',
                )
            if len(self.drones_with_warnings) >= 2:
                return (
                    ModeTrigger.MULTI_DRONE_WARN,
                    f'Multiple drones with warnings '
                    f'({len(self.drones_with_warnings)} drones: '
                    f'{", ".join(sorted(self.drones_with_warnings))})',
                )
            return (None, None)

        # No warnings AND no errors → all-clear.
        return (
            ModeTrigger.ALL_CLEAR,
            'All warnings cleared - auto-recovery to normal operation',
        )


class LifecycleManager(Node):
    """
    Manages lifecycle transitions for drone survey system.

    Services:
        ~/startup: Trigger coordinated startup sequence
        ~/shutdown: Trigger coordinated shutdown sequence

    Parameters:
        drone_names: List of drone names to manage
        transition_timeout: Timeout for state transitions (seconds)
    """

    def __init__(self):
        super().__init__('lifecycle_manager')

        # Parameters
        self.declare_parameter('drone_names', default_drone_names_list())
        self.declare_parameter('transition_timeout', 10.0)
        self.declare_parameter('auto_startup', True)  # Auto-start on init
        # Wall-clock seconds without diagnostics before a node is declared
        # unresponsive (→ SAFE-mode RTH). Diagnostics are published on sim-time
        # timers, so at the sim's real-time factor (~0.37) and especially under
        # the containerised stack, where a CPU-saturated host periodically
        # stalls Gazebo for several seconds, a 5 s wall timeout false-triggers
        # and collapses the whole mission into SAFE-mode return on a transient
        # hiccup. 15 s tolerates those stalls while still catching a genuinely
        # dead node within a reasonable window.
        self.declare_parameter('watchdog_timeout', 15.0)
        self.declare_parameter('watchdog_check_rate', 1.0)  # Hz

        self.drone_names = self.get_parameter('drone_names').value
        self.transition_timeout = self.get_parameter('transition_timeout').value
        self.auto_startup = self.get_parameter('auto_startup').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.watchdog_check_rate = self.get_parameter('watchdog_check_rate').value

        # Build ordered list of managed nodes
        # Order: pheromone_server first, then controllers, then surveyors
        self.managed_nodes = self._build_node_list()

        # heartbeat tracking lifted to
        # `lib/lifecycle/watchdog.HeartbeatTracker`. ``node_heartbeats``
        # and ``unresponsive_nodes`` properties below preserve the
        # public attribute surface for back-compat (the RecoveryPolicy
        # unresponsive_provider + check_watchdog_status still read them).
        from drone_rescue_coordination.lib.lifecycle.watchdog import (
            HeartbeatTracker,
        )
        self._heartbeats = HeartbeatTracker(
            monitored=[],
            timeout_s=self.watchdog_timeout,
            clock_fn=lambda: self.get_clock().now().nanoseconds / 1e9,
        )

        # Nodes to monitor (populated from lifecycle nodes we manage)
        self.monitored_nodes = []

        # Initialize subscription and timer to None
        self.diagnostics_sub = None
        self.watchdog_timer = None
        self.system_mode_pub = None
        self.system_mode_timer = None
        self.updater = None
        self.emergency_srv = None
        self.emergency_sub = None
        self.cancel_recovery_srv = None
        self.stagger_timers = []  # Track stagger timers for cleanup
        self.recovery_cancelled = False
        self.return_home_pubs = {}  # drone_name -> publisher for survey_target
        # Pre-allocated land publishers, populated
        # in start_watchdog_monitoring() and torn down in _do_shutdown().
        self._land_pubs = {}        # drone_name -> publisher for /<drone>/land

        # Initialize mode manager
        self.mode_manager = ModeManager(self)

        # RecoveryDispatcher Protocol + RecoveryPolicy extracted.
        # The handler bodies live in
        # ``lib.lifecycle.recovery_policy``; this LifecycleNode is the
        # adapter that satisfies the dispatcher Protocol via
        # RosRecoveryDispatcher. The legacy `_recovery_*` methods are
        # gone; the table now dispatches directly to the policy.
        from drone_rescue_coordination.lib.lifecycle.recovery_policy import (
            RecoveryPolicy,
        )
        from drone_rescue_coordination.lib.ros_adapter.recovery_dispatcher import (
            RosRecoveryDispatcher,
        )
        self._recovery_dispatcher = RosRecoveryDispatcher(self)
        self._recovery_policy = RecoveryPolicy(
            dispatcher=self._recovery_dispatcher,
            unresponsive_provider=lambda: tuple(self.unresponsive_nodes),
            mode_provider=lambda: self._lib_current_mode(),
            # validate drone_id against the known fleet so namespace
            # drift surfaces as a logger warning instead of silently
            # no-opping the land command.
            known_drone_names=tuple(self.drone_names),
            logger_fn=lambda msg: self.get_logger().warning(msg),
        )

        # Callback group for async service calls
        self.callback_group = ReentrantCallbackGroup()

        # Services to trigger startup/shutdown
        self.startup_srv = self.create_service(
            Trigger,
            '~/startup',
            self.startup_callback
        )
        self.shutdown_srv = self.create_service(
            Trigger,
            '~/shutdown',
            self.shutdown_callback
        )

        # Shared cache of /<node>/change_state clients (populated lazily
        # by ``RclpyChangeStateClient`` so transitions reuse one Client
        # per target).
        self._change_state_clients = {}

        # lifecycle transitions go through the typed
        # ChangeStateClient port.
        from drone_rescue_coordination.lib.ros_adapter.change_state_client import (
            RclpyChangeStateClient,
        )
        self._change_state_port = RclpyChangeStateClient(
            self,
            client_cache=self._change_state_clients,
            callback_group=self.callback_group,
        )

        # phase ordering lifted to
        # ``lib/lifecycle/startup_sequencer.py``; ``_do_startup`` /
        # ``_do_shutdown`` below delegate to it.
        from drone_rescue_coordination.lib.lifecycle.startup_sequencer import (
            StartupSequencer,
        )
        self._sequencer = StartupSequencer(
            self._change_state_port,
            self.managed_nodes,
            self.drone_names,
            logger=self.get_logger(),
            transition_timeout_s=self.transition_timeout,
        )

        # Register signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        self._shutdown_requested = False
        # second-SIGINT abort flag, polled by
        # _wait_for_landing every 0.25 s. Set by _signal_handler on
        # the second invocation.
        self._shutdown_aborted = False

        self.get_logger().info(f'Lifecycle manager initialized with nodes: {self.managed_nodes}')

        # Auto-startup after short delay to let nodes spawn. With the SAR
        # redesign there are 10 lifecycle nodes; give them ~6s to advertise
        # their /change_state services before the first wait_for_service call.
        if self.auto_startup:
            self.startup_timer = self.create_timer(
                6.0,
                self._auto_startup,
            )

    # back-compat properties over the tracker.
    @property
    def unresponsive_nodes(self) -> set:
        return self._heartbeats.unresponsive

    @property
    def node_heartbeats(self) -> dict:
        return self._heartbeats.heartbeats

    def _build_node_list(self):
        """Build ordered list of nodes to manage.

        Delegates to the pure-Python helper in
        `lib/lifecycle/orchestrator.py`. Same ordering rules
        (pheromone first, then controllers, then mission_manager,
        then executors) but unit-testable without a LifecycleNode.
        """
        from drone_rescue_coordination.lib.lifecycle.orchestrator import (
            build_node_list,
        )
        return build_node_list(self.drone_names)

    def diagnostics_callback(self, msg: DiagnosticArray):
        """Process diagnostic messages to track node heartbeats and update system mode.

        Heartbeat bookkeeping routes through the
        lifted ``HeartbeatTracker``; ``record_heartbeat`` returns the
        matched name when an unresponsive flag was cleared as a side
        effect.
        """
        for status in msg.status:
            # Extract node identifier from hardware_id or name
            # Format is typically "namespace/hardware_id" or just "hardware_id"
            node_id = status.hardware_id if status.hardware_id else status.name
            matched = self._heartbeats.record_heartbeat(node_id)
            if matched is not None:
                self.get_logger().info(
                    f'Node {matched} is responsive again'
                )

        # Update system mode based on diagnostics
        self.mode_manager.update_from_diagnostics(msg)

    def watchdog_check_callback(self):
        """Periodic check for unresponsive nodes.

        The tracker's ``find_newly_unresponsive``
        returns only the names that crossed the timeout this tick.
        """
        for node_name, age in self._heartbeats.find_newly_unresponsive():
            self.get_logger().error(
                f'WATCHDOG: Node {node_name} unresponsive! '
                f'No diagnostics for {age:.1f}s (timeout: {self.watchdog_timeout}s)'
            )
            self._handle_unresponsive_node(node_name)

    def _handle_unresponsive_node(self, node_name: str):
        """Handle an unresponsive node by triggering appropriate recovery.

        Classifies the node via a typed `NodeKind` enum
        (replaces the legacy stringly-typed substring chain that would
        mis-route a node literally named e.g. `pheromone_controller`).
        Per-kind recovery handlers live as small private methods so a
        new node kind is one row in `_classify` + one handler.
        """
        self.get_logger().error(
            f'RECOVERY: Node {node_name} unresponsive - initiating recovery'
        )

        # Force diagnostic update to report the issue
        if self.updater:
            self.updater.force_update()

        # NodeKind + classifier in lib/lifecycle/watchdog.
        # Recovery handlers dispatched through
        # ``RecoveryPolicy`` (pure-Python, dispatcher-injected) instead
        # of the inline `_recovery_*` methods.
        from drone_rescue_coordination.lib.lifecycle.watchdog import (
            NodeKind, classify_node,
        )
        kind = classify_node(node_name)
        if kind == NodeKind.PHEROMONE:
            self.get_logger().error(
                'CRITICAL: Pheromone server unresponsive - all drones return home'
            )
            self._recovery_policy.handle_pheromone(node_name)
        elif kind == NodeKind.CONTROLLER:
            self._recovery_policy.handle_controller(node_name)
        elif kind == NodeKind.EXECUTOR_OR_SURVEYOR:
            self._recovery_policy.handle_executor_or_surveyor(node_name)
        else:
            self.get_logger().warning(
                f'Unknown node type for {node_name} - logging only'
            )
            self._recovery_policy.handle_unknown(node_name)

    def _lib_current_mode(self):
        """Adapter: projects the legacy ``SystemModeEnum`` (LifecycleNode-
        side enum) into the typed ``lib.domain.system_mode_machine.SystemMode``
        the policy reasons about."""
        from drone_rescue_coordination.lib.domain.system_mode_machine import (
            SystemMode as _LibSystemMode,
        )
        return _LibSystemMode(self.mode_manager.current_mode.value)

    def check_watchdog_status(self, stat):
        """Diagnostic callback for watchdog status."""
        if len(self.unresponsive_nodes) > 0:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                        f'{len(self.unresponsive_nodes)} node(s) unresponsive')
            stat.add('Unresponsive nodes', ', '.join(self.unresponsive_nodes))
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                        f'All {len(self.monitored_nodes)} nodes responsive')

        # Add per-node heartbeat ages.
        # heartbeats are now float seconds
        # (HeartbeatTracker.heartbeats keyed by node name).
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for node_name, last_seen in self.node_heartbeats.items():
            stat.add(f'{node_name} age', f'{now_sec - last_seen:.1f}s')

        return stat

    def check_system_mode(self, stat):
        """Diagnostic callback for system mode status."""
        mode = self.mode_manager.current_mode

        # Map mode to diagnostic level
        if mode == SystemModeEnum.SAFE:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                        f'System in SAFE mode')
        elif mode == SystemModeEnum.DEGRADED:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                        f'System in DEGRADED mode')
        else:  # NORMAL
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                        f'System in NORMAL mode')

        # Add mode details
        stat.add('Mode', mode.name)
        stat.add('Transition reason', self.mode_manager.last_transition_reason)
        stat.add('Time in mode', f'{self.mode_manager.get_time_in_mode():.1f}s')
        stat.add('Active drones', str(len(self.drone_names)))

        return stat

    def _build_system_mode_msg(self) -> SystemMode:
        """Single SystemMode builder used by both the 1 Hz
        timer (`publish_system_mode`) and the SAFE/DEGRADED transition
        callers (`_trigger_safe_mode` / `_handle_unresponsive_node`).
        Removes the near-duplicate built in `_publish_system_mode`
        that derived `time_in_mode` differently."""
        msg = SystemMode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.mode = self.mode_manager.current_mode.value
        msg.transition_reason = self.mode_manager.last_transition_reason
        msg.time_in_mode = self.mode_manager.get_time_in_mode()
        return msg

    def publish_system_mode(self):
        """Publish current system mode at 1 Hz.

        Message construction lives in
        `_build_system_mode_msg` so callers from SAFE/DEGRADED
        transitions agree on `time_in_mode` derivation.
        """
        if self.system_mode_pub is None:
            return
        self.system_mode_pub.publish(self._build_system_mode_msg())

    def emergency_shutdown_callback(self, request, response):
        """Service handler for manual emergency shutdown."""
        self.get_logger().error('=' * 60)
        self.get_logger().error('EMERGENCY SHUTDOWN REQUESTED BY OPERATOR')
        self.get_logger().error('=' * 60)

        self._trigger_safe_mode('Manual emergency shutdown via service')

        response.success = True
        response.message = (
            f'SAFE mode activated. {len(self.drone_names)} drones '
            'executing staggered return to home.'
        )
        return response

    def emergency_topic_callback(self, msg):
        """Topic handler for programmatic emergency trigger."""
        if msg.data:
            self.get_logger().error('Emergency trigger received via /system/emergency topic')
            self._trigger_safe_mode('Emergency topic message received')

    def cancel_recovery_callback(self, request, response):
        """Service handler for cancelling automatic recovery."""
        self.get_logger().warning('Recovery cancelled by operator')
        self.recovery_cancelled = True

        # Cancel any pending stagger timers
        for timer in self.stagger_timers:
            try:
                timer.cancel()
            except Exception:
                pass
        self.stagger_timers.clear()

        # Transition mode back one level
        if self.mode_manager.current_mode == SystemModeEnum.SAFE:
            self.mode_manager.transition_to(
                SystemModeEnum.DEGRADED,
                'Operator cancelled SAFE recovery'
            )
        elif self.mode_manager.current_mode == SystemModeEnum.DEGRADED:
            self.mode_manager.transition_to(
                SystemModeEnum.NORMAL,
                'Operator cancelled DEGRADED recovery'
            )

        response.success = True
        response.message = f'Recovery cancelled. Current mode: {self.mode_manager.current_mode.name}'
        return response

    def _trigger_safe_mode(self, reason: str):
        """Common logic for triggering SAFE mode."""
        self.recovery_cancelled = False
        self.mode_manager.transition_to(SystemModeEnum.SAFE, reason)
        self._publish_system_mode()
        self._execute_staggered_return()

    def _publish_system_mode(self):
        """Publish current system mode to /system_mode topic.

        Delegates to `publish_system_mode`. The legacy
        method had a divergent `time_in_mode` derivation (inline
        recompute from `mode_start_time`) that drifted from the 1 Hz
        timer's `mode_manager.get_time_in_mode()`. Single builder
        means a SAFE-mode entry log and the 1 Hz timer always agree.
        """
        self.publish_system_mode()

    def _execute_staggered_return(self):
        """Execute staggered return to home for all drones.

        ``rclpy.Node.create_timer`` is a REPEATING
        timer; the legacy ``create_timer(delay, fn)`` here had two
        bugs: (1) drone1 got ``delay=0`` which fires every executor
        tick, spamming ``_command_return_home`` continuously, and
        (2) drones 2-4 would re-fire every 5/10/15 s rather than
        once each. The closure-based self-cancel pattern below makes
        each timer one-shot: the callback cancels its own timer on
        first invocation before commanding the return.
        """
        stagger_delay = 5.0  # 5 seconds between drone departures

        self.get_logger().warning(
            f'Initiating staggered return for {len(self.drone_names)} drones '
            f'({stagger_delay}s intervals)'
        )

        for idx, drone_name in enumerate(self.drone_names):
            delay = idx * stagger_delay
            self._schedule_one_shot_return(drone_name, delay)
            self.get_logger().info(
                f'Scheduled {drone_name} return home in {delay:.0f}s'
            )

    def _schedule_one_shot_return(self, drone_name: str, delay: float) -> None:
        """Schedule a one-shot ``_command_return_home(drone_name)`` after
        ``delay`` seconds. Implemented via a self-cancelling repeating
        timer because rclpy.Node.create_timer has no ``oneshot`` flag
        in Jazzy."""
        # Boxed timer reference so the inner callback can cancel its
        # own timer on the first firing. A zero-or-tiny period would
        # fire every executor tick; clamp to a sensible minimum.
        period = max(float(delay), 0.05)
        timer_box: list = []

        def _fire():
            if timer_box:
                timer_box[0].cancel()
            self._command_return_home(drone_name)

        timer = self.create_timer(period, _fire)
        timer_box.append(timer)
        self.stagger_timers.append(timer)

    def _command_return_home(self, drone_name: str):
        """Command a single drone to return home."""
        if self.recovery_cancelled:
            self.get_logger().info(f'{drone_name}: Recovery cancelled, skipping return')
            return

        # Home position is (0, 0) - at survey altitude
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 10.0  # Survey altitude
        msg.pose.orientation.w = 1.0

        if drone_name in self.return_home_pubs:
            self.return_home_pubs[drone_name].publish(msg)
            self.get_logger().warning(f'{drone_name}: Commanded to return home (SAFE mode)')

    def _command_drone_land(self, drone_name: str):
        """Command a specific drone to land immediately.

        Was creating + destroying a Bool publisher
        on every call. The watchdog can fire this multiple times for
        a transiently-unresponsive drone, causing repeated DDS
        publisher discovery churn. The publishers are now pre-allocated
        in `start_watchdog_monitoring()` (mirroring the existing
        `return_home_pubs` pattern: one initialisation site,
        one teardown site).
        """
        self.get_logger().warning(f'{drone_name}: Commanding immediate land')
        pub = self._land_pubs.get(drone_name)
        if pub is None:
            # Defensive: should not happen if start_watchdog_monitoring
            # ran first. Fall back to the original create/destroy path.
            pub = self.create_publisher(Bool, f'/{drone_name}/land', 10)
            try:
                pub.publish(Bool(data=True))
            finally:
                self.destroy_publisher(pub)
            return
        pub.publish(Bool(data=True))

    def start_watchdog_monitoring(self):
        """Start monitoring node health via diagnostics and publish system mode."""
        # Subscribe to aggregated diagnostics
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics_agg',
            self.diagnostics_callback,
            10
        )

        # Create system mode publisher
        self.system_mode_pub = self.create_publisher(
            SystemMode,
            '/system_mode',
            10
        )

        # Create 1 Hz timer for publishing system mode
        self.system_mode_timer = self.create_timer(
            1.0,  # 1 Hz
            self.publish_system_mode
        )

        # Emergency shutdown service - operator can trigger SAFE mode
        self.emergency_srv = self.create_service(
            Trigger,
            '~/emergency_shutdown',
            self.emergency_shutdown_callback
        )

        # Emergency topic - programmatic SAFE mode trigger
        self.emergency_sub = self.create_subscription(
            Bool,
            '/system/emergency',
            self.emergency_topic_callback,
            10
        )

        # Cancel recovery service - operator override
        self.cancel_recovery_srv = self.create_service(
            Trigger,
            '~/cancel_recovery',
            self.cancel_recovery_callback
        )

        # Create publishers for commanding drones to return home
        for drone_name in self.drone_names:
            pub = self.create_publisher(
                PoseStamped,
                f'/{drone_name}/survey_target',
                10
            )
            self.return_home_pubs[drone_name] = pub

        # Pre-allocate land publishers (was
        # create+destroy per watchdog firing in _command_drone_land).
        for drone_name in self.drone_names:
            self._land_pubs[drone_name] = self.create_publisher(
                Bool, f'/{drone_name}/land', 10,
            )

        self.get_logger().info('Emergency shutdown interfaces created')

        # Populate monitored nodes from lifecycle nodes we manage
        # These should match the node hardware IDs set in diagnostic_updater
        for drone_name in self.drone_names:
            self.monitored_nodes.append(f'{drone_name}-controller')
            self.monitored_nodes.append(f'{drone_name}-executor')
        self.monitored_nodes.append('pheromone-grid')  # From pheromone_server

        # hand the monitored list to the tracker.
        # ``set_monitored`` seeds every name's last-seen timestamp with
        # the current clock reading.
        self._heartbeats.set_monitored(self.monitored_nodes)

        # Start watchdog timer
        self.watchdog_timer = self.create_timer(
            1.0 / self.watchdog_check_rate,
            self.watchdog_check_callback
        )

        # Initialize diagnostic updater for lifecycle manager itself
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID('lifecycle-manager')
        self.updater.add('Watchdog Status', self.check_watchdog_status)
        self.updater.add('System Mode', self.check_system_mode)

        self.get_logger().info(
            f'Watchdog monitoring started for {len(self.monitored_nodes)} nodes'
        )
        self.get_logger().info('System mode monitoring started - publishing to /system_mode at 1 Hz')

    def _auto_startup(self):
        """Automatic startup triggered by timer."""
        self.destroy_timer(self.startup_timer)
        self.get_logger().info('Auto-starting lifecycle management...')
        success = self._do_startup()
        if success:
            self.get_logger().info('All nodes activated successfully!')
            # Start watchdog monitoring after nodes are active
            self.start_watchdog_monitoring()
        else:
            self.get_logger().error('Startup failed - some nodes did not activate')

    def startup_callback(self, request, response):
        """Handle startup service request."""
        success = self._do_startup()
        response.success = success
        response.message = 'Startup complete' if success else 'Startup failed'
        return response

    def shutdown_callback(self, request, response):
        """Handle shutdown service request."""
        success = self._do_shutdown()
        response.success = success
        response.message = 'Shutdown complete' if success else 'Shutdown failed'
        return response

    def _do_startup(self) -> bool:
        """Execute startup sequence: configure then activate all nodes
        in order. Phase ordering lives in
        ``lib/lifecycle/startup_sequencer.py``."""
        return self._sequencer.startup()

    def _do_shutdown(self) -> bool:
        """Execute the configure→deactivate→cleanup shutdown sequence,
        then tear down node-local rclpy resources (watchdog timer,
        diagnostics sub, etc.).

        Phase ordering (deactivate executors → mission
        → controllers → wait_for_landing → pheromone → cleanup-all)
        lives in ``lib/lifecycle/startup_sequencer.py``. The teardown
        block below is node-specific and stays here.
        """
        success = self._sequencer.shutdown(
            wait_for_landing=self._wait_for_landing,
        )

        # Cleanup watchdog resources
        if self.watchdog_timer is not None:
            self.destroy_timer(self.watchdog_timer)
            self.watchdog_timer = None

        if self.system_mode_timer is not None:
            self.destroy_timer(self.system_mode_timer)
            self.system_mode_timer = None

        if self.diagnostics_sub is not None:
            self.destroy_subscription(self.diagnostics_sub)
            self.diagnostics_sub = None

        if self.system_mode_pub is not None:
            self.destroy_publisher(self.system_mode_pub)
            self.system_mode_pub = None

        # Cancel stagger timers
        for timer in self.stagger_timers:
            try:
                timer.cancel()
            except Exception:
                pass
        self.stagger_timers.clear()

        # Cleanup return home publishers
        for pub in self.return_home_pubs.values():
            try:
                self.destroy_publisher(pub)
            except Exception:
                pass
        self.return_home_pubs.clear()

        # Cleanup pre-allocated land publishers.
        for pub in self._land_pubs.values():
            try:
                self.destroy_publisher(pub)
            except Exception:
                pass
        self._land_pubs.clear()

        self.get_logger().info('='*50)
        if success:
            self.get_logger().info('Shutdown complete - all nodes cleaned up')
        else:
            self.get_logger().warn('Shutdown complete with warnings')
        self.get_logger().info('='*50)

        return success

    def _wait_for_landing(self, timeout: float = 30.0):
        """Wait for drones to finish their descent before continuing
        the shutdown sequence.

        Was a hardcoded ``time.sleep(8.0)`` with a
        TODO sentinel. The lifecycle_manager doesn't subscribe to
        drone odometry directly, so true z-altitude polling would need
        a new subscription (out of scope here). The
        practical improvement is to chunk the wait into short polls
        so a SIGINT (or any other ``_shutdown_requested`` flip)
        during the landing window aborts promptly instead of blocking
        the full duration. Total wait is bounded by ``landing_wait_s``
        (default 8.0 s, the original constant, preserved as the
        "estimated descent from ~10 m at ~2 m/s plus buffer" tuning).
        """
        landing_wait_s = 8.0
        poll_interval = 0.25
        elapsed = 0.0
        self.get_logger().info(
            f'  Waiting up to {landing_wait_s:.1f}s for landing to complete...'
        )
        while elapsed < landing_wait_s:
            # Early exit if a second shutdown signal arrives while we're
            # waiting (e.g. operator hit Ctrl+C twice). The first signal
            # got us into this method; honour the second one.
            if getattr(self, '_shutdown_aborted', False):
                self.get_logger().warning(
                    '  Landing wait aborted by second shutdown signal'
                )
                return
            time.sleep(poll_interval)
            elapsed += poll_interval
        self.get_logger().info('  Landing wait complete')

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully.

        First SIGINT initiates graceful shutdown; second SIGINT flips
        ``_shutdown_aborted`` so the ``_wait_for_landing`` poll loop
        returns within 0.25 s. Previously the second signal re-entered
        ``_do_shutdown`` while the first was still running and the
        aborted flag was never set.
        """
        if not self._shutdown_requested:
            self._shutdown_requested = True
            self.get_logger().info('Shutdown signal received, initiating graceful shutdown...')
            self._do_shutdown()
        else:
            self._shutdown_aborted = True
            self.get_logger().warning(
                'Second shutdown signal — aborting any pending wait.'
            )


def main(args=None):
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = LifecycleManager()
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
