"""Drone-rescue mission dashboard: rail/stage/inspector PyQt5 console.

The flat 7-tab layout became a mission-control
console: a MissionBar across the top (phase strip / clock / coverage /
victims / commands), a persistent FleetRail of per-drone cards on the
left, a central stage ([ 2D Plan ] [ 3D View ] [ Cameras ]
[ Overview ]), a contextual InspectorPanel on the right, and the
mission log in a collapsible bottom dock.

Surfaces share state via three subscriber-backed cache objects
(state_cache, log_buffer, image_cache).

ROS 2 plumbing: one rclpy node spins on a background daemon thread and
writes the caches; a single ViewModelBridge pump on the Qt main
thread emits change signals at most every 33 ms, and widgets re-render
on signal instead of polling. cv_bridge converts incoming
sensor_msgs/Image to numpy to QImage to QPixmap once per frame on the
ROS thread.

Launched standalone (`drone_rescue_dashboard/dashboard_app.py main`)
or via `dashboard.launch.py`.
"""

from __future__ import annotations

import sys
import threading
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String as StringMsg, UInt32
from drone_rescue_msgs.msg import (
    DronePeerState, DroneHealth, MissionEvent, CoverageMetrics,
    MissionState, PheromoneMap, VictimCandidate,
)

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QImage
from python_qt_binding.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QComboBox, QDockWidget, QLabel, QLineEdit, QMessageBox,
    QPushButton, QSplitter, QGroupBox,
)

from drone_rescue_dashboard.bridge import ViewModelBridge
from drone_rescue_dashboard.scene_view import MissionSceneView
# Widgets extracted into the ``widgets/`` submodule. Imports preserve
# the call-site shape for tab builders that reference these classes by
# name.
from drone_rescue_dashboard.widgets import (
    FleetRail, ImageTile, InspectorPanel, LiveTrendWidget, MissionBar,
    MissionLogWidget, StateTableWidget, VictimsTableWidget,
)

if False:   # TYPE_CHECKING-equivalent without a typing import churn
    from drone_rescue_ui_common.operator_view import SceneRenderer  # noqa


# Re-export the canonical fleet constant from
# ``drone_rescue_ui_common.constants``; the legacy local symbol
# stays as a list alias.
from drone_rescue_ui_common.constants import DEFAULT_DRONE_NAMES as _DRONE_NAMES_TUPLE
# QoS profiles come from the ui_common factories (single source).
# The saga_confirmed profile is shared verbatim with
# victim_visualizer via transient_local_reliable_qos(depth=64).
from drone_rescue_ui_common.qos import (
    mission_events_qos, peer_state_qos, sensor_qos,
    transient_local_reliable_qos,
)
_DRONE_NAMES = list(_DRONE_NAMES_TUPLE)

# Removed the _TASK_LABEL mirror that imported coordination's TaskType
# for a dict no caller in this file ever read (the actual consumer,
# state_table.py, already reads
# drone_rescue_ui_common.constants.TASK_LABEL). This closes the last
# dashboard->coordination runtime edge.

# Severity colour/label tables live alongside ``MissionLogWidget``
# in ``widgets/mission_log.py``.


# --------------------------------------------------------------- shared caches
@dataclass
class StateCache:
    """Thread-safe (single-writer) snapshot of per-drone state. Subscriber
    callbacks (running on the ROS executor thread) write; Qt timers (running
    on the GUI thread) read.

    The legacy mutable ``peer`` / ``peer_t`` / ``health`` /
    ``health_t`` / ``coverage`` / ``victims`` dicts are gone. All read sites
    consume ``self.view`` (a frozen ``MissionViewModel``). The ``replace()``
    assignment in the callbacks is atomic under the GIL (single attribute
    store), safe for the ROS-thread-writes / Qt-thread-reads pattern.
    ``trails`` is the only mutable per-drone field left because the
    sub-sampled deque shape has no view-model analogue yet.
    """
    # Per-drone trail of recent (x, y) world positions. Capped to a few
    # hundred points so the QPainterPath stays cheap to repaint.
    trails: Dict[str, deque] = field(default_factory=dict)
    # Per-drone monotonic append counter. Scene_view's
    # _refresh uses this (not len(trail)) to detect "new point arrived"
    # since the bounded deque plateaus at maxlen and len() stops growing
    # even as old points are evicted by new ones. Same regression class
    # as the length-based plateau bug in widgets/active_tab.py.
    trails_appended: Dict[str, int] = field(default_factory=dict)
    view: Any = None    # MissionViewModel; typed as Any to avoid an
                        # import-time dependency on drone_rescue_ui_common.
    # Monotonic version counter bumped on every
    # ``view`` replacement; the ViewModelBridge pump compares it to
    # decide whether to emit ``view_changed``. Same change-detection
    # idiom as ``trails_appended`` / ``LogBuffer.total_appended``.
    view_version: int = 0
    # Latest pheromone grid for the 3D glow floor.
    # Deliberately OUTSIDE MissionViewModel (mirrors the
    # pheromone_visualizer exclusion: a coordination-internal
    # stigmergy artifact, not operator mission state). numpy (h, w)
    # float array in [0, 1] + meta dict + monotonic version.
    pheromone: Any = None
    pheromone_meta: Any = None
    pheromone_version: int = 0

    def __post_init__(self):
        if self.view is None:
            from drone_rescue_ui_common.view_model import MissionViewModel
            self.view = MissionViewModel()


@dataclass
class LogBuffer:
    """Bounded ring of recent MissionEvents. Each tab that wants to render
    the log iterates this deque; the deque keeps the most-recent N entries.
    `total_appended` is a monotonically-growing counter the renderer uses
    to detect new events even after the deque hits maxlen (where len()
    plateaus and would otherwise hide new arrivals)."""
    events: deque = field(default_factory=lambda: deque(maxlen=400))
    total_appended: int = 0

    def append(self, event) -> None:
        """Record ``event`` with the wall-clock time it was *received*, as a
        ``(hh:mm:ss, event)`` pair. The timestamp is an attribute of the
        event record, not of the (possibly lagging) render tick;
        capturing it here keeps the log honest under burst traffic."""
        self.events.append((datetime.now().strftime('%H:%M:%S'), event))
        self.total_appended += 1


@dataclass
class ImageCache:
    """Latest decoded QImage per camera topic. The subscriber decodes once
    on the ROS thread; tabs that want to display pull the latest QImage and
    paint it scaled into a QLabel."""
    images: Dict[str, QImage] = field(default_factory=dict)
    bridge: CvBridge = field(default_factory=CvBridge)
    # Per-topic monotonic frame counter for the
    # ViewModelBridge's ``frame_arrived`` change detection; tiles
    # repaint per new frame instead of on a 10 Hz timer.
    frame_counts: Dict[str, int] = field(default_factory=dict)


class OperatorEcho:
    """MissionEvent-shaped record echoing an operator command into the
    log; every command leaves an audit line."""

    __slots__ = ('event_type', 'drone_name', 'detail', 'victim_id',
                 'position', 'severity')

    def __init__(self, detail: str, drone_name: str = ''):
        from types import SimpleNamespace
        self.event_type = 'OPERATOR_CMD'
        self.drone_name = drone_name
        self.detail = detail
        self.victim_id = 0
        self.position = SimpleNamespace(x=0.0, y=0.0)
        self.severity = 1   # WARN: operator actions stand out


# --------------------------------------------------------------- ros bridge node
class DashboardSubscriberNode(Node):
    """Owns all subscriptions; writes into the shared caches."""

    def __init__(self, state: StateCache, log: LogBuffer, images: ImageCache,
                 *, clock=None):
        super().__init__('drone_rescue_dashboard')
        self._state = state
        self._log = log
        self._images = images
        # UiClock port; ``RealUiClock`` default. Folded into the
        # apply_peer_state / apply_health calls below so widget
        # freshness gates read the same monotonic source.
        # Attribute named ``_time`` (not ``_clock``) because
        # rclpy.Node._clock is load-bearing internal state: shadowing
        # it breaks ``declare_parameter`` which reaches for
        # ``self._clock.now()`` to stamp parameter events.
        from drone_rescue_ui_common.clock import RealUiClock
        self._time = clock if clock is not None else RealUiClock()

        # Fleet roster as a ROS parameter, not the hard-coded 4-name
        # module constant. Fleets > 4 previously lost telemetry because the
        # subscription loop only ever iterated the default four names.
        self.declare_parameter('drone_names', list(_DRONE_NAMES))
        self.drone_names = list(self.get_parameter('drone_names').value
                                or _DRONE_NAMES)

        peer_qos = peer_state_qos()
        image_qos = sensor_qos(depth=1)
        for d in self.drone_names:
            self.create_subscription(
                DronePeerState, f'/{d}/peer_state',
                lambda msg, n=d: self._on_peer(n, msg), peer_qos,
            )
            self.create_subscription(
                DroneHealth, f'/{d}/health',
                lambda msg, n=d: self._on_health(n, msg), 10,
            )
            self.create_subscription(
                Image, f'/{d}/camera',
                lambda msg, t=f'/{d}/camera': self._on_image(t, msg),
                image_qos,
            )
            self.create_subscription(
                Image, f'/{d}/follow_cam',
                lambda msg, t=f'/{d}/follow_cam': self._on_image(t, msg),
                image_qos,
            )

        self.create_subscription(
            MissionEvent, '/mission/events',
            self._on_event,
            mission_events_qos(),
        )
        self.create_subscription(
            CoverageMetrics, '/coverage/metrics',
            self._on_coverage, 10,
        )
        # Victims (candidate AND confirmed flow through the same topic, with
        # `confirmed` flag toggling).
        self.create_subscription(
            VictimCandidate, '/victims/candidates',
            self._on_victim, 10,
        )
        # Saga-confirmation channel (mirrors
        # victim_visualizer's subscription). VictimCandidate.confirmed is
        # detection_filter's multi-view fusion gate (>=2 reporters); the
        # cross-drone CONFIRM saga's success is a separate signal that
        # mission_manager emits here. The dashboard ORs both via
        # MissionViewModel.apply_saga_confirmed. TRANSIENT_LOCAL/depth=64
        # matches the publisher so a late dashboard restart recovers
        # the per-mission set of confirmed cluster_ids.
        self.create_subscription(
            UInt32, '/victims/saga_confirmed',
            self._on_saga_confirmed,
            transient_local_reliable_qos(depth=64),
        )
        # /mission/state (1 Hz, RELIABLE/TRANSIENT_LOCAL at the
        # publisher) drives the mission bar's phase strip + sector
        # progress.
        self.create_subscription(
            MissionState, '/mission/state',
            self._on_mission_state,
            transient_local_reliable_qos(depth=1),
        )
        # Pheromone field (2 Hz) feeds the 3D
        # sand-table's glow floor (searched ground lights up).
        # Publisher (pheromone_server) is RELIABLE/TRANSIENT_LOCAL/
        # depth=1; matching gets the latched grid on late join.
        self.create_subscription(
            PheromoneMap, '/pheromone/map',
            self._on_pheromone, transient_local_reliable_qos(depth=1),
        )
        # Per-drone odom for the trail rendering. peer_state ticks at 2 Hz
        # so its position resolution is too coarse for a smooth trail; odom
        # is plentiful (Gazebo bridges at sensor rate).
        for d in self.drone_names:
            self.create_subscription(
                Odometry, f'/{d}/odom',
                lambda msg, n=d: self._on_odom(n, msg),
                image_qos,
            )

    # ------------------------------------------------------------- callbacks
    # Callbacks fold straight into the frozen
    # ``MissionViewModel``; the legacy mutable dicts are gone.
    # ``now`` reads via the UiClock port (named ``self._time``
    # because ``self._clock`` shadows rclpy.Node._clock).
    def _on_peer(self, drone: str, msg: DronePeerState) -> None:
        self._state.view = self._state.view.apply_peer_state(
            msg, now=self._time.monotonic(),
        )
        self._state.view_version += 1

    def _on_health(self, drone: str, msg: DroneHealth) -> None:
        self._state.view = self._state.view.apply_health(
            drone, float(getattr(msg, 'anomaly_score', 0.0)),
            now=self._time.monotonic(),
            reason=str(getattr(msg, 'reason', '')),
            unrecoverable=bool(getattr(msg, 'unrecoverable', False)),
        )
        self._state.view_version += 1

    def _on_event(self, msg: MissionEvent) -> None:
        self._log.append(msg)   # captures receive-time
        self._state.view = self._state.view.append_event(msg)
        self._state.view_version += 1

    def _on_coverage(self, msg: CoverageMetrics) -> None:
        self._state.view = self._state.view.apply_coverage(msg)
        self._state.view_version += 1

    def _on_mission_state(self, msg: MissionState) -> None:
        self._state.view = self._state.view.apply_mission_state(msg)
        self._state.view_version += 1

    def _on_pheromone(self, msg: PheromoneMap) -> None:
        try:
            grid = np.asarray(msg.data, dtype=np.float32).reshape(
                int(msg.height), int(msg.width),
            )
        except ValueError:
            return   # malformed grid; skip the frame
        self._state.pheromone = grid
        self._state.pheromone_meta = {
            'resolution': float(msg.resolution),
            'origin_x': float(msg.origin.x),
            'origin_y': float(msg.origin.y),
        }
        self._state.pheromone_version += 1

    def _on_victim(self, msg: VictimCandidate) -> None:
        self._state.view = self._state.view.apply_victim_candidate(msg)
        self._state.view_version += 1

    def _on_saga_confirmed(self, msg: UInt32) -> None:
        """Record a saga-confirmed cluster_id
        so the dashboard's victim panel paints it as confirmed even
        when ``VictimCandidate.confirmed`` never fired (which is the
        common case under the realistic detection-range setting:
        sector-scanning rarely yields >=2 reporters per cluster)."""
        self._state.view = self._state.view.apply_saga_confirmed(int(msg.data))
        self._state.view_version += 1

    def _on_odom(self, drone: str, msg: Odometry) -> None:
        trail = self._state.trails.get(drone)
        if trail is None:
            trail = deque(maxlen=400)
            self._state.trails[drone] = trail
        # Subsample: only push if the new point is > 0.5 m from the last
        # so the trail isn't a heap of overlapping points.
        p = msg.pose.pose.position
        if trail:
            lx, ly = trail[-1]
            if (lx - p.x) ** 2 + (ly - p.y) ** 2 < 0.25:
                return
        trail.append((p.x, p.y))
        # Monotonic append counter for the trail change-detection in
        # scene_view._refresh. See StateCache ``trails_appended`` for
        # the regression-class note.
        self._state.trails_appended[drone] = (
            self._state.trails_appended.get(drone, 0) + 1
        )

    def _on_image(self, topic: str, msg: Image) -> None:
        try:
            cv = self._images.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:  # pragma: no cover — bridge errors are rare
            self.get_logger().warn(f'cv_bridge failed on {topic}: {e}')
            return
        # cv is HxWx3 uint8 RGB. Build a QImage that COPIES the buffer (the
        # underlying numpy array can be deallocated before Qt paints).
        h, w, _ = cv.shape
        qimg = QImage(cv.data, w, h, w * 3, QImage.Format_RGB888).copy()
        self._images.images[topic] = qimg
        self._images.frame_counts[topic] = (
            self._images.frame_counts.get(topic, 0) + 1
        )


# ImageTile lives in ``widgets/image_tile.py``.


# StateTableWidget lives in ``widgets/state_table.py``.


# MissionLogWidget lives in ``widgets/mission_log.py``.


# CoverageBanner lives in ``widgets/coverage_banner.py``.


# VictimsTableWidget lives in ``widgets/victims_table.py``.


# ----------------------------------------------------------- command adapter
class RosCommandAdapter:
    """OperatorCommandPort over rclpy publishers.

    The production implementation of
    ``drone_rescue_ui_common.command_port.OperatorCommandPort``. Qt
    event handlers call the port; this adapter owns the publishers,
    so no widget ever touches rclpy directly (hexagonal architecture).

    Command semantics (deliberately reusing existing topics):
    - survey start  -> ``/survey/start`` Bool True (latched; the
      mission_manager ignores False, so there is no "pause").
    - survey stop   -> "recall fleet": ``/mission/operator_rth`` with
      ``'*'``; mission_manager issues every drone an RTH task.
    - return home   -> ``/mission/operator_rth`` with the drone name.
      (The original raw home setpoint to ``/<drone>/survey_target``
      was overwritten by the executor's per-tick survey stream within
      one tick, so commands must flow through the task system so the
      executor's BT switches branch.)
    - investigate   -> ``/mission/operator_goal`` PointStamped; the
      mission_manager mints a synthetic candidate and runs the normal
      INVESTIGATE saga.

    rclpy publishers are thread-safe; calls land on the Qt thread.
    """

    def __init__(self, node: Node, drones=None):
        self._node = node
        self._survey_pub = node.create_publisher(
            Bool, '/survey/start', transient_local_reliable_qos(),
        )
        self._goal_pub = node.create_publisher(
            PointStamped, '/mission/operator_goal', 10,
        )
        self._rth_pub = node.create_publisher(
            StringMsg, '/mission/operator_rth', 10,
        )

    def request_survey_start(self) -> None:
        msg = Bool()
        msg.data = True
        self._survey_pub.publish(msg)

    def request_survey_stop(self) -> None:
        self.request_return_home('*')

    def request_return_home(self, drone: str) -> None:
        msg = StringMsg()
        msg.data = str(drone)
        self._rth_pub.publish(msg)

    def request_investigate(self, x: float, y: float) -> None:
        msg = PointStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.point.x = float(x)
        msg.point.y = float(y)
        self._goal_pub.publish(msg)


# --------------------------------------------------------------- main window
class DashboardWindow(QMainWindow):
    """Rail / stage / inspector operator console.

    Layout grammar:
      - MissionBar across the top (phase strip, clock, coverage,
        victims, command actions);
      - persistent FleetRail on the left;
      - central stage QTabWidget (2D Plan / 3D View / Cameras /
        Overview);
      - contextual InspectorPanel on the right;
      - collapsible Mission Log dock along the bottom.

    All widgets are bridge-driven: they re-render when their
    underlying cache actually changed instead of polling timers.
    The scene tabs are held via the ``SceneRenderer`` Protocol,
    so a renderer swap never edits this builder.
    """

    def __init__(self, state: StateCache, log: LogBuffer, images: ImageCache,
                 no_fly_yaml_path: Optional[str] = None, cmd_port=None,
                 drone_names=None):
        super().__init__()
        self.setWindowTitle('Drone Rescue Mission Dashboard')
        self.resize(1480, 900)

        self._state = state
        self._log = log
        self._images = images
        self._cmd = cmd_port
        # Roster from the node's drone_names param (fleets > 4 used to
        # lose their camera tiles and the log drone-filter entry).
        self._drone_names = list(drone_names) if drone_names else list(_DRONE_NAMES)

        self._bridge = ViewModelBridge(state, log, images, parent=self)

        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self._mission_bar = MissionBar(state, bridge=self._bridge)
        outer.addWidget(self._mission_bar)

        splitter = QSplitter(Qt.Horizontal)
        self._rail = FleetRail(state, bridge=self._bridge)
        splitter.addWidget(self._rail)

        self._stage = QTabWidget()
        # SceneRenderer-typed references (2D always; 3D when the GL
        # stack imports; headless CI may lack a GL context).
        self._scene2d: 'SceneRenderer' = MissionSceneView(
            state, no_fly_yaml_path=no_fly_yaml_path, bridge=self._bridge,
        )
        self._stage.addTab(self._scene2d, '2D Plan')
        self._scene3d: Optional['SceneRenderer'] = None
        try:
            from drone_rescue_dashboard.scene3d_view import Scene3DView
            self._scene3d = Scene3DView(
                state, no_fly_yaml_path=no_fly_yaml_path,
                bridge=self._bridge,
            )
            self._stage.addTab(self._scene3d, '3D View')
        except Exception as exc:  # pragma: no cover — GL stack absent
            print(f'[dashboard] 3D view unavailable: {exc}')
        self._stage.addTab(self._make_cameras_panel(), 'Cameras')
        self._stage.addTab(self._make_overview_panel(), 'Overview')
        # Short cross-fade on stage switch.
        # The 3D page is exempt: QGraphicsOpacityEffect renders the
        # subtree to a pixmap, which breaks the GL viewport.
        self._stage.currentChanged.connect(self._on_stage_changed)
        splitter.addWidget(self._stage)

        self._inspector = InspectorPanel(
            state, images, bridge=self._bridge,
        )
        splitter.addWidget(self._inspector)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        # Rail/inspector flex now (no fixed widths); seed proportions
        # and widen the drag handle.
        splitter.setHandleWidth(6)
        splitter.setSizes([190, 1020, 270])
        outer.addWidget(splitter, stretch=1)
        self.setCentralWidget(central)

        # Mission log: collapsible bottom dock with filter toolbar.
        self._log_widget = MissionLogWidget(log, bridge=self._bridge)
        dock_body = QWidget()
        dock_lay = QVBoxLayout(dock_body)
        dock_lay.setContentsMargins(4, 2, 4, 4)
        dock_lay.setSpacing(3)
        dock_lay.addLayout(self._make_log_toolbar())
        dock_lay.addWidget(self._log_widget)
        dock = QDockWidget('Mission Log', self)
        dock.setObjectName('missionLogDock')
        dock.setFeatures(
            QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetClosable,
        )
        dock.setWidget(dock_body)
        dock.setMinimumHeight(140)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)
        self._log_dock = dock

        # Selection wiring: rail / scene / victims table to inspector.
        self._rail.drone_selected.connect(self._inspector.show_drone)
        self._scene2d.drone_clicked.connect(self._inspector.show_drone)
        self._scene2d.victim_clicked.connect(self._inspector.show_victim)
        self._victims_table.victim_selected.connect(
            self._inspector.show_victim,
        )

        # Drone focus windows: double-click a fleet card, or the
        # inspector's Focus button, opens a dedicated big-camera +
        # full-telemetry window. One window per drone, reused.
        self._focus_windows: Dict[str, QWidget] = {}
        self._rail.drone_focused.connect(self._open_focus)
        focus_btn = QPushButton('⛶ Focus view')
        focus_btn.setToolTip(
            'Open the selected drone in its own window '
            '(big camera + full telemetry)'
        )
        focus_btn.clicked.connect(self._on_focus_selected)
        self._inspector.actions.addWidget(focus_btn)

        # Operator commands (only when a port was injected).
        if self._cmd is not None:
            self._build_command_actions()
            self._scene2d.investigate_requested.connect(
                self._on_investigate,
            )

    # ------------------------------------------------- drone focus window
    def _on_focus_selected(self) -> None:
        drone = self._inspector.selected_drone
        if drone is None:
            QMessageBox.information(
                self, 'Focus view', 'Select a drone first.',
            )
            return
        self._open_focus(drone)

    def _open_focus(self, name: str) -> None:
        win = self._focus_windows.get(name)
        if win is None:
            from drone_rescue_dashboard.widgets import DroneFocusWindow
            win = DroneFocusWindow(
                name, self._state, self._images,
                bridge=self._bridge, cmd_port=self._cmd, parent=self,
            )
            win.closed.connect(
                lambda n: self._focus_windows.pop(n, None),
            )
            self._focus_windows[name] = win
        win.show()
        win.raise_()
        win.activateWindow()

    def _on_stage_changed(self, idx: int) -> None:
        page = self._stage.widget(idx)
        if page is None or page is self._scene3d:
            return
        from drone_rescue_ui_common.motion import fade_in
        fade_in(page)

    def _make_log_toolbar(self) -> QHBoxLayout:
        """Severity / drone / free-text filters over the log."""
        bar = QHBoxLayout()
        bar.setSpacing(6)
        self._log_severity = QComboBox()
        self._log_severity.addItem('ALL', None)
        self._log_severity.addItem('INFO', 0)
        self._log_severity.addItem('WARN', 1)
        self._log_severity.addItem('ERROR', 2)
        bar.addWidget(QLabel('severity'))
        bar.addWidget(self._log_severity)
        self._log_drone = QComboBox()
        self._log_drone.addItem('ALL', None)
        for d in self._drone_names:
            self._log_drone.addItem(d, d)
        bar.addWidget(QLabel('drone'))
        bar.addWidget(self._log_drone)
        self._log_search = QLineEdit()
        self._log_search.setPlaceholderText('search events…')
        self._log_search.setClearButtonEnabled(True)
        bar.addWidget(self._log_search, stretch=1)
        for w in (self._log_severity, self._log_drone):
            w.currentIndexChanged.connect(self._apply_log_filters)
        self._log_search.textChanged.connect(self._apply_log_filters)
        return bar

    def _apply_log_filters(self, *_args) -> None:
        self._log_widget.set_filters(
            severity=self._log_severity.currentData(),
            drone=self._log_drone.currentData(),
            search=self._log_search.text(),
        )

    # ------------------------------------------------------ P3 commands
    def _build_command_actions(self) -> None:
        start = QPushButton('▸ Start survey')
        start.setObjectName('actionRun')
        start.setToolTip('Publish /survey/start (latched)')
        start.clicked.connect(self._on_start_survey)
        self._mission_bar.actions.addWidget(start)

        recall = QPushButton('⌂ Recall fleet')
        recall.setObjectName('actionStop')
        recall.setToolTip(
            'Issue every drone an RTH task via /mission/operator_rth '
            '(fly home + land; no further tasks assigned). '
            '/survey/start has no stop semantics.'
        )
        recall.clicked.connect(self._on_recall_fleet)
        self._mission_bar.actions.addWidget(recall)

        rth = QPushButton('⌂ Return home')
        rth.setToolTip('Issue the selected drone an RTH task '
                       '(fly home + land)')
        rth.clicked.connect(self._on_return_home)
        self._inspector.actions.addWidget(rth)

    def _echo(self, text: str, drone: str = '') -> None:
        self._log.append(OperatorEcho(text, drone))

    def _on_start_survey(self) -> None:
        self._cmd.request_survey_start()
        self._echo('survey start requested')

    def _on_recall_fleet(self) -> None:
        if QMessageBox.question(
            self, 'Recall fleet',
            'Recall ALL drones (RTH task: fly home + land)?',
        ) != QMessageBox.Yes:
            return
        self._cmd.request_survey_stop()
        self._echo('fleet recall: RTH task requested for all drones')

    def _on_return_home(self) -> None:
        drone = self._inspector.selected_drone
        if drone is None:
            QMessageBox.information(
                self, 'Return home', 'Select a drone first.',
            )
            return
        if QMessageBox.question(
            self, 'Return home',
            f'Recall {drone} (RTH task: fly home + land)?',
        ) != QMessageBox.Yes:
            return
        self._cmd.request_return_home(drone)
        self._echo('RTH task requested', drone)

    def _on_investigate(self, x: float, y: float) -> None:
        mission = self._state.view.mission
        note = ''
        if not (mission.received and mission.status in (3, 4)):
            note = ('\n\nNote: the mission is not in SCANNING/'
                    'INVESTIGATING — mission_manager will drop goals '
                    'until the survey starts.')
        if QMessageBox.question(
            self, 'Investigate here',
            f'Dispatch an INVESTIGATE goal at ({x:.1f}, {y:.1f})?{note}',
        ) != QMessageBox.Yes:
            return
        self._cmd.request_investigate(x, y)
        self._echo(f'investigate goal at ({x:.1f}, {y:.1f})')

    # ----------------------------------------------------------- panels
    def _make_cameras_panel(self) -> QWidget:
        w = QWidget()
        grid = QGridLayout(w)
        grid.setSpacing(6)
        # N drones × 2 cameras (downward, follow), two drones per row.
        for i, drone in enumerate(self._drone_names):
            row, col = divmod(i, 2)
            cell = QGroupBox(drone)
            cl = QHBoxLayout(cell)
            cl.addWidget(ImageTile(f'/{drone}/camera', self._images,
                                   title=f'{drone} ↓ down',
                                   bridge=self._bridge))
            cl.addWidget(ImageTile(f'/{drone}/follow_cam', self._images,
                                   title=f'{drone} → follow',
                                   bridge=self._bridge))
            grid.addWidget(cell, row, col)
        return w

    def _make_overview_panel(self) -> QWidget:
        """Trend + tables: the analytic complement to the scenes."""
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addWidget(LiveTrendWidget(self._state))
        splitter = QSplitter(Qt.Vertical)
        drones_box = QGroupBox('Drones')
        dbl = QVBoxLayout(drones_box)
        dbl.addWidget(StateTableWidget(self._state, bridge=self._bridge))
        splitter.addWidget(drones_box)
        victims_box = QGroupBox('Victims')
        vbl = QVBoxLayout(victims_box)
        self._victims_table = VictimsTableWidget(
            self._state, bridge=self._bridge,
        )
        vbl.addWidget(self._victims_table)
        splitter.addWidget(victims_box)
        lay.addWidget(splitter)
        return w


# --------------------------------------------------------------- bootstrap
def main(args=None):
    # Align Qt's GL integration with PyOpenGL.
    # Under XWayland Qt defaults to a GL path whose current-context
    # query PyOpenGL cannot see (GLLinePlotItem/GLScatterPlotItem then
    # fail with 'no valid context'); xcb + xcb_egl makes them agree.
    # setdefault keeps an operator's explicit env winning.
    import os
    os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
    os.environ.setdefault('QT_XCB_GL_INTEGRATION', 'xcb_egl')
    # Scaled-display fix: without this, Xft.dpi 192 doubles the text
    # but not the px chrome (see qt_theme.enable_hidpi).
    from drone_rescue_ui_common.qt_theme import enable_hidpi
    enable_hidpi()

    rclpy.init(args=args)

    state = StateCache()
    log = LogBuffer()
    images = ImageCache()
    node = DashboardSubscriberNode(state, log, images)

    # No-fly zones YAML path (used by the Mission Scene to draw overlays).
    node.declare_parameter('no_fly_zones_yaml', '')
    no_fly = node.get_parameter('no_fly_zones_yaml').value or None

    # Spin ROS in a background thread so the Qt main loop stays responsive.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication.instance() or QApplication(sys.argv)
    # Palette-derived application styling; moved behind the shared
    # Fusion + dark-QPalette + qss() entry point (kills light-palette
    # leaks in unstyled corners).
    from drone_rescue_ui_common.qt_theme import apply_app_theme
    apply_app_theme(app)
    cmd_port = RosCommandAdapter(node)
    win = DashboardWindow(state, log, images, no_fly_yaml_path=no_fly,
                          cmd_port=cmd_port, drone_names=node.drone_names)
    win.show()
    try:
        exit_code = app.exec_()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
