"""Mission Control main window: assembles all four tabs and owns the
single per-mission lifecycle.

Threading model: the LaunchSupervisor's stdout reader fires its callbacks
on a background thread. Qt forbids touching widgets from non-GUI threads,
so we route everything through a `QObject`-derived `_Bridge` whose signals
auto-marshal to the main thread via Qt::QueuedConnection (default for
cross-thread emission). This keeps the GUI lock-free.

Mission Control does NOT spin its own ROS node. The lifecycle plumbing
(`ros2 param set`, `ros2 topic pub`) is done via subprocess so the Qt event
loop stays clean. The cost is a few hundred ms per `ros2 param set` call
on activation, which happens once per Run.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from python_qt_binding.QtCore import QObject, Qt, Signal, QTimer
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QMessageBox, QStatusBar,
)

from .scenario_loader import (
    Scenario, default_scenarios_dir, discover_scenarios,
)
from .process_supervisor import LaunchSupervisor
from .ros_control import RosControl
from .widgets.setup_tab import SetupTab
from .widgets.active_tab import ActiveTab
from .widgets.past_runs_tab import PastRunsTab
from .widgets.compare_tab import CompareTab
from .widgets.sweep_tab import SweepTab


# Default runs/ directory: workspace root + 'runs/'.
def _default_runs_dir() -> Path:
    here = Path(__file__).resolve()
    # …/install/.../site-packages/drone_rescue_mission_control/mission_control_app.py
    # …/src/drone_rescue_mission_control/drone_rescue_mission_control/mission_control_app.py
    for ancestor in here.parents:
        if (ancestor / 'src').is_dir() and (ancestor / 'install').is_dir():
            return ancestor / 'runs'
        if ancestor.name == 'drone_rescue_ws':
            return ancestor / 'runs'
    return Path.cwd() / 'runs'


class _Bridge(QObject):
    """Thread-bridge: receives signals from the supervisor reader thread
    and re-emits them on the main thread via QueuedConnection (default
    behavior when the receiver lives on a different thread)."""
    line = Signal(str)
    activated = Signal()
    exited = Signal(int)


class MissionControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Drone Rescue — Mission Control')
        self.resize(1200, 850)

        # State
        self._supervisor: Optional[LaunchSupervisor] = None
        self._current_scenario: Optional[Scenario] = None
        self._current_runtime_params: List[Tuple[str, str, object]] = []
        self._runs_dir = _default_runs_dir()
        self._runs_dir.mkdir(parents=True, exist_ok=True)

        # In-process ROS control. Lazy-started on the first param-set /
        # publish. closeEvent calls shutdown().
        self._ros_control = RosControl()

        # Discover scenarios (install share first, then source fallback).
        scenarios = discover_scenarios(default_scenarios_dir())
        if not scenarios:
            QMessageBox.warning(
                self, 'No scenarios found',
                f'No scenario YAMLs in {default_scenarios_dir()}.\n'
                f'Did you colcon build drone_rescue_bringup?'
            )

        # Bridge for thread-safe supervisor callbacks.
        self._bridge = _Bridge()
        self._bridge.line.connect(self._on_line, Qt.QueuedConnection)
        self._bridge.activated.connect(self._on_activated, Qt.QueuedConnection)
        self._bridge.exited.connect(self._on_exited, Qt.QueuedConnection)

        # Tabs
        self._tabs = QTabWidget()
        self._setup_tab = SetupTab(scenarios)
        self._active_tab = ActiveTab()
        self._past_runs_tab = PastRunsTab(self._runs_dir)
        self._compare_tab = CompareTab(self._past_runs_tab.selected_run_paths)
        self._sweep_tab = SweepTab()
        self._tabs.addTab(self._setup_tab, 'Setup')
        self._tabs.addTab(self._active_tab, 'Active')
        self._tabs.addTab(self._past_runs_tab, 'Past Runs')
        self._tabs.addTab(self._compare_tab, 'Compare Runs')
        self._tabs.addTab(self._sweep_tab, 'Sweep Runs')
        self.setCentralWidget(self._tabs)

        # Status bar
        sb = QStatusBar(self)
        self.setStatusBar(sb)
        sb.showMessage(f'Idle.   Runs: {self._runs_dir}')

        # Wiring
        self._setup_tab.runRequested.connect(self._on_run_requested)
        self._setup_tab.scenarioSaved.connect(self._on_scenario_saved)
        self._active_tab.stopRequested.connect(self._on_stop_requested)
        self._past_runs_tab.selectionChanged.connect(self._compare_tab.refresh)
        self._tabs.currentChanged.connect(self._on_tab_changed)

    # ------------------------------------------------------------ scenario save
    def _on_scenario_saved(self, name: str) -> None:
        """A "Save As…" wrote a new scenario YAML; re-discover the
        scenarios directory and refresh the Setup tab picker so the new
        scenario is selectable immediately."""
        scenarios = discover_scenarios(default_scenarios_dir())
        self._setup_tab.reload_scenarios(scenarios, select_name=name)
        self.statusBar().showMessage(f"Saved scenario '{name}'.")

    # ------------------------------------------------------------ tab focus
    def _on_tab_changed(self, idx: int) -> None:
        """When user opens Past Runs / Compare, refresh from disk."""
        w = self._tabs.widget(idx)
        if isinstance(w, PastRunsTab):
            w.refresh()
        elif isinstance(w, CompareTab):
            w.refresh()
        # Cross-fade the incoming tab.
        if w is not None:
            from drone_rescue_ui_common.motion import fade_in
            fade_in(w)

    # ------------------------------------------------------------ Run
    def _on_run_requested(
        self, launch_args: dict, runtime_params: list,
    ) -> None:
        if self._supervisor is not None and self._supervisor.is_alive:
            QMessageBox.warning(
                self, 'Mission already running',
                'A mission is already running. Stop it before starting another.',
            )
            return

        # SetupTab.runRequested already emits a launch_args dict that
        # includes record_run / scenario_yaml / scenario_name (built by
        # Scenario.launch_args()). We just need to add runs_dir.
        launch_args.setdefault('runs_dir', str(self._runs_dir))
        self._current_runtime_params = list(runtime_params)

        # Reset the active tab and hand control to the supervisor.
        self._active_tab.clear_log()
        self._active_tab.set_state('SPAWNING', 'Launching ros2 launch tree…')
        self._setup_tab.set_running(True)
        self._tabs.setCurrentWidget(self._active_tab)

        self._supervisor = LaunchSupervisor(
            launch_args=launch_args,
            on_line=self._bridge.line.emit,
            on_activated=self._bridge.activated.emit,
            on_exited=self._bridge.exited.emit,
        )
        try:
            self._supervisor.start()
            self.statusBar().showMessage('Mission spawning…')
        except Exception as e:
            self._active_tab.set_state('ERROR', f'Failed to spawn: {e}')
            self._setup_tab.set_running(False)
            self._supervisor = None
            self.statusBar().showMessage('Spawn failed.')
            return

        # Fallback: lifecycle_manager's auto_startup is racy with DDS
        # bringup on this host; sometimes the auto-trigger callback
        # silently doesn't fire and all 9 lifecycle nodes stay stuck in
        # `unconfigured`. After 30 s we proactively invoke
        # /lifecycle_manager/startup so the operator doesn't have to drop
        # to a shell. If activation already happened by then, the call is
        # a no-op (the lifecycle_manager skips already-active nodes).
        QTimer.singleShot(30_000, self._fallback_lifecycle_startup)

    # ------------------------------------------------------------ supervisor callbacks
    def _on_line(self, line: str) -> None:
        self._active_tab.append_stdout(line)

    def _on_activated(self) -> None:
        """Lifecycle nodes are up. Apply runtime params, then publish /survey/start.

        Parameter sets and the /survey/start publish go through the
        in-process ``RosControl`` adapter (rclpy AsyncParameterClient +
        a pre-created TRANSIENT_LOCAL publisher) instead of shelling out
        to the ros2 CLI. Saves 1.4-4 s of CLI startup overhead per Run
        with ~14 params.
        """
        self._active_tab.set_state(
            'ACTIVATING', 'Activated. Applying runtime params…',
        )
        # Group params by target node so we hit each node's
        # set_parameters service once with a batched request.
        by_node: Dict[str, List[Tuple[str, object]]] = {}
        for node, param, value in self._current_runtime_params:
            by_node.setdefault(node, []).append((param, value))
        failures: List[Tuple[str, str, str]] = []
        for node, params in by_node.items():
            results = self._ros_control.set_params(node, params)
            for name, ok, err in results:
                if not ok:
                    failures.append((node, name, err))
                    self._active_tab.append_stdout(
                        f'[mission_control] WARN: param set {node}.{name} → {err}'
                    )

        # /survey/start through the same in-process node, latched
        # (TRANSIENT_LOCAL) so a late-joining mission_manager picks it
        # up on subscribe.
        self._publish_survey_start()
        # The mission IS running, but a param that failed to apply means it
        # runs with a value other than the operator asked for; escalate
        # that to the banner instead of burying it in the stdout tail.
        if failures:
            self._active_tab.set_state(
                'RUNNING',
                f'⚠ {len(failures)} param(s) failed to apply — see log',
            )
            self.statusBar().showMessage(
                f'Mission running — {len(failures)} param(s) not applied.'
            )
        else:
            self._active_tab.set_state('RUNNING', 'Mission in progress.')
            self.statusBar().showMessage('Mission running.')

    def _on_exited(self, rc: int) -> None:
        # Subprocess died (either MISSION_COMPLETE, recorder writes JSONL
        # and the launch tree wraps up, or operator clicked Stop, or crash).
        if rc == 0:
            self._active_tab.set_state('DONE', f'Exited cleanly (rc={rc}).')
            self.statusBar().showMessage('Mission complete.')
        else:
            self._active_tab.set_state(
                'ERROR' if rc < 0 else 'DONE',
                f'Exited (rc={rc}).',
            )
            self.statusBar().showMessage(f'Mission ended (rc={rc}).')
        self._setup_tab.set_running(False)
        self._supervisor = None
        # Reflect new JSONL on disk.
        self._past_runs_tab.refresh()

    # ------------------------------------------------------------ Stop
    def _on_stop_requested(self) -> None:
        if self._supervisor is None or not self._supervisor.is_alive:
            return
        self._active_tab.set_state('STOPPING', 'Sending SIGTERM…')
        # Run stop() off the GUI thread so it doesn't block on the 8 s grace.
        QTimer.singleShot(0, self._supervisor.stop)

    # ------------------------------------------------------------ lifecycle fallback
    def _fallback_lifecycle_startup(self) -> None:
        """If activation sentinel still hasn't fired 30 s after spawn,
        manually invoke /lifecycle_manager/startup. Safe no-op if the
        nodes are already active.

        Last subprocess shell-out in the Qt process replaced by
        ``RosControl.call_service``. Forwards ``response.message`` to
        the operator stdout pane.
        """
        if self._supervisor is None or not self._supervisor.is_alive:
            return
        if self._supervisor.activated:
            return
        self._active_tab.append_stdout(
            '[mission_control] Auto-startup did not fire within 30 s — '
            'calling /lifecycle_manager/startup as a fallback'
        )
        self._active_tab.set_state(
            'ACTIVATING',
            '⚠ Auto-startup slow — invoking lifecycle fallback…',
        )
        from std_srvs.srv import Trigger
        success, message = self._ros_control.call_service(
            '/lifecycle_manager/startup', Trigger, timeout_s=30.0,
        )
        if success:
            tail = f' — {message}' if message else ''
            self._active_tab.append_stdout(
                f'[mission_control] /lifecycle_manager/startup OK{tail} — '
                'waiting for activation sentinel…'
            )
            self._active_tab.set_state(
                'ACTIVATING', 'Lifecycle fallback OK — waiting for activation…',
            )
        else:
            # The fallback was the last resort; if it failed the lifecycle
            # nodes will not activate, so this mission is dead, not slow.
            self._active_tab.append_stdout(
                f'[mission_control] /lifecycle_manager/startup FAILED: '
                f'{message[:200] if message else "unknown"}'
            )
            self._active_tab.set_state(
                'ERROR', 'Lifecycle activation failed — see log',
            )
            self.statusBar().showMessage('Lifecycle activation failed.')

    # ------------------------------------------------------------ ros2 helpers (in-process)
    def _publish_survey_start(self) -> None:
        """Publish ``/survey/start`` once, latched.

        Was a ``subprocess.run(['ros2', 'topic', 'pub', '--once',
        ...])``; now an in-process publisher (TRANSIENT_LOCAL,
        RELIABLE) on the long-lived ``self._ros_control`` adapter.
        Sub-millisecond instead of ~100-300 ms.
        """
        try:
            self._ros_control.publish_survey_start()
        except Exception as e:
            self._active_tab.append_stdout(
                f'[mission_control] WARN: /survey/start publish failed: {e}'
            )

    # ------------------------------------------------------------ shutdown
    def closeEvent(self, event):
        """Make sure we clean up the subprocess tree on window close."""
        if self._supervisor is not None and self._supervisor.is_alive:
            ans = QMessageBox.question(
                self, 'Mission still running',
                'A mission is still running. Stop it and exit?',
                QMessageBox.Yes | QMessageBox.No,
            )
            if ans != QMessageBox.Yes:
                event.ignore()
                return
            try:
                self._supervisor.stop()
            except Exception:
                pass
        # Tear down the in-process ROS node + spin thread cleanly.
        # Idempotent, safe even if RosControl was never lazily started.
        try:
            self._ros_control.shutdown()
        except Exception:
            pass
        event.accept()


def main(argv=None) -> int:
    # Must run before QApplication; see qt_theme.enable_hidpi for the
    # Xft.dpi-192 rationale.
    from drone_rescue_ui_common.qt_theme import enable_hidpi
    enable_hidpi()
    app = QApplication(argv if argv is not None else sys.argv)
    app.setApplicationName('Drone Rescue Mission Control')
    # Shared Fusion + dark-palette + qss() pass; same one-call entry
    # point the dashboard uses.
    from drone_rescue_ui_common.qt_theme import apply_app_theme
    apply_app_theme(app)
    # Dark-theme the embedded Compare/Sweep FigureCanvas plots. GUI
    # only; the report PDF CLI keeps light-on-white print figures.
    import matplotlib
    from drone_rescue_ui_common.style import mpl_rcparams
    matplotlib.rcParams.update(mpl_rcparams())
    win = MissionControlWindow()
    win.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
