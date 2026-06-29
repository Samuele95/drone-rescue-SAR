"""ViewModelBridge: coalescing change-detection pump.

Replaces the five independent per-widget QTimers (100-500 ms) with
one GUI-thread pump that emits Qt signals ONLY when the underlying
caches actually changed, at a coalesced ceiling of ~30 Hz. Widgets
connect to the signals and re-render on change, so update latency
drops from "next 200-500 ms poll tick" to "<= 33 ms after arrival"
while idle scenes cost nothing.

Threading model (unchanged from the original design): the rclpy
executor thread writes the caches; under the GIL, replacing
``StateCache.view`` and incrementing the int version counters are
atomic stores. The pump's QTimer fires on the Qt main thread, reads
the counters, and emits signals synchronously there; every slot
runs on the GUI thread, no cross-thread signal emission, no locks.

Widgets that render *time-derived* state (staleness ages in the
state/victims tables) keep a slow 1 Hz heartbeat of their own:
a silent drone produces no version bump, but its data still ages.
"""

from __future__ import annotations

from typing import Dict

from python_qt_binding.QtCore import QObject, QTimer, Signal


class ViewModelBridge(QObject):
    """One pump, four change signals. Connect; don't poll."""

    #: emitted with the current frozen MissionViewModel snapshot
    view_changed = Signal(object)
    #: emitted when the mission-event log grew
    events_changed = Signal()
    #: emitted when any drone trail gained a point
    trails_changed = Signal()
    #: emitted per camera topic that received a new frame
    frame_arrived = Signal(str)

    #: pump interval: coalescing ceiling (~30 Hz)
    INTERVAL_MS = 33

    def __init__(self, state, log, images, *, start_timer: bool = True,
                 parent=None):
        super().__init__(parent)
        self._state = state
        self._log = log
        self._images = images
        self._seen_view_version = 0
        self._seen_events = 0
        self._seen_trails = 0
        self._seen_frames: Dict[str, int] = {}
        if start_timer:
            self._timer = QTimer(self)
            self._timer.timeout.connect(self.poll_once)
            self._timer.start(self.INTERVAL_MS)

    # ------------------------------------------------------------- pump
    def poll_once(self) -> None:
        """Compare counters; emit a signal per cache that changed.

        Public so tests (and a paused UI) can drive the pump
        deterministically without the timer.
        """
        version = getattr(self._state, 'view_version', 0)
        if version != self._seen_view_version:
            self._seen_view_version = version
            self.view_changed.emit(self._state.view)

        total = self._log.total_appended
        if total != self._seen_events:
            self._seen_events = total
            self.events_changed.emit()

        trails_total = sum(
            (getattr(self._state, 'trails_appended', None) or {}).values()
        )
        if trails_total != self._seen_trails:
            self._seen_trails = trails_total
            self.trails_changed.emit()

        frame_counts = getattr(self._images, 'frame_counts', None) or {}
        for topic, count in frame_counts.items():
            if self._seen_frames.get(topic) != count:
                self._seen_frames[topic] = count
                self.frame_arrived.emit(topic)
