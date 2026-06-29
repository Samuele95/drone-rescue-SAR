"""Unit tests for the ViewModelBridge change-detection pump.

The pump is exercised directly via ``poll_once()`` (no QTimer, no
ROS) so the coalescing logic is deterministic under test.
"""

from __future__ import annotations

import sys

from drone_rescue_dashboard.bridge import ViewModelBridge
from drone_rescue_dashboard.dashboard_app import (
    ImageCache, LogBuffer, StateCache,
)

import pytest

from python_qt_binding.QtWidgets import QApplication


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication(sys.argv)


def _make_bridge(app):
    state, log, images = StateCache(), LogBuffer(), ImageCache()
    bridge = ViewModelBridge(state, log, images, start_timer=False)
    return state, log, images, bridge


def test_no_emissions_when_nothing_changed(app):
    state, log, images, bridge = _make_bridge(app)
    hits = []
    bridge.view_changed.connect(lambda v: hits.append('view'))
    bridge.events_changed.connect(lambda: hits.append('events'))
    bridge.trails_changed.connect(lambda: hits.append('trails'))
    bridge.frame_arrived.connect(lambda t: hits.append(f'frame:{t}'))
    bridge.poll_once()
    bridge.poll_once()
    assert hits == []


def test_view_changed_fires_once_per_version_bump(app):
    state, log, images, bridge = _make_bridge(app)
    hits = []
    bridge.view_changed.connect(lambda v: hits.append(v))
    state.view = state.view.apply_saga_confirmed(1)
    state.view_version += 1
    bridge.poll_once()
    bridge.poll_once()   # no further change; must not re-emit
    assert len(hits) == 1
    assert hits[0] is state.view


def test_events_and_trails_counters(app):
    state, log, images, bridge = _make_bridge(app)
    hits = []
    bridge.events_changed.connect(lambda: hits.append('events'))
    bridge.trails_changed.connect(lambda: hits.append('trails'))

    class _Evt:
        pass

    log.append(_Evt())
    state.trails_appended['drone1'] = 3
    bridge.poll_once()
    assert hits == ['events', 'trails']
    bridge.poll_once()
    assert hits == ['events', 'trails']


def test_frame_arrived_per_topic(app):
    state, log, images, bridge = _make_bridge(app)
    hits = []
    bridge.frame_arrived.connect(lambda t: hits.append(t))
    images.frame_counts['/drone1/camera'] = 1
    bridge.poll_once()
    images.frame_counts['/drone1/camera'] = 2
    images.frame_counts['/drone2/camera'] = 1
    bridge.poll_once()
    assert hits == ['/drone1/camera', '/drone1/camera', '/drone2/camera']
