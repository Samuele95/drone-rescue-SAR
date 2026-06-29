"""Regression test for the parameterised fleet roster.

The dashboard hard-coded a 4-name module constant (_DRONE_NAMES) for its
telemetry subscriptions and its camera/log widgets, so a fleet of more than four
drones silently lost telemetry and camera tiles. The roster is now a ROS
parameter (drone_names) threaded into the window; this test pins that a >4 roster
populates the widgets.
"""

from __future__ import annotations

import sys

import pytest

from python_qt_binding.QtWidgets import QApplication

from drone_rescue_dashboard.dashboard_app import (
    DashboardWindow, ImageCache, LogBuffer, StateCache,
)


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication(sys.argv)


_SIX = ['drone1', 'drone2', 'drone3', 'drone4', 'drone5', 'drone6']


def test_window_honours_a_six_drone_roster(app):
    win = DashboardWindow(
        StateCache(), LogBuffer(), ImageCache(), drone_names=_SIX)
    try:
        assert win._drone_names == _SIX
        # log drone-filter combo: "ALL" + one entry per drone.
        assert win._log_drone.count() == len(_SIX) + 1
        labels = [win._log_drone.itemText(i) for i in range(win._log_drone.count())]
        assert 'drone5' in labels and 'drone6' in labels
    finally:
        win.close()


def test_window_defaults_to_module_roster(app):
    """Without an explicit roster the window falls back to the 4-name default,
    so existing callers are unaffected."""
    win = DashboardWindow(StateCache(), LogBuffer(), ImageCache())
    try:
        assert len(win._drone_names) == 4
    finally:
        win.close()
