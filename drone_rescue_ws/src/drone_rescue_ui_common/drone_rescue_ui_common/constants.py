"""Operator-facing constants shared by the dashboard and Mission Control.

Today these are duplicated in (at least) ``dashboard_app.py`` and
``mission_control/widgets/compare_tab.py`` and ``analytics.py``, every
new task type or severity colour requires editing all three. This
module collapses them into one source so the duplication stops drifting.

Mirrors values from ``drone_rescue_msgs.msg.TaskAssignment`` and
``MissionEvent.severity`` so an operator-facing label change happens
in one place.
"""

from __future__ import annotations

from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P

# TaskAssignment.task_type to human-readable label. Keep in sync with
# the enum in drone_rescue_msgs/msg/TaskAssignment.msg.
TASK_LABEL = {
    0: 'SCAN',
    1: 'INVESTIGATE',
    2: 'CONFIRM',
    3: 'RTH',
    4: 'LAND',
    5: 'IDLE',
}

# MissionEvent.severity to CSS-style colour for live-log rendering.
# Derived from the semantic palette tokens (single source).
SEVERITY_COLOR = {
    0: _P.info,    # INFO: blue
    1: _P.warn,    # WARN: amber
    2: _P.error,   # ERROR: red
}

SEVERITY_LABEL = {
    0: 'INFO',
    1: 'WARN',
    2: 'ERROR',
}

# DroneStatus.state to operator-readable label. Single decoding table
# replacing two silently-diverged ``_DRONE_STATES`` dicts in
# drone_rescue_viz (telemetry_overlay mislabelled HOVER(5) as
# 'EMERGENCY'). The wire contract is the 8-value ``DroneState`` enum in
# drone_rescue_coordination/lib/domain/drone_state.py, published by
# drone_controller as ``status.state = self.state.value``.
CONTROLLER_STATE_LABEL = {
    0: 'IDLE',
    1: 'TAKEOFF',
    2: 'SURVEYING',
    3: 'RETURNING',
    4: 'LANDING',
    5: 'HOVER',
    6: 'NAVIGATING',
    7: 'EMERGENCY',
}

# Default 4-drone fleet name list. Operator-overrideable via launch
# arg / scenario YAML; this is the convenience default.
DEFAULT_DRONE_NAMES = ['drone1', 'drone2', 'drone3', 'drone4']

# Operator-facing per-drone hex colours. Canonical table consumed by
# ``MissionSceneView`` (dashboard) and intended for the RViz overlay
# nodes once they migrate onto MissionViewModel (deferred).
# High-contrast palette; falls back to '#cccccc' for drones beyond
# the seeded names.
DRONE_COLORS = {
    'drone1': '#ef4444',   # red
    'drone2': '#3b82f6',   # blue
    'drone3': '#22c55e',   # green
    'drone4': '#eab308',   # yellow
    'drone5': '#a855f7',   # purple
    'drone6': '#14b8a6',   # teal
    'drone7': '#f97316',   # orange
    'drone8': '#ec4899',   # pink
}
