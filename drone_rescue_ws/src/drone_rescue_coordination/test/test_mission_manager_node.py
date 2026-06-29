"""Smoke tests for the composition-root scaffolding.

The new ``mission_manager_node`` and the ``mission_manager_translator``
module exist as the seam the saga migration will fill in. These tests
assert (a) the modules import cleanly without rclpy.init(), (b) the
feature-flag fallback to the legacy path works, (c) the translator
wires into the existing ``MissionPort`` Protocol so a fake port can be
substituted at the adapter boundary.

No rclpy.init(); these run in <100 ms.
"""

from __future__ import annotations

import os
from types import SimpleNamespace

from drone_rescue_msgs.msg import (
    DroneHealth as RosDroneHealth,
    TaskAssignment as RosTaskAssignment,
    TaskStatus as RosTaskStatus,
    VictimCandidate as RosVictimCandidate,
)

from drone_rescue_coordination import mission_manager_node
from drone_rescue_coordination.lib.domain.value_objects import OutgoingTask
from drone_rescue_coordination.mission_manager_translator import (
    handle_candidate, handle_health, handle_task_status, handle_tick,
)


# feature flag

def test_module_imports_cleanly():
    assert hasattr(mission_manager_node, 'main')


def test_feature_flag_defaults_to_legacy():
    """Legacy path stays the runtime default."""
    if 'USE_LEGACY_MISSION_MANAGER' in os.environ:
        del os.environ['USE_LEGACY_MISSION_MANAGER']
    import importlib
    importlib.reload(mission_manager_node)
    assert mission_manager_node.USE_LEGACY_MISSION_MANAGER is True


def test_feature_flag_off_when_env_var_zero():
    os.environ['USE_LEGACY_MISSION_MANAGER'] = '0'
    try:
        import importlib
        importlib.reload(mission_manager_node)
        assert mission_manager_node.USE_LEGACY_MISSION_MANAGER is False
    finally:
        del os.environ['USE_LEGACY_MISSION_MANAGER']
        import importlib
        importlib.reload(mission_manager_node)


# translator

class _FakePort:
    """Minimal MissionPort fake: records calls, returns no tasks."""

    def __init__(self):
        self.candidates = []
        self.statuses = []
        self.healths = []
        self.ticks = []

    def on_candidate(self, c):
        self.candidates.append(c)
        return []

    def on_task_status(self, s):
        self.statuses.append(s)
        return []

    def on_health(self, h):
        self.healths.append(h)
        return []

    def on_battery_low(self, name):
        return []

    def on_survey_start(self, now_sec):
        return []

    def tick(self, now_sec):
        self.ticks.append(now_sec)
        return [
            OutgoingTask(
                drone_name='drone1', task_type=0,
                waypoints=((1.0, 2.0, 0.0),), target=None,
                victim_id=0, priority=1, hover_seconds=0.0,
            ),
        ]

    def state_snapshot(self):
        return SimpleNamespace()


def test_handle_candidate_translates_and_delegates():
    port = _FakePort()
    msg = RosVictimCandidate()
    msg.candidate_id = 7
    msg.position.x, msg.position.y, msg.position.z = 1.0, 2.0, 0.5
    msg.confidence = 0.9
    out = handle_candidate(port, msg)
    assert len(port.candidates) == 1
    assert port.candidates[0].candidate_id == 7
    assert port.candidates[0].position.x == 1.0
    assert out == []   # fake returns no outgoing tasks


def test_handle_task_status_translates_and_delegates():
    port = _FakePort()
    msg = RosTaskStatus()
    msg.drone_name = 'drone3'
    msg.task_id = 42
    msg.status = 2
    handle_task_status(port, msg)
    assert port.statuses[0].drone_name == 'drone3'
    assert port.statuses[0].task_id == 42


def test_handle_health_translates_and_delegates():
    port = _FakePort()
    msg = RosDroneHealth()
    msg.drone_name = 'drone2'
    msg.anomaly_score = 0.3
    msg.unrecoverable = False
    msg.battery_remaining_s = 80.0
    handle_health(port, msg, battery_rth_threshold_s=60.0)
    assert port.healths[0].drone_name == 'drone2'
    assert port.healths[0].is_down is False
    assert port.healths[0].battery_ok is True   # 80 > 60


def test_handle_tick_round_trips_outgoing_tasks_to_ros_msgs():
    port = _FakePort()
    msgs = handle_tick(port, now_sec=42.0)
    assert port.ticks == [42.0]
    assert len(msgs) == 1
    assert isinstance(msgs[0], RosTaskAssignment)
    assert msgs[0].drone_name == 'drone1'
    assert msgs[0].task_type == 0
    assert len(msgs[0].waypoints) == 1


# /survey/start race fix
# The readiness coordinator publishes /survey/start TRANSIENT_LOCAL (latched).
# If it fires before mission_manager finishes lifecycle activation, the latched
# sample is delivered to the subscription the instant it is created in
# on_configure, while _is_active is still False. The old code dropped it and
# the mission never scanned. The fix defers it to on_activate.

def _bare_mm():
    from drone_rescue_coordination.mission_manager import MissionManager
    from drone_rescue_coordination.lib.domain.state_machines import MissionStage
    mm = object.__new__(MissionManager)
    mm._is_active = False
    mm._survey_start_pending = False
    mm._stage = MissionStage.ARMING
    mm._time = SimpleNamespace(now_sec=lambda: 7.0)
    mm._scan_calls = []
    mm._begin_scan = lambda: mm._scan_calls.append('scan')
    mm.get_logger = lambda: SimpleNamespace(
        info=lambda *a, **k: None, warning=lambda *a, **k: None,
    )
    return mm


def _bool(data):
    from std_msgs.msg import Bool
    m = Bool()
    m.data = data
    return m


def test_survey_start_while_inactive_is_deferred_not_dropped():
    """Latched survey-start during configure (inactive) → recorded, not
    scanned yet."""
    mm = _bare_mm()
    mm._on_survey_start(_bool(True))
    assert mm._survey_start_pending is True
    assert mm._scan_calls == []          # NOT scanned while inactive


def test_on_activate_honours_deferred_survey_start():
    """on_activate processes a pending survey-start → begin_scan runs and
    mission_start_sec is set."""
    mm = _bare_mm()
    mm._on_survey_start(_bool(True))     # arrives during configure
    mm.on_activate(None)                 # ARMING→DEPLOYING + honour pending
    assert mm._survey_start_pending is False
    assert mm._scan_calls == ['scan']
    assert mm._mission_start_sec == 7.0


def test_survey_start_while_active_scans_immediately():
    """Normal (non-race) path: active node receives the live survey-start
    and scans at once; unchanged behaviour."""
    from drone_rescue_coordination.lib.domain.state_machines import MissionStage
    mm = _bare_mm()
    mm._is_active = True
    mm._stage = MissionStage.DEPLOYING
    mm._on_survey_start(_bool(True))
    assert mm._scan_calls == ['scan']


def test_on_activate_without_pending_does_not_scan():
    """No survey-start yet → activation does not spuriously start the
    mission."""
    mm = _bare_mm()
    mm.on_activate(None)
    assert mm._scan_calls == []
    assert mm._survey_start_pending is False
