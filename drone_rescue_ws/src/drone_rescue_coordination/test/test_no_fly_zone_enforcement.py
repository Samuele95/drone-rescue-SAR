"""No-fly-zone enforcement tests.

No-fly zones used to be advisory-only: zone_manager detected breaches and
published warnings/violations, but the only subscriber (surveyor.py) was a dead
node launched nowhere, the coverage planner built WorldModel with an empty zone
tuple, and no waypoint was ever filtered. Enforcement now lives in the live
mission_manager:

* scan waypoints inside a zone are dropped before dispatch
  (lib/domain/no_fly_zone_filter + MissionManager._filter_scan_waypoints), and
* a /zones/violation alert forces the offending drone to RTH
  (MissionManager._on_zone_violation, reusing the battery/operator RTH gate).

Pure helpers are unit-tested directly; the breach->RTH handler uses the
bare-instance pattern (no rclpy.init), same as test_operator_rth.
"""

from __future__ import annotations

from types import SimpleNamespace

from drone_rescue_coordination.lib.domain.drone_state import DroneState
from drone_rescue_coordination.lib.domain.no_fly_zone_filter import (
    drone_name_from_violation,
    filter_waypoints,
    precompute_states,
    waypoint_blocked_by,
)
from drone_rescue_coordination.lib.domain.value_objects import NoFlyZone
from drone_rescue_coordination.mission_manager import MissionManager

from drone_rescue_msgs.msg import TaskAssignment
from std_msgs.msg import String


# A polygon over [20,35]x[10,25], restricted up to 20 m AGL (buffer 2 m), and a
# tall critical zone that reaches survey altitude; mirrors no_fly_zones.yaml.
_LOW_POLY = NoFlyZone(
    name='unstable_structure_1', zone_type='polygon', priority='high',
    vertices=((20.0, 10.0), (35.0, 10.0), (35.0, 25.0), (20.0, 25.0)),
    min_altitude=0.0, max_altitude=20.0, buffer_distance=2.0,
)
_TALL_POLY = NoFlyZone(
    name='gas_leak_zone', zone_type='polygon', priority='critical',
    vertices=((5.0, 15.0), (15.0, 15.0), (15.0, 28.0), (5.0, 28.0)),
    min_altitude=0.0, max_altitude=50.0, buffer_distance=5.0,
)
_ZONES = [_LOW_POLY, _TALL_POLY]
_STATES = precompute_states(_ZONES)


def _pt(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


# waypoint gating

def test_waypoint_inside_zone_is_blocked():
    """A point inside the polygon, within its altitude band, is blocked."""
    assert waypoint_blocked_by(27.0, 17.0, 10.0, _ZONES, _STATES) is _LOW_POLY


def test_waypoint_above_zone_ceiling_is_allowed():
    """The low polygon caps at 20 m; a drone at 25 m AGL over it is allowed."""
    assert waypoint_blocked_by(27.0, 17.0, 25.0, _ZONES, _STATES) is None


def test_tall_zone_blocks_even_at_survey_altitude():
    """The critical gas-leak zone reaches 50 m, so it blocks at 25 m too."""
    assert waypoint_blocked_by(10.0, 21.0, 25.0, _ZONES, _STATES) is _TALL_POLY


def test_waypoint_outside_all_zones_is_allowed():
    assert waypoint_blocked_by(60.0, 60.0, 25.0, _ZONES, _STATES) is None


def test_filter_waypoints_drops_in_zone_points_preserving_order():
    wps = [_pt(0.0, 0.0, 25.0),      # clear
           _pt(10.0, 21.0, 25.0),    # inside tall zone -> dropped
           _pt(60.0, 60.0, 25.0)]    # clear
    kept, removed = filter_waypoints(wps, _ZONES, _STATES)
    assert [(p.x, p.y) for p in kept] == [(0.0, 0.0), (60.0, 60.0)]
    assert [(p.x, p.y) for p in removed] == [(10.0, 21.0)]


def test_filter_waypoints_no_zones_is_identity():
    wps = [_pt(10.0, 21.0, 25.0)]
    kept, removed = filter_waypoints(wps, [], {})
    assert kept == wps and removed == []


# violation alert parse

def test_parse_violation_extracts_drone_name():
    text = 'ZONE_VIOLATION: drone3 in gas_leak_zone (Gas leak - explosion hazard)'
    assert drone_name_from_violation(text) == 'drone3'


def test_parse_violation_rejects_foreign_text():
    assert drone_name_from_violation('hello world') is None
    assert drone_name_from_violation('') is None
    assert drone_name_from_violation('ZONE_VIOLATION:') is None


# breach -> RTH (bare node)

def _drone(task_type=TaskAssignment.SCAN_WAYPOINTS, *, down=False,
           battery_ok=True):
    return SimpleNamespace(
        is_down=down, battery_ok=battery_ok, current_task_type=task_type,
        drone_state=None,
    )


def _bare_mm(drones, active=True):
    mm = object.__new__(MissionManager)
    mm._is_active = active
    mm._drones = drones
    mm._issued = []
    mm._events = []
    mm._issue_task = lambda name, ttype, **kw: mm._issued.append((name, ttype))
    mm._emit_event = lambda *a, **k: mm._events.append((a, k))
    mm.get_logger = lambda: SimpleNamespace(
        info=lambda *a, **k: None, warning=lambda *a, **k: None,
        error=lambda *a, **k: None,
    )
    return mm


def _violation(data):
    m = String()
    m.data = data
    return m


def test_zone_violation_forces_rth_on_offending_drone():
    drones = {'drone1': _drone(), 'drone2': _drone()}
    mm = _bare_mm(drones)
    mm._on_zone_violation(
        _violation('ZONE_VIOLATION: drone1 in gas_leak_zone (hazard)')
    )
    assert mm._issued == [('drone1', TaskAssignment.RTH)]
    assert drones['drone1'].battery_ok is False
    assert drones['drone1'].drone_state == DroneState.RETURNING
    # drone2 untouched.
    assert drones['drone2'].battery_ok is True
    assert any(a[0] == 'ZONE_VIOLATION_RTH' for a, k in mm._events)


def test_zone_violation_unparseable_is_noop():
    drones = {'drone1': _drone()}
    mm = _bare_mm(drones)
    mm._on_zone_violation(_violation('garbage'))
    assert mm._issued == []
    assert drones['drone1'].battery_ok is True


def test_zone_violation_ignored_while_inactive():
    drones = {'drone1': _drone()}
    mm = _bare_mm(drones, active=False)
    mm._on_zone_violation(_violation('ZONE_VIOLATION: drone1 in z (r)'))
    assert mm._issued == []


def test_filter_scan_waypoints_method_drops_in_zone(monkeypatch):
    """MissionManager._filter_scan_waypoints wires the pure filter to the
    node's loaded zones (bare instance, no rclpy)."""
    mm = object.__new__(MissionManager)
    mm._no_fly_zones = _ZONES
    mm._zone_states = _STATES
    mm.get_logger = lambda: SimpleNamespace(warn=lambda *a, **k: None)
    wps = [_pt(0.0, 0.0, 25.0), _pt(10.0, 21.0, 25.0)]   # 2nd inside tall zone
    kept = mm._filter_scan_waypoints('drone1', wps)
    assert [(p.x, p.y) for p in kept] == [(0.0, 0.0)]


# real loader wiring (rclpy)

def test_mission_manager_loads_bringup_zones_and_enforces():
    """A default MissionManager loads the bringup no_fly_zones.yaml and its
    waypoint filter drops a point inside the gas-leak zone, proving the
    param/loader path end-to-end."""
    import rclpy
    rclpy.init()
    try:
        node = MissionManager()
        try:
            assert len(node._no_fly_zones) >= 1
            # (10, 21, 25) is inside gas_leak_zone (reaches 50 m).
            wps = [_pt(10.0, 21.0, 25.0), _pt(70.0, 70.0, 25.0)]
            kept = node._filter_scan_waypoints('drone1', wps)
            assert [(p.x, p.y) for p in kept] == [(70.0, 70.0)]
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
