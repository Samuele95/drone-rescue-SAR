"""Tests for lib/lifecycle helpers.

Pure-Python; no rclpy.init().
"""

from __future__ import annotations

from drone_rescue_coordination.lib.lifecycle.orchestrator import build_node_list
from drone_rescue_coordination.lib.lifecycle.watchdog import (
    HeartbeatTracker,
    NodeKind,
    classify_node,
)


# build_node_list

def test_build_node_list_four_drones():
    nodes = build_node_list(['drone1', 'drone2', 'drone3', 'drone4'])
    assert nodes == [
        'pheromone_server',
        'drone1_controller', 'drone2_controller',
        'drone3_controller', 'drone4_controller',
        'mission_manager',
        'drone1_executor', 'drone2_executor',
        'drone3_executor', 'drone4_executor',
    ]


def test_build_node_list_two_drones_preserves_order():
    nodes = build_node_list(['drone_a', 'drone_b'])
    assert nodes[0] == 'pheromone_server'
    assert nodes[-1] == 'drone_b_executor'
    assert nodes.index('mission_manager') == 3


def test_build_node_list_empty_fleet():
    assert build_node_list([]) == ['pheromone_server', 'mission_manager']


# classify_node

def test_classify_node_pheromone_wins_over_controller():
    """A node literally named pheromone_controller resolves to
    PHEROMONE: fleet-wide critical kind is checked first."""
    assert classify_node('pheromone_controller') == NodeKind.PHEROMONE


def test_classify_node_drone_controller():
    assert classify_node('drone1_controller') == NodeKind.CONTROLLER


def test_classify_node_executor_and_surveyor_both_match():
    assert classify_node('drone2_executor') == NodeKind.EXECUTOR_OR_SURVEYOR
    assert classify_node('surveyor') == NodeKind.EXECUTOR_OR_SURVEYOR


def test_classify_node_unknown_falls_through():
    assert classify_node('mission_manager') == NodeKind.UNKNOWN


def test_classify_node_case_insensitive():
    assert classify_node('PHEROMONE_SERVER') == NodeKind.PHEROMONE


# HeartbeatTracker

def test_heartbeat_tracker_records_and_clears_flag():
    t = [0.0]
    tracker = HeartbeatTracker(
        monitored=['drone1_controller'],
        timeout_s=2.0,
        clock_fn=lambda: t[0],
    )
    tracker.record_heartbeat('drone1_controller')
    assert 'drone1_controller' in tracker.heartbeats


def test_heartbeat_tracker_finds_unresponsive():
    t = [0.0]
    tracker = HeartbeatTracker(
        monitored=['drone1_controller', 'drone2_controller'],
        timeout_s=1.0,
        clock_fn=lambda: t[0],
    )
    tracker.record_heartbeat('drone1_controller')
    tracker.record_heartbeat('drone2_controller')
    t[0] = 2.0
    newly = tracker.find_newly_unresponsive()
    names = sorted([n for n, _ in newly])
    assert names == ['drone1_controller', 'drone2_controller']
    # Once flagged, repeat calls don't re-emit.
    assert tracker.find_newly_unresponsive() == []


def test_heartbeat_tracker_match_by_substring():
    """diagnostic hardware_id is `<namespace>/<name>`; matching is
    substring so a node named 'drone1_controller' matches a
    hardware_id like 'drone1/drone1_controller_aux'."""
    t = [0.0]
    tracker = HeartbeatTracker(
        monitored=['drone1_controller'],
        timeout_s=1.0,
        clock_fn=lambda: t[0],
    )
    tracker.record_heartbeat('drone1_controller_aux')
    assert 'drone1_controller' in tracker.heartbeats


def test_heartbeat_tracker_all_seen():
    t = [0.0]
    tracker = HeartbeatTracker(
        monitored=['a', 'b'],
        timeout_s=1.0,
        clock_fn=lambda: t[0],
    )
    assert not tracker.all_seen()
    tracker.record_heartbeat('a')
    assert not tracker.all_seen()
    tracker.record_heartbeat('b')
    assert tracker.all_seen()
