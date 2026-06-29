"""Tests for the per-drone CV executor split.

The victim detector ran every drone's frame through one timer on a single
threaded executor, so one slow frame stalled the whole fleet's detection. It now
creates one detection timer per drone, each in its own ReentrantCallbackGroup,
driven by a MultiThreadedExecutor, with the shared victim-merge serialised by a
lock. These tests pin the per-drone timer split and the lock's presence.
"""

from __future__ import annotations

import pytest
import rclpy

from drone_rescue_coordination.victim_detector import VictimDetector


@pytest.fixture(scope='module', autouse=True)
def _rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def detector():
    node = VictimDetector()
    yield node
    node.destroy_node()


def test_one_detection_timer_per_drone(detector):
    assert set(detector.detection_timers.keys()) == set(detector.drone_names)
    assert len(detector.detection_timers) == len(detector.drone_names)


def test_each_drone_has_its_own_callback_group(detector):
    groups = list(detector._detection_cb_groups.values())
    # distinct group objects, one per drone (concurrent under MTE).
    assert len(groups) == len(detector.drone_names)
    assert len({id(g) for g in groups}) == len(detector.drone_names)


def test_victim_merge_lock_exists_and_is_free(detector):
    assert hasattr(detector, '_victim_lock')
    assert not detector._victim_lock.locked()


def test_whole_fleet_callback_delegates_without_error(detector):
    """The back-compat detection_callback loops over _detect_for_drone; with no
    images buffered it is a no-op and must not raise."""
    detector.detection_callback()
    assert detector.victim_counter == 0
