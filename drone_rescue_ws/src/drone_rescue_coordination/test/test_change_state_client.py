"""Unit coverage for ChangeStateClient Protocol + InMemoryChangeStateRecorder.

The production adapter (RclpyChangeStateClient) wraps
rclpy.create_client + poll-wait-call-poll-done; that path is integration-
covered through the existing LifecycleManager startup tests. This file
pins the typed-port surface the future StartupSequencer will consume.
"""

from drone_rescue_coordination.lib.ports.change_state_client import (
    InMemoryChangeStateRecorder,
)


def test_recorder_returns_true_by_default():
    r = InMemoryChangeStateRecorder()
    assert r.transition('drone1_controller', 1) is True
    assert r.calls == [('transition', 'drone1_controller', 1)]


def test_recorder_fails_for_configured_targets():
    r = InMemoryChangeStateRecorder(
        fail_for=[('drone2_controller', 1)],
    )
    assert r.transition('drone1_controller', 1) is True
    assert r.transition('drone2_controller', 1) is False
    assert r.transition('drone1_controller', 3) is True


def test_recorder_get_state_logs_and_returns_unconfigured():
    r = InMemoryChangeStateRecorder()
    assert r.get_state('drone1_controller') == 1
    assert r.calls == [('get_state', 'drone1_controller')]


def test_recorder_records_call_sequence_in_order():
    r = InMemoryChangeStateRecorder()
    r.transition('a', 1)
    r.transition('b', 1)
    r.transition('a', 3)
    assert [c for c in r.calls if c[0] == 'transition'] == [
        ('transition', 'a', 1),
        ('transition', 'b', 1),
        ('transition', 'a', 3),
    ]
