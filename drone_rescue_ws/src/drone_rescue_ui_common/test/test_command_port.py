"""Unit tests for the OperatorCommandPort seam.

The port is the driving boundary for operator to simulation
commands; Qt event handlers must depend on it, never on rclpy
publishers. These tests pin the contract shape and the two shipped
adapters (Null for tests, Recording for assertions).
"""

from __future__ import annotations

from drone_rescue_ui_common.command_port import (
    NullCommandAdapter, OperatorCommandPort, RecordingCommandAdapter,
)


def test_null_adapter_satisfies_protocol_and_noops():
    port: OperatorCommandPort = NullCommandAdapter()
    # Must accept every command without effect or error.
    port.request_survey_start()
    port.request_survey_stop()
    port.request_return_home('drone2')
    port.request_investigate(12.5, -3.0)


def test_recording_adapter_captures_commands_in_order():
    port = RecordingCommandAdapter()
    port.request_survey_start()
    port.request_return_home('drone1')
    port.request_investigate(1.0, 2.0)
    port.request_survey_stop()
    assert port.commands == [
        ('survey_start',),
        ('return_home', 'drone1'),
        ('investigate', 1.0, 2.0),
        ('survey_stop',),
    ]


def test_recording_adapter_is_a_structural_operator_command_port():
    # Protocol is structural; a simple isinstance check via typing
    # runtime_checkable would be nice-to-have but the contract is the
    # method set, exercised through the annotated reference.
    port: OperatorCommandPort = RecordingCommandAdapter()
    port.request_investigate(0.0, 0.0)
    assert ('investigate', 0.0, 0.0) in port.commands
