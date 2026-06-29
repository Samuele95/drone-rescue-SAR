"""Unit tests for operator-facing constants.

CONTROLLER_STATE_LABEL is the single decoding table for
``DroneStatus.state`` integers. The wire values
come from ``drone_rescue_coordination.lib.domain.drone_state.DroneState``
(published by drone_controller as ``self.state.value``); this test
pins the table to that 8-value contract so a silent divergence like
the old per-visualizer ``_DRONE_STATES`` dicts (telemetry showed
'EMERGENCY' for a hovering drone) cannot recur.
"""

from __future__ import annotations

from drone_rescue_ui_common.constants import CONTROLLER_STATE_LABEL


def test_controller_state_label_covers_all_eight_wire_states():
    """Every published wire value 0..7 needs a label."""
    for state in range(8):
        assert state in CONTROLLER_STATE_LABEL


def test_controller_state_label_matches_controller_enum_semantics():
    """Pin the load-bearing labels to the DroneState enum names.

    HOVER must be 5 and EMERGENCY must be 7; the old
    telemetry_overlay table had EMERGENCY at 5, mislabelling a
    hovering drone as an emergency.
    """
    assert CONTROLLER_STATE_LABEL[0] == 'IDLE'
    assert CONTROLLER_STATE_LABEL[2] == 'SURVEYING'
    assert CONTROLLER_STATE_LABEL[3] == 'RETURNING'
    assert CONTROLLER_STATE_LABEL[5] == 'HOVER'
    assert CONTROLLER_STATE_LABEL[7] == 'EMERGENCY'
