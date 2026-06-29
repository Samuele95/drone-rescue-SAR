"""drone_rescue_ui_common: shared UI primitives and projections.

Both ``drone_rescue_dashboard`` and ``drone_rescue_mission_control``
consume the same ROS topic set + the same JSONL schema; this package
gives them one source of truth for:

- per-task labels, severity colours, and other operator-facing constants
  (``constants``);
- per-topic QoS profile factories so peer_state, events, sensor topics
  use the same QoS in every consumer (``qos``);
- the ``MissionViewModel`` reducer that folds ``MissionEvent`` /
  ``DronePeerState`` / ``CoverageMetrics`` / ``VictimCandidate`` streams
  into a typed projection (``view_model``).

No PyQt5 or rclpy in this layer: Qt widgets and ROS subscriptions
live in the consumer packages. This package is pure-Python so it can
be unit-tested headlessly.
"""

from .constants import (
    TASK_LABEL, SEVERITY_COLOR, SEVERITY_LABEL, DEFAULT_DRONE_NAMES,
)
from .qos import (
    peer_state_qos, mission_events_qos, sensor_qos, transient_local_reliable_qos,
)
from .view_model import (
    MissionViewModel, DroneViewState, VictimViewState, CoverageViewState,
    MissionStateView, drone_status,
)
from .run_view_model import RunRow, RunViewModel
# UI-overhaul additions: the operator-command port, the SceneRenderer
# contract, and the palette-derived QSS.
from .command_port import (
    NullCommandAdapter, OperatorCommandPort, RecordingCommandAdapter,
)
from .operator_view import OperatorView, SceneRenderer
from .style import MONO_FAMILY, qss

__all__ = [
    'TASK_LABEL', 'SEVERITY_COLOR', 'SEVERITY_LABEL', 'DEFAULT_DRONE_NAMES',
    'peer_state_qos', 'mission_events_qos', 'sensor_qos',
    'transient_local_reliable_qos',
    'MissionViewModel', 'DroneViewState', 'VictimViewState', 'CoverageViewState',
    'MissionStateView', 'drone_status',
    'RunRow', 'RunViewModel',
    'OperatorCommandPort', 'NullCommandAdapter', 'RecordingCommandAdapter',
    'OperatorView', 'SceneRenderer',
    'qss', 'MONO_FAMILY',
]
