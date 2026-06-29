"""Dashboard Qt widgets: per-class modules.

All five dashboard widgets (`CoverageBanner`, `ImageTile`,
`MissionLogWidget`, `StateTableWidget`, `VictimsTableWidget`) live in
per-class modules, mirroring the shape Mission Control already uses
(`widgets/active_tab.py`, `widgets/past_runs_tab.py`, etc.). Adding
the next dashboard widget is a one-file edit, not a 700-LOC monolith
patch.

The rail/stage/inspector layout widgets live here too:
``FleetRail`` (+ ``DroneCard``), ``MissionBar``, ``InspectorPanel``.
"""

from .coverage_banner import CoverageBanner
from .drone_focus import DroneFocusWindow
from .fleet_rail import DroneCard, FleetRail
from .image_tile import ImageTile
from .inspector import InspectorPanel
from .live_trend import LiveTrendWidget, TrendBuffer
from .mission_bar import MissionBar
from .mission_log import MissionLogWidget
from .state_table import StateTableWidget
from .victims_table import VictimsTableWidget

__all__ = [
    'CoverageBanner', 'DroneCard', 'DroneFocusWindow', 'FleetRail',
    'ImageTile', 'InspectorPanel', 'LiveTrendWidget', 'MissionBar',
    'MissionLogWidget', 'StateTableWidget', 'TrendBuffer',
    'VictimsTableWidget',
]
