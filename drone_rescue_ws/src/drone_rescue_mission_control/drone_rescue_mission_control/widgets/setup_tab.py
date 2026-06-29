"""Setup tab: scenario picker + parameter form + Run button.

The form is populated from the chosen scenario YAML on selection. Each
field that the operator tweaks before clicking Run becomes a one-shot
override layered on top of the scenario defaults; the scenario YAML is
never mutated. (The operator can add a "Save As..." button later if they
want to make a tweak persistent.)

When Run is clicked, this widget emits `runRequested(dict launch_args,
list runtime_params)` so Mission Control can spawn the supervisor and
schedule the param sets. Mission Control disables this tab while a
mission is alive.
"""

from __future__ import annotations

import re
from typing import Dict, List, Optional, Tuple

from python_qt_binding.QtCore import Qt, Signal
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox,
    QLabel, QComboBox, QSpinBox, QDoubleSpinBox, QPushButton,
    QSizePolicy, QScrollArea, QFrame, QInputDialog, QMessageBox,
)

from ..scenario_loader import (
    Scenario, save_scenario, default_scenarios_dir,
)


def _slugify(name: str) -> str:
    """Filesystem-safe stem for a scenario YAML derived from its name."""
    slug = re.sub(r'[^a-z0-9]+', '_', name.lower()).strip('_')
    return slug or 'scenario'


# Field type hints: drive which Qt widget we build per parameter.
# Derived from `lib/domain/scenario_schema` so adding a scenario
# parameter is a one-row edit there; this form updates automatically.
# Form widget kinds: 'int', 'float', 'choice'(...), 'str'.
from drone_rescue_coordination.lib.domain.scenario_schema import (
    ParamScope as _ParamScope,
    form_schema_for_scope as _form_schema_for_scope,
    PARAM_SCHEMA as _PARAM_SCHEMA,
)
_LAUNCH_FIELD_TYPES = _form_schema_for_scope(_ParamScope.LAUNCH)
_MISSION_FIELD_TYPES = _form_schema_for_scope(_ParamScope.MISSION)
_DETECTION_FIELD_TYPES = _form_schema_for_scope(_ParamScope.DETECTION)

# schema default lookup so _build_field falls back to the param's
# CANONICAL default (not the form_kind minimum) when the scenario YAML
# doesn't carry a value. Without this, params absent from default.yaml's
# detection: block (e.g. cluster_window_seconds, min_distance_from_drones,
# min_confirm_observations) ended up at the form minimum (0 / 1) and were
# pushed to the running detection_filter at activation, stomping on the
# launch file's correct values.
_SCHEMA_DEFAULTS = {p.name: p.default for p in _PARAM_SCHEMA}


def _build_field(
    parent: QWidget, kind: tuple, default,
) -> QWidget:
    """Return a Qt widget pre-populated with `default`, sized for inline use."""
    if kind[0] == 'int':
        w = QSpinBox(parent)
        w.setRange(int(kind[1]), int(kind[2]))
        w.setValue(int(default) if default is not None else int(kind[1]))
        return w
    if kind[0] == 'float':
        w = QDoubleSpinBox(parent)
        w.setRange(float(kind[1]), float(kind[2]))
        w.setSingleStep(float(kind[3]) if len(kind) > 3 else 0.1)
        w.setDecimals(2)
        w.setValue(float(default) if default is not None else float(kind[1]))
        return w
    if kind[0] == 'choice':
        w = QComboBox(parent)
        choices = kind[1]
        w.addItems(choices)
        if default is not None and str(default) in choices:
            w.setCurrentText(str(default))
        return w
    # 'str': readonly label for now (operator changes via YAML).
    w = QLabel(str(default) if default is not None else '—', parent)
    w.setStyleSheet(f'color: {_P.text_muted};')
    return w


def _read_field(w: QWidget, kind: tuple):
    if kind[0] == 'int':
        return int(w.value())
    if kind[0] == 'float':
        return float(w.value())
    if kind[0] == 'choice':
        return w.currentText()
    # 'str' fields are read-only QLabels. The label shows '—' when the
    # scenario doesn't set a value; treat that as None so we don't forward
    # an em-dash literal to ros2 launch.
    if hasattr(w, 'text'):
        text = w.text()
        if text in ('', '—'):
            return None
        return text
    return None


class SetupTab(QWidget):
    """Scenario picker + parameter form."""

    runRequested = Signal(dict, list)   # (launch_args, runtime_params)
    scenarioSaved = Signal(str)         # new scenario name (for "Save As…")

    def __init__(self, scenarios: List[Scenario], parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._scenarios = scenarios
        self._launch_widgets: Dict[str, Tuple[QWidget, tuple]] = {}
        self._mission_widgets: Dict[str, Tuple[QWidget, tuple]] = {}
        self._detection_widgets: Dict[str, Tuple[QWidget, tuple]] = {}

        outer = QVBoxLayout(self)

        # Scenario picker row (pinned at top)
        picker_row = QHBoxLayout()
        picker_row.addWidget(QLabel('Scenario:'))
        self._scenario_combo = QComboBox()
        for s in scenarios:
            self._scenario_combo.addItem(s.name)
        picker_row.addWidget(self._scenario_combo)
        picker_row.addStretch(1)
        self._save_as_btn = QPushButton('Save As…')
        picker_row.addWidget(self._save_as_btn)
        self._reset_btn = QPushButton('Reset to scenario defaults')
        picker_row.addWidget(self._reset_btn)
        outer.addLayout(picker_row)

        self._description = QLabel('Pick a scenario to load defaults…')
        self._description.setWordWrap(True)
        self._description.setStyleSheet(
            f'color: {_P.text_muted}; padding: 4px;')
        outer.addWidget(self._description)

        # Scroll area for the three form blocks. With ~20 spinboxes the
        # form easily exceeds 800 px tall on a typical window; without
        # this the Run button below would scroll off the bottom of the
        # tab and become unreachable.
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll_inner = QWidget()
        scroll_lay = QVBoxLayout(scroll_inner)
        scroll_lay.setContentsMargins(0, 0, 0, 0)
        scroll_lay.addWidget(self._build_block(
            'Launch args', _LAUNCH_FIELD_TYPES, self._launch_widgets,
        ))
        scroll_lay.addWidget(self._build_block(
            'Mission params', _MISSION_FIELD_TYPES, self._mission_widgets,
        ))
        scroll_lay.addWidget(self._build_block(
            'Detection params', _DETECTION_FIELD_TYPES, self._detection_widgets,
        ))
        scroll_lay.addStretch(1)
        scroll.setWidget(scroll_inner)
        outer.addWidget(scroll, stretch=1)

        # Run row (pinned at bottom, outside the scroll area so it's
        # always visible).
        run_row = QHBoxLayout()
        run_row.addStretch(1)
        self._run_btn = QPushButton('  Run  ▶')
        # Colors come from the shared qss() #actionRun rule; only the
        # outsized launch-button geometry stays local.
        self._run_btn.setObjectName('actionRun')
        self._run_btn.setStyleSheet('padding: 8px 24px; font-size: 14pt;')
        self._run_btn.setMinimumHeight(40)
        run_row.addWidget(self._run_btn)
        outer.addLayout(run_row)

        # Wiring
        self._scenario_combo.currentIndexChanged.connect(self._on_pick)
        self._reset_btn.clicked.connect(self._on_pick)
        self._save_as_btn.clicked.connect(self._on_save_as)
        self._run_btn.clicked.connect(self._emit_run)

        if scenarios:
            self._on_pick()

    # --------------------------------------------------------- block builder
    def _build_block(self, title: str, schema: Dict, target_dict: Dict) -> QGroupBox:
        box = QGroupBox(title)
        form = QFormLayout(box)
        for key, kind in schema.items():
            # seed each widget with the schema's canonical default
            # (was: None, so _build_field fell back to the form-kind
            # minimum, which silently pushed bad values like
            # cluster_window_seconds=0 to detection_filter at activation).
            w = _build_field(box, kind, default=_SCHEMA_DEFAULTS.get(key))
            target_dict[key] = (w, kind)
            form.addRow(key, w)
        return box

    # --------------------------------------------------------- selection
    def _current_scenario(self) -> Optional[Scenario]:
        idx = self._scenario_combo.currentIndex()
        if idx < 0 or idx >= len(self._scenarios):
            return None
        return self._scenarios[idx]

    def _on_pick(self) -> None:
        s = self._current_scenario()
        if s is None:
            return
        self._description.setText(s.description.strip() or '(no description)')
        # Repopulate form fields from the scenario. `seed` is a Scenario
        # top-level field (not in s.launch), so pull from s.seed for it.
        for key, (w, kind) in self._launch_widgets.items():
            if key == 'seed':
                self._set_field_value(w, kind, getattr(s, 'seed', 0))
            else:
                self._set_field_value(w, kind, s.launch.get(key))
        for key, (w, kind) in self._mission_widgets.items():
            self._set_field_value(w, kind, s.mission.get(key))
        for key, (w, kind) in self._detection_widgets.items():
            self._set_field_value(w, kind, s.detection.get(key))

    def _set_field_value(self, w: QWidget, kind: tuple, value):
        if value is None:
            return
        if kind[0] == 'int':
            w.setValue(int(value))
        elif kind[0] == 'float':
            w.setValue(float(value))
        elif kind[0] == 'choice':
            if str(value) in kind[1]:
                w.setCurrentText(str(value))
        else:
            if hasattr(w, 'setText'):
                w.setText(str(value))

    # --------------------------------------------------------- run
    def _emit_run(self) -> None:
        s = self._current_scenario()
        if s is None:
            return
        # Build launch args = scenario base + form overrides on the launch block.
        # Skip None/empty/em-dash placeholder values so we don't forward
        # 'world:=—' style noise to ros2 launch.
        launch_args = s.launch_args()
        for key, (w, kind) in self._launch_widgets.items():
            v = _read_field(w, kind)
            if v is None or v == '' or v == '—':
                continue
            launch_args[key] = str(v)

        # Build runtime params = scenario mission/detection + form overrides.
        runtime_params: List[Tuple[str, str, object]] = []
        for key, (w, kind) in self._mission_widgets.items():
            v = _read_field(w, kind)
            runtime_params.append(('mission_manager', key, v))
        for key, (w, kind) in self._detection_widgets.items():
            v = _read_field(w, kind)
            runtime_params.append(('detection_filter', key, v))

        self.runRequested.emit(launch_args, runtime_params)

    # --------------------------------------------------------- save as
    def _on_save_as(self) -> None:
        """Persist the current form values as a new scenario YAML.

        The form can't edit ``description`` / ``ground_truth_victims`` /
        ``drone_overrides``: those carry over unchanged from the base
        scenario the operator started from.
        """
        base = self._current_scenario()
        if base is None:
            return
        name, ok = QInputDialog.getText(
            self, 'Save scenario as…', 'New scenario name:',
            text=f'{base.name} (copy)',
        )
        name = name.strip()
        if not ok or not name:
            return

        path = default_scenarios_dir() / f'{_slugify(name)}.yaml'
        if path.exists():
            ans = QMessageBox.question(
                self, 'Overwrite scenario?',
                f'{path.name} already exists. Overwrite it?',
                QMessageBox.Yes | QMessageBox.No,
            )
            if ans != QMessageBox.Yes:
                return

        # Read the three form blocks. `seed` is a top-level Scenario field,
        # not part of the launch: block, so it's pulled out separately.
        launch: Dict[str, object] = {}
        seed = int(getattr(base, 'seed', 0))
        for key, (w, kind) in self._launch_widgets.items():
            v = _read_field(w, kind)
            if v is None or v == '' or v == '—':
                continue
            if key == 'seed':
                seed = int(v)
            else:
                launch[key] = v
        mission: Dict[str, object] = {}
        for key, (w, kind) in self._mission_widgets.items():
            v = _read_field(w, kind)
            if v is not None:
                mission[key] = v
        detection: Dict[str, object] = {}
        for key, (w, kind) in self._detection_widgets.items():
            v = _read_field(w, kind)
            if v is not None:
                detection[key] = v

        try:
            save_scenario(
                path, name=name, description=base.description, seed=seed,
                launch=launch, mission=mission, detection=detection,
                ground_truth_victims=base.ground_truth_victims,
                drone_overrides=base.drone_overrides,
            )
        except Exception as e:   # noqa: BLE001 — surface any save failure
            QMessageBox.critical(
                self, 'Save failed', f'Could not save scenario:\n{e}',
            )
            return
        self.scenarioSaved.emit(name)

    # --------------------------------------------------------- reload
    def reload_scenarios(self, scenarios: List[Scenario],
                         select_name: Optional[str] = None) -> None:
        """Replace the scenario list (after a Save As…) and re-select.

        Mission Control calls this once it has re-discovered the scenarios
        directory, so the freshly-saved scenario appears in the picker.
        """
        self._scenarios = scenarios
        self._scenario_combo.blockSignals(True)
        self._scenario_combo.clear()
        for s in scenarios:
            self._scenario_combo.addItem(s.name)
        if select_name is not None:
            idx = self._scenario_combo.findText(select_name)
            if idx >= 0:
                self._scenario_combo.setCurrentIndex(idx)
        self._scenario_combo.blockSignals(False)
        self._on_pick()

    # --------------------------------------------------------- enable
    def set_running(self, running: bool) -> None:
        """Disable the Run button (and the whole form) while a mission is alive."""
        self._run_btn.setEnabled(not running)
        for w, _ in list(self._launch_widgets.values()) + \
                    list(self._mission_widgets.values()) + \
                    list(self._detection_widgets.values()):
            w.setEnabled(not running)
        self._scenario_combo.setEnabled(not running)
        self._reset_btn.setEnabled(not running)
        self._save_as_btn.setEnabled(not running)
