"""MissionLifecycleState: the operator UI's view of the mission lifecycle
as a typed state machine.

The mission lifecycle as seen by the operator
(IDLE, SPAWNING, ACTIVATING, RUNNING, STOPPING/DONE, with ERROR) was an
implicit machine: scattered ``set_state('RUNNING', ...)`` string calls with
no transition validation. This makes it explicit and typed, mirroring the
coordination layer's ``MissionStateMachine`` / ``IllegalTransition`` stance,
and unit-testable without a running Qt application.

The VO is strict by default: ``transition`` raises ``IllegalUiTransition``
on an illegal move. The ROS-driven GUI adapter passes ``raise_on_invalid=
False`` so a rare lifecycle race logs-and-forces rather than crashing the
operator console (the strict default is for tests and future call sites that
can guarantee legality).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Set


class MissionPhase(str, Enum):
    """Closed set of operator-observable mission phases. ``str``-Enum so it
    round-trips with the existing label/colour dicts keyed on these strings."""
    IDLE = 'IDLE'
    SPAWNING = 'SPAWNING'
    ACTIVATING = 'ACTIVATING'
    RUNNING = 'RUNNING'
    STOPPING = 'STOPPING'
    DONE = 'DONE'
    ERROR = 'ERROR'


# Any phase may fail to ERROR; ERROR / DONE return to IDLE for a new run.
VALID_TRANSITIONS: Dict[MissionPhase, Set[MissionPhase]] = {
    MissionPhase.IDLE:       {MissionPhase.SPAWNING},
    MissionPhase.SPAWNING:   {MissionPhase.ACTIVATING, MissionPhase.ERROR},
    MissionPhase.ACTIVATING: {MissionPhase.RUNNING, MissionPhase.ERROR},
    MissionPhase.RUNNING:    {MissionPhase.STOPPING, MissionPhase.DONE,
                              MissionPhase.ERROR},
    MissionPhase.STOPPING:   {MissionPhase.IDLE, MissionPhase.DONE,
                              MissionPhase.ERROR},
    MissionPhase.DONE:       {MissionPhase.IDLE},
    MissionPhase.ERROR:      {MissionPhase.IDLE},
}


class IllegalUiTransition(Exception):
    """Raised when a mission-lifecycle transition violates VALID_TRANSITIONS."""

    def __init__(self, frm: MissionPhase, to: MissionPhase):
        super().__init__(f'illegal UI mission transition {frm.value} → {to.value}')
        self.frm = frm
        self.to = to


@dataclass(frozen=True)
class MissionLifecycleState:
    """Frozen snapshot of the operator-observed mission phase + detail."""
    phase: MissionPhase = MissionPhase.IDLE
    detail: str = ''

    def transition(
        self, to: MissionPhase, detail: str = '',
        *, raise_on_invalid: bool = True,
    ) -> 'MissionLifecycleState':
        """Return the next state. Raises ``IllegalUiTransition`` on an illegal
        move unless ``raise_on_invalid`` is False (then it forces the move;
        the GUI adapter uses this so a lifecycle race never crashes the
        console)."""
        if to not in VALID_TRANSITIONS.get(self.phase, set()):
            if raise_on_invalid:
                raise IllegalUiTransition(self.phase, to)
        return MissionLifecycleState(phase=to, detail=detail)

    def is_active(self) -> bool:
        """True while a mission is alive (Stop is meaningful)."""
        return self.phase in (
            MissionPhase.SPAWNING, MissionPhase.ACTIVATING, MissionPhase.RUNNING,
        )
