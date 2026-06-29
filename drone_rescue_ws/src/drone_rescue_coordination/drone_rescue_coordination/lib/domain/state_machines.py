"""Typed state machines for Mission and Victim.

The legal `(stage, event) -> stage` transitions are encoded once in a
table at module level. ``transition()`` raises ``IllegalTransition``
when called with a (stage, event) pair the table doesn't cover, so
illegal transitions stop being silent re-entries and become typed
errors.

The ``MissionStage`` and ``VictimStage`` IntEnum values are kept
identical to the legacy values in ``mission_manager.py`` so callers
can migrate one at a time without a wire-format change.
"""

from __future__ import annotations

from enum import IntEnum


class IllegalTransition(RuntimeError):
    """Raised when ``transition()`` is asked for a (stage, event) pair
    not in the legal-transition table."""


class MissionStage(IntEnum):
    """Mirror of ``mission_manager.MissionStage``. Same int values so
    the legacy enum and this one are interchangeable during migration.
    """
    INIT = 0
    ARMING = 1
    DEPLOYING = 2
    SCANNING = 3
    INVESTIGATING = 4
    COMPLETE = 5
    ABORTED = 6


class VictimStage(IntEnum):
    """Mirror of ``mission_manager.VictimStage``."""
    DETECTED = 0
    INVESTIGATING = 1
    CONFIRMED = 2
    REJECTED = 3


class TransitionEvent(IntEnum):
    """Closed enum of triggers the mission and victim state machines
    accept. Adding a new trigger requires adding a row to the
    appropriate transition table; illegal triggers raise."""
    # Mission triggers
    LIFECYCLE_CONFIGURED = 1
    LIFECYCLE_ACTIVATED = 2
    SURVEY_STARTED = 3
    INVESTIGATE_DISPATCHED = 4
    SCAN_RESUMED = 5
    MISSION_COMPLETE = 6
    MISSION_TIMEOUT = 7
    MISSION_ABORTED = 8
    # Victim triggers
    CANDIDATE_DETECTED = 100
    INVESTIGATE_BEGAN = 101
    INVESTIGATE_COMPLETED = 102
    CONFIRMED = 103
    REJECTED = 104
    INVESTIGATE_FAILED = 105


# (current_stage, event) → next_stage
_MISSION_TABLE = {
    (MissionStage.INIT, TransitionEvent.LIFECYCLE_CONFIGURED): MissionStage.ARMING,
    (MissionStage.ARMING, TransitionEvent.LIFECYCLE_ACTIVATED): MissionStage.DEPLOYING,
    (MissionStage.DEPLOYING, TransitionEvent.SURVEY_STARTED): MissionStage.SCANNING,
    (MissionStage.SCANNING, TransitionEvent.INVESTIGATE_DISPATCHED): MissionStage.INVESTIGATING,
    (MissionStage.INVESTIGATING, TransitionEvent.SCAN_RESUMED): MissionStage.SCANNING,
    (MissionStage.INVESTIGATING, TransitionEvent.INVESTIGATE_DISPATCHED): MissionStage.INVESTIGATING,
    (MissionStage.SCANNING, TransitionEvent.MISSION_COMPLETE): MissionStage.COMPLETE,
    (MissionStage.INVESTIGATING, TransitionEvent.MISSION_COMPLETE): MissionStage.COMPLETE,
    (MissionStage.SCANNING, TransitionEvent.MISSION_TIMEOUT): MissionStage.COMPLETE,
    (MissionStage.INVESTIGATING, TransitionEvent.MISSION_TIMEOUT): MissionStage.COMPLETE,
    (MissionStage.SCANNING, TransitionEvent.MISSION_ABORTED): MissionStage.ABORTED,
    (MissionStage.INVESTIGATING, TransitionEvent.MISSION_ABORTED): MissionStage.ABORTED,
    (MissionStage.DEPLOYING, TransitionEvent.MISSION_ABORTED): MissionStage.ABORTED,
}

_VICTIM_TABLE = {
    # Initial discovery
    (VictimStage.DETECTED, TransitionEvent.INVESTIGATE_BEGAN): VictimStage.INVESTIGATING,
    # Confirm path
    (VictimStage.INVESTIGATING, TransitionEvent.CONFIRMED): VictimStage.CONFIRMED,
    # Pre-confirmed shortcut: detection_filter already reported the victim
    # as confirmed=True (multi-reporter, high confidence), so the saga
    # skips INVESTIGATING. Legacy mission_manager._on_candidate uses this.
    (VictimStage.DETECTED, TransitionEvent.CONFIRMED): VictimStage.CONFIRMED,
    # Compensation: investigation failed (e.g. drone went down) → back to DETECTED for re-auction
    (VictimStage.INVESTIGATING, TransitionEvent.INVESTIGATE_FAILED): VictimStage.DETECTED,
    # Reject path (low confidence, expired window)
    (VictimStage.DETECTED, TransitionEvent.REJECTED): VictimStage.REJECTED,
    (VictimStage.INVESTIGATING, TransitionEvent.REJECTED): VictimStage.REJECTED,
}


class MissionStateMachine:
    """Pure-function transition table for the mission's top-level
    lifecycle. Use ``MissionStateMachine.transition(stage, event)``
    instead of ``self._stage = MissionStage.X`` at the call site.
    """

    @staticmethod
    def transition(stage: MissionStage, event: TransitionEvent) -> MissionStage:
        next_stage = _MISSION_TABLE.get((stage, event))
        if next_stage is None:
            raise IllegalTransition(
                f'no mission transition from {stage.name} on {event.name}; '
                f'check _MISSION_TABLE in lib/domain/state_machines.py'
            )
        return next_stage

    @staticmethod
    def can_transition(stage: MissionStage, event: TransitionEvent) -> bool:
        return (stage, event) in _MISSION_TABLE


class VictimStateMachine:
    """Pure-function transition table for the per-victim saga."""

    @staticmethod
    def transition(stage: VictimStage, event: TransitionEvent) -> VictimStage:
        next_stage = _VICTIM_TABLE.get((stage, event))
        if next_stage is None:
            raise IllegalTransition(
                f'no victim transition from {stage.name} on {event.name}; '
                f'check _VICTIM_TABLE in lib/domain/state_machines.py'
            )
        return next_stage

    @staticmethod
    def can_transition(stage: VictimStage, event: TransitionEvent) -> bool:
        return (stage, event) in _VICTIM_TABLE
