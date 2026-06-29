"""Behaviour: first-class basis-behaviour port (L1 motor-schema).

The slides (Marcelletti, "Autonomous and Collaborative Robotics",
A.Y. 2025/26, pp. 88-90, 94-100) define behaviour-based control as a
set of distributed, interacting modules called behaviours, each a
tight stimulus->response coupling, combined into a system-level
behaviour. The five basis behaviours lived as module-level pure
functions in ``lib.domain.navigation`` (``compute_*``); they had no
shared interface, so adding or reweighting one meant editing the
blend function's hardwired five-slot signature.

This Protocol promotes a basis behaviour to a first-class object: a
named unit that maps a per-tick ``SurveyorSensors`` snapshot to a raw
2-D vector. Concrete implementations live in
``lib.domain.behaviours`` and delegate to the unchanged ``compute_*``
functions; a ``BehaviourRegistry`` (lib.domain.behaviour_registry)
holds them in an ordered, weighted, enable-able catalogue. Adding a
sixth behaviour is then a new class plus one ``register()`` call,
with no edit to the combination step.

3T boundary: ``LAYER_BOUNDARY = 'L1'``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, Tuple

if TYPE_CHECKING:
    from .surveyor_port import SurveyorSensors


LAYER_BOUNDARY = 'L1'   # 3T architecture annotation.


class Behaviour(Protocol):
    """A single basis behaviour of the motor-schema reactive layer.

    Implementations are pure: ``compute`` reads only the per-tick
    ``SurveyorSensors`` snapshot and the behaviour's own
    construction-time configuration, and returns a raw (unweighted,
    un-normalised) 2-D vector. Weighting and combination are the
    ``ArbitrationStrategy``'s job, not the behaviour's.

    ``name`` is the stable key under which the behaviour is registered
    and by which its weight is looked up; it must be unique within a
    ``BehaviourRegistry``.
    """

    name: str

    def compute(self, sensors: 'SurveyorSensors') -> Tuple[float, float]:
        """Return this behaviour's raw 2-D vector for the tick.

        Returns ``(0.0, 0.0)`` when the behaviour has nothing to
        contribute (no stimulus, missing prerequisite sensor).
        """
        ...
