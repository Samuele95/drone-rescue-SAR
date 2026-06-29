"""AffectMonitor: model-free "stuck/frustration" detector port.

The slides (Marcelletti, "Autonomous and Collaborative Robotics",
A.Y. 2025/26, Unit 10) describe a hybrid behaviour-based architecture
whose coordination layer carries an emotion-based mechanism that
detects when normal decision-making fails: "like in humans, emotions
highlight such situations, using temporal models of intentions
without needing prior knowledge of conditions or goals." A system
with no central world model cannot detect "I am stuck" by comparing
the world against a plan (it has no plan to compare against); instead
it watches the temporal pattern of how its behaviours/intentions are
being exploited and raises a signal when the pattern looks
pathological.

The ingredients were already scattered (the ``StuckRecoveryPolicy``
30-second rule, the ``SystemModeMachine`` NORMAL/DEGRADED/SAFE
pillars, the per-drone health monitor) but no single object watched
exploitation over time. This port names that object. It is
deliberately model-free: it consumes only a stream of
``ExploitationSample`` (which key is currently dominant, did it make
progress, and the ``Clock`` time) and answers two questions: how
frustrated is this key, and is it stuck. The escalation that follows
a stuck signal (climb, re-prioritise, escalate to the planner) stays
with the existing actuators; this port only detects.

3T boundary: ``LAYER_BOUNDARY = 'L2-L3'``, the same
executive-to-planner escalation seam as ``ExecutiveSupervisor``; a
sustained stuck signal is exactly the kind of exception the executive
reports upward.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol


LAYER_BOUNDARY = 'L2-L3'   # 3T architecture annotation.


@dataclass(frozen=True)
class ExploitationSample:
    """One tick's observation that ``key`` was the dominant
    behaviour/intention and whether it made progress.

    ``key`` is a stable identifier: a behaviour name
    (``'B5-goal-seek'``) or an intention id. ``now_sec`` is the
    ``Clock`` time of the observation (never the wall clock directly,
    so the monitor stays deterministic under the evaluation harness).
    ``made_progress`` is the caller's model-free verdict that the key
    advanced its goal this tick (e.g. the drone moved, coverage rose,
    a victim was confirmed).
    """

    key: str
    now_sec: float
    made_progress: bool


@dataclass(frozen=True)
class StuckSignal:
    """Raised when a key has been exploited without progress for at
    least the stuck threshold. ``stuck_for_s`` is the length of the
    current unproductive streak; ``unproductive_samples`` is how many
    observations it spans."""

    key: str
    stuck_for_s: float
    unproductive_samples: int


class AffectMonitor(Protocol):
    """Model-free failure detector over behaviour/intention exploitation.

    Pure with respect to time: every method's result is a function of
    the ``ExploitationSample`` stream alone, so it carries no world
    model and introduces no nondeterminism.
    """

    def observe(self, sample: ExploitationSample) -> None:
        """Record one tick of exploitation for ``sample.key``."""
        ...

    def frustration(self, key: str) -> float:
        """A 0..1 reading of how unproductive ``key`` has been: 0 when
        it just made progress (or is unknown), rising to 1.0 once its
        unproductive streak reaches the stuck threshold."""
        ...

    def is_stuck(self, key: str) -> Optional[StuckSignal]:
        """Return a ``StuckSignal`` when ``key``'s unproductive streak
        has reached the stuck threshold, else ``None``."""
        ...
