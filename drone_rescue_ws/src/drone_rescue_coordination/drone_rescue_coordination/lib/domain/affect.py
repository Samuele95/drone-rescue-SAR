"""ExploitationTracker: the model-free stuck/frustration detector.

Pure-Python implementation of ``AffectMonitor`` (``lib/ports/affect_monitor.py``).
Per registered key, the tracker maintains four scalars:

- ``last_seen_sec``: ``Clock`` time of the most recent ``observe``.
- ``last_progress_sec``: time of the most recent sample whose
  ``made_progress`` was True.
- ``streak_start``: when the current unproductive streak began,
  or ``None`` if the last sample made progress.
- ``unproductive_count``: number of consecutive unproductive samples
  in the current streak.

``frustration(key)`` is the clamped ratio of the current unproductive
duration to the configured stuck threshold; ``is_stuck(key)`` fires
once that ratio reaches 1.0. The detector takes no clock, no RNG, and
no perception input, only the temporal pattern of the
``ExploitationSample`` stream the caller hands it. That is what makes
it the Unit-10 "emotion" analogue: it notices "this is not working"
without ever modelling why.

It deliberately generalises the existing ``StuckRecoveryPolicy``'s
30-second movement rule from "the drone did not move" to "this
behaviour/intention has been dominant-but-unproductive for T seconds",
so the same signal works for goal-level stuckness an intention
workspace would care about.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

from drone_rescue_coordination.lib.ports.affect_monitor import (
    AffectMonitor,
    ExploitationSample,
    StuckSignal,
)


@dataclass
class _KeyState:
    last_seen_sec: float
    last_progress_sec: float
    streak_start: Optional[float]
    unproductive_count: int


@dataclass
class ExploitationTracker(AffectMonitor):
    """Per-key sliding pattern of behaviour/intention exploitation.

    ``stuck_threshold_s`` mirrors ``StuckRecoveryPolicy``'s default
    (30 s) so the new detector is calibrated to the existing one.
    """

    stuck_threshold_s: float = 30.0
    _state: Dict[str, _KeyState] = field(default_factory=dict)

    def observe(self, sample: ExploitationSample) -> None:
        st = self._state.get(sample.key)
        if st is None:
            self._state[sample.key] = _KeyState(
                last_seen_sec=sample.now_sec,
                last_progress_sec=sample.now_sec,
                streak_start=None if sample.made_progress else sample.now_sec,
                unproductive_count=0 if sample.made_progress else 1,
            )
            return
        st.last_seen_sec = sample.now_sec
        if sample.made_progress:
            st.last_progress_sec = sample.now_sec
            st.streak_start = None
            st.unproductive_count = 0
        else:
            if st.streak_start is None:
                st.streak_start = sample.now_sec
                st.unproductive_count = 0
            st.unproductive_count += 1

    def frustration(self, key: str) -> float:
        st = self._state.get(key)
        if st is None or st.streak_start is None:
            return 0.0
        if self.stuck_threshold_s <= 0:
            return 1.0
        ratio = (st.last_seen_sec - st.streak_start) / self.stuck_threshold_s
        if ratio < 0.0:
            return 0.0
        if ratio > 1.0:
            return 1.0
        return ratio

    def is_stuck(self, key: str) -> Optional[StuckSignal]:
        st = self._state.get(key)
        if st is None or st.streak_start is None:
            return None
        duration = st.last_seen_sec - st.streak_start
        if duration < self.stuck_threshold_s:
            return None
        return StuckSignal(
            key=key,
            stuck_for_s=duration,
            unproductive_samples=st.unproductive_count,
        )

    # Diagnostic helpers (not on the Protocol, for inspection).
    def known_keys(self) -> tuple:
        """Return the keys observed so far, in insertion order."""
        return tuple(self._state.keys())
