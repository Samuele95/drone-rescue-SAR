"""MissionCollector: pure-Python state accumulator for mission_recorder.

Extracts the subscriber-side state of ``MissionRecorder`` (the
``_on_event`` / ``_on_coverage`` / ``_on_victim`` / ``_on_peer`` /
``_on_health`` handlers and their parallel-list series) into a
Node-free class so the per-message folding logic can be exercised
without ``rclpy.init()``.

The recorder Node continues to own subscriptions, lifecycle, and
the JSONL emit; it just forwards each message to a collector and
reads the accumulated series back at finalize time. Re-uses the
``_DroneSeries`` shape that ``mission_recorder._finalize`` was
already serialising via ``to_dict``.

Time stamps come from the caller, so the collector is clock-free and
tests can hand it deterministic ``t_s`` floats.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple


class DroneSeries:
    """Per-drone parallel-list state. Lifted verbatim from
    ``mission_recorder.py`` so the serialised ``to_dict`` schema is
    untouched."""

    __slots__ = ('battery', 'task', 'anomaly', 'wp_index', 'wp_total',
                 'position')

    def __init__(self) -> None:
        self.battery: List[Tuple[float, float]] = []
        self.task: List[Tuple[float, int]] = []
        self.anomaly: List[Tuple[float, float]] = []
        self.wp_index: List[Tuple[float, int]] = []
        self.wp_total: int = 0
        self.position: List[Tuple[float, float, float]] = []

    def to_dict(self) -> dict:
        return {
            'battery': self.battery,
            'task': self.task,
            'anomaly': self.anomaly,
            'wp_index': self.wp_index,
            'wp_total': self.wp_total,
            'position': self.position,
        }


class MissionCollector:
    """Accumulates per-message state for one mission run.

    Pure Python, no rclpy imports. Each ``record_*`` method takes
    the relative-second timestamp (``t_s``) plus the message-derived
    fields the recorder previously read off the ROS message directly.
    """

    def __init__(self, drone_names: List[str]) -> None:
        self.coverage_pct: List[Tuple[float, float]] = []
        self.cumulative_confirmed: List[Tuple[float, int]] = []
        self.candidates_count: List[Tuple[float, int]] = []
        self.drone_series: Dict[str, DroneSeries] = {
            d: DroneSeries() for d in drone_names
        }
        self.events: List[dict] = []
        # Latest VictimCandidate snapshot per id, kept opaque (we only
        # need ``.position.x|y`` and ``.confirmed`` downstream).
        self.victims: Dict[int, object] = {}

    # ----------------------------------------------------------- record
    def record_event(self, t_s: float, event: dict) -> None:
        """``event`` is the already-projected dict, exactly the
        shape ``mission_recorder._on_event`` produced. Caller projects
        the ROS message; the collector just appends."""
        self.events.append(event)

    def record_coverage(self, t_s: float, percentage_covered: float,
                        victims_found: int) -> None:
        self.coverage_pct.append((t_s, float(percentage_covered)))
        self.candidates_count.append((t_s, int(victims_found)))

    def record_victim(self, t_s: float, candidate_id: int,
                      candidate: object) -> None:
        """Stores the candidate keyed by id and refreshes the running
        cumulative-confirmed count. Callers must hand in the raw
        ``VictimCandidate`` so the downstream finaliser still gets
        ``.position`` and ``.confirmed``."""
        self.victims[int(candidate_id)] = candidate
        confirmed_ids = sum(
            1 for v in self.victims.values() if getattr(v, 'confirmed', False)
        )
        self.cumulative_confirmed.append((t_s, confirmed_ids))

    def record_peer(self, t_s: float, drone: str, *,
                    battery: float, task_type: int,
                    wp_index: int, wp_total: int,
                    pose_x: float, pose_y: float) -> None:
        s = self.drone_series.get(drone)
        if s is None:
            return
        s.battery.append((t_s, float(battery)))
        s.task.append((t_s, int(task_type)))
        s.wp_index.append((t_s, int(wp_index)))
        if wp_total > 0:
            s.wp_total = int(wp_total)
        # Skip the all-zero default pose that some peers emit before
        # odom is wired up (matches pre-extraction behaviour).
        if not (pose_x == 0.0 and pose_y == 0.0 and len(s.position) == 0):
            s.position.append((t_s, float(pose_x), float(pose_y)))

    def record_health(self, t_s: float, drone: str,
                      anomaly_score: float) -> None:
        s = self.drone_series.get(drone)
        if s is None:
            return
        s.anomaly.append((t_s, float(anomaly_score)))

    # ----------------------------------------------------------- read
    def per_drone_to_dict(self) -> Dict[str, dict]:
        return {d: s.to_dict() for d, s in self.drone_series.items()}
