"""Per-mission recorder: writes one JSONL summary at MISSION_COMPLETE.

3T architecture: cross-cutting infrastructure in the Operator UI bounded
context. This node does not implement any of the three 3T layers
(Behavioural / Executive / Deliberative). It is post-hoc data
acquisition for offline analysis: it subscribes to operator-relevant
topics emitted by the coordination layer's L1/L2/L3 nodes and writes
JSONL summaries the Mission Control GUI later parses. Per the
Marcelletti slides' taxonomy (pp. 33, 85-86), the 3T model describes
the real-time control stack; reporting / persistence infrastructure
sits outside that stack and is cross-cutting. This file belongs to a
separate bounded context and is intentionally excluded from the layer
annotations applied to the control-stack nodes.

Spawned as a regular Node by the launch when `record_run:=true`. Subscribes
to every operator-relevant topic and accumulates a time series + the full
event stream. Writes the summary on MISSION_COMPLETE / MISSION_TIMEOUT, on
SIGTERM (operator clicked Stop in Mission Control), or on KeyboardInterrupt.

The file path is `<runs_dir>/<UTC>__<pattern>__<scenario>.json`. Mission
Control parses these for the Past Runs and Compare Runs tabs; they are also
human-readable for offline analysis.

True/false-positive scoring: for each VICTIM_CONFIRMED event we find the
nearest entry in `ground_truth_victims` (loaded from the scenario YAML); if
within `gt_match_radius_m` it is a true positive. Unmatched confirmed events
are false positives, unmatched ground-truth victims are false negatives.
"""

from __future__ import annotations

import json
import math
import os
import signal
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from drone_rescue_msgs.msg import (
    MissionEvent, CoverageMetrics, DronePeerState, DroneHealth,
    VictimCandidate,
)

# RosClock import hoisted to module top (was inline inside the _now_sec
# lazy fallback). Lazy instantiation is preserved so subclassing tests
# that pre-set self._time still win.
from drone_rescue_coordination.lib.ros_adapter.ros_clock import RosClock
from drone_rescue_coordination.lib.composition import bind_composition
# Pure-Python collector lives here so the per-message folding can be
# unit-tested without rclpy.init().
from .lib.mission_collector import MissionCollector, DroneSeries as _DroneSeries  # noqa: F401  (back-compat re-export)


# Re-export the canonical fleet constant; the legacy local symbol stays
# as a list alias.
from drone_rescue_ui_common.constants import DEFAULT_DRONE_NAMES as _DEFAULT_DRONE_NAMES_TUPLE
_DEFAULT_DRONE_NAMES = list(_DEFAULT_DRONE_NAMES_TUPLE)


class MissionRecorder(Node):
    # composition kwarg: when provided, composition.clock pre-populates
    # self._time so the lazy-construct branch in _now_sec short-circuits.
    def __init__(self, *, composition=None, scenario_repo=None):
        super().__init__('mission_recorder')
        if composition is not None and composition.clock is not None:
            self._time = composition.clock
        # scenario_repo injectable. When None, fall back to a fresh
        # YamlScenarioRepository (production default). Tests pass
        # InMemoryScenarioRepository to avoid touching the filesystem.
        if scenario_repo is None and composition is not None:
            scenario_repo = composition.scenario_repo
        if scenario_repo is None:
            from drone_rescue_mission_control.persistence import (
                YamlScenarioRepository,
            )
            scenario_repo = YamlScenarioRepository()
        self._scenario_repo_instance = scenario_repo

        # --- params ----------------------------------------------------
        self.declare_parameter('runs_dir', os.path.expanduser(
            '~/.drone_rescue/runs',
        ))
        self.declare_parameter('scenario_yaml', '')      # path or ''
        self.declare_parameter('scenario_name', 'unknown')
        self.declare_parameter('coverage_pattern', 'unknown')
        self.declare_parameter('allocation_strategy', 'greedy_auction')
        self.declare_parameter('drone_names', _DEFAULT_DRONE_NAMES)
        # Distance from a confirmed victim to the nearest ground-truth
        # victim that still counts as a true positive. The world's victim
        # bodies are 1-2 m wide; the camera projection is accurate to
        # ~3-5 m at 25 m altitude, so 8 m gives a comfortable margin.
        self.declare_parameter('gt_match_radius_m', 8.0)

        self.runs_dir = Path(str(self.get_parameter('runs_dir').value))
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        self.scenario_yaml = str(self.get_parameter('scenario_yaml').value) or ''
        self.scenario_name = str(self.get_parameter('scenario_name').value)
        self.coverage_pattern = str(self.get_parameter('coverage_pattern').value)
        self.allocation_strategy = str(
            self.get_parameter('allocation_strategy').value)
        self.drone_names: List[str] = list(self.get_parameter('drone_names').value)
        self.gt_match_radius_m = float(self.get_parameter('gt_match_radius_m').value)

        # --- state -----------------------------------------------------
        self._t0_wall = datetime.now(timezone.utc)
        self._t0_sec: Optional[float] = None      # sim seconds at first odom-ish event
        self._ended = False
        self._end_reason = 'OPERATOR_STOP'

        # Per-message folding lives in MissionCollector. The recorder
        # still aliases the lists (self._coverage_pct =
        # self._collector.coverage_pct) so _finalize's reads keep their
        # pre-extraction shape; appends happen through the collector's
        # record_* API.
        self._collector = MissionCollector(self.drone_names)
        self._coverage_pct = self._collector.coverage_pct
        self._cumulative_confirmed = self._collector.cumulative_confirmed
        self._candidates_count = self._collector.candidates_count
        self._drone_series = self._collector.drone_series
        self._events = self._collector.events
        self._victims = self._collector.victims

        # Ground truth victim positions (from scenario YAML, if provided).
        # Each: {'id': int, 'position': [x, y, z]}
        self._ground_truth: List[dict] = self._load_ground_truth()

        # Snapshot of params (also from scenario YAML)
        self._scenario_params_snapshot = self._load_scenario_params()

        # --- subscriptions --------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=10,
        )
        peer_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1,
        )
        events_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, depth=200,
        )
        self.create_subscription(
            MissionEvent, '/mission/events', self._on_event, events_qos,
        )
        self.create_subscription(
            CoverageMetrics, '/coverage/metrics', self._on_coverage, 10,
        )
        self.create_subscription(
            VictimCandidate, '/victims/candidates', self._on_victim, 10,
        )
        for d in self.drone_names:
            self.create_subscription(
                DronePeerState, f'/{d}/peer_state',
                lambda msg, n=d: self._on_peer(n, msg), peer_qos,
            )
            self.create_subscription(
                DroneHealth, f'/{d}/health',
                lambda msg, n=d: self._on_health(n, msg), 10,
            )

        # Record the real runtime performance envelope (RTF / CPU /
        # node-tree RSS) at 1 Hz so the run JSON carries measured numbers.
        # Null envelopes when unmeasured, never fabricated.
        from drone_rescue_mission_control.perf_sampler import PerfSampler
        self._perf = PerfSampler()
        self.create_timer(1.0, self._sample_perf)

        # SIGTERM (Mission Control kill) → flush partial summary
        signal.signal(signal.SIGTERM, self._on_sigterm)

        self.get_logger().info(
            f'mission_recorder up — runs_dir={self.runs_dir}, '
            f'scenario={self.scenario_name}, pattern={self.coverage_pattern}'
        )

    # -------------------------------------------------------- helpers
    # Both helpers consume the ScenarioRepository.by_path Protocol so the
    # YAML I/O boundary lives in one place. Tests can substitute an
    # InMemoryScenarioRepository; the recorder is not coupled to a
    # YAML-file-on-disk.
    def _scenario_repo(self):
        """Return the injected scenario repo (via __init__ kwarg or
        CompositionRoot.scenario_repo) instead of constructing a fresh
        YamlScenarioRepository."""
        return self._scenario_repo_instance

    def _load_ground_truth(self) -> List[dict]:
        if not self.scenario_yaml or not os.path.isfile(self.scenario_yaml):
            return []
        try:
            scenario = self._scenario_repo().by_path(self.scenario_yaml)
            return [
                {'id': v.id, 'position': list(v.position)}
                for v in scenario.ground_truth_victims
            ]
        except Exception as e:
            self.get_logger().warn(f'could not load ground truth: {e}')
            return []

    def _load_scenario_params(self) -> dict:
        if not self.scenario_yaml or not os.path.isfile(self.scenario_yaml):
            return {}
        try:
            scenario = self._scenario_repo().by_path(self.scenario_yaml)
            return {
                'name': scenario.name,
                'description': scenario.description,
                'seed': scenario.seed,
                'launch': dict(scenario.launch),
                'mission': dict(scenario.mission),
                'detection': dict(scenario.detection),
                'drone_overrides': dict(scenario.drone_overrides),
            }
        except Exception:
            return {}

    def _sample_perf(self) -> None:
        """1 Hz performance sample. Wall time is monotonic; sim time is
        the recorder's clock, and their ratio is the RTF."""
        import time
        self._perf.sample(self._now_sec(), time.monotonic())

    def _now_sec(self) -> float:
        # Consume the Clock port. Lazy-construct the RosClock on first
        # access so existing tests that subclass this node (passing a
        # mocked clock pre-construction) can override by setting
        # self._time directly. The guard reads _time, not _clock: a
        # _clock lookup always finds the rclpy Node._clock and shadows
        # the lazy construct.
        clock = getattr(self, '_time', None)
        if clock is None:
            clock = RosClock(self)
            self._time = clock
        return clock.now_sec()

    def _rel_t(self) -> float:
        now = self._now_sec()
        if self._t0_sec is None:
            self._t0_sec = now
            return 0.0
        return now - self._t0_sec

    # -------------------------------------------------------- callbacks
    # Each handler projects the ROS message into a plain shape and
    # delegates to the pure-Python collector.
    def _on_event(self, msg: MissionEvent) -> None:
        t = self._rel_t()
        event = {
            't': t,
            'type': msg.event_type,
            'drone': msg.drone_name,
            'detail': msg.detail,
            'victim_id': int(msg.victim_id),
            'position': [msg.position.x, msg.position.y, msg.position.z],
            'severity': int(msg.severity),
            # Persist the numeric confidence directly so downstream
            # analytics (notably the ROC sweep) doesn't need to re-parse
            # it out of `detail`. 0.0 for events where confidence is N/A
            # (everything except CANDIDATE_DETECTED).
            'confidence': float(getattr(msg, 'confidence', 0.0)),
        }
        self._collector.record_event(t, event)
        if msg.event_type in ('MISSION_COMPLETE', 'MISSION_TIMEOUT') and not self._ended:
            self._end_reason = msg.event_type
            self._finalize()

    def _on_coverage(self, msg: CoverageMetrics) -> None:
        self._collector.record_coverage(
            self._rel_t(),
            percentage_covered=float(msg.percentage_covered),
            victims_found=int(msg.victims_found),
        )

    def _on_victim(self, msg: VictimCandidate) -> None:
        self._collector.record_victim(
            self._rel_t(), int(msg.candidate_id), msg,
        )

    def _on_peer(self, drone: str, msg: DronePeerState) -> None:
        self._collector.record_peer(
            self._rel_t(), drone,
            battery=float(msg.battery),
            task_type=int(msg.task_type),
            wp_index=int(msg.wp_index),
            wp_total=int(msg.wp_total),
            pose_x=float(msg.pose.position.x),
            pose_y=float(msg.pose.position.y),
        )

    def _on_health(self, drone: str, msg: DroneHealth) -> None:
        self._collector.record_health(
            self._rel_t(), drone, float(msg.anomaly_score),
        )

    # -------------------------------------------------------- finalization
    def _on_sigterm(self, signum, frame) -> None:
        self.get_logger().info('mission_recorder caught SIGTERM, finalizing')
        if not self._ended:
            self._end_reason = 'OPERATOR_STOP'
            self._finalize()
        rclpy.shutdown()
        sys.exit(0)

    def _finalize(self) -> None:
        if self._ended:
            return
        self._ended = True

        ended_wall = datetime.now(timezone.utc)
        duration_s = (ended_wall - self._t0_wall).total_seconds()
        ts_id = self._t0_wall.strftime('%Y-%m-%d_%H%M%S')

        # Score true/false positive against ground truth.
        # The authoritative "confirmed victim" set is the saga's
        # VICTIM_CONFIRMED events: each carries the victim position and fires
        # for BOTH confirmation paths (the saga's INVESTIGATE->CONFIRM orbit AND
        # the detection_filter multi-view auto-confirm). The previous scoring
        # read only /victims/candidates.confirmed, the transient flag set solely
        # by the detection_filter's multi-view gate (>=2 distinct drones), which
        # sector-scanning rarely satisfies, so a victim the saga genuinely
        # confirmed metres from ground truth scored true_positives=0. This is
        # SagaConfirmedVictim; its GT-matched subset is GroundTruthMatchedVictim.
        from .lib.run_finaliser import saga_confirmed_positions
        saga_confirmed = saga_confirmed_positions(self._events)
        # Defensive back-compat: also honour any candidate that flipped
        # confirmed=True without (or before) its event was recorded.
        for vid, v in self._victims.items():
            if getattr(v, 'confirmed', False):
                saga_confirmed.setdefault(
                    int(vid), (v.position.x, v.position.y))
        confirmed_positions = list(saga_confirmed.items())
        gt = [
            (gt['id'], (gt['position'][0], gt['position'][1]))
            for gt in self._ground_truth
        ]
        tp, fp, fn = self._score(confirmed_positions, gt, self.gt_match_radius_m)

        # Single-fold accumulation. Replaces five separate filtered
        # passes over self._events; reuses confirm_t_by_id in
        # _compute_detection_latency so the sixth pass goes too.
        from .lib.run_finaliser import fold_events
        evt_fold = fold_events(self._events)
        first_detection = evt_fold.first_detection_t
        first_confirm = evt_fold.first_confirm_t
        drone_down_events = list(evt_fold.drone_down_events)
        sector_reassignments = evt_fold.sector_reassignments
        # victims_confirmed counts the saga-confirmed set (above), not
        # just the transient detection_filter candidate flag.
        confirmed = len(saga_confirmed)
        rejected = evt_fold.rejected

        # The collector's cumulative_confirmed series counts candidates that
        # *currently* carry the transient ``confirmed`` flag, so it bounces
        # 0->1->0 and never exceeds 1 even when the saga confirmed several
        # victims. Rebuild it from the monotonic VICTIM_CONFIRMED events (the
        # same authoritative set the summary uses) so the report's cumulative
        # and survival curves are correct.
        _confirm_ts = sorted(evt_fold.confirm_t_by_id.values())
        self._cumulative_confirmed = [(t, i + 1) for i, t in enumerate(_confirm_ts)]

        final_cov = self._coverage_pct[-1][1] if self._coverage_pct else 0.0

        # Standard classifier scores from TP/FP/FN. Guard against div-by-0
        # so a zero-detection run still produces a numeric (0.0) metric
        # rather than None, which keeps downstream aggregation simple.
        n_tp, n_fp, n_fn = len(tp), len(fp), len(fn)
        precision = n_tp / max(n_tp + n_fp, 1)
        recall = n_tp / max(n_tp + n_fn, 1)
        f1_score = (
            2.0 * precision * recall / (precision + recall)
            if (precision + recall) > 0 else 0.0
        )

        # First sim-second the coverage curve crosses each threshold.
        # None if it never does.
        ttc = {p: self._first_crossing(self._coverage_pct, p)
               for p in (50.0, 80.0, 90.0)}

        # Joules per percent of disk covered. We treat each drone's
        # battery as 1 unit of (notional) energy; total energy spent ≈
        # sum(initial − final). final_coverage_pct is the denominator
        # (clamped at 1e-3 to avoid div-by-0 on flat-zero runs).
        #
        # The "initial" reference is the simulation's full-charge value
        # (1.0), NOT s.battery[0][1]: the first sample can arrive
        # well after t=0 if the recorder spawns late, and using the
        # late first sample silently under-counts energy. Using a
        # constant 1.0 makes the metric reproducible across runs with
        # different recorder startup timing. If a future scenario
        # starts a drone at a degraded charge, this assumption needs
        # to change. We log a WARN if the first observed battery is
        # already noticeably below 1.0 so a reviewer can see when this
        # assumption was strained. See docs/v5-release.md.
        total_battery_spent = 0.0
        for d_name, s in self._drone_series.items():
            if not s.battery:
                continue
            first_obs = float(s.battery[0][1])
            final = float(s.battery[-1][1])
            spent = max(0.0, 1.0 - final)
            total_battery_spent += spent
            if first_obs < 0.95:
                self.get_logger().warning(
                    f'{d_name}: first observed battery {first_obs:.2f} '
                    f'< 0.95 — energy_per_coverage_pct_J assumes a 1.0 '
                    f'initial reference; treat the metric as a lower '
                    f'bound for this run.'
                )
        energy_per_pct = total_battery_spent / max(final_cov, 1e-3)

        # Jain fairness over per-drone total active time (anything that's
        # not IDLE = task_type 5 in TaskAssignment). 1.0 = perfectly fair,
        # 1/n = one drone did everything. Cite Jain, Chiu, Hawe 1984.
        per_drone_active_s: List[float] = []
        for d, s in self._drone_series.items():
            active = self._integrate_active_time(s.task, idle_task_type=5)
            per_drone_active_s.append(active)
        if per_drone_active_s and sum(per_drone_active_s) > 0:
            num = sum(per_drone_active_s) ** 2
            den = len(per_drone_active_s) * sum(t * t for t in per_drone_active_s)
            jain = num / den if den > 0 else 0.0
        else:
            jain = 0.0

        # Per-victim detection latency: for each TP, the sim-second the
        # FIRST drone passed within gt_match_radius_m of the truth
        # location. Latency = confirm_t − pass_t.
        # Reuse the fold's confirm_t_by_id so the latency helper doesn't
        # walk self._events a sixth time.
        latencies = self._compute_detection_latency(
            tp, confirm_t_by_id=evt_fold.confirm_t_by_id,
        )

        summary = {
            'metadata': {
                'started_at': self._t0_wall.isoformat(),
                'ended_at': ended_wall.isoformat(),
                'duration_s': duration_s,
                'ended_by': self._end_reason,
                'scenario': self.scenario_name,
                'pattern': self.coverage_pattern,
                'allocation_strategy': self.allocation_strategy,
                'params_snapshot': self._scenario_params_snapshot,
                'ground_truth_victims': self._ground_truth,
            },
            'summary': {
                'candidates_emitted': len(self._victims),
                'victims_confirmed': confirmed,
                'victims_rejected': rejected,
                'true_positives': n_tp,
                'false_positives': n_fp,
                'false_negatives': n_fn,
                # Make the SagaConfirmed ⊋ GroundTruthMatched divergence
                # first-class. A drone's saga can CONFIRM a victim that matches
                # no ground truth (a near-pad false positive); scoring it only
                # as true_positives=0 hid the mechanism. These name it directly:
                # saga_confirmed (the saga's own count) ⊇ ground_truth_matched
                # (within gt_match_radius_m), and their difference is the count
                # of confirmed-but-unmatched false positives.
                'saga_confirmed_count': confirmed,
                'ground_truth_matched_count': n_tp,
                'saga_confirmed_not_ground_truth_matched': n_fp,
                'precision': precision,
                'recall': recall,
                'f1_score': f1_score,
                'matched_pairs': tp,
                'unmatched_confirmed_ids': fp,
                'unmatched_ground_truth_ids': fn,
                'time_to_first_detection_s': first_detection,
                'time_to_first_confirm_s': first_confirm,
                'time_to_coverage_50pct_s': ttc[50.0],
                'time_to_coverage_80pct_s': ttc[80.0],
                'time_to_coverage_90pct_s': ttc[90.0],
                'energy_per_coverage_pct_J': energy_per_pct,
                'task_fairness_jain': jain,
                'detection_latency_per_victim_s': latencies,
                'drones_down': len(drone_down_events),
                'drone_down_events': drone_down_events,
                'sector_reassignments': sector_reassignments,
                'final_coverage_pct': final_cov,
            },
            # Measured runtime envelope (RTF / CPU / RSS); null when
            # unmeasured, never fabricated.
            'performance': self._perf.summary(),
            'time_series': {
                'coverage_pct': self._coverage_pct,
                'candidates_count': self._candidates_count,
                'cumulative_confirmed': self._cumulative_confirmed,
                **{d: s.to_dict() for d, s in self._drone_series.items()},
            },
            'events': self._events,
        }

        # Filename carries pattern + allocation + scenario so a sweep over
        # allocation strategies does not collide (bench resume is filename-
        # keyed). See bench.py _trial_filename_glob.
        out = self.runs_dir / (
            f'{ts_id}__{self.coverage_pattern}__{self.allocation_strategy}'
            f'__{self.scenario_name}.json'
        )
        try:
            with open(out, 'w') as f:
                json.dump(summary, f, indent=2)
            self.get_logger().info(f'wrote run summary: {out}')
        except Exception as e:
            self.get_logger().error(f'failed to write summary: {e}')

    # ---------------------------------------------------------- metric helpers
    # Pure-Python metric helpers moved to lib/run_finaliser.py so they're
    # unit-testable without rclpy.init(). The methods below are thin shims
    # that adapt the legacy MissionRecorder-bound signatures to the new
    # module-level functions.
    @staticmethod
    def _first_crossing(series, threshold):
        from .lib.run_finaliser import first_crossing
        return first_crossing(series, threshold)

    @staticmethod
    def _integrate_active_time(task_series, idle_task_type):
        from .lib.run_finaliser import integrate_active_time
        return integrate_active_time(task_series, idle_task_type)

    def _compute_detection_latency(self, tp_pairs, *, confirm_t_by_id=None):
        """Caller (_finalize) passes the confirm_t_by_id map from the
        single event fold; falls back to a local pass for back-compat
        callers."""
        from .lib.run_finaliser import compute_detection_latency
        if confirm_t_by_id is None:
            confirm_t_by_id = {}
            for e in self._events:
                if e.get('type') == 'VICTIM_CONFIRMED':
                    cid = int(e.get('victim_id') or 0)
                    if cid > 0 and cid not in confirm_t_by_id:
                        confirm_t_by_id[cid] = float(e['t'])
        gt_pos = {
            gt['id']: (gt['position'][0], gt['position'][1])
            for gt in self._ground_truth
        }
        drone_positions = {
            name: list(s.position) for name, s in self._drone_series.items()
        }
        return compute_detection_latency(
            tp_pairs=tp_pairs,
            confirm_t_by_id=confirm_t_by_id,
            drone_positions_by_drone=drone_positions,
            gt_pos_by_id=gt_pos,
            gt_match_radius_m=self.gt_match_radius_m,
        )

    def _score(self, confirmed, gt, radius_m):
        from .lib.run_finaliser import score
        return score(confirmed, gt, radius_m)


def main(args=None):
    rclpy.init(args=args)
    # Single point of construction; the bind_composition helper supplies
    # the Clock port + adapters.
    node = bind_composition(MissionRecorder())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node._ended:
            node._end_reason = 'OPERATOR_STOP'
            node._finalize()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
