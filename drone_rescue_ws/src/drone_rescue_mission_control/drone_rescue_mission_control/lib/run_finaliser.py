"""Pure-Python metric helpers extracted from MissionRecorder.

These helpers were nested inside the
`mission_recorder.MissionRecorder` LifecycleNode and reachable only
through a ROS node. Lifting them out makes them unit-testable.

The mission_recorder still owns the runtime / subscriber side; the
node calls into these helpers at finalize time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple


@dataclass(frozen=True)
class EventFoldResult:
    """Per-mission event aggregates produced by ``fold_events``.

    Replaces 5 separate filtered passes in
    ``mission_recorder._finalize`` (plus a 6th in
    ``_compute_detection_latency``) with one O(|events|) walk.
    """
    first_detection_t: Optional[float] = None
    first_confirm_t: Optional[float] = None
    drone_down_events: Tuple[Dict[str, Any], ...] = ()
    sector_reassignments: int = 0
    rejected: int = 0
    # candidate_id → first VICTIM_CONFIRMED time. Reuses the same
    # walk so ``_compute_detection_latency`` doesn't need a 6th pass.
    confirm_t_by_id: Dict[int, float] = field(default_factory=dict)


def fold_events(events: Iterable[Dict[str, Any]]) -> EventFoldResult:
    """Single pass over the event log; accumulates every derived
    quantity ``mission_recorder._finalize`` needs.

    Collapses what used to be five filtered
    `[e for e in events if e['type'] == ...]` walks (plus a sixth
    in ``compute_detection_latency``) into one accumulator. The
    result fields mirror the legacy variable names so the recorder
    body reads as a 1-to-1 substitution.
    """
    first_detection_t: Optional[float] = None
    first_confirm_t: Optional[float] = None
    drone_down: List[Dict[str, Any]] = []
    sector_reassignments = 0
    rejected = 0
    confirm_t_by_id: Dict[int, float] = {}
    for e in events:
        et = e.get('type')
        if et == 'CANDIDATE_DETECTED':
            if first_detection_t is None:
                first_detection_t = float(e['t'])
        elif et == 'VICTIM_CONFIRMED':
            if first_confirm_t is None:
                first_confirm_t = float(e['t'])
            cid = int(e.get('victim_id') or 0)
            if cid > 0 and cid not in confirm_t_by_id:
                confirm_t_by_id[cid] = float(e['t'])
        elif et == 'DRONE_DOWN':
            drone_down.append({
                'drone': e.get('drone', ''),
                't_s': float(e['t']),
                'reason': e.get('detail', ''),
            })
        elif et == 'SECTOR_REASSIGNED':
            sector_reassignments += 1
        elif et == 'CANDIDATE_REJECTED':
            rejected += 1
    return EventFoldResult(
        first_detection_t=first_detection_t,
        first_confirm_t=first_confirm_t,
        drone_down_events=tuple(drone_down),
        sector_reassignments=sector_reassignments,
        rejected=rejected,
        confirm_t_by_id=confirm_t_by_id,
    )


def first_crossing(
    series: Iterable[Tuple[float, float]],
    threshold: float,
) -> Optional[float]:
    """First sim-second `series` (an iterable of (t, value)) crosses
    `threshold`. Returns None if it never crosses."""
    for t, v in series:
        if v >= threshold:
            return float(t)
    return None


def integrate_active_time(
    task_series: List[Tuple[float, int]],
    idle_task_type: int,
) -> float:
    """Total seconds the drone spent in any non-idle task.

    Treats `task_series` as a step function: between consecutive
    samples the drone holds whatever task it last reported. Anything
    != `idle_task_type` counts as active time.
    """
    if len(task_series) < 2:
        return 0.0
    total = 0.0
    for i in range(len(task_series) - 1):
        t0, task = task_series[i]
        t1, _ = task_series[i + 1]
        if task != idle_task_type:
            total += max(0.0, t1 - t0)
    return total


def saga_confirmed_positions(
    events: Iterable[Dict[str, Any]],
) -> Dict[int, Tuple[float, float]]:
    """Map victim_id -> (x, y) for each VICTIM_CONFIRMED event (latest wins).

    The saga's VICTIM_CONFIRMED events are the authoritative set of
    confirmed victims: each carries the victim's position and is emitted for
    BOTH confirmation paths (the saga INVESTIGATE->CONFIRM orbit and the
    detection_filter multi-view auto-confirm). Scoring true/false positives
    from this set, rather than the transient /victims/candidates.confirmed
    flag, is what makes SagaConfirmedVictim, and its ground-truth-matched
    subset GroundTruthMatchedVictim, first-class.
    """
    out: Dict[int, Tuple[float, float]] = {}
    for e in events:
        if e.get('type') == 'VICTIM_CONFIRMED':
            pos = e.get('position') or [0.0, 0.0, 0.0]
            out[int(e.get('victim_id', 0))] = (float(pos[0]), float(pos[1]))
    return out


def score(
    confirmed: List[Tuple[int, Tuple[float, float]]],
    gt: List[Tuple[int, Tuple[float, float]]],
    radius_m: float,
) -> Tuple[List[dict], List[int], List[int]]:
    """Nearest-pair matching between confirmed victims and ground truth.

    Returns ``(tp_pairs, fp_ids, fn_ids)``. Each ``tp_pairs`` entry is
    ``{'candidate_id', 'gt_id', 'distance_m'}``.

    Matching is by globally-closest pair first: every within-radius
    (confirmed, ground-truth) pair is sorted by distance and assigned greedily,
    each confirmation and each ground-truth victim used at most once. This is
    order-independent. The previous implementation iterated confirmations in
    emit order and let an earlier, FARTHER confirmation claim a ground-truth
    victim, so a genuinely closer confirmation was scored a false positive,
    i.e. a real confirmed victim could be reported FP purely because of the
    order its VICTIM_CONFIRMED event happened to arrive. Inputs are
    materialised to lists so generator callers don't perturb semantics.
    """
    confirmed_list = list(confirmed)
    gt_list = list(gt)
    r2 = radius_m ** 2
    # All admissible (confirmed, gt) pairs within the match radius.
    pairs = []
    for vid, (cx, cy) in confirmed_list:
        for gid, (gx, gy) in gt_list:
            d2 = (cx - gx) ** 2 + (cy - gy) ** 2
            if d2 <= r2:
                pairs.append((d2, vid, gid))
    # Closest first; (vid, gid) tie-break keeps it deterministic.
    pairs.sort(key=lambda p: (p[0], p[1], p[2]))
    used_vid: set = set()
    used_gid: set = set()
    tp_pairs: List[dict] = []
    for d2, vid, gid in pairs:
        if vid in used_vid or gid in used_gid:
            continue
        used_vid.add(vid)
        used_gid.add(gid)
        tp_pairs.append({
            'candidate_id': vid, 'gt_id': gid,
            'distance_m': math.sqrt(d2),
        })
    fp_ids = [vid for vid, _ in confirmed_list if vid not in used_vid]
    fn_ids = [gid for gid, _ in gt_list if gid not in used_gid]
    return tp_pairs, fp_ids, fn_ids


def compute_detection_latency(
    tp_pairs: List[dict],
    confirm_t_by_id: Dict[int, float],
    drone_positions_by_drone: Dict[str, List[Tuple[float, float, float]]],
    gt_pos_by_id: Dict[int, Tuple[float, float]],
    gt_match_radius_m: float,
) -> List[float]:
    """For each (confirmed, gt) TP pair, compute detection latency:
    (confirm_t) minus (first sim-second any drone passed within
    `gt_match_radius_m` of the truth position).

    first_pass_by_gt is precomputed once in
    O(|drones|·|samples|·|gt|) rather than re-scanned per TP pair.
    Returns entries aligned with `tp_pairs`; pairs with no recorded
    pass-by are dropped (numeric only).
    """
    radius2 = gt_match_radius_m ** 2
    first_pass_by_gt: Dict[int, float] = {}
    for samples in drone_positions_by_drone.values():
        for (t, x, y) in samples:
            for gid_truth, (tx, ty) in gt_pos_by_id.items():
                if (x - tx) ** 2 + (y - ty) ** 2 <= radius2:
                    prev = first_pass_by_gt.get(gid_truth)
                    if prev is None or t < prev:
                        first_pass_by_gt[gid_truth] = t

    out: List[float] = []
    for pair in tp_pairs:
        cid = int(pair.get('candidate_id') or 0)
        gid = int(pair.get('gt_id') or 0)
        confirm_t = confirm_t_by_id.get(cid)
        first_pass = first_pass_by_gt.get(gid)
        if confirm_t is None or first_pass is None:
            continue
        out.append(max(0.0, confirm_t - first_pass))
    return out
