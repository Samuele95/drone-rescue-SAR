"""Scan-time ETA from a windowed coverage rate: pure Python, no rclpy.

coverage_tracker estimated the remaining scan time from a whole-mission linear
rate (``coverage_pct / elapsed``). That lags badly: early slow arming drags the
average down for the whole run, and a late slowdown is invisible until it has
dominated the average. These helpers compute the rate from a recent sliding
window instead, so the ETA tracks the *current* progress rate.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple


def windowed_rate(samples: Sequence[Tuple[float, float]]) -> Optional[float]:
    """Coverage rate (percent per second) across a window of ``(t, pct)``
    samples, time-ascending. Uses the first and last sample in the window:
    ``(pct_last - pct_first) / (t_last - t_first)``. Returns ``None`` when there
    are fewer than two samples or the window has no positive time span.
    """
    if len(samples) < 2:
        return None
    t0, p0 = samples[0]
    t1, p1 = samples[-1]
    dt = t1 - t0
    if dt <= 0.0:
        return None
    return (p1 - p0) / dt


def estimate_remaining(coverage_pct: float, rate: Optional[float]) -> float:
    """Seconds to reach 100% at ``rate`` (percent/second). Returns 0.0 when the
    rate is unknown or non-positive (not making progress means not estimable)."""
    if rate is None or rate <= 0.0:
        return 0.0
    return max(0.0, (100.0 - coverage_pct) / rate)


def prune_window(
    samples: List[Tuple[float, float]],
    now_t: float,
    window_s: float,
) -> List[Tuple[float, float]]:
    """Drop samples older than ``window_s`` behind ``now_t`` (kept in place,
    returns the same list for convenience). Always keeps at least the most
    recent sample so a long stall still yields a (zero-rate) window."""
    cutoff = now_t - window_s
    while len(samples) > 1 and samples[0][0] < cutoff:
        samples.pop(0)
    return samples
