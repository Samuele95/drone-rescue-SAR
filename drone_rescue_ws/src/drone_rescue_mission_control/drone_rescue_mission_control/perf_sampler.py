"""Runtime performance sampler for the mission recorder.

No run ever recorded a resource metric, so the hardware envelope (RTF, CPU, RSS)
in the planning docs was prose-only and unsubstantiated. This sampler records
real measurements during a mission, never fabricated, and summarises them into
the run JSON's ``performance`` block:

* **RTF** (real-time factor) from consecutive (sim-time, wall-time) deltas: how
  fast the simulation advances relative to wall-clock.
* **system CPU %** from psutil.
* **node-tree RSS** (MB): summed resident memory of the processes whose command
  line names a drone_rescue node executable.

The summarisation (:func:`summarize_perf`) is a pure function over the recorded
lists, so it is unit-tested without psutil or a clock. When no samples are
captured (e.g. a headless run with no node processes), every envelope is null:
the block says "unmeasured", it never invents a number.
"""

from __future__ import annotations

from typing import List, Optional, Sequence

# Substrings that identify this project's node processes in a cmdline.
DEFAULT_NODE_HINTS = (
    'mission_manager', 'drone_controller', 'drone_executor', 'victim_detector',
    'detection_filter', 'pheromone_server', 'coverage_tracker', 'zone_manager',
    'environment_monitor', 'drone_health_monitor', 'battery_monitor',
)


def _envelope(xs: Sequence[Optional[float]]) -> dict:
    vals = [x for x in xs if x is not None]
    if not vals:
        return {'mean': None, 'min': None, 'max': None}
    return {
        'mean': sum(vals) / len(vals),
        'min': min(vals),
        'max': max(vals),
    }


def summarize_perf(rtf: Sequence[float],
                   cpu_percent: Sequence[float],
                   rss_mb: Sequence[float]) -> dict:
    """Pure summary of the recorded sample lists into the ``performance`` block.

    Every field is null when its list is empty: the recorder reports
    "unmeasured", never a fabricated value.
    """
    return {
        'rtf': _envelope(rtf),
        'system_cpu_percent': _envelope(cpu_percent),
        'node_tree_rss_mb': _envelope(rss_mb),
        'sample_count': len(rtf),
        'note': ('real psutil/clock samples; null envelopes mean unmeasured '
                 '(values are never fabricated)'),
    }


class PerfSampler:
    """Accumulates RTF / CPU / RSS samples over a mission.

    ``sample(sim_t, wall_t)`` is called periodically (the recorder drives it at
    1 Hz). RTF needs two consecutive samples, so the first call only seeds the
    clock pair. psutil failures are swallowed per-sample so instrumentation
    never destabilises a run.
    """

    def __init__(self, node_hints: Sequence[str] = DEFAULT_NODE_HINTS):
        self._hints = tuple(node_hints)
        self._rtf: List[float] = []
        self._cpu: List[float] = []
        self._rss_mb: List[float] = []
        self._prev = None  # (sim_t, wall_t)

    def sample(self, sim_t: float, wall_t: float) -> None:
        if self._prev is not None:
            d_sim = sim_t - self._prev[0]
            d_wall = wall_t - self._prev[1]
            if d_wall > 0.0:
                self._rtf.append(d_sim / d_wall)
        self._prev = (sim_t, wall_t)
        try:
            import psutil
            self._cpu.append(float(psutil.cpu_percent(interval=None)))
            self._rss_mb.append(self._node_tree_rss_mb(psutil))
        except Exception:
            pass

    def _node_tree_rss_mb(self, psutil) -> float:
        total = 0
        for proc in psutil.process_iter(['cmdline', 'memory_info']):
            try:
                cmd = ' '.join(proc.info.get('cmdline') or [])
                if any(h in cmd for h in self._hints):
                    total += proc.info['memory_info'].rss
            except Exception:
                continue
        return total / (1024.0 * 1024.0)

    def summary(self) -> dict:
        return summarize_perf(self._rtf, self._cpu, self._rss_mb)
