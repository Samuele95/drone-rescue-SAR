"""Headless batch sweep runner.

CLI to spin up `K trials x P patterns x A allocations x S scenarios` of the
drone-rescue sim sequentially (or, optionally, with a small parallelism
budget), collecting one JSONL per trial under a sweep directory along with a
manifest.

Reuses `LaunchSupervisor` from `process_supervisor.py` so the launch
and recorder plumbing is exactly what Mission Control already drives,
no second code path to maintain.

Usage::

    ros2 run drone_rescue_mission_control bench \\
        --patterns spiral_out,parallel_track,random_walk \\
        --allocations greedy_auction,hungarian,round_robin \\
        --scenarios default,cluster \\
        --trials 3 --seed-start 0 --runs-dir runs/v5_baseline

The runs-dir gets:
- `<UTC>__<pattern>__<allocation>__<scenario>.json`  per trial (recorder writes these)
- `manifest.json`  matrix + git SHA + start/end wall times
- `README.md`  one-paragraph human description

Idempotent: a re-run with the same `--runs-dir` SKIPS any (pattern,
scenario, seed) tuple whose JSONL already exists. Pass `--force` to
re-run everything.

Determinism: each trial passes `seed=<seed-start>+<trial>` to the
launch, so trial 0 across two `bench` invocations re-runs the same
seed for numerically identical JSONL summary fields (modulo timestamps
and Gazebo physics determinism; see docs/v5-release.md).
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional

from .process_supervisor import LaunchSupervisor


# Default wall-time ceiling per trial. Mission_timeout_seconds in the
# scenario YAML is sim-time; this is a wall-time backstop in case the
# sim hangs or runs slow. 30 min is generous for a 10-minute mission.
_DEFAULT_TRIAL_WALL_BUDGET_S = 1800
# Grace period after we send SIGTERM to the launch tree before SIGKILL.
_STOP_GRACE_S = 12.0


def _git_sha() -> str:
    """Best-effort git SHA of the current HEAD; empty string if unavailable."""
    try:
        r = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            capture_output=True, text=True, timeout=5.0,
        )
        return r.stdout.strip() if r.returncode == 0 else ''
    except Exception:
        return ''


def _scenario_path(name: str) -> Path:
    """Resolve a scenario short name to its YAML path.

    Consumes the `ScenarioRepository` Protocol. Returns the path
    attached to the Scenario VO. Falls back to a direct filesystem
    lookup so a name
    that hasn't yet been auto-discovered (e.g. a fresh scenario file
    dropped under the source tree mid-bench) still resolves.
    """
    from drone_rescue_mission_control.persistence import (
        YamlScenarioRepository,
    )
    repo = YamlScenarioRepository()
    scenario = repo.load(name)
    if scenario is not None and scenario.path is not None:
        return Path(scenario.path)
    p = repo.scenarios_dir / f'{name}.yaml'
    if p.is_file():
        return p
    raise FileNotFoundError(
        f'scenario {name!r} not found under {repo.scenarios_dir}. '
        f'Did you colcon build drone_rescue_bringup?'
    )


def _trial_filename_glob(runs_dir: Path, pattern: str, allocation: str,
                         scenario: str) -> List[Path]:
    """Return any existing trial JSONL for this (pattern, allocation,
    scenario); used by the resume check. Matches mission_recorder's
    ``<ts>__<pattern>__<allocation>__<scenario>.json`` naming."""
    return sorted(runs_dir.glob(f'*__{pattern}__{allocation}__{scenario}.json'))


def _run_one_trial(
    pattern: str,
    allocation: str,
    scenario_name: str,
    scenario_path: Path,
    seed: int,
    runs_dir: Path,
    wall_budget_s: int,
) -> Optional[Path]:
    """Run a single trial. Returns the new JSONL path on success, None
    on failure / timeout. The recorder writes the JSONL when the launch
    sees `MISSION_COMPLETE` or `MISSION_TIMEOUT`, or when we SIGTERM the
    launch tree.
    """
    # Narrow the detection glob to the (pattern, scenario)-matched
    # filename so a manifest write or a concurrent bench in the same
    # directory cannot be mis-attributed to this trial. Snapshot is
    # taken AFTER `sup.start()` returns to close the race where a
    # previous run's recorder finishes flushing between the snapshot
    # and the spawn.
    trial_glob = f'*__{pattern}__{allocation}__{scenario_name}.json'

    launch_args = {
        'record_run': 'true',
        'scenario_yaml': str(scenario_path),
        'scenario_name': scenario_name,
        'runs_dir': str(runs_dir),
        'coverage_pattern': pattern,
        'allocation_strategy': allocation,
        'seed': str(seed),
        # Headless: no dashboard, no rviz; bench is meant to run
        # unattended overnight, not on a developer's screen.
        'dashboard': 'false',
        'use_rviz': 'false',
    }

    activated = {'flag': False}
    exited = {'rc': None}

    def _on_line(_line: str) -> None:
        pass

    def _on_activated() -> None:
        activated['flag'] = True

    def _on_exited(rc: int) -> None:
        exited['rc'] = rc

    sup = LaunchSupervisor(
        launch_args=launch_args,
        on_line=_on_line,
        on_activated=_on_activated,
        on_exited=_on_exited,
    )
    try:
        sup.start()
    except Exception as e:
        print(f'   [trial] FAIL spawning ros2 launch: {e}', file=sys.stderr)
        return None

    # Snapshot AFTER sup.start() so any pending flush from a previous
    # run is already on disk and won't be attributed to us.
    before = set(runs_dir.glob(trial_glob))

    # Wait for either a new (pattern, scenario)-matching JSONL to
    # appear or the wall budget to expire.
    deadline = time.monotonic() + wall_budget_s
    new_path: Optional[Path] = None
    while time.monotonic() < deadline:
        if not sup.is_alive:
            # Subprocess died on its own. Give the recorder a moment
            # to flush.
            time.sleep(2.0)
            break
        current = set(runs_dir.glob(trial_glob))
        diff = current - before
        if diff:
            new_path = next(iter(diff))
            break
        time.sleep(2.0)

    # Always tear down the launch tree on the way out: even if the
    # JSONL appeared, the supervisor + Gazebo + nodes are still alive
    # and would block the next trial.
    try:
        sup.stop()
    except Exception:
        pass

    if new_path is None:
        # Recheck once after the SIGTERM: the recorder might write the
        # JSONL during teardown (caught SIGTERM, finalize, exit).
        time.sleep(_STOP_GRACE_S)
        current = set(runs_dir.glob(trial_glob))
        diff = current - before
        if diff:
            new_path = next(iter(diff))

    return new_path


def _summarize_jsonl(path: Path) -> str:
    """One-line summary for the progress log."""
    try:
        d = json.loads(path.read_text())
        s = d.get('summary', {})
        return (
            f'cov={s.get("final_coverage_pct", 0):.1f}% '
            f'TP={s.get("true_positives", 0)} '
            f'FP={s.get("false_positives", 0)} '
            f'F1={s.get("f1_score", 0):.2f} '
            f'dur={d.get("metadata", {}).get("duration_s", 0):.0f}s'
        )
    except Exception:
        return '(could not parse JSONL)'


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        prog='bench',
        description='V5 batch sweep runner for the drone-rescue SAR sim.',
    )
    p.add_argument('--patterns', required=True,
                   help='Comma-separated list of coverage_pattern names.')
    p.add_argument('--allocations', default='greedy_auction',
                   help='Comma-separated list of allocation_strategy names '
                        '(default: greedy_auction).')
    p.add_argument('--scenarios', required=True,
                   help='Comma-separated list of scenario short names '
                        '(e.g. default,cluster). Each must resolve to a '
                        'YAML in drone_rescue_bringup/config/scenarios/.')
    p.add_argument('--trials', type=int, default=3,
                   help='Trials per (pattern, scenario) combination. '
                        'Default 3.')
    p.add_argument('--seed-start', type=int, default=0,
                   help='Seed for trial 0; trial i uses seed-start+i. '
                        'Same seed-start → reproducible sweep.')
    p.add_argument('--runs-dir', required=True, type=Path,
                   help='Directory to write per-trial JSONLs and manifest.')
    p.add_argument('--wall-budget', type=int,
                   default=_DEFAULT_TRIAL_WALL_BUDGET_S,
                   help='Wall-time ceiling per trial (s). Default 1800.')
    p.add_argument('--force', action='store_true',
                   help='Re-run trials whose JSONL already exists. By '
                        'default we skip them (idempotent resume).')
    args = p.parse_args(argv)

    patterns = [s.strip() for s in args.patterns.split(',') if s.strip()]
    allocations = [s.strip() for s in args.allocations.split(',') if s.strip()]
    scenarios = [s.strip() for s in args.scenarios.split(',') if s.strip()]
    if not patterns or not scenarios or not allocations:
        print('error: --patterns, --allocations and --scenarios must be '
              'non-empty', file=sys.stderr)
        return 2

    runs_dir: Path = args.runs_dir
    runs_dir.mkdir(parents=True, exist_ok=True)

    # Resolve scenario paths up-front so a typo aborts before we burn
    # 30 minutes on the first trial.
    scen_paths = {name: _scenario_path(name) for name in scenarios}

    # Build the matrix. Each entry: pattern/allocation/scenario/seed/trial.
    matrix = []
    for pattern in patterns:
        for allocation in allocations:
            for scenario in scenarios:
                for trial in range(args.trials):
                    matrix.append({
                        'pattern': pattern,
                        'allocation': allocation,
                        'scenario': scenario,
                        'trial': trial,
                        'seed': args.seed_start + trial,
                    })

    started_at = datetime.datetime.now(datetime.timezone.utc).isoformat()
    sha = _git_sha()
    print(f'[bench] {len(matrix)} trials '
          f'({len(patterns)} patterns × {len(allocations)} allocations × '
          f'{len(scenarios)} scenarios × {args.trials} trials) → {runs_dir}')
    print(f'[bench] git_sha={sha or "(unavailable)"} seed_start={args.seed_start} '
          f'wall_budget={args.wall_budget}s')

    # Write a manifest stub before the first trial so a Ctrl-C halfway
    # through still leaves the dir self-describing.
    manifest = {
        'started_at': started_at,
        'ended_at': None,
        'git_sha': sha,
        'seed_start': args.seed_start,
        'wall_budget_s': args.wall_budget,
        'patterns': patterns,
        'allocations': allocations,
        'scenarios': scenarios,
        'trials': args.trials,
        'matrix': matrix,
        'results': [],
    }
    (runs_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    _write_readme(runs_dir, manifest)

    # Install a signal handler so Ctrl-C tears down the current trial
    # cleanly and writes the manifest before exiting.
    interrupted = {'flag': False}

    def _handle_sigint(*_args):
        interrupted['flag'] = True
        print('\n[bench] caught SIGINT; current trial will be aborted.',
              file=sys.stderr)

    signal.signal(signal.SIGINT, _handle_sigint)

    results = []
    for i, entry in enumerate(matrix, start=1):
        if interrupted['flag']:
            break
        pattern = entry['pattern']
        allocation = entry['allocation']
        scenario = entry['scenario']
        seed = entry['seed']

        prefix = (f'[{i:>3}/{len(matrix)}] pattern={pattern} '
                  f'alloc={allocation} scenario={scenario} seed={seed}')

        # Resume check: if a JSONL for this (pattern, scenario) already
        # exists (and we're not forcing), skip. Per-(pattern,scenario)
        # counting; the seed isn't in the filename, so re-running the
        # same matrix with a different seed would also skip, which is
        # the right behaviour for idempotent rerunning of "the same
        # sweep".
        existing = _trial_filename_glob(runs_dir, pattern, allocation, scenario)
        if existing and not args.force:
            # Only skip if we already have at least `trial+1` JSONLs
            # for this (pattern,scenario); otherwise we still need to
            # produce the next trial.
            if len(existing) > entry['trial']:
                skipped_path = existing[entry['trial']]
                # WARN if the existing JSONL was produced with a
                # different seed than this matrix entry asks for. The
                # skip itself is intentional (idempotent resume), but
                # the user deserves to see the metadata mismatch so
                # they can pass --force or pick a fresh runs_dir if
                # they actually want a re-seeded re-run.
                try:
                    existing_seed = json.loads(
                        skipped_path.read_text()
                    ).get('metadata', {}).get(
                        'params_snapshot', {}
                    ).get('seed', None)
                except Exception:
                    existing_seed = None
                seed_note = ''
                if existing_seed is not None and int(existing_seed) != int(seed):
                    seed_note = (
                        f'  ⚠ seed mismatch: existing={existing_seed} '
                        f'requested={seed}; pass --force to re-seed.'
                    )
                print(f'{prefix} → SKIP (existing: {skipped_path.name}){seed_note}')
                results.append({**entry, 'jsonl': str(skipped_path),
                                'status': 'skipped'})
                continue

        print(f'{prefix} → run...')
        t0 = time.monotonic()
        new_path = _run_one_trial(
            pattern=pattern,
            allocation=allocation,
            scenario_name=scenario,
            scenario_path=scen_paths[scenario],
            seed=seed,
            runs_dir=runs_dir,
            wall_budget_s=args.wall_budget,
        )
        dt = time.monotonic() - t0
        if new_path is None:
            print(f'{prefix} → FAIL (no JSONL after {dt:.0f}s)')
            results.append({**entry, 'jsonl': None, 'status': 'fail',
                            'wall_s': dt})
        else:
            print(f'{prefix} → OK ({dt:.0f}s) {new_path.name}: '
                  f'{_summarize_jsonl(new_path)}')
            results.append({**entry, 'jsonl': str(new_path),
                            'status': 'ok', 'wall_s': dt})

        # Persist the manifest after every trial so a crash mid-sweep
        # leaves a usable record.
        manifest['results'] = results
        (runs_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2))

    manifest['ended_at'] = datetime.datetime.now(
        datetime.timezone.utc).isoformat()
    manifest['results'] = results
    (runs_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    _write_readme(runs_dir, manifest)

    n_ok = sum(1 for r in results if r['status'] == 'ok')
    n_fail = sum(1 for r in results if r['status'] == 'fail')
    n_skip = sum(1 for r in results if r['status'] == 'skipped')
    print(f'[bench] done. ok={n_ok} fail={n_fail} skipped={n_skip} '
          f'matrix={len(matrix)}')
    return 0 if n_fail == 0 else 1


def _write_readme(runs_dir: Path, manifest: dict) -> None:
    """Write a one-paragraph README so the sweep dir is self-describing
    when an examiner stumbles into it."""
    p = runs_dir / 'README.md'
    body = (
        f"# Sweep — {runs_dir.name}\n\n"
        f"Generated by `ros2 run drone_rescue_mission_control bench` "
        f"on {manifest['started_at']}.\n\n"
        f"- Patterns: {', '.join(manifest['patterns'])}\n"
        f"- Allocations: {', '.join(manifest.get('allocations', ['greedy_auction']))}\n"
        f"- Scenarios: {', '.join(manifest['scenarios'])}\n"
        f"- Trials per combination: {manifest['trials']}\n"
        f"- Seeds: trial *i* uses `seed_start + i` "
        f"(seed_start = {manifest['seed_start']})\n"
        f"- git SHA at sweep time: `{manifest['git_sha'] or 'unknown'}`\n\n"
        f"Each trial wrote one JSONL via the standard mission_recorder. "
        f"Open the sweep in Mission Control's **Sweep Runs** tab "
        f"(File → Open sweep…) to see aggregate boxplots and the "
        f"per-pattern statistical summary. To regenerate this sweep "
        f"verbatim, re-run the same `bench` invocation with the same "
        f"`--seed-start` — already-written JSONLs will be skipped "
        f"(pass `--force` to re-run).\n"
    )
    p.write_text(body)


if __name__ == '__main__':
    sys.exit(main())
