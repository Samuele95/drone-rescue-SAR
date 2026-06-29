"""RunRepository: persistence port for RunSummary.

- ``RunRepository`` Protocol: ``list()`` enumerates run handles in
  order, ``load(handle)`` returns a typed RunSummary, ``save(summary,
  dir)`` writes one JSONL.
- ``JsonlRunRepository`` is the production adapter: one ``*.json``
  per run on the filesystem, matches the existing
  ``mission_recorder._finalize`` write pattern.

Tests can implement the Protocol with a SimpleNamespace or a class
holding a ``dict[name, RunSummary]`` for in-memory fixtures.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterator, List, Protocol, Sequence

from .run_summary import RunSummary, RunHandle


class RunRepository(Protocol):
    """Where do mission runs live? Repository port."""

    def list(self) -> Sequence[RunHandle]: ...
    def load(self, handle: RunHandle) -> RunSummary: ...
    def save(self, summary: RunSummary, out_dir: Path) -> Path: ...


class JsonlRunRepository:
    """Concrete adapter: one ``<timestamp>__<pattern>__<scenario>.json``
    per run, matching ``mission_recorder._finalize``'s on-disk shape.
    """

    def __init__(self, runs_dir: Path):
        self._runs_dir = Path(runs_dir)

    def list(self) -> List[RunHandle]:
        """Every ``*.json`` in the directory, sorted by name (which
        sorts by timestamp because the filenames lead with UTC).

        Skips ``manifest.json`` (sweep manifest written by bench.py)
        and ``report.json`` (legacy export name) so the list is just
        runs.
        """
        if not self._runs_dir.is_dir():
            return []
        out: List[RunHandle] = []
        for p in sorted(self._runs_dir.glob('*.json')):
            if p.name in ('manifest.json', 'report.json'):
                continue
            out.append(RunHandle.from_path(p))
        return out

    def load(self, handle: RunHandle) -> RunSummary:
        return RunSummary.from_jsonl(handle.path)

    def iter_summaries(self) -> Iterator[RunSummary]:
        """Convenience: yield typed RunSummary for every run in the dir.

        Equivalent to ``(self.load(h) for h in self.list())`` but
        skips runs that fail to parse instead of raising, useful for
        the analytics path which historically tolerated mixed-era
        JSONLs in the same directory.
        """
        for h in self.list():
            try:
                yield self.load(h)
            except Exception:
                continue

    def save(
        self,
        summary: RunSummary,
        out_dir: Path,
        filename: str | None = None,
    ) -> Path:
        """Write the RunSummary as a JSONL.

        ``filename`` defaults to ``<utc>__<pattern>__<scenario>.json``,
        the convention mission_recorder uses. Returns the absolute
        path written.
        """
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        if filename is None:
            stamp = datetime.now(timezone.utc).strftime('%Y-%m-%d_%H%M%S')
            filename = (
                f'{stamp}__{summary.metadata.pattern}__'
                f'{summary.metadata.scenario}.json'
            )
        out_path = out_dir / filename
        out_path.write_text(json.dumps(summary.to_dict(), indent=2))
        return out_path
