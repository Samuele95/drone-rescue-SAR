"""Wraps `ros2 launch demo.launch.py` as a child process.

Mission Control owns one supervisor at a time. The supervisor:
  * spawns the launch with the resolved scenario args
  * captures stdout/stderr line-by-line into a thread-safe deque
  * exposes hooks for "activation completed" and "process exited"
  * cleanly terminates the subprocess tree on stop()

The activation signal is detected by greping the launch's stdout for the
sentinel `All nodes activated successfully` (printed by lifecycle_manager).
Mission Control uses the activation hook to fire the runtime ros2 param
sets and then publish /survey/start.

Cleanup: on stop() we send SIGTERM to the entire process group; if it
doesn't exit within `STOP_GRACE_S`, we follow with SIGKILL. ros2 launch
catches SIGTERM and propagates it down to its child Nodes, which in turn
gives mission_recorder a chance to flush its JSONL.
"""

from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from collections import deque
from typing import Callable, Dict, List, Optional


_log = logging.getLogger(__name__)

_LAUNCH_PACKAGE = 'drone_rescue_bringup'
_LAUNCH_FILE = 'demo.launch.py'

_ACTIVATION_SENTINEL = 'All nodes activated successfully'
_STOP_GRACE_S = 8.0


class LaunchSupervisor:
    """One running mission's child process. Re-instantiate per run."""

    def __init__(
        self,
        launch_args: Dict[str, str],
        on_line: Optional[Callable[[str], None]] = None,
        on_activated: Optional[Callable[[], None]] = None,
        on_exited: Optional[Callable[[int], None]] = None,
        max_buffer_lines: int = 4000,
    ):
        self._launch_args = dict(launch_args)
        self._on_line = on_line or (lambda _l: None)
        self._on_activated = on_activated or (lambda: None)
        self._on_exited = on_exited or (lambda _rc: None)

        self._proc: Optional[subprocess.Popen] = None
        self._reader: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._activated = False

        # Ring of recent stdout/stderr lines. Active tab tails this for the
        # operator and the supervisor itself uses it for the activation
        # sentinel detection.
        self._buffer: deque = deque(maxlen=max_buffer_lines)
        self._buffer_lock = threading.Lock()

        # Count of hook callbacks that raised. A bug in an on_line /
        # on_activated / on_exited callback (e.g. a hung activation) used to be
        # invisible because the exception was silently swallowed; log-and-count
        # instead.
        self._callback_errors = 0

    # --------------------------------------------------------- spawn
    def start(self) -> None:
        if self._proc is not None:
            raise RuntimeError('LaunchSupervisor.start() called twice')
        cmd = ['ros2', 'launch', _LAUNCH_PACKAGE, _LAUNCH_FILE]
        for k, v in self._launch_args.items():
            cmd.append(f'{k}:={v}')
        # Inherit env so ROS_DISTRO + LD_LIBRARY_PATH propagate.
        # New process group so we can SIGTERM the whole tree on stop().
        self._proc = subprocess.Popen(
            cmd,
            env=os.environ.copy(),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,           # line-buffered
            preexec_fn=os.setsid,  # new session → new pgid
        )
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name='LaunchSupervisorReader',
        )
        self._reader.start()

    def _read_loop(self) -> None:
        assert self._proc is not None
        try:
            for line in iter(self._proc.stdout.readline, ''):
                if self._stop_event.is_set():
                    break
                line = line.rstrip()
                with self._buffer_lock:
                    self._buffer.append(line)
                self._safe_call(self._on_line, line, label='on_line')
                if not self._activated and _ACTIVATION_SENTINEL in line:
                    self._activated = True
                    self._safe_call(self._on_activated, label='on_activated')
        finally:
            rc = self._proc.poll() if self._proc else None
            if rc is None:
                # Stream closed but proc alive: wait briefly so rc is sensible.
                try:
                    rc = self._proc.wait(timeout=2.0)
                except Exception:
                    rc = -1
            self._safe_call(
                self._on_exited, int(rc if rc is not None else -1),
                label='on_exited',
            )

    def _safe_call(self, fn: Callable, *args, label: str) -> None:
        """Invoke a hook callback, logging and counting any exception instead
        of swallowing it silently. A misbehaving hook must not kill the
        reader thread, but it also must not vanish without trace."""
        try:
            fn(*args)
        except Exception:
            self._callback_errors += 1
            _log.exception('LaunchSupervisor %s hook raised', label)

    @property
    def callback_errors(self) -> int:
        """Number of hook callbacks that have raised."""
        return self._callback_errors

    # --------------------------------------------------------- query
    @property
    def is_alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    @property
    def activated(self) -> bool:
        return self._activated

    def stdout_tail(self, n: int = 100) -> List[str]:
        with self._buffer_lock:
            if n >= len(self._buffer):
                return list(self._buffer)
            return list(self._buffer)[-n:]

    # --------------------------------------------------------- stop
    def stop(self) -> None:
        """Terminate the launch tree gracefully, then forcefully if needed."""
        if self._proc is None:
            return
        self._stop_event.set()
        if self._proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + _STOP_GRACE_S
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                return
            time.sleep(0.2)
        # Hard kill the whole pgroup.
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
