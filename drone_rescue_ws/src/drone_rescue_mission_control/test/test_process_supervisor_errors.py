"""Regression test for LaunchSupervisor hook error handling.

The bug: every hook callback (on_line / on_activated / on_exited) ran under a
bare ``except Exception: pass``, so a raising callback (e.g. a parser bug that
hangs activation) was swallowed without a trace, leaving Mission Control stuck
on "Launching…" with no diagnostic. The supervisor now logs and counts hook
exceptions instead.
"""

from __future__ import annotations

from drone_rescue_mission_control.process_supervisor import LaunchSupervisor


def _supervisor(**hooks) -> LaunchSupervisor:
    return LaunchSupervisor(launch_args={}, **hooks)


def test_raising_hook_is_counted_not_swallowed():
    """A hook that raises increments callback_errors and does not propagate."""
    sup = _supervisor(on_line=lambda _l: (_ for _ in ()).throw(ValueError('boom')))
    assert sup.callback_errors == 0
    sup._safe_call(sup._on_line, 'a line', label='on_line')
    sup._safe_call(sup._on_line, 'another', label='on_line')
    assert sup.callback_errors == 2


def test_well_behaved_hook_does_not_count():
    seen = []
    sup = _supervisor(on_line=seen.append)
    sup._safe_call(sup._on_line, 'x', label='on_line')
    assert seen == ['x']
    assert sup.callback_errors == 0


def test_default_hooks_are_noops_and_safe():
    """The default no-op hooks never raise, so the counter stays at zero."""
    sup = _supervisor()
    sup._safe_call(sup._on_activated, label='on_activated')
    sup._safe_call(sup._on_exited, 0, label='on_exited')
    assert sup.callback_errors == 0
