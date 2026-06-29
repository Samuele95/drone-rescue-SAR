"""Shared micro-motion helpers.

Qt QSS has no transitions, so the "fluid" feel is implemented here in
code: short (120-250 ms), ease-out, GUI-thread-cheap animations that
widgets opt into. Imports Qt, so (like ``qt_theme``) this lives
apart from the PyQt5-free ``style``/``palette``/``view_model`` core.

Design rules:
- Every helper is restart-safe: re-invoking while a previous run is
  in flight retargets instead of stacking animations.
- Animations attach to the target widget as a parent, so teardown is
  Qt-ownership-automatic; no global registries.
- Helpers never block and never touch non-GUI threads.
"""

from __future__ import annotations

from python_qt_binding.QtCore import (
    QEasingCurve, QPropertyAnimation, QVariantAnimation,
)
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QGraphicsOpacityEffect

#: house duration band (ms)
FAST_MS = 140
BASE_MS = 180
SLOW_MS = 250


def set_value_animated(bar, value: int, duration: int = BASE_MS) -> None:
    """Animate a QProgressBar (or any widget with a ``value`` Qt
    property) toward ``value`` with ease-out.

    Tiny deltas (≤1 unit) apply instantly; high-frequency telemetry
    refreshes (≈30 Hz bridge ticks) must not keep an animation
    perpetually restarting and never settling.
    """
    cur = int(bar.value())
    value = int(value)
    if abs(value - cur) <= 1:
        anim = getattr(bar, '_value_anim', None)
        if anim is not None:
            anim.stop()
        bar.setValue(value)
        return
    anim = getattr(bar, '_value_anim', None)
    if anim is None:
        anim = QPropertyAnimation(bar, b'value', bar)
        anim.setEasingCurve(QEasingCurve.OutCubic)
        bar._value_anim = anim
    anim.stop()
    anim.setDuration(duration)
    anim.setStartValue(cur)
    anim.setEndValue(value)
    anim.start()


def fade_in(widget, duration: int = FAST_MS) -> None:
    """Fade ``widget`` from transparent to opaque, then DROP the
    graphics effect (a lingering QGraphicsOpacityEffect costs a
    render-to-pixmap pass per paint and can break GL viewports, so
    callers must not apply this to pages hosting a GLViewWidget).
    """
    old = getattr(widget, '_fade_anim', None)
    if old is not None:
        old.stop()
    eff = QGraphicsOpacityEffect(widget)
    eff.setOpacity(0.0)
    widget.setGraphicsEffect(eff)
    anim = QPropertyAnimation(eff, b'opacity', widget)
    anim.setDuration(duration)
    anim.setStartValue(0.0)
    anim.setEndValue(1.0)
    anim.setEasingCurve(QEasingCurve.OutCubic)
    widget._fade_anim = anim

    def _teardown():
        widget.setGraphicsEffect(None)   # deletes eff
        widget._fade_anim = None

    anim.finished.connect(_teardown)
    anim.start()


def pulse_color(label, from_color: str, to_color: str,
                stylesheet_fmt: str = 'color: {color};',
                duration: int = SLOW_MS) -> None:
    """One-shot colour pulse on a QLabel: jump to ``from_color`` and
    ease back to ``to_color`` (its resting colour). ``stylesheet_fmt``
    is .format()-ed with the interpolated ``color`` so callers keep
    their other inline properties (font, padding) intact.
    """
    old = getattr(label, '_pulse_anim', None)
    if old is not None:
        old.stop()
    anim = QVariantAnimation(label)
    anim.setDuration(duration)
    anim.setStartValue(QColor(from_color))
    anim.setEndValue(QColor(to_color))
    anim.setEasingCurve(QEasingCurve.OutCubic)
    anim.valueChanged.connect(
        lambda c: label.setStyleSheet(stylesheet_fmt.format(color=c.name()))
    )
    label._pulse_anim = anim
    anim.start()
