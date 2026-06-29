"""One-call Qt application theming: Fusion + dark QPalette + qss().

The QSS alone leaves Fusion's default *light* palette underneath; it
leaks through every surface the stylesheet doesn't explicitly paint
(scroll-area viewports, item-view corners, QMessageBox, menus).
Setting a palette derived from the same ``UiPalette`` tokens closes
that class of bug once, instead of chasing each leak with another QSS
rule.

This module imports Qt, so it lives apart from the deliberately
PyQt5-free ``style.py`` / ``palette.py`` core. Both desktop apps call
``apply_app_theme(app)`` as their single styling entry point.
"""

from __future__ import annotations

import os

from .palette import DEFAULT_PALETTE, UiPalette
from .style import qss


def enable_hidpi() -> None:
    """Call BEFORE constructing the QApplication.

    On scaled displays (GNOME 200% → Xft.dpi 192) Qt5 scales only
    *fonts* by the DPI while stylesheet px metrics (paddings, fixed
    heights, radii) stay raw, so text outgrows its chrome and every
    button looks too small for its content. Enabling Qt's high-DPI
    scaling gives the app a device-pixel-ratio so px metrics scale
    together with the text. ``setdefault`` means an operator's explicit
    env always wins. No-op on unscaled displays (factor 1).
    """
    os.environ.setdefault('QT_ENABLE_HIGHDPI_SCALING', '1')
    os.environ.setdefault('QT_SCALE_FACTOR_ROUNDING_POLICY',
                          'PassThrough')


def build_qpalette(p: UiPalette = DEFAULT_PALETTE):
    """QPalette mirroring the UiPalette surface/text tokens."""
    from python_qt_binding.QtGui import QColor, QPalette

    pal = QPalette()
    c = QColor
    pal.setColor(QPalette.Window, c(p.bg_dark))
    pal.setColor(QPalette.WindowText, c(p.text_body))
    pal.setColor(QPalette.Base, c(p.bg_deep))
    pal.setColor(QPalette.AlternateBase, c(p.bg_panel))
    pal.setColor(QPalette.Text, c(p.text_body))
    pal.setColor(QPalette.PlaceholderText, c(p.text_muted))
    pal.setColor(QPalette.Button, c(p.bg_raised))
    pal.setColor(QPalette.ButtonText, c(p.text_body))
    pal.setColor(QPalette.ToolTipBase, c(p.bg_raised))
    pal.setColor(QPalette.ToolTipText, c(p.text_body))
    pal.setColor(QPalette.Highlight, c(p.accent_soft))
    pal.setColor(QPalette.HighlightedText, c(p.text_body))
    pal.setColor(QPalette.Link, c(p.accent))
    pal.setColor(QPalette.BrightText, c(p.focus))
    pal.setColor(QPalette.Disabled, QPalette.Text, c(p.text_muted))
    pal.setColor(QPalette.Disabled, QPalette.ButtonText, c(p.text_muted))
    pal.setColor(QPalette.Disabled, QPalette.WindowText, c(p.text_muted))
    return pal


def apply_app_theme(app, p: UiPalette = DEFAULT_PALETTE) -> None:
    """Fusion style + dark palette + the palette-derived stylesheet."""
    app.setStyle('Fusion')
    app.setPalette(build_qpalette(p))
    app.setStyleSheet(qss(p))
