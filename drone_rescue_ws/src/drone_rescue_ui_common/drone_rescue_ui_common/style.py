"""Application-level Qt stylesheet generated from the palette.

One QSS document derived from ``UiPalette`` tokens replaces the
scattered per-widget ``setStyleSheet`` literals, so the whole
dashboard re-themes by swapping the palette instance. Pure
string-building (no PyQt5 import) so it lives in the (PyQt5-free)
``drone_rescue_ui_common`` bounded-context core and is testable
headlessly.

Typography stance: telemetry numerals render in a monospace face with
tabular figures (``MONO_FAMILY``); panel titles are small
letter-spaced uppercase "eyebrows"; one big number (mission clock /
coverage) at ``TYPE_DISPLAY_PT``.
"""

from __future__ import annotations

from .palette import DEFAULT_PALETTE, UiPalette

# Type scale (points). Qt default UI font stays for body text;
# these anchor the deliberate sizes.
TYPE_CAPTION_PT = 9
TYPE_BODY_PT = 10
TYPE_SECTION_PT = 12
TYPE_DISPLAY_PT = 22

#: monospace stack for telemetry numerals; present on every ROS
#: desktop install (DejaVu ships with the distro).
MONO_FAMILY = "'DejaVu Sans Mono', 'Liberation Mono', monospace"


def qss(p: UiPalette = DEFAULT_PALETTE) -> str:
    """Return the app-level stylesheet for palette ``p``.

    Flat underline tabs, 6px radii, hairline borders, visible focus
    rings (``p.accent``), slim flat scrollbars. ``p.accent`` carries
    every *interactive* state; the severity blue ``p.info`` no longer
    appears in chrome.
    """
    return f"""
QMainWindow, QDialog {{
    background-color: {p.bg_dark};
}}
QWidget {{
    color: {p.text_body};
    font-size: {TYPE_BODY_PT}pt;
}}
QTabWidget::pane {{
    border: 1px solid {p.stroke};
    border-radius: 6px;
    background: {p.bg_dark};
    top: -1px;
}}
QTabBar::tab {{
    background: transparent;
    color: {p.text_muted};
    padding: 7px 18px;
    border: none;
    border-bottom: 2px solid transparent;
}}
QTabBar::tab:hover {{
    color: {p.text_body};
}}
QTabBar::tab:selected {{
    color: {p.text_body};
    border-bottom: 2px solid {p.accent};
}}
QGroupBox {{
    border: 1px solid {p.stroke};
    border-radius: 6px;
    margin-top: 14px;
    padding-top: 4px;
    font-size: {TYPE_CAPTION_PT}pt;
    font-weight: bold;
    text-transform: uppercase;
    color: {p.text_muted};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
    letter-spacing: 2px;
}}
QTableWidget, QTableView {{
    background: {p.bg_dark};
    alternate-background-color: {p.bg_panel};
    gridline-color: transparent;
    font-family: {MONO_FAMILY};
    font-size: {TYPE_BODY_PT}pt;
    border: 1px solid {p.stroke};
    border-radius: 6px;
    selection-background-color: {p.accent_soft};
    selection-color: {p.text_body};
}}
QHeaderView::section {{
    background: {p.bg_panel};
    color: {p.text_muted};
    border: none;
    border-bottom: 1px solid {p.stroke};
    padding: 6px 8px;
    font-size: {TYPE_CAPTION_PT}pt;
    font-weight: bold;
    text-transform: uppercase;
    letter-spacing: 1px;
}}
QTextBrowser, QPlainTextEdit, QTextEdit {{
    background: {p.bg_deep};
    color: {p.text_body};
    border: 1px solid {p.stroke};
    border-radius: 6px;
    padding: 4px 8px;
    selection-background-color: {p.accent_soft};
}}
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {{
    background: {p.bg_deep};
    color: {p.text_body};
    border: 1px solid {p.stroke};
    border-radius: 6px;
    padding: 5px 10px;
    selection-background-color: {p.accent_soft};
}}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {{
    border: 1px solid {p.accent};
}}
QComboBox::drop-down {{
    border: none;
    width: 22px;
}}
QComboBox QAbstractItemView {{
    background: {p.bg_raised};
    border: 1px solid {p.stroke};
    selection-background-color: {p.accent_soft};
}}
QPushButton {{
    background: {p.bg_raised};
    color: {p.text_body};
    border: 1px solid {p.stroke};
    border-radius: 6px;
    padding: 7px 18px;
}}
QPushButton:hover {{
    border-color: {p.accent};
    color: {p.focus};
}}
QPushButton:pressed {{
    background: {p.bg_panel};
}}
QPushButton:checked {{
    background: {p.accent_soft};
    border-color: {p.accent};
}}
QPushButton:focus {{
    border: 1px solid {p.accent};
}}
QPushButton:disabled {{
    color: {p.text_muted};
    background: {p.bg_panel};
    border-color: {p.stroke};
}}
QPushButton#actionRun {{
    background: {p.action_run};
    border-color: {p.action_run};
    color: white;
    font-weight: bold;
}}
QPushButton#actionStop {{
    background: {p.action_stop};
    border-color: {p.action_stop};
    color: white;
    font-weight: bold;
}}
QScrollArea {{
    background: transparent;
    border: none;
}}
QScrollArea > QWidget > QWidget {{
    background: transparent;
}}
QMenu {{
    background: {p.bg_raised};
    color: {p.text_body};
    border: 1px solid {p.stroke};
    border-radius: 6px;
    padding: 4px;
}}
QMenu::item {{
    padding: 5px 22px;
    border-radius: 4px;
}}
QMenu::item:selected {{
    background: {p.accent_soft};
}}
QSplitter::handle {{
    background: {p.bg_dark};
}}
QSplitter::handle:hover {{
    background: {p.accent_soft};
}}
QDockWidget {{
    color: {p.text_muted};
    titlebar-close-icon: none;
}}
QStatusBar {{
    background: {p.bg_panel};
    color: {p.text_muted};
    border-top: 1px solid {p.stroke};
}}
QScrollBar:vertical {{
    background: transparent;
    width: 8px;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: {p.bg_raised};
    border-radius: 4px;
    min-height: 24px;
}}
QScrollBar::handle:vertical:hover {{
    background: {p.stroke};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
QScrollBar:horizontal {{
    background: transparent;
    height: 8px;
    margin: 0;
}}
QScrollBar::handle:horizontal {{
    background: {p.bg_raised};
    border-radius: 4px;
    min-width: 24px;
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0;
}}
QToolTip {{
    background: {p.bg_raised};
    color: {p.text_body};
    border: 1px solid {p.accent};
    padding: 4px 8px;
}}
"""


def mpl_rcparams(p: UiPalette = DEFAULT_PALETTE) -> dict:
    """Matplotlib rcParams matching the palette, as a plain dict (this
    module stays matplotlib-free; the GUI caller does
    ``matplotlib.rcParams.update(mpl_rcparams())``).

    Intended for the embedded FigureCanvas in Mission Control's
    Compare / Sweep tabs. The ``report`` PDF CLI deliberately does
    NOT apply this; print figures stay light-on-white.
    """
    return {
        'figure.facecolor': p.bg_dark,
        'savefig.facecolor': p.bg_dark,
        'axes.facecolor': p.bg_deep,
        'axes.edgecolor': p.stroke,
        'axes.labelcolor': p.text_body,
        'axes.titlecolor': p.text_body,
        'text.color': p.text_body,
        'xtick.color': p.text_muted,
        'ytick.color': p.text_muted,
        'grid.color': p.stroke,
        'legend.facecolor': p.bg_panel,
        'legend.edgecolor': p.stroke,
        'figure.titlesize': 'large',
    }
