"""Shared QTableWidget cell helper.

The 9-line ``_set(row, col, text, color)`` body was byte-for-byte
identical in ``PastRunsTab`` (this package) and the dashboard's
``StateTableWidget`` / ``VictimsTableWidget``. Each class now delegates
to this free function. Mirrored per package because
``drone_rescue_ui_common`` is PyQt5-free by stance.
"""

from __future__ import annotations

from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QTableWidget, QTableWidgetItem


def set_cell(table: QTableWidget, row: int, col: int, text: str,
             color: str = '') -> None:
    """Set the item at ``(row, col)``, creating it if needed.

    ``color`` (CSS-style hex) optionally sets the foreground.
    """
    item = table.item(row, col)
    if item is None:
        item = QTableWidgetItem()
        table.setItem(row, col, item)
    item.setText(text)
    if color:
        item.setForeground(QColor(color))
