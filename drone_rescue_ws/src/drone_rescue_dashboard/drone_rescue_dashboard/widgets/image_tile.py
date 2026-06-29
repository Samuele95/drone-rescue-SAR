"""ImageTile: QLabel that polls ImageCache for a camera topic.

Extracted from ``dashboard_app.py`` so the widget is unit-testable
against a synthetic ImageCache.
"""

from __future__ import annotations

from typing import Optional

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import QLabel, QSizePolicy, QWidget

from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P


class ImageTile(QLabel):
    """A QLabel that polls ImageCache for a topic and paints the latest frame
    scaled to fit while preserving aspect ratio."""

    def __init__(self, topic: str, cache, title: str = '',
                 *, bridge=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._topic = topic
        self._cache = cache
        self.setMinimumSize(160, 120)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(
            f'background-color: {_P.bg_deep}; color: {_P.text_muted};'
        )
        self.setText(f'{title}\nwaiting for {topic}...')
        self._title = title
        # Bridge mode: repaint exactly when a new frame for THIS topic
        # arrives (plus on resize); the blanket 10 Hz rescale-and-repaint
        # timer is legacy-only.
        if bridge is not None:
            bridge.frame_arrived.connect(self._on_frame)
        else:
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._refresh)
            self._timer.start(100)   # 10 Hz repaint

    def _on_frame(self, topic: str) -> None:
        if topic == self._topic:
            self._refresh()

    def resizeEvent(self, event) -> None:  # noqa: N802 (Qt override)
        super().resizeEvent(event)
        self._refresh()

    def _refresh(self) -> None:
        img = self._cache.images.get(self._topic)
        if img is None:
            return
        scaled = img.scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )
        self.setPixmap(QPixmap.fromImage(scaled))
