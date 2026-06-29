"""Native Qt top-down mission scene, replacing the embedded RViz.

A QGraphicsView shows the disk, the no-fly zones, the spiral pattern in
the background, the drone trails, the current drone positions (colored
arrow with heading from quaternion), and the victim markers. Refreshes at
5 Hz from the dashboard's StateCache.

World coordinates: x east, y north, in metres. The view scales by
`PIXELS_PER_METER` and flips y so up is north on screen.

We deliberately keep this 2D top-down (no 3D camera). For SAR a top-down
plan view is the canonical operator display anyway, and it avoids
embedding a heavy 3D engine. The view supports mouse-wheel zoom and pan.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from python_qt_binding.QtCore import (
    Qt, QTimer, QPointF, QVariantAnimation, QEasingCurve, Signal,
)
from python_qt_binding.QtWidgets import QMenu
from python_qt_binding.QtGui import (
    QPainter, QColor, QPen, QBrush, QPainterPath, QPolygonF, QFont,
    QTransform,
)
from python_qt_binding.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsItem,
    QGraphicsEllipseItem, QGraphicsPolygonItem, QGraphicsPathItem,
    QGraphicsSimpleTextItem,
)


from drone_rescue_dashboard.no_fly_zones import (
    load_no_fly_zones, polygon_centroid,
)
from drone_rescue_ui_common.constants import DRONE_COLORS as _DRONE_COLOR_HEX
# Semantic status colours from the palette (scene-geometry colours
# stay local per the palette docstring).
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P


PIXELS_PER_METER = 4.0          # 4 px/m → 100 m disk fits ~800 px wide
WORLD_HALF_SPAN_M = 110.0       # scene rect: ±110 m

# Canonical per-drone colours live in
# ``drone_rescue_ui_common.constants.DRONE_COLORS``; this module
# wraps each hex code in a ``QColor`` once at import time so the
# refresh path stays cheap.
_DRONE_COLORS = {name: QColor(hex_code)
                 for name, hex_code in _DRONE_COLOR_HEX.items()}


class MissionSceneView(QGraphicsView):
    # Operator interactions. Click (without drag) selects the nearest
    # drone/victim for the inspector; right-click opens the
    # investigate-here context menu.
    drone_clicked = Signal(str)
    victim_clicked = Signal(int)
    investigate_requested = Signal(float, float)

    #: click-vs-pan discrimination threshold (px)
    _CLICK_SLOP_PX = 6
    #: hit-test radii (scene metres)
    _DRONE_HIT_M = 5.0
    _VICTIM_HIT_M = 4.0

    def __init__(self, state, no_fly_yaml_path: Optional[str] = None,
                 mission_radius: float = 85.0,
                 mission_center: Tuple[float, float] = (0.0, 0.0),
                 bridge=None, parent=None):
        super().__init__(parent)
        self._state = state
        self._mission_radius = mission_radius
        self._mission_center = mission_center
        self._scene = QGraphicsScene(self)
        self._scene.setSceneRect(
            -WORLD_HALF_SPAN_M, -WORLD_HALF_SPAN_M,
            2 * WORLD_HALF_SPAN_M, 2 * WORLD_HALF_SPAN_M,
        )
        self._scene.setBackgroundBrush(QBrush(QColor(_P.bg_deep)))
        self.setScene(self._scene)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # World y-up to screen y-down.
        t = QTransform()
        t.scale(PIXELS_PER_METER, -PIXELS_PER_METER)
        self.setTransform(t)
        self.setDragMode(QGraphicsView.ScrollHandDrag)

        # Static layer: mission disk + grid + sector boundaries + no-fly.
        self._draw_grid()
        self._draw_disk_boundary()
        self._draw_sector_boundaries()
        self._draw_compass()
        self._draw_no_fly_zones(no_fly_yaml_path)

        # Dynamic layer state.
        self._trail_items: Dict[str, QGraphicsPathItem] = {}
        # Last-rendered trail length per drone so the
        # 5 Hz `_refresh` skips QPainterPath rebuild when the deque
        # hasn't grown. _on_odom only appends to the trail when the
        # new point is >0.5 m from the last, so most refresh ticks
        # are no-ops in steady-state hover.
        self._trail_lens: Dict[str, int] = {}
        self._drone_cursors: Dict[str, QGraphicsPolygonItem] = {}
        self._drone_labels: Dict[str, QGraphicsSimpleTextItem] = {}
        self._down_markers: Dict[str, List[QGraphicsItem]] = {}
        self._victim_items: Dict[int, Tuple[QGraphicsItem, QGraphicsSimpleTextItem]] = {}
        self._init_drone_layers()
        self._draw_legend()

        # Per-drone cursor tween animations so the marker glides
        # between updates instead of jumping at the 2 Hz peer-state
        # cadence. GUI-thread only.
        self._cursor_anims: Dict[str, QVariantAnimation] = {}
        # Victims already seen as confirmed, so the one-shot
        # confirmation pulse fires exactly once per victim.
        self._pulsed_confirmed: set = set()

        # Bridge mode renders on change signals; legacy mode keeps the
        # 5 Hz poll.
        if bridge is not None:
            bridge.view_changed.connect(self.render_from)
            bridge.trails_changed.connect(
                lambda: self.render_trails(
                    self._state.trails,
                    getattr(self._state, 'trails_appended', None) or {},
                )
            )
        else:
            self._refresh_timer = QTimer(self)
            self._refresh_timer.timeout.connect(self._refresh)
            self._refresh_timer.start(200)

    # -------------------------------------------------------- static layers
    def _draw_grid(self) -> None:
        # 20 m gridlines.
        pen = QPen(QColor('#1e293b'))
        pen.setWidthF(0.15)
        for v in range(-100, 101, 20):
            self._scene.addLine(v, -100, v, 100, pen)
            self._scene.addLine(-100, v, 100, v, pen)
        # Axes
        axis_pen = QPen(QColor('#334155'))
        axis_pen.setWidthF(0.3)
        self._scene.addLine(-100, 0, 100, 0, axis_pen)
        self._scene.addLine(0, -100, 0, 100, axis_pen)

    def _draw_disk_boundary(self) -> None:
        cx, cy = self._mission_center
        r = self._mission_radius
        item = QGraphicsEllipseItem(cx - r, cy - r, 2 * r, 2 * r)
        pen = QPen(QColor('#94a3b8'))
        pen.setWidthF(0.4)
        pen.setStyle(Qt.DashLine)
        item.setPen(pen)
        item.setBrush(QBrush(Qt.NoBrush))
        self._scene.addItem(item)

    def _draw_sector_boundaries(self, n: int = 4) -> None:
        # Radial lines from origin to disk edge separating drone sectors.
        cx, cy = self._mission_center
        r = self._mission_radius
        pen = QPen(QColor('#334155'))
        pen.setWidthF(0.25)
        for i in range(n):
            theta = i * (2.0 * math.pi / n)
            x = cx + r * math.cos(theta)
            y = cy + r * math.sin(theta)
            self._scene.addLine(cx, cy, x, y, pen)

    def _draw_compass(self) -> None:
        # N/E/S/W labels just outside the disk.
        cx, cy = self._mission_center
        r = self._mission_radius + 6
        font = QFont(); font.setPointSize(2)  # in scene units (4px/m)
        for label, dx, dy in [('N', 0, r), ('E', r, 0), ('S', 0, -r), ('W', -r, 0)]:
            t = self._scene.addSimpleText(label, font)
            t.setBrush(QBrush(QColor('#64748b')))
            br = t.boundingRect()
            t.setPos(cx + dx - br.width() / 2 / PIXELS_PER_METER,
                     cy + dy + br.height() / 2 / PIXELS_PER_METER)
            # Re-flip the text upright since the view is y-flipped.
            tt = QTransform()
            tt.scale(1.0 / PIXELS_PER_METER, -1.0 / PIXELS_PER_METER)
            t.setTransform(tt)

    def _draw_no_fly_zones(self, yaml_path: Optional[str]) -> None:
        # Shared loader + centroid helper; zones arrive normalised, so
        # no per-kind validation here.
        for z in load_no_fly_zones(yaml_path):
            color = QColor('#dc2626')
            color.setAlpha(60)
            pen = QPen(QColor('#dc2626'))
            pen.setWidthF(0.3)
            if z['type'] == 'polygon':
                poly = QPolygonF(
                    [QPointF(p[0], p[1]) for p in z['vertices']]
                )
                item = QGraphicsPolygonItem(poly)
                label_pos = polygon_centroid(z['vertices'])
            else:
                cx, cy = z['center']
                r = z['radius']
                item = QGraphicsEllipseItem(cx - r, cy - r, 2 * r, 2 * r)
                label_pos = (cx, cy)
            item.setPen(pen)
            item.setBrush(QBrush(color))
            self._scene.addItem(item)
            if z['name']:
                font = QFont(); font.setPointSize(2)
                t = self._scene.addSimpleText(z['name'], font)
                t.setBrush(QBrush(QColor('#fca5a5')))
                t.setPos(*label_pos)
                tt = QTransform()
                tt.scale(1.0 / PIXELS_PER_METER, -1.0 / PIXELS_PER_METER)
                t.setTransform(tt)

    # -------------------------------------------------------- drone layers
    def _init_drone_layers(self) -> None:
        for drone, color in _DRONE_COLORS.items():
            # Trail
            trail_pen = QPen(color)
            trail_pen.setWidthF(0.4)
            trail = QGraphicsPathItem()
            trail.setPen(trail_pen)
            self._scene.addItem(trail)
            self._trail_items[drone] = trail
            # Cursor (filled triangle pointing in heading direction)
            tri = QPolygonF([
                QPointF(2.0, 0.0),
                QPointF(-1.2, 1.0),
                QPointF(-1.2, -1.0),
            ])
            cur = QGraphicsPolygonItem(tri)
            cur.setPen(QPen(Qt.NoPen))
            cur.setBrush(QBrush(color))
            cur.setVisible(False)
            self._scene.addItem(cur)
            self._drone_cursors[drone] = cur
            # Label
            font = QFont()
            font.setPointSize(2)
            font.setBold(True)
            label = QGraphicsSimpleTextItem(drone)
            label.setBrush(QBrush(color))
            label.setVisible(False)
            label.setFont(font)
            tt = QTransform()
            tt.scale(1.0 / PIXELS_PER_METER, -1.0 / PIXELS_PER_METER)
            label.setTransform(tt)
            self._scene.addItem(label)
            self._drone_labels[drone] = label

    # -------------------------------------------------------- refresh tick
    def _refresh(self) -> None:
        # SceneRenderer contract: the tick dispatches the two protocol
        # methods; trails and view-model rendering are separate paths
        # (trails are high-frequency append-only history with no
        # MissionViewModel analogue).
        self.render_trails(
            self._state.trails,
            getattr(self._state, 'trails_appended', None) or {},
        )
        self.render_from(self._state.view)

    def render_trails(self, trails, appended) -> None:
        """SceneRenderer Protocol: paint the per-drone trails.

        Rebuild the QPainterPath only when a new point was appended
        since the previous refresh, judged via the monotonic
        ``appended`` counter. An earlier version used ``len(trail)``
        for this, but once the bounded deque hits ``maxlen=400`` len()
        plateaus while new points keep evicting old ones (the same
        plateau bug seen in widgets/active_tab.py); the counter
        (written by DashboardSubscriberNode._on_odom) avoids it.
        """
        for drone, trail in trails.items():
            item = self._trail_items.get(drone)
            if item is None:
                continue
            if not trail:
                continue
            count = appended.get(drone, 0)
            if self._trail_lens.get(drone) == count:
                continue
            path = QPainterPath()
            it = iter(trail)
            x0, y0 = next(it)
            path.moveTo(x0, y0)
            for x, y in it:
                path.lineTo(x, y)
            item.setPath(path)
            self._trail_lens[drone] = count

    def render_from(self, view) -> None:
        """OperatorView Protocol: render the MissionViewModel
        projection onto the Qt scene.

        Cursor + label read from ``view.drones`` (DroneViewState
        carries pose_x, pose_y, yaw, is_down, unrecoverable, wp_index,
        wp_total). Trails are dashboard-side state and stay on the
        timer-driven refresh path."""
        for drone, cursor in self._drone_cursors.items():
            ds = view.drones.get(drone)
            label = self._drone_labels.get(drone)
            if ds is None or ds.peer_last_seen <= 0:
                continue
            x = ds.pose_x
            y = ds.pose_y
            first_show = not cursor.isVisible()
            cursor.setVisible(True)
            self._glide_cursor(drone, cursor, x, y, snap=first_show)
            tr = QTransform()
            tr.rotateRadians(ds.yaw)
            cursor.setTransform(tr)
            is_down = bool(ds.unrecoverable or ds.is_down)
            base_color = QColor(_DRONE_COLORS.get(drone, QColor('#fff')))
            if is_down:
                base_color.setAlpha(120)
                cursor.setBrush(QBrush(base_color))
                self._draw_or_update_down_marker(drone, x, y)
            else:
                base_color.setAlpha(255)
                cursor.setBrush(QBrush(base_color))
                self._clear_down_marker(drone)
            if label is not None:
                label.setVisible(True)
                tag = (f'{drone}  WP {ds.wp_index}/{ds.wp_total}'
                       if ds.wp_total > 0 else drone)
                if is_down:
                    tag = f'{drone}  ✕ DOWN'
                label.setText(tag)
                label.setPos(x + 1.5, y + 2.5)

        # Victims: read from view.victims (VictimViewState).
        for vid, vv in view.victims.items():
            existing = self._victim_items.get(vid)
            confirmed = bool(vv.confirmed)
            color = QColor(_P.ok) if confirmed else QColor(_P.warn)
            r = 1.4 if confirmed else 1.0
            x, y = vv.position[0], vv.position[1]
            if existing is None:
                ring = QGraphicsEllipseItem(x - r, y - r, 2 * r, 2 * r)
                pen = QPen(color); pen.setWidthF(0.3)
                ring.setPen(pen)
                fill = QColor(color); fill.setAlpha(120)
                ring.setBrush(QBrush(fill))
                self._scene.addItem(ring)
                font = QFont(); font.setPointSize(2)
                tx = self._scene.addSimpleText(f'v{vid}', font)
                tx.setBrush(QBrush(color))
                tx.setPos(x + r + 0.3, y + r)
                tt = QTransform()
                tt.scale(1.0 / PIXELS_PER_METER, -1.0 / PIXELS_PER_METER)
                tx.setTransform(tt)
                self._victim_items[vid] = (ring, tx)
            else:
                ring, tx = existing
                ring.setRect(x - r, y - r, 2 * r, 2 * r)
                pen = QPen(color); pen.setWidthF(0.3)
                ring.setPen(pen)
                fill = QColor(color); fill.setAlpha(120)
                ring.setBrush(QBrush(fill))
                tx.setBrush(QBrush(color))
                tx.setPos(x + r + 0.3, y + r)
            # One-shot expanding pulse the first time a victim
            # is seen confirmed (mirrors the RViz flash ring).
            if confirmed and vid not in self._pulsed_confirmed:
                self._pulsed_confirmed.add(vid)
                self._pulse_at(x, y)

    def _glide_cursor(self, drone: str, cursor, x: float, y: float,
                      snap: bool = False) -> None:
        """Tween the cursor towards (x, y) over ~160 ms.

        Peer state arrives at ~2 Hz; an immediate ``setPos`` makes the
        marker jump half a metre per update. The short tween reads as
        continuous motion while staying well under the update period.
        ``snap`` skips the tween (first appearance).
        """
        current = cursor.pos()
        if snap or (abs(current.x() - x) + abs(current.y() - y)) < 1e-6:
            cursor.setPos(x, y)
            return
        anim = self._cursor_anims.get(drone)
        if anim is None:
            anim = QVariantAnimation(self)
            anim.setDuration(160)
            anim.setEasingCurve(QEasingCurve.OutCubic)
            anim.valueChanged.connect(
                lambda value, c=cursor: c.setPos(value)
            )
            self._cursor_anims[drone] = anim
        anim.stop()
        anim.setStartValue(current)
        anim.setEndValue(QPointF(x, y))
        anim.start()

    def _pulse_at(self, x: float, y: float) -> None:
        """700 ms expanding, fading ring at (x, y)."""
        ring = QGraphicsEllipseItem(x - 1, y - 1, 2, 2)
        pen = QPen(QColor(_P.ok))
        pen.setWidthF(0.5)
        ring.setPen(pen)
        ring.setBrush(QBrush(Qt.NoBrush))
        self._scene.addItem(ring)
        anim = QVariantAnimation(self)
        anim.setDuration(700)
        anim.setStartValue(1.0)
        anim.setEndValue(9.0)
        anim.setEasingCurve(QEasingCurve.OutCubic)

        def _step(r, item=ring, cx=x, cy=y):
            item.setRect(cx - r, cy - r, 2 * r, 2 * r)
            item.setOpacity(max(0.0, 1.0 - (r - 1.0) / 8.0))

        anim.valueChanged.connect(_step)
        anim.finished.connect(lambda: self._scene.removeItem(ring))
        anim.start()

    # ---------------------------------------------------- down markers
    def _draw_or_update_down_marker(self, drone: str, x: float, y: float) -> None:
        if drone in self._down_markers:
            return  # already drawn at last-known pose; don't re-add per tick
        size = 2.5
        pen = QPen(QColor(_P.error))
        pen.setWidthF(0.6)
        l1 = self._scene.addLine(x - size, y - size, x + size, y + size, pen)
        l2 = self._scene.addLine(x - size, y + size, x + size, y - size, pen)
        self._down_markers[drone] = [l1, l2]

    def _clear_down_marker(self, drone: str) -> None:
        items = self._down_markers.pop(drone, None)
        if not items:
            return
        for it in items:
            self._scene.removeItem(it)

    # ---------------------------------------------------- legend
    def _draw_legend(self) -> None:
        # Top-left of scene.
        x0 = -WORLD_HALF_SPAN_M + 4
        y0 = WORLD_HALF_SPAN_M - 4
        rows = [
            ('— mission disk', _P.text_muted),
            ('— no-fly zone', '#fca5a5'),     # geometry-local (matches zone fill)
            ('▲ drone', '#3b82f6'),           # drone identity colour, not status
            ('● candidate', _P.warn),
            ('● confirmed', _P.ok),
            ('✕ DOWN', _P.error),
        ]
        font = QFont(); font.setPointSize(2)
        for i, (text, color) in enumerate(rows):
            t = self._scene.addSimpleText(text, font)
            t.setBrush(QBrush(QColor(color)))
            t.setPos(x0, y0 - i * 4)
            tt = QTransform()
            tt.scale(1.0 / PIXELS_PER_METER, -1.0 / PIXELS_PER_METER)
            t.setTransform(tt)

    # -------------------------------------------------------- mouse
    def wheelEvent(self, event) -> None:
        # Zoom around mouse cursor.
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def mousePressEvent(self, event) -> None:  # noqa: N802 (Qt override)
        if event.button() == Qt.LeftButton:
            self._press_pos = event.pos()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802 (Qt override)
        super().mouseReleaseEvent(event)
        press = getattr(self, '_press_pos', None)
        if (event.button() != Qt.LeftButton or press is None
                or (event.pos() - press).manhattanLength()
                > self._CLICK_SLOP_PX):
            return   # it was a pan, not a click
        hit = self._hit_test(self.mapToScene(event.pos()))
        if hit is None:
            return
        kind, key = hit
        if kind == 'drone':
            self.drone_clicked.emit(key)
        else:
            self.victim_clicked.emit(key)

    def contextMenuEvent(self, event) -> None:  # noqa: N802 (Qt override)
        scene_pos = self.mapToScene(event.pos())
        menu = QMenu(self)
        act = menu.addAction(
            f'⌖ Investigate here  ({scene_pos.x():.1f}, {scene_pos.y():.1f})'
        )
        chosen = menu.exec_(event.globalPos())
        if chosen is act:
            self.investigate_requested.emit(scene_pos.x(), scene_pos.y())

    def _hit_test(self, p: QPointF):
        """Nearest drone within _DRONE_HIT_M, else nearest victim
        within _VICTIM_HIT_M, else None."""
        view = self._state.view
        best = None
        best_d2 = self._DRONE_HIT_M ** 2
        for name, ds in view.drones.items():
            if ds.peer_last_seen <= 0:
                continue
            d2 = (ds.pose_x - p.x()) ** 2 + (ds.pose_y - p.y()) ** 2
            if d2 <= best_d2:
                best, best_d2 = ('drone', name), d2
        if best is not None:
            return best
        best_d2 = self._VICTIM_HIT_M ** 2
        for cid, vv in view.victims.items():
            d2 = ((vv.position[0] - p.x()) ** 2
                  + (vv.position[1] - p.y()) ** 2)
            if d2 <= best_d2:
                best, best_d2 = ('victim', cid), d2
        return best
