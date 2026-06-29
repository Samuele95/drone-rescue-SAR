"""Scene3DView: the 3D mission sand-table.

A pyqtgraph.opengl ``GLViewWidget`` rendering the mission in 3D
alongside (never replacing) the canonical 2D top-down plan view:

- terrain grid + mission-disk boundary + sector spokes on the floor;
- the pheromone glow floor: searched ground literally lights up
  (amber) as the swarm covers it; unsearched terrain stays dark.
  This is the dashboard's signature: stigmergic coverage made
  visible;
- drone glyphs at true altitude with heading, an altitude stem down
  to their ground shadow, and identity-coloured trail ribbons;
- victim columns (amber candidate / green confirmed) with labels;
- extruded no-fly volumes;
- camera presets: free orbit (LMB drag / wheel), top-down (matches
  the 2D plan), and follow-drone.

Substrate: pyqtgraph.opengl chosen over VTK / librviz / web, already
installed with the ROS desktop stack, one rosdep line in Docker, and
it renders straight from the same ``MissionViewModel`` + trail deques
the 2D scene uses. Implements the ``SceneRenderer`` Protocol.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

from drone_rescue_dashboard.no_fly_zones import load_no_fly_zones

from drone_rescue_ui_common.constants import (
    DEFAULT_DRONE_NAMES, DRONE_COLORS,
)
from drone_rescue_ui_common.palette import DEFAULT_PALETTE as _P

import numpy as np

from pyqtgraph import Vector
import pyqtgraph.opengl as gl

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget,
)

TRAIL_Z = 0.15          # trails ride just above the glow floor
VICTIM_COLUMN_H = 6.0   # victim marker column height (m)
GLOW_MAX_ALPHA = 110    # pheromone floor peak alpha (0-255)


def _hex_to_rgbaf(hex_color: str, alpha: float = 1.0) -> Tuple[float, ...]:
    h = hex_color.lstrip('#')
    return (int(h[0:2], 16) / 255.0, int(h[2:4], 16) / 255.0,
            int(h[4:6], 16) / 255.0, alpha)


def _drone_glyph_mesh() -> gl.MeshData:
    """Build the drone glyph mesh.

    A flattened tetrahedron pointing +x, the 3D sibling of the 2D
    triangle cursor.
    """
    # ~2x the 2D cursor footprint: readable at the default 240 m
    # orbit distance while still plausibly drone-sized.
    verts = np.array([
        [4.4, 0.0, 0.0],     # nose
        [-2.4, 2.0, 0.0],    # tail left
        [-2.4, -2.0, 0.0],   # tail right
        [-1.2, 0.0, 1.6],    # top fin
    ])
    faces = np.array([[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]])
    return gl.MeshData(vertexes=verts, faces=faces)


class Scene3DView(QWidget):
    """SceneRenderer implementation on a GLViewWidget sand-table."""

    def __init__(self, state, no_fly_yaml_path: Optional[str] = None,
                 mission_radius: float = 85.0, bridge=None, parent=None):
        super().__init__(parent)
        self._state = state
        self._mission_radius = mission_radius
        self._follow: Optional[str] = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        # -- camera preset toolbar -----------------------------------
        bar = QWidget()
        bar.setStyleSheet(
            f'background: {_P.bg_panel};'
            f' border-bottom: 1px solid {_P.stroke};'
        )
        bl = QHBoxLayout(bar)
        bl.setContentsMargins(8, 3, 8, 3)
        bl.setSpacing(6)
        hint = QLabel('orbit: drag  •  zoom: wheel')
        hint.setStyleSheet(f'color: {_P.text_muted}; font-size: 9pt;')
        bl.addWidget(hint)
        bl.addStretch(1)
        self._follow_buttons: Dict[str, QPushButton] = {}
        # Compact-toolbar buttons: the app sheet's 7px button padding
        # makes the sizeHint ~36 px tall, which a 22 px fixed height
        # clipped (glyphs cut mid-character). Inline padding is
        # geometry-only; colors still come from the app sheet.
        for label, slot in (('⊙ top', self._camera_top),
                            ('⟲ orbit', self._camera_orbit)):
            b = QPushButton(label)
            b.setStyleSheet('padding: 2px 10px;')
            b.setFixedHeight(28)
            b.clicked.connect(slot)
            bl.addWidget(b)
        for name in DEFAULT_DRONE_NAMES:
            b = QPushButton(f'⛰ {name[-1]}')
            b.setToolTip(f'follow {name}')
            b.setFixedHeight(28)
            b.setCheckable(True)
            b.clicked.connect(
                lambda checked, n=name: self._toggle_follow(n, checked)
            )
            color = DRONE_COLORS.get(name, '#cccccc')
            b.setStyleSheet(f'color: {color}; padding: 2px 10px;')
            bl.addWidget(b)
            self._follow_buttons[name] = b
        lay.addWidget(bar)

        # -- GL canvas ------------------------------------------------
        self._gl = gl.GLViewWidget()
        self._gl.setBackgroundColor(_P.bg_deep)
        lay.addWidget(self._gl, stretch=1)
        self._camera_orbit()

        self._build_static_layers(no_fly_yaml_path)

        # -- dynamic items --------------------------------------------
        self._drone_items: Dict[str, Dict[str, object]] = {}
        self._trail_items: Dict[str, gl.GLLinePlotItem] = {}
        self._trail_seen: Dict[str, int] = {}
        self._victim_items: Dict[int, Dict[str, object]] = {}
        self._glow_item: Optional[gl.GLImageItem] = None
        self._glow_seen = -1

        if bridge is not None:
            bridge.view_changed.connect(self.render_from)
            bridge.trails_changed.connect(
                lambda: self.render_trails(
                    self._state.trails,
                    getattr(self._state, 'trails_appended', None) or {},
                )
            )
            bridge.view_changed.connect(lambda _v: self._update_glow())
        else:
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._refresh)
            self._timer.start(200)

    # ------------------------------------------------------ static set
    def _build_static_layers(self, no_fly_yaml_path: Optional[str]) -> None:
        grid = gl.GLGridItem()
        grid.setSize(220, 220)
        grid.setSpacing(20, 20)
        grid.setColor((48, 62, 84, 110))
        self._gl.addItem(grid)

        # Mission disk boundary.
        theta = np.linspace(0.0, 2.0 * np.pi, 128)
        r = self._mission_radius
        ring = np.column_stack([
            r * np.cos(theta), r * np.sin(theta), np.full_like(theta, 0.1),
        ])
        self._gl.addItem(gl.GLLinePlotItem(
            pos=ring, color=_hex_to_rgbaf(_P.text_muted, 0.6),
            width=1.5, antialias=True,
        ))
        # Sector spokes (4 sectors, matches the 2D plan).
        for i in range(4):
            a = i * math.pi / 2.0
            spoke = np.array([[0.0, 0.0, 0.1],
                              [r * math.cos(a), r * math.sin(a), 0.1]])
            self._gl.addItem(gl.GLLinePlotItem(
                pos=spoke, color=_hex_to_rgbaf(_P.stroke, 0.6),
                width=1.0, antialias=True,
            ))
        # Compass.
        for label, x, y in (('N', 0, r + 7), ('E', r + 7, 0),
                            ('S', 0, -r - 7), ('W', -r - 7, 0)):
            self._gl.addItem(gl.GLTextItem(
                pos=(x, y, 0.5), text=label,
                color=_hex_to_rgbaf(_P.text_muted),
            ))

        self._build_no_fly_volumes(no_fly_yaml_path)

    def _build_no_fly_volumes(self, yaml_path: Optional[str]) -> None:
        # Shared loader; zones arrive normalised, so no per-kind
        # validation here.
        red = _hex_to_rgbaf(_P.error, 0.25)
        edge = _hex_to_rgbaf(_P.error, 0.8)
        height = 18.0
        for z in load_no_fly_zones(yaml_path):
            if z['type'] == 'circle':
                cx, cy = z['center']
                md = gl.MeshData.cylinder(
                    rows=1, cols=24, radius=[z['radius'], z['radius']],
                    length=height,
                )
                item = gl.GLMeshItem(
                    meshdata=md, smooth=True, color=red,
                    shader='shaded', glOptions='translucent',
                )
                item.translate(cx, cy, 0.0)
                self._gl.addItem(item)
            else:
                pts = z['vertices']
                # Wireframe prism: floor + roof rings and risers.
                for zz in (0.1, height):
                    ring = np.array(
                        [[p[0], p[1], zz] for p in pts + [pts[0]]]
                    )
                    self._gl.addItem(gl.GLLinePlotItem(
                        pos=ring, color=edge, width=1.5, antialias=True,
                    ))
                for p in pts:
                    riser = np.array([[p[0], p[1], 0.1],
                                      [p[0], p[1], height]])
                    self._gl.addItem(gl.GLLinePlotItem(
                        pos=riser, color=edge, width=1.0, antialias=True,
                    ))

    # ------------------------------------------------------ camera ops
    def _camera_top(self) -> None:
        self._set_follow(None)
        self._gl.setCameraPosition(
            pos=Vector(0, 0, 0), distance=260, elevation=90, azimuth=-90,
        )

    def _camera_orbit(self) -> None:
        self._set_follow(None)
        self._gl.setCameraPosition(
            pos=Vector(0, 0, 0), distance=240, elevation=32, azimuth=-60,
        )

    def _toggle_follow(self, name: str, checked: bool) -> None:
        self._set_follow(name if checked else None)

    def _set_follow(self, name: Optional[str]) -> None:
        self._follow = name
        for n, b in self._follow_buttons.items():
            b.setChecked(n == name)

    # ------------------------------------------------------- legacy tick
    def _refresh(self) -> None:
        self.render_trails(
            self._state.trails,
            getattr(self._state, 'trails_appended', None) or {},
        )
        self.render_from(self._state.view)
        self._update_glow()

    # ------------------------------------------------ SceneRenderer API
    def render_from(self, view) -> None:
        """Place drones + victims from the frozen view snapshot.

        SceneRenderer Protocol implementation.
        """
        for name, ds in view.drones.items():
            if ds.peer_last_seen <= 0:
                continue
            items = self._drone_items.get(name)
            if items is None:
                items = self._make_drone_items(name)
                self._drone_items[name] = items
            self._place_drone(items, ds)
            if self._follow == name:
                self._gl.setCameraPosition(
                    pos=Vector(ds.pose_x, ds.pose_y, ds.pose_z),
                )

        for cid, vv in view.victims.items():
            items = self._victim_items.get(cid)
            if items is None:
                items = self._make_victim_items(cid)
                self._victim_items[cid] = items
            self._place_victim(items, vv)

    def render_trails(self, trails, appended) -> None:
        """Paint identity-coloured, ground-projected trail ribbons.

        SceneRenderer Protocol implementation; the decision record
        pins trails to 2D ground projection.
        """
        for name, trail in trails.items():
            if not trail:
                continue
            count = appended.get(name, 0)
            if self._trail_seen.get(name) == count:
                continue
            pts = np.array([[x, y, TRAIL_Z] for x, y in trail])
            item = self._trail_items.get(name)
            if item is None:
                color = _hex_to_rgbaf(
                    DRONE_COLORS.get(name, '#cccccc'), 0.85,
                )
                item = gl.GLLinePlotItem(
                    pos=pts, color=color, width=2.0, antialias=True,
                )
                self._gl.addItem(item)
                self._trail_items[name] = item
            else:
                item.setData(pos=pts)
            self._trail_seen[name] = count

    # ----------------------------------------------------- glow floor
    def _update_glow(self) -> None:
        """Pheromone glow floor: searched ground lights up amber."""
        version = getattr(self._state, 'pheromone_version', 0)
        if version == self._glow_seen:
            return
        grid = getattr(self._state, 'pheromone', None)
        meta = getattr(self._state, 'pheromone_meta', None)
        if grid is None or meta is None:
            return
        self._glow_seen = version
        h, w = grid.shape
        rgba = np.zeros((w, h, 4), dtype=np.ubyte)
        # amber #f59e0b; alpha scales with intensity.
        levels = np.clip(grid, 0.0, 1.0).T
        rgba[..., 0] = 245
        rgba[..., 1] = 158
        rgba[..., 2] = 11
        rgba[..., 3] = (levels * GLOW_MAX_ALPHA).astype(np.ubyte)
        if self._glow_item is None:
            self._glow_item = gl.GLImageItem(rgba, glOptions='additive')
            res = float(meta['resolution'])
            self._glow_item.scale(res, res, 1.0)
            self._glow_item.translate(
                float(meta['origin_x']), float(meta['origin_y']), 0.05,
            )
            self._gl.addItem(self._glow_item)
        else:
            self._glow_item.setData(rgba)

    # -------------------------------------------------------- builders
    def _make_drone_items(self, name: str):
        color = _hex_to_rgbaf(DRONE_COLORS.get(name, '#cccccc'))
        glyph = gl.GLMeshItem(
            meshdata=_drone_glyph_mesh(), smooth=False, color=color,
            shader='shaded', drawEdges=False,
        )
        self._gl.addItem(glyph)
        stem = gl.GLLinePlotItem(
            pos=np.zeros((2, 3)),
            color=_hex_to_rgbaf(DRONE_COLORS.get(name, '#cccccc'), 0.45),
            width=1.0, antialias=True,
        )
        self._gl.addItem(stem)
        shadow = gl.GLScatterPlotItem(
            pos=np.zeros((1, 3)),
            color=_hex_to_rgbaf(DRONE_COLORS.get(name, '#cccccc'), 0.5),
            size=4.0,
        )
        self._gl.addItem(shadow)
        label = gl.GLTextItem(
            pos=(0.0, 0.0, 0.0), text=name,
            color=_hex_to_rgbaf(DRONE_COLORS.get(name, '#cccccc')),
        )
        self._gl.addItem(label)
        return {'glyph': glyph, 'stem': stem, 'shadow': shadow,
                'label': label}

    def _place_drone(self, items, ds) -> None:
        x, y, z = ds.pose_x, ds.pose_y, max(ds.pose_z, 0.0)
        glyph = items['glyph']
        glyph.resetTransform()
        glyph.translate(x, y, z)
        glyph.rotate(math.degrees(ds.yaw), 0, 0, 1, local=True)
        if ds.unrecoverable or ds.is_down:
            glyph.setColor((0.94, 0.27, 0.27, 0.9))   # palette error
        items['stem'].setData(
            pos=np.array([[x, y, 0.0], [x, y, z]]),
        )
        items['shadow'].setData(pos=np.array([[x, y, 0.05]]))
        items['label'].setData(pos=(x + 2.0, y + 2.0, z + 1.5))

    def _make_victim_items(self, cid: int):
        column = gl.GLLinePlotItem(
            pos=np.zeros((2, 3)), color=_hex_to_rgbaf(_P.warn, 0.8),
            width=2.5, antialias=True,
        )
        self._gl.addItem(column)
        head = gl.GLScatterPlotItem(
            pos=np.zeros((1, 3)), color=_hex_to_rgbaf(_P.warn),
            size=10.0,
        )
        self._gl.addItem(head)
        label = gl.GLTextItem(
            pos=(0.0, 0.0, 0.0), text=f'v{cid}',
            color=_hex_to_rgbaf(_P.warn),
        )
        self._gl.addItem(label)
        return {'column': column, 'head': head, 'label': label}

    def _place_victim(self, items, vv) -> None:
        x, y = vv.position
        token = _P.ok if vv.confirmed else _P.warn
        h = VICTIM_COLUMN_H if vv.confirmed else VICTIM_COLUMN_H * 0.6
        items['column'].setData(
            pos=np.array([[x, y, 0.0], [x, y, h]]),
            color=_hex_to_rgbaf(token, 0.8),
        )
        items['head'].setData(
            pos=np.array([[x, y, h]]), color=_hex_to_rgbaf(token),
        )
        items['label'].setData(
            pos=(x + 1.2, y + 1.2, h + 0.8), color=_hex_to_rgbaf(token),
        )
