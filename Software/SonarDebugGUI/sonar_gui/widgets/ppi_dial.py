"""PPIDialWidget — сонарный PPI-экран положения антенны."""
from __future__ import annotations

import math
from collections import deque

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QPainter,
    QPen,
    QPolygonF,
    QRadialGradient,
)
from PySide6.QtWidgets import QWidget

from ..device_state import DeviceState
from ..metrics import METRICS
from ..theme import COLORS, mono_font, ui_font

_TRAIL_LEN = 34
_FPS_MS = 33
_HUD_TOP = 22.0
_HUD_BOTTOM = 44.0
_MARGIN = 22.0
_BEAM_HALF_DEG = 3.5

_TICK_VECS = tuple(
    (math.sin(math.radians(d)), -math.cos(math.radians(d)))
    for d in range(0, 360, 5)
)
_LABEL_VECS = tuple(
    (math.sin(math.radians(d)), -math.cos(math.radians(d)))
    for d in range(0, 360, 30)
)
_LABEL_TEXTS = tuple(str(d) for d in range(0, 360, 30))


def _col(key: str) -> QColor:
    return QColor(COLORS[key])


class PPIDialWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)

        self._cp: float | None = None
        self._tp: float | None = None
        self._ec: int | None = None
        self._scan = None
        self._connected = False
        self._has_telemetry = False

        self._scale = 1.0
        self._offset = 0.0
        self._sign = 1
        self._trail: deque[float] = deque(maxlen=_TRAIL_LEN)

        self._init_pens()
        self._geom_key = None
        self._b_disk = QBrush(_col("ppi_edge"))
        self._b_cone = QBrush(Qt.NoBrush)

        self._timer = QTimer(self)
        self._timer.setInterval(_FPS_MS)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start()

    def _init_pens(self) -> None:
        self._c_bg = _col("bg")
        self._c_well = _col("well")
        self._c_border = _col("border")
        self._c_border2 = _col("border2")
        self._c_accent = _col("accent")
        self._c_accent2 = _col("accent2")
        self._c_cp = _col("cp")
        self._c_amber = _col("amber")
        self._c_danger = _col("danger")
        self._c_text_dim = _col("text_dim")
        self._c_text_faint = _col("text_faint")
        self._c_ppi_grid = _col("ppi_grid")
        self._c_ppi_label = _col("ppi_label")
        self._c_disk_center = _col("ppi_center")
        self._c_disk_mid = QColor("#091620")
        self._c_disk_edge = _col("ppi_edge")

        scan_fill = QColor(self._c_accent2)
        scan_fill.setAlpha(32)
        beam_glow = QColor(self._c_cp)
        beam_glow.setAlpha(40)
        target_ray = QColor(self._c_amber)
        target_ray.setAlpha(90)

        self._pen_grid = QPen(self._c_ppi_grid, 1)
        self._pen_tick_minor = QPen(self._c_ppi_grid, 1)
        self._pen_tick_major = QPen(self._c_text_dim, 1.6)
        self._pen_bezel = QPen(self._c_border2, 2)
        self._pen_scan_edge = QPen(QColor(self._c_accent2.red(), self._c_accent2.green(),
                                          self._c_accent2.blue(), 120), 1, Qt.DashLine)
        self._pen_scan_arc = QPen(self._c_accent2, 2)
        self._pen_target = QPen(target_ray, 1.4, Qt.DashLine)
        self._pen_beam = QPen(self._c_cp, 2.5, Qt.SolidLine, Qt.RoundCap)
        self._pen_beam_glow = QPen(beam_glow, 7, Qt.SolidLine, Qt.RoundCap)
        self._pen_trail = QPen(self._c_cp, 2)
        self._pen_border = QPen(self._c_border, 1)
        self._pen_label = QPen(self._c_ppi_label)
        self._pen_label_card = QPen(self._c_accent)
        self._pen_accent = QPen(self._c_accent)
        self._pen_amber = QPen(self._c_amber)
        self._pen_danger = QPen(self._c_danger)
        self._pen_faint = QPen(self._c_text_faint)
        self._b_scan = QBrush(scan_fill)
        self._b_target = QBrush(self._c_amber)
        self._b_beam_dot = QBrush(self._c_accent)
        self._b_beam_core = QBrush(QColor("#ffffff"))
        self._b_strip = QBrush(self._c_well)
        self._c_trail = QColor(self._c_cp)
        self._f_tick = ui_font(9)
        self._f_tick_card = ui_font(9, bold=True)
        self._f_nodata = ui_font(12, bold=True)
        self._f_mono_big = mono_font(20, bold=True)
        self._f_mono_small = mono_font(11)

    def apply_state(self, st: DeviceState) -> None:
        self._connected = st.connected
        self._has_telemetry = st.has_telemetry
        self._cp = st.cp
        self._tp = st.tp
        self._ec = st.ec
        if not st.connected:
            self._trail.clear()

    def set_scan(self, sector) -> None:
        self._scan = sector

    def set_config(self, scale: float, offset: float, sign: int) -> None:
        self._scale = scale
        self._offset = offset
        self._sign = 1 if sign >= 0 else -1

    def antenna_angle(self, cp: float) -> float:
        return self._sign * (cp * self._scale) + self._offset

    def _on_tick(self) -> None:
        if self._connected and self._cp is not None:
            self._trail.append(self.antenna_angle(self._cp))
        self.update()

    def _point(self, cx, cy, r, angle_deg) -> QPointF:
        a = math.radians(angle_deg)
        return QPointF(cx + r * math.sin(a), cy - r * math.cos(a))

    def _ensure_geom(self, cx: float, cy: float, r: float) -> None:
        key = (round(cx, 1), round(cy, 1), round(r, 1))
        if key == self._geom_key:
            return
        self._geom_key = key
        disk = QRadialGradient(cx, cy, r * 1.1)
        disk.setColorAt(0.0, self._c_disk_center)
        disk.setColorAt(0.78, self._c_disk_mid)
        disk.setColorAt(1.0, self._c_disk_edge)
        self._b_disk = QBrush(disk)
        cone = QRadialGradient(cx, cy, r)
        c0 = QColor(self._c_cp)
        c0.setAlpha(90)
        c1 = QColor(self._c_cp)
        c1.setAlpha(0)
        cone.setColorAt(0.0, c0)
        cone.setColorAt(1.0, c1)
        self._b_cone = QBrush(cone)

    def paintEvent(self, _ev) -> None:
        w, h = float(self.width()), float(self.height())
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), self._c_bg)
        self._draw_strips(p, w, h)

        top, bottom = _HUD_TOP, h - _HUD_BOTTOM
        avail = bottom - top
        cx = w / 2.0
        cy = top + avail / 2.0
        r = min(w / 2.0, avail / 2.0) - _MARGIN
        if r < 40:
            p.end()
            return

        self._ensure_geom(cx, cy, r)
        show_data = self._connected and self._has_telemetry and self._cp is not None

        if not show_data:
            p.setOpacity(0.35)
            self._draw_disk(p, cx, cy, r)
            self._draw_grid(p, cx, cy, r)
            self._draw_ticks(p, cx, cy, r)
            self._draw_bezel(p, cx, cy, r)
            p.setOpacity(1.0)
            p.setFont(self._f_nodata)
            p.setPen(self._pen_faint)
            msg = "НЕТ ДАННЫХ" if self._connected else "НЕ ПОДКЛЮЧЕНО"
            p.drawText(QRectF(cx - 120, cy - 12, 240, 24), Qt.AlignCenter, msg)
            self._draw_hud_bottom(p, w, h, None, None)
            p.end()
            return

        self._draw_disk(p, cx, cy, r)
        self._draw_grid(p, cx, cy, r)
        if self._scan is not None:
            self._draw_scan(p, cx, cy, r)
        self._draw_trail(p, cx, cy, r)
        if self._tp is not None:
            self._draw_target(p, cx, cy, r)
        self._draw_beam(p, cx, cy, r)
        self._draw_ticks(p, cx, cy, r)
        self._draw_bezel(p, cx, cy, r)
        self._draw_hub(p, cx, cy)
        if self._ec:
            self._draw_ec(p, w)
        ang = self.antenna_angle(self._cp) % 360.0
        self._draw_hud_bottom(p, w, h, ang, self._cp)
        p.end()

    def _draw_strips(self, p, w, h) -> None:
        p.setPen(Qt.NoPen)
        p.setBrush(self._b_strip)
        p.drawRect(QRectF(0, 0, w, _HUD_TOP))
        p.drawRect(QRectF(0, h - _HUD_BOTTOM, w, _HUD_BOTTOM))
        p.setPen(self._pen_border)
        p.drawLine(QPointF(0, _HUD_TOP), QPointF(w, _HUD_TOP))
        p.drawLine(QPointF(0, h - _HUD_BOTTOM), QPointF(w, h - _HUD_BOTTOM))

    def _draw_disk(self, p, cx, cy, r) -> None:
        p.setPen(Qt.NoPen)
        p.setBrush(self._b_disk)
        p.drawEllipse(QPointF(cx, cy), r, r)

    def _draw_bezel(self, p, cx, cy, r) -> None:
        p.setBrush(Qt.NoBrush)
        p.setPen(self._pen_bezel)
        p.drawEllipse(QPointF(cx, cy), r, r)

    def _draw_grid(self, p, cx, cy, r) -> None:
        p.setBrush(Qt.NoBrush)
        p.setPen(self._pen_grid)
        for k in (0.25, 0.5, 0.75, 1.0):
            rr = r * k
            p.drawEllipse(QPointF(cx, cy), rr, rr)
        p.drawLine(QPointF(cx - r, cy), QPointF(cx + r, cy))
        p.drawLine(QPointF(cx, cy - r), QPointF(cx, cy + r))

    def _draw_ticks(self, p, cx, cy, r) -> None:
        for i, (ux, uy) in enumerate(_TICK_VECS):
            major = (i % 6 == 0)
            r0 = r - (11.0 if major else 5.0)
            r1 = r - 1.0
            p.setPen(self._pen_tick_major if major else self._pen_tick_minor)
            p.drawLine(QPointF(cx + ux * r0, cy + uy * r0),
                       QPointF(cx + ux * r1, cy + uy * r1))
        for j, (ux, uy) in enumerate(_LABEL_VECS):
            cardinal = (j % 3 == 0)
            p.setFont(self._f_tick_card if cardinal else self._f_tick)
            p.setPen(self._pen_label_card if cardinal else self._pen_label)
            lx = cx + ux * (r + 13.0)
            ly = cy + uy * (r + 13.0)
            p.drawText(QRectF(lx - 18, ly - 8, 36, 16),
                       Qt.AlignCenter, _LABEL_TEXTS[j])

    def _draw_scan(self, p, cx, cy, r) -> None:
        a1 = self.antenna_angle(self._scan[0])
        a2 = self.antenna_angle(self._scan[1])
        lo, hi = (a1, a2) if a1 <= a2 else (a2, a1)
        rect = QRectF(cx - r, cy - r, 2 * r, 2 * r)
        start16 = int(round((90 - hi) * 16))
        span16 = int(round((hi - lo) * 16))
        p.setPen(Qt.NoPen)
        p.setBrush(self._b_scan)
        p.drawPie(rect, start16, span16)
        p.setBrush(Qt.NoBrush)
        p.setPen(self._pen_scan_edge)
        p.drawLine(QPointF(cx, cy), self._point(cx, cy, r, lo))
        p.drawLine(QPointF(cx, cy), self._point(cx, cy, r, hi))
        p.setPen(self._pen_scan_arc)
        p.drawArc(rect, start16, span16)

    def _draw_trail(self, p, cx, cy, r) -> None:
        n = len(self._trail)
        if n < 2:
            return
        pen = self._pen_trail
        col = self._c_trail
        center = QPointF(cx, cy)
        for i, ang in enumerate(self._trail):
            k = (i + 1) / n
            col.setAlpha(int(110 * k * k))
            pen.setColor(col)
            p.setPen(pen)
            p.drawLine(center, self._point(cx, cy, r - 1, ang))

    def _draw_target(self, p, cx, cy, r) -> None:
        ang = self.antenna_angle(self._tp)
        tip = self._point(cx, cy, r - 2.0, ang)
        p.setBrush(Qt.NoBrush)
        p.setPen(self._pen_target)
        p.drawLine(QPointF(cx, cy), tip)
        a = math.radians(ang)
        ux, uy = math.sin(a), -math.cos(a)
        px_, py_ = -uy, ux
        b0 = QPointF(tip.x() + ux * 9 + px_ * 5, tip.y() + uy * 9 + py_ * 5)
        b1 = QPointF(tip.x() + ux * 9 - px_ * 5, tip.y() + uy * 9 - py_ * 5)
        p.setPen(Qt.NoPen)
        p.setBrush(self._b_target)
        p.drawPolygon(QPolygonF([tip, b0, b1]))

    def _draw_beam(self, p, cx, cy, r) -> None:
        ang = self.antenna_angle(self._cp)
        rect = QRectF(cx - r, cy - r, 2 * r, 2 * r)
        start16 = int(round((90 - (ang + _BEAM_HALF_DEG)) * 16))
        span16 = int(round((2 * _BEAM_HALF_DEG) * 16))
        p.setPen(Qt.NoPen)
        p.setBrush(self._b_cone)
        p.drawPie(rect, start16, span16)
        center = QPointF(cx, cy)
        tip = self._point(cx, cy, r, ang)
        p.setPen(self._pen_beam_glow)
        p.drawLine(center, tip)
        p.setPen(self._pen_beam)
        p.drawLine(center, tip)
        p.setPen(Qt.NoPen)
        p.setBrush(self._b_beam_dot)
        p.drawEllipse(tip, 3.5, 3.5)
        p.setBrush(self._b_beam_core)
        p.drawEllipse(tip, 1.6, 1.6)

    def _draw_hub(self, p, cx, cy) -> None:
        c = QPointF(cx, cy)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(self._c_border2))
        p.drawEllipse(c, 7, 7)
        p.setBrush(QBrush(QColor(self._c_accent.red(), self._c_accent.green(),
                                 self._c_accent.blue(), 160)))
        p.drawEllipse(c, 5, 5)
        p.setBrush(QBrush(self._c_bg))
        p.drawEllipse(c, 2.5, 2.5)

    def _draw_ec(self, p, w) -> None:
        from .. import protocol as P
        p.setFont(ui_font(10, bold=True))
        p.setPen(self._pen_danger)
        txt = f"{METRICS['ec'].short}: {P.EC_LEGEND.get(self._ec, self._ec)}"
        p.drawText(QRectF(12, 0, w - 24, _HUD_TOP), Qt.AlignRight | Qt.AlignVCenter, txt)

    def _draw_hud_bottom(self, p, w, h, ang: float | None, cp: float | None) -> None:
        rect = QRectF(12, h - _HUD_BOTTOM, w - 24, _HUD_BOTTOM)
        if ang is None or cp is None:
            p.setFont(self._f_mono_big)
            p.setPen(self._pen_faint)
            p.drawText(rect, Qt.AlignCenter, "— °")
            return
        rev = int(abs(cp) // 360)
        small = f"{cp:.2f}° · {rev} об."
        p.setFont(self._f_mono_small)
        p.setPen(self._pen_label)
        p.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, small)
        p.setFont(self._f_mono_big)
        p.setPen(self._pen_accent)
        p.drawText(rect, Qt.AlignCenter, f"{ang:6.2f}°")
