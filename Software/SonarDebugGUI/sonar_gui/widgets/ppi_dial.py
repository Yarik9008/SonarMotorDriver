"""PPIDialWidget — сонарный PPI-экран (вид сверху) положения антенны.

Одна ось вращения. Угол антенны = sign*(cp*scale) + offset (настройки диаграммы).
Рисует: диск с радиальным градиентом и безелем, кольца дальности, риски 5°/30°
с подписями (кардинальные — акцентом), сектор скана с дугой по ободу,
послесвечение луча, луч-конус со свечением и точкой на ободе, ромбик цели, хаб
и две HUD-полосы: сверху режим/ошибка энкодера, снизу CP / крупный угол / TP
(цифровой блок вынесен ИЗ круга, чтобы не наезжать на метку «180»).
Без подключения — приглушённый диск и надпись «НЕТ ДАННЫХ».
"""
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

from ..protocol import EC_LEGEND
from ..theme import COLORS

_TRAIL_LEN = 34
_FPS_MS = 33

_HUD_TOP = 26.0      # высота верхней HUD-полосы
_HUD_BOTTOM = 40.0   # высота нижней полосы с цифрами (ВНЕ круга)
_MARGIN = 22.0       # запас вокруг круга под подписи градусов (radius+16 ≤ края)

_BEAM_HALF_DEG = 3.5  # полуширина конуса луча

# Единичные направления рисок каждые 5° (экран: 0° вверх, рост по часовой)
_TICK_VECS = tuple(
    (math.sin(math.radians(d)), -math.cos(math.radians(d)))
    for d in range(0, 360, 5)
)
# Направления и тексты подписей каждые 30°
_LABEL_VECS = tuple(
    (math.sin(math.radians(d)), -math.cos(math.radians(d)))
    for d in range(0, 360, 30)
)
_LABEL_TEXTS = tuple(str(d) for d in range(0, 360, 30))


def _col(key: str, fallback: str) -> QColor:
    """Цвет из темы с запасным значением (пока тема может не иметь ppi_*-ключей)."""
    return QColor(COLORS.get(key, fallback))


def _mono_font(pixel_size: int, weight: QFont.Weight = QFont.Normal) -> QFont:
    f = QFont()
    f.setFamilies(["Cascadia Mono", "Consolas", "monospace"])
    f.setPixelSize(pixel_size)
    f.setWeight(weight)
    return f


def _ui_font(pixel_size: int, weight: QFont.Weight = QFont.Normal) -> QFont:
    f = QFont("Segoe UI")
    f.setPixelSize(pixel_size)
    f.setWeight(weight)
    return f


class PPIDialWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)

        self._cp = 0.0
        self._tp = 0.0
        self._mode = "cl"
        self._ec = 0
        self._scan = None            # (start, end) в единицах cp или None
        self._connected = False

        self._scale = 1.0
        self._offset = 0.0
        self._sign = 1

        self._trail: deque[float] = deque(maxlen=_TRAIL_LEN)

        # ── Кэш перьев/кистей/шрифтов (никаких аллокаций в paintEvent) ──────
        self._c_bg = _col("bg", "#0a0d13")
        self._c_well = _col("well", "#0b0f16")
        self._c_border = _col("border", "#1f2735")
        self._c_border2 = _col("border2", "#33405a")
        self._c_accent = _col("accent", "#37d6ff")
        self._c_accent2 = _col("accent2", "#35e0c8")
        self._c_cp = _col("cp", "#37d6ff")
        self._c_amber = _col("amber", "#ffb454")
        self._c_green = _col("green", "#3ddc84")
        self._c_danger = _col("danger", "#ff5d5d")
        self._c_text_dim = _col("text_dim", "#93a0b4")
        self._c_text_faint = _col("text_faint", "#5c6880")
        self._c_ppi_grid = _col("ppi_grid", "#164252")
        self._c_ppi_label = _col("ppi_label", "#5fb8d4")
        self._c_disk_center = _col("ppi_center", "#0c2530")
        self._c_disk_mid = QColor("#091620")
        self._c_disk_edge = _col("ppi_edge", "#070b10")

        bezel_glow = QColor(self._c_accent)
        bezel_glow.setAlpha(60)
        scan_edge = QColor(self._c_accent2)
        scan_edge.setAlpha(120)
        scan_arc = QColor(self._c_accent2)
        scan_arc.setAlpha(220)
        scan_fill = QColor(self._c_accent2)
        scan_fill.setAlpha(32)
        scan_label = QColor(self._c_accent2)
        scan_label.setAlpha(180)
        beam_glow = QColor(self._c_cp)
        beam_glow.setAlpha(40)
        target_ray = QColor(self._c_amber)
        target_ray.setAlpha(90)
        dot_glow1 = QColor(self._c_cp)
        dot_glow1.setAlpha(50)
        dot_glow2 = QColor(self._c_cp)
        dot_glow2.setAlpha(110)
        hub_core = QColor(self._c_accent)
        hub_core.setAlpha(160)

        self._pen_grid = QPen(self._c_ppi_grid, 1)
        self._pen_tick_minor = QPen(self._c_ppi_grid, 1)
        self._pen_tick_major = QPen(self._c_text_dim, 1.6)
        self._pen_bezel = QPen(self._c_border2, 2)
        self._pen_bezel_glow = QPen(bezel_glow, 1)
        self._pen_scan_edge = QPen(scan_edge, 1, Qt.DashLine)
        self._pen_scan_arc = QPen(scan_arc, 2)
        self._pen_scan_label = QPen(scan_label)
        self._pen_target = QPen(target_ray, 1.4, Qt.DashLine)
        self._pen_beam = QPen(self._c_cp, 2.5, Qt.SolidLine, Qt.RoundCap)
        self._pen_beam_glow = QPen(beam_glow, 7, Qt.SolidLine, Qt.RoundCap)
        self._pen_trail = QPen(self._c_cp, 2)
        self._pen_border = QPen(self._c_border, 1)
        self._pen_label = QPen(self._c_ppi_label)
        self._pen_label_card = QPen(self._c_accent)
        self._pen_accent = QPen(self._c_accent)
        self._pen_amber = QPen(self._c_amber)
        self._pen_green = QPen(self._c_green)
        self._pen_danger = QPen(self._c_danger)
        self._pen_faint = QPen(self._c_text_faint)

        self._b_scan = QBrush(scan_fill)
        self._b_target = QBrush(self._c_amber)
        self._b_beam_dot = QBrush(self._c_accent)
        self._b_beam_core = QBrush(QColor("#ffffff"))
        self._b_dot_glow1 = QBrush(dot_glow1)
        self._b_dot_glow2 = QBrush(dot_glow2)
        self._b_hub_outer = QBrush(self._c_border2)
        self._b_hub_mid = QBrush(hub_core)
        self._b_hub_core = QBrush(self._c_bg)
        self._b_strip = QBrush(self._c_well)

        self._c_trail = QColor(self._c_cp)  # мутируем только alpha в цикле следа

        self._f_tick = _ui_font(9)
        self._f_tick_card = _ui_font(9, QFont.Bold)
        self._f_hud = _ui_font(10, QFont.Bold)
        self._f_nodata = _ui_font(12, QFont.Bold)
        self._f_mono_small = _mono_font(11)
        self._f_mono_big = _mono_font(20, QFont.Bold)

        # Геометрозависимые кисти (радиальные градиенты) — пересобираем по ключу
        self._geom_key = None
        self._b_disk = QBrush(self._c_disk_edge)
        self._b_cone = QBrush(Qt.NoBrush)

        self._timer = QTimer(self)
        self._timer.setInterval(_FPS_MS)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start()

    # ── Публичное API ──────────────────────────────────────────────────────
    def set_state(self, cp: float, tp: float, mode: str, ec: int) -> None:
        self._cp = cp
        self._tp = tp
        self._mode = mode
        self._ec = ec

    def set_scan(self, sector) -> None:
        self._scan = sector

    def set_connected(self, on: bool) -> None:
        self._connected = on
        if not on:
            self._trail.clear()

    def set_config(self, scale: float, offset: float, sign: int) -> None:
        self._scale = scale
        self._offset = offset
        self._sign = 1 if sign >= 0 else -1

    def antenna_angle(self, cp: float) -> float:
        return self._sign * (cp * self._scale) + self._offset

    # ── Анимация послесвечения ─────────────────────────────────────────────
    def _on_tick(self) -> None:
        if self._connected:
            self._trail.append(self.antenna_angle(self._cp))
        self.update()

    # ── Геометрия ──────────────────────────────────────────────────────────
    def _point(self, cx, cy, r, angle_deg) -> QPointF:
        a = math.radians(angle_deg)
        return QPointF(cx + r * math.sin(a), cy - r * math.cos(a))

    def _ensure_geom(self, cx: float, cy: float, r: float) -> None:
        """Пересобирает радиальные градиенты только при изменении геометрии."""
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

    # ── Отрисовка ──────────────────────────────────────────────────────────
    def paintEvent(self, _ev) -> None:
        w, h = float(self.width()), float(self.height())
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), self._c_bg)

        self._draw_strips(p, w, h)

        # Круг — в остатке между HUD-полосами, подписи обязаны помещаться
        top, bottom = _HUD_TOP, h - _HUD_BOTTOM
        avail = bottom - top
        cx = w / 2.0
        cy = top + avail / 2.0
        r = min(w / 2.0, avail / 2.0) - _MARGIN
        if r < 40:
            p.end()
            return

        self._ensure_geom(cx, cy, r)

        if not self._connected:
            # Приглушённые диск и сетка + «НЕТ ДАННЫХ»; HUD-полосы пустые
            p.setOpacity(0.35)
            self._draw_disk(p, cx, cy, r)
            self._draw_grid(p, cx, cy, r)
            self._draw_ticks(p, cx, cy, r)
            self._draw_bezel(p, cx, cy, r)
            p.setOpacity(1.0)
            p.setFont(self._f_nodata)
            p.setPen(self._pen_faint)
            p.drawText(QRectF(cx - 120, cy - 12, 240, 24),
                       Qt.AlignCenter, "НЕТ ДАННЫХ")
            p.end()
            return

        self._draw_disk(p, cx, cy, r)
        self._draw_grid(p, cx, cy, r)
        if self._scan is not None:
            self._draw_scan(p, cx, cy, r)
        self._draw_trail(p, cx, cy, r)
        self._draw_target(p, cx, cy, r)
        self._draw_beam(p, cx, cy, r)
        self._draw_ticks(p, cx, cy, r)
        self._draw_bezel(p, cx, cy, r)
        self._draw_hub(p, cx, cy)
        self._draw_hud_top(p, w)
        # Широкая полоса: CP/TP уходят на пустые фланги по бокам круга,
        # внизу остаётся ОДНО крупное текущее значение (нет дублей)
        flanks = (cx - r - _MARGIN) >= 120.0
        if flanks:
            self._draw_flanks(p, cx, cy, r)
        self._draw_hud_bottom(p, w, h, with_sides=not flanks)
        p.end()

    def _draw_strips(self, p, w, h) -> None:
        """Фоновые HUD-полосы сверху и снизу с тонкими разделителями."""
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
        p.setPen(self._pen_bezel_glow)
        p.drawEllipse(QPointF(cx, cy), r + 3, r + 3)

    def _draw_grid(self, p, cx, cy, r) -> None:
        """Кольца дальности 0.25/0.5/0.75/1.0 и перекрестие."""
        p.setBrush(Qt.NoBrush)
        p.setPen(self._pen_grid)
        for k in (0.25, 0.5, 0.75, 1.0):
            rr = r * k
            p.drawEllipse(QPointF(cx, cy), rr, rr)
        p.drawLine(QPointF(cx - r, cy), QPointF(cx + r, cy))
        p.drawLine(QPointF(cx, cy - r), QPointF(cx, cy + r))

    def _draw_ticks(self, p, cx, cy, r) -> None:
        # Риски: каждые 5° короткие, каждые 30° длинные
        for i, (ux, uy) in enumerate(_TICK_VECS):
            major = (i % 6 == 0)  # i*5 % 30 == 0
            r0 = r - (11.0 if major else 5.0)
            r1 = r - 1.0
            p.setPen(self._pen_tick_major if major else self._pen_tick_minor)
            p.drawLine(QPointF(cx + ux * r0, cy + uy * r0),
                       QPointF(cx + ux * r1, cy + uy * r1))
        # Подписи каждые 30°; кардинальные (0/90/180/270) — акцентом и жирнее
        for j, (ux, uy) in enumerate(_LABEL_VECS):
            cardinal = (j % 3 == 0)  # j*30 % 90 == 0
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
        # Заливка сектора
        p.setPen(Qt.NoPen)
        p.setBrush(self._b_scan)
        p.drawPie(rect, start16, span16)
        # Границы-радиусы штрихом
        p.setBrush(Qt.NoBrush)
        p.setPen(self._pen_scan_edge)
        p.drawLine(QPointF(cx, cy), self._point(cx, cy, r, lo))
        p.drawLine(QPointF(cx, cy), self._point(cx, cy, r, hi))
        # Дуга по ободу
        p.setPen(self._pen_scan_arc)
        p.drawArc(rect, start16, span16)
        # Подписи граничных углов сектора — внутри обода, приглушённым тилом
        p.setFont(self._f_tick)
        p.setPen(self._pen_scan_label)
        for ang in (lo, hi):
            pt = self._point(cx, cy, r - 22.0, ang)
            p.drawText(QRectF(pt.x() - 20, pt.y() - 8, 40, 16),
                       Qt.AlignCenter, f"{ang % 360.0:.0f}°")

    def _draw_trail(self, p, cx, cy, r) -> None:
        """Послесвечение: alpha растёт квадратично 0→110 к свежим отсчётам."""
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
        """Цель tp: тонкий янтарный радиус + янтарный треугольник-указатель на ободе.

        Треугольник смотрит остриём внутрь с внешнего кольца — читается мгновенно
        и не тонет в циановом веере следа (цвет связан с кривой tp на графике).
        """
        ang = self.antenna_angle(self._tp)
        tip = self._point(cx, cy, r - 2.0, ang)
        p.setBrush(Qt.NoBrush)
        p.setPen(self._pen_target)
        p.drawLine(QPointF(cx, cy), tip)
        # Треугольник: остриё на ободе, основание снаружи
        a = math.radians(ang)
        ux, uy = math.sin(a), -math.cos(a)       # радиальное направление
        px_, py_ = -uy, ux                        # перпендикуляр
        base = 9.0
        half = 5.0
        b0 = QPointF(tip.x() + ux * base + px_ * half,
                     tip.y() + uy * base + py_ * half)
        b1 = QPointF(tip.x() + ux * base - px_ * half,
                     tip.y() + uy * base - py_ * half)
        p.setPen(Qt.NoPen)
        p.setBrush(self._b_target)
        p.drawPolygon(QPolygonF([tip, b0, b1]))

    def _draw_beam(self, p, cx, cy, r) -> None:
        """Луч cp: конус ±3.5°, свечение, осевая линия и точка на ободе."""
        ang = self.antenna_angle(self._cp)
        rect = QRectF(cx - r, cy - r, 2 * r, 2 * r)
        start16 = int(round((90 - (ang + _BEAM_HALF_DEG)) * 16))
        span16 = int(round((2 * _BEAM_HALF_DEG) * 16))
        p.setPen(Qt.NoPen)
        p.setBrush(self._b_cone)
        p.drawPie(rect, start16, span16)

        center = QPointF(cx, cy)
        tip = self._point(cx, cy, r, ang)
        p.setPen(self._pen_beam_glow)   # «свечение» — широкая полупрозрачная
        p.drawLine(center, tip)
        p.setPen(self._pen_beam)        # осевая линия
        p.drawLine(center, tip)
        # Точка на ободе: мягкий glow (два круга с убывающей alpha) + ядро
        p.setPen(Qt.NoPen)
        p.setBrush(self._b_dot_glow1)
        p.drawEllipse(tip, 8.0, 8.0)
        p.setBrush(self._b_dot_glow2)
        p.drawEllipse(tip, 5.5, 5.5)
        p.setBrush(self._b_beam_dot)
        p.drawEllipse(tip, 3.5, 3.5)
        p.setBrush(self._b_beam_core)
        p.drawEllipse(tip, 1.6, 1.6)

    def _draw_hub(self, p, cx, cy) -> None:
        c = QPointF(cx, cy)
        p.setPen(Qt.NoPen)
        p.setBrush(self._b_hub_outer)
        p.drawEllipse(c, 7, 7)
        p.setBrush(self._b_hub_mid)
        p.drawEllipse(c, 5, 5)
        p.setBrush(self._b_hub_core)
        p.drawEllipse(c, 2.5, 2.5)

    def _draw_hud_top(self, p, w) -> None:
        """Верхняя полоса: режим слева, ошибка энкодера справа."""
        rect = QRectF(12, 0, w - 24, _HUD_TOP)
        closed = (self._mode == "cl")
        p.setFont(self._f_hud)
        p.setPen(self._pen_green if closed else self._pen_amber)
        mtxt = "● CL — ЗАМКНУТЫЙ" if closed else "● OL — ОТКРЫТЫЙ"
        p.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, mtxt)
        if self._ec != 0:
            p.setPen(self._pen_danger)
            p.drawText(rect, Qt.AlignRight | Qt.AlignVCenter,
                       f"ENC: {EC_LEGEND.get(self._ec, self._ec)}")

    def _draw_flanks(self, p, cx, cy, r) -> None:
        """CP/TP крупно на флангах по бокам круга (когда полоса PPI широкая)."""
        fw = cx - r - _MARGIN - 12.0
        rx = cx + r + _MARGIN + 4.0
        p.setFont(self._f_hud)
        p.setPen(self._pen_faint)
        p.drawText(QRectF(8, cy - 36, fw, 18), Qt.AlignCenter, "CP — ТЕКУЩИЙ")
        p.drawText(QRectF(rx, cy - 36, fw, 18), Qt.AlignCenter, "TP — ЦЕЛЬ")
        p.setFont(self._f_mono_big)
        p.setPen(self._pen_accent)
        p.drawText(QRectF(8, cy - 16, fw, 30), Qt.AlignCenter,
                   f"{self._cp:.2f}°")
        p.setPen(self._pen_amber)
        p.drawText(QRectF(rx, cy - 16, fw, 30), Qt.AlignCenter,
                   f"{self._tp:.2f}°")

    def _draw_hud_bottom(self, p, w, h, with_sides: bool) -> None:
        """Нижняя полоса ВНЕ круга: крупный угол антенны по центру.

        CP/TP добавляются по краям только если для фланговых блоков нет места.
        """
        rect = QRectF(12, h - _HUD_BOTTOM, w - 24, _HUD_BOTTOM)
        if with_sides:
            p.setFont(self._f_mono_small)
            p.setPen(self._pen_label)
            p.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, f"CP {self._cp:7.2f}°")
            p.setPen(self._pen_amber)
            p.drawText(rect, Qt.AlignRight | Qt.AlignVCenter, f"TP {self._tp:7.2f}°")
        ang = self.antenna_angle(self._cp) % 360.0
        p.setFont(self._f_mono_big)
        p.setPen(self._pen_accent)
        p.drawText(rect, Qt.AlignCenter, f"{ang:6.2f}°")
