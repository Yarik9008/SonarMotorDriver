"""Единый словарь метрик протокола — все подписи UI берутся только отсюда."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Metric:
    key: str
    label: str
    short: str
    unit: str
    tooltip: str = ""


METRICS: dict[str, Metric] = {
    "cp": Metric(
        "cp", "Позиция", "Поз.", "°",
        "Текущая позиция вала (накопленный угол, cp)"),
    "tp": Metric(
        "tp", "Цель", "Цель", "°",
        "Целевой угол регулятора (tp)"),
    "pe": Metric(
        "pe", "Ошибка", "Ошибка", "°",
        "Ошибка положения (pe), только при debug=1"),
    "u": Metric(
        "u", "Управление", "Упр.", "",
        "Выход регулятора (u), не в градусах — только при debug=1"),
    "ec": Metric(
        "ec", "Ошибки энкодера", "Энк.", "",
        "Код ошибки BiSS-C энкодера (ec)"),
    "drp": Metric(
        "drp", "Потери кадров", "Потери", "",
        "Счётчик потерянных кадров телеметрии (drp)"),
    "op": Metric(
        "op", "Период телеметрии", "Период", "мс",
        "Интервал посылки телеметрии (op), мс; 0 — выкл"),
    "kp": Metric("kp", "Kp", "Kp", "", "Пропорциональный коэффициент PID"),
    "ki": Metric("ki", "Ki", "Ki", "", "Интегральный коэффициент PID"),
    "kd": Metric("kd", "Kd", "Kd", "", "Дифференциальный коэффициент PID"),
    "v": Metric("v", "Скорость", "Скор.", "°/с", "Предел скорости движения (v=)"),
    "a": Metric("a", "Ускорение", "Ускор.", "°/с²", "Предел ускорения (a=)"),
}


def label(key: str, *, with_unit: bool = True) -> str:
    m = METRICS[key]
    if with_unit and m.unit:
        return f"{m.label}, {m.unit}"
    return m.label


def short_label(key: str) -> str:
    return METRICS[key].short


def legend_label(key: str) -> str:
    return METRICS[key].label


def mode_label(mode: str | None) -> str:
    if not mode:
        return "—"
    m = mode.lower()
    if m == "cl":
        return "● CL — ЗАМКНУТЫЙ"
    if m == "ol":
        return "● OL — ОТКРЫТЫЙ"
    return mode.upper()
