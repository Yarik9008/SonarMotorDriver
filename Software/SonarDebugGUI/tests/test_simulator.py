"""Юнит-тесты sonar_gui/simulator.py — модели прошивки FW_SonarMotorDriver.

Модель нужна затем, чтобы протокол и GUI проверялись без платы, поэтому
проверяется её поведение, а не внутренние поля: кольцевая координата и
кратчайший путь, профиль скорости и ускорения (v=/a=), сценарии скана и
переходы по sync=, тихая пауза hold=, событийная телеметрия om= и «молчание»
парсера на аргументы, которые прошивка не принимает.

Время — модельное: тик 1 мс, как период главного цикла платы (POLL_FREQ_HZ).
"""
from __future__ import annotations

import math
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import _support     # noqa: F401,E402 - ставит sonar_gui на путь импорта
from _support import DEADBAND, P, S, Sim, TICK_MS      # noqa: E402


class RingCoordinateTests(unittest.TestCase):
    """Один оборот, ход к цели кратчайшим путём — как wrap360/shortest_path_err."""

    def test_target_echo_is_wrapped(self):
        """Прошивка приводит цель к [0,360) и эхом отдаёт уже приведённый угол."""
        sim = Sim()
        self.assertEqual(sim.send("t=-30"), ["ok:t=330.00"])
        self.assertAlmostEqual(sim.fw.target_deg, 330.0, places=6)
        self.assertEqual(sim.send("t=370.5"), ["ok:t=10.50"])
        self.assertAlmostEqual(sim.fw.target_deg, 10.5, places=6)

    def test_goes_forward_through_zero(self):
        """От 350 к 10 вал идёт вперёд через ноль на 20°, а не назад на 340°."""
        sim = Sim("op=0", "a=0", "v=120")
        sim.fw.cur_deg = 350.0
        sim.ok("t=10")
        deltas = sim.path(400)
        self.assertGreater(sum(deltas), 19.0)     # суммарный путь ~ +20°
        self.assertLess(sum(deltas), 20.1)
        self.assertTrue(all(d >= 0.0 for d in deltas), "были ходы в обратную сторону")
        self.assertLess(abs(P.wrap180(sim.fw.cur_deg - 10.0)), DEADBAND)

    def test_goes_backward_through_zero(self):
        """Симметрично: от 10 к 350 — назад через ноль."""
        sim = Sim("op=0", "a=0", "v=120")
        sim.fw.cur_deg = 10.0
        sim.ok("t=350")
        deltas = sim.path(400)
        self.assertLess(sum(deltas), -19.0)
        self.assertGreater(sum(deltas), -20.1)
        self.assertTrue(all(d <= 0.0 for d in deltas))

    def test_position_stays_in_ring(self):
        """Позиция и цель всегда в [0,360): накопителя оборотов в прошивке нет."""
        sim = Sim("op=0", "a=0", "v=1200")
        sim.ok("t=+")
        for _ in range(20):
            sim.run(100)
            self.assertGreaterEqual(sim.fw.cur_deg, 0.0)
            self.assertLess(sim.fw.cur_deg, 360.0)
            self.assertGreaterEqual(sim.fw.target_deg, 0.0)
            self.assertLess(sim.fw.target_deg, 360.0)

    def test_half_turn_error_sign(self):
        """Ровно 180°: ошибка равна +180 (диапазон (-180,180]) — идём вперёд."""
        sim = Sim("op=0", "a=0", "v=360")
        sim.ok("t=180")
        deltas = sim.path(100)
        self.assertTrue(all(d > 0.0 for d in deltas))

    def test_error_field_is_shortest_path(self):
        """Поле pe телеметрии — ошибка по кольцу, а не арифметическая разность."""
        sim = Sim("op=0", "a=0", "v=1", "debug=1")
        sim.fw.cur_deg = 350.0
        sim.ok("t=10")
        data = P.parse_telemetry(sim.fw.telemetry_line())
        self.assertAlmostEqual(data["cp"], 350.0, places=2)
        self.assertAlmostEqual(data["pe"], 20.0, places=2)   # не -340

    def test_tracking_through_zero_keeps_cp_in_the_ring(self):
        """Ход к цели через ноль не выводит позицию за [0,360).

        Ошибка тут ничего не покажет: она и так считается по кратчайшему
        пути. Признак — поле cp, которое уходит хосту: вал, проехавший
        350°→10° без приведения к кольцу, отдал бы cp:370.00, и диаграмма
        GUI показала бы его за пределами круга.
        """
        sim = Sim("op=1", "om=0", "debug=0", "a=0", "v=600")
        sim.fw.cur_deg = 350.0
        sim.ok("t=10")
        sim.settle(limit_ms=3000)
        sim.run(5)                      # дать отработать доводочному шагу
        self.assertAlmostEqual(sim.fw.cur_deg, 10.0, places=6)
        cps = [P.parse_telemetry(f)["cp"] for f in sim.frames]
        self.assertGreater(len(cps), 20)
        self.assertTrue(all(0.0 <= cp < 360.0 for cp in cps),
                        f"cp вне кольца: {max(cps)}")

    def test_continuous_rotation_carries_the_target(self):
        """В непрерывном вращении цель едет вместе с валом (main.c:965).

        От этого зависит поле pe: хост, включивший t=+, видит ошибку
        слежения около нуля, а не растущую разность «брошенная цель минус
        позиция». Отстающая цель, кроме того, тормозила бы вал тормозным
        конвертом профиля, когда «ошибка» пройдёт через ноль.
        """
        sim = Sim("op=20", "om=0", "debug=1", "a=0", "v=360")
        sim.ok("t=+")
        sim.run(500)
        pes = [P.parse_telemetry(f)["pe"] for f in sim.frames]
        self.assertGreaterEqual(len(pes), 20)
        self.assertTrue(all(abs(p) <= 0.5 for p in pes),
                        f"цель отстала от вала: pe={max(pes, key=abs)}")
        # Вал за это время действительно уехал (иначе проверка ничего не стоит)
        self.assertGreater(abs(P.wrap180(sim.fw.cur_deg - 0.0)), 90.0)
        self.assertAlmostEqual(sim.fw.target_deg, sim.fw.cur_deg, places=6)


class MotionProfileTests(unittest.TestCase):
    """Профиль движения: предел скорости v= и предел ускорения a=."""

    def test_speed_limit_sets_travel_time(self):
        """90° на 60 °/с: путь на пределе v= плюс экспоненциальный дожим ПИД.

        Предел скорости держится ровно пока его требует регулятор. Прошивка
        берёт скорость с выхода ПИД (main.c:976-978: «float pid =
        PID_Update(&g_pid, err, DT_S); float v = Motion_Limit(pid, err, 1);»),
        то есть kp·err градусов за такт = kp/DT_S·err °/с. При kp=0.025 это
        25·err, и предел 60 °/с перестаёт быть узким местом уже на err=2.4°.
        Дальше ошибка спадает экспонентой с постоянной DT_S/kp = 40 мс, и до
        мёртвой зоны 0.05° остаётся ещё ln(2.4/0.05)·40 ≈ 155 мс.
        """
        vmax = 60.0
        sim = Sim("op=0", "a=0", f"v={vmax:g}")
        sim.ok("t=90")
        spent = sim.settle(limit_ms=5000)
        rate = P.DEFAULTS.kp / S.CONTROL_DT_S       # °/с на градус ошибки
        e_lin = vmax / rate                         # ошибка схода с предела v=
        expect = ((90.0 - e_lin) / vmax
                  + math.log(e_lin / DEADBAND) / rate) * 1000.0
        self.assertAlmostEqual(spent, expect, delta=20.0)
        self.assertLess(abs(P.wrap180(sim.fw.cur_deg - 90.0)), DEADBAND)

    def test_accel_zero_means_instant_speed(self):
        """a=0 — ограничения ускорения нет: первый же тик идёт на полной v=."""
        sim = Sim("op=0", "a=0", "v=600")
        sim.ok("t=180")
        deltas = sim.path(3)
        for d in deltas:
            self.assertAlmostEqual(d, 600.0 * TICK_MS / 1000.0, places=9)

    def test_accel_limits_velocity_change(self):
        """a=200: скорость меняется не быстрее 200 °/с за секунду."""
        accel = 200.0
        sim = Sim("op=0", f"a={accel:g}", "v=1200")
        sim.ok("t=300")
        limit = accel * TICK_MS / 1000.0
        prev, jumps = 0.0, []
        # Приход в цель считаем отдельно: там прошивка сбрасывает рампу
        # (Control_Reset), и скорость падает в ноль сразу — это не разгон.
        for _ in range(4000):
            moving = abs(sim.err()) > DEADBAND
            sim.run(TICK_MS)
            if moving:
                jumps.append(abs(sim.fw.vel - prev))
            prev = sim.fw.vel
        self.assertLessEqual(max(jumps), limit + 1e-9)
        self.assertAlmostEqual(max(jumps), limit, delta=1e-9)   # разгон идёт на пределе

    def test_braking_envelope(self):
        """Скорость держится тормозного конверта sqrt(2*a*|err|).

        Именно он не даёт проскочить цель: подъехав вплотную, вал уже ползёт.
        Проверяем на ходу, пока до цели больше градуса: последний градус — уже
        дело мёртвой зоны, там модель просто встаёт, и конверт (стремящийся к
        нулю) смысла не имеет. Допуск 5% — дискретная рампа догоняет конверт
        не мгновенно, а за несколько тиков.
        """
        accel = 500.0
        sim = Sim("op=0", f"a={accel:g}", "v=1200")
        sim.ok("t=120")
        step = accel * TICK_MS / 1000.0     # один шаг рампы за тик
        worst, checked = 0.0, 0
        while abs(sim.err()) > DEADBAND and sim.t_ms < 5000.0:
            err = abs(sim.err())            # ошибка ДО шага: по ней и решает модель
            sim.run(TICK_MS)
            if err < 1.0:
                continue
            checked += 1
            worst = max(worst, abs(sim.fw.vel) / (math.sqrt(2.0 * accel * err) + step))
        self.assertGreater(checked, 100, "движение не наблюдалось")
        self.assertLessEqual(worst, 1.05, "скорость выше тормозного конверта")

    def test_approach_is_decelerated(self):
        """Подход к цели — торможение: у цели скорость много ниже пиковой."""
        sim = Sim("op=0", "a=500", "v=1200")
        sim.ok("t=120")
        peak, near = 0.0, None
        while abs(sim.err()) > DEADBAND and sim.t_ms < 5000.0:
            sim.run(TICK_MS)
            peak = max(peak, abs(sim.fw.vel))
            if abs(sim.err()) < 1.0 and near is None:
                near = abs(sim.fw.vel)
        self.assertIsNotNone(near)
        self.assertGreater(peak, 100.0)
        self.assertLess(near, 0.25 * peak)

    def test_no_overshoot(self):
        """Цель не проскакивается: знак ошибки за весь ход не меняется."""
        sim = Sim("op=0", "a=500", "v=1200")
        sim.ok("t=120")
        signs = set()
        while abs(sim.err()) > DEADBAND and sim.t_ms < 5000.0:
            sim.run(TICK_MS)
            signs.add(sim.err() > 0.0)
        self.assertEqual(signs, {True}, "вал перелетал цель")

    def test_triangular_profile_duration(self):
        """Разгон-торможение на 100° при a=100: пик ~ sqrt(a*err), время ~ 2 с."""
        accel, dist = 100.0, 100.0
        sim = Sim("op=0", f"a={accel:g}", "v=1200")
        sim.ok(f"t={dist:g}")
        peak = [0.0]
        spent = sim.run_until(lambda fw: not fw.is_moving(), limit_ms=6000)
        # run_until не даёт перехватить пик, поэтому считаем по второму прогону
        sim2 = Sim("op=0", f"a={accel:g}", "v=1200")
        sim2.ok(f"t={dist:g}")
        sim2.run(spent + 100.0, on_tick=lambda fw: peak.__setitem__(
            0, max(peak[0], abs(fw.vel))))
        self.assertAlmostEqual(spent, 2.0 * math.sqrt(dist / accel) * 1000.0,
                               delta=120.0)
        self.assertAlmostEqual(peak[0], math.sqrt(accel * dist), delta=2.0)

    def test_speed_limit_caps_accel_profile(self):
        """При большом a= скорость упирается в v= и держится на ней.

        Время хода — как в test_speed_limit_sets_travel_time: путь на пределе
        v= плюс дожим ПИД последних v/(kp/DT_S) = 2° (main.c:976-978).
        """
        vmax = 50.0
        sim = Sim("op=0", "a=100000", f"v={vmax:g}")
        sim.ok("t=180")
        peak = [0.0]
        spent = sim.run_until(lambda fw: not fw.is_moving(), limit_ms=8000)
        rate = P.DEFAULTS.kp / S.CONTROL_DT_S
        e_lin = vmax / rate
        expect = ((180.0 - e_lin) / vmax
                  + math.log(e_lin / DEADBAND) / rate) * 1000.0
        self.assertAlmostEqual(spent, expect, delta=40.0)
        sim2 = Sim("op=0", "a=100000", "v=50")
        sim2.ok("t=180")
        sim2.run(spent, on_tick=lambda fw: peak.__setitem__(
            0, max(peak[0], abs(fw.vel))))
        self.assertLessEqual(peak[0], 50.0 + 1e-6)

    def test_speed_out_of_range_keeps_previous_limit(self):
        """v= вне диапазона — явная ошибка, прежний предел не меняется."""
        sim = Sim("v=600")
        self.assertEqual(sim.send("v=5000"), ["err:bad arg (v=1..1200)"])
        self.assertEqual(sim.send("v=0"), ["err:bad arg (v=1..1200)"])
        self.assertAlmostEqual(sim.fw.vmax, 600.0, places=6)

    def test_accel_out_of_range_keeps_previous_limit(self):
        sim = Sim("a=2000")
        self.assertEqual(sim.send("a=-1"), ["err:bad arg (a=0..100000)"])
        self.assertAlmostEqual(sim.fw.accel, 2000.0, places=6)


class PidSaturationTests(unittest.TestCase):
    """Откуда берутся границы насыщения ПИД (PID_State.output_min/max).

    Прошивка задаёт их дважды и по-разному: статическим инициализатором g_pid
    (main.c:416) — от аппаратного потолка MAX_DEG_PER_TICK, и обработчиком v=
    (main.c:1519-1520) — от текущего vmax. Совпадение этих двух чисел на
    старте держится равенством SPEED_DEFAULT_DEG_S == MAX_SPEED_DEG_S в
    board.h, которое ничем не закреплено (его стережёт отдельный тест в
    test_board_constants.py).

    Видно различие через anti-windup: пока выход не упёрся в границу,
    интеграл копится, а в насыщении откатывается обратно и остаётся нулём.
    Ошибка здесь берётся малая (1°), чтобы выход ПИД лёг МЕЖДУ двумя
    кандидатами в границы и тем их различил.
    """

    def test_start_bounds_come_from_the_hardware_ceiling(self):
        """vmax, выставленный не командой v=, границы ПИД не двигает.

        Это и есть случай SPEED_DEFAULT_DEG_S != MAX_SPEED_DEG_S: прошивка
        стартует с границами от MAX_SPEED_DEG_S, о g_vmax_deg_s статический
        инициализатор ничего не знает.
        """
        sim = Sim("op=0", "ki=0.5")
        sim.fw.vmax = 1.0               # как если бы board.h развёл константы
        self.assertAlmostEqual(sim.fw.pid_out_max,
                               P.MAX_SPEED_DEG_S * S.CONTROL_DT_S, places=9)
        sim.ok("t=1")
        sim.run(50)
        self.assertGreater(
            sim.fw.pid_i, 0.0,
            "выход ПИД до границы от MAX_SPEED не достаёт — интеграл обязан копиться")

    def test_v_command_moves_the_bounds(self):
        """v= пересчитывает границы, и анти-виндап начинает держать интеграл."""
        sim = Sim("op=0", "ki=0.5", "v=1")
        self.assertAlmostEqual(sim.fw.pid_out_max, 1.0 * S.CONTROL_DT_S, places=9)
        sim.ok("t=1")
        sim.run(50)
        self.assertEqual(sim.fw.pid_i, 0.0,
                         "в насыщении интеграл откатывается назад (anti-windup)")


class ScanTests(unittest.TestCase):
    """Сценарии scan=: зигзаг, сектор через ноль, бесконечное вращение."""

    # Окно наблюдения считается по времени хода между точками, а его задаёт
    # ПИД: при kp=0.025 скорость равна kp/DT_S·err = 25·err °/с, и предел v=
    # на коротком шаге не работает вовсе. Шаг 5° идёт ln(5/0.05)/25 ≈ 184 мс,
    # шаг 90° — ≈ 300 мс; пауза delay добавляется к этому сверху.
    def points(self, cmd: str, ms: float, *setup: str) -> list[float]:
        sim = Sim("op=0", "a=0", "v=1200", *setup)
        sim.ok(cmd)
        return sim.scan_points(ms)

    def test_zigzag_sequence(self):
        """Сектор 0..20 шагом 5: до края и обратно, края не дублируются."""
        pts = self.points("scan=0,20,5,10", 2000)
        self.assertEqual(pts[:9], [0, 5, 10, 15, 20, 15, 10, 5, 0])
        self.assertTrue(all(0.0 <= p <= 20.0 for p in pts))

    def test_zigzag_through_zero(self):
        """Сектор 350..10 (через ноль) разворачивается на своих краях."""
        pts = self.points("scan=350,10,5,20", 2500)
        self.assertEqual(pts[:9], [350, 355, 0, 5, 10, 5, 0, 355, 350])

    def test_step_larger_than_sector(self):
        """Шаг больше сектора: вал ходит между краями, а не уезжает за них."""
        pts = self.points("scan=0,10,15,10", 1500)
        self.assertEqual(pts[:6], [0, 10, 0, 10, 0, 10])

    def test_infinite_forward_and_backward(self):
        """Бесконечный скан: границ нет, угол заворачивается на нуле."""
        self.assertEqual(self.points("scan=0,+,90,10", 2000)[:5],
                         [0, 90, 180, 270, 0])
        self.assertEqual(self.points("scan=0,-,90,10", 2000)[:5],
                         [0, 270, 180, 90, 0])

    def test_delay_sets_dwell_time(self):
        """Пауза delay отсчитывается от прихода в точку, а не от команды."""
        sim = Sim("op=0", "a=0", "v=60")
        sim.ok("scan=0,90,90,200")          # ход 90° на 60 °/с = 1.5 с
        arrive = sim.run_until(lambda fw: fw.scan_st == S.SCAN_DELAY, 3000)
        leave = sim.run_until(lambda fw: fw.scan_st == S.SCAN_MOVING, 3000)
        self.assertAlmostEqual(leave, 200.0, delta=2.0, msg=f"приход на {arrive} мс")

    def test_scan_sets_target_to_start(self):
        """Первая точка скана — start, приведённый к [0,360)."""
        sim = Sim("op=0")
        self.assertEqual(sim.send("scan=-10,10,5,50"), ["ok:scan=350.00,10.00,5.00,50"])
        self.assertAlmostEqual(sim.fw.target_deg, 350.0, places=6)
        self.assertEqual(sim.fw.scan_range(), (350.0, 10.0))

    def test_infinite_scan_has_no_sector(self):
        """У бесконечного скана сектора нет — подсвечивать на диаграмме нечего."""
        sim = Sim("op=0")
        sim.ok("scan=0,+,5,50")
        self.assertIsNone(sim.fw.scan_range())

    def test_stop_ends_scan(self):
        """stop гасит скан и оставляет цель на текущей позиции."""
        sim = Sim("op=0", "a=0", "v=1200")
        sim.ok("scan=0,90,5,50")
        sim.run(200)
        self.assertTrue(sim.fw.scan_active)
        sim.ok("stop")
        self.assertFalse(sim.fw.scan_active)
        self.assertAlmostEqual(sim.fw.target_deg, sim.fw.cur_deg, places=6)
        pos = sim.fw.cur_deg
        sim.run(300)
        self.assertAlmostEqual(sim.fw.cur_deg, pos, places=6)

    def test_stop_clears_the_infinite_scan_flag(self):
        """stop гасит и признак бесконечного скана (main.c:1499).

        Поведением этот признак наружу не выходит: читают его только при
        активном скане (Scan_Tick), а любая команда scan= задаёт его заново,
        — поэтому проверяется сам флаг. Смысл строки в прошивке в том, чтобы
        остановленный скан не остался в состоянии платы «бесконечным»: по
        этому же признаку GUI решает, есть ли сектор для подсветки.
        """
        sim = Sim("op=0", "a=0", "v=1200")
        sim.ok("scan=0,+,10,50")
        sim.run(200)
        self.assertEqual(sim.fw.scan_inf, 1)
        self.assertIsNone(sim.fw.scan_range())
        sim.ok("stop")
        self.assertFalse(sim.fw.scan_active)
        self.assertEqual(sim.fw.scan_inf, 0)

    def test_target_command_does_not_cancel_scan(self):
        """t= уводит вал в заданный угол, но скан продолжает вести точки.

        Так в прошивке: CMD_SET_TARGET трогает только g_cont_dir, состояние
        скана не сбрасывается.
        """
        sim = Sim("op=0", "a=0", "v=1200")
        sim.ok("scan=0,90,45,50")
        sim.run(100)
        sim.ok("t=200")
        self.assertTrue(sim.fw.scan_active)
        sim.run(300)
        self.assertTrue(sim.fw.scan_active)

    def test_invalid_scan_arguments(self):
        """Неверный аргумент: разобранный — err:scan, неразобранный — молчание."""
        explicit_error = ("scan=0,90,0,50",       # шаг 0
                          "scan=0,90,-5,50",      # шаг отрицательный
                          "scan=0,90,5,0",        # пауза 0
                          "scan=0,360,5,50",      # вырожденный сектор
                          "scan=45,45,5,50")
        silence = ("scan=0,90,5",                 # нет поля delay
                   "scan=0,90,5,",
                   "scan=abc,90,5,50",
                   "scan=0,90,5,70000",           # delay не помещается в uint16
                   "scan=0 90 5 50")
        for cmd in explicit_error:
            with self.subTest(cmd=cmd):
                self.assertEqual(Sim().send(cmd), ["err:scan"])
        for cmd in silence:
            with self.subTest(cmd=cmd):
                self.assertEqual(Sim().send(cmd), [])

    def test_failed_scan_does_not_start(self):
        """Отвергнутая команда скана не оставляет модель в полусостоянии."""
        sim = Sim("op=0")
        sim.send("scan=0,90,0,50")
        self.assertFalse(sim.fw.scan_active)
        sim.send("scan=0,90,5,70000")
        self.assertFalse(sim.fw.scan_active)


class SyncTests(unittest.TestCase):
    """Источник перехода к следующей точке скана (sync=0/1/2)."""

    def start(self, mode: int) -> Sim:
        sim = Sim("op=0", "a=0", "v=1200", f"sync={mode}")
        sim.ok("scan=0,20,5,100")
        return sim

    def test_timer_mode_advances_by_delay(self):
        # Цикл точки = ход + пауза: 5° ПИД проходит за ~184 мс (kp/DT_S·err),
        # delay добавляет 100 мс — за 1 с точек набирается четыре с запасом.
        sim = self.start(0)
        self.assertEqual(sim.scan_points(1000)[:4], [0, 5, 10, 15])

    def test_external_mode_waits_for_edge(self):
        """sync=1: без фронта SYNC_IN скан стоит на точке сколь угодно долго."""
        sim = self.start(1)
        self.assertEqual(sim.scan_points(2000), [0])
        sim.pulse_sync_in()
        self.assertEqual(sim.scan_points(300), [5])
        sim.pulse_sync_in()
        self.assertEqual(sim.scan_points(300), [10])

    def test_external_timeout_mode_advances_without_edges(self):
        """sync=2: delay работает как тайм-аут, фронтов может и не быть."""
        sim = self.start(2)
        self.assertEqual(sim.scan_points(1000)[:4], [0, 5, 10, 15])

    def test_external_timeout_mode_advances_early_on_edge(self):
        """sync=2: пришедший фронт уводит с точки раньше тайм-аута."""
        sim = self.start(2)
        sim.run_until(lambda fw: fw.scan_st == S.SCAN_DELAY, 500)
        sim.run(20)
        sim.pulse_sync_in()
        spent = sim.run_until(lambda fw: fw.scan_sp != 0.0, 200)
        self.assertLess(spent, 80.0, "фронт не сработал раньше тайм-аута")

    def test_edge_before_arrival_is_discarded(self):
        """Фронт, пришедший до прихода в точку, не считается.

        В прошивке g_syncin_trig сбрасывается в момент, когда SYNC_OUT уходит
        в HIGH: иначе «ранний» импульс мгновенно снял бы вал с точки, на
        которой он ещё не постоял.
        """
        sim = Sim("op=0", "a=0", "v=60", "sync=1")
        sim.ok("scan=90,110,5,100")        # до первой точки ехать 1.5 с
        sim.run(200)
        self.assertEqual(sim.fw.scan_st, S.SCAN_MOVING)
        sim.pulse_sync_in()
        sim.run_until(lambda fw: fw.scan_st == S.SCAN_DELAY, 3000)
        self.assertEqual(sim.scan_points(1000), [90.0])

    def test_scan_start_discards_an_early_sync_edge(self):
        """Фронт SYNC_IN, пришедший до команды scan=, не считается (main.c:1485).

        Точкой скана это не проверить: до первой точки вал всё равно идёт
        через SCAN_MOVING, где триггер гасится второй раз (та же защита на
        приходе — test_edge_before_arrival_is_discarded). Сброс в самой
        команде закрывает другое окно: фронт, пойманный опросом SYNC_IN
        между последним тиком и разбором scan=. Поэтому сверяется признак
        перехода, а следом — что настоящий фронт по-прежнему работает.
        """
        sim = Sim("op=0", "a=0", "v=1200", "sync=1")
        sim.pulse_sync_in()
        self.assertEqual(sim.fw.syncin_trig, 1, "фронт не дошёл — проверять нечего")
        sim.ok("scan=0,20,5,100")
        self.assertEqual(sim.fw.syncin_trig, 0)
        self.assertEqual(sim.scan_points(800), [0.0])
        sim.pulse_sync_in()
        self.assertEqual(sim.scan_points(400), [5.0])

    def test_sync_out_level_marks_point(self):
        """SYNC_OUT поднят, пока вал стоит в точке, и снят на ходу."""
        sim = Sim("op=0", "a=0", "v=60", "sync=0")
        sim.ok("scan=0,90,90,100")
        sim.run_until(lambda fw: fw.scan_st == S.SCAN_DELAY, 3000)
        self.assertEqual(P.parse_sync(sim.send("sync")[0])["sync_out"], 1)
        sim.run_until(lambda fw: fw.scan_st == S.SCAN_MOVING, 500)
        self.assertEqual(P.parse_sync(sim.send("sync")[0])["sync_out"], 0)

    def test_sync_query_reports_mode_and_edges(self):
        sim = Sim("sync=2")
        sim.pulse_sync_in()
        sim.pulse_sync_in()
        got = P.parse_sync(sim.send("sync")[0])
        self.assertEqual(got["sync_mode"], 2)
        self.assertEqual(got["sync_edges"], 2)

    def test_invalid_sync_mode_is_silent(self):
        sim = Sim("sync=2")
        for cmd in ("sync=3", "sync=-1", "sync=abc"):
            with self.subTest(cmd=cmd):
                self.assertEqual(sim.send(cmd), [])
        self.assertEqual(sim.fw.sync_mode, 2)


class HoldTests(unittest.TestCase):
    """hold= — тихая пауза: ток снят, цель и точка скана сохранены."""

    def test_hold_off_stops_the_shaft(self):
        sim = Sim("op=0", "a=0", "v=60")
        sim.ok("t=180")
        sim.run(200)
        pos = sim.fw.cur_deg
        self.assertEqual(sim.send("hold=0"), ["ok:hold=0"])
        sim.run(500)
        self.assertAlmostEqual(sim.fw.cur_deg, pos, places=9)
        self.assertFalse(sim.fw.is_moving())
        self.assertAlmostEqual(sim.fw.target_deg, 180.0, places=6)

    def test_hold_on_resumes_to_the_same_target(self):
        sim = Sim("op=0", "a=0", "v=60")
        sim.ok("t=90")
        sim.run(200)
        sim.ok("hold=0")
        sim.run(500)
        sim.ok("hold=1")
        sim.settle(limit_ms=3000)
        self.assertLess(abs(P.wrap180(sim.fw.cur_deg - 90.0)), DEADBAND)

    def test_hold_off_freezes_scan_dwell(self):
        """Таймер паузы в точке не тикает: точку нельзя проехать без тока."""
        sim = Sim("op=0", "a=0", "v=1200")
        sim.ok("scan=0,20,5,100")
        sim.run_until(lambda fw: fw.scan_st == S.SCAN_DELAY, 500)
        sim.run(20)
        sim.ok("hold=0")
        point, dwell = sim.fw.scan_sp, sim.fw.scan_dwell_ms
        sim.run(1000)
        self.assertAlmostEqual(sim.fw.scan_sp, point, places=9)
        self.assertAlmostEqual(sim.fw.scan_dwell_ms, dwell, places=9)
        self.assertTrue(sim.fw.scan_active, "скан не должен гаснуть от hold=0")
        sim.ok("hold=1")
        # Пауза досчитывается с того места, где её заморозили: 100 - 20 = 80 мс
        left = sim.run_until(lambda fw: fw.scan_sp != point, 500)
        self.assertAlmostEqual(left, 80.0, delta=3.0)
        self.assertAlmostEqual(sim.fw.scan_sp, 5.0, places=6)

    def test_hold_restarts_the_speed_ramp(self):
        """hold= сбрасывает рампу скорости (main.c:1621 Control_Reset()).

        За паузу без тока накопленная скорость протухла — вал всё это время
        стоял. Не обнулить её значит после hold=1 тронуться сразу с прежней
        уставки, минуя разгон: на плате это рывок на полном токе.
        """
        sim = Sim("op=0", "a=2000", "v=1200")
        sim.ok("t=180")
        sim.run(100)                      # разгон: 2000 °/с² × 0.1 с = 200 °/с
        self.assertGreater(sim.fw.vel, 150.0, "вал не разогнался — проверять нечего")
        sim.ok("hold=0")
        self.assertAlmostEqual(sim.fw.vel, 0.0, places=9)
        sim.run(200)
        sim.ok("hold=1")
        self.assertAlmostEqual(sim.fw.vel, 0.0, places=9)
        # Первые 10 мс после возврата тока: разгон с нуля даёт ≈0.1°,
        # сохранённая рампа — больше 2° (200 °/с × 10 мс).
        moved = abs(sum(sim.path(10)))
        self.assertLess(moved, 0.5, f"вал рванул с прежней скорости: {moved:.2f}°")

    def test_jog_and_enable_restore_hold(self):
        """t=+ и en возвращают удержание: иначе вал не тронулся бы с места."""
        sim = Sim("op=0", "a=0", "v=60")
        sim.ok("hold=0")
        sim.ok("t=+")
        self.assertEqual(sim.fw.hold, 1)
        sim.ok("stop")
        sim.ok("hold=0")
        sim.ok("en")
        self.assertEqual(sim.fw.hold, 1)

    def test_invalid_hold_is_silent(self):
        sim = Sim()
        for cmd in ("hold=2", "hold=-1", "hold=on"):
            with self.subTest(cmd=cmd):
                self.assertEqual(sim.send(cmd), [])
        self.assertEqual(sim.fw.hold, 1)


class OutputModeTests(unittest.TestCase):
    """om= — источник кадра телеметрии: период op=, приход в цель или оба.

    Окна ожидания здесь считаются по времени прихода в цель. Оно задано ПИД,
    а не пределом v=: скорость равна kp/DT_S·err = 25·err °/с, поэтому ход
    90° при a=0, v=1200 занимает не 75 мс, а ~310 мс — предел скорости держит
    только первые 42°, остальное дожимает регулятор (main.c:976-978).
    """

    def test_event_frame_once_per_target(self):
        """om=1 при op=0: ровно один кадр ev:1 на цель и больше ничего."""
        sim = Sim("op=0", "om=1", "a=0", "v=1200")
        sim.ok("t=90")
        sim.run(500)
        self.assertEqual(len(sim.frames), 1)
        self.assertEqual(len(sim.events()), 1)
        data = P.parse_telemetry(sim.events()[0])
        self.assertEqual(data["ev"], 1)
        self.assertAlmostEqual(data["cp"], 90.0, places=2)
        sim.run(2000)
        self.assertEqual(len(sim.frames), 1, "кадры сыплются на стоянке в цели")

    def test_new_target_arms_new_event(self):
        """Новая цель — новый кадр, даже если угол тот же (Target_ResetReached)."""
        sim = Sim("op=0", "om=1", "a=0", "v=1200")
        for _ in range(3):
            sim.ok("t=90")
            sim.run(400)
        self.assertEqual(len(sim.events()), 3)

    def test_period_mode_gives_no_event_frames(self):
        """om=0: событийных кадров нет вовсе, только периодические."""
        sim = Sim("op=20", "om=0", "a=0", "v=1200")
        sim.ok("t=90")
        sim.run(500)
        self.assertEqual(sim.events(), [])
        self.assertGreaterEqual(len(sim.periodic()), 20)

    def test_both_sources(self):
        """om=2: периодический поток идёт, и на цель добавляется один ev:1."""
        sim = Sim("op=50", "om=2", "a=0", "v=1200")
        sim.ok("t=45")
        sim.run(300)
        self.assertEqual(len(sim.events()), 1)
        self.assertGreaterEqual(len(sim.periodic()), 5)

    def test_event_per_scan_point(self):
        """Каждая точка скана — отдельная цель, значит отдельный кадр.

        Считать точки по формуле «окно / delay» нельзя: цикл точки — это ход
        плюс пауза, а ход задаёт ПИД (5° ≈ 184 мс против delay 50 мс). Поэтому
        сверяем кадры с фактически пройденными точками.
        """
        sim = Sim("op=0", "om=1", "a=0", "v=1200")
        sim.ok("scan=0,20,5,50")
        points = sim.scan_points(1500)
        sim.run(TICK_MS)        # событие взводится тиком после доводочного шага
        self.assertGreaterEqual(len(points), 4, "скан почти не двигался")
        self.assertEqual(len(sim.events()), len(points))
        self.assertEqual(len(sim.periodic()), 0)

    def test_no_events_during_continuous_rotation(self):
        """В непрерывном вращении понятия «цель достигнута» нет."""
        sim = Sim("op=0", "om=1", "a=0", "v=360")
        sim.ok("t=+")
        sim.run(2000)
        self.assertEqual(sim.frames, [])

    def test_event_frame_holds_arrival_snapshot(self):
        """Кадр несёт угол момента прихода, даже если ушёл позже.

        На плате кадр может задержаться на пару тиков (очередь TX занята), и
        за это время скан уже уедет — хосту нужен угол того момента, к
        которому привязан замер.
        """
        sim = Sim("op=0", "om=1", "a=0", "v=1200")
        sim.ok("t=90")
        sim.run(400, telemetry=False)          # кадр взведён, но не выдан
        sim.ok("t=270")
        sim.run(50, telemetry=False)
        self.assertNotAlmostEqual(sim.fw.cur_deg, 90.0, places=1)
        sim.run(TICK_MS)
        self.assertEqual(len(sim.events()), 1)
        self.assertAlmostEqual(P.parse_telemetry(sim.events()[0])["cp"], 90.0,
                               places=2)

    def test_lost_event_frame_counted_in_drp(self):
        """Вытесненный событийный кадр виден хосту счётчиком drp."""
        sim = Sim("op=0", "om=1", "a=0", "v=1200", "debug=1")
        sim.ok("t=90")
        sim.run(400, telemetry=False)
        self.assertEqual(sim.fw.drp, 0)
        sim.ok("t=180")
        sim.run(400, telemetry=False)          # приход в новую цель вытесняет кадр
        self.assertEqual(sim.fw.drp, 1)
        sim.run(TICK_MS)
        self.assertEqual(P.parse_telemetry(sim.events()[0])["drp"], 1)

    def test_switching_mode_drops_armed_event(self):
        """Смена om= отменяет взведённый кадр: хост ждёт кадр по своей цели."""
        sim = Sim("op=0", "om=1", "a=0", "v=1200")
        sim.ok("t=90")
        sim.run(400, telemetry=False)
        sim.ok("om=2")
        sim.run(500)
        self.assertEqual(sim.events(), [])

    def test_debug_enforces_minimum_period(self):
        """debug=1 не даёт слать длинные кадры чаще OUTPUT_PERIOD_MS_DEBUG_MIN.

        Ожидаемые числа записаны явно, а не выражены через саму проверяемую
        константу: 200 мс / op=4 мс = 50 кадров при debug=0 и 200 мс / 20 мс
        (board.h OUTPUT_PERIOD_MS_DEBUG_MIN) = 10 кадров при debug=1. Само
        совпадение 20 мс с board.h стережёт tests/test_board_constants.py.
        """
        fast = Sim("op=4", "om=0", "debug=0")
        fast.run(200)
        slow = Sim("op=4", "om=0", "debug=1")
        slow.run(200)
        self.assertEqual(len(fast.frames), 50)
        self.assertEqual(len(slow.frames), 10)
        # Период поднимается только до минимума: op= крупнее него не трогается
        rare = Sim("op=50", "om=0", "debug=1")
        rare.run(200)
        self.assertEqual(len(rare.frames), 4)

    def test_period_counter_restarts_after_a_silent_stretch(self):
        """Счётчик периода обнуляется, пока периодического источника нет.

        main.c:1217: в ветке «периода нет» (op=0 или om=1) cnt = 0. Иначе
        после возврата op= первый кадр ушёл бы не через полный период, а на
        остатке счётчика, накопленном до выключения, — и хост, меряющий
        частоту потока по первым кадрам, увидел бы неверный период.
        """
        for off, on in (("om=1", "om=0"), ("op=0", "op=20")):
            with self.subTest(off=off):
                sim = Sim("op=20", "om=0")
                sim.run(15)                 # счётчик дошёл до 15 из 20
                self.assertEqual(sim.frames, [])
                sim.ok(off)
                sim.run(50)
                self.assertEqual(sim.frames, [], "поток не остановился")
                sim.ok(on)
                sim.run(15)
                self.assertEqual(sim.frames, [],
                                 "кадр ушёл раньше полного периода op=")
                sim.run(6)
                self.assertEqual(len(sim.frames), 1)

    def test_switching_to_event_mode_at_target_gives_no_frame(self):
        """Кадр «приехали» — это переход «еду → приехал», а не состояние.

        Уровень g_at_target (охранник main.c:504) держится, пока цель не
        сменилась, поэтому om=1, включённый на уже стоящем в цели вале,
        кадра не даёт: хост получит его по своей следующей цели. Без этого
        охранника кадр уходил бы каждый тик стоянки.
        """
        sim = Sim("op=0", "om=0", "a=0", "v=1200")
        sim.ok("t=90")
        sim.settle(limit_ms=3000)
        sim.run(50)
        sim.ok("om=1")
        sim.run(500)
        self.assertEqual(sim.frames, [])
        sim.ok("t=180")                     # новая цель — кадр появляется
        sim.run(500)
        self.assertEqual(len(sim.events()), 1)

    def test_return_to_deadband_gives_no_second_frame(self):
        """Право на кадр выдаётся только сменой цели (охранник main.c:509).

        Вал, вышедший из мёртвой зоны и вернувшийся без новой цели (его
        подвинули рукой на снятом удержании — ровно то, для чего hold=0 и
        нужен; на плате то же даёт шум энкодера у самой границы зоны),
        второго кадра по той же цели не даёт.
        """
        sim = Sim("op=0", "om=1", "a=0", "v=1200")
        sim.ok("t=90")
        sim.run(500)
        self.assertEqual(len(sim.events()), 1)
        sim.ok("hold=0")
        sim.fw.cur_deg = 90.5               # вал подвинули без тока в обмотках
        sim.ok("hold=1")
        back = sim.run_until(lambda fw: not fw.is_moving(), 2000)
        self.assertLess(back, 2000.0, "вал не вернулся в зону — проверять нечего")
        sim.run(200)
        self.assertEqual(len(sim.events()), 1,
                         "второй кадр по той же цели")

    def test_period_zero_silences_periodic_stream(self):
        sim = Sim("op=0", "om=0")
        sim.ok("t=90")
        sim.run(1000)
        self.assertEqual(sim.frames, [])

    def test_invalid_output_mode_is_silent(self):
        sim = Sim("om=1")
        for cmd in ("om=3", "om=-1", "om=abc"):
            with self.subTest(cmd=cmd):
                self.assertEqual(sim.send(cmd), [])
        self.assertEqual(sim.fw.telem_mode, 1)


class StateQueryTests(unittest.TestCase):
    """Запросы состояния без аргумента: om и hold.

    Отвечают одной строкой данных БЕЗ префикса ok:, как запрос sync
    (main.c CMD_GET_OUTPUT_MODE: SendResponse("om=%u\\r\\n", g_telem_mode);
    CMD_GET_HOLD: SendResponse("hold=%u\\r\\n", g_hold)). Хосту это нужно
    после переподключения: плата могла перезагрузиться, и своё последнее
    отправленное значение о ней уже ничего не говорит.
    """

    def test_output_mode_query_reports_current_mode(self):
        sim = Sim()
        self.assertEqual(sim.send("om"), ["om=0"],
                         "старт — TELEMETRY_MODE_DEFAULT из board.h")
        for n in (1, 2, 0):
            with self.subTest(mode=n):
                self.assertEqual(sim.ok(f"om={n}"), f"ok:om={n}")
                self.assertEqual(sim.send("om"), [f"om={n}"])

    def test_hold_query_reports_current_hold(self):
        sim = Sim()
        self.assertEqual(sim.send("hold"), ["hold=1"],
                         "прошивка стартует с удержанием (g_hold = 1)")
        self.assertEqual(sim.ok("hold=0"), "ok:hold=0")
        self.assertEqual(sim.send("hold"), ["hold=0"])
        # en возвращает удержание, не отвечая ok:hold= (main.c CMD_ENABLE):
        # именно поэтому запрос и нужен — по эху ok: это состояние не узнать.
        sim.ok("en")
        self.assertEqual(sim.send("hold"), ["hold=1"])

    def test_query_answers_are_replies_not_telemetry(self):
        """GUI обязан считать эти строки ответом на команду.

        Иначе контроллер не снимет ожидание ответа и запрос упрётся в
        таймаут, а строка уйдёт в разбор телеметрии и потеряется.
        """
        sim = Sim("om=2", "hold=0")
        om, hold = sim.send("om")[0], sim.send("hold")[0]
        for line in (om, hold):
            with self.subTest(line=line):
                self.assertEqual(P.classify_line(line), "reply")
                self.assertIsNone(P.parse_telemetry(line))
                self.assertFalse(line.startswith("ok:"),
                                 "запрос отвечает данными, а не эхом ok:")
        self.assertEqual(P.parse_state_reply(om), ("output_mode", 2))
        self.assertEqual(P.parse_state_reply(hold), ("hold", 0))

    def test_query_does_not_change_state(self):
        """Запрос только читает: взведённый событийный кадр он не трогает.

        В обработчике CMD_GET_OUTPUT_MODE прошивки нет ничего, кроме
        SendResponse, — в отличие от установки om=N, которая отменяет
        событие прежнего режима (test_switching_mode_drops_armed_event).
        Скопируй эту отмену в запрос — и кадр ev:1 по уже заданной цели
        пропал бы.

        Ловится это только в окне «кадр взведён, но ещё не выдан»: до
        прихода в цель отменять нечего, а после выдачи — поздно. Окно
        держится тем же приёмом, что и в соседних тестах: telemetry=False
        не даёт модели выдать кадр (как занятая очередь TX у платы).
        """
        sim = Sim("op=0", "om=1", "a=0", "v=1200")
        sim.ok("t=90")
        sim.run(400, telemetry=False)          # кадр взведён, но не выдан
        self.assertTrue(sim.fw._evt_pending, "кадр не взведён — окна нет")
        self.assertEqual(sim.send("om"), ["om=1"])
        self.assertEqual(sim.send("hold"), ["hold=1"])
        sim.run(TICK_MS)
        self.assertEqual(len(sim.events()), 1,
                         "запрос состояния съел взведённый событийный кадр")
        self.assertEqual(sim.fw.telem_mode, 1)
        self.assertEqual(sim.fw.hold, 1)

    def test_query_with_a_stray_space_is_unknown(self):
        """«om », «hold » и прочие вариации — не запросы, а err:unknown.

        Cmd_Parse сравнивает строку целиком (strcmp(line, "om")), пробелы по
        краям сборщик не снимает (line_reader.c:52-60), регистр strcmp не
        прощает — путь ровно тот же, что у «sync ».
        """
        for cmd in ("om ", " om", "hold ", " hold", "OM", "Hold", "omm"):
            with self.subTest(cmd=cmd):
                self.assertEqual(Sim().send(cmd), ["err:unknown"])


class CommandParsingTests(unittest.TestCase):
    """Разбор аргументов повторяет strtof/strtol прошивки."""

    def test_trailing_garbage_is_ignored(self):
        """Прошивка читает числовой префикс и хвост не замечает (cmd_parser.c)."""
        cases = (("op=10x", "ok:op=10"), ("t=90abc", "ok:t=90.00"),
                 ("om=2)", "ok:om=2"), ("hold=1x", "ok:hold=1"),
                 ("icur 7 3 9", "ok:icur=7,3"), ("v=600 ", "ok:v=600.0"))
        for cmd, expect in cases:
            with self.subTest(cmd=cmd):
                self.assertEqual(Sim().send(cmd), [expect])

    def test_unreadable_argument_is_silent(self):
        """Нечитаемое число — парсер вернул 0, прошивка не отвечает ничем."""
        for cmd in ("t=", "t=abc", "op=", "op=abc", "v=x", "debug=2",
                    "irun 4000", "irun abc", "icur 600", "mstep abc"):
            with self.subTest(cmd=cmd):
                self.assertEqual(Sim().send(cmd), [])

    def test_infinities_rejected(self):
        """inf/nan прошивка отбрасывает проверкой isfinite и молчит."""
        for cmd in ("t=inf", "t=nan", "t=-inf", "v=1e400"):
            with self.subTest(cmd=cmd):
                self.assertEqual(Sim().send(cmd), [])

    def test_hex_literals_are_read_by_strtof_only(self):
        """Вещественный аргумент читается по C99 вместе с формой 0x, целый — нет.

        strtof в newlib разбирает шестнадцатеричные вещественные литералы (в
        прошивку слинкованы __gethex/__hexdig_fun), поэтому t=0x10 плата
        выполняет как поворот на 16°. strtol же вызывается с основанием 10
        (cmd_parser.c:29) и останавливается на 'x', так что у целых аргументов
        шестнадцатеричной формы нет — читается только ведущий ноль.
        """
        for cmd, expect in (("t=0x10", "ok:t=16.00"),
                            ("t=0X10", "ok:t=16.00"),
                            ("t=0x1.8p1", "ok:t=3.00"),
                            ("t=-0x10", "ok:t=344.00"),
                            ("v=0x64", "ok:v=100.0"),
                            # хвост без цифр экспоненты strtof откатывает
                            ("t=0x1p", "ok:t=1.00"),
                            # целые: strtol дочитал ноль и встал на 'x'
                            ("op=0x10", "ok:op=0"),
                            ("hold=0x1", "ok:hold=0"),
                            ("sync=0x2", "ok:sync=0")):
            with self.subTest(cmd=cmd):
                self.assertEqual(Sim().send(cmd), [expect])

    def test_hex_prefix_without_digits_is_a_plain_zero(self):
        """Префикс 0x без шестнадцатеричных цифр — обычный ноль, курсор на 'x'.

        __gethex возвращает STRTOG_NoNumber, и _strtod_l ставит курсор сразу
        за ведущим нулём (проверено по дизассемблеру strtof в firmware.elf).
        """
        for cmd in ("t=0x", "t=0xg", "t=0x.p"):
            with self.subTest(cmd=cmd):
                self.assertEqual(Sim().send(cmd), ["ok:t=0.00"])

    def test_value_out_of_float_range_is_silent(self):
        """strtof отдаёт float: 1e40 в него не влезает и гибнет на !isfinite."""
        for cmd in ("t=1e40", "kp=-1e40", "a=0x1p9999"):
            with self.subTest(cmd=cmd):
                self.assertEqual(Sim().send(cmd), [])
        # ...а то, что в float влезает, принимается
        self.assertTrue(Sim().send("kp=1e38")[0].startswith("ok:kp="))

    def test_leading_whitespace_follows_isspace(self):
        """Ведущие пробелы strtof/strtol снимают по isspace — вместе с \\v и \\f."""
        self.assertEqual(Sim().send("t=\v90"), ["ok:t=90.00"])
        self.assertEqual(Sim().send("op=\f5"), ["ok:op=5"])

    def test_unknown_command(self):
        self.assertEqual(Sim().send("hello"), ["err:unknown"])
        self.assertEqual(Sim().send("scan"), ["err:unknown"])
        self.assertEqual(Sim().send(""), [])

    def test_spaces_are_part_of_the_command(self):
        """Пробелы по краям прошивка не снимает — команда с ними неизвестна.

        Сборщик строки (line_reader.c:52-60) копирует все байты, пропуская
        только CR и LF, а Cmd_Parse сравнивает строку как есть (strcmp/strncmp,
        cmd_parser.c:66-96). Поэтому « en» и «   » — это CMD_UNKNOWN, на
        который ProcessCommand отвечает err:unknown (main.c CMD_UNKNOWN).
        """
        for cmd in ("   ", " en", "en ", " t=90", "\ten"):
            with self.subTest(cmd=cmd):
                self.assertEqual(Sim().send(cmd), ["err:unknown"])

    def test_eol_is_stripped_by_the_line_reader(self):
        """CR/LF в строку команды не попадают: их снимает сборщик строки.

        Пустую строку (одни CR/LF) line_reader пропускает и команду не отдаёт
        вовсе — прошивка на неё не отвечает ничем.
        """
        self.assertEqual(Sim().send("en\r\n"), ["ok:en"])
        self.assertEqual(Sim().send("t=90\n"), ["ok:t=90.00"])
        self.assertEqual(Sim().send("\r\n"), [])

    def test_empty_line_does_not_stop_the_reader(self):
        """Пустую строку сборщик пропускает и ищет следующую в том же кольце.

        line_reader.c:66-69: наружу пустая строка не уходит, но и разбор на
        ней не прекращается — цикл while(1) идёт за следующей. Поэтому
        "\\r\\nen\\r\\n" плата выполняет, а не молчит на первом пустом куске.
        Команды здесь завершены терминатором: без него сборщик их и не
        соберёт, это отдельное свойство (см. тесты про хвост ниже).
        """
        sim = Sim()
        self.assertEqual(sim.send_raw("\r\nen\r\n"), ["ok:en"])
        self.assertEqual(sim.send_raw("\n\n\nt=90\n"), ["ok:t=90.00"])
        self.assertAlmostEqual(sim.fw.target_deg, 90.0, places=6)

    def test_several_commands_in_one_chunk_all_run(self):
        """Две завершённые команды в одном куске — обе выполняются, по порядку.

        Плата вычитывает из кольца по одной строке за тик главного цикла
        (main.c:1350-1356 PollCommands), поэтому ответы приходят подряд и в
        том же порядке; модель отдаёт их одним списком.
        """
        sim = Sim()
        self.assertEqual(sim.send_raw("en\rdis\r"), ["ok:en", "ok:dis"])
        self.assertFalse(sim.fw.enabled, "вторую команду куска съели")
        self.assertEqual(sim.send_raw("t=30\r\nt=60\r\n"),
                         ["ok:t=30.00", "ok:t=60.00"])
        self.assertAlmostEqual(sim.fw.target_deg, 60.0, places=6)

    def test_unterminated_tail_waits_in_the_receiver(self):
        """Без CR/LF команда не выполняется — хвост ждёт терминатора.

        line_reader.c:39-46: не найдя конца строки, сборщик возвращает 0 и
        tail не двигает, то есть байты остаются в кольце до следующего
        вызова. Поэтому «t=9» ответа не даёт вовсе, а выполняется тогда,
        когда терминатор приедет следующим куском.
        """
        sim = Sim()
        self.assertEqual(sim.send_raw("t=9"), [])
        self.assertAlmostEqual(sim.fw.target_deg, 0.0, places=6,
                               msg="незавершённая команда уже выполнилась")
        self.assertEqual(sim.send_raw("0\r\n"), ["ok:t=90.00"])
        self.assertAlmostEqual(sim.fw.target_deg, 90.0, places=6)

    def test_unterminated_tail_glues_to_the_next_chunk(self):
        """Хвост без терминатора склеивается со следующим куском.

        «en\\rdis» — это одна завершённая команда и хвост «dis» в кольце.
        Пришедшая следом «t=90\\r\\n» достраивает строку «dist=90», которую
        Cmd_Parse не узнаёт: err:unknown, а не ok:t=90.00. Именно на этом
        месте модель раньше расходилась с платой — она выполняла хвост сразу.
        """
        sim = Sim()
        self.assertEqual(sim.send_raw("en\rdis"), ["ok:en"])
        self.assertTrue(sim.fw.enabled, "хвост куска выполнился как команда")
        self.assertEqual(sim.send_raw("t=90\r\n"), ["err:unknown"])
        self.assertAlmostEqual(sim.fw.target_deg, 0.0, places=6)

    def test_crlf_split_between_chunks_adds_no_command(self):
        """Пара CR+LF, разорванная на границе кусков, лишней команды не даёт.

        Кольцо кончается на CR, и заглянуть за него сборщику некуда
        (line_reader.c:30-33 смотрит следующий байт только при t != head),
        поэтому строка закрывается по CR. Пришедший следом LF образует
        пустую строку, которую сборщик пропускает (line_reader.c:66-69).
        """
        sim = Sim()
        self.assertEqual(sim.send_raw("en\r"), ["ok:en"])
        self.assertEqual(sim.send_raw("\ndis\r\n"), ["ok:dis"])
        self.assertFalse(sim.fw.enabled)


class DriverCommandTests(unittest.TestCase):
    """Команды TMC2209: токи, микрошаг, mcfg и диагностика энкодера."""

    def test_currents_reported_by_mcfg(self):
        sim = Sim("irun 800", "ihold 250", "mstep 32")
        cfg = P.parse_mcfg(sim.send("mcfg")[0])
        self.assertEqual((cfg["run"], cfg["hold"], cfg["microsteps"]),
                         (800, 250, 32))
        self.assertEqual(cfg["ready"], 1)

    def test_current_out_of_range_is_silent(self):
        sim = Sim("irun 800")
        for cmd in ("irun 3001", "ihold 5000", "icur 4000 300", "irun -1"):
            with self.subTest(cmd=cmd):
                self.assertEqual(sim.send(cmd), [])
        self.assertEqual(sim.fw.irun, 800)

    def test_microstep_rejected_on_moving_shaft(self):
        """Порядок проверок как в прошивке: сначала «мотор крутится».

        Различить порядок можно только сочетанием «негодное значение + вал в
        движении»: на нём прошивка отвечает err:busy, потому что проверку
        tmc2209_motor_is_moving() она делает ДО обращения к драйверу
        (main.c:1564-1577), а значение микрошага разбирает уже сам драйвер.
        Поменяй две проверки местами — и тот же mstep 3 дал бы err:bad arg.
        """
        sim = Sim("op=0", "a=0", "v=60")
        sim.ok("t=180")
        sim.run(50)
        self.assertTrue(sim.fw.is_moving())
        self.assertEqual(sim.send("mstep 16"), ["err:busy stop motor first"])
        # Ключевая проверка порядка: значение негодное И вал крутится
        self.assertEqual(sim.send("mstep 3"), ["err:busy stop motor first"])
        self.assertEqual(sim.fw.microsteps, P.DEFAULTS.microsteps,
                         "отвергнутая команда не должна менять микрошаг")
        sim.ok("stop")
        self.assertEqual(sim.send("mstep 16"), ["ok:mstep=16"])
        # Вал стоит — до драйвера дошло, и негодное значение отвергает уже он
        self.assertEqual(sim.send("mstep 3"),
                         ["err:bad arg (1/2/4/8/16/32/64/128/256)"])
        self.assertEqual(sim.fw.microsteps, 16)

    def test_microstep_above_uint16_is_silent(self):
        """Верхняя граница аргумента mstep проверяется ДО приведения к uint16.

        cmd_parser.c, блок «mstep »: `parse_int(&p, &v) != 0 || v < 0 ||
        v > 65535` — не сойдясь, Cmd_Parse возвращает 0, и до обработчика
        команда не доходит вовсе: ответа нет никакого. Раньше границы не
        было, и «mstep 65537» усекалось до microsteps=1: плата отвечала
        ok:mstep=1 и переключала драйвер на полный шаг, меняя масштаб
        градус↔шаг в 256 раз, — а модель на ту же строку отвечала
        err:bad arg. Значения 65537 (усечение дало бы годную единицу) и
        66048 (усечение дало бы 512) закрывают обе ветки того усечения.
        """
        sim = Sim("mstep 16")
        for cmd in ("mstep 65536", "mstep 65537", "mstep 66048",
                    "mstep 131072", "mstep -1"):
            with self.subTest(cmd=cmd):
                self.assertEqual(sim.send(cmd), [])
        self.assertEqual(sim.fw.microsteps, 16,
                         "молчаливая команда не должна менять микрошаг")
        # 65535 в uint16 влезает: команда доходит до обработчика, и уже он
        # отвергает значение как неподходящее для чипа — явной ошибкой.
        self.assertEqual(sim.send("mstep 65535"),
                         ["err:bad arg (1/2/4/8/16/32/64/128/256)"])

    def test_microstep_above_uint16_is_silent_even_on_a_moving_shaft(self):
        """Парсер отбраковывает раньше, чем обработчик смотрит на движение.

        ProcessCommand вызывается только при удачном разборе (main.c:1354:
        `if (UART_ReadLine(...) > 0 && Cmd_Parse(line, &cmd))`), поэтому на
        «mstep 65537» прошивка молчит даже на крутящемся вале — err:busy
        достаётся лишь разобранным командам.
        """
        sim = Sim("op=0", "a=0", "v=60")
        sim.ok("t=180")
        sim.run(50)
        self.assertTrue(sim.fw.is_moving())
        self.assertEqual(sim.send("mstep 65537"), [])
        self.assertEqual(sim.send("mstep 16"), ["err:busy stop motor first"])

    def test_snap_step_window_counts_as_movement(self):
        """Доводочная серия шагов — это движение, хотя ошибка уже нулевая.

        Прошивка отдаёт серию таймеру (DoSteps → s_pulses_left > 0), и пока
        импульсы не отработаны, tmc2209_motor_is_moving() истинно
        (main.c:1006-1009). Критерий «|err| > DEADBAND» такого окна не знает
        вовсе и пустил бы в него mstep и diag.
        """
        sim = Sim("op=0")
        sim.ok("t=10")
        spent = sim.run_until(lambda fw: fw.snap_busy)
        self.assertNotEqual(spent, float("inf"), "доводочного шага не случилось")
        self.assertLess(abs(sim.err()), DEADBAND,
                        "доводка уже посадила вал в цель")
        self.assertTrue(sim.fw.is_moving())
        self.assertEqual(sim.send("diag"), ["err:busy stop motor first"])
        self.assertEqual(sim.send("mstep 16"), ["err:busy stop motor first"])
        # Серия укладывается в один период опроса — следующим тиком окно закрыто
        sim.run(TICK_MS)
        self.assertFalse(sim.fw.is_moving())
        self.assertEqual(sim.send("diag")[0], "ok:diag")
        self.assertEqual(sim.send("mstep 16"), ["ok:mstep=16"])

    def test_diag_requires_stopped_shaft(self):
        """diag — замер спреда: на ходу и в скане не выполняется."""
        sim = Sim("op=0", "a=0", "v=60")
        sim.ok("t=180")
        sim.run(50)
        self.assertEqual(sim.send("diag"), ["err:busy stop motor first"])
        sim.ok("stop")
        rep = sim.send("diag")
        self.assertEqual(rep[0], "ok:diag")
        self.assertTrue(rep[1].startswith("enc:ok "))
        self.assertEqual(P.classify_line(rep[1]), "other")
        sim.ok("scan=0,90,5,50")
        self.assertEqual(sim.send("diag"), ["err:busy stop motor first"])

    def test_disable_stops_the_shaft(self):
        """dis снимает разрешение: вал стоит, команда t= движения не даёт."""
        sim = Sim("op=0", "a=0", "v=60")
        sim.ok("dis")
        sim.ok("t=90")
        sim.run(500)
        self.assertAlmostEqual(sim.fw.cur_deg, 0.0, places=9)
        sim.ok("en")
        self.assertAlmostEqual(sim.fw.target_deg, sim.fw.cur_deg, places=9)

    def test_boot_lines_are_banner_then_diagnostics(self):
        """Стартовые строки платы: boot-баннер, затем отчёт диагностики.

        Порядок и формат — как в загрузочной последовательности прошивки:
        Boot_Banner() шлёт «boot:rst=<флаги> fw=<версия>» сразу после подъёма
        UART (main.c:309), а enc:ok приходит уже после стартовой диагностики
        энкодера. Телеметрией ни та, ни другая строка не считается.
        """
        lines = Sim().fw.boot_lines()
        self.assertEqual(len(lines), 2)
        self.assertRegex(lines[0], r"^boot:rst=[a-z,\-]+ fw=\S+$")
        self.assertTrue(lines[1].startswith("enc:ok "))
        for line in lines:
            self.assertEqual(P.classify_line(line), "other")


if __name__ == "__main__":
    unittest.main(verbosity=2)
