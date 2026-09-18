"""Юнит-тесты sonar_gui/protocol.py — общего для GUI и модели слоя протокола.

Проверяется ровно то, от чего зависит работа с живой платой:
- строка телеметрии совпадает с форматом прошивки байт в байт и разбирается
  обратно без потерь (debug=0 и debug=1, событийный кадр ev:1);
- входящая строка относится к тому классу, по которому GUI её раздаёт;
- клиентский валидатор отсекает ровно те аргументы, на которые прошивка
  молчит (иначе GUI ждал бы ответа, которого не будет);
- команда, собранная построителем, принимается моделью прошивки.
"""
from __future__ import annotations

import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import _support     # noqa: F401,E402 - ставит sonar_gui на путь импорта
from _support import P, Sim                        # noqa: E402


class TelemetryFormatTests(unittest.TestCase):
    """Формат кадра телеметрии и симметрия format_telemetry/parse_telemetry."""

    ARGS = dict(cp=123.456, tp=359.994, pe=-179.25, u=0.012345, mode="cl",
                ec=6, kp=0.0250, ki=1.5, kd=0.000125, drp=7,
                vmax=1200.0, accel=2000.0, outliers=42)

    def test_debug0_line_matches_firmware(self):
        """debug=0: строка ровно 'cp:%.2f,ec:%u' (main.c)."""
        line = P.format_telemetry(debug=False, **self.ARGS)
        self.assertEqual(line, "cp:123.46,ec:6")

    def test_debug1_line_matches_firmware(self):
        """debug=1: порядок, имена и точность полей — как в snprintf прошивки."""
        line = P.format_telemetry(debug=True, **self.ARGS)
        self.assertEqual(
            line,
            "cp:123.46,tp:359.99,pe:-179.25,u:0.0123,m:cl,ec:6,"
            "kp:0.0250,ki:1.5000,kd:0.0001,v:1200.0,a:2000.0,of:42,drp:7")

    def test_debug0_roundtrip_keys(self):
        """В коротком кадре есть только cp и ec — лишних ключей парсер не выдумывает."""
        got = P.parse_telemetry(P.format_telemetry(debug=False, **self.ARGS))
        self.assertEqual(set(got), {"cp", "ec"})
        self.assertAlmostEqual(got["cp"], 123.46, places=6)
        self.assertEqual(got["ec"], 6)

    def test_debug1_roundtrip_values(self):
        """Полный кадр разбирается обратно с точностью формата прошивки."""
        got = P.parse_telemetry(P.format_telemetry(debug=True, **self.ARGS))
        self.assertEqual(set(got), {"cp", "tp", "pe", "u", "m", "ec", "kp",
                                    "ki", "kd", "v", "a", "of", "drp"})
        for key, src, places in (("cp", "cp", 2), ("tp", "tp", 2),
                                 ("pe", "pe", 2), ("u", "u", 4),
                                 ("kp", "kp", 4), ("ki", "ki", 4),
                                 ("kd", "kd", 4), ("v", "vmax", 1),
                                 ("a", "accel", 1)):
            with self.subTest(field=key):
                self.assertAlmostEqual(got[key], self.ARGS[src],
                                       delta=0.5 * 10 ** (-places))
        self.assertEqual(got["m"], "cl")
        self.assertEqual(got["ec"], 6)
        self.assertEqual(got["of"], 42)
        self.assertEqual(got["drp"], 7)

    def test_mode_ol_survives_roundtrip(self):
        """Режим контура m — строка, а не число: 'ol' не должен стать 0.0."""
        args = dict(self.ARGS, mode="ol")
        got = P.parse_telemetry(P.format_telemetry(debug=True, **args))
        self.assertEqual(got["m"], "ol")

    def test_integer_fields_are_int(self):
        """Счётчики ec/of/drp — целые: в подписи виджета '7', а не '7.0'."""
        got = P.parse_telemetry(P.format_telemetry(debug=True, **self.ARGS))
        for key in ("ec", "of", "drp"):
            with self.subTest(field=key):
                self.assertIsInstance(got[key], int)

    def test_event_frame_flag(self):
        """Кадр по приходу в цель: ',ev:1' в конце, остальные поля целы."""
        base = P.format_telemetry(debug=True, **self.ARGS)
        got = P.parse_telemetry(base + ",ev:1")
        self.assertEqual(got["ev"], 1)
        self.assertIsInstance(got["ev"], int)
        self.assertEqual(got["drp"], 7)
        self.assertAlmostEqual(got["cp"], 123.46, places=6)
        # У периодического кадра поля ev нет вовсе (main.c: ev = "" без события)
        self.assertNotIn("ev", P.parse_telemetry(base))

    def test_event_frame_short_form(self):
        """Метка ev:1 дописывается и к короткому кадру (debug=0)."""
        got = P.parse_telemetry("cp:90.00,ec:0,ev:1")
        self.assertEqual(got, {"cp": 90.0, "ec": 0, "ev": 1})

    def test_parse_rejects_foreign_lines(self):
        """Не-телеметрия разбору не поддаётся: None, а не полупустой словарь."""
        for line in ("ok:t=90.00", "err:unknown",
                     "mode=STEP_DIR run=600 hold=300 microsteps=256 ready=1",
                     "sync=0 in=0 out=0 n=0", "enc:ok n=16/16 spread=0.000 pos=0.00",
                     "", "  "):
            with self.subTest(line=line):
                self.assertIsNone(P.parse_telemetry(line))

    def test_parse_tolerates_truncated_frame(self):
        """Обрезанный UART-ом кадр отдаёт то, что успело прийти, а не мусор."""
        got = P.parse_telemetry("cp:12.34,tp:20.00,pe:7.6")
        self.assertEqual(got["cp"], 12.34)
        self.assertEqual(got["tp"], 20.0)
        self.assertNotIn("ec", got)


class ClassifyLineTests(unittest.TestCase):
    """Классификация входящих строк — по ней контроллер раздаёт их обработчикам."""

    def test_classification(self):
        cases = (
            ("cp:12.34,ec:0", "telemetry"),
            ("cp:90.00,ec:0,ev:1", "telemetry"),
            ("cp:1.00,tp:2.00,pe:1.00,u:0.0000,m:cl,ec:0,kp:0.0250,ki:0.0000,"
             "kd:0.0000,v:1200.0,a:2000.0,of:0,drp:0", "telemetry"),
            ("mode=STEP_DIR run=600 hold=300 microsteps=256 ready=1", "mcfg"),
            ("sync=2 in=1 out=0 n=17", "sync"),
            ("ok:t=90.00", "reply"),
            ("ok:sync=2", "reply"),          # эхо команды, а не статус sync
            # Ответы на запросы состояния: строка данных без префикса ok:
            ("om=0", "reply"),
            ("om=2", "reply"),
            ("hold=0", "reply"),
            ("hold=1", "reply"),
            ("err:busy stop motor first", "reply"),
            ("enc:ok n=16/16 spread=0.000 pos=12.34", "other"),
            ("err:enc spread=1.234", "reply"),
            ("", "other"),
        )
        for line, expect in cases:
            with self.subTest(line=line):
                self.assertEqual(P.classify_line(line), expect)

    def test_trailing_crlf_does_not_change_class(self):
        """Прошивка шлёт строки с \\r\\n — класс не должен от этого зависеть."""
        self.assertEqual(P.classify_line("cp:1.00,ec:0\r\n"), "telemetry")
        self.assertEqual(P.classify_line("ok:en\r\n"), "reply")

    def test_sync_status_parsed(self):
        got = P.parse_sync("sync=2 in=1 out=0 n=17")
        self.assertEqual(got, {"sync_mode": 2, "sync_in": 1,
                               "sync_out": 0, "sync_edges": 17})

    def test_mcfg_parsed(self):
        got = P.parse_mcfg("mode=STEP_DIR run=600 hold=300 microsteps=256 ready=1")
        self.assertEqual(got, {"mode": "STEP_DIR", "run": 600, "hold": 300,
                               "microsteps": 256, "ready": 1})

    def test_ok_reply_confirms_parameters(self):
        """ok:-эхо подтверждает параметр в состоянии устройства."""
        cases = (("ok:op=50", ("op_ms", 50)), ("ok:sync=1", ("sync_mode", 1)),
                 ("ok:v=120.5", ("v", 120.5)), ("ok:a=0.0", ("a", 0.0)),
                 ("ok:kp=0.0250", ("kp", 0.025)))
        for line, expect in cases:
            with self.subTest(line=line):
                self.assertEqual(P.parse_ok_reply(line), expect)
        # ok:en параметра не несёт, err: — тем более
        self.assertIsNone(P.parse_ok_reply("ok:en"))
        self.assertIsNone(P.parse_ok_reply("err:bad arg (v=1..1200)"))

    def test_state_reply_confirms_parameters(self):
        """Ответ на запрос (om=N / hold=N) кладётся в состояние как и ok:-эхо.

        Формат — из прошивки: main.c CMD_GET_OUTPUT_MODE печатает
        "om=%u\\r\\n", CMD_GET_HOLD — "hold=%u\\r\\n", префикса ok: у них нет.
        Ключи те же, что у ok:om= / ok:hold= в контроллере (output_mode,
        hold), иначе индикатор не обновился бы.
        """
        cases = (("om=0", ("output_mode", 0)), ("om=2", ("output_mode", 2)),
                 ("hold=0", ("hold", 0)), ("hold=1", ("hold", 1)),
                 ("om=1\r\n", ("output_mode", 1)))
        for line, expect in cases:
            with self.subTest(line=line):
                self.assertEqual(P.parse_state_reply(line), expect)
        # Чужие строки состояния не несут: ни эхо установки, ни статус sync,
        # ни значение, которого прошивка выдать не может.
        for line in ("ok:om=1", "ok:hold=0", "sync=0 in=0 out=0 n=0",
                     "om=7", "hold=2", "om=abc", "hold=", "cp:0.00,ec:0"):
            with self.subTest(line=line):
                self.assertIsNone(P.parse_state_reply(line))


class RangeValidatorTests(unittest.TestCase):
    """Клиентские границы аргументов (board.h / cmd_parser.c)."""

    def assert_ok(self, cmd):
        ok, why = P.validate(cmd)
        self.assertTrue(ok, f"{cmd}: отвергнута зря ({why})")

    def assert_bad(self, cmd):
        ok, why = P.validate(cmd)
        self.assertFalse(ok, f"{cmd}: принята зря")
        self.assertTrue(why.strip(), f"{cmd}: причина отказа пустая")

    def test_speed(self):
        for cmd in ("v=1", "v=1200", "v=30.5"):
            self.assert_ok(cmd)
        for cmd in ("v=0", "v=0.99", "v=1200.1", "v=-30", "v=abc", "v="):
            self.assert_bad(cmd)

    def test_accel(self):
        for cmd in ("a=0", "a=2000", "a=100000"):    # 0 = без ограничения
            self.assert_ok(cmd)
        for cmd in ("a=-1", "a=100001", "a=nan"):
            self.assert_bad(cmd)

    def test_output_period(self):
        for cmd in ("op=0", "op=4", "op=65535"):
            self.assert_ok(cmd)
        for cmd in ("op=-1", "op=65536", "op=abc"):
            self.assert_bad(cmd)

    def test_output_mode(self):
        for cmd in ("om=0", "om=1", "om=2"):
            self.assert_ok(cmd)
        for cmd in ("om=3", "om=-1", "om=abc", "om="):
            self.assert_bad(cmd)

    def test_hold(self):
        for cmd in ("hold=0", "hold=1"):
            self.assert_ok(cmd)
        for cmd in ("hold=2", "hold=-1", "hold=on", "hold="):
            self.assert_bad(cmd)

    def test_debug(self):
        self.assert_ok("debug=0")
        self.assert_ok("debug=1")
        for cmd in ("debug=2", "debug=-1", "debug=yes"):
            self.assert_bad(cmd)

    def test_microsteps(self):
        for n in P.MSTEP_VALUES:
            self.assert_ok(f"mstep {n}")
        for cmd in ("mstep 3", "mstep 0", "mstep 512", "mstep -16", "mstep x",
                    # За uint16 парсер прошивки не пускает вовсе (молчание),
                    # так что отправлять такое тем более незачем.
                    f"mstep {P.MSTEP_ARG_MAX + 1}", "mstep 65537",
                    "mstep 66048"):
            self.assert_bad(cmd)

    def test_currents(self):
        for cmd in ("irun 0", "irun 3000", "ihold 300", "icur 600 300"):
            self.assert_ok(cmd)
        for cmd in ("irun 3001", "irun -1", "ihold 5000", "irun abc",
                    "icur 600", "icur 600 300 100", "icur 4000 300"):
            self.assert_bad(cmd)

    def test_sync_mode(self):
        for n in P.SYNC_MODE_VALUES:
            self.assert_ok(f"sync={n}")
        for cmd in ("sync=3", "sync=-1", "sync=x"):
            self.assert_bad(cmd)
        self.assert_ok("sync")          # запрос состояния

    def test_state_queries_pass_validation(self):
        """Запросы без аргумента отправлять можно: прошивка их узнаёт."""
        for cmd in ("sync", "om", "hold", "mcfg", "diag"):
            self.assert_ok(cmd)

    def test_target(self):
        for cmd in ("t=0", "t=90", "t=-30", "t=370.5", "t=+", "t=-"):
            self.assert_ok(cmd)         # t=370.5 и t=-30 прошивка приводит к [0,360)
        for cmd in ("t=", "t=abc", "t=inf", "t=nan"):
            self.assert_bad(cmd)        # inf/nan прошивка отбрасывает по isfinite

    def test_pid_gains(self):
        for cmd in ("kp=0.025", "ki=0", "kd=-0.5"):
            self.assert_ok(cmd)
        for cmd in ("kp=", "ki=abc", "kd=nan"):
            self.assert_bad(cmd)

    def test_scan(self):
        for cmd in ("scan=0,90,5,100", "scan=350,10,5,100",   # сектор через ноль
                    "scan=0,+,5,100", "scan=180,-,2.5,1",
                    "scan=0,90,5,65535"):
            self.assert_ok(cmd)
        for cmd in ("scan=0,90,5",            # мало полей
                    "scan=0,90,0,100",        # шаг 0
                    "scan=0,90,-5,100",       # шаг отрицательный
                    "scan=0,90,5,0",          # пауза 0 - прошивка ответит err:scan
                    "scan=0,90,5,65536",      # delay не помещается в uint16
                    "scan=0,360,5,100",       # вырожденный сектор (та же точка)
                    "scan=10,10,5,100",
                    "scan=a,b,c,d"):
            self.assert_bad(cmd)

    def test_empty_and_unknown(self):
        self.assert_bad("")
        self.assert_bad("   ")
        # Неизвестную команду отправлять можно: плата честно ответит err:unknown
        self.assert_ok("hello")


class ValidatorMatchesModelTests(unittest.TestCase):
    """Валидатор и модель прошивки не должны расходиться.

    Смысл клиентской проверки один: не дать GUI отправить то, на что плата
    промолчит (ответа нет - контроллер зря ждёт таймаут и ругается). Поэтому
    инварианта два: отвергнутое валидатором модель не подтверждает ok:, а
    разрешённое - не проваливается в молчание.
    """

    REJECTED = ("v=0", "v=1200.1", "a=-1", "a=100001", "op=-1", "op=65536",
                "om=3", "om=-1", "hold=2", "debug=2", "sync=3",
                "mstep 3", "mstep 512", "mstep 65537", "mstep 66048",
                "irun 3001", "ihold 5000",
                "icur 600", "icur 4000 300", "t=", "t=abc", "t=inf", "t=nan",
                "scan=0,90,5", "scan=0,90,0,100", "scan=0,90,5,0",
                "scan=0,90,5,65536", "scan=0,360,5,100", "")

    ACCEPTED = ("en", "dis", "stop", "t=+", "t=-", "t=90", "t=-30", "t=370.5",
                "kp=0.03", "ki=0", "kd=0", "v=600", "a=0", "op=0", "op=20",
                "om=0", "om=1", "om=2", "hold=0", "hold=1", "debug=1",
                "sync=0", "sync=1", "sync=2", "irun 600", "ihold 300",
                "icur 600 300", "mstep 256",
                "scan=0,90,5,100", "scan=350,10,5,100", "scan=0,+,5,100")

    def test_rejected_commands_are_never_confirmed(self):
        for cmd in self.REJECTED:
            with self.subTest(cmd=cmd):
                self.assertFalse(P.validate(cmd)[0])
                for line in Sim().send(cmd):
                    self.assertFalse(line.startswith("ok:"),
                                     f"{cmd}: модель подтвердила отвергнутое ({line})")

    def test_accepted_commands_are_answered(self):
        for cmd in self.ACCEPTED:
            with self.subTest(cmd=cmd):
                self.assertTrue(P.validate(cmd)[0])
                rep = Sim().send(cmd)
                self.assertTrue(rep, f"{cmd}: молчание вместо ответа")
                self.assertTrue(rep[0].startswith("ok:"), (cmd, rep))


class CommandBuilderTests(unittest.TestCase):
    """Команда, собранная GUI, должна приниматься парсером прошивки."""

    def build_and_check(self, cmd: str, expect: str):
        ok, why = P.validate(cmd)
        self.assertTrue(ok, f"{cmd}: собственный валидатор отверг команду ({why})")
        rep = Sim().send(cmd)
        self.assertTrue(rep, f"{cmd}: модель промолчала")
        self.assertEqual(rep[0], expect)

    def test_simple_commands(self):
        for cmd, expect in ((P.cmd_enable(), "ok:en"),
                            (P.cmd_disable(), "ok:dis"),
                            (P.cmd_stop(), "ok:stop"),
                            (P.cmd_diag(), "ok:diag")):
            with self.subTest(cmd=cmd):
                self.build_and_check(cmd, expect)

    def test_target_wraps_like_firmware(self):
        """Эхо прошивки — приведённый к [0,360) угол, его и ждёт GUI."""
        for deg, expect in ((90, "ok:t=90.00"), (-30, "ok:t=330.00"),
                            (370, "ok:t=10.00"), (0.5, "ok:t=0.50")):
            with self.subTest(deg=deg):
                self.build_and_check(P.cmd_target(deg), expect)

    def test_jog(self):
        self.build_and_check(P.cmd_jog("+"), "ok:t=+")
        self.build_and_check(P.cmd_jog("-"), "ok:t=-")

    def test_parameters(self):
        cases = ((P.cmd_kp(0.03), "ok:kp=0.0300"),
                 (P.cmd_ki(0.5), "ok:ki=0.5000"),
                 (P.cmd_kd(0), "ok:kd=0.0000"),
                 (P.cmd_speed(120.5), "ok:v=120.5"),
                 (P.cmd_accel(0), "ok:a=0.0"),
                 (P.cmd_op(50), "ok:op=50"),
                 (P.cmd_output_mode(2), "ok:om=2"),
                 (P.cmd_hold(False), "ok:hold=0"),
                 (P.cmd_hold(True), "ok:hold=1"),
                 (P.cmd_debug(True), "ok:debug=1"),
                 (P.cmd_debug(False), "ok:debug=0"),
                 (P.cmd_sync_mode(2), "ok:sync=2"))
        for cmd, expect in cases:
            with self.subTest(cmd=cmd):
                self.build_and_check(cmd, expect)

    def test_driver_commands(self):
        cases = ((P.cmd_irun(800), "ok:irun=800"),
                 (P.cmd_ihold(300), "ok:ihold=300"),
                 (P.cmd_icur(700, 200), "ok:icur=700,200"),
                 (P.cmd_mstep(16), "ok:mstep=16"))
        for cmd, expect in cases:
            with self.subTest(cmd=cmd):
                self.build_and_check(cmd, expect)

    def test_scan_builders(self):
        cases = ((P.cmd_scan_sector(0, 90, 5, 100), "ok:scan=0.00,90.00,5.00,100"),
                 (P.cmd_scan_sector(350, 10, 5, 100), "ok:scan=350.00,10.00,5.00,100"),
                 (P.cmd_scan_sector(-10, 10, 2.5, 20), "ok:scan=350.00,10.00,2.50,20"),
                 (P.cmd_scan_infinite(0, "+", 7.5, 20), "ok:scan=0.00,+,7.50,20"),
                 (P.cmd_scan_infinite(180, "-", 1, 5), "ok:scan=180.00,-,1.00,5"))
        for cmd, expect in cases:
            with self.subTest(cmd=cmd):
                self.build_and_check(cmd, expect)

    def test_query_commands_return_data_lines(self):
        """mcfg, sync, om и hold отвечают строкой данных, а не эхом ok:."""
        sim = Sim()
        mcfg = sim.send(P.cmd_mcfg())
        self.assertEqual(P.classify_line(mcfg[0]), "mcfg")
        self.assertIsNotNone(P.parse_mcfg(mcfg[0]))
        sync = sim.send(P.cmd_sync_query())
        self.assertEqual(P.classify_line(sync[0]), "sync")
        self.assertIsNotNone(P.parse_sync(sync[0]))
        # om и hold: ответ разбирается parse_state_reply и отражает то
        # состояние, которое подтвердила установка (ok:om= / ok:hold=).
        for build, setup, expect in (
                (P.cmd_output_mode_query, P.cmd_output_mode(2), ("output_mode", 2)),
                (P.cmd_hold_query, P.cmd_hold(False), ("hold", 0))):
            with self.subTest(cmd=build()):
                ok, why = P.validate(build())
                self.assertTrue(ok, f"{build()}: валидатор отверг запрос ({why})")
                sim.ok(setup)
                rep = sim.send(build())
                self.assertEqual(len(rep), 1, f"{build()}: ждали одну строку")
                self.assertEqual(P.classify_line(rep[0]), "reply")
                self.assertEqual(P.parse_state_reply(rep[0]), expect)

    def test_fmt_num_is_compact_and_parsable(self):
        """Число в команде — без хвостовых нулей, но читаемое strtof прошивки."""
        for value, expect in ((90.0, "90"), (45.5, "45.5"), (-0.0, "0"),
                              (0.0, "0"), (-12.25, "-12.25"), (1e-6, "0")):
            with self.subTest(value=value):
                self.assertEqual(P.fmt_num(value), expect)


class RingArithmeticTests(unittest.TestCase):
    """Кольцевая координата: та же арифметика, что в wrap360/shortest_path_err."""

    def test_wrap360(self):
        for src, expect in ((0.0, 0.0), (359.9, 359.9), (360.0, 0.0),
                            (370.0, 10.0), (-30.0, 330.0), (-360.0, 0.0),
                            (720.5, 0.5), (-1e-15, 0.0)):   # -1e-15 округляется до 360.0
            with self.subTest(src=src):
                self.assertAlmostEqual(P.wrap360(src), expect, places=6)

    def test_wrap360_range(self):
        """Результат всегда в [0,360): 360.0 из-за округления не допускается."""
        for src in (-1e-15, -1e-9, 359.9999999999, 720.0, -720.0):
            with self.subTest(src=src):
                got = P.wrap360(src)
                self.assertGreaterEqual(got, 0.0)
                self.assertLess(got, 360.0)

    def test_wrap180_shortest_path(self):
        for src, expect in ((0.0, 0.0), (10.0, 10.0), (-10.0, -10.0),
                            (180.0, 180.0), (-180.0, 180.0), (181.0, -179.0),
                            (350.0, -10.0), (-350.0, 10.0), (540.0, 180.0)):
            with self.subTest(src=src):
                self.assertAlmostEqual(P.wrap180(src), expect, places=6)

    def test_wrap180_range(self):
        """Ошибка по кратчайшему пути лежит в (-180, 180] при любом аргументе."""
        for k in range(-720, 721, 7):
            with self.subTest(deg=k):
                got = P.wrap180(float(k))
                self.assertGreater(got, -180.0)
                self.assertLessEqual(got, 180.0)

    def test_scan_span(self):
        """Протяжённость сектора считается по кольцу от start вверх."""
        self.assertAlmostEqual(P.scan_span(0, 90), 90.0, places=6)
        self.assertAlmostEqual(P.scan_span(350, 10), 20.0, places=6)
        self.assertAlmostEqual(P.scan_span(10, 350), 340.0, places=6)
        self.assertEqual(P.scan_span(0, 360), 0.0)      # вырожденный - err:scan
        self.assertEqual(P.scan_span(45, 45), 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
