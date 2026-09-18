"""Настоящий парсер прошивки, собранный и запущенный, — а не пересказанный.

Остальные тесты привязаны к прошивке статически: test_firmware_commands.py и
test_board_constants.py читают cmd_parser.h/.c регулярными выражениями. Такая
привязка не исполняет код и потому не видит подмену сравнения: мутация
`strcmp(line, "om")` → `strncmp(line, "om", 2)` проходит мимо всех регулярок,
хотя после неё плата отвечает «om=0» на строки «omx» и «omm».

Здесь Software/lib/sonar_proto/src/cmd_parser.c СОБИРАЕТСЯ хостовым gcc
(tools/run_cmd_parser_probe.py, по образцу
FW_AS5047P_STM32F103C8/tools/run_filter_model.sh) и ПРОГОНЯЕТСЯ на корпусе
строк. Сверяется он с двумя источниками правды, которые есть в репозитории:

* sonar_gui.simulator.FirmwareSimulator — модель прошивки. Соответствие
  трёхзначное и полное: парсер отверг строку (rc=0) ⟺ модель молчит; парсер
  вернул CMD_UNKNOWN ⟺ модель отвечает err:unknown; парсер узнал команду ⟺
  модель отвечает по существу. Плюс числа: если модель ответила эхом ok:, в
  нём обязано стоять ровно то значение, которое вынул настоящий парсер.
* sonar_gui.protocol — то, что отправляет GUI. Каждый построитель обязан
  собирать строку, которую парсер узнаёт нужным типом и с нужными
  аргументами, а константы границ (CURRENT_MAX, OP_MAX, MSTEP_ARG_MAX, …)
  обязаны совпадать с поведением парсера НА границе: значение принимается,
  значение+1 — нет.

Без компилятора C тест пропускается: gcc не входит в требования GUI, и его
отсутствие на машине разработчика не должно выглядеть как поломка протокола.
В CI шаг «Хостовый прогон парсера прошивки» гоняет зонд отдельно и там
отсутствие gcc обязано быть ошибкой, а не пропуском.
"""
from __future__ import annotations

import importlib.util
import os
import shutil
import struct
import sys
import tempfile
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import _support     # noqa: F401,E402 - ставит sonar_gui на путь импорта
from _support import P, REPO_ROOT, Sim                 # noqa: E402

RUNNER_REL = os.path.join("Software", "lib", "sonar_proto", "tools",
                          "run_cmd_parser_probe.py")
RUNNER_PATH = os.path.join(REPO_ROOT, RUNNER_REL)


def _load_runner():
    """Модуль раннера по пути (пакетом он не является и ставить его некуда)."""
    spec = importlib.util.spec_from_file_location("run_cmd_parser_probe",
                                                  RUNNER_PATH)
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _f32(value: float) -> float:
    """Округление до float — в Cmd_Result числа лежат во float, не в double."""
    return struct.unpack("<f", struct.pack("<f", value))[0]


# ── Команды, которые парсер обязан узнавать ТОЛЬКО целиком (strcmp) ─────────
# Здесь и ломается protocol: «om» с любым хвостом — это не запрос режима
# телеметрии, а неизвестная команда, и плата обязана ответить err:unknown.
EXACT_LITERALS = {
    "en": "CMD_ENABLE",
    "dis": "CMD_DISABLE",
    "stop": "CMD_STOP",
    "t=+": "CMD_CONTINUOUS",
    "t=-": "CMD_CONTINUOUS",
    "sync": "CMD_GET_SYNC",
    "om": "CMD_GET_OUTPUT_MODE",
    "hold": "CMD_GET_HOLD",
    "mcfg": "CMD_GET_MCFG",
    "diag": "CMD_DIAG",
}

# Чем «портим» строку команды: хвост, ведущий пробел, другой регистр.
SUFFIXES = ("x", "m", " ", "0", "1", "=")

# ── Построители protocol.py → тип и аргументы, которые обязан вынуть парсер ──
BUILT = (
    (P.cmd_enable(),                    "CMD_ENABLE",            {}),
    (P.cmd_disable(),                   "CMD_DISABLE",           {}),
    (P.cmd_stop(),                      "CMD_STOP",              {}),
    (P.cmd_target(90),                  "CMD_SET_TARGET",        {"target": 90.0}),
    (P.cmd_target(-30),                 "CMD_SET_TARGET",        {"target": -30.0}),
    (P.cmd_target(45.5),                "CMD_SET_TARGET",        {"target": 45.5}),
    (P.cmd_jog("+"),                    "CMD_CONTINUOUS",        {"dir": 1}),
    (P.cmd_jog("-"),                    "CMD_CONTINUOUS",        {"dir": -1}),
    (P.cmd_kp(0.025),                   "CMD_SET_KP",            {"kp": 0.025}),
    (P.cmd_ki(0.5),                     "CMD_SET_KI",            {"ki": 0.5}),
    (P.cmd_kd(0.1),                     "CMD_SET_KD",            {"kd": 0.1}),
    (P.cmd_speed(600),                  "CMD_SET_VMAX",          {"vmax": 600.0}),
    (P.cmd_accel(2000),                 "CMD_SET_ACCEL",         {"accel": 2000.0}),
    (P.cmd_op(20),                      "CMD_SET_OUTPUT_PERIOD", {"op": 20}),
    (P.cmd_output_mode(1),              "CMD_SET_OUTPUT_MODE",   {"om": 1}),
    (P.cmd_debug(True),                 "CMD_SET_DEBUG",         {"debug": 1}),
    (P.cmd_debug(False),                "CMD_SET_DEBUG",         {"debug": 0}),
    (P.cmd_scan_sector(0, 90, 5, 100),  "CMD_SCAN",
     {"start": 0.0, "end": 90.0, "step": 5.0, "delay": 100, "inf": 0}),
    (P.cmd_scan_infinite(0, "+", 5, 100), "CMD_SCAN",
     {"start": 0.0, "end": 0.0, "step": 5.0, "delay": 100, "inf": 1}),
    (P.cmd_scan_infinite(30, "-", 2.5, 250), "CMD_SCAN",
     {"start": 30.0, "end": 0.0, "step": 2.5, "delay": 250, "inf": -1}),
    (P.cmd_irun(800),                   "CMD_SET_IRUN",          {"irun": 800}),
    (P.cmd_ihold(300),                  "CMD_SET_IHOLD",         {"ihold": 300}),
    (P.cmd_icur(700, 200),              "CMD_SET_ICUR",  {"irun": 700, "ihold": 200}),
    (P.cmd_mstep(16),                   "CMD_SET_MSTEP",         {"mstep": 16}),
    (P.cmd_mcfg(),                      "CMD_GET_MCFG",          {}),
    (P.cmd_diag(),                      "CMD_DIAG",              {}),
    (P.cmd_sync_mode(2),                "CMD_SET_SYNC",          {"sync": 2}),
    (P.cmd_sync_query(),                "CMD_GET_SYNC",          {}),
    (P.cmd_hold(False),                 "CMD_SET_HOLD",          {"hold": 0}),
    (P.cmd_hold(True),                  "CMD_SET_HOLD",          {"hold": 1}),
    (P.cmd_output_mode_query(),         "CMD_GET_OUTPUT_MODE",   {}),
    (P.cmd_hold_query(),                "CMD_GET_HOLD",          {}),
)

# ── Границы аргументов: шаблон, предел из protocol.py, тип и поле ──────────
# Исполняемый вариант test_firmware_commands.test_parser_ranges_match_protocol_
# constants: там границы вычитываются из текста cmd_parser.c регулярками, здесь
# парсер спрашивают напрямую — предел принимается, предел+1 отвергается.
LIMITS = (
    ("irun {}",        P.CURRENT_MAX, "CMD_SET_IRUN",          "irun",  "CURRENT_MAX"),
    ("ihold {}",       P.CURRENT_MAX, "CMD_SET_IHOLD",         "ihold", "CURRENT_MAX"),
    ("icur {} 0",      P.CURRENT_MAX, "CMD_SET_ICUR",          "irun",  "CURRENT_MAX"),
    ("icur 0 {}",      P.CURRENT_MAX, "CMD_SET_ICUR",          "ihold", "CURRENT_MAX"),
    ("op={}",          P.OP_MAX,      "CMD_SET_OUTPUT_PERIOD", "op",    "OP_MAX"),
    ("om={}", max(P.OUTPUT_MODE_VALUES), "CMD_SET_OUTPUT_MODE", "om",
     "OUTPUT_MODE_VALUES"),
    ("sync={}", max(P.SYNC_MODE_VALUES), "CMD_SET_SYNC", "sync",
     "SYNC_MODE_VALUES"),
    ("scan=0,90,5,{}", P.SCAN_DELAY_MAX, "CMD_SCAN",           "delay", "SCAN_DELAY_MAX"),
    ("mstep {}",   P.MSTEP_ARG_MAX,   "CMD_SET_MSTEP",         "mstep", "MSTEP_ARG_MAX"),
    # hold= и debug= принимают ровно два значения, границей это не назовёшь,
    # но проверяется так же: 1 принимается, 2 — нет.
    ("hold={}",  1, "CMD_SET_HOLD",  "hold",  "0|1"),
    ("debug={}", 1, "CMD_SET_DEBUG", "debug", "0|1"),
)


def _echo_scan(res) -> str:
    """Эхо прошивки на scan= из полей, которые вынул парсер (main.c CMD_SCAN)."""
    start = P.wrap360(res.num("start"))
    step, delay = res.num("step"), int(res.num("delay"))
    if int(res.num("inf")):
        sign = "+" if res.num("inf") > 0 else "-"
        return f"ok:scan={start:.2f},{sign},{step:.2f},{delay}"
    span = P.wrap360(res.num("end") - res.num("start"))
    return f"ok:scan={start:.2f},{P.wrap360(start + span):.2f},{step:.2f},{delay}"


# Как прошивка подтверждает команду (main.c). Ключ — Cmd_Type, значение —
# функция от разбора: точная строка ok:, которую обязана выдать модель.
ECHO = {
    "CMD_ENABLE":            lambda r: "ok:en",
    "CMD_DISABLE":           lambda r: "ok:dis",
    "CMD_STOP":              lambda r: "ok:stop",
    "CMD_CONTINUOUS":        lambda r: "ok:t=" + ("+" if r.num("dir") > 0 else "-"),
    "CMD_SET_TARGET":        lambda r: f"ok:t={P.wrap360(r.num('target')):.2f}",
    "CMD_SET_KP":            lambda r: f"ok:kp={r.num('kp'):.4f}",
    "CMD_SET_KI":            lambda r: f"ok:ki={r.num('ki'):.4f}",
    "CMD_SET_KD":            lambda r: f"ok:kd={r.num('kd'):.4f}",
    "CMD_SET_VMAX":          lambda r: f"ok:v={r.num('vmax'):.1f}",
    "CMD_SET_ACCEL":         lambda r: f"ok:a={r.num('accel'):.1f}",
    "CMD_SET_OUTPUT_PERIOD": lambda r: f"ok:op={int(r.num('op'))}",
    "CMD_SET_OUTPUT_MODE":   lambda r: f"ok:om={int(r.num('om'))}",
    "CMD_SET_DEBUG":         lambda r: f"ok:debug={int(r.num('debug'))}",
    "CMD_SET_SYNC":          lambda r: f"ok:sync={int(r.num('sync'))}",
    "CMD_SET_HOLD":          lambda r: f"ok:hold={int(r.num('hold'))}",
    "CMD_SET_IRUN":          lambda r: f"ok:irun={int(r.num('irun'))}",
    "CMD_SET_IHOLD":         lambda r: f"ok:ihold={int(r.num('ihold'))}",
    "CMD_SET_ICUR":  lambda r: f"ok:icur={int(r.num('irun'))},{int(r.num('ihold'))}",
    "CMD_SET_MSTEP":         lambda r: f"ok:mstep={int(r.num('mstep'))}",
    "CMD_SCAN":              _echo_scan,
    "CMD_DIAG":              lambda r: "ok:diag",
}

# Запросы состояния отвечают строкой данных без префикса ok: (main.c).
STATE_REPLY_PREFIX = {
    "CMD_GET_MCFG":        "mode=",
    "CMD_GET_SYNC":        "sync=",
    "CMD_GET_OUTPUT_MODE": "om=",
    "CMD_GET_HOLD":        "hold=",
}


class HostProbeAvailabilityTests(unittest.TestCase):
    """Раннер обязан лежать в репозитории — его отсутствие не «пропуск»."""

    def test_runner_is_present(self):
        self.assertTrue(
            os.path.isfile(RUNNER_PATH),
            "нет раннера хостового зонда: " + RUNNER_REL +
            ". Если он переехал, поправьте RUNNER_REL в этом тесте.")

    def test_runner_imports(self):
        module = _load_runner()
        self.assertIsNotNone(module, "раннер " + RUNNER_REL + " не импортируется")
        for name in ("build", "run", "probe", "find_cc", "CORPUS", "CFLAGS"):
            self.assertTrue(hasattr(module, name),
                            f"в раннере нет {name}() — тест сверять нечем")

    def test_probe_source_is_present(self):
        module = _load_runner()
        self.assertIsNotNone(module)
        for path in (module.SRC_PARSER, module.SRC_PROBE):
            self.assertTrue(os.path.isfile(path), "нет исходника: " + path)


class HostParserTests(unittest.TestCase):
    """Разбор, сделанный настоящим Cmd_Parse(), против модели и protocol.py."""

    tmp_dir = None

    @classmethod
    def setUpClass(cls):
        cls.runner = _load_runner()
        if cls.runner is None:
            raise unittest.SkipTest("раннер " + RUNNER_REL + " не импортируется")
        if cls.runner.find_cc() is None:
            raise unittest.SkipTest(
                "компилятор C не найден (искали " +
                ", ".join(cls.runner.CC_CANDIDATES) + " в PATH и $CC) — "
                "настоящий cmd_parser.c собрать нечем. Поставьте gcc либо "
                "задайте CC; в CI этот прогон идёт отдельным шагом и там "
                "пропуска быть не должно.")

        cls.tmp_dir = tempfile.mkdtemp(prefix="cmd_parser_probe-test-")
        exe = cls.runner.build(cls.tmp_dir)

        # Все строки — одним прогоном: зонд читает stdin до конца, и запускать
        # его по разу на строку незачем.
        lines = list(cls.runner.CORPUS)
        lines += [cmd for cmd, _, _ in BUILT]
        for literal in EXACT_LITERALS:
            lines.append(literal)
            lines += [literal + s for s in SUFFIXES]
            lines.append(" " + literal)
            lines.append(literal.upper())
        for tpl, limit, _type, _field, _name in LIMITS:
            lines += [tpl.format(limit), tpl.format(limit + 1), tpl.format(-1)]
        ordered = list(dict.fromkeys(lines))    # без повторов, порядок сохранён
        cls.results = {r.text: r for r in cls.runner.run(exe, ordered)}

    @classmethod
    def tearDownClass(cls):
        if cls.tmp_dir:
            shutil.rmtree(cls.tmp_dir, ignore_errors=True)

    def parse(self, line: str):
        """Разбор строки настоящим парсером (из прогона, сделанного в setUpClass)."""
        self.assertIn(line, self.results, f"строка {line!r} в прогон не попала")
        return self.results[line]

    # ── Соответствие модели прошивки ───────────────────────────────────────
    def test_acceptance_matches_model(self):
        """Принял/отверг/не распознал — у настоящего парсера и у модели одно.

        Именно эта проверка ловит подмену strcmp на strncmp: «omx» настоящий
        парсер обязан отдать как CMD_UNKNOWN, и модель обязана ответить
        err:unknown — расхождение любой стороны валит тест.
        """
        for line in self.results:
            with self.subTest(line=line):
                res = self.parse(line)
                reply = Sim().send(line)
                if res.silent:
                    self.assertEqual(
                        reply, [],
                        f"парсер отверг {line!r} (rc=0) — плата промолчит, "
                        f"а модель ответила {reply}")
                elif res.unknown:
                    self.assertEqual(
                        reply, ["err:unknown"],
                        f"парсер вернул CMD_UNKNOWN на {line!r}, "
                        f"а модель ответила {reply}")
                else:
                    self.assertTrue(
                        reply, f"парсер узнал {line!r} как {res.type}, "
                               f"а модель промолчала")
                    self.assertNotEqual(
                        reply[0], "err:unknown",
                        f"парсер узнал {line!r} как {res.type}, "
                        f"а модель считает команду неизвестной")

    def test_values_match_model_echo(self):
        """Число в эхе модели — ровно то, которое вынул настоящий парсер."""
        for line in self.results:
            res = self.parse(line)
            if not res.accepted or res.type not in ECHO:
                continue
            reply = Sim().send(line)
            if not reply or not reply[0].startswith("ok:"):
                # err:bad arg / err:scan / err:busy — обработчик прошивки
                # отказал уже после разбора, сверять эхо не с чем.
                continue
            with self.subTest(line=line):
                self.assertEqual(
                    reply[0], ECHO[res.type](res),
                    f"модель подтвердила {line!r} как {reply[0]!r}, а парсер "
                    f"вынул {res}")

    def test_state_queries_answer_without_ok(self):
        """Запросы sync/om/hold/mcfg модель отвечает строкой данных, не ok:."""
        for line, cmd_type in (("sync", "CMD_GET_SYNC"), ("om", "CMD_GET_OUTPUT_MODE"),
                               ("hold", "CMD_GET_HOLD"), ("mcfg", "CMD_GET_MCFG")):
            with self.subTest(line=line):
                self.assertEqual(self.parse(line).type, cmd_type)
                reply = Sim().send(line)
                self.assertTrue(reply, f"модель промолчала на {line!r}")
                self.assertTrue(
                    reply[0].startswith(STATE_REPLY_PREFIX[cmd_type]),
                    f"на {line!r} модель ответила {reply[0]!r}, ожидалось "
                    f"начало {STATE_REPLY_PREFIX[cmd_type]!r}")

    # ── Команды, узнаваемые только целиком ─────────────────────────────────
    def test_exact_commands_reject_any_tail(self):
        """«om», «hold», «sync», «en» … — только точное совпадение (strcmp).

        Подмена strcmp на strncmp здесь и ловится: «omx» стал бы запросом
        режима телеметрии, и плата ответила бы «om=0» вместо err:unknown.
        """
        for literal, cmd_type in EXACT_LITERALS.items():
            with self.subTest(literal=literal):
                self.assertEqual(
                    self.parse(literal).type, cmd_type,
                    f"парсер перестал узнавать команду {literal!r}")
            for variant in ([literal + s for s in SUFFIXES] +
                            [" " + literal, literal.upper()]):
                if variant == literal:
                    continue
                with self.subTest(literal=literal, variant=variant):
                    res = self.parse(variant)
                    self.assertNotEqual(
                        res.type, cmd_type,
                        f"{variant!r} разобрано как {cmd_type}: команда "
                        f"{literal!r} узнаётся не целиком (strcmp → strncmp?)")

    # ── Сверка с protocol.py ───────────────────────────────────────────────
    def test_protocol_builders_are_parsed_as_intended(self):
        """Что собрал GUI, то настоящий парсер узнаёт нужным типом и значением."""
        for cmd, cmd_type, fields in BUILT:
            with self.subTest(cmd=cmd):
                res = self.parse(cmd)
                self.assertTrue(res.accepted,
                                f"парсер отверг команду GUI {cmd!r} — плата промолчит")
                self.assertEqual(res.type, cmd_type,
                                 f"{cmd!r} разобрано как {res.type}")
                self.assertEqual(
                    set(res.fields), set(fields),
                    f"{cmd!r}: зонд напечатал поля {sorted(res.fields)}, "
                    f"тест ждал {sorted(fields)}")
                for key, expect in fields.items():
                    self.assertEqual(
                        _f32(res.num(key)), _f32(expect),
                        f"{cmd!r}: поле {key} = {res.fields[key]}, "
                        f"ожидалось {expect}")

    def test_protocol_builders_pass_own_validation(self):
        """Собственный валидатор GUI не блокирует то, что парсер принимает."""
        for cmd, _cmd_type, _fields in BUILT:
            with self.subTest(cmd=cmd):
                ok, why = P.validate(cmd)
                self.assertTrue(ok, f"валидатор protocol.py отверг {cmd!r}: {why}")

    def test_parser_limits_match_protocol_constants(self):
        """Граница из protocol.py принимается парсером, граница+1 — нет."""
        for tpl, limit, cmd_type, field, name in LIMITS:
            at = tpl.format(limit)
            over = tpl.format(limit + 1)
            under = tpl.format(-1)
            with self.subTest(cmd=at):
                res = self.parse(at)
                self.assertTrue(
                    res.accepted and res.type == cmd_type,
                    f"парсер не принял {at!r} (protocol.{name} = {limit}); "
                    f"разбор: {res}")
                self.assertEqual(
                    int(res.num(field)), limit,
                    f"{at!r}: парсер вынул {field}={res.fields[field]}")
            with self.subTest(cmd=over):
                self.assertTrue(
                    self.parse(over).silent,
                    f"парсер принял {over!r}, хотя protocol.{name} = {limit}: "
                    f"граница в прошивке и в GUI разошлись")
            with self.subTest(cmd=under):
                self.assertTrue(
                    self.parse(under).silent,
                    f"парсер принял {under!r} — отрицательный аргумент")

    # ── Края, названные в разборе дефекта ──────────────────────────────────
    def test_edge_cases_are_rejected(self):
        """Строки, на которые плата обязана промолчать (Cmd_Parse вернул 0)."""
        for line in ("t=", "hold=", "om=", "op=", "sync=", "debug=", "scan=",
                     "mstep ", "irun ", "icur 700", "t=++", "t=inf", "t=nan",
                     "t=1e40", "mstep 65536", "mstep 65537", ""):
            with self.subTest(line=line):
                self.assertTrue(
                    self.parse(line).silent,
                    f"парсер принял {line!r}: {self.parse(line)}")

    def test_edge_cases_are_unknown(self):
        """Строки, на которые плата обязана ответить err:unknown."""
        for line in ("omx", "omm", "OM", " om", "om ", "holdx", "syncx",
                     "en ", "ens", "icur", "mstep", "   ", "\t", "x" * 300):
            with self.subTest(line=line):
                res = self.parse(line)
                self.assertTrue(res.unknown,
                                f"{line!r} разобрано как {res.type}, "
                                f"ожидалось CMD_UNKNOWN")

    def test_edge_cases_are_accepted(self):
        """Строки, которые парсер обязан принять (проверка значения — дальше).

        «mstep 3» и «mstep 0» здесь не случайны: набор 1/2/4…256 проверяет
        обработчик прошивки (err:bad arg), а не Cmd_Parse, и путать две
        границы нельзя — на этом и стоит protocol.MSTEP_ARG_MAX.
        """
        for line, cmd_type in (("ihold 300", "CMD_SET_IHOLD"),
                               ("mstep 0", "CMD_SET_MSTEP"),
                               ("mstep 3", "CMD_SET_MSTEP"),
                               ("mstep 256", "CMD_SET_MSTEP"),
                               ("mstep 65535", "CMD_SET_MSTEP"),
                               ("t=0x10", "CMD_SET_TARGET"),
                               ("op=0x10", "CMD_SET_OUTPUT_PERIOD"),
                               ("om= 1", "CMD_SET_OUTPUT_MODE"),
                               ("icur 700\t200", "CMD_SET_ICUR")):
            with self.subTest(line=line):
                res = self.parse(line)
                self.assertEqual(res.type, cmd_type, f"{line!r}: {res}")

    def test_hex_float_argument(self):
        """t=0x10 прошивка выполняет как 16° — strtof читает и hex-форму."""
        self.assertEqual(_f32(self.parse("t=0x10").num("target")), 16.0)
        # strtol вызывается с основанием 10, поэтому у целых аргументов формы
        # 0x нет: «op=0x10» — это op=0, а не op=16.
        self.assertEqual(int(self.parse("op=0x10").num("op")), 0)

    def test_long_line_is_not_truncated_into_a_command(self):
        """Очень длинная строка остаётся неизвестной командой, а не обрезком."""
        res = self.parse("x" * 300)
        self.assertTrue(res.unknown)
        self.assertEqual(len(res.text), 300, "зонд обрезал длинную строку")
        # «t=» с 300 девятками выходит за float — парсер обязан отвергнуть
        # строку целиком (isfinite в parse_float), а не подставить inf.
        self.assertTrue(self.parse("t=" + "9" * 300).silent)


if __name__ == "__main__":
    unittest.main(verbosity=2)
