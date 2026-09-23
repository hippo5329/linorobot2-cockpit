"""`base_controller.console: uart0` moves an ESP32-S3's console to UART0 at boot.

The S3 images are built with CDC-on-boot, so `Serial` is the native USB. A
board whose only USB is a bridge on UART0 (Yahboom YB-EET01) needs the same
image to talk on Serial0 -- an env key, not a build variant.
"""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(ROOT, "firmware")
HDR = os.path.join(FW, "common", "lib", "mcu_env", "lino_console.h")
CPP = os.path.join(FW, "common", "lib", "mcu_env", "lino_console.cpp")


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def test_the_wrapper_exists_only_where_there_is_a_choice():
    h = _read(HDR)
    assert "#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT" in h
    assert "#define Serial lino_console" in h
    assert "#define LINO_CONSOLE_SELECTABLE 0" in h, "boards without CDC-on-boot define nothing"


def test_the_wrapper_forwards_to_serial0_or_the_native_port():
    c = _read(CPP)
    assert "#undef Serial" in c, "the .cpp must see the real objects"
    for fn in ("begin", "end", "setRxBufferSize", "setTxBufferSize"):
        body = c[c.index(f"LinoConsole::{fn}("):]
        body = body[:body.index("}")]
        assert "Serial0." in body and "Serial." in body, fn
    assert 'envGet("console", "usb")' in c
    h = _read(HDR)
    assert "operator bool() { return uart0_ ? true : (bool)Serial; }" in h, "a bridge never waits for a host"


def test_every_translation_unit_that_prints_reaches_the_wrapper():
    # A TU that includes neither mcu_env.h nor lino_console.h keeps the real
    # `Serial`, and its lines vanish on a UART0 board.
    missing = []
    for dirpath, _, files in os.walk(FW):
        if ".pio" in dirpath or os.sep + "examples" in dirpath:
            continue
        for f in files:
            if not f.endswith((".cpp", ".h", ".ino")):
                continue
            p = os.path.join(dirpath, f)
            src = _read(p)
            if "Serial." not in src or f in ("lino_console.h", "lino_console.cpp"):
                continue
            # a header that includes another of ours is covered transitively
            if re.search(r'#include\s+"(mcu_env|lino_console|imu_interface|mag_interface|default_imu|fake_ld19)\.h"', src):
                continue
            missing.append(os.path.relpath(p, FW))
    assert not missing, f"print without the console wrapper: {missing}"


def test_setup_selects_the_console_before_the_first_print():
    m = _read(os.path.join(FW, "src", "main.cpp"))
    code = re.sub(r"//[^\n]*", "", m)          # comments mention Serial too
    sel = code.index("lino_console.selectFromEnv();")
    setup = code.index("void setup()")
    first_print = code.index("Serial.", setup)
    assert setup < sel < first_print


def test_the_env_carries_the_key_and_refuses_nonsense():
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import mcu_env
    base = {"robot": {"name": "x"}, "base_controller": {"name": "x", "mcu": "esp32s3", "transport": "serial",
            "baudrate": 921600, "console": "uart0", "pins": {}, "sensors": {}}}
    env = mcu_env.hardware_env(base)
    assert env["console"] == "uart0"
    base["base_controller"]["console"] = "usb"
    assert mcu_env.hardware_env(base)["console"] == "usb"
    del base["base_controller"]["console"]
    assert "console" not in mcu_env.hardware_env(base)
    base["base_controller"]["console"] = "uart9"
    try:
        mcu_env.hardware_env(base)
    except ValueError as e:
        assert "console" in str(e)
    else:
        raise AssertionError("uart9 accepted")


def test_the_yahboom_reference_and_preset_use_uart0():
    import yaml
    with open(os.path.join(ROOT, "config", "reference", "yb_eet01_config.yaml")) as fh:
        d = yaml.safe_load(fh)
    assert d["base_controller"]["console"] == "uart0"
    js = _read(os.path.join(ROOT, "web", "frontend", "app-presets.js"))
    blk = js[js.index('id: "yb_eet01"'):]
    blk = blk[:blk.index("id: \"crawler_esp32s3\"")]
    assert 'console: "uart0"' in blk


def test_the_backend_saves_the_key_and_only_its_two_spellings():
    src = _read(os.path.join(ROOT, "web", "backend", "routes_config.py"))
    blk = src[src.index('if "console" in data:'):]
    blk = blk[:blk.index('if "mcu" in data:')]
    assert 'ctrl["console"] = console' in blk
    assert '("usb", "uart0")' in blk and "400" in blk
