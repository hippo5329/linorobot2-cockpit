"""The IMU's DATA_RDY line, end to end, and the Yahboom board as a reference design.

`pins.imu.int` -> env key `imu_int` -> `IMU_INT_PIN` in the header -> read by
the firmware -> a row in the wiring table -> a pin the inspector knows about.
Default -1 everywhere: a config that predates the key polls exactly as before.
"""
import os
import re
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import mcu_env  # noqa: E402
import pin_catalog  # noqa: E402
import gen_wiring_table  # noqa: E402

YAHBOOM = os.path.join(REPO_ROOT, "config", "reference", "yahboom_esp32s3_config.yaml")
MAIN = os.path.join(REPO_ROOT, "firmware", "src", "main.cpp")
IFACE = os.path.join(REPO_ROOT, "firmware", "common", "lib", "imu", "imu_interface.h")
ISR_CPP = os.path.join(REPO_ROOT, "firmware", "common", "lib", "imu", "imu_interface.cpp")
GEN = os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py")


def params(pins_imu=None, **sensors):
    p = {"robot": {"name": "t"},
         "base_controller": {"name": "esp32s3", "mcu": "esp32s3",
                             "sensors": sensors or {"imu": "auto"},
                             "pins": {"i2c": {"sda": 40, "scl": 39}}}}
    if pins_imu is not None:
        p["base_controller"]["pins"]["imu"] = pins_imu
    return p


def test_env_key_defaults_to_polling():
    assert int(mcu_env.hardware_env(params())["imu_int"]) == -1


def test_env_key_carries_the_pin():
    assert int(mcu_env.hardware_env(params({"int": 41}))["imu_int"]) == 41


def test_firmware_reads_the_key_and_the_header_defines_the_macro():
    main = open(MAIN, encoding="utf-8").read()
    assert 'envInt("imu_int", IMU_INT_PIN)' in main
    assert re.search(r"#define IMU_INT_PIN -1", main), "main.cpp needs a -1 fallback for a header without the macro"
    assert "IMU_INT_PIN" in open(GEN, encoding="utf-8").read()


def test_getdata_polls_when_the_line_never_fires():
    """A driver that cannot enable DATA_RDY, on a pin that stays quiet, must not
    return the same sample forever."""
    src = open(IFACE, encoding="utf-8").read()
    assert "poll_fallback_" in src and "never fired" in src
    assert "IMU_INT_STALE_MS" in src, "a line that stops firing needs a staleness ceiling"
    assert "virtual bool enableDataReadyInterrupt() { return false; }" in src


def test_the_isr_only_sets_a_flag():
    # Declared in the header, defined out of line: an IRAM_ATTR function that
    # is inline lands in a COMDAT section whose literal pool the Xtensa linker
    # places after the code, and every ESP32 image fails to link.
    hdr = open(IFACE, encoding="utf-8").read()
    assert re.search(r"static void IMU_ISR_ATTR dataReadyISR\(\);", hdr), "declare it; define it in the .cpp"
    src = open(ISR_CPP, encoding="utf-8").read()
    isr = re.search(r"IMUInterface::dataReadyISR\(\)\s*\{(.*?)\}", src, re.S).group(1)
    # "data_ready_" is the flag it sets; what must NOT be there is bus traffic.
    for forbidden in ("Wire", "readGyroscope", "readAccelerometer", "I2Cdev", "getData"):
        assert forbidden not in isr, f"{forbidden} inside the ISR: no I2C from an ISR"
    assert "data_ready_ = true" in isr


def test_inspector_sees_the_pin_and_a_collision():
    p = params({"int": 41})
    roles = {r for r, g, d in pin_catalog._collect(p["base_controller"])}
    assert "imu.int" in roles
    p["base_controller"]["pins"]["i2c"]["scl"] = 41            # same GPIO twice
    findings = pin_catalog.check_config(p)
    assert any("41" in str(f) for f in findings), "the inspector must flag GPIO 41 used twice"


def test_wiring_table_lists_the_line():
    rows = gen_wiring_table.rows_for(params({"int": 41})) if hasattr(gen_wiring_table, "rows_for") else None
    if rows is None:
        src = open(os.path.join(REPO_ROOT, "scripts", "gen_wiring_table.py"), encoding="utf-8").read()
        assert "IMU interrupt" in src
    else:
        assert any("IMU interrupt" in r[0] for r in rows)


def test_yahboom_reference_loads_and_is_the_board():
    with open(YAHBOOM) as fh:
        d = yaml.safe_load(fh)
    bc = d["base_controller"]
    assert d["robot"]["name"] == "yahboom_esp32s3" == bc["name"]
    assert bc["mcu"] == "esp32s3" and bc["driver_type"] == "BTS7960"
    pins = bc["pins"]
    # the board's own pinout
    assert (pins["motor1"]["in_a"], pins["motor1"]["in_b"], pins["motor1"]["pwm"]) == (4, 5, -1)
    assert (pins["motor2"]["in_a"], pins["motor2"]["in_b"], pins["motor2"]["pwm"]) == (15, 16, -1)
    assert (pins["encoder1"]["pin_a"], pins["encoder1"]["pin_b"]) == (6, 7)
    assert (pins["encoder2"]["pin_a"], pins["encoder2"]["pin_b"]) == (47, 48)
    assert (pins["i2c"]["sda"], pins["i2c"]["scl"], pins["imu"]["int"]) == (40, 39, 41)
    assert pins["led"] == 45 and pins["battery"]["pin"] == 3
    assert str(bc["sensors"]["imu"]).lower() == "icm42670", "the V2.0 board answers 0x68 / WHO_AM_I 0x67: ICM-42670-P"
    assert int(mcu_env.hardware_env(d)["imu_int"]) == 41


def test_yahboom_reference_passes_the_inspector_with_only_the_expected_warnings():
    """GPIO 45 (the board's status LED) is a strapping pin; the inspector should
    warn about it and raise nothing else. GPIO 3 is a strapping pin too but is
    used as an ADC input, which the inspector does not flag."""
    with open(YAHBOOM) as fh:
        d = yaml.safe_load(fh)
    findings = pin_catalog.check_config(d)
    text = " | ".join(str(f) for f in findings)
    assert all(str(f).startswith("('warn'") or "'warn'" in str(f) for f in findings), text
    assert any("45" in str(f) and "strapping" in str(f) for f in findings), text
    assert len(findings) == 1, f"expected only the LED-45 warning, got: {text}"
