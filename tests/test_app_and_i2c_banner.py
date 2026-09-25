"""Two diagnostics that reported the wrong thing, found by sweeping every app.

Both are the same class of fault as the rest of the env work: the code that
*acts* reads the env partition, while the code that *reports* read the compile
-time macro, so a prebuilt image told the truth with its behaviour and a lie
with its output.

Measured on a real Pico 2 with an MPU6050 on SDA 0 / SCL 1 (2026-09-21):

  * `i2c_detect` printed "Scanning I2C bus (SDA:-1, SCL:-1)..." and then found
    the chip at 0x68 -- because `initBoard()` opens the bus with
    `envInt("i2c_sda", SDA_PIN)` while the banner printed `SDA_PIN` alone. On a
    release image, built from a generated bare config, that banner is wrong for
    every user of the unified image.
  * `--app adc_calibrate` was accepted and reported success on a part with no
    DAC. The firmware's fallback to `base` is correct and deliberate, but its
    only trace is a boot line on a serial port nobody is attached to during a
    4 KB env write.
"""
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

I2C_DETECT = os.path.join(REPO_ROOT, "firmware", "src", "tools", "i2c_detect.cpp")
ADC_LUT_H = os.path.join(REPO_ROOT, "firmware", "common", "lib", "adc_lut", "adc_lut.h")
TOOLS_CPP = os.path.join(REPO_ROOT, "firmware", "src", "tools", "tools.cpp")


def read(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def test_i2c_detect_banner_reads_the_env_not_only_the_macro():
    """The scan banner must resolve its pins the same way initBoard() does."""
    src = read(I2C_DETECT)
    banner = re.search(r'Serial\.printf\("Scanning I2C bus[^;]*;', src, re.S)
    assert banner, "the scan banner is gone; keep one that names the pins"
    assert 'envInt("i2c_sda"' in src and 'envInt("i2c_scl"' in src, (
        "i2c_detect must print the pins the bus is actually on: initBoard() uses "
        'envInt("i2c_sda", SDA_PIN), so a banner printing SDA_PIN alone reports '
        "-1 on every prebuilt image while happily scanning the env's pins"
    )


def test_i2c_detect_says_where_the_pins_came_from():
    """env or config header -- a wiring check that cannot be traced is not one."""
    src = read(I2C_DETECT)
    assert re.search(r'"env"', src) and re.search(r'"config header"', src), (
        "name the source of the pins in the banner, so a blank env still reads honestly"
    )


def test_i2c_scan_macros_always_defined():
    """They are a function argument now, not only a #if condition."""
    src = read(I2C_DETECT)
    head = src.split("namespace i2c_detect", 1)[0]
    assert head.count("#define I2C_SCAN_SDA") >= 3, (
        "every branch of the macro chain must define I2C_SCAN_SDA, including the "
        "fallback, because it is passed to envInt() unconditionally"
    )


def test_dac_env_list_matches_the_firmware_condition():
    """One fact, stated once: the flasher's list and ADC_LUT_SUPPORTED agree."""
    import flash_mcu

    gate = read(ADC_LUT_H)
    supported = gate.split("#define ADC_LUT_SUPPORTED 1", 1)[0]
    assert "CONFIG_IDF_TARGET_ESP32S2" in supported
    assert "esp32s2" in flash_mcu._DAC_ENVS and "esp32" in flash_mcu._DAC_ENVS
    for absent in ("esp32s3", "pico", "picow", "pico2", "pico2w"):
        assert absent not in flash_mcu._DAC_ENVS, (
            f"{absent} has no hardware DAC; adc_calibrate is not compiled in for it"
        )


def test_warn_fires_only_for_adc_calibrate_on_a_dacless_board(capsys):
    import flash_mcu

    flash_mcu.warn_if_app_unsupported("adc_calibrate", "pico2")
    warned = capsys.readouterr().out
    assert "hardware DAC" in warned and "boot `base`" in warned

    flash_mcu.warn_if_app_unsupported("adc_calibrate", "esp32")
    assert capsys.readouterr().out == "", "the classic ESP32 has a DAC; do not warn"

    for app in ("base", "test_sensors", "i2c_detect", "test_motors"):
        flash_mcu.warn_if_app_unsupported(app, "pico2")
        assert capsys.readouterr().out == "", f"{app} runs on every board"


def test_adc_calibrate_is_gated_in_the_tool_table():
    """The firmware side of the same fact, so the two cannot drift apart."""
    src = read(TOOLS_CPP)
    entry = re.search(r"#if ADC_LUT_SUPPORTED(.*?)#endif", src, re.S)
    assert entry and "adc_calibrate" in entry.group(1), (
        "adc_calibrate must stay inside the ADC_LUT_SUPPORTED guard in TOOLS[]"
    )
