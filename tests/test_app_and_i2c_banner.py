"""Two diagnostics that reported the wrong thing, found by sweeping every tool.

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
  * the battery ADC calibration was offered on a part with no DAC. It is the
    tool's `adc` command now, compiled in only where ADC_LUT_SUPPORTED holds, and
    the web UI offers it on the same parts only (app-adc.js mcuHasDac).
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


def test_the_web_uis_dac_list_matches_the_firmware_condition():
    """One fact, stated twice: the panel's list and ADC_LUT_SUPPORTED agree."""
    gate = read(ADC_LUT_H)
    supported = gate.split("#define ADC_LUT_SUPPORTED 1", 1)[0]
    assert "CONFIG_IDF_TARGET_ESP32S2" in supported
    js = read(os.path.join(REPO_ROOT, "web", "frontend", "app-adc.js"))
    fn = js[js.index("function mcuHasDac(env)"):]
    fn = fn[:fn.index("\n}")]
    assert '"esp32"' in fn and '"esp32s2"' in fn
    for absent in ("esp32s3", "pico", "picow", "pico2", "pico2w"):
        assert f'"{absent}"' not in fn, f"{absent} has no hardware DAC"


def test_adc_is_gated_in_the_tool():
    """The firmware side of the same fact: no `adc` command where no DAC is."""
    src = read(TOOLS_CPP)
    blocks = re.findall(r"#if ADC_LUT_SUPPORTED(.*?)#endif", src, re.S)
    assert any('!strcmp(cmd, "adc")' in b for b in blocks), "the `adc` command left its guard"
    assert any("adc_calibrate_run" in b for b in blocks)
    assert "adc_calibrate" not in src.split("#if ADC_LUT_SUPPORTED", 1)[0].split("extern \"C\"", 1)[0]
