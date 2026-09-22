"""When no application banner arrives, the flasher reads the ROM at 115200 and
says what it heard -- a boot loop's cause, not "no banner"."""
import importlib.util
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ROM = """ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3808,len:0x4bc
entry 0x403c98d0
E (212) spi_flash: Detected size(4096k) smaller than the size in the binary image header(8192k). Probe failed.
assert failed: do_core_init startup.c:328 (flash_ret == ESP_OK)
Backtrace: 0x40377f66:0x3fceb180 0x4037d025:0x3fceb1a0
ELF file SHA256: f8375ad979e491f5
E (283) esp_core_dump_flash: Core dump flash config is corrupted! CRC=0x7bd5c66f instead of 0x0
Rebooting...
ESP-ROM:esp32s3-20210327
rst:0xc (RTC_SW_CPU_RST),boot:0x8 (SPI_FAST_FLASH_BOOT)
E (216) spi_flash: Detected size(4096k) smaller than the size in the binary image header(8192k). Probe failed.
Rebooting...
"""


def _load():
    spec = importlib.util.spec_from_file_location("flash_mcu", os.path.join(ROOT, "scripts", "flash_mcu.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_rom_lines_keep_the_diagnosis_and_drop_the_repeats():
    fm = _load()
    lines = fm.rom_boot_lines(ROM)
    text = "\n".join(lines)
    assert "Detected size(4096k) smaller than the size in the binary image header(8192k)" in text
    assert "assert failed: do_core_init" in text
    assert "Rebooting..." in text
    assert "load:0x3fce3808" not in text and "SPIWP" not in text
    assert lines.count("Rebooting...") == 1, "repeats collapse"
    assert len(lines) <= 8


def test_silence_yields_nothing():
    fm = _load()
    assert fm.rom_boot_lines("") == []
    assert fm.rom_boot_lines("linorobot2_hardware app=base\n") == []


def test_the_fallback_runs_only_when_the_app_banner_is_missing_on_an_esp():
    with open(os.path.join(ROOT, "scripts", "flash_mcu.py")) as fh:
        src = fh.read()
    blk = src[src.index("rom_lines = []"):src.index("if banner:", src.index("rom_lines = []"))]
    assert "if not banner and app_written and is_esp_family(env)" in blk
    assert "listen_for_banner(port, 115200, 3.0, reset=True)" in blk
    assert "boot loop" in blk
