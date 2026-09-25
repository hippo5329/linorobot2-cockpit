#!/usr/bin/env python3
"""What each MCU's pins can do, and what a robot config asks of them.

The GenDrv boot-looped for an afternoon because a config drove GPIO 6, 8, 11
and 1 on an ESP32-WROOM: the SPI flash bus and U0TXD. Nothing in the tree knew
those pins were off limits, so the mistake reached the board and came back as
an interrupt watchdog reset with no pin named anywhere. This module is that
knowledge, in one place: the usable GPIO range per MCU, the pins that must
never be driven, the ones that are input-only or strapping, which pins are
ADC, and which pairs an RP2 I2C block accepts. `check_config()` reads a robot
config the way gen_firmware_header.py does and returns what is wrong or
doubtful, so the message arrives at header time -- before anything is flashed.

Levels: "error" means the board will not work or may be damaged; "warn"
means it may work and a human should look.
"""
from typing import Dict, List, Tuple

Finding = Tuple[str, str]

# RP2040 / RP2350 on the Pico form factor. GP23/24/25/29 are internal on the
# board (SMPS mode, VBUS sense, the LED, VSYS/3); on the W boards the same
# four belong to the CYW43 interface.
_RP2_HEADER_GPIO = set(range(0, 23)) | {26, 27, 28}
CYW43_LED = 64   # arduino-pico PIN_LED on picow/pico2w; a non-W Pico maps it to GP25
_RP2_I2C = {
    0: ({0, 4, 8, 12, 16, 20, 28}, {1, 5, 9, 13, 17, 21}),
    1: ({2, 6, 10, 14, 18, 26}, {3, 7, 11, 15, 19, 27}),
}

CATALOG: Dict[str, dict] = {
    "rp2040": {
        "label": "RP2040 (Pico)",
        "gpio": _RP2_HEADER_GPIO | {25},
        "never": set(),
        "input_only": set(),
        "strapping": set(),
        "adc": {26, 27, 28},
        "adc_wifi_conflict": set(),
        "led": 25,
        "i2c": _RP2_I2C,
    },
    "rp2350": {
        "label": "RP2350 (Pico 2)",
        "gpio": _RP2_HEADER_GPIO | {25},
        "never": set(),
        "input_only": set(),
        "strapping": set(),
        "adc": {26, 27, 28},
        "adc_wifi_conflict": set(),
        "led": 25,
        "i2c": _RP2_I2C,
    },
    "esp32": {
        "label": "ESP32 (WROOM)",
        "gpio": (set(range(0, 40)) - {20, 24, 28, 29, 30, 31}),
        # SPI flash bus. Touching these resets the chip.
        "never": {6, 7, 8, 9, 10, 11},
        "input_only": {34, 35, 36, 37, 38, 39},
        "strapping": {0, 2, 5, 12, 15},
        "uart0": {1, 3},
        "adc": set(range(32, 40)) | {0, 2, 4, 12, 13, 14, 15, 25, 26, 27},
        # ADC2 is owned by the Wi-Fi driver while the radio is up.
        "adc_wifi_conflict": {0, 2, 4, 12, 13, 14, 15, 25, 26, 27},
        "led": 2,
        "i2c": None,   # any GPIO through the matrix
    },
    "esp32s3": {
        "label": "ESP32-S3",
        "gpio": (set(range(0, 49)) - {22, 23, 24, 25}),
        # SPI flash / PSRAM on every module.
        "never": {26, 27, 28, 29, 30, 31, 32},
        "input_only": set(),
        "strapping": {0, 3, 45, 46},
        "usb": {19, 20},
        "psram_octal": {33, 34, 35, 36, 37},
        "adc": set(range(1, 21)),
        "adc_wifi_conflict": set(range(11, 21)),
        "led": 48,
        "i2c": None,
    },
}

_MCU_ALIASES = {
    "pico": "rp2040", "picow": "rp2040", "rp2040": "rp2040",
    "pico2": "rp2350", "pico2w": "rp2350", "rp2350": "rp2350",
    "esp32": "esp32", "esp32s3": "esp32s3", "esp32-s3": "esp32s3",
}


def mcu_key(name: str) -> str:
    return _MCU_ALIASES.get(str(name or "").strip().lower(), "")


def _pin(value) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return -1


def _collect(tgt: dict) -> List[Tuple[str, int, str]]:
    """(role, gpio, direction) for every pin the config names. direction:
    'out', 'in', 'adc', 'io' (I2C)."""
    pins = tgt.get("pins") or {}
    out = []
    for n in range(1, 5):
        m = pins.get(f"motor{n}") or {}
        for k in ("pwm", "in_a", "in_b"):
            out.append((f"motor{n}.{k}", _pin(m.get(k, -1)), "out"))
        e = pins.get(f"encoder{n}") or {}
        for k in ("pin_a", "pin_b"):
            out.append((f"encoder{n}.{k}", _pin(e.get(k, -1)), "in"))
    i2c = pins.get("i2c") or {}
    out.append(("i2c.sda", _pin(i2c.get("sda", -1)), "io"))
    out.append(("i2c.scl", _pin(i2c.get("scl", -1)), "io"))
    # imu.int is still collected so a config that declares it is checked for
    # conflicts, but nothing reads it any more: the IMU data-ready interrupt was
    # removed on 2026-09-24 in favour of polling the chip's own FIFO. Left in the
    # catalogue rather than rejected, so an existing config keeps validating.
    imu = pins.get("imu") or {}
    out.append(("imu.int", _pin(imu.get("int", -1)), "in"))
    out.append(("led", _pin(pins.get("led", -1)), "out"))
    bat = pins.get("battery") or {}
    out.append(("battery.pin", _pin(bat.get("pin", -1)), "adc"))
    sonar = pins.get("sonar") or {}
    out.append(("sonar.trigger", _pin(sonar.get("trigger", -1)), "out"))
    out.append(("sonar.echo", _pin(sonar.get("echo", -1)), "in"))
    lidar = tgt.get("lidar") or {}
    if str(lidar.get("comm_mode", "")).lower() == "serial":
        out.append(("lidar.rx_pin", _pin(lidar.get("rx_pin", -1)), "in"))
    return [(r, g, d) for (r, g, d) in out if g >= 0]


def check_config(params: dict) -> List[Finding]:
    tgt = params.get("base_controller") or {}
    key = mcu_key(tgt.get("mcu") or tgt.get("name"))
    if not key:
        return [("warn", f"no pin catalogue for mcu '{tgt.get('mcu')}'; pins unchecked")]
    cat = CATALOG[key]
    board = str(tgt.get("mcu") or tgt.get("name") or "").lower()
    wireless_rp2 = board in ("picow", "pico2w")
    wifi = bool((tgt.get("wifi") or {}).get("enabled", False)) or \
        str(tgt.get("transport", "")).lower() in ("udp4", "udp", "wifi")
    findings: List[Finding] = []
    used = _collect(tgt)

    for role, gpio, direction in used:
        # arduino-pico's PIN_LED on a W board: the CYW43's WL_GPIO0, not a header pin.
        if role == "led" and gpio == CYW43_LED and wireless_rp2:
            continue
        if gpio not in cat["gpio"]:
            findings.append(("error", f"{role}: GPIO {gpio} does not exist on the {cat['label']}"))
            continue
        if gpio in cat["never"]:
            findings.append(("error", f"{role}: GPIO {gpio} is the {cat['label']}'s flash/PSRAM bus; driving it resets or bricks the board"))
            continue
        if direction == "out" and gpio in cat["input_only"]:
            findings.append(("error", f"{role}: GPIO {gpio} is input-only on the {cat['label']}"))
        if direction == "out" and gpio in cat["strapping"]:
            findings.append(("warn", f"{role}: GPIO {gpio} is a strapping pin; a driver holding it at boot can change the boot mode"))
        if gpio in cat.get("uart0", set()):
            findings.append(("warn", f"{role}: GPIO {gpio} is UART0, the console and flashing port"))
        if gpio in cat.get("usb", set()):
            findings.append(("warn", f"{role}: GPIO {gpio} is USB D+/D-; the native USB console dies with it"))
        if gpio in cat.get("psram_octal", set()):
            findings.append(("warn", f"{role}: GPIO {gpio} is PSRAM on octal-PSRAM modules"))
        if direction == "adc":
            if gpio not in cat["adc"]:
                findings.append(("error", f"{role}: GPIO {gpio} is not an ADC input on the {cat['label']}"))
            elif wifi and gpio in cat["adc_wifi_conflict"]:
                findings.append(("warn", f"{role}: GPIO {gpio} is on ADC2, which the Wi-Fi driver owns while the radio is up; reads will fail"))
        if wireless_rp2 and gpio in (23, 24, 25, 29):
            findings.append(("warn", f"{role}: GPIO {gpio} belongs to the wireless interface on a {board}; the LED is on the CYW43 (led: -1)"))

    # One pin, one job -- except a shared enable line, which is the same job
    # on several motors (the two-PWM drivers hold it HIGH; the writes are
    # idempotent).
    seen: Dict[int, List[str]] = {}
    for role, gpio, _ in used:
        seen.setdefault(gpio, []).append(role)
    for gpio, roles in sorted(seen.items()):
        if len(roles) < 2:
            continue
        if all(r.endswith(".pwm") for r in roles) and \
                str(tgt.get("driver_type", "")).upper() == "BTS7960":
            continue
        findings.append(("error", f"GPIO {gpio} is used by {', '.join(roles)}"))

    # RP2 I2C blocks accept fixed pin sets, and SDA/SCL must be one block.
    i2c = cat.get("i2c")
    if i2c:
        pins = tgt.get("pins") or {}
        sda, scl = _pin((pins.get("i2c") or {}).get("sda", -1)), _pin((pins.get("i2c") or {}).get("scl", -1))
        if sda >= 0 or scl >= 0:
            ok = any(sda in s and scl in c for s, c in i2c.values())
            if not ok:
                findings.append(("error", f"i2c: GP{sda}/GP{scl} is not a valid SDA/SCL pair on one {cat['label']} I2C block "
                                          f"(I2C0 SDA {sorted(i2c[0][0])} SCL {sorted(i2c[0][1])}; I2C1 SDA {sorted(i2c[1][0])} SCL {sorted(i2c[1][1])})"))
    return findings


def main() -> int:
    import argparse, sys
    import yaml
    ap = argparse.ArgumentParser(description="Check a robot config's pins against the MCU's catalogue")
    ap.add_argument("--params", required=True)
    a = ap.parse_args()
    with open(a.params) as f:
        params = yaml.safe_load(f) or {}
    findings = check_config(params)
    for level, msg in findings:
        print(f"[pins] {level}: {msg}")
    if not findings:
        print("[pins] ok")
    return 1 if any(l == "error" for l, _ in findings) else 0


if __name__ == "__main__":
    raise SystemExit(main())
