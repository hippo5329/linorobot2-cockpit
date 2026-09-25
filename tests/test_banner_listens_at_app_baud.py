"""The post-flash banner listener must use the APPLICATION's baud, not the upload's.

The pipeline caps uploads at 921600 (FLASH_BAUD_CEILING) and hands flash_mcu that
rate. record_stamp() listened for the boot banner at it -- so on the GenDrv, whose
runtime rate is 1.5 Mbaud, the banner was always noise and every flash recorded
banner_confirmed: false. Only the bare ESP32 ever confirmed, because both of its
rates are 921600. Found 2026-09-25 once the firmware began repeating its banner
and three of four boards were heard: the GenDrv still was not.
"""
import os
import re

SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "scripts", "flash_mcu.py")


def _record_stamp_body():
    src = open(SRC, encoding="utf-8").read()
    start = src.index("def record_stamp(")
    end = src.index("\ndef ", start + 1)
    return src[start:end]


def test_no_banner_listen_uses_the_upload_rate():
    body = _record_stamp_body()
    assert not re.search(r"listen_for_banner\(port,\s*baud\b", body), \
        "a banner listen runs at the upload rate again"
    assert not re.search(r"esp32_reset_into_app\(port,\s*baud\b", body)
    assert body.count("listen_for_banner(port, app_baud") == 2


def test_the_application_rate_comes_from_the_robot_config():
    body = _record_stamp_body()
    assert '.get("base_controller")' in body and '.get("baudrate")' in body, \
        "the runtime rate is no longer read from base_controller.baudrate"
    # ...and the ROM diagnostic still listens at the ROM's own 115200.
    assert "listen_for_banner(port, 115200" in body
