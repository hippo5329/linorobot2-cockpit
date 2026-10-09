"""A board this port flashed over USB is waited for before a Wi-Fi robot goes over the air.

The probe resets an ESP32-S3; it re-enumerates on the host within a second and a cell's
passthrough re-adds the node seconds later. flash_mcu used to take the absent node as
"the robot has left the cable" and go over the air -- where a board parked on a serial
env is not -- and S3 Wi-Fi legs failed [NO BOARD] before anything ran.
"""
import os
import sys
import types

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))

import flash_mcu  # noqa: E402


def args(port):
    return types.SimpleNamespace(env="esp32s3", port=str(port))


def test_a_stamped_port_that_comes_back_is_waited_for(tmp_path, monkeypatch):
    port = tmp_path / "ttyACM0"
    monkeypatch.setattr(flash_mcu, "stamp_for", lambda env, p: {"board_id": "5C62F462F2A0"})
    calls = {"n": 0}
    real_sleep = flash_mcu.time.sleep

    def sleep(s):                      # the node reappears on the third poll
        calls["n"] += 1
        if calls["n"] == 3:
            port.write_text("")
        real_sleep(0)
    monkeypatch.setattr(flash_mcu.time, "sleep", sleep)
    assert flash_mcu.wait_for_stamped_port(args(port), timeout=5) is True
    assert calls["n"] == 3


def test_no_stamp_no_wait(tmp_path, monkeypatch):
    monkeypatch.setattr(flash_mcu, "stamp_for", lambda env, p: {})
    monkeypatch.setattr(flash_mcu.time, "sleep", lambda s: (_ for _ in ()).throw(AssertionError("waited")))
    assert flash_mcu.wait_for_stamped_port(args(tmp_path / "ttyACM0")) is False


def test_a_port_that_never_returns_gives_up_after_the_timeout(tmp_path, monkeypatch):
    monkeypatch.setattr(flash_mcu, "stamp_for", lambda env, p: {"board_id": "x"})
    assert flash_mcu.wait_for_stamped_port(args(tmp_path / "ttyACM0"), timeout=0.3, poll=0.05) is False


def test_the_wait_comes_before_the_over_the_air_choice():
    src = open(os.path.join(REPO, "scripts", "flash_mcu.py"), encoding="utf-8").read()
    i_wait = src.index("        wait_for_stamped_port(args)")
    i_ota = src.index("        return flash_over_air(args, prebuilt_dir, ota_build_dir)")
    assert i_wait < i_ota
