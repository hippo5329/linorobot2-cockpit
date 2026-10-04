"""The Wi-Fi radio is kept awake on every link: power save cost 148-481 ms of
round trip on a weak link and starved /scan (GenDrv on Wi-Fi, 2026-10-04)."""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = open(os.path.join(ROOT, "firmware", "common", "lib", "wifi", "wifis.cpp")).read()


def test_both_cores_switch_power_save_off():
    body = SRC[SRC.index("static void wifiAwake(void)"):]
    body = body[:body.index("\n}\n")]
    assert "WiFi.setSleep(false);" in body          # ESP32
    assert "WiFi.noLowPowerMode();" in body         # arduino-pico (Pico W, Pico 2 W)
    assert 'envFlag("wifi_sleep", false)' in body   # opt back in, default awake


def test_it_runs_on_the_first_link_and_on_every_reconnect():
    first = SRC[SRC.index('Serial.println("WIFI connected");'):]
    assert first[:200].count("wifiAwake();") == 1
    run = SRC[SRC.index("void runWifis(void)"):]
    assert "if (!was_connected)" in run and "wifiAwake();" in run
