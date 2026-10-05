"""An ESP32 fallback flash waits for a re-enumerating board's port to return."""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import flash_mcu  # noqa: E402


def test_waits_for_the_port_to_come_back(tmp_path, monkeypatch):
    monkeypatch.setattr(flash_mcu, "PORT_RETURN_WAIT_S", 5.0)
    port = tmp_path / "ttyACM0"
    threading.Timer(1.0, port.touch).start()          # the node returns after 1 s
    t0 = time.time()
    assert flash_mcu.wait_for_port_return(str(port), "the test attempt")
    assert 1.0 <= time.time() - t0 < 4.0


def test_reports_a_port_that_never_returns(tmp_path, monkeypatch):
    monkeypatch.setattr(flash_mcu, "PORT_RETURN_WAIT_S", 0.5)
    assert not flash_mcu.wait_for_port_return(str(tmp_path / "ttyACM9"), "the test attempt")


def test_present_port_returns_at_once(tmp_path):
    port = tmp_path / "ttyUSB0"
    port.touch()
    t0 = time.time()
    assert flash_mcu.wait_for_port_return(str(port), "the test attempt")
    assert time.time() - t0 < 0.1


def test_every_esp_fallback_attempt_waits_for_the_port():
    src = open(flash_mcu.__file__).read()
    loop = src[src.index("for safe_baud in [460800, 115200]:"):]
    loop = loop[:loop.index("flashed_ok(")]
    assert "wait_for_port_return(args.port" in loop
