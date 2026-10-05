"""The Hardware Tests tab on an Arduino UNO Q, and Flash base on a lyrical stack.

The tab switches the one image's application with a 4 KB env write (app=<tool>). On the
UNO Q that write goes over SWD (flash_mcu.py -> unoq_swd.py), and the tool's text comes
out the micro-ROS link, /dev/ttyHS1, which Monitor reads like any board's serial port.

Three faults this guards, each found while adding the UNO Q (2026-10-06):
* mcu_identity.pio_env_for("unoq") gave "pico2": the UNO Q is in no platformio.ini, so a
  tool switch asked for on a UNO Q went off as a Pico 2 flash;
* /api/status read the UNO Q's empty USB bus as "no board", and the UI switched the robot
  to the Sim MCU -- the STM32 is on the board, wired to a UART and GPIOs, never on USB;
* Flash base asked for `<board>-jazzy` whatever the stack's distro: on a lyrical stack the
  board enumerated and never held a session (micro-ROS is fixed at link time).
"""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import flash_mcu  # noqa: E402
import mcu_identity  # noqa: E402
import unoq_swd  # noqa: E402


def test_the_unoq_is_an_env_though_platformio_has_never_heard_of_it():
    assert mcu_identity.pio_env_for("unoq", "pico2") == "unoq"
    assert mcu_identity.pio_env_for("unoq_lyrical", "pico2") == "unoq_lyrical"
    assert flash_mcu.is_unoq_family("unoq") and flash_mcu.is_unoq_family("unoq_lyrical")
    assert not flash_mcu.is_unoq_family("pico2")


@pytest.mark.parametrize("env, distro, want", [
    ("pico2", "jazzy", "pico2"),
    ("pico2", "lyrical", "pico2_lyrical"),
    ("pico2w", "lyrical", "pico2w_lyrical"),
    ("esp32s3", "lyrical", "esp32s3_lyrical"),
    ("unoq", "lyrical", "unoq_lyrical"),       # Zephyr: no platformio.ini entry needed
    ("unoq", "jazzy", "unoq"),
    ("pico2_lyrical", "lyrical", "pico2_lyrical"),
    ("sim", "lyrical", "sim"),
])
def test_the_flashed_image_speaks_the_stacks_distro(env, distro, want):
    assert mcu_identity.env_for_distro(env, distro) == want


def test_no_build_for_the_distro_is_an_error_not_the_jazzy_image():
    with pytest.raises(ValueError) as e:
        mcu_identity.env_for_distro("nosuchboard", "lyrical")
    assert "jazzy" in str(e.value)


def test_the_board_it_runs_on_is_read_from_its_device_tree(tmp_path):
    (tmp_path / "compatible").write_bytes(b"arduino,imola\0qcom,qcm2290\0qcom,qrb2210\0")
    fam, chip, port = mcu_identity.onboard_controller(str(tmp_path))
    assert (fam, port) == ("unoq", "/dev/ttyHS1") and "UNO Q" in chip
    (tmp_path / "compatible").write_bytes(b"raspberrypi,5-model-b\0brcm,bcm2712\0")
    assert mcu_identity.onboard_controller(str(tmp_path))[0] is None
    assert mcu_identity.onboard_controller(str(tmp_path / "absent"))[0] is None


class _Args:
    def __init__(self, env_only, app="test_sensors"):
        self.env_only, self.app, self.env = env_only, app, "unoq"


@pytest.fixture
def swd(monkeypatch, tmp_path):
    calls = []
    env_bin = tmp_path / "env.bin"
    env_bin.write_bytes(b"\0" * 64)
    monkeypatch.setattr(flash_mcu, "resolve_env_bin", lambda args, d: str(env_bin))
    monkeypatch.setattr(unoq_swd, "identify", lambda: (0x482, "20393543584350050018003C"))
    monkeypatch.setattr(unoq_swd, "flash", lambda e, elf=None: calls.append((e, elf)))
    return calls, str(env_bin)


def test_a_tool_switch_is_an_env_write_over_swd_and_leaves_the_image(swd):
    calls, env_bin = swd
    assert flash_mcu.flash_unoq(_Args(env_only=True), None) == 0
    assert calls == [(env_bin, None)]


def test_flash_base_writes_the_release_image_and_the_env(swd, tmp_path):
    calls, env_bin = swd
    assert flash_mcu.flash_unoq(_Args(env_only=False, app="base"), str(tmp_path)) == 0
    assert calls == [(env_bin, os.path.join(str(tmp_path), "firmware.elf"))]


def test_an_image_flash_without_a_prebuilt_is_refused(swd):
    calls, _ = swd
    assert flash_mcu.flash_unoq(_Args(env_only=False), None) == 1
    assert calls == []


def test_a_board_that_does_not_answer_on_swd_fails_the_flash(swd, monkeypatch):
    def nobody():
        raise unoq_swd.SwdError("OpenOCD did not reach the STM32 over SWD")
    monkeypatch.setattr(unoq_swd, "identify", nobody)
    calls, _ = swd
    assert flash_mcu.flash_unoq(_Args(env_only=True), None) == 1
    assert calls == []


def test_the_unoq_never_reaches_the_usb_paths(monkeypatch, tmp_path):
    """main() hands the UNO Q to SWD before any USB stamp, MCU check or port release."""
    seen = []
    monkeypatch.setattr(flash_mcu, "flash_unoq", lambda args, d: seen.append(args.env) or 0)
    for name in ("remember_usb_path", "release_serial_port", "ensure_port_permissions"):
        monkeypatch.setattr(flash_mcu, name, lambda *a, **k: pytest.fail("a USB path ran for the UNO Q"))
    monkeypatch.setattr(flash_mcu.mcu_identity, "identify_target",
                        lambda *a, **k: pytest.fail("the USB MCU check ran for the UNO Q"))
    params = tmp_path / "unoq_config.yaml"
    params.write_text("robot: {name: unoq}\n")
    monkeypatch.setattr(sys, "argv", ["flash_mcu.py", "--env", "unoq", "--env-only", "--port", "/dev/ttyHS1",
                                      "--params", str(params), "--app", "test_sensors"])
    assert flash_mcu.main() == 0
    assert seen == ["unoq"]


def test_the_ui_offers_the_unoq_and_assigns_its_reference_pins():
    html = open(os.path.join(ROOT, "web", "frontend", "index.html")).read()
    assert html.count('<option value="unoq"') == 2          # cfg-mcu and hw-flash-env
    hw = open(os.path.join(ROOT, "web", "frontend", "app-hardware.js")).read()
    assert 'mcu === "unoq"' in hw
    presets = open(os.path.join(ROOT, "web", "frontend", "app-presets.js")).read()
    assert 'return "unoq"' in presets


def test_monitor_releases_the_port_without_lsof():
    routes = open(os.path.join(ROOT, "web", "backend", "routes_hardware.py")).read()
    a = routes.index('elif action == "monitor":')
    code = [ln for ln in routes[a:routes.index("    else:", a)].splitlines() if not ln.strip().startswith("#")]
    assert not any("lsof" in ln for ln in code)
    assert any("release_serial_port" in ln for ln in code)


def test_the_unoq_baud_survives_a_load_and_a_save():
    """The config's 4000000 had no option: the select went blank and Save wrote 921600."""
    html = open(os.path.join(ROOT, "web", "frontend", "index.html")).read()
    a = html.index('id="cfg-baudrate"')
    assert '<option value="4000000"' in html[a:html.index("</select>", a)]
    hw = open(os.path.join(ROOT, "web", "frontend", "app-hardware.js")).read()
    a = hw.index('const elBaud = document.getElementById("cfg-baudrate");')
    assert "elBaud.add(new Option(" in hw[a:a + 800]    # any rate the list lacks is added, not dropped
