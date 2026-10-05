"""1-Click on the Arduino UNO Q: the release image and the env block go to the STM32 over SWD.

The UNO Q's MCU has no USB and no banner on its micro-ROS link, so the USB check, the
serial probe, picotool and esptool do not apply. The pipeline asks the flash itself
(OpenOCD verify_image), installs unoq-<distro> when it differs, and writes the env every
run, read back and compared (scripts/unoq_swd.py).
"""
import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import one_click_pipeline as ocp  # noqa: E402
import unoq_swd  # noqa: E402


def test_unoq_is_its_own_silicon_and_resolves_per_distro_without_platformio():
    assert ocp.is_unoq_env("unoq") and ocp.is_unoq_env("unoq_lyrical")
    assert not ocp.is_unoq_env("pico2")
    assert ocp.resolve_pio_env("unoq", "jazzy") == "unoq"
    assert ocp.resolve_pio_env("unoq", "lyrical") == "unoq_lyrical"   # no [env:unoq_lyrical] needed


def test_a_local_build_is_refused_for_the_unoq():
    with pytest.raises(SystemExit) as e:
        ocp.firmware_source("build", "unoq", "jazzy")
    assert "Zephyr" in str(e.value)


def test_the_release_profile_is_unoq_distro(monkeypatch):
    asked = []
    monkeypatch.setattr(ocp.fetch_prebuilt, "fetch", lambda p: asked.append(p) or f"/x/{p}")
    assert ocp.firmware_source("auto", "unoq_lyrical", "lyrical") == ("prebuilt", "/x/unoq-lyrical")
    assert asked == ["unoq-lyrical"]


def _prebuilt(tmp_path, commit="abc1234", distro="jazzy"):
    (tmp_path / "firmware.elf").write_bytes(b"\x7fELF")
    (tmp_path / "manifest.json").write_text(json.dumps({"commit": commit, "ros_distro": distro}))
    return str(tmp_path)


def test_probe_current_flash_means_env_only(tmp_path, monkeypatch):
    monkeypatch.setattr(unoq_swd, "identify", lambda: (0x482, "UID"))
    monkeypatch.setattr(unoq_swd, "verify", lambda elf: True)
    b = ocp.probe_unoq(_prebuilt(tmp_path))
    assert b["verdict"] == "current" and not b["firmware_differs"]


def test_probe_other_image_means_install(tmp_path, monkeypatch):
    monkeypatch.setattr(unoq_swd, "identify", lambda: (0x482, "UID"))
    monkeypatch.setattr(unoq_swd, "verify", lambda elf: False)
    b = ocp.probe_unoq(_prebuilt(tmp_path))
    assert b["verdict"] == "stale" and b["firmware_differs"] and b["local"]["git"] == "abc1234"


def test_probe_that_cannot_reach_swd_is_a_failed_probe_not_a_stale_board(tmp_path, monkeypatch):
    def boom():
        raise unoq_swd.SwdError("no OpenOCD")
    monkeypatch.setattr(unoq_swd, "identify", boom)
    b = ocp.probe_unoq(_prebuilt(tmp_path))
    assert b["probe_failed"] and b["verdict"] == "unknown"


def test_swd_refuses_an_mcu_that_is_not_an_stm32u5(monkeypatch):
    monkeypatch.setattr(unoq_swd, "run", lambda cmds, timeout=120: "0xe0044000: 10006413 \n0x0bfa0700: 1 2 3\n")
    with pytest.raises(unoq_swd.SwdError) as e:
        unoq_swd.identify()
    assert "0x413" in str(e.value)


def test_swd_reads_an_stm32u585(monkeypatch):
    monkeypatch.setattr(unoq_swd, "run", lambda cmds, timeout=120:
                        "0xe0044000: 20016482 \n0x0bfa0700: 0018003c 58435005 20393543 \n")
    dev, uid = unoq_swd.identify()
    assert dev == 0x482 and uid == "20393543584350050018003C"   # this board's UID


def test_openocd_runs_through_the_host_loader_inside_the_container(tmp_path, monkeypatch):
    od = tmp_path / "openocd"; (od / "bin").mkdir(parents=True)
    (od / "bin" / "openocd").write_text(""); (od / "openocd_gpiod.cfg").write_text("")
    hl = tmp_path / "host-lib"; hl.mkdir(); (hl / "ld-linux-aarch64.so.1").write_text("")
    monkeypatch.setattr(unoq_swd, "OPENOCD_DIR", str(od)); monkeypatch.setattr(unoq_swd, "HOST_LIB", str(hl))
    argv = unoq_swd.openocd_argv()
    assert argv[:3] == [str(hl / "ld-linux-aarch64.so.1"), "--library-path", str(hl)]
    assert str(od / "share" / "openocd" / "scripts") in argv


def test_port_holders_reads_proc_and_needs_no_lsof(tmp_path):
    """lsof did not return in 30 s inside the UNO Q's container; /proc answers directly."""
    dev = tmp_path / "ttyHS1"; dev.write_text("")
    proc = tmp_path / "proc"
    for pid, target in (("101", dev), ("202", tmp_path / "other")):
        (proc / pid / "fd").mkdir(parents=True)
        os.symlink(str(target), proc / pid / "fd" / "3")
    (proc / "self").mkdir()                     # not a pid: skipped
    assert ocp.port_holders(str(dev), proc=str(proc)) == ["101"]
    src = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()
    assert '"lsof"' not in src
