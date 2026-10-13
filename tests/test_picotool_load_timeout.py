"""A picotool load gets time for the image it loads.

The load erases, writes and verifies the whole image over full-speed USB, so it
takes longer as the image grows. A fixed 20 s held until the RP2040 image
reached 1.34 MB. On a slower USB host controller that load took 23 s, every
attempt was killed part-way, and the board was left in BOOTSEL with half its
flash erased: every leg on that cell failed (rc-20261013.2 gate). The same file,
loaded by hand with time to finish, went through in 23 s and ran.
"""
import os
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import flash_mcu  # noqa: E402

MEASURED_S = 23          # 1.34 MB on the slow controller
IMAGE_BYTES = 1_337_344  # the rc-20261013.2 pico-lyrical firmware.uf2


def test_the_limit_sits_well_past_the_slowest_load_seen(tmp_path):
    uf2 = tmp_path / "firmware.uf2"
    uf2.write_bytes(b"\x00" * IMAGE_BYTES)
    assert flash_mcu.picotool_load_timeout(str(uf2)) >= 3 * MEASURED_S


def test_a_bigger_image_gets_more_time(tmp_path):
    small, big = tmp_path / "s.uf2", tmp_path / "b.uf2"
    small.write_bytes(b"\x00" * 300_000)
    big.write_bytes(b"\x00" * 2_000_000)
    assert flash_mcu.picotool_load_timeout(str(big)) > flash_mcu.picotool_load_timeout(str(small))


def test_a_missing_image_still_gets_a_sane_limit(tmp_path):
    assert flash_mcu.picotool_load_timeout(str(tmp_path / "nope.uf2")) >= 60


def test_every_application_load_uses_it(tmp_path, monkeypatch):
    """Every form of the load the flasher tries, not just the first."""
    uf2 = tmp_path / "firmware.uf2"
    uf2.write_bytes(b"\x00" * IMAGE_BYTES)
    seen = []

    class _Res:
        returncode = 1
        stdout = "unknown option"   # also drives the plain fallback
        stderr = "permission"       # and the sudo forms

    def stub_run(cmd, timeout=None, **kw):
        seen.append((cmd, timeout))
        return _Res()

    monkeypatch.setattr(flash_mcu, "find_picotool_binaries", lambda: ["/usr/bin/picotool"])
    monkeypatch.setattr(flash_mcu, "picotool_target", lambda **k: [])   # no target: -f forms too
    monkeypatch.setattr(flash_mcu, "run_tool", stub_run)
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: _Res())

    assert flash_mcu.flash_via_picotool(str(uf2), "pico") is False
    loads = [(c, t) for c, t in seen if "load" in c and str(uf2) in c]
    assert len(loads) >= 4, loads
    want = flash_mcu.picotool_load_timeout(str(uf2))
    for cmd, t in loads:
        assert t == want, (cmd, t)
