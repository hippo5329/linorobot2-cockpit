"""Flash MCU through /api/firmware/flash (web/backend/hardware_service.py): the image for the
stack's ROS 2 distro, and the Arduino UNO Q.

Two faults from a web UI walkthrough (2026-10-16):
* prebuilt_dir_for_env() took the FIRST firmware/prebuilt/<profile>, alphabetically, whose
  manifest named the env. unoq-jazzy and unoq-lyrical both say `pio_env: unoq`, so a lyrical
  robot was flashed the jazzy image (micro-ROS is fixed at link time: it never holds a
  session), and a variant such as unoq-jazzy-bno sorted in front of the release image;
* the flash knew only esptool and picotool, so the UNO Q (an STM32U585 on SWD) could not be
  flashed at all. It now goes through scripts/flash_mcu.py --prebuilt unoq-<distro> with the
  robot's config, so the board gets a fresh env block with its image.
"""
import contextlib
import json
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import hardware_service as hs  # noqa: E402


def profile(root, name, pio_env, distro=None, files=("firmware.elf",)):
    d = root / name
    d.mkdir()
    m = {"profile": name, "pio_env": pio_env, "files": []}
    if distro:
        m["ros_distro"] = distro
    (d / "manifest.json").write_text(json.dumps(m))
    for f in files:
        (d / f).write_bytes(b"\0")
    return str(d)


@pytest.fixture
def prebuilt(tmp_path, monkeypatch):
    root = tmp_path / "prebuilt"
    root.mkdir()
    monkeypatch.setattr(hs, "PREBUILT_ROOT", str(root))
    return root


def test_the_unoq_profile_follows_the_distro(prebuilt):
    # the variants sort in front of the release images, and all of them say pio_env unoq
    profile(prebuilt, "unoq-jazzy-bno", "unoq", "jazzy")
    profile(prebuilt, "unoq-jazzy", "unoq", "jazzy")
    profile(prebuilt, "unoq-jazzy.rel", "unoq", "jazzy")
    profile(prebuilt, "unoq-lyrical-diag", "unoq", "lyrical")
    profile(prebuilt, "unoq-lyrical", "unoq", "lyrical")
    assert os.path.basename(hs.prebuilt_dir_for_env("unoq", "lyrical")) == "unoq-lyrical"
    assert os.path.basename(hs.prebuilt_dir_for_env("unoq", "jazzy")) == "unoq-jazzy"
    assert os.path.basename(hs.prebuilt_dir_for_env("unoq_lyrical")) == "unoq-lyrical"
    assert os.path.basename(hs.prebuilt_dir_for_env("unoq")) == "unoq-jazzy"


def test_a_variant_alone_is_never_the_image(prebuilt):
    profile(prebuilt, "unoq-jazzy-bno", "unoq", "jazzy")
    profile(prebuilt, "unoq-lyrical-diag", "unoq", "lyrical")
    assert hs.prebuilt_dir_for_env("unoq", "jazzy") is None
    assert hs.prebuilt_dir_for_env("unoq", "lyrical") is None


def test_the_other_distro_is_never_the_image(prebuilt):
    profile(prebuilt, "unoq-jazzy", "unoq", "jazzy")
    assert hs.prebuilt_dir_for_env("unoq", "lyrical") is None
    # a profile whose name and manifest disagree is not trusted either
    profile(prebuilt, "unoq-lyrical", "unoq", "jazzy")
    assert hs.prebuilt_dir_for_env("unoq", "lyrical") is None


def test_rp2_profiles_built_from_the_w_env_are_found_by_board(prebuilt):
    # the release ships pico2-<distro>, built from pico2w (fetch_prebuilt.profile_for_env)
    profile(prebuilt, "pico2-jazzy", "pico2w", "jazzy", files=("firmware.uf2",))
    profile(prebuilt, "pico2-lyrical", "pico2w_lyrical", "lyrical", files=("firmware.uf2",))
    assert os.path.basename(hs.prebuilt_dir_for_env("pico2_lyrical", "lyrical")) == "pico2-lyrical"
    assert os.path.basename(hs.prebuilt_dir_for_env("pico2", "lyrical")) == "pico2-lyrical"
    assert os.path.basename(hs.prebuilt_dir_for_env("pico2", "jazzy")) == "pico2-jazzy"


def test_another_plain_profile_naming_the_env_is_found(prebuilt):
    # not the release name, but a plain <board>-<distro> profile whose manifest names the env
    profile(prebuilt, "esp32w-lyrical", "esp32_lyrical", "lyrical", files=("firmware.bin",))
    assert os.path.basename(hs.prebuilt_dir_for_env("esp32_lyrical", "lyrical")) == "esp32w-lyrical"
    assert hs.prebuilt_dir_for_env("esp32_lyrical", "jazzy") is None


def test_the_unoq_is_its_own_family_and_its_artifact_is_the_prebuilt_elf(prebuilt):
    assert hs.family_for_env("unoq") == hs.family_for_env("unoq_lyrical") == "unoq"
    assert hs.family_for_env("pico2_lyrical") == "rp" and hs.family_for_env("esp32s3") == "esp"
    profile(prebuilt, "unoq-lyrical", "unoq", "lyrical")
    a = hs.resolve_artifact("firmware", "unoq_lyrical", "lyrical")
    assert a["family"] == "unoq" and not a.get("error")
    assert a["artifact"].endswith(os.path.join("unoq-lyrical", "firmware.elf"))
    missing = hs.resolve_artifact("firmware", "unoq", "jazzy")
    assert "unoq-jazzy" in missing["error"]


@pytest.fixture
def unoq_flash(prebuilt, monkeypatch, tmp_path):
    """flash() with the tools faked: what it would run, in order."""
    ran = []
    result = {"fetch": 0, "flash": 0}

    def fake_stream(cmd, emit, timeout=300):
        ran.append(cmd)
        return result["fetch"] if cmd[2].endswith("fetch_prebuilt.py") else result["flash"]

    @contextlib.contextmanager
    def no_agent(*a, **kw):
        yield {"paused": False}

    monkeypatch.setattr(hs, "_stream_tool", fake_stream)
    monkeypatch.setattr(hs, "microros_agent_paused", no_agent)
    cfg = tmp_path / "robot1_config.yaml"
    cfg.write_text("base_controller:\n  name: unoq\n")
    return ran, result, str(cfg)


def test_the_unoq_is_flashed_by_flash_mcu_with_the_distros_image_and_a_fresh_env(unoq_flash, prebuilt):
    ran, _, cfg = unoq_flash
    profile(prebuilt, "unoq-jazzy", "unoq", "jazzy")
    profile(prebuilt, "unoq-lyrical", "unoq", "lyrical")
    r = hs.flash(env="unoq_lyrical", port="/dev/ttyHS1", distro="lyrical", params_path=cfg)
    assert r["ok"] is True, r
    fetch, flash = ran
    assert fetch[-1] == "unoq-lyrical"
    assert flash[2].endswith("flash_mcu.py")
    args = flash[3:]
    assert args[args.index("--prebuilt") + 1] == "unoq-lyrical"
    assert args[args.index("--params") + 1] == cfg            # the env block comes from it
    assert args[args.index("--app") + 1] == "base"
    assert args[args.index("--port") + 1] == "/dev/ttyHS1"
    assert "--env-only" not in args


def test_an_offline_flash_uses_the_image_already_here(unoq_flash, prebuilt):
    ran, result, cfg = unoq_flash
    result["fetch"] = 1
    profile(prebuilt, "unoq-jazzy", "unoq", "jazzy")
    r = hs.flash(env="unoq", port="/dev/ttyHS1", distro="jazzy", params_path=cfg)
    assert r["ok"] is True and ran[-1][ran[-1].index("--prebuilt") + 1] == "unoq-jazzy"


def test_no_image_for_the_distro_is_an_error_not_the_other_one(unoq_flash, prebuilt):
    ran, result, cfg = unoq_flash
    result["fetch"] = 1
    profile(prebuilt, "unoq-jazzy", "unoq", "jazzy")
    r = hs.flash(env="unoq", port="/dev/ttyHS1", distro="lyrical", params_path=cfg)
    assert r["ok"] is False and "unoq-lyrical" in r["error"]
    assert not any(c[2].endswith("flash_mcu.py") for c in ran)


def test_a_unoq_flash_without_the_robot_config_writes_nothing(unoq_flash, prebuilt):
    ran, _, _ = unoq_flash
    profile(prebuilt, "unoq-jazzy", "unoq", "jazzy")
    r = hs.flash(env="unoq", port="/dev/ttyHS1", distro="jazzy", params_path=None)
    assert r["ok"] is False and "env block" in r["error"] and ran == []


def test_the_route_flashes_the_stacks_distro(monkeypatch):
    pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
    os.environ["COCKPIT_AUTH"] = "off"
    import routes_hardware as rh
    from fastapi import HTTPException
    monkeypatch.setattr(rh.mcu_identity, "image_distro", lambda: "lyrical")
    assert rh._flash_target("unoq") == ("unoq_lyrical", "lyrical")
    assert rh._flash_target("pico2") == ("pico2_lyrical", "lyrical")
    assert rh._flash_target("unoq", "jazzy") == ("unoq", "jazzy")
    monkeypatch.setattr(rh.mcu_identity, "env_for_distro",
                        lambda env, d: (_ for _ in ()).throw(ValueError("no 'lyrical' build")))
    with pytest.raises(HTTPException) as e:
        rh._flash_target("pico2")
    assert e.value.status_code == 400 and "lyrical" in e.value.detail
