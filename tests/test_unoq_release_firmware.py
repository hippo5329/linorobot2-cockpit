"""The Arduino UNO Q firmware ships in the release, one image per distro, checked like the rest.

Its STM32U585 image is built with Zephyr, not PlatformIO, so build_prebuilt.py never made
it; scripts/package_unoq.py gives it the same archive and manifest shape, after the same
/cmd_vel contract check on its bytes. A lyrical UNO Q image once carried a fixed "jazzy"
stamp, subscribed Twist, and never moved under lyrical's Nav2.
"""
import json
import os
import sys
import tarfile

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import package_unoq  # noqa: E402
import build_prebuilt  # noqa: E402

BOTH = b"\0geometry_msgs/msg/Twist\0geometry_msgs/msg/TwistStamped\0"


def _build(tmp_path, stamp):
    z = tmp_path / "build-fw" / "zephyr"
    z.mkdir(parents=True)
    img = b"\0" * 64 + BOTH + f"\0FW_ROS_DISTRO={stamp}\0".encode() + b"\0" * 64
    (z / "zephyr.bin").write_bytes(img)
    (z / "zephyr.elf").write_bytes(b"\x7fELF" + img)
    return str(tmp_path / "build-fw")


@pytest.fixture
def prebuilt(tmp_path, monkeypatch):
    d = tmp_path / "prebuilt"
    d.mkdir()
    monkeypatch.setattr(build_prebuilt, "PREBUILT_DIR", str(d))
    return d


@pytest.mark.parametrize("distro", ["jazzy", "lyrical"])
def test_a_matching_image_is_packaged_like_the_other_profiles(tmp_path, prebuilt, distro):
    m = package_unoq.package(_build(tmp_path, distro), distro, dist=str(tmp_path / "dist"))
    assert m["profile"] == f"unoq-{distro}" and m["ros_distro"] == distro
    assert m["env_partition"]["offset"] == "0x081FE000"
    assert {f["name"] for f in m["files"]} == {"firmware.bin", "firmware.elf"}
    assert all(len(f["sha256"]) == 64 for f in m["files"])
    with tarfile.open(tmp_path / "dist" / f"linorobot2-firmware-unoq-{distro}.tar.gz") as t:
        assert sorted(t.getnames()) == ["firmware.bin", "firmware.elf", "manifest.json"]
        assert json.load(t.extractfile("manifest.json"))["profile"] == f"unoq-{distro}"


def test_an_image_stamped_for_the_other_distro_is_refused(tmp_path, prebuilt):
    with pytest.raises(SystemExit) as e:
        package_unoq.package(_build(tmp_path, "jazzy"), "lyrical")
    assert "FW_ROS_DISTRO=jazzy" in str(e.value)
    assert not (prebuilt / "unoq-lyrical").exists()


def test_the_release_builds_both_distros_and_publishes_only_after_them():
    wf = yaml.safe_load(open(os.path.join(ROOT, ".github", "workflows", "release.yml")))
    job = wf["jobs"]["firmware-unoq"]
    assert sorted(job["strategy"]["matrix"]["distro"]) == ["jazzy", "lyrical"]
    steps = " ".join(str(s.get("run", "")) for s in job["steps"])
    assert "firmware/zephyr/unoq/build.sh" in steps and "MICROROS_DISTRO=${{ matrix.distro }}" in steps
    assert "scripts/package_unoq.py" in steps
    pub = wf["jobs"]["publish-firmware"]
    assert "firmware-unoq" in pub["needs"]
    green = [s for s in pub["steps"] if s.get("name") == "The firmware matrix must have been green"][0]
    assert "needs.firmware-unoq.result" in green["if"]


def test_a_missing_unoq_archive_points_at_the_zephyr_build_not_platformio():
    import fetch_prebuilt
    assert "firmware/zephyr/unoq/build.sh" in fetch_prebuilt.build_hint("unoq-lyrical")
    assert "MICROROS_DISTRO=lyrical" in fetch_prebuilt.build_hint("unoq-lyrical")
    assert fetch_prebuilt.build_hint("pico2-jazzy") == "python3 scripts/build_prebuilt.py pico2-jazzy"
