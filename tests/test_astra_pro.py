"""The Orbbec Astra Pro as a depth camera: openni2_camera plus Orbbec's own OpenNI2 driver.

The original Astra Pro (depth 2bc5:0403 over OpenNI2, colour 2bc5:0501 as UVC) is not
covered by Orbbec's current ROS 2 driver, and its old one (ros2_astra_camera) declares
no license. ros-<distro>-openni2-camera is released for jazzy and lyrical, but Debian's
OpenNI2 has only PrimeSense's drivers: the robot fetches Orbbec's liborbbec.so from
Orbbec's own release (scripts/install_orbbec_openni2.py), and Debian's OpenNI2 loads it
only under a versioned name (2026-10-08: it took libPS1080.so.0, skipped liborbbec.so).
"""
import hashlib
import os
import re
import sys
import zipfile

import pytest

HERE = os.path.dirname(__file__)
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import depth_camera as dc  # noqa: E402
import install_orbbec_openni2 as inst  # noqa: E402

ASTRA = {"depth_camera": {"model": "astra_pro"}}


def _read(*parts):
    with open(os.path.join(ROOT, *parts)) as fh:
        return fh.read()


def test_the_astra_pro_is_an_openni2_camera():
    family, label, frame, depth, info = dc.DEPTH_MODELS["astra_pro"]
    assert family == "openni2" and frame == "camera_link"
    assert (depth, info) == ("/camera/depth/image_raw", "/camera/depth/camera_info")
    assert dc.DRIVER_PACKAGE["openni2"] == "openni2_camera"
    assert dc.depth_model(ASTRA) == "astra_pro"


def test_its_launch_file_is_ours_and_exists():
    pkg, rel, args = dc.driver_launch("astra_pro", "lyrical", "camera_link")
    assert pkg == "linorobot2_cockpit" and rel == "launchers/openni2_depth.launch.py"
    assert args == {"namespace": "camera", "camera_frame": "camera_link"}
    assert os.path.isfile(os.path.join(ROOT, rel))


def test_the_launch_file_turns_registration_off_and_names_frames_without_a_slash():
    src = _read("launchers", "openni2_depth.launch.py")
    assert '"depth_registration": False' in src, "an Astra Pro has no OpenNI colour stream"
    assert "depth_camera.openni2_frames(camera_frame)" in src
    frames = dc.openni2_frames
    assert frames("camera_link") == ("camera_depth_frame", "camera_depth_optical_frame")
    for name in frames("robot1/camera_link"):
        assert not name.startswith("/") and name.startswith("robot1/camera_depth")


def test_the_scan_uses_the_astra_pro_optics():
    assert dc.depth_optics(ASTRA) == (58.4, 0.6)
    assert dc.scan_range_min(ASTRA) == 0.6
    assert dc.scan_fov_deg(ASTRA) == pytest.approx(58.4)
    # the simulated camera is a D435 whatever the config names
    sim = {"depth_camera": {"model": "astra_pro"}, "sensors": {"use_sim_depth": True}}
    assert dc.depth_optics(sim) == pytest.approx((87.0, dc.SCAN_RANGE_MIN))
    # a model with no entry keeps the D435 figures
    assert dc.depth_optics({"depth_camera": {"model": "realsense"}}) == pytest.approx((87.0, 0.45))


def test_bringup_and_nav2_take_the_near_limit_from_the_model():
    assert "depth_camera.scan_range_min(controller)" in _read("launchers", "bringup.launch.py")
    assert "depth_camera.scan_range_min(" in _read("launchers", "nav2.launch.py")


def test_one_name_for_the_driver_everywhere():
    assert dc.ORBBEC_DRIVER == inst.DRIVER_NAME == "liborbbec.so.0"
    assert dc.ORBBEC_DRIVER_DIR == inst.SUBDIR.replace(os.sep, "/") == "drivers/openni2"
    entry = _read("docker", "entrypoint.sh")
    assert 'drivers/openni2/*.so.0' in entry and "/usr/lib/*/OpenNI2/Drivers" in entry
    assert "install_orbbec_openni2.py" in _read("launchers", "bringup.launch.py")


def test_only_the_astra_needs_orbbecs_driver():
    assert dc.orbbec_driver_missing(None) is False
    assert dc.orbbec_driver_missing("realsense") is False


def test_the_installer_pins_orbbecs_own_release():
    assert inst.SDK_URL.startswith("https://github.com/orbbec/OpenNI_SDK/releases/download/")
    for h in [inst.SDK_SHA256, inst.INI[1]] + [d[1] for d in inst.DRIVERS.values()]:
        assert re.fullmatch(r"[0-9a-f]{64}", h)
    assert set(inst.DRIVERS) == {"x86_64", "aarch64"}


def _fake_sdk(tmp_path, payload=b"\x7fELF fake driver"):
    z = tmp_path / "sdk.zip"
    with zipfile.ZipFile(z, "w") as zf:
        for member, _ in inst.DRIVERS.values():
            zf.writestr(member, payload)
        zf.writestr(inst.INI[0], b"[Device]\n")
    return str(z), payload


def test_the_installer_refuses_files_that_are_not_orbbecs(tmp_path):
    z, _ = _fake_sdk(tmp_path)
    with pytest.raises(SystemExit, match="refusing"):
        inst.extract(z, "x86_64")


def test_the_installer_keeps_the_driver_under_its_versioned_name(tmp_path, monkeypatch):
    z, payload = _fake_sdk(tmp_path)
    ok = hashlib.sha256(payload).hexdigest()
    monkeypatch.setattr(inst, "DRIVERS", {k: (m, ok) for k, (m, _) in inst.DRIVERS.items()})
    monkeypatch.setattr(inst, "INI", (inst.INI[0], hashlib.sha256(b"[Device]\n").hexdigest()))
    files = inst.extract(z, "aarch64")
    assert set(files) == {"liborbbec.so.0", "orbbec.ini"}
    cfg = tmp_path / "config"
    written = inst.install(files, str(cfg))
    assert (cfg / "drivers" / "openni2" / "liborbbec.so.0").read_bytes() == payload
    assert any(p.endswith("liborbbec.so.0") for p in written)
    with pytest.raises(SystemExit, match="no Orbbec OpenNI2 driver"):
        inst.extract(z, "riscv64")


def test_the_image_carries_openni2_camera():
    assert "ros-${ROS_DISTRO}-openni2-camera" in _read("docker", "Dockerfile")


def test_every_model_can_be_chosen_in_the_ui():
    html = _read("web", "frontend", "index.html")
    block = html[html.index('id="cfg-depth-camera"'):]
    block = block[:block.index("</select>")]
    offered = set(re.findall(r'<option value="([a-z0-9_]+)"', block)) - {"none"}
    assert offered == set(dc.DEPTH_MODELS)
