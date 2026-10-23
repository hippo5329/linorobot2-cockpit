"""The Orbbec Astra Pro as a depth camera, on either of its two drivers.

The original Astra Pro (depth 2bc5:0403 over OpenNI2, colour 2bc5:0501 as UVC).
`astra_pro`: Orbbec's ros2_astra_camera, built into the robot image from a pinned
upstream commit (docker/Dockerfile stage astra, docker/astra_camera_fixes.sh) --
colour, depth and intrinsics at 30 Hz on a real camera (2026-10-22). Orbbec's current
ROS 2 driver, OrbbecSDK_ROS2, has no colour for it and misreads its depth.
`astra_pro_openni2`: depth only, on ros-<distro>-openni2-camera. Debian's OpenNI2 has
only PrimeSense's drivers, and Orbbec's liborbbec.so segfaults inside it (2026-10-10),
so the robot fetches Orbbec's SDK release (scripts/install_orbbec_openni2.py) and the
driver node runs on the SDK's own libOpenNI2, put first on its LD_LIBRARY_PATH.
"""
import subprocess
import hashlib
import math
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
OPENNI = {"depth_camera": {"model": "astra_pro_openni2"}}


def _read(*parts):
    with open(os.path.join(ROOT, *parts)) as fh:
        return fh.read()


def test_the_astra_pro_runs_on_orbbecs_astra_camera():
    family, label, frame, depth, info = dc.DEPTH_MODELS["astra_pro"]
    assert family == "astra" and frame == "camera_link"
    # ros2_astra_camera: depth/image_raw (16UC1, mm) under the namespace
    assert (depth, info) == ("/camera/depth/image_raw", "/camera/depth/camera_info")
    assert dc.DRIVER_PACKAGE["astra"] == "astra_camera"
    assert dc.depth_model(ASTRA) == "astra_pro"
    pkg, rel, args = dc.driver_launch("astra_pro", "jazzy", "camera_link")
    assert pkg == "linorobot2_cockpit" and rel == "launchers/astra_camera_depth.launch.py"
    assert args == {"namespace": "camera", "camera_frame": "camera_link"}
    assert os.path.isfile(os.path.join(ROOT, rel))


def test_the_astra_launch_file_runs_the_image_driver_on_its_own_openni2():
    src = _read("launchers", "astra_camera_depth.launch.py")
    assert 'executable=os.path.join(ws, "lib", "astra_camera", "astra_camera_node")' in src
    assert '"LD_LIBRARY_PATH": _path(os.path.join(ws, "lib"), os.environ.get("LD_LIBRARY_PATH", ""))' in src
    # the UVC colour camera by id, as ints (launch types the package file's hex values so)
    assert '"uvc_vendor_id": 0x2BC5' in src and '"uvc_product_id": 0x0501' in src
    assert '"use_uvc_camera": True' in src and '"enable_depth": True' in src
    assert '"enable_ir": False' in src and '"enable_point_cloud": False' in src
    assert '"camera_link_frame_id": camera_frame' in src
    assert dc.ASTRA_WS == "/opt/astra_ws/install"
    assert dc.stream_frames("robot1/camera_link", "color") == (
        "robot1/camera_color_frame", "robot1/camera_color_optical_frame")


def test_the_astra_driver_is_missing_until_the_image_carries_it(tmp_path):
    assert dc.astra_driver_missing(str(tmp_path)) is True
    node = tmp_path / "lib" / "astra_camera" / "astra_camera_node"
    node.parent.mkdir(parents=True)
    node.write_text("#!/bin/sh\n")
    node.chmod(0o755)
    assert dc.astra_driver_missing(str(tmp_path)) is True, "no OpenNI2 beside it"
    (tmp_path / "lib" / "libOpenNI2.so").write_bytes(b"x")
    assert dc.astra_driver_missing(str(tmp_path)) is False
    launch = _read("launchers", "bringup.launch.py")
    assert "depth_camera.astra_driver_missing()" in launch


def test_the_image_builds_orbbecs_driver_at_a_pinned_commit():
    df = _read("docker", "Dockerfile")
    assert "AS astra" in df
    assert re.search(r"ARG ASTRA_CAMERA_COMMIT=[0-9a-f]{40}\n", df)
    assert "https://github.com/orbbec/ros2_astra_camera.git" in df
    assert "bash /tmp/astra_camera_fixes.sh /opt/astra_ws/src/ros2_astra_camera" in df
    assert "COPY --from=astra /opt/astra_ws/install /opt/astra_ws/install" in df
    assert "libuvc-dev" in df


def _humble_tree(root):
    """The lines of ros2_astra_camera f7e71d9 that astra_camera_fixes.sh changes, in its layout."""
    pkg = root / "astra_camera"
    (pkg / "include" / "astra_camera").mkdir(parents=True)
    (pkg / "src" / "point_cloud_proc").mkdir(parents=True)
    (pkg / "include" / "astra_camera" / "constants.h").write_text("#pragma once\n#include <string>\n")
    (pkg / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.5)\nfind_package(ament_cmake REQUIRED)\n"
        "ament_target_dependencies(${PROJECT_NAME}\n  ${dependencies}\n  )\n")
    (pkg / "src" / "a.cpp").write_text(
        "#include <cv_bridge/cv_bridge.h>\n#include <image_geometry/pinhole_camera_model.h>\n"
        "#include <message_filters/sync_policies/approximate_time.h>\n"
        "#include <tf2/LinearMath/Quaternion.h>\n#include <tf2_ros/static_transform_broadcaster.h>\n"
        "NodeParametersInterface::OnParametersSetCallbackType cb;\n")
    (pkg / "src" / "point_cloud_proc" / "point_cloud_xyzrgb.cpp").write_text(
        '  sub_info_.subscribe(node_, "color/camera_info", info_qos_profile_);\n')
    return pkg


def _fix(root, distro):
    fixes = os.path.join(ROOT, "docker", "astra_camera_fixes.sh")
    env = {**os.environ, "ROS_DISTRO": distro}
    subprocess.run(["bash", fixes, str(root)], check=True, capture_output=True, env=env)


@pytest.mark.parametrize("distro", ["jazzy", "lyrical"])
def test_the_fixes_make_the_humble_sources_build_on_both_distros(tmp_path, distro):
    pkg = _humble_tree(tmp_path)
    _fix(tmp_path, distro)
    a = (pkg / "src" / "a.cpp").read_text()
    for header in ("cv_bridge/cv_bridge.hpp", "image_geometry/pinhole_camera_model.hpp",
                   "message_filters/sync_policies/approximate_time.hpp",
                   "tf2/LinearMath/Quaternion.hpp", "tf2_ros/static_transform_broadcaster.hpp"):
        assert f"<{header}>" in a, header
    assert "OnSetParametersCallbackType" in a and "OnParametersSetCallbackType" not in a
    assert "#include <cstdint>" in (pkg / "include" / "astra_camera" / "constants.h").read_text()
    cmake = (pkg / "CMakeLists.txt").read_text()
    # the shim sits right after find_package(ament_cmake) and only defines a missing command
    assert cmake.index("find_package(ament_cmake REQUIRED)") < cmake.index("if(NOT COMMAND ament_target_dependencies)")
    assert cmake.count("lino: ament_target_dependencies shim") == 1
    sub = (pkg / "src" / "point_cloud_proc" / "point_cloud_xyzrgb.cpp").read_text()
    if distro == "jazzy":   # Jazzy's message_filters takes the rmw profile, and only that
        assert 'subscribe(node_, "color/camera_info", info_qos_profile_);' in sub
    else:                   # Lyrical's takes only an rclcpp::QoS
        assert ("rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(info_qos_profile_), "
                "info_qos_profile_)") in sub
    # a second run (a cached layer re-run) changes nothing and still passes
    before = {f: f.read_text() for f in pkg.rglob("*") if f.is_file()}
    _fix(tmp_path, distro)
    assert {f: f.read_text() for f in pkg.rglob("*") if f.is_file()} == before


def test_the_openni2_fallback_is_an_openni2_camera():
    family, label, frame, depth, info = dc.DEPTH_MODELS["astra_pro_openni2"]
    assert family == "openni2" and frame == "camera_link"
    # openni2_camera 2.x: depth_raw/image (16UC1, mm); there is no depth/image_raw
    assert (depth, info) == ("/camera/depth_raw/image", "/camera/depth_raw/camera_info")
    assert dc.DRIVER_PACKAGE["openni2"] == "openni2_camera"
    assert dc.depth_model(OPENNI) == "astra_pro_openni2"


def test_its_launch_file_is_ours_and_exists():
    pkg, rel, args = dc.driver_launch("astra_pro_openni2", "lyrical", "camera_link")
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
    assert dc.depth_optics(OPENNI) == (58.4, 0.6)
    assert dc.depth_optics(ASTRA) == (58.4, 0.6)
    assert dc.scan_range_min(ASTRA) == 0.6
    assert dc.scan_fov_deg(ASTRA) == pytest.approx(58.4)
    # the simulated camera renders the configured model, so its figures hold in simulation too
    sim = {"depth_camera": {"model": "astra_pro"}, "sensors": {"use_sim_depth": True}}
    assert dc.depth_optics(sim) == (58.4, 0.6)
    # a model with no entry keeps the D435 figures
    assert dc.depth_optics({"depth_camera": {"model": "realsense"}}) == pytest.approx((87.0, 0.45))


def test_bringup_and_nav2_take_the_near_limit_from_the_model():
    assert "depth_camera.scan_range_min(controller)" in _read("launchers", "bringup.launch.py")
    assert "depth_camera.scan_range_min(" in _read("launchers", "nav2.launch.py")


def test_one_place_for_orbbecs_openni2_everywhere():
    assert dc.ORBBEC_SDK_DIR == inst.SDK_DIR.replace(os.sep, "/") == "drivers/openni2/sdk"
    for arch, files in inst.FILES.items():
        assert set(dc.ORBBEC_SDK_FILES) <= set(files), arch
    assert dc.orbbec_sdk_dir("/cfg") == "/cfg/drivers/openni2/sdk"
    assert "install_orbbec_openni2.py" in _read("launchers", "bringup.launch.py")


def test_nothing_goes_into_debians_openni2_any_more():
    """Orbbec's driver in Debian's Drivers directory is what crashed the device open."""
    assert "OpenNI2/Drivers" not in _read("docker", "entrypoint.sh")
    assert "/usr/lib" not in _read("scripts", "install_orbbec_openni2.py").split('"""', 2)[2]


def test_the_driver_node_runs_on_orbbecs_libopenni2():
    src = _read("launchers", "openni2_depth.launch.py")
    assert 'additional_env={"LD_LIBRARY_PATH": ld_path}' in src
    assert "sdk = depth_camera.orbbec_sdk_dir()" in src
    assert 'ld_path = ":".join(p for p in (sdk, os.environ.get("LD_LIBRARY_PATH", "")) if p)' in src


def test_only_the_astra_needs_orbbecs_driver(tmp_path):
    assert dc.orbbec_driver_missing(None) is False
    assert dc.orbbec_driver_missing("realsense") is False
    assert dc.orbbec_driver_missing("astra_pro", str(tmp_path)) is False, "Orbbec's driver carries its own"
    assert dc.orbbec_driver_missing("astra_pro_openni2", str(tmp_path)) is True
    sdk = tmp_path / "drivers" / "openni2" / "sdk"
    for f in dc.ORBBEC_SDK_FILES[:-1]:
        (sdk / f).parent.mkdir(parents=True, exist_ok=True)
        (sdk / f).write_bytes(b"x")
    assert dc.orbbec_driver_missing("astra_pro_openni2", str(tmp_path)) is True, "a part is missing"
    (sdk / dc.ORBBEC_SDK_FILES[-1]).parent.mkdir(parents=True, exist_ok=True)
    (sdk / dc.ORBBEC_SDK_FILES[-1]).write_bytes(b"x")
    assert dc.orbbec_driver_missing("astra_pro_openni2", str(tmp_path)) is False


def test_the_installer_pins_orbbecs_own_release():
    assert inst.SDK_URL.startswith("https://github.com/orbbec/OpenNI_SDK/releases/download/")
    assert re.fullmatch(r"[0-9a-f]{64}", inst.SDK_SHA256)
    assert set(inst.FILES) == {"x86_64", "aarch64"}
    for files in inst.FILES.values():
        for _, h in files.values():
            assert re.fullmatch(r"[0-9a-f]{64}", h)
    # the libraries differ per CPU; the settings files are shared
    x, a = inst.FILES["x86_64"], inst.FILES["aarch64"]
    assert x["libOpenNI2.so.0"] != a["libOpenNI2.so.0"]
    assert x["OpenNI2/Drivers/liborbbec.so"] != a["OpenNI2/Drivers/liborbbec.so"]
    assert x["OpenNI.ini"] == a["OpenNI.ini"]


def _fake_sdk(tmp_path, payload=b"\x7fELF fake"):
    z = tmp_path / "sdk.zip"
    members = {m for files in inst.FILES.values() for m, _ in files.values()}
    with zipfile.ZipFile(z, "w") as zf:
        for member in sorted(members):
            zf.writestr(member, payload)
    return str(z), payload


def test_the_installer_refuses_files_that_are_not_orbbecs(tmp_path):
    z, _ = _fake_sdk(tmp_path)
    with pytest.raises(SystemExit, match="refusing"):
        inst.extract(z, "x86_64")


def test_the_installer_lays_out_the_sdk_with_the_versioned_library(tmp_path, monkeypatch):
    z, payload = _fake_sdk(tmp_path)
    ok = hashlib.sha256(payload).hexdigest()
    monkeypatch.setattr(inst, "FILES", {k: {n: (m, ok) for n, (m, _) in f.items()}
                                        for k, f in inst.FILES.items()})
    files = inst.extract(z, "aarch64")
    assert set(files) == {"libOpenNI2.so.0", "OpenNI.ini", "OpenNI2/Drivers/liborbbec.so",
                          "OpenNI2/Drivers/orbbec.ini"}
    cfg = tmp_path / "config"
    inst.install(files, str(cfg))
    inst.install(files, str(cfg))          # a second run replaces, never fails on the link
    sdk = cfg / "drivers" / "openni2" / "sdk"
    assert (sdk / "OpenNI2" / "Drivers" / "liborbbec.so").read_bytes() == payload
    assert os.readlink(sdk / "libOpenNI2.so") == "libOpenNI2.so.0"
    assert dc.orbbec_driver_missing("astra_pro_openni2", str(cfg)) is False
    with pytest.raises(SystemExit, match="no Orbbec OpenNI2 for CPU"):
        inst.extract(z, "riscv64")


def test_the_image_carries_openni2_camera():
    assert "ros-${ROS_DISTRO}-openni2-camera" in _read("docker", "Dockerfile")


def test_every_model_can_be_chosen_in_the_ui():
    html = _read("web", "frontend", "index.html")
    block = html[html.index('id="cfg-depth-camera"'):]
    block = block[:block.index("</select>")]
    offered = set(re.findall(r'<option value="([a-z0-9_]+)"', block)) - {"none"}
    assert offered == set(dc.DEPTH_MODELS)


def test_the_simulated_camera_is_the_astra_pro_when_the_robot_has_one():
    """A simulated Astra sees what the real one will: 58.4 x 45.5 deg, 0.6-8 m,
    640x480 at 30 Hz -- and nothing at all inside 0.6 m."""
    pytest.importorskip("numpy")
    sim = {"depth_camera": {"model": "astra_pro"}, "sensors": {"use_sim_depth": True}}
    assert dc.sim_camera_key(sim) == "astra_pro"
    assert dc.sim_camera_key({"depth_camera": {"model": "astra_pro_openni2"}}) == "astra_pro"
    assert dc.sim_camera_key({"sensors": {"use_sim_depth": True}}) == "d435"
    assert dc.sim_camera_key({"depth_camera": {"model": "realsense"}}) == "d435"
    cam = dc.SIM_CAMERAS["astra_pro"]
    assert (cam.width, cam.height, cam.rate_hz) == (640, 480, 30.0)
    angles = dc.column_angles(cam)
    assert math.degrees(angles[0] - angles[-1]) == pytest.approx(58.4, abs=0.2)
    segs = dc.room_segments(dc.sim_room({}))
    # the obstacle wall 2 m ahead is seen; from 0.5 m away it vanishes, where the D435 still reads it
    far = dc.depth_row(0.0, 0.0, 0.0, segs, angles, cam=cam)
    near = dc.depth_row(1.5, 0.0, 0.0, segs, angles, cam=cam)
    d435 = dc.depth_row(1.5, 0.0, 0.0, segs, dc.column_angles())
    centre = cam.width // 2
    assert far[centre] == pytest.approx(2.0)
    assert near[centre] == 0.0
    assert d435[len(d435) // 2] == pytest.approx(0.5)


def test_bringup_tells_the_simulated_camera_which_camera_to_be():
    launch = _read("launchers", "bringup.launch.py")
    assert '"camera": cam_key' in launch
    assert "depth_camera.SIM_CAMERAS[cam_key].rate_hz" in launch
    node = _read("scripts", "sim_depth_node.py")
    assert 'declare_parameter("camera", "d435")' in node
