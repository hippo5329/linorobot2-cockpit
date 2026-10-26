"""The depth camera as a user meets it: one control, an Install button that installs, a status
that agrees with bringup, a health row that shows the camera working, and the Astra Pro's
launch parameters as the node receives them.

Review 2026-10-26 (four readers: engineer, web designer, robot builder, beginner) found the
Sensors tab with two depth-camera selects -- the saved one and one that fed the Bringup
summary, the Nav2 export and the Install tab; the Install endpoint reading a query parameter
the page never sent (`apt-get install -y ros-lyrical-` for every sensor); the page's Astra
status looser than bringup's; and nothing on the health card that read the camera.
"""
import asyncio
import os
import re
import sys
import types

import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(ROOT, "web", "backend"))

import depth_camera as dc  # noqa: E402


def _read(*parts):
    with open(os.path.join(ROOT, *parts)) as fh:
        return fh.read()


# ---- one control -------------------------------------------------------------------------

def test_the_sensors_tab_has_one_depth_camera_select():
    html = _read("web", "frontend", "index.html")
    assert html.count('id="cfg-depth-camera"') == 1
    assert 'id="cfg-depth-sensor"' not in html
    for js in ("app-setup.js", "app-hardware.js", "app-studio.js"):
        assert "cfg-depth-sensor" not in _read("web", "frontend", js), js


def test_the_bringup_summary_reads_the_saved_select():
    src = _read("web", "frontend", "app-hardware.js")
    body = src[src.index("function updateBringupSummary"):]
    body = body[:body.index("\n}\n")]
    assert 'getElementById("cfg-depth-camera")' in body


def test_the_astra_labels_say_which_to_pick():
    html = _read("web", "frontend", "index.html")
    assert re.search(r'value="astra_pro">[^<]*recommended', html)
    assert re.search(r'value="astra_pro_openni2">[^<]*depth only[^<]*fallback', html)


# ---- the Install button ------------------------------------------------------------------

import system_utils as su  # noqa: E402


def test_every_install_names_a_package_or_a_step():
    for key in list(su.LASER_SENSORS) + list(su.DEPTH_SENSORS):
        entry = su.DEPTH_SENSORS.get(key) or su.LASER_SENSORS[key]
        keys = entry.get("install_by_model") or [key]
        for k in keys:
            cmd = su.build_sensor_install_cmd(k, distro="jazzy")["command"]
            assert cmd, k
            assert not re.search(r"ros-jazzy-(\s|&&|$)", cmd), (k, cmd)
            assert "$ROS_DISTRO" not in cmd and "$REPO_ROOT" not in cmd


def test_the_astra_pro_installs_nothing_and_its_fallback_runs_the_installer():
    pro = su.build_sensor_install_cmd("astra_pro", skip_udev=True)["command"]
    assert pro.startswith("echo ") and "built into" in pro
    fb = su.build_sensor_install_cmd("astra_pro_openni2", skip_udev=True)["command"]
    assert fb == f"python3 {su.REPO_ROOT}/scripts/install_orbbec_openni2.py"
    assert "docker exec" not in fb, "the step runs inside the cockpit container already"
    # the family alone does not say which driver
    fam = su.build_sensor_install_cmd("astra")
    assert fam["command"] == "" and "astra_pro_openni2" in fam["detail"]


def test_udev_steps_follow_the_checkboxes():
    full = su.build_sensor_install_cmd("astra_pro_openni2")["command"]
    assert "56-orbbec-usb.rules" in full and "install_orbbec_openni2.py" in full
    only = su.build_sensor_install_cmd("astra_pro_openni2", udev_only=True)["command"]
    assert "56-orbbec-usb.rules" in only and "install_orbbec_openni2.py" not in only


def test_the_install_tab_lists_the_astra_models():
    reg = su.sensor_registry()["depth"]["astra"]
    assert reg["install_by_model"] == ["astra_pro", "astra_pro_openni2"]
    assert reg["has_install"]


class _Req:
    method = "POST"

    def __init__(self, body):
        self._body = body

    async def json(self):
        return self._body


def test_the_endpoint_reads_what_the_page_sends():
    pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
    os.environ["COCKPIT_AUTH"] = "off"
    import actions
    import routes_hardware
    # app-setup.js fetchSensorInstallCmd's body, as sent
    body = {"kind": "depth", "key": "astra_pro_openni2", "skip_udev": True, "udev_only": False,
            "workspace_path": "/ws"}
    res = asyncio.run(routes_hardware.api_sensor_install_cmd(_Req(body)))
    assert "install_orbbec_openni2.py" in res["command"]
    assert actions.claim(res["handle"]) == res["command"]


# ---- the status agrees with bringup ------------------------------------------------------

def test_the_astra_status_is_bringups_test(monkeypatch):
    monkeypatch.setattr(dc, "astra_driver_missing", lambda ws=None: True)
    assert su.get_sensor_driver_status("astra_pro")["installed"] is False
    monkeypatch.setattr(dc, "astra_driver_missing", lambda ws=None: False)
    assert su.get_sensor_driver_status("astra_pro")["installed"] is True


def test_a_missing_driver_says_what_to_do(monkeypatch):
    monkeypatch.setattr(dc, "astra_driver_missing", lambda ws=None: True)
    fix = su.get_sensor_driver_status("astra_pro")["fix"]
    assert "pull the latest image" in fix and "Install tab" not in fix, "its Install step installs nothing"
    monkeypatch.setattr(su, "ros_pkg_installed", lambda pkg, *a, **k: False)
    assert "Install tab, step 3" in su.get_sensor_driver_status("astra_pro_openni2")["fix"]
    monkeypatch.setattr(dc, "astra_driver_missing", lambda ws=None: False)
    assert su.get_sensor_driver_status("astra_pro")["fix"] == ""
    js = _read("web", "frontend", "app-setup.js")
    body = js[js.index("async function refreshDepthCameraStatus"):]
    assert "d.fix" in body[:body.index("\n}\n")]


def test_the_fallback_is_not_installed_without_orbbecs_openni2(monkeypatch):
    monkeypatch.setattr(su, "ros_pkg_installed", lambda pkg, *a, **k: True)
    monkeypatch.setattr(dc, "orbbec_driver_missing", lambda model, config_dir=None: True)
    assert su.get_sensor_driver_status("astra_pro_openni2")["installed"] is False
    monkeypatch.setattr(dc, "orbbec_driver_missing", lambda model, config_dir=None: False)
    assert su.get_sensor_driver_status("astra_pro_openni2")["installed"] is True


# ---- the health card shows the camera ----------------------------------------------------

def test_health_rows_for_a_camera_beside_a_lidar():
    import runners
    ctrl = {"lidar": {"model": "ld19"}, "depth_camera": {"model": "astra_pro"}}
    rows = {r[0]: r for r in runners.depth_health_rows(ctrl)}
    assert rows["depth_image"][1] == "/camera/depth/image_raw"
    assert rows["camera_scan"][1] == "/camera/scan"
    assert set(rows) <= set(runners.BRINGUP_HEALTH_TYPES)


def test_health_rows_for_a_camera_that_is_the_scan():
    import runners
    rows = {r[0] for r in runners.depth_health_rows({"lidar": {"model": "none"},
                                                     "depth_camera": {"model": "astra_pro"}})}
    assert rows == {"depth_image"}, "its scan is /scan, which has its own row"


def test_no_camera_no_rows_and_the_simulated_one_has_no_image():
    import runners
    assert runners.depth_health_rows({"depth_camera": {"model": "none"}}) == []
    sim = {"lidar": {"model": "ld19"}, "sensors": {"use_sim_depth": True}}
    assert {r[0] for r in runners.depth_health_rows(sim)} == {"camera_scan"}


# ---- the Astra Pro's launch parameters, as the node gets them ----------------------------

def test_the_astra_node_gets_registration_off_and_static_tf(monkeypatch):
    captured = {}

    class Node:
        def __init__(self, **kw):
            captured.update(kw)

    fake = {
        "launch": types.SimpleNamespace(LaunchDescription=lambda a: a),
        "launch.actions": types.SimpleNamespace(DeclareLaunchArgument=lambda *a, **k: None,
                                                OpaqueFunction=lambda **k: None),
        "launch_ros": types.SimpleNamespace(),
        "launch_ros.actions": types.SimpleNamespace(Node=Node),
    }
    for name, mod in fake.items():
        monkeypatch.setitem(sys.modules, name, mod)
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "astra_launch_under_test", os.path.join(ROOT, "launchers", "astra_camera_depth.launch.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = types.SimpleNamespace(launch_configurations={"namespace": "camera", "camera_frame": "camera_link"})
    monkeypatch.setenv("LD_LIBRARY_PATH", "/opt/ros/lib")
    mod._actions(ctx)
    p = captured["parameters"][0]
    assert p["depth_registration"] is False, "registration on makes the depth camera_info NaN"
    assert p["tf_publish_rate"] == 0.0 and p["publish_tf"] is True
    assert p["enable_ir"] is False and p["enable_point_cloud"] is False
    assert p["camera_link_frame_id"] == "camera_link"
    assert (p["depth_frame_id"], p["depth_optical_frame_id"]) == dc.stream_frames("camera_link", "depth")
    assert p["color_info_url"] == ""
    lib = captured["additional_env"]["LD_LIBRARY_PATH"].split(":")
    assert lib[0] == os.path.join(dc.ASTRA_WS, "lib"), "Orbbec's OpenNI2 must come before Debian's"
    assert captured["executable"].startswith(dc.ASTRA_WS)
