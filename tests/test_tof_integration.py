"""The 8x8 ToF (base_controller.tof) end to end: config, scan source, env, description,
Nav2 sources, bringup, the USB "Pico" guard and the firmware's I2C driver."""
import ast
import copy
import math
import os
import re

import pytest
import yaml

import depth_camera
import gen_robot_description as grd
import mcu_env
import tof_matrix as tm

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(REPO, "firmware")


def _read(*p):
    with open(os.path.join(REPO, *p)) as f:
        return f.read()


def _ref(name="pico2_mecanum"):
    with open(os.path.join(REPO, "config", "reference", f"{name}_config.yaml")) as f:
        return yaml.safe_load(f)


def _no_lidar(params):
    bc = params["base_controller"]
    bc["lidar"] = {"model": "none"}
    bc.setdefault("sensors", {})["use_sim_ld19"] = False
    bc["sensors"]["use_sim_depth"] = False
    bc.pop("depth_camera", None)
    return params


def _with_tof(params, **tof):
    t = {"model": "sen0628", "connection": "i2c", "i2c_address": 0x33}
    t.update(tof)
    params["base_controller"]["tof"] = t
    return params


# -- config ---------------------------------------------------------------------------
def test_no_tof_block_is_no_tof():
    assert tm.tof_settings({}) is None
    assert tm.tof_settings({"tof": {"model": "none"}}) is None


def test_a_usb_tof_needs_its_port():
    with pytest.raises(ValueError, match="usb_port"):
        tm.tof_settings({"lidar": {"model": "none"}, "tof": {"model": "sen0628", "connection": "usb"}})


@pytest.mark.parametrize("bad", [{"i2c_address": 0x29}, {"rotation": 45}, {"model": "vl53l5"},
                                 {"connection": "spi"}])
def test_a_half_configured_tof_is_refused(bad):
    t = {"model": "sen0628", "connection": "i2c", "i2c_address": 0x33}
    t.update(bad)
    with pytest.raises(ValueError):
        tm.tof_settings({"lidar": {"model": "none"}, "tof": t})


def test_a_tof_beside_a_lidar_is_refused_everywhere():
    """Both are infrared time-of-flight and blind each other (user, 2026-11-11)."""
    params = _with_tof(_ref())                 # pico2_mecanum has an LD19
    bc = params["base_controller"]
    with pytest.raises(ValueError, match="LiDAR"):
        tm.tof_settings(bc)
    with pytest.raises(ValueError, match="LiDAR"):
        depth_camera.scan_source(bc)            # bringup and the pipeline both ask this


def test_the_simulated_ld19_counts_as_a_lidar():
    bc = {"name": "sim", "sensors": {"use_sim_ld19": True},
          "tof": {"model": "sen0628", "connection": "i2c"}}
    with pytest.raises(ValueError, match="LiDAR"):
        tm.tof_settings(bc)


# -- who makes /scan --------------------------------------------------------------------
def test_with_no_lidar_and_no_camera_the_tof_is_the_scan():
    bc = _with_tof(_no_lidar(_ref()))["base_controller"]
    assert depth_camera.scan_source(bc) == "tof"
    assert depth_camera.scan_fov_deg(bc) == tm.FOV_DEG


def test_a_depth_camera_is_the_scan_before_the_tof():
    params = _with_tof(_no_lidar(_ref()))
    params["base_controller"]["depth_camera"] = {"model": "realsense"}
    assert depth_camera.scan_source(params["base_controller"]) == "depth"


def test_the_tof_scan_takes_the_narrow_fov_slam_overrides():
    params = _with_tof(_no_lidar(_ref()))
    params.setdefault("slam", {})["narrow_fov_overrides"] = {"minimum_travel_heading": 0.1}
    assert depth_camera.narrow_slam_overrides(params) == {"minimum_travel_heading": 0.1}


def test_scan_ranges_run_right_to_left_along_the_beam():
    f = [4000] * 64
    f[3 * 8 + 7] = 1000          # row 3, rightmost column
    f[4 * 8 + 0] = 2000          # row 4, leftmost column
    r = tm.to_scan_ranges(f)
    amin, inc = tm.scan_angles()
    assert amin == pytest.approx(math.radians(-26.25)) and inc == pytest.approx(math.radians(7.5))
    assert r[0] == pytest.approx(1.0 / math.cos(math.radians(26.25)))    # first beam: the right edge
    assert r[-1] == pytest.approx(2.0 / math.cos(math.radians(26.25)))
    assert all(math.isinf(x) for x in r[1:-1])


def test_scan_ignores_rows_off_the_horizon():
    f = [4000] * 64
    f[0] = 500                   # top row: not part of the scan
    assert all(math.isinf(x) for x in tm.to_scan_ranges(f))


# -- the firmware's env -------------------------------------------------------------------
def test_tof_addr_is_written_for_an_i2c_tof_only():
    p = _with_tof(_no_lidar(_ref()), i2c_address=0x31)
    assert mcu_env.hardware_env(p)["tof_addr"] == 0x31
    p = _with_tof(_no_lidar(_ref()), connection="usb", usb_port="/dev/serial/by-id/usb-Raspberry_Pi_Pico_X-if00")
    assert mcu_env.hardware_env(p)["tof_addr"] == 0
    assert mcu_env.hardware_env(_ref())["tof_addr"] == 0


def test_sim_mode_leaves_the_real_tof_alone_and_simulates_no_lidar(tmp_path):
    p = _with_tof(_no_lidar(_ref()))
    path = tmp_path / "robot_config.yaml"
    path.write_text(yaml.safe_dump(p))
    env = mcu_env.hardware_env(p)
    env.setdefault("sim_ld19", "0")
    mcu_env.apply_sensor_mode(env, "sim", params_path=str(path))
    assert env["tof_addr"] == 0
    assert env["sim_ld19"] == "0"


def test_the_firmware_reads_the_key_it_is_written():
    assert 'envInt("tof_addr"' in _read("firmware", "src", "main.cpp")


# -- the robot description --------------------------------------------------------------
def test_tof_link_only_on_a_robot_with_one():
    assert "tof_link" not in grd.build_urdf(_ref())
    urdf = grd.build_urdf(_with_tof(_no_lidar(_ref())))
    assert '<link name="tof_link"' in urdf and 'child link="tof_link"' in urdf


def test_ground_clearance_is_the_wheel_radius_less_its_z():
    p = _ref()
    assert grd.ground_clearance_m(p) == pytest.approx(p["kinematics"]["wheel_diameter"] / 2.0
                                                      - grd.effective_geometry(p)["wheel"]["z"])


# -- Nav2 -----------------------------------------------------------------------------
def _launcher_funcs(*names):
    """Lift functions out of nav2.launch.py into one namespace (it imports `launch`)."""
    path = os.path.join(REPO, "launchers", "nav2.launch.py")
    tree = ast.parse(_read("launchers", "nav2.launch.py"))
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert len(nodes) == len(names), f"missing from nav2.launch.py: {names}"
    ns = {}
    exec(compile(ast.Module(nodes, []), path, "exec"), ns)
    return ns


def _nav2(params):
    return copy.deepcopy(params["nav2"])


def test_the_tof_joins_the_costmaps_and_the_monitor_with_the_floor_cut():
    f = _launcher_funcs("footprint_reach", "add_tof_obstacle_source")["add_tof_obstacle_source"]
    params = _with_tof(_no_lidar(_ref()), floor_clearance=0.05, max_height=0.5)
    nav2 = _nav2(params)
    changed = f(nav2, params)
    assert "collision_monitor" in changed and any(c.startswith("local_costmap") for c in changed)
    clear = grd.ground_clearance_m(params)
    lc = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    layer = next(lc[l] for l in lc["plugins"] if isinstance(lc.get(l), dict) and "tof" in lc[l])
    src = layer["tof"]
    assert src["data_type"] == "PointCloud2" and src["topic"] == "/tof/points"
    assert src["min_obstacle_height"] == pytest.approx(0.05 - clear, abs=1e-3)
    assert src["max_obstacle_height"] == pytest.approx(0.5 - clear, abs=1e-3)
    if "z_voxels" in layer:
        assert float(layer["origin_z"]) <= -clear + 1e-6, "the voxel grid must reach the floor"
    cm = nav2["collision_monitor"]["ros__parameters"]
    assert cm["tof"]["type"] == "pointcloud" and cm["tof"]["min_height"] == pytest.approx(0.05 - clear, abs=1e-3)
    assert {"TofSlow", "TofStop"} <= set(cm["polygons"])
    for p in cm["polygons"]:
        if p in ("TofSlow", "TofStop"):
            assert cm[p]["sources_names"] == ["tof"]
        elif isinstance(cm.get(p), dict) and cm[p].get("sources_names") is not None:
            assert "tof" not in cm[p]["sources_names"], f"{p}: a bumper sensor in a footprint polygon freezes the robot"


def test_a_tof_that_is_the_scan_is_not_heard_twice():
    f = _launcher_funcs("footprint_reach", "add_tof_obstacle_source")["add_tof_obstacle_source"]
    params = _with_tof(_no_lidar(_ref()))
    nav2 = _nav2(params)
    f(nav2, params)
    cm = nav2["collision_monitor"]["ros__parameters"]
    assert "scan" not in cm["observation_sources"] and "tof" in cm["observation_sources"]
    for p in cm["polygons"]:
        assert "scan" not in (cm[p].get("sources_names") or [])


def test_no_tof_no_source():
    f = _launcher_funcs("footprint_reach", "add_tof_obstacle_source")["add_tof_obstacle_source"]
    params = _ref()
    nav2 = _nav2(params)
    before = copy.deepcopy(nav2)
    assert f(nav2, params) == [] and nav2 == before


def test_nav2_launch_adds_it():
    assert "add_tof_obstacle_source(nav2_data, params)" in _read("launchers", "nav2.launch.py")


# -- bringup and the pipeline ----------------------------------------------------------------
def test_bringup_starts_the_node_and_declares_sim_tof():
    src = _read("launchers", "bringup.launch.py")
    assert "nodes += tof_actions(" in src and '"sim_tof"' in src
    assert '"tof_matrix_node.py"' in src
    assert '"scan_topic": "scan" if scan_from == "tof" else ""' in src


def test_the_pipeline_simulates_it_in_sim_mode():
    src = _read("scripts", "one_click_pipeline.py")
    assert 'sim_tof:=true' in src


# -- the USB "Pico" ------------------------------------------------------------------
def test_a_configured_usb_tof_is_claimed(tmp_path):
    d = tmp_path / "cfg"
    d.mkdir()
    p = _with_tof(_no_lidar(_ref()), connection="usb",
                  usb_port="/dev/serial/by-id/usb-Raspberry_Pi_Pico_0123456789ABCDEF-if00")
    (d / "bot_config.yaml").write_text(yaml.safe_dump(p))
    assert tm.claimed_serials(str(d)) == {"0123456789abcdef"}
    assert tm.usb_serial("/dev/serial/by-id/usb-Raspberry_Pi_Pico_0123456789ABCDEF-if00") == "0123456789ABCDEF"


def test_flash_and_detection_honour_the_claim():
    assert "tof_matrix.is_claimed_tof(args.port)" in _read("scripts", "flash_mcu.py")
    assert 'family = "tof"' in _read("web", "backend", "hardware_service.py")


# -- firmware ----------------------------------------------------------------------------
def test_the_driver_never_blocks_or_allocates():
    src = _read("firmware", "common", "lib", "tof", "tof.cpp")
    code = re.sub(r"//.*", "", src)
    for bad in ("delay(", "malloc(", "new ", "while ("):
        assert bad not in code, f"tof.cpp: {bad!r} -- tofPoll() runs inside the micro-ROS loop"


def test_the_probe_names_it_without_register_reads():
    src = _read("firmware", "common", "lib", "i2c_probe", "i2c_probe.cpp")
    block = src[src.index("addr >= 0x31 && addr <= 0x33"):]
    block = block[:block.index("return;")]
    assert "SEN0628" in block and "readRegister" not in block


def test_main_publishes_tof_raw_best_effort_and_masks_it_in_sim():
    src = _read("firmware", "src", "main.cpp")
    assert 'topicName("tof/raw")' in src
    i = src.index('topicName("tof/raw")')
    assert "rclc_publisher_init_best_effort" in src[i - 300:i]
    assert "tof_present = !sonar_simd && initTof(" in src


# -- the web backend's save ----------------------------------------------------------------
class _Req:
    def __init__(self, body):
        self._body = body

    async def json(self):
        return self._body


def _backend(monkeypatch, params):
    import asyncio  # noqa: F401
    pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
    os.environ["COCKPIT_AUTH"] = "off"
    import sys
    sys.path.insert(0, os.path.join(REPO, "web", "backend"))
    import routes_config
    saved = {}

    class _Res:
        returncode, stdout, stderr = 0, "", ""

    monkeypatch.setattr(routes_config, "load_params", lambda *a, **k: params)
    monkeypatch.setattr(routes_config, "save_params", lambda p, *a, **k: saved.update(p=copy.deepcopy(p)))
    monkeypatch.setattr(routes_config, "regenerate_firmware_headers", lambda *a, **k: _Res())
    monkeypatch.setattr(routes_config, "regenerate_robot_description", lambda *a, **k: {})
    return routes_config, saved


def test_the_page_cannot_save_a_tof_beside_a_lidar(monkeypatch):
    import asyncio
    p = _ref()                                   # has an LD19
    rc, saved = _backend(monkeypatch, p)         # skips without fastapi, so import it after
    from fastapi import HTTPException
    with pytest.raises(HTTPException) as e:
        asyncio.run(rc.api_save_hardware_config(_Req({
            "controller": p["base_controller"]["name"],
            "tof": {"model": "sen0628", "connection": "i2c", "i2c_address": 0x33}})))
    assert e.value.status_code == 400 and "LiDAR" in e.value.detail
    assert "p" not in saved


def test_the_page_saves_a_tof_on_a_robot_without_a_lidar(monkeypatch):
    import asyncio
    p = _no_lidar(_ref())
    rc, saved = _backend(monkeypatch, p)
    asyncio.run(rc.api_save_hardware_config(_Req({
        "controller": p["base_controller"]["name"],
        "tof": {"model": "sen0628", "connection": "i2c", "i2c_address": 0x32}})))
    assert saved["p"]["base_controller"]["tof"]["i2c_address"] == 0x32
    assert tm.tof_settings(saved["p"]["base_controller"])["connection"] == "i2c"
