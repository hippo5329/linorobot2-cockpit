"""The first pin assigned makes a bare module a real robot, and a real robot simulates nothing.

User, 2026-10-13: "in web ui, when user assign a pin, it moves from bare module to a real robot,
turn sim mode sim devices off automatically." Real mode already ignored the flags at run time;
the saved config kept them, so the Sensors tab still ticked them, Flash MCU wrote them into the
env, the tool refused the wheels as simulated, and with sim_wheel on every motor pin went to -1.
"""
import asyncio
import copy
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import gen_bare_config  # noqa: E402
import one_click_pipeline as ocp  # noqa: E402


def _bare(name="myrobot"):
    p = copy.deepcopy(gen_bare_config.bare_config("pico2"))
    p.setdefault("robot", {})["name"] = name
    return p


def _sims(p):
    s = p["base_controller"]["sensors"]
    return {k: v for k, v in s.items() if k.startswith("use_sim_")}


def test_a_bare_module_keeps_its_simulation():
    p = _bare()
    before = _sims(p)
    assert any(before.values()), "the bare module should simulate something"
    assert ocp.simulate_nothing_if_real(p) == []
    assert _sims(p) == before


def test_the_first_pin_turns_every_simulated_device_off():
    p = _bare()
    on = [k for k, v in _sims(p).items() if v]
    p["base_controller"]["pins"]["motor1"]["pwm"] = 2
    assert not ocp.is_bare_config(p), "the test must assign a pin the catalogue counts"
    assert ocp.simulate_nothing_if_real(p) == on
    assert not any(_sims(p).values())
    assert ocp.simulate_nothing_if_real(p) == [], "the second save has nothing left to do"


def test_a_robot_built_from_a_design_simulates_nothing():
    p = _bare()
    p["robot"]["reference"] = "gendrv"
    p["base_controller"]["sensors"]["use_sim_imu"] = True
    assert "use_sim_imu" in ocp.simulate_nothing_if_real(p)
    assert p["base_controller"]["sensors"]["use_sim_imu"] is False


def test_the_simulated_lidars_own_output_pin_is_not_wiring():
    """The GenDrv's LIDAR_RXD carries the SIMULATED LiDAR out: it does not make the robot real."""
    p = _bare()
    p["base_controller"]["sensors"]["use_sim_ld19"] = True
    p["base_controller"].setdefault("lidar", {})["rx_pin"] = 4
    if not ocp.is_bare_config(p):
        pytest.skip("this board's catalogue places the LiDAR pin elsewhere")
    assert ocp.simulate_nothing_if_real(p) == []


# --- the backend: the save turns them off and says so; simulation cannot be turned back on.
# The handlers are called directly with a stub request: the robot image has fastapi but not
# httpx, so a TestClient test would skip in the one place the backend runs.

class _Req:
    def __init__(self, body):
        self._body = body

    async def json(self):
        return self._body


def _backend(monkeypatch, params):
    pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
    os.environ["COCKPIT_AUTH"] = "off"
    sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
    import routes_config

    saved = {}

    class _Res:
        returncode, stdout, stderr = 0, "", ""

    monkeypatch.setattr(routes_config, "load_params", lambda *a, **k: params)
    monkeypatch.setattr(routes_config, "save_params", lambda p, *a, **k: saved.update(p=copy.deepcopy(p)))
    monkeypatch.setattr(routes_config, "regenerate_firmware_headers", lambda *a, **k: _Res())
    monkeypatch.setattr(routes_config, "regenerate_robot_description", lambda *a, **k: {})
    return routes_config, saved


def test_saving_a_pin_turns_simulation_off_in_the_saved_config(monkeypatch):
    p = _bare()
    rc, saved = _backend(monkeypatch, p)
    pins = copy.deepcopy(p["base_controller"]["pins"])
    pins["motor1"]["pwm"] = 2
    body = asyncio.run(rc.api_save_hardware_config(_Req({
        "controller": p["base_controller"]["name"], "pins": pins,
        "sensors": dict(p["base_controller"]["sensors"])})))
    assert body["sim_turned_off"], body
    assert "real robot" in body["message"]
    assert not any(v for k, v in saved["p"]["base_controller"]["sensors"].items()
                   if k.startswith("use_sim_"))


def test_a_real_robot_cannot_be_put_back_into_simulation(monkeypatch):
    p = _bare()
    p["base_controller"]["pins"]["motor1"]["pwm"] = 2
    rc, saved = _backend(monkeypatch, p)
    from fastapi import HTTPException
    with pytest.raises(HTTPException) as e:
        asyncio.run(rc.api_toggle_sim_mode(_Req({"enabled": True})))
    assert e.value.status_code == 400 and "simulates nothing" in e.value.detail
    assert "p" not in saved


def test_the_page_unticks_what_the_save_turned_off():
    js = open(os.path.join(REPO_ROOT, "web", "frontend", "app-hardware.js")).read()
    assert "result.sim_turned_off" in js and "box.checked = false" in js


# --- and back: a real robot reverted to a bare module (user, 2026-10-13: "offer an option to
# revert a real robot to bare module, ie, every pin unassigned")

import yaml  # noqa: E402
import robot_templates  # noqa: E402


def _real_from_design(design="gendrv"):
    with open(os.path.join(REPO_ROOT, "config", "reference", f"{design}_config.yaml")) as fh:
        d = yaml.safe_load(fh)
    robot = _bare("myrobot")
    robot["kinematics"]["wheel_diameter"] = 0.0812       # the user's own chassis
    robot["nav2"] = {"marker": "my tuning"}
    out, _kept, _kit = robot_templates.apply_design(robot, d, design)
    out["base_controller"]["serial_port"] = "/dev/ttyUSB7"
    out["base_controller"]["domain_id"] = 42
    return out


def test_revert_unassigns_every_pin_and_simulates_every_device():
    real = _real_from_design()
    assert not ocp.is_bare_config(real) and ocp.is_real_robot(real)
    bare = robot_templates.revert_to_bare(real)
    assert ocp.is_bare_config(bare), "only the LED may stay assigned"
    assert not ocp.is_real_robot(bare)
    assert ocp.robot_kind("myrobot", (bare.get("robot") or {}).get("reference"),
                          ocp.is_bare_config(bare)) == "sim"
    s = bare["base_controller"]["sensors"]
    assert s.get("use_sim_wheel") and s.get("use_sim_imu") and s.get("use_sim_ld19")
    assert "reference" not in bare["robot"] and bare["robot"]["name"] == "myrobot"


def test_revert_keeps_the_robots_own_chassis_tuning_and_port():
    real = _real_from_design()
    bare = robot_templates.revert_to_bare(real)
    assert bare["kinematics"] == real["kinematics"]
    assert bare["geometry"] == real["geometry"]
    assert bare["nav2"] == real["nav2"]
    assert bare["base_controller"]["serial_port"] == "/dev/ttyUSB7"
    assert bare["base_controller"]["domain_id"] == 42
    assert bare["base_controller"]["mcu"] == real["base_controller"]["mcu"]


def test_revert_on_every_design():
    ref = os.path.join(REPO_ROOT, "config", "reference")
    for fn in sorted(os.listdir(ref)):
        if fn.endswith("_config.yaml"):
            bare = robot_templates.revert_to_bare(_real_from_design(fn[:-len("_config.yaml")]))
            assert ocp.is_bare_config(bare), fn


def test_the_revert_endpoint(monkeypatch):
    real = _real_from_design()
    rc, saved = _backend(monkeypatch, real)
    monkeypatch.setattr(rc, "_named_robot_or_400", lambda *a: ("/tmp/myrobot_config.yaml", real, "myrobot"))
    monkeypatch.setattr(rc, "get_robots_list", lambda *a, **k: [])
    body = asyncio.run(rc.revert_bare(_Req({})))
    assert body["status"] == "ok" and "bare" in body["message"]
    assert ocp.is_bare_config(saved["p"])


def test_the_page_offers_the_revert_only_on_a_real_robot():
    root = os.path.join(REPO_ROOT, "web", "frontend")
    html = open(os.path.join(root, "index.html")).read()
    presets = open(os.path.join(root, "app-presets.js")).read()
    core = open(os.path.join(root, "app-core.js")).read()
    assert 'id="btn-revert-bare"' in html and 'id="btn-revert-bare-pins"' in html
    assert '"/api/robot/revert_bare"' in presets and "revertToBareModule" in presets
    assert 'for (const id of ["btn-revert-bare", "btn-revert-bare-pins"])' in core
    assert "if (b) b.hidden = !real;" in core


def test_a_rebuild_meant_for_another_robot_is_refused(monkeypatch):
    """apply_bare / apply_reference / revert_bare rebuild WHICHEVER robot is active. The page's
    two robot switches raced (2026-10-19): the board's default robot was being made a bare
    module while a Wi-Fi robot was picked, and the bare module landed on the Wi-Fi robot. The
    request names its robot; once another is active, nothing is applied."""
    HTTPException = pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn").HTTPException
    p = _bare("uileg_udp4")
    rc, saved = _backend(monkeypatch, p)
    monkeypatch.setattr(rc, "get_active_params_path", lambda: "/tmp/uileg_udp4_config.yaml")
    monkeypatch.setattr(rc, "get_robots_list", lambda *a, **k: [])
    for call, body in ((rc.apply_bare, {"mcu": "esp32", "robot": "lino_esp32"}),
                       (rc.apply_reference, {"design": "gendrv", "robot": "lino_esp32"}),
                       (rc.revert_bare, {"robot": "lino_esp32"})):
        with pytest.raises(HTTPException) as e:
            asyncio.run(call(_Req(body)))
        assert e.value.status_code == 409 and "uileg_udp4" in e.value.detail
    assert "p" not in saved, "nothing may be saved for a robot that is no longer active"
    # the robot it was meant for, or a request that names none (an older page), still applies
    assert asyncio.run(rc.apply_bare(_Req({"mcu": "esp32", "robot": "uileg_udp4"})))["status"] == "ok"
    assert asyncio.run(rc.apply_bare(_Req({"mcu": "esp32"})))["status"] == "ok"
