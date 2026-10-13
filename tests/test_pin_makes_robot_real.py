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
