"""A real robot's bring-up (scripts/bringup_checklist.py, robot_bringup.py, the web UI's
Bring-up card through web/backend/routes_bringup.py): the arithmetic that fixes a wheel, a
tachometer reading or a battery divider, the record both front ends share, and the gate the
1-Click asks before it moves a real robot (user, 2026-10-14: "one click can be used on real
robot only after sensors and motors are working correctly")."""
import os
import shutil
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))

import bringup_checklist as bc  # noqa: E402
import lidar_check  # noqa: E402


# ---------------------------------------------------------------- motors
@pytest.mark.parametrize("forward,rpm,motor,encoder", [
    (True, 120.0, False, False),     # right
    (True, -120.0, False, True),     # the wheel is right, the encoder counts down
    (False, -120.0, True, False),    # the motor is backwards; the encoder follows it
    (False, 120.0, True, True),      # both: after the motor flip the encoder would count down
])
def test_invert_fix_names_the_flags_a_wheel_needs(forward, rpm, motor, encoder):
    assert bc.invert_fix(forward, rpm) == {"flip_motor": motor, "flip_encoder": encoder}


def test_invert_fix_refuses_a_silent_encoder():
    with pytest.raises(ValueError, match="encoder counted nothing"):
        bc.invert_fix(True, 0.3)


def test_invert_changes_flip_what_the_config_has():
    params = {"base_controller": {"pins": {"motor2": {"invert": True}, "encoder2": {}}}}
    ch = bc.changes_for(params, "invert", wheel=2, flip_motor=True, flip_encoder=True)
    assert ch == {"base_controller.pins.motor2.invert": False, "base_controller.pins.encoder2.invert": True}


# ---------------------------------------------------------------- rpm
def test_cpr_from_a_tachometer():
    # the encoder says 200 RPM with cpr 1000; the wheel really turns 180: there are more counts per rev
    assert bc.cpr_from_tacho(1000, 200.0, 180.0) == 1111


def test_cpr_from_a_tachometer_refuses_a_missing_gear_ratio():
    with pytest.raises(ValueError, match="gear ratio or the quadrature"):
        bc.cpr_from_tacho(44, 9000.0, 300.0)          # 30:1 gearbox left out


def test_cpr_from_hand_turns():
    assert bc.cpr_from_turns(-13200, 10) == 1320
    with pytest.raises(ValueError, match="not counting"):
        bc.cpr_from_turns(3, 10)


def test_max_rpm_is_bounded():
    assert bc.changes_for({}, "max_rpm", rpm=176.6) == {"kinematics.max_rpm": 177}
    with pytest.raises(ValueError):
        bc.changes_for({}, "max_rpm", rpm=5)


# ---------------------------------------------------------------- battery
def test_battery_trim_moves_r1_by_the_meters_ratio():
    t = bc.battery_trim(30000, 7500, 11.00, 11.55)          # reads 5 % low
    assert t["scale"] == 1.05
    assert t["ratio"] == pytest.approx(5.0 * 1.05, abs=1e-4)
    assert t["r1"] == pytest.approx((5.25 - 1) * 7500, abs=0.1)


def test_battery_trim_refuses_a_wrong_divider():
    with pytest.raises(ValueError, match="check the pin"):
        bc.battery_trim(30000, 7500, 6.0, 12.0)
    with pytest.raises(ValueError, match="nothing on the pin"):
        bc.battery_trim(30000, 7500, 0.1, 12.0)


# ---------------------------------------------------------------- sensors
def _still(n=10):
    return [{"cmd": "sensors", "acc": [0.1, -0.05, 9.79], "gyr": [0.002, -0.001, 0.0],
             "mag_ut": [21.0, -4.0, -40.0], "bat": 11.6} for _ in range(n)]


def _moving():
    out = []
    for i in range(40):
        s = 1 if i % 2 else -1
        out.append({"cmd": "sensors", "acc": [4.0 * s, 2.0, 8.0], "gyr": [0.1, 1.5 * s, 0.8],
                    "mag_ut": [21.0 + 25 * s, -4.0 - 10 * s, -40.0]})
    return out


GENDRV = {"wheels_real": True, "wheels": 2, "imu": True, "mag": True, "battery": True,
          "battery_adc": False, "battery_chip": "INA219", "lidar": True, "wifi": True, "mecanum": False}


def test_sensors_pass_when_every_chip_follows_the_robot():
    ok, lines, _ = bc.sensors_judge(_still(), _moving(), GENDRV)
    assert ok, lines
    assert any("magnetometer" in ln and "follows" in ln for ln in lines)


def test_a_stuck_magnetometer_fails_even_when_the_imu_moves():
    moving = _moving()
    for r in moving:
        r["mag_ut"] = [21.0, -4.0, -40.0]
    ok, lines, _ = bc.sensors_judge(_still(), moving, GENDRV)
    assert not ok
    assert any("did not change" in ln for ln in lines)


def test_a_robot_held_still_does_not_pass():
    ok, lines, _ = bc.sensors_judge(_still(30), [], GENDRV)
    assert not ok and any("did not follow" in ln for ln in lines)


def test_a_wrong_accelerometer_scale_fails():
    rows = [dict(r, acc=[0.0, 0.0, 1.0]) for r in _still()]        # g, not m/s^2
    ok, lines, _ = bc.sensors_judge(rows, _moving(), dict(GENDRV, mag=False))
    assert not ok and any("not ~9.8" in ln for ln in lines)


# ---------------------------------------------------------------- the record and the gate
@pytest.fixture
def cfgdir(tmp_path, monkeypatch):
    monkeypatch.setenv("COCKPIT_CONFIG_DIR", str(tmp_path))
    path = tmp_path / "ts_config.yaml"
    shutil.copy(os.path.join(REPO_ROOT, "config", "reference", "gendrv_config.yaml"), path)
    params = yaml.safe_load(path.read_text())
    return str(path), params


def test_the_gate_holds_a_real_robot_until_the_bench_steps_pass(cfgdir):
    path, params = cfgdir
    ok, missing = bc.gate("ts", params, path)
    assert not ok
    assert [m.split(":")[0] for m in missing] == ["sensors", "battery", "motors", "rpm", "lidar"]
    for st in ("sensors", "battery", "motors", "rpm"):
        bc.mark("ts", params, st, "pass", "ok")
    bc.mark("ts", params, "lidar", "skip", "a person decided")
    assert bc.gate("ts", params, path) == (True, [])


def test_the_floor_steps_do_not_hold_the_1click(cfgdir):
    path, params = cfgdir
    for st in bc.GATE:
        bc.mark("ts", params, st, "pass")
    rows = {r["stage"]: r for r in bc.checklist("ts", params, path)["stages"]}
    assert rows["speed"]["state"] == "todo" and not rows["speed"]["gate"]
    assert bc.gate("ts", params, path)[0]


def test_a_pass_goes_stale_when_its_values_change(cfgdir):
    path, params = cfgdir
    for st in bc.GATE:
        bc.mark("ts", params, st, "pass")
    done = bc.apply(path, {"kinematics.counts_per_rev": 999})
    assert done["kinematics.counts_per_rev"]["to"] == 999
    params2 = yaml.safe_load(open(path))
    rows = {r["stage"]: r["state"] for r in bc.checklist("ts", params2, path)["stages"]}
    assert rows["rpm"] == "stale" and rows["motors"] == "pass"
    ok, missing = bc.gate("ts", params2, path)
    assert not ok and missing[0].startswith("rpm: the config changed")


def test_apply_keeps_the_file_it_replaced(cfgdir, tmp_path):
    path, _ = cfgdir
    before = open(path).read()
    bc.apply(path, {"base_controller.pins.motor1.invert": True})
    baks = list((tmp_path / "state" / "bringup").glob("ts_config.yaml.*.bak"))
    assert len(baks) == 1 and baks[0].read_text() == before
    assert yaml.safe_load(open(path))["base_controller"]["pins"]["motor1"]["invert"] is True
    assert bc.apply(path, {"base_controller.pins.motor1.invert": True}) == {}     # no change, no backup


def test_simulated_wheels_are_not_gated(tmp_path, monkeypatch):
    monkeypatch.setenv("COCKPIT_CONFIG_DIR", str(tmp_path))
    params = yaml.safe_load(open(os.path.join(REPO_ROOT, "config", "reference", "gendrv_config.yaml")))
    params["base_controller"].setdefault("sensors", {})["use_sim_wheel"] = True
    c = bc.checklist("bench", params)
    assert not c["real_robot"] and c["gate_ok"]


def test_mark_refuses_an_unknown_step(cfgdir):
    _, params = cfgdir
    with pytest.raises(ValueError):
        bc.mark("ts", params, "warp", "pass")
    with pytest.raises(ValueError):
        bc.mark("ts", params, "rpm", "maybe")


# ---------------------------------------------------------------- the LiDAR
def _scan(stamp, n=450, r=2.0, blocked=()):
    import math
    rr = [float("inf") if any(a <= i * 360.0 / n < b for a, b in blocked) else r for i in range(n)]
    return {"stamp": stamp, "angle_min": 0.0, "angle_increment": 2 * math.pi / n,
            "range_min": 0.02, "range_max": 12.0, "ranges": rr}


def test_lidar_check_passes_a_healthy_ld19():
    res = lidar_check.analyse([_scan(t / 10.0) for t in range(100)], 10.0, 10.0)
    assert res["pass"], res["reasons"]
    assert res["hz"] == pytest.approx(10.0, abs=0.1) and res["coverage"] == 1.0


def test_lidar_check_fails_a_slow_or_half_blind_lidar():
    slow = lidar_check.analyse([_scan(t / 4.0) for t in range(40)], 10.0, 10.0)
    assert not slow["pass"] and "rate" in slow["reasons"][0]
    half = lidar_check.analyse([_scan(t / 10.0, blocked=((90, 270),)) for t in range(100)], 10.0, 10.0)
    assert not half["pass"] and any("of the circle" in r for r in half["reasons"])


def test_lidar_rate_comes_from_the_model():
    assert lidar_check.expected_hz({"base_controller": {"lidar": {"model": "LD19"}}}) == 10.0
    assert lidar_check.expected_hz({}) == 0.0


# ---------------------------------------------------------------- the 1-Click and the firmware
def test_the_pipeline_refuses_an_unready_real_robot():
    src = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert '"--skip-bringup-check"' in src
    assert "bringup_checklist.gate(robot_name, params, params_path)" in src
    assert "[BRINGUP REQUIRED]" in src


def test_the_web_ui_passes_the_override_only_when_asked():
    js = open(os.path.join(REPO_ROOT, "web", "frontend", "app-workflow.js")).read()
    assert "buOneClickGate" in js and "skip_bringup_check=true" in js
    routes = open(os.path.join(REPO_ROOT, "web", "backend", "routes_hardware.py")).read()
    assert 'cmd.append("--skip-bringup-check")' in routes


def test_the_firmware_streams_sensors_at_up_to_20_hz():
    tools = open(os.path.join(REPO_ROOT, "firmware", "src", "tools", "tools.cpp")).read()
    assert 'usage("sensors [hz 1..20]")' in tools and "test_sensors_rate(" in tools
    import tool_session
    assert tool_session.validate("sensors 10") == "sensors 10"
    for bad in ("sensors 0", "sensors 21", "sensors x", "sensors 1 2"):
        with pytest.raises(tool_session.Refused):
            tool_session.validate(bad)


# ---------------------------------------------------------------- the routes
@pytest.fixture
def client(cfgdir, monkeypatch):
    pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
    pytest.importorskip("httpx", reason="FastAPI's TestClient needs httpx")
    os.environ["COCKPIT_AUTH"] = "off"
    import core
    import routes_bringup
    from fastapi.testclient import TestClient
    path, _ = cfgdir
    monkeypatch.setattr(routes_bringup, "get_active_params_path", lambda: path)
    return TestClient(core.app), path


def test_route_checklist_and_mark(client):
    c, path = client
    j = c.get("/api/bringup/checklist").json()
    assert j["real_robot"] and not j["gate_ok"] and j["parts"]["battery_chip"] == "INA219"
    assert c.post("/api/bringup/mark", json={"stage": "motors", "status": "pass"}).status_code == 200
    rows = {r["stage"]: r for r in c.get("/api/bringup/checklist").json()["stages"]}
    assert rows["motors"]["state"] == "pass" and rows["motors"]["by"] == "gui"
    assert c.post("/api/bringup/mark", json={"stage": "nope", "status": "pass"}).status_code == 400


def test_route_calc_does_the_arithmetic(client):
    c, _ = client
    j = c.post("/api/bringup/calc", json={"kind": "invert", "wheel": 1, "turned_forward": False, "rpm": -50}).json()
    assert j["fix"] == {"flip_motor": True, "flip_encoder": False}
    assert list(j["changes"]) == ["base_controller.pins.motor1.invert"]
    r = c.post("/api/bringup/calc", json={"kind": "battery", "reported": 11, "measured": 11.5})
    assert r.status_code == 400 and "no ADC divider" in r.json()["detail"]      # an INA219 robot


def test_route_apply_writes_only_calibration_keys(client):
    c, path = client
    r = c.post("/api/bringup/apply", json={"changes": {"base_controller.transport": "serial"}})
    assert r.status_code == 400 and "not a bring-up calibration" in r.json()["detail"]
    r = c.post("/api/bringup/apply", json={"changes": {"base_controller.pins.motor1.invert": "yes"}})
    assert r.status_code == 400
    r = c.post("/api/bringup/apply", json={"changes": {"kinematics.counts_per_rev": 1234}})
    assert r.status_code == 200 and r.json()["handle"]
    assert yaml.safe_load(open(path))["kinematics"]["counts_per_rev"] == 1234


def test_route_run_is_robot_bringup_py(client):
    c, path = client
    import actions
    h = c.post("/api/bringup/run", json={"stage": "acc", "write": True}).json()["handle"]
    cmd = actions.claim(h)
    assert "robot_bringup.py" in cmd and cmd.endswith("acc --write") and "--by gui" in cmd
    assert c.post("/api/bringup/run", json={"stage": "rm"}).status_code == 400


def test_the_command_line_works_on_the_robot_the_web_ui_selected(tmp_path, monkeypatch):
    import cockpit_paths
    monkeypatch.setenv("COCKPIT_CONFIG_DIR", str(tmp_path))
    for n in ("bare_sim", "walkbot"):
        (tmp_path / f"{n}_config.yaml").write_text("robot: {name: %s}\n" % n)
    assert cockpit_paths.active_robot_path().endswith("bare_sim_config.yaml")     # nothing selected
    (tmp_path / ".active_robot").write_text("walkbot_config.yaml\n")
    assert cockpit_paths.active_robot_path().endswith("walkbot_config.yaml")
    (tmp_path / ".active_robot").write_text("../secrets.yaml\n")
    assert cockpit_paths.active_robot_path().endswith("bare_sim_config.yaml")
