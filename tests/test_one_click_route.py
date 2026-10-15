"""The 1-Click's HTTP side (web/backend/routes_hardware.py /api/workflow/one-click/stream):
one run at a time, reserved before the response streams; the tool and a run lock each other
out; the Run options reach the pipeline as its own flags, validated at the door. The pipeline
itself is faked: nothing is started."""
import inspect
import os
import sys
import time

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))

pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
pytest.importorskip("httpx", reason="FastAPI's TestClient needs httpx")
os.environ["COCKPIT_AUTH"] = "off"
import core  # noqa: E402
import routes_hardware as rh  # noqa: E402
import routes_tool  # noqa: E402
import tool_session  # noqa: E402
from fastapi import HTTPException  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

URL = "/api/workflow/one-click/stream?controller=pico2&robot=robot1&mode=real"


class FakeProc:
    started = []

    def __init__(self, cmd, **kw):
        FakeProc.started.append(cmd)
        self.cmd = cmd
        self.pid = 999999
        self.returncode = 0
        self.stdout = iter_lines(["[pipeline] fake run", "✅ 1-Click Pipeline Finished Successfully!"])

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        return self.returncode


class iter_lines:
    def __init__(self, lines):
        self._lines = [ln + "\n" for ln in lines]

    def readline(self):
        return self._lines.pop(0) if self._lines else ""


@pytest.fixture
def fake_run(monkeypatch, tmp_path):
    FakeProc.started = []
    monkeypatch.setattr(rh.subprocess, "Popen", FakeProc)
    maps = tmp_path / "maps"
    maps.mkdir()
    (maps / "kitchen.yaml").write_text("image: kitchen.pgm\n")
    monkeypatch.setattr(rh, "maps_dir", lambda: str(maps))
    rh._ONE_CLICK.update(proc=None, reserved_at=None)
    yield maps
    rh._ONE_CLICK.update(proc=None, reserved_at=None)


def call_route(**kw):
    """The route function itself, with FastAPI's Query defaults resolved: it returns the
    StreamingResponse without running the generator, as the server does before the first byte."""
    args = {}
    for name, p in inspect.signature(rh.stream_one_click_workflow).parameters.items():
        d = p.default
        args[name] = getattr(d, "default", d) if type(d).__name__ in ("Query", "FieldInfo") or hasattr(d, "json_schema_extra") else d
    args.update(kw)
    return rh.stream_one_click_workflow(**args)


def test_a_second_start_is_refused_before_the_first_streams(fake_run):
    first = call_route(controller="pico2", robot="robot1")
    assert first is not None and rh._ONE_CLICK["reserved_at"] is not None
    with pytest.raises(HTTPException) as e:
        call_route(controller="pico2", robot="robot1")
    assert e.value.status_code == 409 and "already in progress" in e.value.detail
    assert FakeProc.started == [], "nothing started yet: the slot is reserved, not run"


def test_a_stale_reservation_frees_the_slot(fake_run):
    rh._ONE_CLICK["reserved_at"] = time.monotonic() - rh._RESERVE_S - 1
    assert not rh.one_click_busy()


def test_a_finished_run_frees_the_slot_and_echoes_every_option(fake_run):
    c = TestClient(core.app)
    q = ("&explore=true&explore_timeout=600&explore_min_area=12.5&explore_beyond_room=true"
         "&goal_round_trips=0&require_goal=true&goal_tolerance=0.2&home_yaw_tolerance_deg=90"
         "&clearance=0.5&inch=false&inch_max=0.8&test_acc=true&test_acc_clearance=1.5"
         "&map_output=garage_1&no_drive_test=true&explore_sec=20")
    r = c.get(URL + q)
    assert r.status_code == 200, r.text
    cmd = FakeProc.started[0]
    joined = " ".join(cmd)
    for flag in ("--explore", "--explore-timeout 600", "--explore-min-area 12.5", "--explore-beyond-room",
                 "--goal-round-trips 0", "--require-goal", "--goal-tolerance 0.2",
                 "--home-yaw-tolerance 1.5708", "--clearance 0.5", "--no-inch", "--inch-max 0.8",
                 "--test-acc", "--test-acc-clearance 1.5", "--no-drive-test", "--explore-sec 20"):
        assert flag in joined, flag
    assert cmd[cmd.index("--map-output") + 1] == os.path.join(str(fake_run), "garage_1")
    assert ">>> Options: " in r.text and "--home-yaw-tolerance 1.5708" in r.text
    assert not rh.one_click_busy(), "the run ended: the slot is free again"


def test_empty_options_send_nothing(fake_run):
    r = TestClient(core.app).get(URL)
    assert r.status_code == 200, r.text
    joined = " ".join(FakeProc.started[0])
    for flag in ("--explore-timeout", "--goal-round-trips", "--clearance", "--no-inch", "--map ",
                 "--map-output", "--topics-only", "--home-yaw-tolerance", "--goal-x"):
        assert flag not in joined, flag


def test_a_saved_map_turns_exploration_and_test_acc_off(fake_run):
    r = TestClient(core.app).get(URL + "&saved_map=kitchen&explore=true&test_acc=true")
    assert r.status_code == 200, r.text
    cmd = FakeProc.started[0]
    assert cmd[cmd.index("--map") + 1] == os.path.join(str(fake_run), "kitchen.yaml")
    assert "--no-explore" in cmd and "--explore" not in cmd and "--test-acc" not in cmd


@pytest.mark.parametrize("q, code", [
    ("&explore_sec=-1", 422), ("&explore_sec=500", 422),
    ("&saved_map=../etc/passwd", 422), ("&saved_map=nosuchmap", 400),
    ("&map_output=a/b", 422), ("&map_output=" + "x" * 65, 422),
    ("&explore_timeout=30", 422), ("&goal_tolerance=2", 422), ("&clearance=-0.1", 422),
    ("&inch_max=5", 422), ("&test_acc_clearance=0.3", 422), ("&goal_x=99", 422),
    ("&home_yaw_tolerance_deg=0", 422), ("&explore_min_area=-1", 422),
])
def test_values_outside_their_range_are_refused(fake_run, q, code):
    r = TestClient(core.app).get(URL + q)
    assert r.status_code == code, (q, r.text)
    assert FakeProc.started == []
    assert not rh.one_click_busy(), "a refused request reserves nothing"


# ---------------------------------------------------------------- the tool and the 1-Click
class QuietLink:
    def __init__(self):
        self.sent, self.closed = [], False

    def write(self, data):
        self.sent.append(data)

    def read(self):
        return b""

    def close(self):
        self.closed = True


def test_the_one_click_refuses_while_the_tool_is_open(fake_run):
    tool_session.REGISTRY.add(tool_session.ToolSession("robot1", QuietLink(), "serial"), start_threads=False)
    try:
        r = TestClient(core.app).get(URL)
        assert r.status_code == 409 and "Back to robot" in r.json()["detail"]
        assert FakeProc.started == []
    finally:
        tool_session.REGISTRY.remove("robot1")


def test_the_tool_refuses_while_a_one_click_runs(fake_run, monkeypatch):
    monkeypatch.setattr(routes_tool, "_robot", lambda: {
        "params_path": "/tmp/robot1_config.yaml", "params": {}, "robot": "robot1",
        "tgt": {}, "env": {}, "pio_env": "esp32", "wifi": False, "sim": False})
    rh._ONE_CLICK["reserved_at"] = time.monotonic()
    r = TestClient(core.app).post("/api/tool/open", json={})
    assert r.status_code == 409 and "1-Click run is in progress" in r.json()["detail"]
    assert tool_session.REGISTRY.get("robot1") is None


def test_stop_reaches_the_session_the_page_holds_after_a_robot_switch(monkeypatch):
    link = QuietLink()
    tool_session.REGISTRY.add(tool_session.ToolSession("robot1", link, "serial"), start_threads=False)
    # The header now shows another robot: the active one is not the tool's.
    monkeypatch.setattr(routes_tool, "_robot", lambda: {"robot": "robot2"})
    try:
        c = TestClient(core.app)
        assert c.post("/api/tool/stop", json={}).status_code == 404
        r = c.post("/api/tool/stop", json={"robot": "robot1"})
        assert r.status_code == 200 and r.json()["robot"] == "robot1"
        assert link.sent == [b"stop\r\n"]
        assert c.post("/api/tool/stop", json={"robot": "../x"}).status_code == 400
    finally:
        tool_session.REGISTRY.remove("robot1")
