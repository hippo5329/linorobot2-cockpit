"""The tool's HTTP side (web/backend/routes_tool.py): the OTA password never leaves the
server, the grammar guards /api/tool/cmd, Stop reaches the board, and leaving writes
`app=base` env-only -- never a reflash. The robot and its link are faked."""
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))

pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
pytest.importorskip("httpx", reason="FastAPI's TestClient needs httpx")
os.environ["COCKPIT_AUTH"] = "off"
import core  # noqa: E402
import routes_tool  # noqa: E402
import tool_session  # noqa: E402
import actions  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

PASSWORD = "s3cret-ota-pw-7"


class FakeLink:
    def __init__(self):
        self.sent, self.closed = [], False

    def write(self, data):
        self.sent.append(data)

    def read(self):
        return b""

    def close(self):
        self.closed = True


@pytest.fixture
def robot(monkeypatch):
    r = {"params_path": "/tmp/robot1_config.yaml", "params": {}, "robot": "robot1",
         "tgt": {"serial_port": "/dev/ttyUSB9"}, "env": {"ota_password": PASSWORD, "baud": 921600},
         "pio_env": "esp32", "wifi": True, "sim": False}
    monkeypatch.setattr(routes_tool, "_robot", lambda: r)
    grams, links = [], []

    def fake_open_wifi(rr):
        link = FakeLink()
        links.append(link)
        s = tool_session.ToolSession(rr["robot"], link, "wifi", ip="192.0.2.9",
                                     send_datagram=lambda ip, port, d: grams.append(d))
        s.feed(b"[fw] linorobot2_hardware app=test_motors distro=jazzy\n[app] test_motors\n")
        return s
    monkeypatch.setattr(routes_tool, "_open_wifi", fake_open_wifi)
    yield r, grams, links
    tool_session.REGISTRY.remove("robot1")


def test_a_session_end_to_end_without_the_password(robot):
    r, grams, links = robot
    c = TestClient(core.app)
    bodies = []
    res = c.post("/api/tool/open", json={})
    assert res.status_code == 200, res.text
    bodies.append(res.text)
    res = c.get("/api/tool/state"); bodies.append(res.text)
    assert res.json()["open"] is True
    res = c.post("/api/tool/cmd", json={"line": "motor 1 30 5"}); bodies.append(res.text)
    assert res.status_code == 200 and links[0].sent[-1] == b"motor 1 30 5\r\n"
    res = c.post("/api/tool/cmd", json={"line": "set ota_password x"}); bodies.append(res.text)
    assert res.status_code == 400
    res = c.post("/api/tool/cmd", json={"line": "status\nmotor 1 100"}); bodies.append(res.text)
    assert res.status_code == 400
    res = c.post("/api/tool/stop"); bodies.append(res.text)
    assert res.status_code == 200 and grams == [b"lino-stop"]
    res = c.post("/api/tool/close"); bodies.append(res.text)
    j = res.json()
    assert j["needs_env_base"] is True and j["handle"]
    cmd = actions.claim(j["handle"])
    assert "--env-only" in cmd and "--app base" in cmd and "--firmware-dir" not in cmd and "--prebuilt" not in cmd
    for b in bodies + [cmd]:
        assert PASSWORD not in b, "the OTA password reached a response or a command line"


def test_the_sim_mcu_is_refused_with_its_reason(monkeypatch):
    monkeypatch.setattr(routes_tool, "_robot", lambda: {"robot": "bare_sim", "sim": True, "wifi": False,
                                                        "pio_env": "host", "tgt": {}, "env": {}})
    res = TestClient(core.app).post("/api/tool/open", json={})
    assert res.status_code == 409 and "Sim MCU" in res.json()["detail"]


def test_commands_need_an_open_session(robot):
    res = TestClient(core.app).post("/api/tool/cmd", json={"line": "status"})
    assert res.status_code == 404


def test_the_stream_is_a_protected_ticketed_path():
    import access
    assert "/api/tool/stream" in access.PROTECTED_GETS and access.is_stream_path("/api/tool/stream")
