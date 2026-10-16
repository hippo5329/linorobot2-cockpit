"""The web UI's tool is open only once the board's tool reads lines.

`[app] tool` is printed before the tool sets up its hardware; only `[tool] ready` and the
prompt say its shell is listening. The backend returned on `[app] tool`, so the bring-up card's
sensors step -- which opens the tool and sends at once -- got "no answer: sensors 10" on the
lyrical Sim MCU and never enabled its Turn button (2026-10-16). The command-line tool already
waited for the prompt (lino_tool.catch_window)."""
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
import routes_tool  # noqa: E402
import tool_session  # noqa: E402


class ScriptedLink:
    """read() hands out the script one item per call, then nothing."""

    def __init__(self, script):
        self.script = list(script)
        self.reads = 0
        self.sent = []          # (reads so far, bytes)

    def write(self, data):
        self.sent.append((self.reads, data))

    def read(self):
        self.reads += 1
        return self.script.pop(0) if self.script else b""

    def close(self):
        pass


BOOT = [b"", b"linorobot2_hardware app=tool\n", b"[app] tool\n", b"", b"", b"", b"",
        b"[tool] ready -- 'help' lists the commands\n", b"tool> "]


def session_on(link):
    return tool_session.ToolSession("robot1", link, "sim")


def test_not_open_on_app_tool_alone():
    link = ScriptedLink(BOOT)
    s = session_on(link)
    assert routes_tool._until_ready(s, 5) is True
    # It read on past "[app] tool" to the ready line: the shell is listening.
    assert any("[tool] ready" in ln for ln in s._lines)
    assert link.reads >= BOOT.index(b"[tool] ready -- 'help' lists the commands\n") + 1


def test_the_knock_stops_once_the_board_is_in_the_tool():
    link = ScriptedLink(BOOT)
    s = session_on(link)
    assert routes_tool._until_ready(s, 5, knock=True) is True
    entered_read = BOOT.index(b"[app] tool\n") + 1
    late = [r for r, d in link.sent if r > entered_read and d == b"tool\r\n"]
    assert late == [], "every `tool` typed after entry reaches the shell as a command"


def test_a_firmware_without_the_ready_line_is_let_in_after_the_grace(monkeypatch):
    monkeypatch.setattr(tool_session, "READY_GRACE_S", 0.05)
    s = session_on(ScriptedLink([b"[app] tool\n"]))
    assert routes_tool._until_ready(s, 5) is True


def test_nothing_from_the_board_is_not_open():
    s = session_on(ScriptedLink([]))
    assert routes_tool._until_ready(s, 0.1) is False


def test_the_prompt_alone_counts():
    s = session_on(ScriptedLink([b"tool> "]))
    assert routes_tool._until_ready(s, 1) is True
