"""Entering the tool waits for its prompt, not a fixed 4 s (scripts/lino_tool.py catch_window).

After "[app] tool" the board prints its hardware matrix before its first prompt. The UNO Q's
STM32 scans its I2C bus slowly, and robot_bringup's first command, sent 4 s after the window
opened, landed in the middle of the scan and went unanswered.
"""
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import lino_tool  # noqa: E402


class ScriptedLink:
    """Replies to `tool` with the window line, then each chunk after its delay."""

    def __init__(self, chunks):
        self.chunks, self.t0, self.written = list(chunks), None, b""

    def write(self, data):
        self.written += data
        if self.t0 is None and b"tool" in data:
            self.t0 = time.time()

    def keepalive(self):
        pass

    def read(self):
        if self.t0 is None or not self.chunks:
            return b""
        delay, data = self.chunks[0]
        if time.time() - self.t0 >= delay:
            self.chunks.pop(0)
            return data
        return b""


def test_a_slow_scan_is_waited_out_to_the_prompt(monkeypatch):
    monkeypatch.setattr(lino_tool, "TOOL_READY_S", 10.0)
    link = ScriptedLink([(0.0, b"[app] tool\r\nScanning I2C bus...\r\n"),
                         (5.0, b"70: -- -- --\r\n[tool] ready\r\n"),
                         (5.2, b"tool> ")])
    t0 = time.time()
    assert lino_tool.catch_window(link, 3.0) is True
    assert time.time() - t0 >= 5.0, "returned before the prompt"


def test_no_prompt_is_not_an_entry(monkeypatch):
    monkeypatch.setattr(lino_tool, "TOOL_READY_S", 1.0)
    link = ScriptedLink([(0.0, b"[app] tool\r\nScanning I2C bus...\r\n")])
    assert lino_tool.catch_window(link, 3.0) is False


def test_a_prompt_in_the_same_read_returns_at_once(monkeypatch):
    monkeypatch.setattr(lino_tool, "TOOL_READY_S", 10.0)
    link = ScriptedLink([(0.0, b"[app] tool\r\n[tool] ready\r\ntool> ")])
    t0 = time.time()
    assert lino_tool.catch_window(link, 3.0) is True
    assert time.time() - t0 < 2.0
