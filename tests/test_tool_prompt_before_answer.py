"""A print-only command's prompt comes before its answer (scripts/lino_tool.py run_commands).

tools.cpp answerLine gives `get`, `status` and `help` their "done" answer after the command
has printed its prompt: "tool> ok: get m1_inv (done)". run_commands then waited 30 s for a
second prompt that never came, so every command after the first took 30 s.
"""
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import lino_tool  # noqa: E402


class BoardLink:
    """Answers each line the way the firmware does: a print-only command prompts, then
    answers; a `set` of an invert answers, then prompts."""

    def __init__(self):
        self.out, self.written = b"", b""

    def write(self, data):
        self.written += data
        line = data.decode().strip()
        if line.startswith("set "):
            self.out += f"ok: {line} (saved, live now, envcrc 00000000 -> 11111111)\r\ntool> ".encode()
        else:
            self.out += f"  m1_inv           0\r\ntool> ok: {line} (done)\r\n".encode()

    def keepalive(self):
        pass

    def read(self):
        d, self.out = self.out, b""
        return d


def test_print_only_commands_do_not_wait_for_a_second_prompt():
    link = BoardLink()
    answers = []
    t0 = time.time()
    lino_tool.run_commands(link, ["get m1_inv", "g m1_inv", "set m1_inv 1", "status", "help"], answers)
    assert time.time() - t0 < 5.0, "a command waited for a prompt it had already read"
    assert [a["ok"] for a in answers] == [True] * 5
    assert link.written.count(b"\r\n") == 5
