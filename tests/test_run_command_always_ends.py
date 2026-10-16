"""Every console run reports how it ended, even when its stream does not say.

runCommand called onDone only on a parsed `done` event. A stream that closed
without one, or whose last event had no blank line after it, left the caller
waiting for ever: the LiDAR bring-up step hung on "Preparing micro-ROS agent"
after AGENT_FOUND, and every bring-up button stayed dead until a reload
(2026-10-16, a UNO Q robot over Wi-Fi).
"""
import json
import pathlib
import shutil
import subprocess

import pytest

FRONTEND = pathlib.Path(__file__).resolve().parents[1] / "web" / "frontend"


def _run_command_src():
    js = (FRONTEND / "app-core.js").read_text()
    start = js.index("function runCommand(")
    end = js.index("\n}\n", start) + len("\n}\n")
    return js[start:end]


HARNESS = r"""
const lines = [];
function openTerminal() {}
function logLine(l) { lines.push(l); }
const enc = new TextEncoder();
function streamOf(chunks, failAt) {
  let i = 0;
  return { getReader: () => ({ read: async () => {
    if (failAt !== undefined && i === failAt) throw new Error("network error");
    if (i >= chunks.length) return { done: true };
    return { done: false, value: enc.encode(chunks[i++]) };
  } }) };
}
async function trial(chunks, failAt, reject) {
  lines.length = 0;
  globalThis.fetch = async () => {
    if (reject) throw new Error("connection refused");
    return { ok: true, status: 200, body: streamOf(chunks, failAt) };
  };
  const codes = [];
  try { await runCommand("true", { onDone: (c) => codes.push(c) }); } catch (e) { codes.push("threw"); }
  return { codes, lines: [...lines] };
}
(async () => {
  const out = {};
  out.normal = await trial(['event: output\ndata: {"line": "AGENT_FOUND"}\n\n', 'event: done\ndata: {"exit_code": 0}\n\n']);
  out.noTrailingBlank = await trial(['event: output\ndata: {"line": "x"}\n\n', 'event: done\ndata: {"exit_code": 3}']);
  out.noDone = await trial(['event: output\ndata: {"line": "AGENT_FOUND"}\n\n']);
  out.broken = await trial(['event: output\ndata: {"line": "x"}\n\n'], 1);
  out.refused = await trial([], undefined, true);
  console.log(JSON.stringify(out));
})();
"""


@pytest.fixture(scope="module")
def runs():
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    out = subprocess.run([node, "-e", _run_command_src() + HARNESS],
                         capture_output=True, text=True, check=True)
    return json.loads(out.stdout)


def test_a_normal_run_ends_once_with_its_code(runs):
    assert runs["normal"]["codes"] == [0]


def test_a_last_event_without_a_blank_line_still_counts(runs):
    assert runs["noTrailingBlank"]["codes"] == [3]


def test_a_stream_that_closes_without_done_ends_the_run(runs):
    assert runs["noDone"]["codes"] == [-1]
    assert "[console] the stream closed without an exit code" in runs["noDone"]["lines"]


def test_a_broken_stream_ends_the_run_and_still_rejects(runs):
    assert runs["broken"]["codes"] == [-1, "threw"]


def test_a_refused_request_ends_the_run(runs):
    assert runs["refused"]["codes"] == [-1, "threw"]
