# Copyright (c) 2026 Linorobot contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""The board's tool answers every command in words (firmware tools.cpp ack()):

    ok: motor 1 40 3 (running)        the command was taken (and what it is doing)
    fail: motor 3 40 -- this robot has 2 wheels: motor 1 or 2, or all
    done: motor 1 40 3                a running command finished
    stopped: motor 1 40 3 (you typed stop -- motors off)

One reader for every program that drives the tool: the cockpit's backend (tool_session.py),
lino_tool.py and robot_bringup.py. The command text never holds '(' or ')' -- the firmware
blanks them -- so the forms cannot be confused with the line they quote.
"""
import re

TIMEOUT_S = 2.0          # an answer comes at once; none in this long is said
_ANSWER = re.compile(r"^\s*(ok|fail|done|stopped): (.*?)(?: \(([^()]*)\)| -- (.*))?\s*$")


def parse(text: str):
    """{"kind": ok|fail|done|stopped, "line", "ok", "msg"} for an answer line, else None.
    "ok" is True for ok and done, False for fail and stopped."""
    m = _ANSWER.match(text or "")
    if not m:
        return None
    kind, line, paren, why = m.groups()
    return {"kind": kind, "line": line.strip(), "ok": kind in ("ok", "done"),
            "msg": (paren if paren is not None else why or "").strip()}


def words(a) -> str:
    """An answer as a person reads it."""
    if a is None:
        return "no answer"
    return f"{a['kind']}: {a['line']}" + (f" ({a['msg']})" if a.get("msg") and a["kind"] != "fail"
                                         else (f" -- {a['msg']}" if a.get("msg") else ""))
