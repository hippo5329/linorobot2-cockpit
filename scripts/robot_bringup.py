#!/usr/bin/env python3
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
"""Bring up a real robot, one checked step at a time -- the command-line twin of the web UI's
Bring-up checklist (Hardware Tests). Both record into the same file, so a step done in one
shows as done in the other, and the 1-Click asks the same record before it moves a robot.

    robot_bringup.py status                       the checklist, and whether the 1-Click may run
    robot_bringup.py sensors  [--seconds 20]      turn and shake the robot; every IMU/mag must follow
    robot_bringup.py battery  --measured 11.62    the multimeter's reading; trims an ADC divider
    robot_bringup.py motors   [--pct 40]          each wheel forward: did it turn forward? fixes inverts
    robot_bringup.py rpm      [--pct 100]         each wheel at power, a tachometer's RPM: CPR, max_rpm
    robot_bringup.py cpr-turns --wheel 1 --turns 10   counts per rev by turning a wheel by hand
    robot_bringup.py lidar    [--seconds 10]      rate, coverage, ranges (robot stack running)
    robot_bringup.py speed                        speed_check.py: wheels vs LiDAR (floor, 0.6 m clear)
    robot_bringup.py acc      [--write]           test_acc with 1.0 m clear and back home (floor)
    robot_bringup.py mark STAGE pass|fail|skip [--summary TEXT]
    robot_bringup.py gate                         exit 0 when the 1-Click may move this robot

Every step works on the active robot (or --params). The tool steps (sensors, battery,
motors, rpm, cpr-turns) put the board in its tool -- over USB with --port, else over Wi-Fi
-- and always end with `stop` and `base`. A calibration is written to the robot's config
only after you say yes (or with --yes), and then the board's env block is written (4 KB,
no reflash) so it runs the new values; the step is re-run to prove it. Nothing moves on
the floor before `lidar` has passed; `speed` and `acc` check the room themselves.

THE WHEELS TURN in motors, rpm and battery's load test: lift the robot first.
THE ROBOT DRIVES in speed and acc: on the floor, with room all round.
"""
import argparse
import json
import os
import shlex
import statistics
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import bringup_checklist as bc  # noqa: E402
import cockpit_paths  # noqa: E402


SIM_FLAG = [False]
WALK_FLAG = [None]


def say(msg=""):
    print(msg, flush=True)


class Robot:
    def __init__(self, params_path: str = None):
        import mcu_env
        self.path = os.path.abspath(params_path or cockpit_paths.active_robot_path())
        if not os.path.isfile(self.path):
            raise SystemExit(f"no robot config at {self.path}: give --params")
        self.reload()
        self.env = mcu_env.env_from_config(self.path, cockpit_paths.secrets_path())

    def reload(self):
        import mcu_env
        self.params = mcu_env.load_yaml(self.path)
        self.tgt = self.params.get("base_controller") or {}
        self.name = mcu_env.robot_name_of(self.params, self.path)
        self.pio_env = str(self.tgt.get("pio_env") or self.tgt.get("mcu") or "")
        self.parts = bc.parts(self.params, self.path)
        # The Sim MCU: its tool is the host firmware, started by this program (--sim, or a
        # config whose controller is the Sim MCU); it reads the config each time it starts.
        self.sim = SIM_FLAG[0] or str(self.tgt.get("name") or "").lower() == "sim" \
            or str(self.tgt.get("mcu") or "").lower() in ("host", "sim")


def ask(question: str, choices: str, default: str = None, yes: bool = False) -> str:
    """One key from `choices`; --yes takes the default."""
    if yes and default:
        say(f"{question} [{choices}] -> {default} (--yes)")
        return default
    if not sys.stdin.isatty():
        raise SystemExit(f"{question}: needs an answer (run in a terminal, or --yes)")
    while True:
        a = input(f"{question} [{choices}] ").strip().lower()[:1]
        if not a and default:
            return default
        if a and a in choices:
            return a
        say(f"  one of: {', '.join(choices)}")


def ask_float(question: str) -> float:
    if not sys.stdin.isatty():
        raise SystemExit(f"{question}: needs a number (run in a terminal)")
    while True:
        raw = input(f"{question} ").strip()
        try:
            return float(raw)
        except ValueError:
            say("  a number, please")


# ---------------------------------------------------------------- the tool link
class Tool:
    """The board's tool shell (lino_tool), read line by line with [TOOL_JSON] parsed."""

    def __init__(self, robot: Robot, port: str = None, ip: str = None):
        import lino_tool
        self.lt = lino_tool
        password = str(robot.env.get("ota_password") or "")
        baud = int(robot.env.get("baud") or 921600)
        if robot.parts and port is None and not robot.parts["wifi"]:
            port = str(robot.tgt.get("serial_port") or "") or None
        where = "the Sim MCU" if robot.sim else ("USB " + port if port else "Wi-Fi")
        say(f"entering the tool on {where} (the robot leaves micro-ROS until this step ends)")
        self.link = lino_tool.connect(port, robot.pio_env, baud, robot.path, ip, password, robot.env,
                                      sim=robot.sim, walk=WALK_FLAG[0])
        if self.link is None:
            raise SystemExit("the tool was not reached (see above)")
        self.buf = b""
        self.driving = False
        self.backlog = []

    def send(self, line: str) -> dict:
        """One command, and the board's answer to it (taken or refused); a refusal or no
        answer stops this step -- nothing is assumed to run."""
        self.link.write(line.encode() + b"\r\n")
        self.driving = self.driving or line.split()[0] in ("motor", "motors", "ramp", "acc")
        import tool_answer
        for text, j in self.lines(self.lt.ACK_TIMEOUT_S, keep=True):
            a = tool_answer.parse(text)
            if not a or a["kind"] not in ("ok", "fail") or a["line"] != line.strip():
                continue
            say(f"  {tool_answer.words(a)}")
            if not a["ok"]:
                raise SystemExit(f"the board refused '{line}': {a.get('msg')}")
            return a
        self.stop()
        raise SystemExit(f"no answer from the board to '{line}' in {self.lt.ACK_TIMEOUT_S:.0f} s "
                         "(flash the MCU: older firmware does not answer; or the link is down)")

    def waiting(self, question, *args):
        """Ask the person while a drive runs on. The tool's deadman stops a drive whose
        console goes quiet, and a person reading a tachometer is quiet for as long as it
        takes: the link keeps its keep-alives and is read meanwhile (the board's lines
        would otherwise fill it), and a drive that ended anyway is said."""
        import threading
        import tool_answer
        done, seen = threading.Event(), []

        def pump():
            while not done.is_set() and not getattr(self.link, "closed", False):
                self.link.keepalive()
                d = self.link.read()
                if d:
                    seen.append(d)
                else:
                    time.sleep(0.05)

        t = threading.Thread(target=pump, daemon=True)
        t.start()
        try:
            return question(*args)
        finally:
            done.set()
            t.join(2.0)
            for raw in b"".join(seen).decode(errors="replace").splitlines():
                a = tool_answer.parse(raw)
                if a and a["kind"] in ("stopped", "fail"):
                    say(f"  {tool_answer.words(a)}")

    def lines(self, seconds: float, keep: bool = False):
        """(text, json-or-None) for every line the board prints in `seconds` -- first the
        ones read while waiting for an answer (send). keep=True: also kept for the next call."""
        if not keep:
            while self.backlog:
                yield self.backlog.pop(0)
        end = time.time() + seconds
        while time.time() < end:
            self.link.keepalive()
            d = self.link.read()
            if not d:
                if getattr(self.link, "closed", False):
                    raise SystemExit("the board hung up")
                time.sleep(0.02)
                continue
            self.buf += d
            while b"\n" in self.buf:
                raw, self.buf = self.buf.split(b"\n", 1)
                text = raw.decode(errors="replace").rstrip("\r")
                j = None
                at = text.find("[TOOL_JSON] ")
                if at >= 0:
                    try:
                        j = json.loads(text[at + 12:])
                    except ValueError:
                        j = None
                if keep:
                    self.backlog.append((text, j))
                yield text, j

    def stop(self):
        self.link.write(b"stop\r\n")
        self.backlog.clear()
        for _ in self.lines(1.0):
            pass
        self.driving = False

    def close(self):
        """stop, then base: the board boots the robot again."""
        try:
            if not getattr(self.link, "closed", False):
                self.link.write(b"stop\r\n")
                self.lt.pump(self.link, 0.5)
                self.link.write(b"base\r\n")
                self.lt.pump(self.link, 3, until=(b"restarting into base",))
        finally:
            self.link.close()


def env_write(robot: Robot, port: str = None) -> bool:
    """The robot's env block from its (new) config, app=base: 4 KB, no reflash. Over USB
    when the port is here, else over the air (flash_mcu decides)."""
    if robot.sim:
        say("the Sim MCU reads the config each time it starts: nothing to write")
        return True
    import mcu_identity
    env = mcu_identity.pio_env_for(robot.pio_env, "pico2")
    port = port or str(robot.tgt.get("serial_port") or "/dev/ttyACM0")
    baud = int(robot.env.get("baud") or 921600)
    argv = [sys.executable, "-u", os.path.join(HERE, "flash_mcu.py"), "--env", env, "--env-only",
            "--port", port, "--params", robot.path, "--app", "base", "--baud", str(baud)]
    say("writing the board's env block (app=base, the new values)...")
    r = subprocess.run(argv)
    return r.returncode == 0


def write_changes(robot: Robot, changes: dict, a) -> bool:
    if not changes:
        return False
    say("the config change:")
    for k, v in changes.items():
        say(f"    {k}: {bc._get(robot.params, k)} -> {v}")
    if ask("write it to the robot's config and its board?", "yn", "y" if a.yes else None, a.yes) != "y":
        say("  not written")
        return False
    done = bc.apply(robot.path, changes)
    robot.reload()
    for k, v in done.items():
        say(f"    wrote {k}: {v['from']} -> {v['to']}")
    if not env_write(robot, a.port):
        say("  the env write FAILED: the config has the new values, the board does not yet")
        return False
    return True


# ---------------------------------------------------------------- steps
def cmd_status(robot: Robot, a) -> int:
    c = bc.checklist(robot.name, robot.params, robot.path)
    p = c["parts"]
    say(f"robot {robot.name} ({os.path.basename(robot.path)})")
    has = [k for k in ("imu", "mag", "battery", "lidar") if p[k]]
    say(f"  real parts: wheels {'real' if p['wheels_real'] else 'SIMULATED'} ({p['wheels']}), "
        + (", ".join(has) or "no sensors") + (", Wi-Fi" if p["wifi"] else ", serial"))
    if not c["real_robot"]:
        say("  simulated wheels: not a real robot -- nothing here gates it")
    for r in c["stages"]:
        mark = {"pass": "PASS ", "fail": "FAIL ", "skip": "skip ", "stale": "STALE",
                "todo": "todo ", "not needed": "  -  "}[r["state"]]
        gate = "*" if r["gate"] and r["required"] else " "
        say(f"  {mark} {gate} {r['stage']:8} {r['title']}"
            + (f"  ({r['at']}, {r['by']})" if r.get("at") else ""))
        if r["summary"]:
            say(f"              {r['summary']}")
    say("  (* = the 1-Click needs it on this robot)")
    if c["gate_ok"]:
        say("1-Click: may move this robot")
    else:
        say("1-Click: HELD until --")
        for m in c["missing"]:
            say(f"    {m}")
    return 0


def cmd_gate(robot: Robot, a) -> int:
    ok, missing = bc.gate(robot.name, robot.params, robot.path)
    say("BRINGUP_GATE OK" if ok else "BRINGUP_GATE HELD: " + "; ".join(missing))
    return 0 if ok else 1


def cmd_mark(robot: Robot, a) -> int:
    e = bc.mark(robot.name, robot.params, a.stage, a.status, a.summary or "", by=a.by)
    say(f"{a.stage}: {e['status']} ({e['at']})")
    return 0


def collect_sensors(tool: Tool, seconds: float, hz: int = 10) -> list:
    tool.send(f"sensors {hz}")
    out = []
    for text, j in tool.lines(seconds):
        if j and j.get("cmd") == "sensors":
            out.append(j)
        elif text.startswith(("[+]", "[-]", "[!]")) or "not found" in text or "initialized" in text:
            say(f"  {text}")
    tool.stop()
    return out


def cmd_sensors(robot: Robot, a) -> int:
    p = robot.parts
    if not (p["imu"] or p["mag"]):
        say("this robot's config names no real IMU or magnetometer: nothing to check")
        bc.mark(robot.name, robot.params, "sensors", "skip", "no real IMU or magnetometer", by=a.by)
        return 0
    tool = Tool(robot, a.port, a.ip)
    try:
        say("hold the robot STILL for 5 s...")
        rest = collect_sensors(tool, 5.0)
        say(f"now TURN it (yaw, pitch, roll) and SHAKE it for {a.seconds:.0f} s...")
        moving = collect_sensors(tool, a.seconds)
    finally:
        tool.close()
    ok, lines, m = bc.sensors_judge(rest, moving, p)
    for ln in lines:
        say("  " + ln)
    say(f"SENSORS {'PASS' if ok else 'FAIL'} ({len(rest) + len(moving)} readings)")
    bc.mark(robot.name, robot.params, "sensors", "pass" if ok else "fail", "; ".join(lines),
            values=m, by=a.by)
    return 0 if ok else 1


def cmd_battery(robot: Robot, a) -> int:
    p = robot.parts
    if not p["battery"]:
        say("this robot's config has no battery sense: nothing to check")
        bc.mark(robot.name, robot.params, "battery", "skip", "no battery sense", by=a.by)
        return 0
    tool = Tool(robot, a.port, a.ip)
    try:
        say("reading the battery for 5 s...")
        rows = collect_sensors(tool, 5.0, hz=5)
    finally:
        tool.close()
    vs = [r["bat"] for r in rows if isinstance(r.get("bat"), (int, float))]
    if not vs:
        say("BATTERY FAIL: the board reported no battery voltage")
        bc.mark(robot.name, robot.params, "battery", "fail", "no voltage reported", by=a.by)
        return 1
    rep = statistics.median(vs)
    meas = a.measured if a.measured is not None else ask_float("the multimeter reads (V):")
    diff = rep - meas
    say(f"  the board reports {rep:.2f} V, the multimeter {meas:.2f} V ({diff:+.2f} V)")
    if p["battery_adc"]:
        bat = (robot.tgt.get("pins") or {}).get("battery") or {}
        r1, r2 = float(bat.get("r1") or 0), float(bat.get("r2") or 0)
        if abs(diff) <= 0.02 * meas:
            ok, summary = True, f"{rep:.2f} V vs {meas:.2f} V on the meter: within 2 %"
        else:
            try:
                t = bc.battery_trim(r1, r2, rep, meas)
            except ValueError as e:
                say(f"BATTERY FAIL: {e}")
                bc.mark(robot.name, robot.params, "battery", "fail", str(e), by=a.by)
                return 1
            say(f"  the divider: R1 {r1:g} -> {t['r1']:g} (R2 {r2:g}), ratio x{t['scale']}")
            if write_changes(robot, bc.changes_for(robot.params, "battery", r1=t["r1"]), a):
                say("  run `battery` again to confirm the new reading")
            bc.mark(robot.name, robot.params, "battery", "fail",
                    f"trimmed R1 {r1:g} -> {t['r1']:g}; confirm with a second reading", by=a.by)
            return 1
    else:
        # A current monitor (INA219) reads its own volts; behind the board's input diode it
        # reads ~0.6 V under the pack, which bat_min/bat_max are then stated against.
        ok = abs(diff) <= 1.0
        summary = (f"{p['battery_chip'] or 'monitor'} reads {rep:.2f} V, meter {meas:.2f} V ({diff:+.2f} V)"
                   + ("" if ok else ": more than 1 V apart -- check the wiring"))
        if ok and abs(diff) > 0.2:
            say("  (a monitor behind an input diode reads ~0.6 V under the pack: state "
                "pins.battery min_v/max_v as the board reads them)")
    say(f"BATTERY {'PASS' if ok else 'FAIL'}: {summary}")
    bc.mark(robot.name, robot.params, "battery", "pass" if ok else "fail", summary,
            values={"reported": rep, "measured": meas}, by=a.by)
    return 0 if ok else 1


def run_wheel(tool: Tool, wheel: int, pct: int, secs: int) -> list:
    """RPM samples of one wheel while it runs `secs` at `pct` (the first second dropped)."""
    tool.send(f"motor {wheel} {pct} {secs}")
    rpms, t0 = [], time.time()
    for text, j in tool.lines(secs + 2.0):
        if j and isinstance(j.get("rpm"), list) and time.time() - t0 > 1.2:
            rpms.append(float(j["rpm"][wheel - 1]))
        if "DIR WRONG" in text or "RAMP" in text:
            say(f"  {text.strip()}")
    tool.stop()
    return rpms


def cmd_motors(robot: Robot, a) -> int:
    n = robot.parts["wheels"]
    wheels = [a.wheel] if a.wheel else list(range(1, n + 1))
    say(f"LIFT THE ROBOT: wheels {wheels} will turn FORWARD at {a.pct} % for {a.secs} s each")
    ask("wheels off the floor?", "y", "y", a.yes)
    tool = Tool(robot, a.port, a.ip)
    fixes, results = {}, {}
    try:
        for w in wheels:
            say(f"wheel {w}: forward...")
            rpms = run_wheel(tool, w, a.pct, a.secs)
            rpm = statistics.median(rpms) if rpms else 0.0
            say(f"  encoder: {rpm:+.1f} RPM")
            seen = ask(f"  did wheel {w} turn the way that drives the robot FORWARD? "
                       "(f = forward, b = backward, n = did not turn)", "fbn")
            if seen == "n":
                results[w] = "did not turn"
                say("  check the motor's supply, its driver pins and the battery")
                continue
            try:
                fix = bc.invert_fix(seen == "f", rpm)
            except ValueError as e:
                results[w] = str(e)
                say(f"  {e}")
                continue
            if fix["flip_motor"] or fix["flip_encoder"]:
                fixes[w] = fix
                results[w] = ("invert the motor" if fix["flip_motor"] else "") + \
                    (" and " if fix["flip_motor"] and fix["flip_encoder"] else "") + \
                    ("invert the encoder" if fix["flip_encoder"] else "")
            else:
                results[w] = f"right ({rpm:+.0f} RPM)"
            say(f"  wheel {w}: {results[w]}")
    finally:
        tool.close()
    changes = {}
    for w, fix in fixes.items():
        changes.update(bc.changes_for(robot.params, "invert", wheel=w, **fix))
    wrote = write_changes(robot, changes, a)
    ok = all(v.startswith("right") for v in results.values()) and len(results) == len(wheels)
    summary = "; ".join(f"wheel {w}: {v}" for w, v in results.items())
    if wrote:
        summary += " -- fixed; run `motors` again to confirm"
    say(f"MOTORS {'PASS' if ok else 'FAIL'}: {summary}")
    if len(wheels) == n:
        bc.mark(robot.name, robot.params, "motors", "pass" if ok else "fail", summary,
                values={str(k): v for k, v in results.items()}, by=a.by)
    return 0 if ok else 1


def cmd_rpm(robot: Robot, a) -> int:
    import gen_firmware_header
    n = robot.parts["wheels"]
    wheels = [a.wheel] if a.wheel else list(range(1, n + 1))
    cpr = gen_firmware_header.counts_per_rev(robot.params.get("kinematics") or {})
    say(f"LIFT THE ROBOT: each wheel runs at {a.pct} % until you have the tachometer reading "
        f"(counts_per_rev now {cpr})")
    ask("wheels off the floor, reflective tape on each wheel?", "y", "y", a.yes)
    tool = Tool(robot, a.port, a.ip)
    rows = {}
    try:
        for w in wheels:
            say(f"wheel {w} at {a.pct} %: point the tachometer at it")
            tool.send(f"motor {w} {a.pct} 0")
            rpms, t0 = [], time.time()
            for _, j in tool.lines(4.0):
                if j and isinstance(j.get("rpm"), list) and time.time() - t0 > 1.5:
                    rpms.append(abs(float(j["rpm"][w - 1])))
            enc = statistics.median(rpms) if rpms else 0.0
            say(f"  encoder: {enc:.1f} RPM")
            tacho = a.tacho if a.tacho is not None else \
                tool.waiting(ask_float, f"  the tachometer reads (RPM) for wheel {w}:")
            tool.stop()
            rows[w] = {"encoder": round(enc, 1), "tacho": tacho}
    finally:
        tool.close()
    cprs, tachos, notes = [], [], []
    for w, r in rows.items():
        try:
            cprs.append(bc.cpr_from_tacho(cpr, r["encoder"], r["tacho"]))
            tachos.append(r["tacho"])
            notes.append(f"wheel {w}: encoder {r['encoder']:.0f} / tacho {r['tacho']:.0f} RPM")
        except ValueError as e:
            notes.append(f"wheel {w}: {e}")
    if not cprs:
        say("RPM FAIL: " + "; ".join(notes))
        bc.mark(robot.name, robot.params, "rpm", "fail", "; ".join(notes), by=a.by)
        return 1
    new_cpr = int(round(statistics.median(cprs)))
    off = abs(new_cpr - cpr) / cpr
    changes = {}
    if off > 0.02:
        changes.update(bc.changes_for(robot.params, "cpr", cpr=new_cpr))
    if a.pct == 100 and len(tachos) == n:
        # max_rpm is what the slowest wheel reaches at full power: the speed every wheel can hold
        cur = (robot.params.get("kinematics") or {}).get("max_rpm")
        slow = min(tachos)
        if cur is None or abs(slow - float(cur)) / max(slow, 1.0) > 0.05:
            changes.update(bc.changes_for(robot.params, "max_rpm", rpm=slow))
    for ln in notes:
        say("  " + ln)
    say(f"  counts_per_rev {cpr} -> {new_cpr} ({off * 100:.1f} % off)")
    wrote = write_changes(robot, changes, a)
    # Only a counts_per_rev correction needs a second run to confirm; max_rpm is the
    # measurement itself, written as read (a tracked robot: the step failed on a max_rpm write
    # alone and named a counts_per_rev change it had not made).
    ok = off <= 0.02
    summary = "; ".join(notes) + (f"; counts_per_rev {cpr} agrees" if ok else
                                  f"; counts_per_rev {cpr} -> {new_cpr}" +
                                  (", confirm with a second run" if wrote else ", not written"))
    if wrote and "kinematics.max_rpm" in changes:
        summary += f"; max_rpm {changes['kinematics.max_rpm']}"
    say(f"RPM {'PASS' if ok else 'FAIL'}: {summary}")
    if len(wheels) == n:
        bc.mark(robot.name, robot.params, "rpm", "pass" if ok else "fail", summary,
                values={str(k): v for k, v in rows.items()}, by=a.by)
    return 0 if ok else 1


def cmd_cpr_turns(robot: Robot, a) -> int:
    import gen_firmware_header
    cpr = gen_firmware_header.counts_per_rev(robot.params.get("kinematics") or {})
    say(f"mark wheel {a.wheel}, then turn it FORWARD by hand exactly {a.turns:g} turns; "
        "press Enter when done")
    tool = Tool(robot, a.port, a.ip)
    try:
        tool.send("enc")
        first = last = None
        if sys.stdin.isatty():
            import select
            t_end = time.time() + 600
            while time.time() < t_end:
                for _, j in tool.lines(0.3):
                    if j and isinstance(j.get("count"), list):
                        v = int(j["count"][a.wheel - 1])
                        first = v if first is None else first
                        last = v
                        print(f"\r  count {last - first:+d}   ", end="", flush=True)
                r, _, _ = select.select([sys.stdin], [], [], 0)
                if r:
                    sys.stdin.readline()
                    break
        tool.stop()
    finally:
        tool.close()
    say("")
    if first is None:
        say("CPR FAIL: the encoder reported nothing")
        return 1
    try:
        new = bc.cpr_from_turns(last - first, a.turns)
    except ValueError as e:
        say(f"CPR FAIL: {e}")
        return 1
    say(f"  {last - first:+d} counts in {a.turns:g} turns: {new} counts per rev (config: {cpr})")
    if last - first < 0:
        say("  the count went DOWN on a forward turn: the encoder needs inverting (run `motors`)")
    if abs(new - cpr) / cpr > 0.02:
        write_changes(robot, bc.changes_for(robot.params, "cpr", cpr=new), a)
    return 0


def ros_tool(robot: Robot, a, script: str, extra: list) -> tuple:
    """A ROS-side check (it enters the stack's environment itself): (exit code, its lines)."""
    argv = [sys.executable, "-u", os.path.join(HERE, script), "--params", robot.path] + extra
    say("$ " + " ".join(shlex.quote(x) for x in argv[2:]))
    p = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    lines = []
    for ln in p.stdout:
        print(ln, end="", flush=True)
        lines.append(ln.rstrip("\n"))
    rc = p.wait()
    return rc, lines


def cmd_lidar(robot: Robot, a) -> int:
    if not robot.parts["lidar"]:
        say("this robot's config has no real LiDAR: nothing to check")
        bc.mark(robot.name, robot.params, "lidar", "skip", "no real LiDAR", by=a.by)
        return 0
    rc, lines = ros_tool(robot, a, "lidar_check.py", ["--seconds", str(a.seconds)])
    res = {}
    for ln in lines:
        if ln.startswith("LIDAR_CHECK_JSON "):
            try:
                res = json.loads(ln[len("LIDAR_CHECK_JSON "):])
            except ValueError:
                pass
    verdict = next((ln for ln in lines if ln.startswith("LIDAR_CHECK ")), "LIDAR_CHECK FAIL: no verdict")
    ok = rc == 0 and verdict.startswith("LIDAR_CHECK PASS")
    summary = (f"{res.get('hz')} Hz, {res.get('beams')} beams, {int((res.get('coverage') or 0) * 100)} % "
               f"coverage, nearest {res.get('nearest')} m" if res.get("scans") else verdict)
    if not ok:
        summary = verdict.replace("LIDAR_CHECK ", "")
    bc.mark(robot.name, robot.params, "lidar", "pass" if ok else "fail", summary, values=res, by=a.by)
    return 0 if ok else 1


def cmd_speed(robot: Robot, a) -> int:
    held = _floor_held(robot)
    if held:
        say(f"SPEED REFUSED: {held}")
        return 2
    out = os.path.join(tempfile.gettempdir(), f"speed_check_{os.getpid()}.json")
    rc, lines = ros_tool(robot, a, "speed_check.py", ["--json", out])
    if any(ln.startswith("SPEED_CHECK REFUSED") for ln in lines) or rc == 2:
        return 2                        # nothing moved: no verdict to record
    try:
        res = json.load(open(out))
    except (OSError, ValueError):
        res = {}
    import speed_check
    rat = res.get("ratios") or {}
    bad = {k: v for k, v in rat.items() if k in ("lin", "ang", "lat") and not speed_check.fine(v, 0.05)}
    ok = rc == 0 and bool(rat) and not bad
    summary = ", ".join(f"{k} LiDAR/odom {v:.3f}" for k, v in rat.items())
    if bad:
        summary += " -- outside 5 %: apply the change printed above (wheel_diameter / angular_scale)"
    say(f"SPEED {'PASS' if ok else 'FAIL'}: {summary or 'no ratios'}")
    bc.mark(robot.name, robot.params, "speed", "pass" if ok else "fail", summary, values=rat, by=a.by)
    return 0 if ok else 1


def _floor_held(robot: Robot) -> str:
    """Why the robot may not drive on the floor yet: the bench steps first."""
    rows = {r["stage"]: r for r in bc.checklist(robot.name, robot.params, robot.path)["stages"]}
    need = [s for s in ("motors", "rpm", "lidar") if rows[s]["required"] and rows[s]["state"] not in ("pass", "skip")]
    return ("the bench steps first: " + ", ".join(need)) if need else ""


def cmd_acc(robot: Robot, a) -> int:
    """test_acc on the floor: the room checked (1.0 m all round), the run, the robot driven
    back to where it started by the LiDAR and the EKF set back -- the 1-Click's --test-acc
    step on its own, with the robot brought up and nothing else running."""
    import gen_firmware_header
    import one_click_pipeline as ocp
    # A Wi-Fi board sends its results by syslog; a serial one (the UNO Q) prints them on its
    # tty, which test_acc_run.py --serial reads while the agent there is paused.
    on_serial = not robot.parts["wifi"]
    held = _floor_held(robot)
    if held:
        say(f"ACC REFUSED: {held}")
        return 2
    distro = os.environ.get("ROS_DISTRO") or "auto"
    stamped = " --stamped" if ocp.wants_stamped_cmd_vel(distro, robot.tgt, robot.params) else ""
    snap = os.path.join(tempfile.gettempdir(), f"bringup_acc_{os.getpid()}.json")
    radius = gen_firmware_header.nav2_robot_radius(robot.params)
    say(f"the room: {a.clear:.1f} m all round (THE ROBOT DRIVES AT FULL POWER next)")
    chk = ocp.run_ros(f"python3 {os.path.join(HERE, 'clearance.py')} check --need {a.clear:.2f} --save {snap}",
                      timeout=60, distro=distro)
    for ln in (chk.stdout or "").splitlines():
        say("  " + ln)
    if chk.returncode != 0:
        say(f"ACC REFUSED: less than {a.clear:.1f} m clear -- move the robot (nothing moved)")
        return 2
    found = {} if on_serial else ocp.wireless_board(robot.tgt, robot.path, robot.params)
    if not on_serial and not found.get("ip"):
        say("ACC REFUSED: the board did not answer on Wi-Fi")
        return 2
    pose0 = ocp.odom_pose(distro)
    transcript = os.path.join(ocp.LOG_DIR, f"test_acc_{time.strftime('%Y%m%d-%H%M%S')}.txt")
    # LOG_DIR is the 1-Click's per-run folder, which only the 1-Click creates: run on its own,
    # test_acc lost its whole transcript at the end (a tracked robot) -- the board had finished.
    os.makedirs(os.path.dirname(transcript), exist_ok=True)
    import mcu_identity
    port = str((a.port if on_serial else None) or robot.tgt.get("serial_port") or "/dev/ttyACM0")
    baud = int(robot.env.get("baud") or 921600)
    # the env writes go at the upload rate (esptool's ceiling); a serial robot's console is
    # read at the micro-ROS rate
    argv = [sys.executable, "-u", os.path.join(HERE, "test_acc_run.py"), "--params", robot.path,
            "--env", mcu_identity.pio_env_for(robot.pio_env, "esp32"), "--port", port,
            "--baud", str(min(baud, ocp.FLASH_BAUD_CEILING)),
            "--controller", str(robot.tgt.get("name") or robot.pio_env),
            "--transcript", transcript] + (["--serial", "--console-baud", str(baud)] if on_serial
                                           else ["--ip", found["ip"]])
    res = subprocess.run(argv)
    ocp.wait_for_topic("/odom/unfiltered", timeout_sec=120, require_publisher=True, distro=distro)
    ocp.wait_for_topic("/scan", timeout_sec=90, distro=distro, require_message="header.frame_id")
    say("back to where test_acc started, by the LiDAR...")
    home = ocp.run_ros(f"python3 {os.path.join(HERE, 'clearance.py')} home --snap {snap} "
                       f"--radius {radius:.2f}{stamped}", timeout=180, distro=distro)
    for ln in (home.stdout or "").splitlines():
        say("  " + ln)
    if home.returncode == 0 and pose0:
        say(f"  the EKF set back: {'done' if ocp.ekf_set_pose(pose0, distro) else 'FAILED'}")
    if res.returncode != 0:
        say("ACC FAIL: test_acc did not finish (see above)")
        bc.mark(robot.name, robot.params, "acc", "fail", "test_acc did not finish", by=a.by)
        return 1
    argv = [sys.executable, os.path.join(HERE, "drivetrain_report.py"), "--params", robot.path,
            "--from-test-acc", transcript]
    if a.write:
        argv.append("--write")
    subprocess.run(argv)
    summary = f"transcript {os.path.basename(transcript)}" + ("; gains written" if a.write else "")
    if a.write:
        robot.reload()
        env_write(robot, a.port)
    else:
        say(f"to keep the derived gains: robot_bringup.py acc --write, or drivetrain_report.py "
            f"--params {robot.path} --from-test-acc {transcript} --write")
    ok = home.returncode == 0
    say(f"ACC {'PASS' if ok else 'FAIL'}: {summary}" + ("" if ok else "; NOT back home by the LiDAR"))
    bc.mark(robot.name, robot.params, "acc", "pass" if ok else "fail", summary,
            values={"transcript": transcript}, by=a.by)
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--params", help="the robot's config (default: the active robot)")
    ap.add_argument("--port", help="the board's USB serial port (default: Wi-Fi for a Wi-Fi robot)")
    ap.add_argument("--ip", help="the board's address on Wi-Fi (default: found by its name)")
    ap.add_argument("--yes", action="store_true", help="take the default answer to every question")
    ap.add_argument("--sim", action="store_true", help="the Sim MCU's tool (default for a Sim MCU robot)")
    ap.add_argument("--walk", action="store_true",
                    help="with the Sim MCU: its walkthrough mode -- its own models count as parts")
    ap.add_argument("--by", default="cli", help=argparse.SUPPRESS)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    sub.add_parser("gate")
    m = sub.add_parser("mark")
    m.add_argument("stage", choices=bc.STAGES)
    m.add_argument("status", choices=bc.STATUSES)
    m.add_argument("--summary", default="")
    s = sub.add_parser("sensors")
    s.add_argument("--seconds", type=float, default=20.0)
    b = sub.add_parser("battery")
    b.add_argument("--measured", type=float, help="the multimeter's reading on the battery, V")
    mo = sub.add_parser("motors")
    mo.add_argument("--wheel", type=int, choices=(1, 2, 3, 4))
    mo.add_argument("--pct", type=int, default=40)
    mo.add_argument("--secs", type=int, default=3)
    r = sub.add_parser("rpm")
    r.add_argument("--wheel", type=int, choices=(1, 2, 3, 4))
    r.add_argument("--pct", type=int, default=100)
    r.add_argument("--tacho", type=float, help="the tachometer's RPM (one wheel; else asked per wheel)")
    c = sub.add_parser("cpr-turns")
    c.add_argument("--wheel", type=int, choices=(1, 2, 3, 4), required=True)
    c.add_argument("--turns", type=float, default=10.0)
    li = sub.add_parser("lidar")
    li.add_argument("--seconds", type=float, default=10.0)
    sub.add_parser("speed")
    ac = sub.add_parser("acc")
    ac.add_argument("--clear", type=float, default=1.0, help="room needed all round, m (1.0)")
    ac.add_argument("--write", action="store_true", help="keep the derived gains and limits in the config")
    a = ap.parse_args(argv)
    SIM_FLAG[0] = a.sim
    WALK_FLAG[0] = True if a.walk else None
    robot = Robot(a.params)
    fn = {"status": cmd_status, "gate": cmd_gate, "mark": cmd_mark, "sensors": cmd_sensors,
          "battery": cmd_battery, "motors": cmd_motors, "rpm": cmd_rpm, "cpr-turns": cmd_cpr_turns,
          "lidar": cmd_lidar, "speed": cmd_speed, "acc": cmd_acc}[a.cmd]
    return fn(robot, a)


if __name__ == "__main__":
    sys.exit(main())
