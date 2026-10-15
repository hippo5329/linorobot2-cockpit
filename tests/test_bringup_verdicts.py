"""The bring-up steps' verdicts, run through robot_bringup's commands with a fake tool: a wheel
that does not turn fails the rpm step, a pack outside the config's voltage range fails the
battery step, and acc reports what drivetrain_report and the env write really did."""
import os
import shutil
import sys
import types

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import bringup_checklist as bc  # noqa: E402
import robot_bringup as rb  # noqa: E402


class FakeTool:
    """Streams one JSON line per call of lines(): rpm per wheel, or a battery voltage."""

    def __init__(self, rpm_by_wheel=None, bat=None):
        self.rpm_by_wheel, self.bat, self.cur = rpm_by_wheel or {}, bat, None
        self.sent = []

    def send(self, cmd):
        self.sent.append(cmd)
        parts = cmd.split()
        if parts[0] == "motor":
            self.cur = int(parts[1])

    def lines(self, secs):
        import time
        t0 = time.time()
        while time.time() - t0 < 1.7:
            time.sleep(0.05)
        if self.bat is not None:
            for _ in range(5):
                yield "", {"cmd": "sensors", "bat": self.bat}
            return
        rpm = [0.0] * 4
        if self.cur:
            rpm[self.cur - 1] = self.rpm_by_wheel.get(self.cur, 0.0)
        for _ in range(5):
            yield "", {"rpm": rpm}

    def waiting(self, fn, *args):
        return fn(*args)

    def stop(self):
        self.cur = None

    def close(self):
        pass


@pytest.fixture
def robot(tmp_path, monkeypatch):
    monkeypatch.setenv("COCKPIT_CONFIG_DIR", str(tmp_path))
    path = tmp_path / "walker_config.yaml"
    shutil.copy(os.path.join(REPO_ROOT, "config", "reference", "gendrv_config.yaml"), path)
    params = yaml.safe_load(path.read_text())
    r = types.SimpleNamespace(path=str(path), params=params, tgt=params["base_controller"],
                              name="walker", parts=bc.parts(params, str(path)), sim=False)
    return r


def _args(**kw):
    base = dict(wheel=None, pct=40, tacho=None, yes=True, by="test", port=None, ip=None,
                measured=None, write=False)
    base.update(kw)
    return types.SimpleNamespace(**base)


def test_a_wheel_that_does_not_turn_fails_the_rpm_step(robot, monkeypatch):
    n = robot.parts["wheels"]
    rpms = {w: 100.0 for w in range(1, n + 1)}
    rpms[2] = 0.0                                  # wheel 2: dead
    monkeypatch.setattr(rb, "Tool", lambda *a, **k: FakeTool(rpm_by_wheel=rpms))
    monkeypatch.setattr(rb, "ask_float", lambda q: 100.0)
    monkeypatch.setattr(rb, "write_changes", lambda *a, **k: False)
    assert rb.cmd_rpm(robot, _args()) == 1


def test_every_wheel_turning_passes_the_rpm_step(robot, monkeypatch):
    n = robot.parts["wheels"]
    rpms = {w: 100.0 for w in range(1, n + 1)}
    monkeypatch.setattr(rb, "Tool", lambda *a, **k: FakeTool(rpm_by_wheel=rpms))
    monkeypatch.setattr(rb, "ask_float", lambda q: 100.0)
    monkeypatch.setattr(rb, "write_changes", lambda *a, **k: False)
    assert rb.cmd_rpm(robot, _args()) == 0


def _with_range(robot, lo, hi):
    bat = robot.tgt.setdefault("pins", {}).setdefault("battery", {})
    if not isinstance(bat, dict):
        robot.tgt["pins"]["battery"] = bat = {"pin": bat}
    bat.update({"min_v": lo, "max_v": hi})
    robot.parts = bc.parts(robot.params, robot.path)


def test_a_pack_above_the_configs_range_fails_the_battery_step(robot, monkeypatch, capsys):
    if not robot.parts["battery"]:
        pytest.skip("the reference has no battery sense")
    _with_range(robot, 6.0, 8.4)                   # a 2S range ...
    monkeypatch.setattr(rb, "Tool", lambda *a, **k: FakeTool(bat=12.1))   # ... on a 3S pack
    monkeypatch.setattr(rb, "write_changes", lambda *a, **k: False)
    assert rb.cmd_battery(robot, _args(measured=12.1)) == 1
    assert "fewer cells" in capsys.readouterr().out


def test_a_pack_inside_its_range_passes(robot, monkeypatch):
    if not robot.parts["battery"]:
        pytest.skip("the reference has no battery sense")
    _with_range(robot, 9.0, 12.6)
    monkeypatch.setattr(rb, "Tool", lambda *a, **k: FakeTool(bat=11.6))
    monkeypatch.setattr(rb, "write_changes", lambda *a, **k: False)
    assert rb.cmd_battery(robot, _args(measured=11.6)) == 0
