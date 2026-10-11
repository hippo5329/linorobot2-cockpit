"""A core topic that comes in short or silent is measured once more before it fails.

The rate is judged on the first 12 messages, a quarter of a second at 50 Hz, so one
stall in that window halves it: on 2026-10-11 the UNO Q failed /imu/data at 16.8 Hz
(and /odom read 27.6) against 52-60 Hz on every other leg of the same gate. The
second look decides; the first is printed beside it, so a retried pass is visible.
A topic that stays short, or stays silent, still fails.
"""
import importlib.util
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("vt_imu_split", os.path.join(HERE, "test_verify_topics_imu_split.py"))
_split = importlib.util.module_from_spec(spec)
spec.loader.exec_module(_split)
MOD = _split.MOD


class Msg:
    pass


class FakeVerifier:
    """Feeds each topic at the rate its schedule gives for the current look."""

    def __init__(self, schedule, min_samples=12):
        # schedule: topic -> [Hz on look 1, Hz on look 2, ...]; 0 = silent
        self.schedule = schedule
        self.min_samples = min_samples
        self.timestamps = {t: [] for t in schedule}
        self.samples = {t: None for t in schedule}
        self.thresholds = {"/odom": 10.0, "/imu/data": 20.0, "/scan": 5.0}
        self.thresholds = {t: self.thresholds[t] for t in schedule}
        self.optional_samples = {}
        self.look = -1
        self.clock = 0.0

    def is_complete(self):
        return all(len(v) >= self.min_samples for v in self.timestamps.values())

    def tick(self):
        for t, rates in self.schedule.items():
            hz = rates[min(self.look, len(rates) - 1)]
            if hz and len(self.timestamps[t]) < self.min_samples:
                ts = self.timestamps[t]
                ts.append((ts[-1] + 1.0 / hz) if ts else self.clock)
                self.samples[t] = self.samples[t] or Msg()

    def count_publishers(self, topic):
        return 1

    def aux_publishers(self, topic):
        return 0

    def destroy_node(self):
        pass


def run(monkeypatch, capsys, schedule, *argv):
    fake = FakeVerifier(schedule)
    rclpy = MOD.rclpy
    monkeypatch.setattr(rclpy, "init", lambda *a, **k: None, raising=False)
    monkeypatch.setattr(rclpy, "shutdown", lambda *a, **k: None, raising=False)
    monkeypatch.setattr(rclpy, "ok", lambda *a, **k: True, raising=False)
    monkeypatch.setattr(rclpy, "spin_once", lambda v, timeout_sec=0.0: v.tick(), raising=False)
    monkeypatch.setattr(MOD, "TopicVerifier", lambda **kw: fake)
    state = {"in_look": False}

    # each collect() starts a new look: is_complete() false at its first call after a reset
    orig_is_complete = fake.is_complete

    def is_complete():
        if not state.get("in_look"):
            state["in_look"] = True
            fake.look += 1
        done = orig_is_complete()
        if done:
            state["in_look"] = False
        return done
    fake.is_complete = is_complete
    for name in ("format_odom", "format_imu", "format_scan"):
        monkeypatch.setattr(MOD, name, lambda m: "sample")
    monkeypatch.setattr(sys, "argv", ["verify_topics.py", "--no-scan", "--timeout", "0.5", *argv])
    with pytest.raises(SystemExit) as ex:
        MOD.main()
    return ex.value.code, capsys.readouterr().out, fake


def test_a_short_first_look_is_measured_again_and_the_second_decides(monkeypatch, capsys):
    code, out, fake = run(monkeypatch, capsys, {"/odom": [50], "/imu/data": [16.8, 50]})
    assert code == 0, out
    assert "measuring again: /imu/data (16.8 Hz)" in out
    assert "PASS (50.0 Hz >= 20.0 Hz) -- on the second look; the first: 16.8 Hz" in out
    assert fake.look == 1


def test_a_topic_short_twice_still_fails(monkeypatch, capsys):
    code, out, _ = run(monkeypatch, capsys, {"/odom": [50], "/imu/data": [16.8, 15.0]})
    assert code == 1
    assert "FAIL (rate 15.0 Hz < min 20.0 Hz) -- on the second look; the first: 16.8 Hz" in out


def test_a_silent_topic_is_looked_at_twice_then_fails(monkeypatch, capsys):
    code, out, _ = run(monkeypatch, capsys, {"/odom": [50], "/imu/data": [0, 0]})
    assert code == 1
    assert "/imu/data (no data)" in out
    assert "NO DATA (0 msgs received in 0.5s, twice)" in out


def test_a_good_first_look_is_not_repeated(monkeypatch, capsys):
    code, out, fake = run(monkeypatch, capsys, {"/odom": [50], "/imu/data": [50]})
    assert code == 0
    assert "measuring again" not in out and "second look" not in out
    assert fake.look == 0


def test_retries_zero_keeps_the_single_look(monkeypatch, capsys):
    code, out, _ = run(monkeypatch, capsys, {"/odom": [50], "/imu/data": [16.8, 50]}, "--retries", "0")
    assert code == 1
    assert "FAIL (rate 16.8 Hz < min 20.0 Hz)" in out


def test_measured_hz():
    assert MOD.measured_hz([]) == 0.0
    assert MOD.measured_hz([1.0]) == 0.0
    assert MOD.measured_hz([0.0, 0.1, 0.2]) == pytest.approx(10.0)
