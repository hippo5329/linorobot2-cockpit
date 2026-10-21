"""A required auxiliary topic is judged after a bounded wait for its publisher, not at one instant.

r149 (2026-10-21): an S3 with its BMP280 answering was refused "/pressure NOT ADVERTISED" about a
second after the agent connected, twice, and passed test_sensors on the same board at once.
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import importlib.util  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "vt_imu_split_for_aux", os.path.join(REPO_ROOT, "tests", "test_verify_topics_imu_split.py"))
_split = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_split)
vt = _split._load_module()   # verify_topics with the ROS 2 imports stubbed out


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class _Verifier:
    def __init__(self, clock, appear):
        self.clock, self.appear = clock, appear  # topic -> time its publisher appears

    def aux_publishers(self, topic):
        return 1 if self.clock() >= self.appear.get(topic, float("inf")) else 0


def _spin(clock):
    def spin():
        clock.t += 0.1
    return spin


def test_a_publisher_that_comes_late_is_waited_for():
    c = _Clock()
    v = _Verifier(c, {"/pressure": 1.5, "/temperature": 1.6})
    w = vt.wait_for_advertised(v, ["/pressure", "/temperature"], limit=8.0, clock=c, spin=_spin(c))
    assert 1.4 < w["/pressure"] < 1.7 and 1.5 < w["/temperature"] < 1.8
    assert v.aux_publishers("/pressure") == 1


def test_a_chip_that_never_answers_still_fails_after_the_bound():
    c = _Clock()
    v = _Verifier(c, {})
    w = vt.wait_for_advertised(v, ["/pressure"], limit=8.0, clock=c, spin=_spin(c))
    assert w == {"/pressure": float("inf")}
    assert 7.9 <= c.t <= 8.2


def test_nothing_is_waited_for_when_every_topic_is_there():
    c = _Clock()
    v = _Verifier(c, {"/pressure": 0.0})
    assert vt.wait_for_advertised(v, ["/pressure"], clock=c, spin=_spin(c)) == {}
    assert c.t == 0.0
