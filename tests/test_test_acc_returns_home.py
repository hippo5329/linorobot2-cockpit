"""test_acc comes back to where it started (2026-10-08).

User: "we should recenter the robot before each run, verify the trajectory, change it so
that it return to home." Forward-only identification steps walked the TS100 ~2 m into a
wall: one wheel forward, then the other, is a robot crawling forward in arcs.
"""
import os
import re

SRC = open(os.path.join(os.path.dirname(__file__), "..", "firmware", "src", "tools", "test_acc.cpp")).read()


def _body(name):
    m = re.search(r"\nvoid %s\([^)]*\)\n\{.*?\n\}\n" % name, SRC, re.S)
    assert m, name
    return m.group(0)


def test_every_encoder_read_is_counted():
    # getRPM() is ticks since the previous read: a read outside readRPM() loses travel
    assert re.findall(r"->getRPM\(\)", SRC) == ["->getRPM()"]       # the one in readRPM()
    assert "encN(i)->getRPM()" in SRC


def test_each_identification_step_runs_both_ways():
    for name, kind in (("deadzone", "deadzone_rev"), ("plant", "plant_rev"), ("loopStep", "loop_rev")):
        body = _body(name)
        assert "for (int dir = 1; dir >= -1; dir -= 2)" in body, name
        assert kind in body, name


def test_it_drives_home_after_the_identification_and_after_the_runs():
    assert 'returnHome("ident");' in _body("run")
    assert 'ident::returnHome("runs");' in SRC
    assert "HOME %s net_m=" in SRC
