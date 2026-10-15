"""The header selects a robot whose name has a hyphen (mc200-unoq).

The backend's /api/robot/select accepts [a-zA-Z0-9_.-] and the firmware maps the name's hyphen
to "_" for its node name (rc-20261009.5), but selectRobot() refused anything outside
[a-z0-9_]: the MC200 could not be chosen in its own web UI (2026-10-15).
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _name_rule():
    js = open(os.path.join(ROOT, "web", "frontend", "app-core.js")).read()
    body = js[js.index("async function selectRobot("):]
    m = re.search(r"if \(!name \|\| !/(.+?)/\.test\(name\)\)", body)
    assert m, "selectRobot's name check moved"
    return re.compile(m.group(1))


def test_a_hyphenated_name_is_selectable():
    rule = _name_rule()
    for ok in ("mc200-unoq", "ts100_gendrv", "bare_sim", "rover2"):
        assert rule.match(ok), ok


def test_shapes_the_backend_would_refuse_stay_refused():
    rule = _name_rule()
    for bad in ("-lead", "Upper", "a b", "../x", "a/b", ""):
        assert not rule.match(bad), bad
