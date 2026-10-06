"""The virtual gamepad's stall check measures something (it was a stub until 2026-10-06).

/api/gamepad/stall always answered `stalled: false`, so the Teleop tab's "Robot is not moving
-- something is in the way" could never fire, though the page's comment said the server
compared the command against odometry. Now the publisher does: it hears /odom, judges with
scripts/stall_check.py, and writes the verdict the cockpit serves.
"""
import json
import os
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import stall_check as sc  # noqa: E402

FWD = (0.3, 0.0, 0.0)
SPIN = (0.0, 0.0, 1.0)


@pytest.mark.parametrize("cmd, meas, held, age, want", [
    (FWD, (0.0, 0.0, 0.0), 2.0, 0.1, True),          # told to drive, not moving
    (FWD, (0.28, 0.0, 0.0), 2.0, 0.1, False),        # following
    (FWD, (0.0, 0.0, 0.0), 0.5, 0.1, False),         # still spinning up
    (SPIN, (0.0, 0.0, 0.05), 2.0, 0.1, True),        # a turn that does not turn
    (SPIN, (0.0, 0.0, 0.9), 2.0, 0.1, False),
    ((0.02, 0.0, 0.05), (0.0, 0.0, 0.0), 5.0, 0.1, False),   # a creep is not judged
    (FWD, (0.0, 0.0, 0.0), 2.0, None, None),         # no odometry: cannot tell
    (FWD, (0.0, 0.0, 0.0), 2.0, 3.0, None),          # stale odometry: cannot tell
    ((0.3, 0.0, 1.0), (0.3, 0.0, 0.0), 2.0, 0.1, True),      # drives but does not turn
])
def test_the_verdict(cmd, meas, held, age, want):
    assert sc.verdict(cmd, meas, held, age)["stalled"] is want


def _runner(tmp_path):
    sys.path.insert(0, os.path.join(ROOT, "web", "backend"))
    import runners
    r = runners.GamepadRunner(ROOT)
    r.status_file = str(tmp_path / "gp.json")
    return r


def test_the_runner_serves_a_fresh_verdict(tmp_path):
    r = _runner(tmp_path)
    with open(r.status_file, "w") as fh:
        json.dump({"t": time.time(), "stalled": True, "reason": "x",
                   "commanded": {"linear_x": 0.3}, "measured": {"linear_x": 0.0}}, fh)
    st = r.status()
    assert st["stalled"] is True and st["measured"]["linear_x"] == 0.0


def test_a_stale_or_missing_verdict_is_unknown_never_moving(tmp_path):
    r = _runner(tmp_path)
    assert r.status()["stalled"] is None
    with open(r.status_file, "w") as fh:
        json.dump({"t": time.time() - 10, "stalled": False}, fh)
    assert r.status()["stalled"] is None


def test_the_publisher_measures_and_the_endpoint_serves_it():
    pub = open(os.path.join(ROOT, "scripts", "gamepad_publisher.py")).read()
    assert "create_subscription(Odometry" in pub and "stall_check.verdict(" in pub
    assert "--status-file" in open(os.path.join(ROOT, "web", "backend", "runners.py")).read()
    routes = open(os.path.join(ROOT, "web", "backend", "routes_exec.py")).read()
    a = routes.index("def api_gamepad_stall")
    assert "gamepad_runner.status()" in routes[a:a + 400]
    assert '"stalled": False' not in routes[a:a + 400]
