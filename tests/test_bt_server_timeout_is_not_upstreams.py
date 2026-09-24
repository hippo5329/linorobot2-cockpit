"""`default_server_timeout` must stay well above Nav2's 20 ms default.

It is how long a behaviour-tree action node waits for a server to ACKNOWLEDGE a
goal -- not to do the work. Upstream ships 20 ms, and at 20 ms a brief hiccup in
`controller_server` aborts the entire navigation:

    Begin navigating from current location (0.20, -0.21) to (3.00, 0.00)
    Timed out while waiting for action server to acknowledge goal request for follow_path
    goalCompleted error 107: Behavior Tree action client timed out waiting..

The leg is then scored as a navigation failure with the robot untouched at home,
`traversed 0.001 m`. Measured twice in 261 rounds of the 2026-09-24 soak (rounds 157
and 258), both on `follow_path`.

~0.8% of legs is small in a soak and NOT small in a gate: across 30 legs it is
better than a one-in-five chance of one spurious red per matrix, and a red with a
Nav2 error code on it reads like a robot fault. This is a plausible share of the
gate flakiness this project has been unable to attribute.

Pinned by a test because the value is invisible: nothing fails when it is wrong, a
small fraction of legs simply lie. Regenerating a config from an upstream template,
or copying a params file from a Nav2 tutorial, would silently restore 20 ms.
"""
import glob
import os

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Above any plausible scheduler hiccup on a loaded bench, and still ~4500x below
# action_server_result_timeout, so a genuinely dead server is still caught quickly.
MIN_MS = 100


def _configs():
    found = sorted(glob.glob(os.path.join(REPO_ROOT, "config", "reference", "*_config.yaml")))
    assert found, "no reference configs found; this test has stopped checking anything"
    return found


def test_every_reference_config_raises_the_bt_server_timeout():
    for path in _configs():
        with open(path, encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh)
        bt = cfg.get("nav2", {}).get("bt_navigator", {}).get("ros__parameters", {})
        assert "default_server_timeout" in bt, \
            f"{os.path.basename(path)}: bt_navigator has no default_server_timeout"
        got = bt["default_server_timeout"]
        assert got >= MIN_MS, (
            f"{os.path.basename(path)}: default_server_timeout is {got} ms. "
            f"At Nav2's 20 ms default a hiccup in controller_server aborts the whole "
            f"navigation with error_code=107 and the leg is blamed on the robot "
            f"(measured twice in 261 soak rounds). Keep it >= {MIN_MS} ms.")


def test_the_reason_is_recorded_beside_the_value():
    """A bare number invites being 'tidied' back to the upstream default.

    The comment is the only thing that tells the next reader this is deliberate, so
    it is part of the fix rather than decoration.
    """
    for path in _configs():
        text = open(path, encoding="utf-8").read()
        i = text.index("default_server_timeout")
        preceding = text[max(0, i - 1400):i]
        assert "107" in preceding and "acknowledge" in preceding, (
            f"{os.path.basename(path)}: default_server_timeout has lost the note "
            "explaining why it is not upstream's 20 ms")


def test_all_configs_agree_on_the_value():
    """One default chassis: the Nav2 template is shared, so a divergence here means
    one robot silently got the flaky timeout back."""
    values = {}
    for path in _configs():
        with open(path, encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh)
        values[os.path.basename(path)] = (cfg["nav2"]["bt_navigator"]["ros__parameters"]
                                          ["default_server_timeout"])
    assert len(set(values.values())) == 1, f"configs disagree: {values}"
