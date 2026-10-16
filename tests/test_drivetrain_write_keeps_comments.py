"""drivetrain_report --write keeps the config's comments.

It wrote the measured limits back with yaml.safe_dump, which threw away every
comment in the robot's config: on the TS100 (2026-10-16) the user's dated reasons
for its DDS domain, its LiDAR mask, its BMS cutoff and its heading-turn cap. The
values were right; the file had lost why they were what they were.
"""
import os
import subprocess
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import drivetrain_report as dr  # noqa: E402

REFERENCE = os.path.join(ROOT, "config", "reference", "gendrv_config.yaml")

TRANSCRIPT = """IDENT gains kp=0.6000 ki=0.8000 kd=0.5000 pwm_max=1023 rate_hz=50
IDENT robot base=0 wheels=2 max_rpm=140 ratio=0.850 wheel_d=0.1000
IDENT deadzone wheel=1 pwm=40 duty=0.039
IDENT deadzone wheel=2 pwm=61 duty=0.060
IDENT plant wheel=1 steady_rpm=135.6 tau_ms=182 K=0.13255 peak_rpm=137.0
IDENT plant wheel=2 steady_rpm=118.2 tau_ms=240 K=0.11554 peak_rpm=119.9
IDENT loop wheel=1 sp=23.8 overshoot=0.121 crossings=3 settle_ms=1180 err=0.31 final=23.9
IDENT loop wheel=2 sp=23.8 overshoot=0.201 crossings=5 settle_ms=-1 err=0.88 final=24.6
IDENT done
MAX PWM 1023.0 -1023.0
MAX VEL   0.71   0.00 m/s    0.03 rad/s
time to 0.9x max vel   0.42 sec
"""


def _comments(text):
    return [l[l.index("#"):] for l in text.splitlines() if "#" in l]


def test_changed_scalars_and_list_items_are_rewritten_where_they_stand():
    text = ("a:\n"
            "  # why b is what it is\n"
            "  b: 1.0   # inline reason\n"
            "  v:\n"
            "  - 0.21\n"
            "  - 0.0   # y\n"
            "  - 0.73\n"
            "  mask:\n"
            "  - - 140\n"
            "    - 160\n"
            "c: 'x'\n")
    params = yaml.safe_load(text)
    params["a"]["b"] = 0.961
    params["a"]["v"][0] = 0.23
    params["c"] = "2026-10-16"
    out = dr.edit_in_place(text, params)
    assert out is not None
    assert yaml.safe_load(out) == params
    assert _comments(out) == _comments(text)
    assert "  b: 0.961   # inline reason\n" in out
    assert "  - 0.23\n" in out


def test_a_new_key_goes_under_its_parent_and_the_comments_stay():
    text = "kinematics:\n  # the wheels\n  max_rpm: 150\nnav2:\n  x: 1\n"
    params = yaml.safe_load(text)
    params["kinematics"]["step_response"] = {"max_vel": 0.5037, "measured": "2026-10-16"}
    out = dr.edit_in_place(text, params)
    assert out is not None and yaml.safe_load(out) == params
    assert _comments(out) == _comments(text)
    assert out.index("step_response") < out.index("nav2:")


def test_what_cannot_be_written_in_place_says_so():
    text = "v:\n- 1\n- 2\n"
    assert dr.edit_in_place(text, {"v": [1, 2, 3]}) is None


def test_write_on_a_reference_config_keeps_every_comment(tmp_path):
    """End to end on a shipped config: the measurement and the derived limits land,
    and not one comment is lost."""
    cfg = tmp_path / "robot_config.yaml"
    before = open(REFERENCE, encoding="utf-8").read()
    cfg.write_text(before)
    log = tmp_path / "test_acc.txt"
    log.write_text(TRANSCRIPT)
    res = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "drivetrain_report.py"),
                          "--params", str(cfg), "--from-test-acc", str(log), "--write"],
                         capture_output=True, text=True)
    assert res.returncode in (0, 1), res.stderr
    assert "comments are gone" not in res.stdout
    after = cfg.read_text()
    assert _comments(after) == _comments(before)
    data = yaml.safe_load(after)
    assert data["kinematics"]["step_response"]["max_vel"] > 0
    assert after != before
