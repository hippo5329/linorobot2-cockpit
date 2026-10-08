"""The drivetrain identification runs inside the base app (2026-10-08).

User: "since we unify the firmware, we can call test_acc with base firmware mode?" -- no
env write, no reboot: micro-ROS, /odom and the LiDAR stay up while it drives.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "firmware")
MAIN = open(os.path.join(ROOT, "src", "main.cpp")).read()
IDENT = open(os.path.join(ROOT, "src", "base_ident.cpp")).read()
OTA = open(os.path.join(ROOT, "common", "lib", "wifi", "ota.cpp")).read()


def test_it_is_started_and_stopped_from_the_ping_port():
    assert 'strncmp(req, "lino-ident", 10)' in OTA and '"lino-ident ok "' in OTA
    assert "hostIdentRequested()" in IDENT and "hostStopRequested()" in IDENT
    assert "hostStopClear();" in IDENT          # a stop that ended the last run must not end the next


def test_moveBase_hands_it_the_motors_and_keeps_everything_else():
    body = MAIN[MAIN.index("void moveBase()"):]
    body = body[:body.index("odometry->update(")]
    i_ident = body.index("baseIdentTick(")
    assert body.index("current_rpm1 = motor1_encoder.getRPM()") < i_ident
    assert i_ident < body.index("motor1_controller.spin(pwm1)")
    assert "else {" in body[i_ident:body.index("motor1_controller.spin(pwm1)")]


def test_operators_win_and_simulated_wheels_are_refused():
    assert '"/cmd_vel took over"' in IDENT and "IDENT refused: the env says these wheels are simulated" in IDENT


def test_every_step_goes_out_and_comes_back():
    steps = re.findall(r"\{K_(DEADZONE|PLANT|LOOP|LIN|SPIN), ([+-]1), ([0-9.]+)f", IDENT)
    seen = {}
    for kind, d, lvl in steps:
        seen.setdefault((kind, lvl), []).append(int(d))
    for key, dirs in seen.items():
        assert sorted(dirs) == [-1, 1], key


def test_it_speaks_test_acc_and_the_parser_reads_it():
    for fmt in ("IDENT %s wheel=%u pwm=%d duty=", "IDENT %s wheel=%u steady_rpm=", "IDENT %s wheel=%u sp=%.1f overshoot=",
                "MAX PWM %6.1f %6.1f", "MAX VEL %6.2f %6.2f m/s", "time to 0.9x max vel", "IDENT done"):
        assert fmt in IDENT, fmt
    code = "\n".join(l.split("//")[0] for l in IDENT.splitlines())
    assert "delay(" not in code                 # it lives inside one control tick
