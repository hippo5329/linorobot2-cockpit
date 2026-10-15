"""test_acc keeps the EKF and SLAM out of it.

test_acc runs as its own application, so the board publishes no /odom/unfiltered and no
/imu/data for the whole of it, and robot_localization goes on predicting from its last
velocity: on a real robot /odom ran ~300 m in a straight line while the robot spun in place,
SLAM followed, and the next Nav2 goal aborted INCOHERENT. SLAM is paused for test_acc (the
service is a toggle: exactly two calls) and the EKF is set back to the pose /odom had before
test_acc -- only once the robot is back there by the LiDAR.
"""
import importlib.util
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load():
    spec = importlib.util.spec_from_file_location("ocp_ekf", os.path.join(ROOT, "scripts", "one_click_pipeline.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _run(monkeypatch, home_reached: bool, sim: bool = False, pose=(0.5, -0.2, 0.3)):
    ocp = _load()
    calls = []
    monkeypatch.setattr(ocp, "odom_pose", lambda distro: (calls.append("read"), pose)[1])
    monkeypatch.setattr(ocp, "slam_pause_toggle", lambda distro: (calls.append("toggle"), True)[1])
    monkeypatch.setattr(ocp, "ekf_set_pose", lambda p, distro: (calls.append(("set", p)), True)[1])

    def body(*a, sim=False, state=None, **k):
        calls.append("body")
        if home_reached and state is not None:
            state["home"] = True
        return ("", "/tmp/tuned.yaml") if home_reached else ("test_acc: not back where it started", None)
    monkeypatch.setattr(ocp, "_test_acc_body", body)
    args = types.SimpleNamespace(distro="jazzy")
    out = ocp.test_acc_phase(args, {}, {}, "p.yaml", "esp32", "/dev/ttyUSB0", 921600, "esp32", "r", sim=sim)
    return out, calls


def test_slam_paused_around_test_acc_and_the_ekf_set_back_once_home(monkeypatch):
    out, calls = _run(monkeypatch, home_reached=True)
    assert out == ("", "/tmp/tuned.yaml")
    assert calls == ["read", "toggle", "body", ("set", (0.5, -0.2, 0.3)), "toggle"]


def test_not_home_leaves_the_ekf_but_still_resumes_slam(monkeypatch):
    out, calls = _run(monkeypatch, home_reached=False)
    assert out[1] is None
    assert calls == ["read", "toggle", "body", "toggle"]


def test_the_sim_mcu_never_stops_publishing_so_nothing_is_touched(monkeypatch):
    out, calls = _run(monkeypatch, home_reached=True, sim=True)
    assert calls == ["body"]


def test_the_body_marks_home_after_the_lidar_return():
    src = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py"), encoding="utf-8").read()
    body = src[src.index("def _test_acc_body"):]
    i = body.index('state["home"] = True')
    assert body.index("return \"test_acc: not back where it started") < i < body.index("changed = drivetrain_report.tune_from_test_acc")


def test_the_ekf_is_stopped_before_the_drive_home_and_a_refusal_says_why(monkeypatch, capsys, tmp_path):
    """The drive home is closed on /odom, which ran off for the whole of test_acc (a TS100,
    2026-10-15: 258 m, the drive home refused, the EKF never set back). Once the board is
    back, the EKF goes to the start snapshot's pose BEFORE the drive home; a refused drive
    home prints its own reason."""
    ocp = _load()
    calls = []

    def clearance_step(args, params, cfg, need, save=None, max_travel=None):
        open(save, "w").write('{"odom": [0.4, -0.1, 0.2]}')
        return True
    monkeypatch.setattr(ocp, "clearance_step", clearance_step)
    monkeypatch.setattr(ocp, "test_acc_room", lambda params, c: 1.0)
    monkeypatch.setattr(ocp, "LOG_DIR", str(tmp_path))
    monkeypatch.setattr(ocp, "run_streamed", lambda *a, **k: (calls.append("test_acc"), types.SimpleNamespace(returncode=0))[1])
    monkeypatch.setattr(ocp, "wait_for_topic", lambda *a, **k: True)
    monkeypatch.setattr(ocp, "ekf_set_pose", lambda p, distro: (calls.append(("set", p)), True)[1])
    monkeypatch.setattr(ocp, "wants_stamped_cmd_vel", lambda *a: False)
    monkeypatch.setattr(ocp, "holonomic_flag", lambda p: "")
    monkeypatch.setattr(ocp, "home_guesses", lambda t: "")
    monkeypatch.setattr(ocp, "blind_flags", lambda c, look=False: "")

    def run_ros(cmd, timeout=None, distro=None):
        calls.append("home")
        return types.SimpleNamespace(returncode=1, stdout="", stderr="HOME REFUSED: /odom moved 1.2 m at rest")
    monkeypatch.setattr(ocp, "run_ros", run_ros)
    monkeypatch.setitem(sys.modules, "gen_firmware_header", types.SimpleNamespace(nav2_robot_radius=lambda p: 0.2))
    args = types.SimpleNamespace(distro="jazzy", test_acc_clearance=1.0)
    state = {}
    msg, tuned = ocp._test_acc_body(args, {}, {"transport": "serial", "baudrate": 921600}, "p.yaml", "esp32",
                                    "/dev/ttyUSB0", 921600, "esp32", "r", sim=False, state=state)
    assert calls == ["test_acc", ("set", (0.4, -0.1, 0.2)), "home"]
    assert tuned is None and "not back where it started" in msg and "home" not in state
    assert "HOME REFUSED: /odom moved 1.2 m at rest" in capsys.readouterr().out
