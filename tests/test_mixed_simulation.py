"""Sim and real cannot mix; the result of a mix is useless.

A real IMU with simulated wheels or a simulated LiDAR tells the odometry, the EKF
and SLAM about a different robot each. A bare module simulates everything, a real
robot nothing; the diagnostic tools are the exception by design (they test one real
part of a half-built robot). The rule lives in mcu_env.mixed_simulation() and is
enforced where the env is written and again at bringup.
"""
import copy
import os
import subprocess
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import mcu_env  # noqa: E402

REF = os.path.join(ROOT, "config", "reference")


def _ref(name):
    with open(os.path.join(REF, f"{name}_config.yaml")) as fh:
        return yaml.safe_load(fh)


def _sensors(cfg, **flags):
    c = copy.deepcopy(cfg)
    c["base_controller"].setdefault("sensors", {}).update(flags)
    return c


ALL_SIM = dict(use_sim_imu=True, use_sim_mag=True, use_sim_wheel=True, use_sim_ld19=True,
               use_sim_env=True, use_sim_battery=True)


def test_a_real_reference_design_is_all_real_in_every_mode():
    for name in ("gendrv", "pico2_mecanum", "xrp", "yb_eet01"):
        cfg = _ref(name)
        for mode in ("config", "real", "sim"):
            assert mcu_env.mixed_simulation(cfg, mode) is None, (name, mode)


def test_a_robot_simulated_all_the_way_through_runs():
    cfg = _sensors(_ref("gendrv"), **ALL_SIM)
    assert mcu_env.mixed_simulation(cfg, "config") is None


def test_a_real_imu_on_simulated_wheels_is_refused():
    cfg = _sensors(_ref("gendrv"), **dict(ALL_SIM, use_sim_imu=False))
    why = mcu_env.mixed_simulation(cfg, "config")
    assert why and "Sim and real cannot mix" in why and "useless" in why
    assert "IMU (qmi8658)" in why and "wheels" in why
    # Sim mode simulates the whole robot, so the same config runs there.
    assert mcu_env.mixed_simulation(cfg, "sim") is None


def test_real_wheels_with_a_simulated_lidar_are_refused_unless_real_mode():
    cfg = _sensors(_ref("gendrv"), use_sim_ld19=True)
    why = mcu_env.mixed_simulation(cfg, "config")
    assert why and "LiDAR (use_sim_ld19)" in why
    assert mcu_env.mixed_simulation(cfg, "real") is None


def test_a_real_robot_with_no_imu_named_runs_a_simulated_one_and_is_refused():
    # imu: NONE reaches the board as "sim": on real wheels that is a gyro nothing measured.
    cfg = _sensors(_ref("gendrv"), imu="NONE")
    why = mcu_env.mixed_simulation(cfg, "real")
    assert why and "no real IMU named" in why


def test_imu_auto_on_a_real_robot_is_the_bus_probe_not_a_simulation():
    cfg = _sensors(_ref("gendrv"), imu="AUTO")
    assert mcu_env.mixed_simulation(cfg, "config") is None


def test_the_sim_mcu_is_never_mixed():
    cfg = _sensors(_ref("gendrv"), **dict(ALL_SIM, use_sim_imu=False))
    cfg["base_controller"]["mcu"] = "host"
    assert mcu_env.mixed_simulation(cfg, "config") is None


def _build(tmp_path, cfg, app):
    p = tmp_path / "robot_config.yaml"
    p.write_text(yaml.safe_dump(cfg))
    s = tmp_path / "secrets.yaml"
    s.write_text("{}\n")
    return subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "mcu_env.py"), "build",
                           "--params", str(p), "--secrets", str(s), "--host-ip", "127.0.0.1",
                           "--out", str(tmp_path / "env.bin"), "--set", f"app={app}"],
                          capture_output=True, text=True)


def test_the_env_writer_refuses_a_mixed_robot_firmware_and_writes_nothing(tmp_path):
    cfg = _sensors(_ref("gendrv"), use_sim_ld19=True)
    res = _build(tmp_path, cfg, "base")
    assert res.returncode == mcu_env.MIXED_SIM_EXIT, res.stdout + res.stderr
    assert mcu_env.MIXED_SIM_TAG in res.stdout
    assert not (tmp_path / "env.bin").exists()


def test_a_diagnostic_tool_on_a_half_built_robot_is_not_judged_by_this_rule(tmp_path):
    # test_sensors on a real IMU with the wheels not wired yet: that is how a robot is built.
    cfg = _sensors(_ref("gendrv"), use_sim_wheel=True, use_sim_ld19=True)
    res = _build(tmp_path, cfg, "test_sensors")
    assert res.returncode == 0, res.stdout + res.stderr


def test_flash_stops_on_the_refusal_and_bringup_checks_too():
    flash = open(os.path.join(ROOT, "scripts", "flash_mcu.py")).read()
    assert "res.returncode == 3" in flash and "mcu_env.MIXED_SIM_EXIT" in flash
    assert mcu_env.MIXED_SIM_EXIT == 3
    launch = open(os.path.join(ROOT, "launchers", "bringup.launch.py")).read()
    assert "mcu_env.mixed_simulation(" in launch and "raise RuntimeError" in launch
