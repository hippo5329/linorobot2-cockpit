"""AUTO with nothing on the bus is a robot with no IMU and no magnetometer.

User, 2026-10-04: "no sim imu on real robot. auto no detect mean no imu no mag."
The firmware kept the name AUTO when the probe found nothing, and createIMU()
turns a name it does not know into a SimIMU: a real robot publishing a simulated
IMU beside its real wheels. Now the probe makes it `none`, the board says so at
boot, and the topic gate asks the graph: no /imu/data publisher means no IMU.
A chip NAMED in the config that does not answer is still the fatal error it was.
""" 
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))


PROBE = open(os.path.join(ROOT, "firmware", "common", "lib", "i2c_probe", "i2c_probe.cpp")).read()
MAIN = open(os.path.join(ROOT, "firmware", "src", "main.cpp")).read()


def test_the_probe_turns_auto_with_nothing_into_none_and_keeps_a_named_chip():
    body = PROBE[PROBE.index("static const char *adoptOrNone"):]
    body = body[:body.index("\n}\n")]
    assert 'strcasecmp(configured, "auto") == 0' in body
    assert 'return "none";' in body and "if (!dev && is_auto)" in body
    assert 'adoptOrNone("IMU"' in PROBE and 'adoptOrNone("MAG"' in PROBE


def test_a_nine_axis_imus_own_magnetometer_is_named_not_simulated():
    assert 'if (strcasecmp(imu, "icm20948") == 0) return "icm20948";' in PROBE
    assert 'if (strcasecmp(imu, "mpu9250") == 0)  return "ak8963";' in PROBE
    assert 'if (strcasecmp(imu, "mpu9150") == 0)  return "ak8975";' in PROBE


def test_presence_is_decided_after_the_probe_has_spoken():
    probe_at = MAIN.index("i2cProbeSelect(strcasecmp(imu_name")
    assert MAIN.index('imu_present = (strcasecmp(imu_name, "none") != 0);') > probe_at
    assert MAIN.index('mag_present = (strcasecmp(mag_name, "none") != 0);') > probe_at


def test_the_gate_asks_the_graph_not_the_boot_text():
    """The probe's boot lines print before the host has the port open (the bench, the
    capture held the banner, [uros] and [wdt], and no [i2c]), so they are no
    witness. The firmware makes /imu/data only for an IMU it found: with AUTO,
    no publisher at all is a robot without one."""
    verify = open(os.path.join(ROOT, "scripts", "verify_topics.py")).read()
    assert '"--imu-auto"' in verify
    assert 'verifier.count_publishers("/imu/data") == 0' in verify
    pipe = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()
    assert 'verify_flag += " --imu-auto"' in pipe
    assert 'in ("AUTO", "")' in pipe
    assert "if (imu_present)" in MAIN and "rclc_publisher_init" in MAIN


def test_auto_goes_to_the_board_as_auto_not_sim():
    """The host wrote AUTO as "sim": the board never saw AUTO, and a real robot with
    an empty bus ran a SimIMU. The bus decides; an empty bus is `none` there."""
    import copy, yaml
    import mcu_env
    cfg = yaml.safe_load(open(os.path.join(ROOT, "config", "reference", "gendrv_config.yaml")))
    c = copy.deepcopy(cfg)
    c["base_controller"]["sensors"].update({"imu": "AUTO", "mag": "AUTO"})
    env = mcu_env.hardware_env(c)
    assert env["imu"] == "auto" and env["mag"] == "auto"
    assert mcu_env.robot_has_imu(c, "config")          # the board's report decides later
    assert mcu_env.mixed_simulation(c, "config") is None
    # Sim mode still simulates both, whatever the config says.
    mcu_env.apply_sensor_mode(env, "sim")
    assert env["imu"] == "sim" and env["mag"] == "sim"


def test_an_unresolved_auto_on_the_board_is_none_not_a_simulation():
    assert 'if (strcasecmp(imu_name, "auto") == 0) imu_name = "none";' in MAIN
    assert 'if (strcasecmp(mag_name, "auto") == 0) mag_name = "none";' in MAIN
    assert MAIN.index('imu_name = "none";') < MAIN.index('imu_present = (strcasecmp(imu_name, "none") != 0);')
