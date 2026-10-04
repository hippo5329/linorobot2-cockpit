"""AUTO with nothing on the bus is a robot with no IMU and no magnetometer.

User, 2026-10-04: "no sim imu on real robot. auto no detect mean no imu no mag."
The firmware kept the name AUTO when the probe found nothing, and createIMU()
turns a name it does not know into a SimIMU: a real robot publishing a simulated
IMU beside its real wheels. Now the probe makes it `none`, the board says so at
boot, the flasher records it, and the topic gate and the EKF stop expecting it.
A chip NAMED in the config that does not answer is still the fatal error it was.
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import flash_mcu  # noqa: E402
import mcu_probe  # noqa: E402

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


def test_the_flasher_reads_the_boot_lines():
    boot = ("[fw] linorobot2_hardware app=base built=x git=y\n"
            "[i2c] 0 device(s) on the bus\n[i2c] no device answered on the bus\n"
            "[i2c] IMU: nothing answered - this robot has none (AUTO)\n"
            "[i2c] MAG: nothing answered - this robot has none (AUTO)\n")
    assert flash_mcu.sensors_absent_from_boot(boot) == ["imu", "mag"]
    assert flash_mcu.sensors_absent_from_boot("[i2c] IMU: ICM20948 at 0x68, as configured\n") == []


def _stamp(d, name, **kw):
    with open(os.path.join(d, name), "w") as fh:
        json.dump(kw, fh)


def test_the_newest_heard_stamp_for_the_port_decides(tmp_path, monkeypatch):
    monkeypatch.setattr(mcu_probe, "STAMP_DIR", str(tmp_path))
    _stamp(tmp_path, "esp32_ttyUSB0.json", flashed_at="2026-10-04T05:00:00",
           banner_confirmed=True, sensors_absent=["imu", "mag"])
    assert mcu_probe.sensors_absent("/dev/ttyUSB0") == {"imu", "mag"}
    # a later flash of the same port, with the IMU found, replaces it
    _stamp(tmp_path, "esp32_lyrical_ttyUSB0.json", flashed_at="2026-10-04T06:00:00",
           banner_confirmed=True, sensors_absent=[])
    assert mcu_probe.sensors_absent("/dev/ttyUSB0") == set()
    # a banner nobody heard says nothing: the config stands
    _stamp(tmp_path, "esp32_jazzy_ttyUSB0.json", flashed_at="2026-10-04T07:00:00",
           banner_confirmed=False, sensors_absent=["imu"])
    assert mcu_probe.sensors_absent("/dev/ttyUSB0") == set()
    assert mcu_probe.sensors_absent("/dev/ttyACM0") == set()


def test_the_gate_and_the_ekf_both_read_it():
    pipe = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()
    assert 'if "imu" in mcu_probe.sensors_absent(serial_port):' in pipe
    launch = open(os.path.join(ROOT, "launchers", "bringup.launch.py")).read()
    assert 'sensors_absent(serial_port)' in launch
    assert 'if has_imu and "imu" in absent:' in launch and '"mag" in absent' in launch


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
