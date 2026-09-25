"""The generated header: the fallback for a blank env, one image per MCU."""
import os
import re

import gen_firmware_header as gh
from gen_bare_config import bare_config

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _macros(text):
    out = {}
    for m in re.finditer(r"^#define (\w+)(?:\s+(.*))?$", text, re.M):
        out[m.group(1)] = (m.group(2) or "").split("//")[0].strip()
    return out


def _header(reference, name, distro="jazzy"):
    params = reference(name)
    return _macros(gh.generate_header(params, {}, None, no_embed_secrets=True, distro=distro))


def test_gendrv_pins_and_driver(reference):
    m = _header(reference, "gendrv")
    assert m["BAUDRATE"] == "1500000"     # the GenDrv's CP2102N rate; see the config
    assert m["MOTOR_DRIVER_DEFAULT"] == '"bts7960"'
    assert (m["MOTOR1_PWM"], m["MOTOR1_IN_A"], m["MOTOR1_IN_B"]) == ("25", "21", "17")
    assert (m["MOTOR1_ENCODER_A"], m["MOTOR1_ENCODER_B"]) == ("34", "35")
    assert (m["MOTOR2_ENCODER_A"], m["MOTOR2_ENCODER_B"]) == ("16", "27")
    assert (m["SDA_PIN"], m["SCL_PIN"]) == ("32", "33")
    assert m["LIDAR_RXD"] == "4" and m["LIDAR_SERIAL"] == "1"


def test_dual_core_is_an_env_key_and_esp32_only(reference):
    """Opt-in, from the env, and only where the silicon has a second core.

    It was the USE_DUAL_CORE macro; a released image is built per MCU, so the
    robot's choice is the env key `dual_core` now (2026-09-25). Both halves are
    constructed, not read from whichever reference happens to set it.
    """
    import mcu_env
    on = reference("gendrv")
    on["base_controller"]["use_dual_core"] = True
    assert mcu_env.hardware_env(on)["dual_core"] == "1"
    off = reference("gendrv")
    off["base_controller"]["use_dual_core"] = False
    assert mcu_env.hardware_env(off)["dual_core"] == "0"
    absent = reference("gendrv")
    absent["base_controller"].pop("use_dual_core", None)
    assert "dual_core" not in mcu_env.hardware_env(absent), "dual core must be opt-in"
    for params in (on, off, absent):
        assert "USE_DUAL_CORE" not in _macros(gh.generate_header(params, {}, None, True, "jazzy"))
    main = open(os.path.join(REPO_ROOT, "firmware", "src", "main.cpp")).read()
    assert re.search(r"#if defined\(ESP32\) && !defined\(CONFIG_FREERTOS_UNICORE\)\s*\n#define HAS_DUAL_CORE", main), \
        "only ESP32 silicon has the second core the key can use"
    assert "static const bool DUAL_CORE_DEFAULT = false;" in main


def test_mecanum_four_motors_and_geometry(reference):
    m = _header(reference, "pico2_mecanum")
    assert m["LINO_BASE"] == "MECANUM"
    assert m["MOTOR4_IN_A"] == "10" and m["MOTOR4_ENCODER_A"] == "18"
    # The shared default 4WD wheelbase. 0.24 put the body's half-diagonal --
    # which includes the wheels sticking out past it -- at 0.280 m, wider than
    # the robot_radius every default shares, so no plan could start.
    assert m["FR_WHEELS_DISTANCE"] == "0.18"
    assert m["COUNTS_PER_REV1"] == "4000"
    # The IMU is a NAME now, not a macro. `USE_MPU6050_IMU` selected a driver
    # at compile time; sensor_factory dispatches on this string at boot and the
    # I2C probe overrides it when the bus disagrees.
    assert m["IMU_DEFAULT_NAME"] == '"mpu6050"' 


def test_generated_bare_config_is_pinless(reference):
    """Was rover_pico2_config.yaml; the bare designs are generated now.

    The rule a bare module has to satisfy is that nothing is driven and nothing
    is waited on, so check it straight out of the generator, on every board --
    a stored file could only ever have checked one.
    """
    for mcu in ("pico", "pico2", "esp32", "esp32s3"):
        m = _macros(gh.generate_header(bare_config(mcu), {}, None, True, "jazzy"))
        assert m["MOTOR1_PWM"] == "-1", mcu
        assert m["SDA_PIN"] == "-1", mcu
        # The sims are DEFAULTS now, not gates: the drivers are compiled into
        # every image and the env decides. A bare module is the one design that
        # wants them all on with no wiring, so it says so as a value.
        assert m["SIM_WHEEL_DEFAULT"] == "true", mcu
        assert m["IMU_DEFAULT_NAME"] == '"sim"', mcu
        assert m["SIM_LD19_DEFAULT"] == "true", mcu
        # Pins stay absent: nothing is wired, so nothing is driven. (The
        # drivers still compile -- range.cpp and battery.cpp take -1 and say
        # so at boot -- which is what lets one image serve a wired board.)
        for absent in ("TRIG_PIN", "ECHO_PIN", "BATTERY_PIN"):
            assert absent not in m, (mcu, absent)


def test_stamped_cmd_vel_is_a_distro_fact_the_firmware_keeps(reference):
    """No macro: the firmware defaults from its own distro, the env overrides."""
    import mcu_env
    for distro in ("jazzy", "lyrical"):
        assert "USE_STAMPED_CMD_VEL" not in _header(reference, "pico2_mecanum", distro)
    params = reference("pico2_mecanum")
    params["base_controller"].pop("stamped_cmd_vel", None)
    params.setdefault("kinematics", {})["stamped_cmd_vel"] = "auto"
    assert "stamped_cmd_vel" not in mcu_env.hardware_env(params), "auto is the firmware's default"
    params["kinematics"]["stamped_cmd_vel"] = True
    assert mcu_env.hardware_env(params)["stamped_cmd_vel"] == "1"


def test_wifi_capability_is_compiled_from_the_config(reference):
    """WIFI_DEFAULT_ENABLED says what the CONFIG wants; the radio code is
    compiled into every ESP32 build regardless, so one image serves both the
    serial robot and the udp4 one and the env key `wifi` picks at boot."""
    assert _header(reference, "gendrv")["WIFI_DEFAULT_ENABLED"] == "1"
    off = reference("gendrv")
    off["base_controller"]["wifi"] = {"enabled": False}
    off["base_controller"]["transport"] = "serial"
    m = _macros(gh.generate_header(off, {}, None, True, "jazzy"))
    assert m["WIFI_DEFAULT_ENABLED"] == "0"
    # ...and the radio is still there to be switched on from the env.
    assert "WIFI_AP_LIST" in m
