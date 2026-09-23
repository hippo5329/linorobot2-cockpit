"""The generated header: the fallback for a blank env, one image per MCU."""
import re

import gen_firmware_header as gh
from gen_bare_config import bare_config


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
    assert "USE_BTS7960_MOTOR_DRIVER" in m
    assert (m["MOTOR1_PWM"], m["MOTOR1_IN_A"], m["MOTOR1_IN_B"]) == ("25", "21", "17")
    assert (m["MOTOR1_ENCODER_A"], m["MOTOR1_ENCODER_B"]) == ("34", "35")
    assert (m["MOTOR2_ENCODER_A"], m["MOTOR2_ENCODER_B"]) == ("16", "27")
    assert (m["SDA_PIN"], m["SCL_PIN"]) == ("32", "33")
    assert m["LIDAR_RXD"] == "4" and m["LIDAR_SERIAL"] == "1"


def test_dual_core_is_opt_in_and_esp32_only(reference):
    """Both halves are built, not read from whichever file happens to set it.

    This used gendrv as the "off" exemplar, which made it a test of that file
    rather than of the key: when gendrv went to use_dual_core: true on
    2026-09-23 the test went red having found nothing wrong. The key is what is
    under test, so both cases are constructed.
    """
    on = reference("gendrv")
    on["base_controller"]["use_dual_core"] = True
    assert "USE_DUAL_CORE" in _macros(gh.generate_header(on, {}, None, True, "jazzy"))
    off = reference("gendrv")
    off["base_controller"]["use_dual_core"] = False
    assert "USE_DUAL_CORE" not in _macros(gh.generate_header(off, {}, None, True, "jazzy"))
    absent = reference("gendrv")
    absent["base_controller"].pop("use_dual_core", None)
    assert "USE_DUAL_CORE" not in _macros(gh.generate_header(absent, {}, None, True, "jazzy")), \
        "dual core must be opt-in, never the default"
    # No RP2 port exists: the implementation is xTaskCreatePinnedToCore under
    # `#if defined(ESP32)`, so the macro is withheld whatever the config says.
    rp2 = reference("pico2_mecanum")
    rp2["base_controller"]["use_dual_core"] = True
    assert "USE_DUAL_CORE" not in _macros(gh.generate_header(rp2, {}, None, True, "jazzy"))


def test_dual_core_absent_key_means_off(reference):
    """The default flipped to OFF on 2026-09-20.

    Splitting the loop across two cores earned its spinlock when the 50 Hz
    topics were still reliable-QoS on an ESP32 serial link. Best effort is the
    default for those topics now, which removed the stall it was compensating
    for, so a config that wants it has to say so.
    """
    params = reference("gendrv")
    params["base_controller"]["use_dual_core"] = True     # start from the opt-in
    params["base_controller"].pop("use_dual_core", None)  # then take the key away
    assert "USE_DUAL_CORE" not in _macros(gh.generate_header(params, {}, None, True, "jazzy"))


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
        # The fakes are DEFAULTS now, not gates: the drivers are compiled into
        # every image and the env decides. A bare module is the one design that
        # wants them all on with no wiring, so it says so as a value.
        assert m["FAKE_WHEEL_DEFAULT"] == "true", mcu
        assert m["IMU_DEFAULT_NAME"] == '"fake"', mcu
        assert m["FAKE_LD19_DEFAULT"] == "true", mcu
        # Pins stay absent: nothing is wired, so nothing is driven. (The
        # drivers still compile -- range.cpp and battery.cpp take -1 and say
        # so at boot -- which is what lets one image serve a wired board.)
        for absent in ("TRIG_PIN", "ECHO_PIN", "BATTERY_PIN"):
            assert absent not in m, (mcu, absent)


def test_stamped_cmd_vel_is_a_distro_fact(reference):
    assert "USE_STAMPED_CMD_VEL" not in _header(reference, "pico2_mecanum", "jazzy")
    assert "USE_STAMPED_CMD_VEL" in _header(reference, "pico2_mecanum", "lyrical")


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
