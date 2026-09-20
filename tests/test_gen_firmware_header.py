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
    assert m["BAUDRATE"] == "921600"
    assert "USE_BTS7960_MOTOR_DRIVER" in m
    assert (m["MOTOR1_PWM"], m["MOTOR1_IN_A"], m["MOTOR1_IN_B"]) == ("25", "21", "17")
    assert (m["MOTOR1_ENCODER_A"], m["MOTOR1_ENCODER_B"]) == ("34", "35")
    assert (m["MOTOR2_ENCODER_A"], m["MOTOR2_ENCODER_B"]) == ("16", "27")
    assert (m["SDA_PIN"], m["SCL_PIN"]) == ("32", "33")
    assert m["LIDAR_RXD"] == "4" and m["LIDAR_SERIAL"] == "1"


def test_dual_core_is_opt_in_and_esp32_only(reference):
    assert "USE_DUAL_CORE" in _header(reference, "esp32")            # use_dual_core: true
    assert "USE_DUAL_CORE" not in _header(reference, "esp32_wifi")   # use_dual_core: false
    # No RP2 port exists: the implementation is xTaskCreatePinnedToCore under
    # `#if defined(ESP32)`, so the macro is withheld whatever the config says.
    assert "USE_DUAL_CORE" not in _header(reference, "pico2_mecanum")


def test_dual_core_absent_key_means_off(reference):
    """The default flipped to OFF on 2026-09-20.

    Splitting the loop across two cores earned its spinlock when the 50 Hz
    topics were still reliable-QoS on an ESP32 serial link. Best effort is the
    default for those topics now, which removed the stall it was compensating
    for, so a config that wants it has to say so.
    """
    params = reference("esp32")
    params["base_controller"].pop("use_dual_core", None)
    assert "USE_DUAL_CORE" not in _macros(gh.generate_header(params, {}, None, True, "jazzy"))


def test_mecanum_four_motors_and_geometry(reference):
    m = _header(reference, "pico2_mecanum")
    assert m["LINO_BASE"] == "MECANUM"
    assert m["MOTOR4_IN_A"] == "10" and m["MOTOR4_ENCODER_A"] == "18"
    assert m["FR_WHEELS_DISTANCE"] == "0.24"
    assert m["COUNTS_PER_REV1"] == "1320"
    assert "USE_MPU6050_IMU" in m


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
        assert "USE_FAKE_WHEEL" in m, mcu
        assert "USE_FAKE_IMU" in m, mcu
        # Nothing optional gets compiled into a board with nothing attached.
        for absent in ("TRIG_PIN", "ECHO_PIN", "BATTERY_PIN", "USE_BMP280"):
            assert absent not in m, (mcu, absent)


def test_stamped_cmd_vel_is_a_distro_fact(reference):
    assert "USE_STAMPED_CMD_VEL" not in _header(reference, "pico2_mecanum", "jazzy")
    assert "USE_STAMPED_CMD_VEL" in _header(reference, "pico2_mecanum", "lyrical")


def test_wifi_capability_is_compiled_from_the_config(reference):
    esp_wifi = _header(reference, "esp32_wifi")
    assert esp_wifi["WIFI_DEFAULT_ENABLED"] == "1"
    esp = _header(reference, "esp32")
    assert esp["WIFI_DEFAULT_ENABLED"] == "0"
