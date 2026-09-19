"""The generated header: the fallback for a blank env, one image per MCU."""
import re

import gen_firmware_header as gh


def _macros(text):
    out = {}
    for m in re.finditer(r"^#define (\w+)(?:\s+(.*))?$", text, re.M):
        out[m.group(1)] = (m.group(2) or "").split("//")[0].strip()
    return out


def _header(reference, name, distro="jazzy"):
    params = reference(name)
    return _macros(gh.generate_header(params, {}, None, no_embed_secrets=True, distro=distro))


def test_gendrv_real_pins_and_driver(reference):
    m = _header(reference, "gendrv_real")
    assert m["BAUDRATE"] == "1500000"
    assert "USE_BTS7960_MOTOR_DRIVER" in m
    assert (m["MOTOR1_PWM"], m["MOTOR1_IN_A"], m["MOTOR1_IN_B"]) == ("25", "21", "17")
    assert (m["MOTOR1_ENCODER_A"], m["MOTOR1_ENCODER_B"]) == ("34", "35")
    assert (m["MOTOR2_ENCODER_A"], m["MOTOR2_ENCODER_B"]) == ("16", "27")
    assert (m["SDA_PIN"], m["SCL_PIN"]) == ("32", "33")
    assert m["LIDAR_RXD"] == "4" and m["LIDAR_SERIAL"] == "1"


def test_dual_core_default_on_for_esp32_unless_refused(reference):
    assert "USE_DUAL_CORE" in _header(reference, "esp32")
    assert "USE_DUAL_CORE" in _header(reference, "gendrv_real")
    assert "USE_DUAL_CORE" not in _header(reference, "esp32_wifi")   # use_dual_core: false
    assert "USE_DUAL_CORE" not in _header(reference, "pico2w")       # no second core


def test_dual_core_absent_key_means_on_for_esp32(reference):
    params = reference("esp32")
    params["base_controller"].pop("use_dual_core", None)
    assert "USE_DUAL_CORE" in _macros(gh.generate_header(params, {}, None, True, "jazzy"))


def test_mecanum_four_motors_and_geometry(reference):
    m = _header(reference, "pico2_mecanum")
    assert m["LINO_BASE"] == "MECANUM"
    assert m["MOTOR4_IN_A"] == "10" and m["MOTOR4_ENCODER_A"] == "18"
    assert m["FR_WHEELS_DISTANCE"] == "0.24"
    assert m["COUNTS_PER_REV1"] == "1320"
    assert "USE_MPU6050_IMU" in m


def test_fake_reference_is_pinless(reference):
    m = _header(reference, "rover_pico2")
    assert m["MOTOR1_PWM"] == "-1" and m["SDA_PIN"] == "-1"
    assert "USE_FAKE_WHEEL" in m


def test_stamped_cmd_vel_is_a_distro_fact(reference):
    assert "USE_STAMPED_CMD_VEL" not in _header(reference, "rover_pico2", "jazzy")
    assert "USE_STAMPED_CMD_VEL" in _header(reference, "rover_pico2", "lyrical")


def test_wifi_capability_is_compiled_from_the_config(reference):
    esp_wifi = _header(reference, "esp32_wifi")
    assert esp_wifi["WIFI_DEFAULT_ENABLED"] == "1"
    esp = _header(reference, "esp32")
    assert esp["WIFI_DEFAULT_ENABLED"] == "0"
