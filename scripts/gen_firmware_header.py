#!/usr/bin/env python3
# ==============================================================================
# gen_firmware_header.py — Linorobot2 Firmware Header Generator
#
# Generates firmware/include/custom/lino_base_config.h from the
# single source of truth (<config dir>/<robot>_config.yaml) and its secrets.yaml.
#
# Enforces:
# 1. Parenthesized PWM_MAX cast: ((float)(pow(2, PWM_BITS) - 1))
# 2. Bare-board fake IMU / fake MAG definitions to prevent I2C bus stall timeouts
# 3. Base controller selection (gendrv, pico, pico2) -- one per robot config
# ==============================================================================

import argparse
import math
import os
import sys
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mcu_env   # resolve_syslog_port: one copy of the 514 -> 5140 rule

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ROBOT = "rover_pico2"
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cockpit_paths  # noqa: E402
import pin_catalog  # noqa: E402
import gen_robot_description  # noqa: E402  (geometry: the fake LiDAR raycasts from the configured pose)
DEFAULT_PARAMS = cockpit_paths.robot_config_path()
DEFAULT_SECRETS = cockpit_paths.secrets_path()
DEFAULT_OUT = os.path.join(REPO_ROOT, "firmware", "include", "custom", "lino_base_config.h")


def load_yaml(path):
    if not os.path.isfile(path):
        return {}
    with open(path, "r") as f:
        try:
            return yaml.safe_load(f) or {}
        except yaml.YAMLError as e:
            print(f"Error parsing {path}: {e}", file=sys.stderr)
            sys.exit(1)


WIFI_TRANSPORTS = ("udp4", "udp", "wifi")
DEFAULT_PIO_INI = os.path.join(REPO_ROOT, "firmware", "platformio.ini")


def env_uses_wifi_transport(env_name, ini_path=DEFAULT_PIO_INI):
    """True/False if the PlatformIO env builds micro-ROS with the Wi-Fi transport.

    Returns None when the question cannot be answered -- no ini, unknown env, or
    `board_microros_transport = custom`, which is the normal case now. An env
    built with the custom transport contains BOTH paths and chooses between them
    at boot from the env partition (firmware/common/lib/uros_transport), so
    there is no compile-time pairing left to get wrong and nothing for the
    caller to reject. The check below stays for envs that still pin a transport.
    """
    import configparser

    if not env_name or not os.path.isfile(ini_path):
        return None
    cp = configparser.ConfigParser(strict=False)
    try:
        # Both files, because `extends = base_esp32` points at a section that
        # lives in the shared base ini. Reading only the per-firmware ini makes
        # the chain dead-end at an unknown section, and the walk below then
        # falls through -- which used to be reported as "serial" rather than as
        # "unknown", and rejected a perfectly good config.
        cp.read([ini_path, os.path.join(os.path.dirname(ini_path), "..", "common",
                                        "platformio_base.ini")])
    except configparser.Error:
        return None

    section = f"env:{env_name}"
    if not cp.has_section(section):
        return None
    # `extends` chains are shallow here (env -> base_<board>); follow them anyway
    # so a transport set on a shared base section is not missed.
    seen = set()
    while section and section not in seen:
        seen.add(section)
        if cp.has_option(section, "board_microros_transport"):
            declared = cp.get(section, "board_microros_transport").strip()
            if declared == "custom":
                return None      # both transports present; nothing to pair
            return declared == "wifi"
        parent = cp.get(section, "extends", fallback="").strip()
        section = parent if parent and cp.has_section(parent) else None
    # Nothing in the chain declared a transport. That is "cannot tell", not
    # "serial": answering False here asserts something the ini never said.
    return None


RP2_MCUS = ("pico", "picow", "pico2", "pico2w")


def check_pico_family_transport(params, controller_name):
    """Reject udp4 on the RP2 family — the radio cannot carry the control loop.

    picow/pico2w do have Wi-Fi, so this is not a capability check and the board
    is not radioless: the CYW43 hangs off an SPI/PIO link, and that link cannot
    sustain the 50 Hz control loop once micro-ROS traffic rides over it. Wi-Fi on
    those boards is there for **syslog and OTA**, which are bursty and tolerate
    the bandwidth; the micro-ROS transport is not, and stays serial.

    The firmware already refuses at boot -- uros_transport gates UROS_HAVE_UDP on
    ESP32 and prints "this board has no Wi-Fi -- using serial" -- but that is a
    runtime fallback discovered on hardware, after a build and a flash. A config
    that asks for it is wrong, so say so before either happens.
    """
    tgt = params.get("base_controller") or {}
    transport = tgt.get("transport", "serial")
    if transport not in WIFI_TRANSPORTS:
        return
    mcu = str(tgt.get("mcu") or tgt.get("pio_env") or controller_name or "").lower()
    if mcu not in RP2_MCUS:
        return
    print(
        f"Error: base_controller '{tgt.get('name', controller_name)}' asks for transport "
        f"'{transport}' on '{mcu}'. The RP2 family is excluded from the Wi-Fi micro-ROS "
        f"transport: the CYW43 radio is behind an SPI/PIO link that cannot sustain the "
        f"50 Hz control loop. Use 'serial'. (Wi-Fi on picow/pico2w remains available for "
        f"syslog and OTA, which this does not disable.)",
        file=sys.stderr)
    sys.exit(1)


def check_transport_env_pairing(params, controller_name):
    """Fail early when a config's transport and its PlatformIO env disagree.

    A serial env built against a Wi-Fi header fails deep in the micro-ROS
    library with `micro_ros_agent_locator has incomplete type`, which says
    nothing about the actual cause. The config is the thing that is wrong, so
    name it here instead.
    """
    tgt = params.get("base_controller") or {}
    transport = tgt.get("transport", "serial")
    env_name = tgt.get("pio_env") or tgt.get("mcu") or controller_name
    env_is_wifi = env_uses_wifi_transport(env_name)
    if env_is_wifi is None:
        return
    wants_wifi = transport in WIFI_TRANSPORTS
    if wants_wifi == env_is_wifi:
        return

    if wants_wifi:
        detail = (
            f"transport '{transport}' needs a Wi-Fi-transport env, but env "
            f"'{env_name}' builds micro-ROS over serial. The generated header "
            f"defines AGENT_IP / USE_WIFI_TRANSPORT and the build will fail with "
            f"'micro_ros_agent_locator has incomplete type'."
        )
    else:
        detail = (
            f"transport '{transport}' is a serial link, but env '{env_name}' sets "
            f"board_microros_transport = wifi, so the firmware will look for a "
            f"UDP agent that the header never configures."
        )
    print(f"Error: base_controller '{tgt.get('name', controller_name)}' is inconsistent — {detail}",
          file=sys.stderr)
    print("Fix base_controller.transport or base_controller.pio_env so the two agree.",
          file=sys.stderr)
    sys.exit(1)



# The generator names motor drivers as USE_<X>_MOTOR_DRIVER macros; the env
# names them as short lowercase strings. One mapping, so the YAML, the env and
# the firmware cannot drift apart.
_DRIVER_ENV_NAMES = {
    "GENERIC_2_IN": "generic2",
    "GENERIC_1_IN": "generic1",
    "BTS7960": "bts7960",
    "ESC": "esc",
}


def _driver_env_name(driver_macro):
    return _DRIVER_ENV_NAMES.get(str(driver_macro).upper(), "generic2")


# nav2 1.4 (kilted) flipped nav2_util::TwistPublisher to TwistStamped by default,
# so every distro from kilted on drives /cmd_vel stamped and jazzy and older drive
# it plain. Spelled as "everything newer than the last unstamped release" rather
# than a list of stamped ones, so the release after lyrical does not silently fall
# back to the jazzy contract the way an allow-list would.
UNSTAMPED_CMD_VEL_DISTROS = ("humble", "iron", "jazzy")


def distro_stamps_cmd_vel(distro) -> bool:
    d = (distro or "").strip().lower()
    if not d:
        return False
    return d not in UNSTAMPED_CMD_VEL_DISTROS


def counts_per_rev(kine: dict) -> int:
    """Encoder counts per WHEEL revolution.

    `counts_per_rev` is the number the firmware wants, and the one people get
    wrong: 11 PPR x 4 (quadrature) x 30:1 is 1320, and a factor left out puts
    the odometry off by 4x or 30x. Given the parts instead -- `encoder_ppr`,
    `gear_ratio`, `quadrature` (4 unless stated) -- the product is computed
    here; `counts_per_rev` still wins when it is present.
    """
    if kine.get("counts_per_rev") is not None:
        return int(kine["counts_per_rev"])
    ppr = kine.get("encoder_ppr")
    if ppr is not None:
        gear = float(kine.get("gear_ratio", 1))
        quad = int(kine.get("quadrature", 4))
        return int(round(float(ppr) * quad * gear))
    return 144000


def config_warnings(params: dict) -> list:
    """Things the ROS side of a config gets wrong for its base type."""
    out = []
    base = str((params.get("kinematics") or {}).get("base_type", "2wd")).lower()

    # Two shapes are in the wild: the flat one the launchers wrap
    # (`ekf: {frequency: ..}`, `nav2: {controller_server: {..}}`) and the
    # already-wrapped one (`ekf: {ekf_filter_node: {ros__parameters: {..}}}`).
    def node_params(section, node=None):
        section = section or {}
        if node and node in section:
            section = section[node] or {}
        return section.get("ros__parameters", section) or {}

    if base == "mecanum":
        ekf = node_params(params.get("ekf"), "ekf_filter_node")
        cfg = ekf.get("odom0_config")
        if isinstance(cfg, list) and len(cfg) > 7 and not cfg[7]:
            out.append("mecanum base but ekf odom0_config does not fuse vy (index 7 is false)")
        ctl = node_params(params.get("nav2"), "controller_server")
        thr = ctl.get("min_y_velocity_threshold")
        if thr is not None and float(thr) >= 0.1:
            out.append(f"mecanum base but nav2 controller_server.min_y_velocity_threshold is {thr} (a differential value; use 0.001)")
        vs = node_params(params.get("nav2"), "velocity_smoother")
        mv = vs.get("max_velocity")
        if isinstance(mv, list) and len(mv) > 1 and float(mv[1]) == 0:
            out.append("mecanum base but nav2 velocity_smoother.max_velocity[1] (vy) is 0")
    out.extend(gen_robot_description.geometry_warnings(params))
    return out


def generate_header(params, secrets, controller_name, no_embed_secrets=False, distro=None):
    # One robot per config file, one base controller per robot -- the block is
    # the controller; there is no map to index into.
    tgt = params.get("base_controller") or {}
    if not isinstance(tgt, dict) or not tgt:
        print("Error: config has no base_controller: block.", file=sys.stderr)
        print("Run scripts/migrate_config_schema.py to convert a legacy targets: file.", file=sys.stderr)
        sys.exit(1)

    declared = tgt.get("name")
    if controller_name and declared and controller_name != declared:
        print(f"Note: config declares base_controller '{declared}', generating for '{controller_name}'.", file=sys.stderr)
    controller_name = controller_name or declared or "pico2"
    kine = params.get("kinematics", {})
    base_type = kine.get("base_type", "2wd").lower()

    # Map base type to macro
    base_macro = "DIFFERENTIAL_DRIVE"
    if base_type in ("4wd", "skid_steer"):
        base_macro = "SKID_STEER"
    elif base_type == "mecanum":
        base_macro = "MECANUM"

    # Kinematics & specs
    wheel_diam = float(kine.get("wheel_diameter", 0.152))
    lr_dist = float(kine.get("lr_wheels_distance", 0.271))
    fr_dist = float(kine.get("fr_wheels_distance", 0.0))
    max_rpm = int(kine.get("max_rpm", 140))
    rpm_ratio = float(kine.get("max_rpm_ratio", 0.85))
    cpr = counts_per_rev(kine)
    pwm_bits = int(kine.get("pwm_bits", 10))
    pwm_freq = int(kine.get("pwm_frequency", 20000))
    op_v = float(kine.get("motor_operating_voltage", 24.0))
    max_v = float(kine.get("motor_power_max_voltage", 12.0))

    pid = kine.get("pid", {})
    kp = float(pid.get("kp", 0.6))
    ki = float(pid.get("ki", 0.8))
    kd = float(pid.get("kd", 0.5))

    # Target hardware
    driver = tgt.get("driver_type", "GENERIC_2_IN")
    baudrate = int(tgt.get("baudrate", 921600))
    sensors = tgt.get("sensors", {})
    pins = tgt.get("pins", {})
    lidar = tgt.get("lidar", {})

    lines = [
        "// ==============================================================================",
        f"// Auto-generated by gen_firmware_header.py for base controller: {controller_name}",
        "// DO NOT EDIT DIRECTLY. Update config/<robot>_config.yaml and regenerate.",
        "// ==============================================================================",
        "#ifndef LINO_BASE_CONFIG_H",
        "#define LINO_BASE_CONFIG_H",
        "",
        "#include <math.h>",
        "",
        f"// --- Robot Base Kinematics ---",
        f"#define LINO_BASE {base_macro}",
        f"#define USE_{driver}_MOTOR_DRIVER",
        "// The same two facts as strings, for the env. The macros above still",
        "// drive the compile-time paths; these are what createKinematics() and",
        "// createMotor() fall back to when the env partition says nothing, so a",
        "// board with a blank env is wired exactly as this config describes.",
        f'#define KINEMATICS_BASE_DEFAULT "{base_macro.lower()}"',
        f'#define MOTOR_DRIVER_DEFAULT "{_driver_env_name(driver)}"',
        "",
        f"// --- PID Tuning ---",
        f"#define K_P {kp}",
        f"#define K_I {ki}",
        f"#define K_D {kd}",
        "",
        f"// --- Kinematics & Motor Geometry ---",
        f"#define MOTOR_MAX_RPM {max_rpm}",
        f"#define MAX_RPM_RATIO {rpm_ratio}",
        f"#define MOTOR_OPERATING_VOLTAGE {op_v}",
        f"#define MOTOR_POWER_MAX_VOLTAGE {max_v}",
        f"#define COUNTS_PER_REV1 {cpr}",
        f"#define COUNTS_PER_REV2 {cpr}",
        f"#define COUNTS_PER_REV3 {cpr}",
        f"#define COUNTS_PER_REV4 {cpr}",
        f"#define WHEEL_DIAMETER {wheel_diam}",
        f"#define LR_WHEELS_DISTANCE {lr_dist}",
        f"#define FR_WHEELS_DISTANCE {fr_dist}",
        f"#define PWM_BITS {pwm_bits}",
        f"#define PWM_FREQUENCY {pwm_freq}",
        "",
        "// --- Inviolable Core Invariant: Parenthesized PWM_MAX cast ---",
        "#define PWM_MAX ((float)(pow(2, PWM_BITS) - 1))",
        "#define PWM_MIN (-PWM_MAX)",
        "",
    ]

    # Motor pins
    m1 = pins.get("motor1", {})
    m2 = pins.get("motor2", {})
    lines.extend([
        "// --- Motor Pins ---",
        f"#define MOTOR1_PWM {m1.get('pwm', -1)}",
        f"#define MOTOR1_IN_A {m1.get('in_a', -1)}",
        f"#define MOTOR1_IN_B {m1.get('in_b', -1)}",
        f"#define MOTOR1_INV {str(m1.get('invert', False)).lower()}",
        "",
        f"#define MOTOR2_PWM {m2.get('pwm', -1)}",
        f"#define MOTOR2_IN_A {m2.get('in_a', -1)}",
        f"#define MOTOR2_IN_B {m2.get('in_b', -1)}",
        f"#define MOTOR2_INV {str(m2.get('invert', False)).lower()}",
        "",
    ])

    m3 = pins.get("motor3", {})
    m4 = pins.get("motor4", {})
    lines.extend([
        f"#define MOTOR3_PWM {m3.get('pwm', -1)}",
        f"#define MOTOR3_IN_A {m3.get('in_a', -1)}",
        f"#define MOTOR3_IN_B {m3.get('in_b', -1)}",
        f"#define MOTOR3_INV {str(m3.get('invert', False)).lower()}",
        "",
        f"#define MOTOR4_PWM {m4.get('pwm', -1)}",
        f"#define MOTOR4_IN_A {m4.get('in_a', -1)}",
        f"#define MOTOR4_IN_B {m4.get('in_b', -1)}",
        f"#define MOTOR4_INV {str(m4.get('invert', False)).lower()}",
        "",
    ])

    # Encoder pins
    e1 = pins.get("encoder1", {})
    e2 = pins.get("encoder2", {})
    lines.extend([
        "// --- Encoder Pins ---",
        f"#define MOTOR1_ENCODER_A {e1.get('pin_a', -1)}",
        f"#define MOTOR1_ENCODER_B {e1.get('pin_b', -1)}",
        f"#define MOTOR1_ENCODER_INV {str(e1.get('invert', False)).lower()}",
        "",
        f"#define MOTOR2_ENCODER_A {e2.get('pin_a', -1)}",
        f"#define MOTOR2_ENCODER_B {e2.get('pin_b', -1)}",
        f"#define MOTOR2_ENCODER_INV {str(e2.get('invert', False)).lower()}",
        "",
    ])

    e3 = pins.get("encoder3", {})
    e4 = pins.get("encoder4", {})
    lines.extend([
        f"#define MOTOR3_ENCODER_A {e3.get('pin_a', -1)}",
        f"#define MOTOR3_ENCODER_B {e3.get('pin_b', -1)}",
        f"#define MOTOR3_ENCODER_INV {str(e3.get('invert', False)).lower()}",
        "",
        f"#define MOTOR4_ENCODER_A {e4.get('pin_a', -1)}",
        f"#define MOTOR4_ENCODER_B {e4.get('pin_b', -1)}",
        f"#define MOTOR4_ENCODER_INV {str(e4.get('invert', False)).lower()}",
        "",
    ])

    # I2C bus and board bring-up.
    #
    # These used to reach the firmware only through a BOARD_INIT macro -- a
    # block of C statements pasted into the header and expanded inside setup().
    # A board was then a build. initBoard() reads them as data instead
    # (firmware/common/lib/board_init), so the same binary serves any pinout,
    # and SDA_PIN / SCL_PIN are finally honoured on their own.
    i2c = pins.get("i2c", {})
    if "sda" in i2c and "scl" in i2c:
        lines.extend([
            "// --- I2C Bus Pins (applied by initBoard(), env keys i2c_sda / i2c_scl) ---",
            f"#define SDA_PIN {i2c['sda']}",
            f"#define SCL_PIN {i2c['scl']}",
            f"#define I2C_CLOCK {int(i2c.get('clock', 400000))}",
            "",
        ])

    # Pins the board must drive at boot (enable lines, power rails, resets),
    # as "pin=level,pin=level". `late` is driven at the end of setup(), for a
    # motor-driver enable that must stay low until the PWM pins are settled.
    def _out_pins(key):
        spec = pins.get(key)
        if not spec:
            return None
        if isinstance(spec, str):
            return spec
        parts = []
        for item in spec:
            if isinstance(item, dict):
                parts.append(f"{item['pin']}={1 if item.get('level', 1) else 0}")
            else:
                parts.append(f"{item}=1")
        return ",".join(parts)

    boot_pins = _out_pins("gpio_out")
    late_pins = _out_pins("gpio_out_late")
    if boot_pins or late_pins:
        lines.append("// --- Boot-time output pins (env keys gpio_out / gpio_out_late) ---")
        if boot_pins:
            lines.append(f'#define BOARD_OUT_PINS "{boot_pins}"')
        if late_pins:
            lines.append(f'#define BOARD_OUT_PINS_LATE "{late_pins}"')
        lines.append("")

    # Auxiliary Pins (LED, Battery, Sonar)
    led = pins.get("led")
    led_pin = led.get("pin") if isinstance(led, dict) else led
    if led_pin is not None and int(led_pin) >= 0:
        lines.append(f"#define LED_PIN {led_pin}")

    # The ADC battery monitor: the pin, the divider it reads through and the
    # pack it describes. These are the compile-time fallbacks; mcu_env.py writes
    # the same facts as bat_pin / bat_r1 / bat_r2 / bat_min / bat_max / bat_cap
    # so a prebuilt image reads the right pack. Before this, r1/r2 were saved
    # by Config Studio and read by nothing, and a battery pin made the build
    # fail on an undefined BATTERY_ADJUST.
    bat = pins.get("battery", {})
    bat_pin = bat.get("pin") if isinstance(bat, dict) else bat
    if bat_pin is not None and int(bat_pin) >= 0:
        lines.append(f"#define BATTERY_PIN {bat_pin}")
    if isinstance(bat, dict):
        for macro, key in (("BATTERY_R1", "r1"), ("BATTERY_R2", "r2"),
                           ("BATTERY_MIN", "min_v"), ("BATTERY_MAX", "max_v"),
                           ("BATTERY_CAP", "capacity_ah")):
            if bat.get(key) is not None:
                lines.append(f"#define {macro} {float(bat[key])}")

    sonar = pins.get("sonar", {})
    if isinstance(sonar, dict):
        trig = sonar.get("trigger", -1)
        echo = sonar.get("echo", -1)
        if trig is not None and echo is not None and int(trig) >= 0 and int(echo) >= 0:
            lines.append(f"#define TRIG_PIN {trig}")
            lines.append(f"#define ECHO_PIN {echo}")
    lines.append("")

    # Sensors & Bare MCU Fallback
    lines.append("// --- Sensor Definitions & Fallback Directives ---")
    # The agent address, resolved for every config. A serial robot still gets
    # these because the same binary can be switched to udp4 by writing
    # transport= into the env, and it needs somewhere to aim when that happens.
    # `transport` is assigned further down, in the serial/micro-ROS block; read it
    # from the config here rather than reordering that block around this one.
    transport_default = tgt.get("transport", "serial")
    _micro = secrets.get("micro_ros", {}) or {}
    _agent_ip = tgt.get("agent_ip") or _micro.get("agent_ip") or "192.168.1.10"
    agent_port_default = tgt.get("udp_port") or _micro.get("agent_port", 8888)
    try:
        agent_octets = [int(x.strip()) for x in str(_agent_ip).split(".")]
        if len(agent_octets) != 4:
            raise ValueError()
    except Exception:
        agent_octets = [192, 168, 1, 10]

    # The model names are emitted as STRINGS as well as macros. The macros still
    # drive the legacy compile-time `#define IMU <class>` path; the strings are
    # what the runtime factory falls back to when the env partition says nothing,
    # so a board with a blank env behaves exactly as its config describes while
    # the same binary can be re-pointed at different silicon by writing `imu=`
    # into the env. Lowercase, matching the YAML spelling, so the config, the env
    # and the firmware all name a sensor identically.
    imu_name = "fake" if sensors.get("use_fake_imu", False) else str(sensors.get("imu", "fake")).lower()
    mag_name = "fake" if sensors.get("use_fake_mag", False) else str(sensors.get("mag", "fake")).lower()
    if imu_name in ("none", ""):
        imu_name = "fake"
    if mag_name in ("none", ""):
        mag_name = "fake"
    lines.extend([
        "// --- Runtime transport selection (env key `transport`) ---",
        "// The transport is installed at boot by initUrosTransport(); this is only",
        "// what a board with no env block falls back to.",
        f'#define TRANSPORT_DEFAULT "{transport_default}"',
        # Emitted for every config, not only the udp4 ones. uros_transport.cpp
        # compiles its UDP branch on any ESP32 -- that is the point, one binary
        # for both transports -- so it references these even in a build whose
        # config says serial. Guarding them behind the transport would put the
        # serial ESP32 build back to failing on an undefined macro.
        f'#ifndef AGENT_IP_DEFAULT',
        f'#define AGENT_IP_DEFAULT IPAddress({agent_octets[0]}, {agent_octets[1]}, '
        f'{agent_octets[2]}, {agent_octets[3]})',
        f'#endif',
        f'#ifndef AGENT_PORT_DEFAULT',
        f'#define AGENT_PORT_DEFAULT {agent_port_default}',
        f'#endif',
        "// --- Runtime sensor selection (env keys `imu` and `mag`) ---",
        "#define USE_RUNTIME_SENSORS",
        f'#define IMU_DEFAULT_NAME "{imu_name}"',
        f'#define MAG_DEFAULT_NAME "{mag_name}"',
    ])

    if sensors.get("use_fake_imu", False):
        lines.append("#define USE_FAKE_IMU  // Prevent I2C NACK loop stall on bare bench")
    else:
        imu_model = sensors.get("imu", "NONE").upper()
        if imu_model != "NONE":
            lines.append(f"#define USE_{imu_model}_IMU")

    if sensors.get("use_fake_mag", False):
        lines.append("#define USE_FAKE_MAG  // Prevent compass NACK stall on bare bench")
    else:
        mag_model = sensors.get("mag", "NONE").upper()
        if mag_model != "NONE":
            lines.append(f"#define USE_{mag_model}_MAG")

    if sensors.get("current", "").upper() == "INA219":
        lines.append("#define USE_INA219")

    if str(sensors.get("env", "")).upper() in ("BMP280", "BME280") or sensors.get("bmp280", False):
        lines.append("#define USE_BMP280")
        if sensors.get("use_fake_env", False):
            lines.append("#define USE_FAKE_ENV  // Simulate barometer readings on bench")

    if sensors.get("use_fake_wheel", False):
        lines.append("#define USE_FAKE_WHEEL  // Simulate encoder ticks on bench")

    # The MCU-side fake scan and the host-side one are separate things. Bringup
    # launches scripts/fake_laser_node.py off sensors.use_fake_ld19, whereas this
    # define makes the firmware raycast the room itself and stream it as raw_scan
    # over the micro-ROS link. That stream does not fit alongside the 50 Hz
    # control loop on a 921600-baud serial ESP32 (README, "Wi-Fi Transport &
    # LiDAR UDP in Fake Mode"), so an explicit lidar.use_fake_ld19 overrides the
    # sensors flag and leaves the synthesis on the host.
    if isinstance(lidar, dict) and "use_fake_ld19" in lidar:
        mcu_fake_ld19 = bool(lidar.get("use_fake_ld19"))
    else:
        mcu_fake_ld19 = bool(sensors.get("use_fake_ld19", False))
    if mcu_fake_ld19:
        lines.append("#define USE_FAKE_LD19   // Raycast virtual 10x6m room on bench")
        # The emulator raycasts from where the config says the LiDAR sits, so
        # /scan and the TF tree agree (a 12 cm mismatch smeared the map by
        # 0.196 m median once). The env key lidar_x overrides at run time.
        laser = gen_robot_description.effective_geometry(params)["laser"]
        lines.append(f"#define FAKE_LIDAR_OFFSET_X {float(laser['x'])}f")
    lines.append("")

    # LiDAR settings
    if lidar:
        # LIDAR_RXD is only meaningful when the MCU has a UART role in the scan
        # path, and which modes those are depends on who generates the scan:
        #   topic          -> none; the scan travels as raw_scan over micro-ROS
        #   udp  + fake    -> none; the synthetic scan goes straight out on UDP
        #   udp  + real    -> the MCU taps the physical LiDAR here and forwards
        #                     it to the UDP server (the real-robot exception:
        #                     everywhere else a real LiDAR plugs into the robot
        #                     PC by USB and the MCU never sees it)
        #   serial + fake  -> the MCU emits an emulated LD19 out this pin, which
        #                     on the gendrv bench feeds a USB-serial bridge
        # Zeroing it for every UDP mode, as this once did, silently disabled the
        # real esp32-wifi wiring.
        comm_mode = lidar.get("comm_mode")
        if comm_mode == "topic" or (comm_mode in ("udp", "udp_server") and mcu_fake_ld19):
            rx_pin = -1
        else:
            rx_pin = lidar.get("rx_pin", -1)
        lines.extend([
            "// --- LiDAR Hardware Configuration ---",
            f"#define LIDAR_RXD {rx_pin}",
            f"#define LIDAR_BAUDRATE {lidar.get('baudrate', 230400)}",
            f"#define LIDAR_SERIAL 1",
            "",
        ])

    # Communication & Transport
    transport = tgt.get("transport", "serial")
    lines.extend([
        "// --- Serial & micro-ROS Communication ---",
        f"#define BAUDRATE {baudrate}",
        f'#define NODE_NAME "{params.get("robot", {}).get("name", "linorobot2")}_base_node"',
        "",
    ])

    # Wi-Fi Station Configuration (for micro-ROS UDP transport, Syslog telemetry, and ArduinoOTA)
    has_wifi = bool(
        tgt.get("use_wifi", False)
        or (isinstance(tgt.get("wifi"), dict) and tgt.get("wifi").get("enabled", True))
        or (isinstance(tgt.get("wifi"), bool) and tgt.get("wifi"))
        or transport in ("udp4", "udp", "wifi")
        or tgt.get("mcu") in ("picow", "pico2w")
    )
    # WIFI_DEFAULT_ENABLED says what this config WANTS; the Wi-Fi code itself is
    # compiled into every ESP32 build regardless, because one binary has to be
    # able to serve both the serial and the udp4 robot. wifiWanted() reads this
    # as the default and the env key `wifi` overrides it.
    lines.extend([
        f"#define WIFI_DEFAULT_ENABLED {1 if has_wifi else 0}",
        "",
    ])

    # An ESP32 always gets the Wi-Fi block, whether or not its config asks for
    # Wi-Fi. WIFI_AP_LIST is what gates the whole wifi module, so a serial-only
    # config used to compile a binary with no radio support at all -- and that
    # binary could then never be switched to udp4 by writing to the env.
    mcu = str(tgt.get("mcu") or tgt.get("pio_env") or controller_name or "").lower()
    wifi_capable = mcu.startswith("esp32") or mcu in ("picow", "pico2w") or \
        controller_name in ("gendrv", "gendrv_real", "esp32", "esp32_wifi", "esp32s3")
    if has_wifi or wifi_capable:
        # Load credentials exclusively from gitignored secrets.yaml (or secrets.yaml.example template)
        wifi_secrets = secrets.get("wifi", {})
        ssid = wifi_secrets.get("ssid", "")
        password = wifi_secrets.get("password", "")
        if not ssid:
            example_path = cockpit_paths.SECRETS_EXAMPLE_PATH
            example_secrets = load_yaml(example_path)
            ssid = example_secrets.get("wifi", {}).get("ssid", "YOUR_WIFI_SSID")
            password = example_secrets.get("wifi", {}).get("password", "YOUR_WIFI_PASSWORD")

        # Site-specific values live in the `env` flash partition, not in the
        # application image (firmware/common/lib/mcu_env, scripts/mcu_env.py).
        # The credentials and the three addresses below describe the LAN the
        # robot is standing on, so an image with them compiled in works on one
        # site and nowhere else -- which is exactly what makes a prebuilt image
        # worth shipping or worthless. envGet() reads them at run time and every
        # macro here is only the fallback for a board with no env block yet.
        lines.extend([
            "// --- Wi-Fi Station Configuration ---",
            "// Credentials come from the `env` flash partition at run time; see",
            "// scripts/mcu_env.py. The compiled list below is only a fallback,",
            "// and it is empty when the header was generated with",
            "// --no-embed-secrets (which is how firmware/prebuilt is built).",
            '#include "mcu_env.h"',
            "#define USE_MCU_ENV",
            "#ifndef USE_WIFI",
            "#define USE_WIFI",
            "#endif",
            "#ifndef USE_STAY_CONNECTED",
            "#define USE_STAY_CONNECTED",
            "#endif",
            ("#define WIFI_AP_LIST {{NULL, NULL}}"
             if no_embed_secrets else
             f'#define WIFI_AP_LIST {{{{"{ssid}", "{password}"}}, {{NULL, NULL}}}}'),
            "#define WIFI_MONITOR 2  // Send RSSI to syslog every 2 min",
            "",
        ])


    # micro-ROS UDP Transport (only when transport is UDP / Wi-Fi)
    if transport in ("udp4", "udp", "wifi"):
        agent_ip_str = tgt.get("agent_ip") or secrets.get("micro_ros", {}).get("agent_ip") or "192.168.1.10"
        agent_port = tgt.get("udp_port") or secrets.get("micro_ros", {}).get("agent_port", 8888)
        try:
            octets = [int(x.strip()) for x in str(agent_ip_str).split(".")]
            if len(octets) != 4:
                raise ValueError()
        except Exception:
            octets = [192, 168, 1, 10]

        lines.extend([
            "// --- micro-ROS Wi-Fi UDP Transport ---",
            "#ifndef MICRO_ROS_TRANSPORT_ARDUINO_WIFI",
            "#define MICRO_ROS_TRANSPORT_ARDUINO_WIFI",
            "#endif",
            "#define USE_WIFI_TRANSPORT",
            '#define AGENT_IP   envIP("agent_ip", AGENT_IP_DEFAULT)',
            '#define AGENT_PORT envU16("agent_port", AGENT_PORT_DEFAULT)',
            "",
        ])

    # Syslog Remote UDP Telemetry
    telemetry = tgt.get("telemetry", {})
    use_syslog = telemetry.get("syslog", True) if has_wifi else bool(tgt.get("use_syslog", False))
    if use_syslog:
        syslog_srv_str = telemetry.get("syslog_server") or secrets.get("telemetry", {}).get("syslog_server") or tgt.get("agent_ip") or secrets.get("micro_ros", {}).get("agent_ip") or "192.168.1.10"
        # The Cockpit's syslog sink is on 5140 unconditionally (see
        # mcu_env.resolve_syslog_port): a private sink for one robot does not
        # need the privileged well-known port, and a fixed port cannot disagree
        # between two machines the way the old 514-with-fallback could.
        # Previously: unprivileged users cannot bind UDP 514, so the sink was on
        # 5140 whenever it runs rootless. The rule now lives in mcu_env.py so
        # the compiled-in default and the env block cannot disagree about it --
        # they did, and a fresh env silently undid this correction.
        syslog_port = mcu_env.resolve_syslog_port(
            telemetry.get("syslog_port") or secrets.get("telemetry", {}).get("syslog_port", 5140))

        hostname = telemetry.get("hostname", controller_name)
        try:
            sys_octets = [int(x.strip()) for x in str(syslog_srv_str).split(".")]
            if len(sys_octets) != 4:
                raise ValueError()
        except Exception:
            sys_octets = [192, 168, 1, 10]

        lines.extend([
            "// --- Syslog Remote UDP Telemetry ---",
            "#define USE_SYSLOG",
            f"#define SYSLOG_SERVER IPAddress({sys_octets[0]}, {sys_octets[1]}, {sys_octets[2]}, {sys_octets[3]})",
            f"#define SYSLOG_PORT {syslog_port}",
            "// initSyslog() overrides both from the env partition at run time.",
            "// They stay plain constants here because syslogv is a global: it is",
            "// constructed during static initialisation, before the flash",
            "// partition API is usable.",
            f'#define DEVICE_HOSTNAME "{hostname}"',
            '#define APP_NAME "hardware"',
            "",
        ])

    # ArduinoOTA Wireless Firmware Flashing
    use_ota = telemetry.get("ota", True) if has_wifi else bool(tgt.get("use_ota", False))
    if use_ota:
        lines.extend([
            "// --- ArduinoOTA Wireless Firmware Flashing ---",
            "#define USE_ARDUINO_OTA",
            "",
        ])


    # LiDAR UDP Streaming
    if lidar and lidar.get("comm_mode") in ("udp", "udp_server"):
        lidar_srv_str = lidar.get("server_ip") or tgt.get("agent_ip") or secrets.get("micro_ros", {}).get("agent_ip") or "192.168.1.10"
        lidar_port = lidar.get("udp_port", 8889)
        try:
            srv_octets = [int(x.strip()) for x in str(lidar_srv_str).split(".")]
            if len(srv_octets) != 4:
                raise ValueError()
        except Exception:
            srv_octets = [192, 168, 1, 10]

        lines.extend([
            "// --- LiDAR UDP Streaming ---",
            "#define USE_LIDAR_UDP",
            f"#define LIDAR_SERVER_DEFAULT IPAddress({srv_octets[0]}, {srv_octets[1]}, {srv_octets[2]}, {srv_octets[3]})",
            f"#define LIDAR_PORT_DEFAULT {lidar_port}",
            '#define LIDAR_SERVER envIP("lidar_ip", LIDAR_SERVER_DEFAULT)',
            '#define LIDAR_PORT   envU16("lidar_port", LIDAR_PORT_DEFAULT)',
            "",
        ])

    # FreeRTOS Dual-Core Architecture (ESP32 / ESP32-S3) -- ON unless refused.
    #
    # controlTask pins moveBase() to core 0 at priority configMAX_PRIORITIES - 2
    # under portENTER_CRITICAL(&controlMux), so the micro-ROS core only reads
    # sensors and publishes. Measured on the GenDrv (QMI8658 + AK09918 + INA219 +
    # BMP280, 1.5 Mbaud serial, radio off): 43 Hz on one core, 49 Hz on two.
    #
    # It was switched off by default once, because that spinlock disables
    # interrupts on core 0 -- the Wi-Fi driver's and lwIP's core -- and a Wi-Fi
    # robot inherited a 50 Hz interrupt blackout on its radio. That hazard is now
    # handled where it belongs: the firmware ignores dual_core whenever the radio
    # is wanted (wifi=1 or transport=udp4) and says so at boot. So the default
    # is the serial robot's best setting, and `use_dual_core: false` is the way
    # to refuse it. Boards without a second core never see the macro.
    if tgt.get("mcu", "").lower() in ("esp32", "esp32s3") and tgt.get("use_dual_core", True):
        lines.extend([
            "// --- FreeRTOS Dual-Core Architecture ---",
            "#define USE_DUAL_CORE  // moveBase() on core 0; ignored at boot when the radio is on",
            "",
        ])

    # Stamped cmd_vel Support (geometry_msgs/msg/TwistStamped)
    #
    # `auto` means "whatever the ROS 2 distro this image is built for drives
    # /cmd_vel with", and it has to be resolved HERE because the subscriber's
    # type is compiled in. nav2 1.4 (kilted) flipped TwistPublisher to
    # TwistStamped by default, so a lyrical stack publishes TwistStamped while
    # a jazzy one publishes Twist.
    #
    # Getting this wrong is silent in the worst way: the type hashes differ, so
    # nav2 publishes into a topic nothing is subscribed to with that type, and
    # the robot simply never moves. Nav2 reports "Failed to make progress" and
    # every recovery behaviour times out the same way -- nothing anywhere names
    # cmd_vel. linorobot2's console branch hit exactly this (6e27659) and solved
    # it from the other end, forcing nav2 back to unstamped, because its firmware
    # has no TwistStamped subscriber. This firmware does, so the wire contract is
    # chosen once, from the distro, and one_click_pipeline.py passes the same
    # --distro to this script and to the launcher.
    #
    # Before this, `auto` fell through every truth test and meant "off", so a
    # lyrical image always listened for plain Twist.
    stamped_cmd = tgt.get("stamped_cmd_vel", kine.get("stamped_cmd_vel", "auto"))
    if str(stamped_cmd).lower() in ("auto", ""):
        stamped_cmd = distro_stamps_cmd_vel(distro)
    if stamped_cmd is True or str(stamped_cmd).lower() in ("true", "1", "yes"):
        lines.extend([
            "// --- Stamped cmd_vel Support (geometry_msgs/msg/TwistStamped) ---",
            "#define USE_STAMPED_CMD_VEL  // Enable geometry_msgs/msg/TwistStamped subscription",
            "",
        ])

    lines.extend([
        "#endif // LINO_BASE_CONFIG_H",
        ""
    ])

    return "\n".join(lines)


def bare_mcu_params(mcu: str) -> dict:
    """A robot-shaped dict describing nothing but the silicon.

    Every value here is either an MCU fact or a deliberate "unset": pins are -1
    (the firmware then touches no GPIO), sensors are absent (the I2C probe finds
    whatever is really there), and the kinematics are placeholders the env
    overwrites. The generated header is the FALLBACK layer -- what a board with
    a blank or corrupt env falls back to -- so the only correct content for a
    release image is "nothing in particular".
    """
    mcu = str(mcu).strip().lower()
    known = ("esp32", "esp32s3", "pico", "pico2", "picow", "pico2w")
    if mcu not in known:
        print(f"Error: unknown --mcu '{mcu}'. Known: {', '.join(known)}", file=sys.stderr)
        sys.exit(2)
    unset_motor = {"pwm": -1, "in_a": -1, "in_b": -1, "invert": False}
    unset_enc = {"pin_a": -1, "pin_b": -1, "invert": False}
    return {
        "robot": {"name": mcu, "description": f"{mcu} release image (no robot)"},
        "base_controller": {
            "name": mcu,
            "mcu": mcu,
            "description": f"{mcu}, one image per silicon; every robot fact is an env key",
            "driver_type": "GENERIC_2_IN",
            # Serial is the transport a board can always fall back to; udp4 is
            # selected by the env like everything else.
            "transport": "serial",
            "baudrate": 921600,
            "sensors": {},
            "pins": {
                "motor1": dict(unset_motor), "motor2": dict(unset_motor),
                "motor3": dict(unset_motor), "motor4": dict(unset_motor),
                "encoder1": dict(unset_enc), "encoder2": dict(unset_enc),
                "encoder3": dict(unset_enc), "encoder4": dict(unset_enc),
                "i2c": {"sda": -1, "scl": -1},
                "led": -1,
            },
        },
        "kinematics": {
            "base_type": "2wd",
            "wheel_diameter": 0.1,
            "lr_wheels_distance": 0.2,
            "max_rpm": 140,
            "max_rpm_ratio": 0.85,
            "counts_per_rev": 4000,
            "pwm_bits": 10,
            "pwm_frequency": 20000,
            "motor_operating_voltage": 12,
            "motor_power_max_voltage": 12,
            "pid": {"kp": 1.0, "ki": 0.5, "kd": 0.1},
        },
    }


def main():
    parser = argparse.ArgumentParser(description="Generate Linorobot2 C++ base configuration header")
    parser.add_argument("--params", default=DEFAULT_PARAMS, help="Path to <config dir>/<robot>_config.yaml")
    parser.add_argument("--mcu", default=None, metavar="MCU",
                        help="Generate a header for a BARE MCU instead of for a robot "
                             "(esp32, esp32s3, pico, pico2). This is how the release "
                             "images are built: one image per silicon, carrying no "
                             "robot's pins, sensors, baud rate or kinematics -- all of "
                             "which reach the board through the env partition at flash "
                             "time (scripts/mcu_env.py). A reference design is a robot "
                             "someone can own, not a build input.")
    parser.add_argument("--secrets", default=DEFAULT_SECRETS, help="Path to secrets.yaml")
    parser.add_argument("--no-pin-check", action="store_true",
                    help="generate even when the pin catalogue reports an error")
    parser.add_argument("--no-embed-secrets", action="store_true",
                        help="Omit the Wi-Fi credentials from the header entirely. The "
                             "firmware then reads them only from the `env` flash "
                             "partition (scripts/mcu_env.py). This is how the images in "
                             "firmware/prebuilt are built: an image with an SSID baked "
                             "in works on one LAN, so it could not be shipped.")
    parser.add_argument("--controller", "--target", dest="controller", default=None,
                        help="Base controller / PlatformIO env (e.g. gendrv, pico2). Defaults to base_controller.name")
    parser.add_argument("--out", default=DEFAULT_OUT, help="Path to output header file")
    parser.add_argument("--distro", default=os.environ.get("ROS_DISTRO", ""),
                        help="ROS 2 distro this image is built for. Only resolves "
                             "`stamped_cmd_vel: auto`, which is a build-time choice: "
                             "kilted and newer drive /cmd_vel as TwistStamped, jazzy "
                             "and older as Twist.")
    args = parser.parse_args()

    if args.mcu:
        params = bare_mcu_params(args.mcu)
        secrets = {}
    else:
        params = load_yaml(args.params)
        secrets = load_yaml(args.secrets)

    declared = (params.get("base_controller") or {}).get("name")
    controller_name = args.controller or declared or "pico2"
    check_pico_family_transport(params, controller_name)
    check_transport_env_pairing(params, controller_name)
    # Pins against the MCU's catalogue, before anything is written for them.
    # An error is a config that cannot work on this silicon (the GenDrv once
    # drove its flash bus for an afternoon); a warning is for a human.
    errors = 0
    for level, msg in pin_catalog.check_config(params):
        print(f"[pins] {level}: {msg}", file=sys.stderr)
        errors += level == "error"
    for msg in config_warnings(params):
        print(f"[config] warn: {msg}", file=sys.stderr)
    if errors and not args.no_pin_check:
        print(f"Error: {errors} pin error(s); fix the config or pass --no-pin-check.", file=sys.stderr)
        sys.exit(1)

    header_content = generate_header(params, secrets, controller_name, args.no_embed_secrets, args.distro)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        f.write(header_content)

    print(f"✅ Generated firmware header for base controller [{controller_name}]: {args.out}")


if __name__ == "__main__":
    main()
