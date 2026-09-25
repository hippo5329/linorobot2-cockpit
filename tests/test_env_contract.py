"""Every env key one side writes, the other side reads.

The class of bug this guards against: a key spelled one way by the writer and
another by (or never by) the reader, which fails silently -- the value simply
never arrives. Found by hand on 2026-09-19: pwm_min/pwm_max read and never
written; lidar_ip/lidar_port written and read only through a header macro;
bat r1/r2 saved by Config Studio and read by nothing.
"""
import glob
import os
import re

import mcu_env

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF = os.path.join(REPO_ROOT, "config", "reference")
SECRETS_EXAMPLE = os.path.join(REPO_ROOT, "config", "secrets.yaml.example")

# Keys the firmware reads that no config produces: bench tools set them with
# `mcu_env.py set`, the flasher writes them, or a tool application
# (adc_calibrate: dac_pin) takes them from the operator.
BENCH_ONLY = {"diag_tx", "diag_baud", "app", "dac_pin", "boot_serial_wait",
              "banner_hold"}

# A config exercising every optional key the writer knows, so the reference
# set need not carry each one for it to count as written.
MAXIMAL_CONFIG = {
    "robot": {"name": "maximal"},
    "kinematics": {"base_type": "2wd", "wheel_diameter": 0.1, "lr_wheels_distance": 0.3,
                   "max_rpm": 100, "counts_per_rev": 100, "pwm_bits": 8, "pwm_frequency": 1000,
                   # The wheelbase and the skid-steer scrub. Neither belongs in a
                   # reference config -- fr is 0 on a 2wd and the scrub is 1.0
                   # until somebody measures their own chassis -- so this is the
                   # only place the contract can see them written.
                   "fr_wheels_distance": 0.18, "angular_scale": 1.25,
                   "pid": {"kp": 1, "ki": 0, "kd": 0}},
    "base_controller": {
        "name": "esp32", "mcu": "esp32", "baudrate": 921600, "transport": "serial", "console": "uart0",
        "qos": "reliable", "use_dual_core": True, "boot_delay": 2,
        "telemetry": {"ota_port": 3232},
        "sensors": {"imu": "auto", "mag": "auto", "current": "INA219", "env": "BMP280",
                    "use_sim_ld19": True},
        "lidar": {"model": "ld19", "comm_mode": "serial", "rx_pin": 4, "baudrate": 230400},
        "pins": {"i2c": {"sda": 1, "scl": 2}, "led": 3,
                 "gpio_out": [{"pin": 5, "level": 1}], "gpio_out_late": "6=1",
                 "battery": {"pin": 7, "r1": 1000, "r2": 100, "min_v": 9, "max_v": 12.6, "capacity_ah": 2},
                 "motor1": {"pwm": 8, "in_a": 9, "in_b": 10, "invert": False},
                 "encoder1": {"pin_a": 11, "pin_b": 12, "invert": True},
                 "dac": 25,
                 "imu": {"int": 41}, "sonar": {"trigger": 13, "echo": 14}},
        # Optional blocks exist so that every key the firmware reads has a
        # writer SOMEWHERE. A key only a hand-edited YAML can produce is a key
        # nobody will ever set.
        "bmp280_addr": "0x76",
        "topic_prefix": "robot1",
        "wdt_timeout": 8,
        "rpm_track_voltage": True,
        "stall_detect": {"enabled": True, "ms": 1200, "rpm_floor": 6},
        "imu_tuning": {"accel_cov": 0.5, "gyro_cov": 0.5, "ori_cov": 0.5,
                       "mag_cov": 0.5, "env_cov": [3, 0.25, 9e-4],
                       "pose_cov": [1, 2, 3, 4, 5, 6], "twist_cov": 0.001,
                       "mag_bias": [1.5, -2.25, 0.75]},
        "simulation": {"map_width": 10.0, "map_height": 6.0, "wall_obstacle": True,
                       "wall_x1": 2.0, "wall_y1": -1.5, "wall_x2": 2.0,
                       "wall_y2": 1.5, "robot_mass": 3.5, "wheel_noise_rpm": 1.0,
                       # The simulated drivetrain's losses, swept from the env so
                       # a sensitivity test costs a 4 KB write, not a build.
                       "gear_efficiency": 0.75, "gear_drag_rpm": 12.0,
                       "battery_sag": 0.25, "battery_sag_tau_ms": 400.0,
                       "driver_drop": 0.03, "driver_resistance": 0.10,
                       "motor_stall_amps": 2.5, "driver_current_limit": 2.0,
                       # How the IMU is mounted. Zero is a level part and belongs
                       # in every reference config by omission, so -- like
                       # fr_wheels_distance and angular_scale above -- this is the
                       # only place the contract can see these two written.
                       "imu_mount_roll_deg": 2.0,
                       "imu_mount_pitch_deg": -1.5},
    },
}


def _firmware_sources():
    text = ""
    for pattern in ("firmware/src/**/*.cpp", "firmware/src/**/*.h",
                    "firmware/common/lib/**/*.cpp", "firmware/common/lib/**/*.h",
                    "scripts/gen_firmware_header.py"):
        for f in glob.glob(os.path.join(REPO_ROOT, pattern), recursive=True):
            if ".pio" in f:
                continue
            with open(f, errors="ignore") as fh:
                text += fh.read()
    return text


def _norm(key):
    return re.sub(r"^m(?:\d|%d|%u)_", "mN_", key)


def _written_keys():
    keys = set()
    for path in glob.glob(os.path.join(REF, "*_config.yaml")):
        env = mcu_env.env_from_config(path, SECRETS_EXAMPLE, "192.0.2.1")
        keys |= {_norm(k) for k in env}
    keys |= {_norm(k) for k in mcu_env.hardware_env(MAXIMAL_CONFIG)}
    return keys


def _read_keys(src):
    # envGet("key"), envU16("key", ..), envFlag("key", ..), envPin(i, "suffix", ..)
    # FloatVec reads a diagonal covariance; it is a read like any other, and
    # leaving it out of this pattern made four covariance keys look dead.
    keys = set(re.findall(r'\benv(?:Get|U16|U32|IP|FloatVec|Float|Flag|FlagMain|Int)\(\s*"([a-z0-9_]+)"', src))
    keys |= {"mN_" + s for s in re.findall(r'\benvPin\(\s*\w+,\s*"([a-z_]+)"', src)}
    keys |= {_norm(k) for k in re.findall(r'"(m%d_[a-z_]+)"', src)}
    keys |= {_norm(k) for k in re.findall(r'"(m%u_[a-z_]+)"', src)}
    return keys


def test_every_written_key_is_read_by_the_firmware():
    written = _written_keys()
    read = _read_keys(_firmware_sources())
    dead = sorted(written - read)
    assert not dead, f"mcu_env.py writes keys the firmware never reads: {dead}"


def test_every_key_the_firmware_reads_has_a_writer():
    written = _written_keys() | BENCH_ONLY
    read = _read_keys(_firmware_sources())
    orphans = sorted(read - written)
    assert not orphans, f"the firmware reads keys nothing writes: {orphans}"


def test_the_deploy_endpoint_writes_kinematics_keys_the_firmware_reads():
    """/api/ai/deploy_robot once wrote track_width and wheelbase (read by nothing)."""
    import system_utils
    design = system_utils.generate_custom_robot_specs("a 4wd rover")["design"]
    assert {"wheel_diameter", "lr_wheels_distance", "fr_wheels_distance"} <= set(design)
    assert not {"track_width", "wheelbase", "wheel_diameter_m"} & set(design)


def test_every_run_writes_the_env_block():
    """A test run must never inherit an env block it did not write.

    want_env keyed on the probe calling it stale, and the probe compares against
    what THIS host recorded -- so a board carrying an env from an older config,
    another host, or a --skip-flash run kept it. A GenDrv leg "passed" that way
    with sim_ld19 off, inheriting an older env that had the emulator on.
    """
    src = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert "want_env = bool(board) and not args.skip_flash" in src, \
        "the env write must not be conditional on the probe's staleness verdict"
    assert 'needs_env_write")) and not args.skip_flash' not in src
