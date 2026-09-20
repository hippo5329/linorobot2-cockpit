#!/usr/bin/env python3
"""
mcu_env.py — build, read and edit the MCU environment block.

The environment block is a U-Boot environment, byte for byte:

    offset 0   uint32  CRC32 (little-endian) of everything after it
    offset 4   bytes   "key=value\0key=value\0...\0\0", zero-padded to ENV_SIZE

It lives in its own flash partition (`env`, see firmware/common/partitions_lino.csv)
and NOT in the application image. That separation is the whole point: Wi-Fi
credentials and the address of the machine running micro_ros_agent are properties
of the site, not of the robot, so a firmware image with them compiled in only
ever works on one LAN. Shipping a prebuilt image that a user must rebuild to
enter their own SSID would defeat the purpose of shipping one.

With the split, re-keying a robot never involves a compiler:

    python3 scripts/mcu_env.py build --out env.bin      # from <config dir>/secrets.yaml
    esptool write_flash 0x3ff000 env.bin

and reflashing the application leaves the keys alone, because writing the app at
0x10000 does not touch 0x3ff000.

The env does not only carry the site keys. It carries the whole robot, because
there are only three firmware images -- one per MCU -- and a Waveshare General
Driver board and a bare DevKit run the same esp32 binary. Everything that makes
one of them a particular robot is here:

    site        wifi_ssid  wifi_psk  wifi
                agent_ip   agent_port   transport  baud  node
                syslog_ip  syslog_port
                lidar_ip   lidar_port  lidar_rx  lidar_baud

    runtime     dual_core  i2c_scan  pub_mag  pub_battery  pub_env  best_effort
                fake_ld19  lidar_x   (the MCU-side LiDAR emulator, and where on
                                      the robot it raycasts from: geometry.laser.x)
                ota_port
                diag_tx  diag_baud   (a second UART that prints the loop's
                                      counters once a second; a bench tool,
                                      set with --set, never from the config)

    board       i2c_sda  i2c_scl  i2c_clock
                gpio_out  gpio_out_late  boot_delay
                imu  mag
                battery_pin  bat_r1  bat_r2  bat_min  bat_max  bat_cap
                                     (the ADC battery monitor: pin, divider, pack)

    drivetrain  m<N>_pwm  m<N>_in_a  m<N>_in_b  m<N>_inv
                m<N>_enc_a  m<N>_enc_b  m<N>_cpr  m<N>_enc_inv   (N = 1..4)
                motor_driver  pwm_freq  pwm_bits  pwm_min  pwm_max
                kp  ki  kd  fake_wheel

    kinematics  base  max_rpm  rpm_ratio  wheel_d  lr_dist
                motor_v  power_v

Each key falls back to the macro the image was generated with, so a board with a
blank env still boots as its config describes. Unknown keys are preserved and
ignored, exactly as in U-Boot.

Usage:
    python3 scripts/mcu_env.py build [--params <config dir>/<robot>_config.yaml] [--out env.bin]
    python3 scripts/mcu_env.py print env.bin
    python3 scripts/mcu_env.py set env.bin wifi_ssid=other-ap agent_ip=192.168.1.10
"""
import argparse
import os
import sys
import zlib

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cockpit_paths  # noqa: E402
import gen_firmware_header  # noqa: E402  (counts_per_rev)

# Must match the `env` row of firmware/common/partitions_lino.csv.
ENV_OFFSET = 0x3FF000
ENV_SIZE = 0x1000
CRC_LEN = 4
DATA_LEN = ENV_SIZE - CRC_LEN

# The RP2 boards have no partition table. arduino-pico instead reserves the last
# 4 KB of flash for its EEPROM emulation -- its builder computes
#
#     eeprom_start        = 0x10000000 + flash_size - 4096
#     maximum_sketch_size = flash_size - 4096 - filesystem_size
#
# so that sector sits above both the sketch and the filesystem on every board,
# whatever the flash size and whether or not a filesystem is configured. The env
# block goes there, in the same layout as the ESP32 partition, and is read in
# place by firmware/common/lib/mcu_env (the region is XIP-mapped).
#
# Because it is above maximum_sketch_size it survives an application update:
# `picotool load firmware.uf2` writes only the blocks the UF2 carries, and
# arduino-pico's OTA stages into the filesystem area and copies down over the
# sketch. Neither reaches the last sector. Only a full-chip erase clears it.
RP2_ENV_OFFSETS = {
    "pico":    0x101FF000,   # 2 MB
    "picow":   0x101FF000,
    "pico2":   0x103FF000,   # 4 MB
    "pico2w":  0x103FF000,
}


def env_offset(board: str) -> int:
    """Flash address the env block is written to on `board`.

    `board` is the PlatformIO env name, so the *_lyrical variants resolve to the
    same address as the board they are built for.
    """
    name = (board or "").replace("_lyrical", "")
    return RP2_ENV_OFFSETS.get(name, ENV_OFFSET)


def host_ip() -> str:
    """This machine's address on the LAN the robot will join.

    The agent, the syslog sink and the LiDAR UDP listener all run on whichever
    computer is driving the robot, so its address is the only sensible default
    for all three -- and a hard-coded address is right on exactly one bench.
    Connecting a UDP socket sends no packets; it only asks the routing table
    which local address would be used to reach the outside, which is the address
    an ESP32 on the same LAN has to aim at.
    """
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("10.255.255.255", 1))
        return sock.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        sock.close()


SYSLOG_PORT_DEFAULT = 5140


def resolve_syslog_port(port) -> int:
    """The port the Cockpit's syslog sink listens on. Default 5140, always.

    This used to ask 514 first and fall back to 5140 -- geteuid(), then an actual
    bind probe -- because 514 is privileged and `SyslogManager` cannot take it
    when the Cockpit runs unprivileged, which is every rootless container, every
    container and every `python3 web/backend/main.py` started as a user. The
    adjustment worked, but it made the port a RUNTIME fact: two machines running
    the same config could disagree about it, and a board pointed at 514 with a
    sink on 5140 fails in total silence -- the board keeps sending, the syslog
    tab stays empty, and there is no RSSI to read.

    5140 unconditionally removes the question. This is a private sink for one
    robot, not a host syslog daemon: nothing needs the well-known port, and an
    unprivileged port is the one both root and non-root can always bind. An
    explicit port in the config is still the operator's choice and is used as
    written.
    """
    try:
        return int(port)
    except (TypeError, ValueError):
        return SYSLOG_PORT_DEFAULT


def load_yaml(path):
    try:
        import yaml
    except ImportError:
        sys.exit("mcu_env: pyyaml is required (pip install pyyaml)")
    if not os.path.isfile(path):
        return {}
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


# --------------------------------------------------------------------- encode
def encode(env: dict) -> bytes:
    """Pack {key: value} into the U-Boot on-flash layout."""
    body = bytearray()
    for key in sorted(env):
        value = env[key]
        if value is None or value == "":
            continue                       # U-Boot drops empty variables
        if "=" in key or "\0" in key:
            sys.exit(f"mcu_env: illegal key {key!r}")
        body += f"{key}={value}".encode("utf-8") + b"\0"
    body += b"\0"                          # the empty entry that ends the list
    if len(body) > DATA_LEN:
        sys.exit(f"mcu_env: environment is {len(body)} bytes, partition holds {DATA_LEN}")
    body += b"\xff" * (DATA_LEN - len(body))   # erased-flash padding, as U-Boot leaves it
    crc = zlib.crc32(bytes(body)) & 0xFFFFFFFF
    return crc.to_bytes(4, "little") + bytes(body)


def decode(blob: bytes) -> dict:
    """Unpack a flash image, verifying the CRC32 first."""
    if len(blob) < ENV_SIZE:
        sys.exit(f"mcu_env: image is {len(blob)} bytes, expected {ENV_SIZE}")
    stored = int.from_bytes(blob[:4], "little")
    body = blob[4:ENV_SIZE]
    actual = zlib.crc32(body) & 0xFFFFFFFF
    if stored != actual:
        sys.exit(f"mcu_env: CRC32 mismatch — stored {stored:#010x}, computed {actual:#010x}")
    env = {}
    for entry in body.split(b"\0"):
        if not entry or entry.startswith(b"\xff"):
            break
        text = entry.decode("utf-8", "replace")
        if "=" in text:
            key, _, value = text.partition("=")
            env[key] = value
    return env


# ---------------------------------------------------------------------- build
def env_from_config(params_path: str, secrets_path: str, default_host: str = None) -> dict:
    """The environment a robot config and its secrets.yaml imply.

    secrets.yaml stays the only home for credentials (AGENTS.md §3). This reads
    it; it never writes it, and the generated env.bin is gitignored for the same
    reason the secrets file is.
    """
    params = load_yaml(params_path)
    secrets = load_yaml(secrets_path)
    tgt = params.get("base_controller", {}) or {}

    wifi = secrets.get("wifi", {}) or {}
    ssid = wifi.get("ssid", "")
    psk = wifi.get("password", "")
    if not ssid:
        example = load_yaml(cockpit_paths.SECRETS_EXAMPLE_PATH)
        ssid = (example.get("wifi", {}) or {}).get("ssid", "")
        psk = (example.get("wifi", {}) or {}).get("password", "")
        if ssid:
            print(f"[mcu_env] {os.path.basename(secrets_path)} has no wifi.ssid — "
                  f"falling back to the placeholders in secrets.yaml.example. "
                  f"The robot will not join a real network until you re-run this "
                  f"with real credentials and rewrite the env partition.",
                  file=sys.stderr)

    # All three addresses default to the machine this runs on. They are the agent,
    # the syslog sink and the LiDAR UDP listener, and on every topology in this
    # project those are the same computer -- so one answer serves all three and
    # nobody has to look an address up to flash a prebuilt image.
    micro = secrets.get("micro_ros", {}) or {}
    fallback_host = default_host or host_ip()
    agent_ip = tgt.get("agent_ip") or micro.get("agent_ip") or fallback_host
    agent_port = tgt.get("udp_port") or micro.get("agent_port", 8888)

    telemetry = tgt.get("telemetry", {}) or {}
    syslog_ip = (telemetry.get("syslog_server")
                 or (secrets.get("telemetry", {}) or {}).get("syslog_server")
                 or agent_ip)
    syslog_port = resolve_syslog_port(
        telemetry.get("syslog_port")
        or (secrets.get("telemetry", {}) or {}).get("syslog_port", SYSLOG_PORT_DEFAULT))

    lidar = tgt.get("lidar", {}) or {}
    lidar_ip = lidar.get("server_ip") or agent_ip
    lidar_port = lidar.get("udp_port", 8889)

    env = {
        "wifi_ssid": ssid,
        "wifi_psk": psk,
        "agent_ip": agent_ip,
        "agent_port": agent_port,
        "syslog_ip": syslog_ip,
        "syslog_port": syslog_port,
        "lidar_ip": lidar_ip,
        "lidar_port": lidar_port,
    }
    env.update(hardware_env(params))
    return env


def _bool(value) -> str:
    return "1" if value else "0"


def hardware_env(params: dict) -> dict:
    """The pin matrix, drivetrain and kinematics a robot config implies.

    This is what lets three images serve every board. The firmware falls back to
    the macros it was compiled with, so these keys are not strictly required --
    but a PREBUILT image was compiled for somebody else's robot, and its
    fallbacks are that robot's pins. For a prebuilt image the env is not an
    override, it is the only description of the hardware that is true.
    """
    tgt = params.get("base_controller", {}) or {}
    pins = tgt.get("pins", {}) or {}
    kin = params.get("kinematics", {}) or {}
    sensors = tgt.get("sensors", {}) or {}
    env = {}

    # --- board
    # The micro-ROS serial rate. A released image is built for an MCU, not for a
    # robot, so this cannot be a build setting: the GenDrv needs 1.5 Mbaud to
    # carry four I2C sensors at 50 Hz, a bare DevKit is fine at 921600, and both
    # run the same binary. main.cpp reads it as envU32("baud", BAUDRATE).
    baud = tgt.get("baudrate")
    if baud:
        env["baud"] = int(baud)

    # Whether to pin the control loop to the second core. It was a build macro,
    # which meant a released image decided it for every robot that flashed it --
    # and the critical section it installs disables interrupts on the same core
    # the Wi-Fi driver runs on, so the wrong answer is expensive.
    if tgt.get("use_dual_core") is not None:
        env["dual_core"] = _bool(tgt["use_dual_core"])
    # QoS of the 50 Hz topics. Best effort is the firmware's default (the
    # launch tree's madgwick and EKF subscribe best-effort; measured 50 Hz at
    # 921600 where reliable managed 25); `qos: reliable` is for a consumer
    # that insists on it -- a reliable subscriber never matches a best-effort
    # writer.
    qos = str(tgt.get("qos", "") or "").strip().lower()
    if qos in ("reliable",):
        env["best_effort"] = 0
    elif qos in ("best_effort", "best-effort", "besteffort"):
        env["best_effort"] = 1

    # The ROS node name, so two robots on one image are not both
    # `linorobot_base_node`.
    robot_name = (params.get("robot", {}) or {}).get("name")
    if robot_name:
        env["node"] = f"{robot_name}_base_node"

    # Which sensor topics this robot publishes. A chip answering the I2C probe
    # is not on its own a reason to spend link budget on it: `mag: NONE` in the
    # config means "do not publish /imu/mag", even on a board that carries an
    # AK09918. This is what makes "only the fake IMU, nothing else" expressible
    # on the 921600 boards, where six publishers at 50 Hz do not fit.
    #   auto (or the key left out)  the I2C probe decides
    #   NONE / off / disable        never publish, even though the chip is there
    #   a model name (INA219, ...)  publish even if the probe missed it, so a
    #                               wiring fault reads as a dead topic rather
    #                               than as one that was never configured
    for key, field in (("pub_mag", "mag"), ("pub_battery", "current"), ("pub_env", "env")):
        declared = sensors.get(field)
        if declared is None:
            continue
        value = str(declared).strip().upper()
        if value in ("AUTO", ""):
            continue
        env[key] = 0 if value in ("NONE", "OFF", "DISABLE", "FALSE") else 1

    # `imu: auto` / `mag: auto` mean the same thing one level down: take whatever
    # answered the bus. The probe is what decides, so ask for it explicitly --
    # otherwise a board with fake wheels skips the scan (see all_fake in
    # main.cpp) and "auto" would quietly mean "fake".
    if str(sensors.get("imu", "")).strip().upper() == "AUTO" or \
       str(sensors.get("mag", "")).strip().upper() == "AUTO":
        env["i2c_scan"] = 1

    # LiDAR wiring: which pin the scan leaves on and how fast. -1 means no UART.
    lidar_cfg = tgt.get("lidar", {}) or {}
    # Which sink the synthetic scan leaves by. The firmware reads this as
    # `lidar_comm` and falls back to LIDAR_COMM_DEFAULT, the comm_mode the image
    # was built from -- so this key is what lets ONE esp32 image serve a robot
    # wired for a serial LD19 and one streaming over UDP.
    if lidar_cfg.get("comm_mode"):
        mode = str(lidar_cfg["comm_mode"]).strip().lower()
        env["lidar_comm"] = "udp" if mode in ("udp", "udp_server") else mode
    if lidar_cfg.get("rx_pin") is not None:
        env["lidar_rx"] = int(lidar_cfg["rx_pin"])
    if lidar_cfg.get("baudrate") is not None:
        env["lidar_baud"] = int(lidar_cfg["baudrate"])
    # Whether the board runs the LiDAR emulator at all. It was a build macro
    # only, so a prebuilt image (built from a fake-mode reference) raycast its
    # room and streamed it on every robot that flashed it, real LiDAR or not.
    # An explicit lidar.use_fake_ld19 outranks the sensors flag, as in the
    # header generator.
    if isinstance(lidar_cfg, dict) and "use_fake_ld19" in lidar_cfg:
        env["fake_ld19"] = _bool(lidar_cfg["use_fake_ld19"])
    elif "use_fake_ld19" in sensors:
        env["fake_ld19"] = _bool(sensors["use_fake_ld19"])
    # Where the emulator raycasts from: the LiDAR's place on the robot, the
    # same number the URDF puts the laser frame at.
    laser = gen_firmware_header.gen_robot_description.effective_geometry(params)["laser"]
    env["lidar_x"] = float(laser["x"])

    i2c = pins.get("i2c", {}) or {}
    if "sda" in i2c and "scl" in i2c:
        env["i2c_sda"] = i2c["sda"]
        env["i2c_scl"] = i2c["scl"]
        env["i2c_clock"] = i2c.get("clock", 400000)
    # The status LED. It is the only feedback an assembled robot gives before
    # micro-ROS is up -- boot, agent-waiting, IMU failure and the fake-wall
    # contact are all blink patterns -- so it must travel with the board rather
    # than with whichever robot's header the image was compiled from. -1 means
    # no LED is wired (the Waveshare GenDrv), and the firmware then touches no pin.
    if "led" in pins:
        env["led"] = pins["led"]
    for key in ("gpio_out", "gpio_out_late"):
        spec = pins.get(key)
        if spec:
            if not isinstance(spec, str):
                spec = ",".join(
                    f"{item['pin']}={1 if item.get('level', 1) else 0}"
                    if isinstance(item, dict) else f"{item}=1"
                    for item in spec)
            env[key] = spec
    if tgt.get("boot_delay"):
        env["boot_delay"] = tgt["boot_delay"]
    # The ADC battery monitor. Config Studio saved r1/r2 for years and nothing
    # read them; the firmware had a BATTERY_ADJUST it never defined.
    bat = pins.get("battery")
    if isinstance(bat, dict):
        for key, src in (("battery_pin", "pin"), ("bat_r1", "r1"), ("bat_r2", "r2"),
                         ("bat_min", "min_v"), ("bat_max", "max_v"), ("bat_cap", "capacity_ah")):
            if bat.get(src) is not None:
                env[key] = bat[src]
    elif bat is not None:
        env["battery_pin"] = bat
    # The HC-SR04. These were compile-time only (TRIG_PIN / ECHO_PIN in the
    # generated header) until 2026-09-20, which meant a released image either
    # had a sonar welded into it or could never have one -- and since no
    # reference config carried the pins, every shipped image was the latter.
    # range.cpp reads these now; -1 or absent means "not wired".
    sonar = pins.get("sonar")
    if isinstance(sonar, dict):
        for key, src in (("sonar_trig", "trigger"), ("sonar_echo", "echo")):
            if sonar.get(src) is not None:
                env[key] = int(sonar[src])
    telemetry = tgt.get("telemetry", {}) or {}
    if telemetry.get("ota_port") is not None:
        env["ota_port"] = int(telemetry["ota_port"])

    # --- sensors. Fake wins: a config asking for a fake IMU on a board that
    # also names a QMI8658 wants the simulation, not the chip.
    # `auto` and `NONE` both come out as "fake" on the wire: it is the only name
    # createIMU()/createMAG() are guaranteed to accept, and for `auto` the I2C
    # probe overwrites it a moment later. Writing "none" or "auto" verbatim used
    # to reach the firmware as an unknown driver, which it reported and then
    # fell back to fake anyway -- the same outcome, with a warning that looked
    # like a fault.
    def _driver_name(field, fake_flag):
        if sensors.get(fake_flag):
            return "fake"
        value = str(sensors.get(field, "fake")).strip()
        if value.upper() in ("AUTO", "NONE", "OFF", "DISABLE", ""):
            return "fake"
        return value.lower()

    env["imu"] = _driver_name("imu", "use_fake_imu")
    env["mag"] = _driver_name("mag", "use_fake_mag")

    # --- transport and radio
    env["transport"] = tgt.get("transport", "serial")
    env["wifi"] = _bool((tgt.get("wifi", {}) or {}).get("enabled", False))

    # --- drivetrain
    for n in range(1, 5):
        motor = pins.get(f"motor{n}", {}) or {}
        enc = pins.get(f"encoder{n}", {}) or {}
        for key, src, default in ((f"m{n}_pwm", motor, "pwm"),
                                  (f"m{n}_in_a", motor, "in_a"),
                                  (f"m{n}_in_b", motor, "in_b"),
                                  (f"m{n}_enc_a", enc, "pin_a"),
                                  (f"m{n}_enc_b", enc, "pin_b")):
            if default in src:
                env[key] = src[default]
        if "invert" in motor:
            env[f"m{n}_inv"] = _bool(motor["invert"])
        if "invert" in enc:
            env[f"m{n}_enc_inv"] = _bool(enc["invert"])
        # Only for a motor this robot actually has. A 2wd config that emitted
        # m3_cpr and m4_cpr would be describing wheels that are not there.
        if enc and (kin.get("counts_per_rev") is not None or kin.get("encoder_ppr") is not None):
            env[f"m{n}_cpr"] = gen_firmware_header.counts_per_rev(kin)

    driver = str(tgt.get("driver_type", "GENERIC_2_IN")).upper()
    env["motor_driver"] = {
        "GENERIC_2_IN": "generic2",
        "GENERIC_1_IN": "generic1",
        "BTS7960": "bts7960",
        "ESC": "esc",
    }.get(driver, "generic2")
    env["fake_wheel"] = _bool(sensors.get("use_fake_wheel", False))
    # The rest of the runtime flags that used to be compile-time macros. Each
    # one is a firmware behaviour that a config can now ask for WITHOUT a
    # rebuild -- but only if it reaches the partition, and `fake_env` did not:
    # env.cpp fell back to the image's compiled default forever, so a bench
    # board flashed with a fake-mode image reported a synthetic 25 C / 1013 hPa
    # as a "BMP280" no matter what its config said.
    env["fake_env"] = _bool(sensors.get("use_fake_env", False))
    # The simulated ultrasonic cone. Only ever used when the LiDAR emulator is
    # running (it raycasts from the same room) and no real sonar is wired, so
    # the firmware gates it anyway; this says whether the bench wants it.
    env["fake_sonar"] = _bool(sensors.get("use_fake_sonar", True))
    # Short brake: both half-bridges to the same rail so the windings damp the
    # rotor, instead of coasting. On by default -- that is what the code has
    # always been written to do, though the macro that selected it was emitted
    # by nothing, so every board shipped coasting.
    env["short_brake"] = _bool(tgt.get("short_brake", True))
    # The forward hazard stop. OFF by default: it brakes the robot, and it has
    # never been compiled into a shipped image, so it should be asked for.
    safety = tgt.get("safety_stop")
    if isinstance(safety, dict):
        env["safety_stop"] = _bool(safety.get("enabled", False))
        env["safety_stop_m"] = safety.get("range_m", 0.25)
    else:
        env["safety_stop"] = _bool(safety) if safety is not None else "0"
        env["safety_stop_m"] = 0.25
    # The LiDAR power-gate pin, -1 when the board has none. A pin, like every
    # other pin: the env carries it so one image serves boards that gate their
    # LiDAR's power and boards that do not.
    env["lidar_poweroff"] = int(lidar_cfg.get("poweroff_pin", -1)) if isinstance(lidar_cfg, dict) else -1

    for key, src in (("pwm_freq", "pwm_frequency"), ("pwm_bits", "pwm_bits")):
        if kin.get(src) is not None:
            env[key] = kin[src]
    pid = kin.get("pid", {}) or {}
    for key in ("kp", "ki", "kd"):
        if pid.get(key) is not None:
            env[key] = pid[key]

    # --- kinematics
    for key, src in (("base", "base_type"),
                     ("max_rpm", "max_rpm"),
                     ("rpm_ratio", "max_rpm_ratio"),
                     ("wheel_d", "wheel_diameter"),
                     ("lr_dist", "lr_wheels_distance"),
                     ("motor_v", "motor_operating_voltage"),
                     ("power_v", "motor_power_max_voltage")):
        if kin.get(src) is not None:
            env[key] = kin[src]
    return env


def redact(env: dict) -> dict:
    out = dict(env)
    if out.get("wifi_psk"):
        out["wifi_psk"] = "*" * 8
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("build", help="write an env image from the config and secrets")
    b.add_argument("--params", default=cockpit_paths.robot_config_path())
    b.add_argument("--secrets", default=cockpit_paths.secrets_path())
    b.add_argument("--out", default="env.bin")
    b.add_argument("--board", default=None,
                   help="PlatformIO env name, used only to report the flash offset "
                        "(the RP2 boards keep the block in their top sector rather "
                        "than in a partition)")
    b.add_argument("--host-ip", default=None,
                   help="address of the computer running micro_ros_agent; defaults to "
                        "this machine's own LAN address, and supplies agent_ip, "
                        "syslog_ip and lidar_ip unless the config overrides them")
    b.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                   help="override or add a variable (repeatable)")

    p = sub.add_parser("print", help="decode an env image")
    p.add_argument("image")
    p.add_argument("--show-secrets", action="store_true")

    s = sub.add_parser("set", help="edit variables in an existing image, in place")
    s.add_argument("image")
    s.add_argument("assignments", nargs="+", metavar="KEY=VALUE")

    a = ap.parse_args()

    if a.cmd == "build":
        env = env_from_config(a.params, a.secrets, a.host_ip)
        for item in a.set:
            key, _, value = item.partition("=")
            env[key] = value
        with open(a.out, "wb") as fh:
            fh.write(encode(env))
        offset = env_offset(a.board) if a.board else ENV_OFFSET
        print(f"[mcu_env] wrote {a.out} ({ENV_SIZE} bytes) for flashing at {offset:#x}")
        for key, value in sorted(redact(env).items()):
            print(f"    {key:12} {value}")

    elif a.cmd == "print":
        with open(a.image, "rb") as fh:
            env = decode(fh.read())
        shown = env if a.show_secrets else redact(env)
        for key, value in sorted(shown.items()):
            print(f"{key}={value}")

    elif a.cmd == "set":
        with open(a.image, "rb") as fh:
            env = decode(fh.read())
        for item in a.assignments:
            key, _, value = item.partition("=")
            env[key] = value
        with open(a.image, "wb") as fh:
            fh.write(encode(env))
        print(f"[mcu_env] updated {a.image}")
        for key, value in sorted(redact(env).items()):
            print(f"    {key:12} {value}")


if __name__ == "__main__":
    main()
