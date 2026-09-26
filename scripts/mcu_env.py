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

    runtime     dual_core  i2c_scan  pub_mag  pub_battery  pub_env  best_effort  console
                sim_ld19  lidar_x   (the MCU-side LiDAR emulator, and where on
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
                kp  ki  kd  sim_wheel

    kinematics  base  max_rpm  rpm_ratio  wheel_d  lr_dist  fr_dist  angular_scale
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
    # The simulation keys were `fake_*` before 2026-09-24. Reading such an image is
    # fine -- `print` and `set` both should work on one -- but it must not be
    # mistaken for a current one: the firmware no longer recognises those keys, so
    # every one of them is a compiled-in default silently taken on the board.
    stale = sorted(k for k in env if k.startswith("fake_"))
    if stale:
        print(f"[mcu_env] WARNING: this image predates the sim_ rename; {len(stale)} "
              f"key(s) still use `fake_` ({', '.join(stale[:4])}"
              f"{', ...' if len(stale) > 4 else ''}). Current firmware ignores them. "
              "Rebuild the image with `mcu_env.py build` rather than editing it.",
              file=sys.stderr)
    return env


# ---------------------------------------------------------------------- build
def _refuse_pre_rename_config(params: dict, path: str) -> None:
    """A config written before the sim_ rename must not be read as a blank one.

    The simulation flags were `use_fake_*` until 2026-09-24. Every reader here tests
    for the key by name (`if "use_sim_ld19" in sensors`), so an old config does not
    fail -- the key is simply absent, the flag falls back to a compiled-in default,
    and a bare module is flashed to expect hardware that is not fitted. That is the
    exact shape this project keeps paying for: an unrecognised key is not an error,
    it is a default silently taken.

    Refusing rather than translating is deliberate. A silent translation would let
    two spellings live indefinitely, and the second one only ever surfaces when
    somebody has already spent an afternoon on a board that will not see its IMU.
    The fix is one search-and-replace in the user's own file, so say exactly that.
    """
    stale = []

    def walk(node, trail):
        if isinstance(node, dict):
            for key, value in node.items():
                if isinstance(key, str) and key.startswith("use_fake_"):
                    stale.append(".".join(trail + [key]))
                walk(value, trail + [str(key)])
        elif isinstance(node, list):
            for item in node:
                walk(item, trail)

    walk(params, [])
    if stale:
        sys.exit(
            f"mcu_env: {path} predates the sim_ rename and would be read as a config "
            f"with NO simulation flags set.\n"
            f"  found: {', '.join(sorted(stale))}\n"
            f"  Rename use_fake_* to use_sim_* in that file (nothing else changed).\n"
            f"  Refused rather than guessed: an absent flag is a compiled-in default, "
            f"so a bare module would be flashed to expect hardware it does not have.")


def env_from_config(params_path: str, secrets_path: str, default_host: str = None) -> dict:
    """The environment a robot config and its secrets.yaml imply.

    secrets.yaml stays the only home for credentials (AGENTS.md §3). This reads
    it; it never writes it, and the generated env.bin is gitignored for the same
    reason the secrets file is.
    """
    params = load_yaml(params_path)
    secrets = load_yaml(secrets_path)
    _refuse_pre_rename_config(params, params_path)
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


def _truthy(v) -> bool:
    """How the env spells yes: "1" from _bool, or a real True."""
    return str(v).strip() in ("1", "true", "True")


def _bool(value) -> str:
    return "1" if value else "0"


SENSOR_MODES = ("config", "sim", "real")


def apply_sensor_mode(env: dict, mode: str, params_path: str = None) -> list:
    """Force the env's sensor flags to a MODE, whatever the config said.

    "sim" is what the pipeline's --mode sim means: simulate everything the
    bench does not have. It used to leave the config's flags alone, so
    gendrv_config.yaml -- a real LD19 on GPIO 4, use_sim_ld19: false -- went to
    the board with sim_ld19 0 under --mode sim, emitted nothing on the LiDAR
    bridge, and /scan structurally could not arrive, on both distros. A mode
    called sim that waits for hardware the bench lacks is the config's mode
    with a misleading label.

    "real" is the opposite. "config" (or None) lets the YAML stand -- that is
    --mode auto, and what the real-sensor legs use. The status LED is not a
    sensor and is never touched: simulation mode drives the real one.

    Returns the keys it changed, so the caller can say so in the transcript.
    """
    if not mode or mode == "config":
        return []
    if mode not in SENSOR_MODES:
        raise ValueError(f"sensor mode must be one of {SENSOR_MODES}, not {mode!r}")
    before = dict(env)
    if mode == "sim":
        env["imu"] = "sim"
        env["mag"] = "sim"
        env["sim_wheel"] = "1"
        env["sim_ld19"] = "1"
        env["sim_env"] = "1"
        env["sim_battery"] = "1"
        # ...except the LD19 on a robot whose scan comes from a depth camera: it
        # has no LiDAR to simulate, and the host's simulated camera is its scan
        # (depth_camera.scan_source). An emulated LD19 there streams frames to a
        # driver bringup never starts.
        if params_path:
            import yaml
            import depth_camera
            with open(params_path) as fh:
                controller = (yaml.safe_load(fh) or {}).get("base_controller", {}) or {}
            try:
                if depth_camera.scan_source(controller) == "depth":
                    env["sim_ld19"] = "0"
            except ValueError:
                pass
    else:
        env["sim_wheel"] = "0"
        env["sim_ld19"] = "0"
        env["sim_env"] = "0"
        env["sim_battery"] = "0"
        # A driver can only be un-simd if the config names one.
        sensors = {}
        if params_path:
            import yaml
            with open(params_path) as fh:
                sensors = (yaml.safe_load(fh) or {}).get("base_controller", {}).get("sensors", {}) or {}
        for field in ("imu", "mag"):
            name = str(sensors.get(field, "")).strip()
            if name and name.upper() not in ("AUTO", "NONE", "OFF", "DISABLE", "SIM"):
                env[field] = name.lower()
    return [k for k in env if env.get(k) != before.get(k)]


def _num(value) -> str:
    """Compact, round-trippable text for a number in the env.

    repr() round-trips and the firmware's strtod reads everything it produces,
    including 2.3e-14 -- but it renders a whole number as "3.0", and the
    partition is 4 KB with covariance vectors of up to six values in it, so
    the two bytes matter more than the decimal point does.
    """
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if number == int(number) and abs(number) < 1e16:
        return str(int(number))
    return repr(number)


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
    # /cmd_vel's type. Written only when the config says true or false: `auto`
    # (the default) is the firmware's own default, from the distro its image
    # was built for -- the rule gen_firmware_header.distro_stamps_cmd_vel keeps.
    stamped = tgt.get("stamped_cmd_vel", (params.get("kinematics") or {}).get("stamped_cmd_vel", "auto"))
    if isinstance(stamped, bool) or str(stamped).strip().lower() in ("true", "false", "1", "0", "yes", "no"):
        env["stamped_cmd_vel"] = _bool(str(stamped).strip().lower() in ("true", "1", "yes"))
    # Minutes between Wi-Fi RSSI lines to syslog (0 = off); firmware default 2.
    tel = tgt.get("telemetry") or {}
    if tel.get("wifi_monitor_min") is not None:
        env["wifi_monitor"] = int(tel["wifi_monitor_min"])
    # QoS of the 50 Hz topics. Best effort is the firmware's default (the
    # launch tree's EKF subscribes best-effort; measured 50 Hz at
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
    # AK09918. This is what makes "only the simulated IMU, nothing else" expressible
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

    # The simulated magnetometer is a publisher by intent. A bare module says
    # `mag: NONE` -- there is no chip -- and `use_sim_mag: true`, and the rule
    # above read the first and set pub_mag=0, so the sim field the firmware
    # rotates to the room heading (SimIMUFromWheels::applyMag, built for one
    # purpose: to anchor the heading) was computed and never sent. /imu/mag had
    # no publisher at all, and the madgwick node of the day, waiting for
    # imu/data_raw AND imu/mag as a synchronised pair, published nothing --
    # /imu/data went to 0 Hz and the EKF was left blind. Measured 2026-09-22.
    # The board fuses the field itself now, but magnetometer_calibration still
    # reads /imu/mag. `NONE` means
    # "no chip"; it must not also mean "silence the simulation of one".
    if sensors.get("use_sim_mag"):
        env["pub_mag"] = 1
    # The same rule for the barometer: every bare config says `env: NONE` (no
    # chip) with `use_sim_env: true`, and that NONE set pub_env=0 -- the
    # synthetic barometer was built, read, and never sent.
    if sensors.get("use_sim_env"):
        env["pub_env"] = 1
    # And the battery: `current: NONE` (no INA219, no divider) set pub_battery=0.
    if sensors.get("use_sim_battery"):
        env["pub_battery"] = 1

    # `imu: auto` / `mag: auto` mean the same thing one level down: take whatever
    # answered the bus. The probe is what decides, so ask for it explicitly --
    # otherwise a board with simulated wheels skips the scan (see all_sim in
    # main.cpp) and "auto" would quietly mean "sim".
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
    # only, so a prebuilt image (built from a simulation-mode reference) raycast its
    # room and streamed it on every robot that flashed it, real LiDAR or not.
    # An explicit lidar.use_sim_ld19 outranks the sensors flag, as in the
    # header generator.
    if isinstance(lidar_cfg, dict) and "use_sim_ld19" in lidar_cfg:
        env["sim_ld19"] = _bool(lidar_cfg["use_sim_ld19"])
    elif "use_sim_ld19" in sensors:
        env["sim_ld19"] = _bool(sensors["use_sim_ld19"])
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
    # micro-ROS is up -- boot, agent-waiting, IMU failure and the sim-wall
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
                         ("bat_min", "min_v"), ("bat_max", "max_v"), ("bat_cap", "capacity_ah"),
                         # the sag detector's threshold, percent below the average
                         ("bat_dip", "dip_pct")):
            if bat.get(src) is not None:
                env[key] = bat[src]
    elif bat is not None:
        env["battery_pin"] = bat
    # The HC-SR04. These were compile-time only (TRIG_PIN / ECHO_PIN in the
    # generated header) until 2026-09-20, which meant a released image either
    # had a sonar welded into it or could never have one -- and since no
    # reference config carried the pins, every shipped image was the latter.
    # range.cpp reads these now; -1 or absent means "not wired".
    # Always written, -1 when the robot has none. Omitting them let the firmware
    # fall back to TRIG_PIN/ECHO_PIN from the header -- which is whatever
    # reference the IMAGE was built from -- so a bare module flashed with the
    # release came up driving GP27 and announcing an HC-SR04 that is not there.
    # A pin a config does not mention must read as "not wired", not as "inherit
    # whatever the build happened to know".
    sonar = pins.get("sonar") if isinstance(pins.get("sonar"), dict) else {}
    env["sonar_trig"] = int(sonar.get("trigger", -1))
    env["sonar_echo"] = int(sonar.get("echo", -1))
    # The pin adc_calibrate sweeps its DAC out of, jumpered to the battery ADC
    # input. firmware/src/tools/adc_calibrate.cpp has always read it --
    # envU16("dac_pin", DAC_PIN) -- and nothing has ever written it, so the
    # selector on the ADC Calibration panel could not change anything and the
    # tool always used its compiled-in default.
    #
    # ESP32/S2 ONLY. Building the table means sweeping a hardware DAC, and the
    # ESP32-S3, the C-series and the RP2040/RP2350 do not have one -- adc_lut.h
    # gates the whole facility on exactly that condition, and adc_calibrate
    # refuses to run there. Writing the key anyway would spend bytes of a 4 KB
    # partition describing a pin that no code on that board will ever read.
    mcu = str(tgt.get("mcu") or "").lower()
    has_dac = mcu in ("esp32", "esp32s2")
    if has_dac and pins.get("dac") is not None:
        try:
            dac = int(pins["dac"])
        except (TypeError, ValueError):
            dac = -1
        if dac >= 0:
            env["dac_pin"] = dac
    telemetry = tgt.get("telemetry", {}) or {}
    if telemetry.get("ota_port") is not None:
        env["ota_port"] = int(telemetry["ota_port"])

    # --- sensors. Sim wins: a config asking for a simulated IMU on a board that
    # also names a QMI8658 wants the simulation, not the chip.
    # `auto` and `NONE` both come out as "sim" on the wire: it is the only name
    # createIMU()/createMAG() are guaranteed to accept, and for `auto` the I2C
    # probe overwrites it a moment later. Writing "none" or "auto" verbatim used
    # to reach the firmware as an unknown driver, which it reported and then
    # fell back to sim anyway -- the same outcome, with a warning that looked
    # like a fault.
    def _driver_name(field, sim_flag):
        if sensors.get(sim_flag):
            return "sim"
        value = str(sensors.get(field, "sim")).strip()
        if value.upper() in ("AUTO", "NONE", "OFF", "DISABLE", ""):
            return "sim"
        return value.lower()

    env["imu"] = _driver_name("imu", "use_sim_imu")
    env["mag"] = _driver_name("mag", "use_sim_mag")

    # --- transport and radio
    env["transport"] = tgt.get("transport", "serial")
    # The DDS domain the agent puts this board's participant in (main.cpp,
    # createEntities). Absent means 0, which is what every board has always
    # used; only written when the config names one.
    if tgt.get("domain_id") is not None:
        env["domain_id"] = int(tgt["domain_id"])
    # Which port is the console on an ESP32-S3: its native USB (the DevKit) or
    # UART0 through a bridge (the Yahboom YB-EET01, whose only USB is a CP2102
    # on GPIO 43/44). Same MCU, same image; the env decides. Absent means usb.
    console = str(tgt.get("console", "") or "").strip().lower()
    if console:
        if console not in ("usb", "uart0"):
            raise ValueError(f"base_controller.console must be 'usb' or 'uart0', not {console!r}")
        env["console"] = console
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
    env["sim_wheel"] = _bool(sensors.get("use_sim_wheel", False))
    # The rest of the runtime flags that used to be compile-time macros. Each
    # one is a firmware behaviour that a config can now ask for WITHOUT a
    # rebuild -- but only if it reaches the partition, and `sim_env` did not:
    # env.cpp fell back to the image's compiled default forever, so a bench
    # board flashed with a simulation-mode image reported a synthetic 25 C / 1013 hPa
    # as a "BMP280" no matter what its config said.
    # --- covariance, and the simulated world -------------------------------
    #
    # Ported from linorobot2_hardware's robot_config_engine, which carries all
    # of this in an `imu_tuning` block and emits it as compile-time macros.
    # Here it goes in the env, because in this project a robot is a
    # configuration and not a build: the same released image has to serve a
    # board with an MPU6050 and a board with a BNO085, whose accelerometer
    # variances differ by a factor of seven.
    #
    # The firmware keeps its #ifndef defaults as the fallback, so a blank env
    # still boots with sane values; envFloatVec() leaves them alone when a key
    # is absent. A scalar expands to every axis -- most people have one number,
    # and the ones who measured per-axis values must not have to average them.
    tuning = tgt.get("imu_tuning") or {}
    if not isinstance(tuning, dict):
        tuning = {}

    # Datasheet-derived variances, from the config engine: roughly
    # (noise_density * sqrt(100 Hz))^2. Without these a config that names its
    # IMU still ships the firmware's 1e-5 placeholder, which tells the EKF the
    # sensor is nearly perfect and lets it trust a cheap MPU6050 as much as a
    # BNO085.
    _imu_cov = {
        "BNO085": {"accel_cov": 2.2e-4, "gyro_cov": 1.5e-6, "ori_cov": 4e-3},
        "LSM6DSOX": {"accel_cov": 4.7e-5, "gyro_cov": 4.4e-7},
        "ICM20948": {"accel_cov": 5.1e-4, "gyro_cov": 6.9e-6},
        "ICM42670": {"accel_cov": 4.9e-5, "gyro_cov": 1.4e-6},   # 70 ug/rtHz, 3.8 mdps/rtHz
        "QMI8658": {"accel_cov": 8e-5, "gyro_cov": 2e-6},
        "MPU9250": {"accel_cov": 9e-4, "gyro_cov": 3e-6},
        "MPU9150": {"accel_cov": 1.5e-3, "gyro_cov": 3e-6},
        "MPU6050": {"accel_cov": 1.5e-3, "gyro_cov": 3e-6},
        "GY85": {"accel_cov": 1.8e-3, "gyro_cov": 4.4e-5},
    }
    _mag_cov = {"AK09918": 2.3e-14, "ICM20948": 2.3e-14, "QMC5883L": 4e-14,
                "HMC5883L": 4e-14, "AK8963": 9e-14, "AK8975": 9e-14}
    # pressure Pa^2 (sigma ~1.7 Pa), temperature C^2 (+/-0.5 C), humidity (0..1)^2 (+/-3 %RH)
    _env_cov = {"BMP280": [3, 0.25, 0], "BME280": [3, 0.25, 9e-4]}

    def _clean(name):
        return str(name or "").upper().replace("USE_", "").replace("_IMU", "").replace("_MAG", "")

    defaults = dict(_imu_cov.get(_clean(sensors.get("imu")), {}))
    mcov = _mag_cov.get(_clean(sensors.get("mag")))
    if mcov is not None:
        defaults["mag_cov"] = mcov
    ecov = _env_cov.get(_clean(sensors.get("env")))
    if ecov is not None:
        defaults["env_cov"] = ecov

    def _cov(key, length):
        value = tuning.get(key)
        if value is None or value == "":
            value = defaults.get(key)
        if value is None or value == "":
            return
        values = list(value) if isinstance(value, (list, tuple)) else [value]
        if len(values) not in (1, length):
            return
        env[key] = ",".join(_num(v) for v in values)

    for _key, _len in (("accel_cov", 3), ("gyro_cov", 3), ("ori_cov", 3),
                       ("mag_cov", 3), ("pose_cov", 6), ("twist_cov", 6),
                       ("env_cov", 3)):
        _cov(_key, _len)

    # Hard-iron offsets. Three axes or nothing: an all-zero bias is not a
    # calibration, and a partial one would silently zero the axes it omits.
    bias = tuning.get("mag_bias")
    if isinstance(bias, (list, tuple)) and len(bias) == 3:
        try:
            values = [float(v) for v in bias]
        except (TypeError, ValueError):
            values = []
        if values and any(v != 0.0 for v in values):
            env["mag_bias"] = ",".join(_num(v) for v in values)

    # How long the board may stop feeding its watchdog before it resets, in
    # seconds. 0 disables it; the firmware's own default is 8 s.
    #
    # One key for both families because it is one question -- and until now the
    # RP2 armed a hardware watchdog at a hardcoded 8000 ms while the ESP32
    # armed nothing at all, so the family with the radio, the one that can
    # actually stall on a network, was the one with no watchdog.
    wdt = tgt.get("wdt_timeout")
    if wdt is not None:
        try:
            seconds = int(wdt)
        except (TypeError, ValueError):
            seconds = -1
        # The config engine validates 1-300 s; 0 is "off", which it does not
        # offer and this does.
        if 0 <= seconds <= 300:
            env["wdt_timeout"] = seconds
        else:
            print(f"[mcu_env] ignoring wdt_timeout {wdt!r}: give 0 to disable, "
                  f"or 1-300 seconds")

    # The topic namespace. Two robots on one DDS domain used to need a rebuild
    # each, because the prefix was pasted onto every topic name at compile
    # time -- and the published images, built from the generated bare config,
    # could not carry one at all. A trailing slash is added when it is missing,
    # so `robot1` and `robot1/` mean the same thing (the config engine, which
    # emits TOPIC_PREFIX, normalises it the same way).
    prefix = tgt.get("topic_prefix")
    if prefix is not None:
        prefix = str(prefix).strip().strip('"')
        if prefix and not prefix.endswith("/"):
            prefix += "/"
        # Only what a ROS 2 topic name may contain. A prefix the middleware
        # rejects leaves the robot with NO topics and nothing to say why, so it
        # is better refused here, where a person is watching.
        if prefix and all(c.isalnum() or c in "_/" for c in prefix):
            env["topic_prefix"] = prefix
        elif prefix:
            print(f"[mcu_env] ignoring topic_prefix {prefix!r}: a ROS 2 topic "
                  f"name may only contain letters, digits, underscore and /")

    # The barometer's address. 0x77 on the Waveshare General Driver board,
    # 0x76 on most breakouts -- env.cpp probes the compiled-in one only.
    if tgt.get("bmp280_addr") is not None:
        try:
            env["bmp280_addr"] = int(str(tgt["bmp280_addr"]), 0)
        except (TypeError, ValueError):
            pass

    # The simulated world. Simulation mode is this project's DEFAULT, so the room the
    # emulator raycasts and the mass it accelerates are configuration, not
    # constants -- a Nav2 test wants to move the obstacle wall without
    # rebuilding, and a 20 kg robot does not accelerate like a 3.5 kg one.
    sim = tgt.get("simulation") or {}
    if isinstance(sim, dict) and sim.get("walls"):
        # Interior walls: one key, "x1,y1,x2,y2;..." (sim_ld19.h applyEnvRoom).
        import depth_camera
        env["sim_walls"] = depth_camera.walls_env(depth_camera.sim_walls({"base_controller": tgt}))
    if isinstance(sim, dict):
        for key, cast in (("sim_map_w", float), ("sim_map_h", float),
                          ("sim_wall", int), ("sim_wall_x1", float),
                          ("sim_wall_y1", float), ("sim_wall_x2", float),
                          ("sim_wall_y2", float), ("sim_mass", float),
                          ("sim_noise_rpm", float), ("sim_gear_eff", float),
                          ("sim_coulomb", float), ("sim_sag", float),
                          ("sim_sag_tau", float), ("sim_drv_drop", float),
                          ("sim_drv_r", float), ("sim_stall_a", float),
                          ("sim_ilimit_a", float),
                          ("sim_imu_mount_roll", float),
                          ("sim_imu_mount_pitch", float)):
            src = {"sim_map_w": "map_width", "sim_map_h": "map_height",
                   "sim_wall": "wall_obstacle", "sim_wall_x1": "wall_x1",
                   "sim_wall_y1": "wall_y1", "sim_wall_x2": "wall_x2",
                   "sim_wall_y2": "wall_y2", "sim_mass": "robot_mass",
                   "sim_noise_rpm": "wheel_noise_rpm",
                   # The drivetrain's losses. Sweeping these is how you find out
                   # which one a navigation failure was sensitive to, and it must
                   # not cost a firmware build per value.
                   "sim_gear_eff": "gear_efficiency",
                   "sim_coulomb": "gear_drag_rpm",
                   "sim_sag": "battery_sag",
                   # The pack's sag LAGS -- a held load sags deeper than a brief
                   # one -- and the bridge keeps some of the voltage for itself,
                   # instantly rather than with the pack's chemistry.
                   "sim_sag_tau": "battery_sag_tau_ms",
                   "sim_drv_drop": "driver_drop",
                   "sim_drv_r": "driver_resistance",
                   # Many small drivers chop at a fixed current, which caps
                   # TORQUE rather than speed -- the one limit that bites hardest
                   # from rest, where a robot is judged. 0 means none fitted.
                   "sim_stall_a": "motor_stall_amps",
                   "sim_ilimit_a": "driver_current_limit",
                   # How the IMU is actually mounted, in degrees. An
                   # accelerometer reports specific force, so a tilt leans
                   # gravity into ax and ay -- the two the EKF fuses. Zero here
                   # is a perfectly level part, which is what this model assumed
                   # unconditionally until 2026-09-25, and no simulated robot
                   # could reach the code that exists to correct for it.
                   "sim_imu_mount_roll": "imu_mount_roll_deg",
                   "sim_imu_mount_pitch": "imu_mount_pitch_deg"}[key]
            if sim.get(src) is None:
                continue
            try:
                value = cast(sim[src])
            except (TypeError, ValueError):
                continue
            env[key] = int(value) if cast is int else _num(value)
        # The robot's own structure in the simulated LD19's view (lidar_mask.py):
        # sectors of the robot frame that return a near range, so the bench can
        # prove the host's lidar.mask removes them. Validated here, where a person
        # is watching, rather than parsed wrong on the board.
        import lidar_mask
        env.update(lidar_mask.occlusion_env({"base_controller": tgt}))

    # How close the simulated robot's centre may come to a simulated wall.
    #
    # This used to be compiled in (#define SIM_ROBOT_RADIUS), derived from the
    # robot_radius of whatever reference config happened to build the image --
    # so a released image carried one robot's dimensions and any other robot
    # flashed with it clamped at the wrong distance. When the two disagree the
    # failure is not cosmetic: at 0.20 m against a config planning with 0.26 m
    # the clamp parks the robot INSIDE Nav2's own footprint, the cell is lethal,
    # the planner will not plan out of it, and a soak sat there for 154
    # consecutive goals while the planner emitted escape paths the controller
    # refused. It is a robot fact, so it travels as an env key like the room.
    #
    # `simulation.robot_radius` overrides; otherwise it follows the largest
    # radius the robot's own costmaps plan with, which is the number that has
    # to agree.
    radius = sim.get("robot_radius") if isinstance(sim, dict) else None
    if radius is None:
        radius = gen_firmware_header.nav2_robot_radius(params)
    try:
        env["sim_radius"] = _num(float(radius))
    except (TypeError, ValueError):
        pass

    env["sim_env"] = _bool(sensors.get("use_sim_env", False))
    # A simulated battery voltage sensor (battery.cpp): a pack that sags with the
    # simulated wheels' load and drains as the robot drives.
    env["sim_battery"] = _bool(sensors.get("use_sim_battery", False))
    # The simulated ultrasonic cone. Only ever used when the LiDAR emulator is
    # running (it raycasts from the same room) and no real sonar is wired, so
    # the firmware gates it anyway; this says whether the bench wants it.
    env["sim_sonar"] = _bool(sensors.get("use_sim_sonar", True))
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
    # A REAL sensor only. main.cpp computes
    #     range_sim = sim_lidar_on && envFlag("sim_sonar", true)
    # and would happily brake on a range raycast out of the simulated room, so
    # a bench board running a real robot's config in simulation mode would behave
    # differently from every other bench board -- and a hazard stop is the
    # last thing that should be exercised against an imaginary obstacle.
    #
    # Same rule the firmware uses, mirrored here so the decision is visible in
    # the env rather than implied by two flags. Simulation mode overriding an
    # explicit config setting is the established behaviour for every other
    # sim_* key (see apply_sensor_mode).
    if _truthy(env.get("sim_ld19")) and _truthy(env.get("sim_sonar")):
        env["safety_stop"] = "0"
    # Track the speed ceiling against the live pack voltage (INA219 / divider)
    # rather than the static config voltage. OFF by default: it changes how the
    # robot feels as the battery sags, and wants a real load to be worth it.
    rpm_track = tgt.get("rpm_track_voltage")
    if rpm_track is not None:
        env["rpm_track_voltage"] = _bool(rpm_track)
    # Per-motor stall / encoder-loss guard. OFF by default: it needs real
    # encoders (a SimEncoder always tracks the command), and a mistuned floor
    # could stop a slow-ramping robot. `stall_detect` enables it; `stall_ms` is
    # how long a commanded-but-not-counting wheel is tolerated (200-10000 ms);
    # `stall_rpm_floor` is the |rpm| below which a wheel counts as not turning.
    stall = tgt.get("stall_detect")
    if isinstance(stall, dict):
        env["stall_detect"] = _bool(stall.get("enabled", False))
        env["stall_ms"] = _num(stall.get("ms", 1500))
        env["stall_rpm_floor"] = _num(stall.get("rpm_floor", 5.0))
    elif stall is not None:
        env["stall_detect"] = _bool(stall)
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
                     ("fr_dist", "fr_wheels_distance"),
                     ("angular_scale", "angular_scale"),
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
    b.add_argument("--sensors", choices=SENSOR_MODES, default="config",
                   help="sim: simulate every sensor whatever the config says (the "
                        "pipeline's --mode sim); real: the opposite; config: let the "
                        "YAML stand (--mode auto). The status LED is never touched.")

    p = sub.add_parser("print", help="decode an env image")
    p.add_argument("image")
    p.add_argument("--show-secrets", action="store_true")

    s = sub.add_parser("set", help="edit variables in an existing image, in place")
    s.add_argument("image")
    s.add_argument("assignments", nargs="+", metavar="KEY=VALUE")

    a = ap.parse_args()

    if a.cmd == "build":
        env = env_from_config(a.params, a.secrets, a.host_ip)
        changed = apply_sensor_mode(env, a.sensors, a.params)
        if changed:
            print(f"[mcu_env] --sensors {a.sensors} overrode the config: {', '.join(changed)}")
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
