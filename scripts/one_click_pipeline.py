#!/usr/bin/env python3
# ==============================================================================
# one_click_pipeline.py — 1-Click Pipeline from Bare Module to Nav2/SLAM/Map
#
# Runs ON THE ROBOT COMPUTER: the machine the microcontroller is plugged into
# and the machine that runs ROS 2. There is no other machine in the picture --
# the browser that pressed the button is just a browser.
#
#   config -> firmware (local build or prebuilt release image) -> probe
#          -> flash (only what the probe says is needed) -> bringup
#          -> topic gate -> SLAM -> Nav2 -> map
#
# Process safety (AGENTS.md): targeted PID signalling only, never pkill or
# killall; ports 8000, 5173 and 9090 are immune.
# ==============================================================================

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import cockpit_paths  # noqa: E402
import fetch_prebuilt  # noqa: E402
import gen_bare_config  # noqa: E402
import mcu_identity  # noqa: E402
import robot_stack  # noqa: E402  (what a kept-running stack leaves behind)

CONFIG_DIR = cockpit_paths.ensure_config_dir(quiet=True)
DEFAULT_ROBOT = cockpit_paths.DEFAULT_ROBOT
LOG_DIR = os.path.join(REPO_ROOT, "logs")


# ------------------------------------------------------------------ robot config
def select_robot_config(robot: str = None, controller: str = None) -> str:
    """The robot config this run is for.

    --controller has to select the config too, not just rename what is in it:
    the Cockpit passes --controller alone, and resolving the DEFAULT robot's
    file and then overwriting nothing but the name once ran one robot's
    controller against another robot's port and pins. Resolution order:
    --robot, then <controller>_config.yaml, then the config whose
    base_controller.name matches (first by filename, and the run says which).
    """
    if robot:
        return cockpit_paths.robot_config_path(robot)
    if controller:
        direct = os.path.join(CONFIG_DIR, f"{controller}_config.yaml")
        if os.path.isfile(direct):
            return direct
        matches = []
        for path in cockpit_paths.robot_config_files():
            try:
                with open(path) as fh:
                    cand = yaml.safe_load(fh) or {}
            except Exception:
                continue
            if (cand.get("base_controller") or {}).get("name") == controller:
                matches.append(path)
        if matches:
            if len(matches) > 1:
                names = ", ".join(os.path.basename(m) for m in matches)
                print(f"  ⚠️ {len(matches)} robot configs declare base controller "
                      f"'{controller}' ({names}); using {os.path.basename(matches[0])}. "
                      f"Pass --robot <name> to choose explicitly.")
            return matches[0]
    return cockpit_paths.robot_config_path()


def detect_default_distro() -> str:
    env_distro = os.environ.get("ROS_DISTRO")
    if env_distro:
        return env_distro
    for d in ("lyrical", "jazzy", "rolling"):
        if os.path.exists(f"/opt/ros/{d}"):
            return d
    return "jazzy"


# ------------------------------------------------------------------- shell env
def get_ros_env(distro: str = "auto") -> str:
    """The shell prefix every ROS 2 step of the run is executed under.

    The PlatformIO venv dirs go LAST on PATH, and that is load-bearing: a venv
    bin dir also owns `python3`, and reached through it Python loses
    /usr/lib/python3/dist-packages -- numpy vanishes and rclpy.node fails
    several frames deep. `pio` has no competitor on PATH and loses nothing by
    being resolved last.

    Every step also gets the Fast DDS service QoS profile: rmw_fastrtps drops a
    service reply when the response writer has not matched within 100 ms, and
    bringing nav2 up starts enough participants at once to miss that window
    (docs/ros2-stack.md). It is a ceiling, not a delay.
    """
    path_export = ("export PATH=$HOME/.local/bin:/usr/local/bin:$PATH:"
                   "$HOME/.pioenv/bin:$HOME/.platformio/penv/bin")
    qos = cockpit_paths.FASTDDS_PROFILE
    qos_export = (f'[ -f "{qos}" ] && export FASTDDS_DEFAULT_PROFILES_FILE="{qos}"; true')
    ros_setup = (
        f"if [ -n \"{distro}\" ] && [ \"{distro}\" != \"auto\" ] && [ -f /opt/ros/{distro}/setup.bash ]; "
        f"then source /opt/ros/{distro}/setup.bash; "
        "elif [ -f /opt/ros/lyrical/setup.bash ]; then source /opt/ros/lyrical/setup.bash; "
        "elif [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; "
        "elif [ -f /opt/ros/rolling/setup.bash ]; then source /opt/ros/rolling/setup.bash; fi"
    )
    uros_setup = (
        "if [ -f /uros_ws/install/setup.bash ]; then source /uros_ws/install/setup.bash; "
        "elif [ -f /opt/uros_ws/install/setup.bash ]; then source /opt/uros_ws/install/setup.bash; "
        "elif [ -f $HOME/uros_ws/install/setup.bash ]; then source $HOME/uros_ws/install/setup.bash; fi"
    )
    # Nav2 built from source, on the distros where it has no binary package
    # (lyrical has the nav2_* components but no nav2_bringup). Sourced before
    # the cockpit overlay so the overlay still wins on any shared name.
    nav2_setup = (
        "if [ -f /opt/nav2_ws/install/setup.bash ]; then "
        "source /opt/nav2_ws/install/setup.bash; fi"
    )
    ws_setup = (
        "if [ -f /opt/lino_ws/setup.bash ]; then source /opt/lino_ws/setup.bash; "
        f"elif [ -f {REPO_ROOT}/install/setup.bash ]; then source {REPO_ROOT}/install/setup.bash; "
        f"elif [ -f {REPO_ROOT}/../../install/setup.bash ]; then source {REPO_ROOT}/../../install/setup.bash; "
        "elif [ -f $HOME/cockpit_ws/install/setup.bash ]; then source $HOME/cockpit_ws/install/setup.bash; "
        "elif [ -f $HOME/linorobot2_ws/install/setup.bash ]; then source $HOME/linorobot2_ws/install/setup.bash; fi"
    )
    return (f"{path_export} && {qos_export} && {ros_setup} && {uros_setup}"
            f" && {nav2_setup} && {ws_setup}")


# Every place a built ROS 2 workspace can be, in the order the shell chain below
# tries them. The container image this project ships puts one at
# /opt/lino_ws/setup.bash -- NOT at <ws>/install/setup.bash -- and the cockpit's
# own "is the workspace built?" check only knew the source-checkout layout, so
# on the official image the Bringup tab refused to start and ran a colcon build
# in a directory that does not exist. One list, both readers.
WORKSPACE_SETUPS = (
    "/opt/lino_ws/setup.bash",
    os.path.join(REPO_ROOT, "install", "setup.bash"),
    os.path.join(REPO_ROOT, "..", "..", "install", "setup.bash"),
    os.path.expanduser("~/cockpit_ws/install/setup.bash"),
    os.path.expanduser("~/linorobot2_ws/install/setup.bash"),
)


def workspace_setup() -> str:
    """The first workspace setup.bash that exists, or "" when none does."""
    for path in WORKSPACE_SETUPS:
        if os.path.isfile(path):
            return os.path.abspath(path)
    return ""


def wants_stamped_cmd_vel(distro: str, controller_cfg: dict, params: dict) -> bool:
    """Is /cmd_vel TwistStamped for this run?

    nav2 1.4 (kilted) flipped TwistPublisher to TwistStamped, so the distro
    decides unless the robot config says otherwise. The firmware answers the
    same question at build time through gen_firmware_header.py's
    `stamped_cmd_vel: auto`, from the same --distro.
    """
    if controller_cfg.get("stamped_cmd_vel") not in (None, "auto", ""):
        return str(controller_cfg["stamped_cmd_vel"]).lower() in ("true", "1", "yes")
    kine = (params.get("kinematics") or {}).get("stamped_cmd_vel")
    if isinstance(kine, bool):
        return kine
    return distro not in ("humble", "iron", "jazzy")


# What each fitted sensor is expected to publish. The firmware guards every one
# of these behind a compile-time macro that gen_firmware_header.py derives from
# this same `sensors:` block, so the config is the only thing that knows which
# chips a board carries:
#
#   imu      -> /imu/data (already a hard requirement of the gate) + /imu/data_raw
#   mag      -> /imu/mag        firmware/src/main.cpp, #ifdef PUBLISH_MAG
#   current  -> /battery        BATTERY_PIN, or an INA219 found on the bus
#   env      -> /pressure, /temperature   a barometer on the bus, and only when
#                                         the chip answered at boot (env_present)
#
# `env` is the one that can legitimately be configured and still silent: the
# publisher is created only if the barometer answered. That is precisely the
# failure worth catching on a real base, so it is required like the rest.
SENSOR_TOPICS = {
    "mag":     ["/imu/mag"],
    "current": ["/battery"],
    "env":     ["/pressure", "/temperature"],
}


def sensor_topics(controller_cfg: dict) -> list:
    """Auxiliary topics the fitted sensors must publish, in a stable order.

    A sensor counts as fitted when the config names a chip for it AND does not
    ask for the fake version -- `use_fake_mag: true` means the driver is
    synthesising values, which is a different thing to verify and never a real
    chip on the bus.
    """
    sensors = controller_cfg.get("sensors") or {}
    topics = []
    for key, tops in SENSOR_TOPICS.items():
        if not sensors.get(key):
            continue
        if sensors.get(f"use_fake_{key}"):
            continue
        topics.extend(tops)
    return topics


def resolve_pio_env(pio_env: str, distro: str) -> str:
    """The PlatformIO env that matches the ROS 2 distro of the run.

    The distro is the one property of an image that cannot travel in the env
    partition: board_microros_distro is fixed at link time, and a jazzy image
    never holds a session with a lyrical agent -- silently. Anything but jazzy
    maps to the `<env>_<distro>` variant firmware/platformio.ini declares.
    """
    if not distro or distro == "jazzy" or pio_env.endswith(f"_{distro}"):
        return pio_env
    candidate = f"{pio_env}_{distro}"
    ini = os.path.join(REPO_ROOT, "firmware", "platformio.ini")
    try:
        declared = f"[env:{candidate}]" in open(ini).read()
    except OSError:
        declared = False
    if not declared:
        raise SystemExit(
            f"\n❌ No PlatformIO env '{candidate}' in firmware/platformio.ini.\n"
            f"   board_microros_distro is fixed at link time, so a '{distro}' run needs its own\n"
            f"   image; flashing '{pio_env}' would link jazzy micro_ros and never hold a session."
        )
    return candidate


# ------------------------------------------------------------- process safety
IMMUNE_PORTS = (8000, 5173, 9090)


def _immune_pids() -> set:
    """PIDs that must never be signalled: ours, and whoever owns an immune port.

    The flasher runs as a child of the very backend that holds 8000, so
    "release the serial port" must never be allowed to reach it.
    """
    immune = {os.getpid(), os.getppid()}
    try:
        immune.add(os.getpgid(0))
    except Exception:
        pass
    try:
        res = subprocess.run(["ss", "-tlnp"], capture_output=True, text=True)
        for line in res.stdout.splitlines():
            if not any(f":{port} " in line or line.rstrip().endswith(f":{port}") for port in IMMUNE_PORTS):
                continue
            for match in re.findall(r"pid=(\d+)", line):
                immune.add(int(match))
    except Exception:
        pass
    return immune


def release_serial_port(serial_port: str):
    """Release the serial port from micro_ros_agent or any other holder, by PID."""
    if not serial_port or not os.path.exists(serial_port):
        return
    immune = _immune_pids()

    def signal_pid(pid: int, sig) -> bool:
        if pid in immune or pid <= 1:
            return False
        try:
            os.kill(pid, sig)
            return True
        except Exception:
            return False

    # By EXECUTABLE NAME, never by a substring of the command line: a shell that
    # sources uros_ws or an editor holding the file open mentions the string too.
    try:
        ps_res = subprocess.run(["ps", "-eo", "pid=,comm=,args="], capture_output=True, text=True)
        for line in ps_res.stdout.splitlines():
            parts = line.split(None, 2)
            if len(parts) < 2:
                continue
            argv = parts[2] if len(parts) > 2 else ""
            exe = os.path.basename(argv.split()[0]) if argv.split() else ""
            if parts[1] != "micro_ros_agent" and exe != "micro_ros_agent":
                continue
            try:
                signal_pid(int(parts[0]), signal.SIGINT)
            except ValueError:
                pass
    except Exception:
        pass

    try:
        for _ in range(8):
            res = subprocess.run(["lsof", "-t", serial_port], capture_output=True, text=True)
            if res.returncode != 0 or not res.stdout.strip():
                break
            signalled = False
            for pid in res.stdout.split():
                try:
                    pid_i = int(pid)
                except ValueError:
                    continue
                if pid_i in immune:
                    print(f"  ⚠️ {serial_port} is held by PID {pid_i}, which owns an immune "
                          f"port -- not signalling it.")
                    continue
                if signal_pid(pid_i, signal.SIGINT):
                    signalled = True
                    time.sleep(0.2)
                    signal_pid(pid_i, signal.SIGTERM)
            if not signalled:
                break
            time.sleep(0.5)
    except Exception:
        pass
    time.sleep(1.0)


# ------------------------------------------------------------------ execution
def run_streamed(full_cmd: list, timeout: int, log_tag: str = None,
                 prefix: str = "  ") -> subprocess.CompletedProcess:
    """Run a long step with its output relayed live and mirrored to logs/<tag>.log.

    Each step gets its own process group; a timeout signals the group so no
    pio/picotool child is left holding the serial port.
    """
    sink = None
    if log_tag:
        os.makedirs(LOG_DIR, exist_ok=True)
        try:
            sink = open(os.path.join(LOG_DIR, f"{log_tag}.log"), "w")
        except Exception:
            sink = None
    try:
        proc = subprocess.Popen(full_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1, start_new_session=True)
    except FileNotFoundError as exc:
        if sink:
            sink.close()
        return subprocess.CompletedProcess(full_cmd, 127, "", str(exc))

    lines = []

    def pump():
        try:
            for line in proc.stdout:
                lines.append(line)
                sys.stdout.write(prefix + line)
                sys.stdout.flush()
                if sink:
                    sink.write(line)
                    sink.flush()
        except Exception:
            pass

    reader = threading.Thread(target=pump, daemon=True)
    reader.start()
    timed_out = False
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
        print(f"  ⏱ Step exceeded {timeout}s — terminating its process group.", flush=True)
        try:
            pgid = os.getpgid(proc.pid)
        except Exception:
            pgid = None
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                if pgid is not None:
                    os.killpg(pgid, sig)
                else:
                    proc.send_signal(sig)
                proc.wait(timeout=3)
                break
            except ProcessLookupError:
                break
            except Exception:
                continue
    reader.join(timeout=3)
    try:
        proc.stdout.close()
    except Exception:
        pass
    if sink:
        sink.close()
    rc = 124 if timed_out else (proc.returncode if proc.returncode is not None else 1)
    err = f"Command timed out after {timeout}s" if timed_out else ""
    return subprocess.CompletedProcess(full_cmd, rc, "".join(lines), err)


def run_ros(cmd_str: str, timeout: int = 60, distro: str = "jazzy") -> subprocess.CompletedProcess:
    """A short ROS 2 command, under the run's environment, output captured."""
    full_cmd = ["bash", "-c", f"{get_ros_env(distro)} && {cmd_str}"]
    try:
        return subprocess.run(full_cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired as e:
        out = (e.stdout.decode() if isinstance(e.stdout, bytes) else e.stdout) or ""
        err = (e.stderr.decode() if isinstance(e.stderr, bytes) else e.stderr) or ""
        return subprocess.CompletedProcess(full_cmd, 124, out, err or f"Command timed out after {timeout}s")


open_log_files = []


def launch_bg(cmd_str: str, log_tag: str = "launch", distro: str = "jazzy") -> subprocess.Popen:
    """A long step (bringup, SLAM, Nav2) in its own process group, logged to logs/<tag>.log."""
    os.makedirs(LOG_DIR, exist_ok=True)
    f = open(os.path.join(LOG_DIR, f"{log_tag}.log"), "w")
    open_log_files.append(f)
    full_cmd = ["bash", "-c", f"{get_ros_env(distro)} && exec {cmd_str}"]
    return subprocess.Popen(full_cmd, stdout=f, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)


def stop_bg(proc, first=signal.SIGINT):
    """Stop a background step: its process GROUP, by the pid we started, SIGINT
    first because `ros2 launch` tears its nodes down on SIGINT and is merely
    killed by SIGTERM. Never a name match."""
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except Exception:
        return
    for sig in (first, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pgid, sig)
            proc.wait(timeout=2)
            break
        except ProcessLookupError:
            break
        except Exception:
            continue


def wait_for_nav2_activation(timeout_sec: int = 90) -> tuple:
    """Watch logs/nav2.log for lifecycle_manager's verdict.

    "Launched" and "active" are different events: one node that fails to
    configure aborts the whole managed set, after the healthy ones logged a
    clean configure. Returns (ok, detail); detail names the failed node when
    the log shows it.
    """
    log_path = os.path.join(LOG_DIR, "nav2.log")
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        try:
            with open(log_path, "r", errors="replace") as fh:
                text = fh.read()
        except OSError:
            text = ""
        if "Failed to bring up all requested nodes" in text:
            who = ""
            for line in text.splitlines():
                if "Failed to change state for node" in line:
                    who = line.rsplit(":", 1)[-1].strip()
            fatal = [ln for ln in text.splitlines() if "[FATAL]" in ln]
            detail = f"node '{who}' failed to configure" if who else "a node failed to configure"
            if fatal:
                detail += f" -- {fatal[-1].split('] ', 2)[-1].strip()}"
            return False, detail
        if "Managed nodes are active" in text:
            return True, "managed nodes are active"
        time.sleep(1.0)
    return False, f"lifecycle_manager reported neither success nor failure within {timeout_sec}s"


def _odom_xy(distro: str):
    """The base's current (x, y) from one /odom/unfiltered message, or None."""
    res = run_ros("ros2 topic echo --once --field pose.pose.position /odom/unfiltered",
                  timeout=15, distro=distro)
    m = re.search(r"x:\s*(-?[0-9.eE+-]+)\s*y:\s*(-?[0-9.eE+-]+)", res.stdout or "")
    return (float(m.group(1)), float(m.group(2))) if m else None


# How far from (0, 0) the base and the EKF may sit after the reset and still
# count as "at the origin". The goal is 3 m away and the start gap the gate
# needs is 1 m, so a few cm change nothing; 0.05 argued with a healthy GenDrv.
ORIGIN_TOL = 0.10


def _odom_speed(distro: str):
    """|v| of the base from one /odom/unfiltered message, or None."""
    res = run_ros("ros2 topic echo --once --field twist.twist.linear /odom/unfiltered",
                  timeout=15, distro=distro)
    m = re.search(r"x:\s*(-?[0-9.eE+-]+)\s*y:\s*(-?[0-9.eE+-]+)", res.stdout or "")
    return math.hypot(float(m.group(1)), float(m.group(2))) if m else None


def _ekf_xy(distro: str):
    """The EKF's current (x, y) from one /odom message -- what Nav2 steers by -- or None."""
    res = run_ros("ros2 topic echo --once --field pose.pose.position /odom",
                  timeout=15, distro=distro)
    m = re.search(r"x:\s*(-?[0-9.eE+-]+)\s*y:\s*(-?[0-9.eE+-]+)", res.stdout or "")
    return (float(m.group(1)), float(m.group(2))) if m else None


def wait_for_topic(topic_name: str, timeout_sec: int = 30, require_publisher: bool = False,
                   distro: str = "jazzy", require_message: str = None) -> bool:
    """Wait for a topic to exist, to have a publisher, or to actually carry data.

    `require_message` is a field path to echo, and it is the only one of the
    three that proves anything about a LIFECYCLE node. slam_toolbox creates its
    publishers in on_configure(), so /map is listed and has a publisher the
    moment the node configures -- whether or not it ever activates. A pico2
    release test passed this gate with "✅ /map is active" against a
    slam_toolbox that had logged "Configuring", chosen its solver and then
    stopped; the map frame never existed, and Nav2 failed thirty seconds later
    with planner_server unable to transform base_link to map. The gate named
    the wrong step, which is worse than no gate.
    """
    start = time.time()
    while time.time() - start < timeout_sec:
        try:
            if require_message:
                res = run_ros(f"timeout 5 ros2 topic echo {topic_name} --once "
                              f"--field {require_message} 2>/dev/null",
                              timeout=10, distro=distro)
                if res.returncode == 0 and res.stdout.strip():
                    return True
            elif require_publisher:
                res = run_ros(f"ros2 topic info {topic_name} 2>/dev/null", timeout=5, distro=distro)
                if res.returncode == 0:
                    for line in res.stdout.splitlines():
                        if "Publisher count:" in line:
                            parts = line.split(":")
                            if len(parts) > 1 and int(parts[1].strip()) > 0:
                                return True
            else:
                res = run_ros(f"ros2 topic list | grep -w '{topic_name}'", timeout=5, distro=distro)
                if res.returncode == 0 and topic_name in res.stdout:
                    return True
        except Exception:
            pass
        time.sleep(1.0)
    return False


# ------------------------------------------------------------------- firmware
def pio_available(distro: str) -> bool:
    return run_ros("command -v pio >/dev/null 2>&1", timeout=15, distro=distro).returncode == 0


def firmware_source(mode: str, pio_env: str, distro: str) -> tuple:
    """("build", None) or ("prebuilt", <dir>): where the image for this run comes from.

    auto: build when PlatformIO is on this machine (a developer's tree should
    track the tree), otherwise the release image for this env, fetched if it is
    not already in firmware/prebuilt/. Building is the exception on a robot; a
    Pi can, through `docker compose run --rm pio pio run -e <env>`, but most
    users never need to.
    """
    if mode == "build":
        return "build", None
    if mode == "auto" and pio_available(distro):
        return "build", None
    profile = fetch_prebuilt.profile_for_env(pio_env)
    try:
        return "prebuilt", fetch_prebuilt.fetch(profile)
    except SystemExit as exc:
        raise SystemExit(f"{exc}\n   (no PlatformIO here either; install it or use the pio build image)")


def probe_board(pio_env: str, port: str, baud: int, params_path: str,
                app: str = "base", prebuilt_dir: str = None) -> dict:
    """Ask the board what it is running before anything is written to it."""
    argv = [sys.executable, os.path.join(REPO_ROOT, "scripts", "mcu_probe.py"),
            "--env", pio_env, "--port", port, "--baud", str(baud),
            "--params", params_path, "--app", app, "--json"]
    if prebuilt_dir:
        argv += ["--prebuilt-dir", prebuilt_dir]
    res = subprocess.run(argv, capture_output=True, text=True, check=False)
    try:
        board = json.loads(res.stdout[res.stdout.index("{"):])
    except Exception:
        # A probe that cannot run must not read as "nothing to do", and must
        # not read as "stale" either: auto-update acts only on a board that
        # ANSWERED and answered differently. The env write stays on (4 KB,
        # reversible).
        print(f"  ⚠️  could not probe the board: "
              f"{(res.stderr or res.stdout or 'no output').strip().splitlines()[-1:]}")
        return {"verdict": "unknown", "needs_env_write": True, "firmware_differs": True,
                "probe_failed": True, "installed": {}, "local": {"git": "unknown"}, "_human": ""}
    try:
        import mcu_probe
        board["_human"] = mcu_probe.human(board)
    except Exception:
        board["_human"] = json.dumps(board, indent=2)
    return board


def sensors_for_mode(mode: str):
    """What --mode means for the env block's sensor flags.

    fake forces every fake_* flag on and real forces them off, whatever the
    config says; auto lets the YAML stand. This is the whole meaning of the
    switch: a config that describes a real LD19 run in fake mode used to reach
    the board with fake_ld19 0, and /scan then structurally could not arrive.
    """
    return {"fake": "fake", "real": "real"}.get(mode)


def write_env_only(pio_env: str, port: str, baud: int, params_path: str,
                   controller: str, timeout: int = 600, sensors: str = None) -> bool:
    """The 4 KB write: the board's description, without touching its firmware."""
    argv = [sys.executable, "-u", os.path.join(REPO_ROOT, "scripts", "flash_mcu.py"), "--env-only",
            "--env", pio_env, "--port", port, "--baud", str(baud),
            "--params", params_path, "--firmware-name", controller,
            "--app", "base", "--total-timeout", str(timeout)]
    if sensors:
        argv += ["--sensors", sensors]
    return run_streamed(argv, timeout=timeout + 60, log_tag="flash", prefix="    | ").returncode == 0


def flash_firmware(pio_env: str, port: str, baud: int, params_path: str, controller: str,
                   source: str, prebuilt_dir: str, args) -> bool:
    """Write the image (and the env block) with flash_mcu.py — never `pio run -t upload`."""
    argv = [sys.executable, "-u", os.path.join(REPO_ROOT, "scripts", "flash_mcu.py"),
            "--port", port, "--baud", str(baud), "--firmware-name", controller,
            "--params", params_path, "--app", "base",
            "--timeout", str(args.flash_attempt_timeout), "--total-timeout", str(args.flash_timeout)]
    if source == "prebuilt":
        argv += ["--prebuilt", os.path.basename(prebuilt_dir)]
    else:
        argv += ["--firmware-dir", "firmware", "--env", pio_env]
    sensors = sensors_for_mode(args.mode)
    if sensors:
        argv += ["--sensors", sensors]
    return run_streamed(argv, timeout=args.flash_timeout + 60, log_tag="flash", prefix="    | ").returncode == 0


# ----------------------------------------------------------------------- main
def main():
    _default_distro = detect_default_distro()
    parser = argparse.ArgumentParser(
        description="Linorobot2 1-Click Bringup Pipeline (config, firmware, flash, topics, bringup, SLAM, Nav2, map)")
    parser.add_argument("--robot", default=None,
                        help=f"Robot to bring up -> <config dir>/<robot>_config.yaml (default: {DEFAULT_ROBOT})")
    parser.add_argument("--controller", "--target", dest="controller", default=None,
                        help="Override the robot's base controller / PlatformIO env (e.g. pico2, gendrv)")
    parser.add_argument("--distro", default=_default_distro, choices=["jazzy", "lyrical", "rolling", "auto"],
                        help="ROS 2 distribution")
    parser.add_argument("--mode", default="fake", choices=["fake", "auto", "real"], help="Bringup mode (default: fake)")
    parser.add_argument("--firmware", default="auto", choices=["auto", "build", "prebuilt"],
                        help="Where the image comes from: a local `pio run` (build), the release image "
                             "for this env (prebuilt), or build-if-PlatformIO-is-here (auto, default)")
    parser.add_argument("--goal-x", type=float, default=3.0, help="Nav2 goal X behind the obstacle wall (m)")
    parser.add_argument("--goal-y", type=float, default=0.0, help="Nav2 goal Y (m)")
    parser.add_argument("--map-output", default=os.path.join(REPO_ROOT, "maps", "one_click_map"),
                        help="Output path prefix for the map saver")
    parser.add_argument("--no-nav2", action="store_true", help="Skip Nav2 (run SLAM only)")
    parser.add_argument("--flash", action="store_true",
                        help="Write the application even when the probe says the board already runs this build.")
    # Auto-update, ON by default: a stale board is brought to this tree's build
    # without being asked. Off leaves the board strictly alone -- the setting
    # for an assembled robot in the field. Neither touches the env block, which
    # is written on its own terms, and neither suppresses the blank-board install.
    parser.add_argument("--auto-update", dest="auto_update", action="store_true", default=True,
                        help="Update the firmware when the probe reports it stale (default: on).")
    parser.add_argument("--no-auto-update", dest="auto_update", action="store_false",
                        help="Never write the application on account of a stale verdict; --flash still overrides.")
    parser.add_argument("--skip-build", action="store_true", help="Flash what is already built; do not run pio")
    parser.add_argument("--skip-flash", action="store_true", help="Do not touch the board at all")
    parser.add_argument("--skip-mcu-check", action="store_true",
                        help="Flash even when the USB bus says the board is different silicon "
                             "than the config builds for")
    parser.add_argument("--explore-sec", type=int, default=15, help="Seconds to simulate mapping movement")
    parser.add_argument("--drive-test", dest="drive_test", action="store_true", default=True,
                        help="Run the six-manoeuvre drive suite after the topic gate (default: on)")
    parser.add_argument("--no-drive-test", dest="drive_test", action="store_false",
                        help="Skip the drive suite")
    # The stack STAYS UP. Pressing Start 1-Click is how a person gets a running
    # robot; tearing bringup, SLAM and Nav2 down the instant the pipeline
    # finished handed them one that had just been switched off, with no way to
    # drive it and nothing on /scan. The pipeline exits, its children keep
    # running, and scripts/robot_stack.py remembers them so the cockpit's Stop
    # buttons still have something to signal.
    #
    # --shutdown-when-done is for automation: a bench run that leaves a stack
    # behind floods the DDS domain for whatever runs next.
    parser.add_argument("--shutdown-when-done", dest="keep_running",
                        action="store_false", default=True,
                        help="Stop bringup, SLAM and Nav2 when the pipeline finishes "
                             "(default: leave them running until they are stopped)")
    parser.add_argument("--build-timeout", type=int, default=900,
                        help="Seconds allowed for the PlatformIO build (a first build downloads the toolchain)")
    parser.add_argument("--goal-round-trips", type=int, default=4,
                        help="drive to the goal behind the wall and back home this many times "
                             "(default 4); every leg must arrive and plan around the wall. "
                             "0 = a single one-way goal.")
    parser.add_argument("--no-pose-reset", dest="pose_reset", action="store_false",
                        help="do not return the simulated robot to the origin between the "
                             "drive suite and SLAM (fake mode only; a real base is never touched)")
    parser.add_argument("--require-goal", action="store_true",
                        help="the Nav2 goal must actually be reached -- judged by the "
                             "displacement from the goal pose, with Nav2's error_code "
                             "reported when it is not. A verified plan is not enough.")
    parser.add_argument("--goal-tolerance", type=float, default=0.30,
                        help="metres from the goal pose that count as reached (--require-goal)")
    parser.add_argument("--goal-timeout", type=int, default=25,
                        help="seconds the Nav2 goal test waits. 25 suits the default check, "
                             "which asks whether the planner routed around the wall. "
                             "--require-goal needs far more: the goal sits BEHIND the wall "
                             "(x=2, y=-1.5..1.5), so the path around it is 7-8 m, or ~30 s "
                             "of driving at the 0.26 m/s these configs cap at, before any "
                             "rotation or recovery. Measured at 45 s: 899 commands at a full "
                             "0.260 m/s, the base tracking at 0.262, and still short. Use 120.")
    parser.add_argument("--flash-timeout", type=int, default=600,
                        help="Seconds allowed for the whole flash, including every recovery stage")
    parser.add_argument("--flash-attempt-timeout", type=int, default=90,
                        help="Seconds allowed for a single upload attempt inside the flasher")
    args = parser.parse_args()
    if args.distro == "auto":
        args.distro = _default_distro


    # A bare module is a rule, not a file (gen_bare_config.py), so its config is
    # regenerated on every run. The cells had been testing bare_*_config.yaml
    # files written by a release build two days earlier: 0.152 m wheels, inverted
    # even-numbered motors, LED -1 and a different Nav2 template than the one
    # every other default carries -- while the repo said otherwise.
    bare_mcu = re.fullmatch(r"bare_([a-z0-9]+)", args.robot or "")
    if bare_mcu and bare_mcu.group(1) in gen_bare_config.BOARDS:
        bare_path = os.path.join(CONFIG_DIR, f"{args.robot}_config.yaml")
        with open(bare_path, "w") as fh:
            yaml.safe_dump(gen_bare_config.bare_config(bare_mcu.group(1)), fh, sort_keys=False)
        print(f"[0/6] [CONFIG] {os.path.basename(bare_path)} regenerated from the bare rule "
              f"(one default chassis, every sensor faked, LED on).")

    params_path = select_robot_config(args.robot, args.controller)
    with open(params_path, "r") as f:
        params = yaml.safe_load(f) or {}
    controller_cfg = params.get("base_controller") or {}
    if not controller_cfg:
        raise SystemExit(f"{os.path.basename(params_path)} has no base_controller: block. "
                         "Run scripts/migrate_config_schema.py to convert it.")
    robot_name = params.get("robot", {}).get("name") or DEFAULT_ROBOT
    controller = args.controller or controller_cfg.get("name") or "pico2"
    is_real = (args.mode == "real") or (args.mode == "auto" and controller == "gendrv")
    has_lidar = bool(controller_cfg.get("lidar", {}).get("model")) or \
        controller_cfg.get("sensors", {}).get("use_fake_ld19", False)

    # On a real base the sensors are soldered to the board and named in the
    # config, so their topics are evidence, not options. The firmware probes the
    # I2C bus at boot and adopts the drivers for what answered (i2cProbeSelect),
    # which is exactly why a silent topic has to fail: without this the run went
    # green with a dead magnetometer, because every auxiliary topic was optional
    # everywhere. In FAKE mode the list stays empty -- there is no chip to be
    # silent about.
    required_aux = sensor_topics(controller_cfg) if is_real else []

    print("==================================================================")
    print(f"🚀 Linorobot2 Cockpit 1-Click Pipeline [robot: {robot_name}, controller: {controller}, "
          f"distro: {args.distro}, mode: {'REAL' if is_real else 'FAKE'}]")
    print(f"   Config: {params_path}")
    if required_aux:
        fitted = controller_cfg.get("sensors", {})
        roster = ", ".join(f"{k}={fitted[k]}" for k in ("imu", "mag", "current", "env")
                           if fitted.get(k))
        print(f"   Sensors (must publish): {roster}")
        print(f"   Required topics: {', '.join(required_aux)}")
    print("   Sequence: Config -> Firmware -> Probe -> Flash -> Bringup -> Topics -> SLAM -> Nav2 -> Map")
    print("==================================================================")
    os.makedirs(os.path.dirname(args.map_output), exist_ok=True)

    # Step 1: the firmware header (the compile-time fallback for the env block).
    # --distro resolves `stamped_cmd_vel: auto`; the launcher and the goal test
    # get the same --distro so build and run agree on one /cmd_vel type.
    print(f"\n[1/6] [CONFIG] Generating firmware header for base controller '{controller}'...")
    res = subprocess.run([sys.executable, os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py"),
                          "--params", params_path, "--controller", controller, "--distro", args.distro],
                         capture_output=True, text=True)
    if res.returncode != 0:
        print(f"❌ Failed to generate firmware header: {res.stderr}")
        return 1
    print(f"  ✅ Firmware header generated for '{controller}'.")

    # The PlatformIO env comes from the base_controller block ONLY while that
    # block still describes the controller being run. `pio_env:`/`mcu:` are
    # properties of a named controller, not of the file, so an explicit
    # --controller that differs must not inherit them.
    #
    # It used to inherit them unconditionally, and the result contradicted
    # itself in the log: running `--controller pico` against pico2_mecanum
    # (whose block says `mcu: pico2`) produced
    #
    #   [2/6] [FIRMWARE] Image source for pico2: prebuilt release image .../pico2
    #   ❌ [MCU MISMATCH] 'pico' builds for RP2350, but the board is RP2040
    #
    # -- 'pico' builds for RP2040, of course. The env had quietly stayed pico2,
    # so an RP2350 image was queued for an RP2040 board and only the bus check
    # stopped it. Unreachable from the UI until it began sending the controller
    # the user actually sees; latent, not harmless.
    cfg_controller_name = (controller_cfg.get("name") or "").strip()
    if controller == cfg_controller_name:
        pio_env = controller_cfg.get("pio_env") or controller_cfg.get("mcu") or controller
    else:
        pio_env = controller
    if controller == "gendrv":
        pio_env = "esp32"
    pio_env = resolve_pio_env(pio_env, args.distro)
    # The config may name the board by its udev by-id path, which is the only
    # name that follows it across reboots and plug order -- two Picos on one
    # bench swapped ttyACM numbers overnight and the flash went at the other
    # board. Resolve it once, here, so every step below gets the real node.
    configured_port = controller_cfg.get("serial_port", "/dev/ttyACM0")
    serial_port = mcu_identity.resolve_port(configured_port)
    if serial_port != configured_port:
        print(f"  → {configured_port} -> {serial_port}")
    baudrate = controller_cfg.get("baudrate", 921600)

    # Step 2: what the board needs, before anything is written to it. Three
    # answers, three costs: up to date -> nothing; config changed -> the 4 KB
    # env block; different build -> the image (auto-update) or a warning.
    source, prebuilt_dir = "build", None
    board = None
    if not args.skip_flash:
        source, prebuilt_dir = firmware_source(args.firmware, pio_env, args.distro)
        print(f"\n[2/6] [FIRMWARE] Image source for {pio_env}: "
              + ("local build (PlatformIO)" if source == "build" else f"prebuilt release image {prebuilt_dir}"))
        if not os.path.exists(serial_port) and not sys.platform.startswith("linux"):
            print(f"  ❌ {serial_port} is not on this machine, and this is not Linux; the robot "
                  f"computer must be a Linux machine with the board plugged in.")
            return 1
        # Does the board plugged in agree with the config we are about to flash?
        # The controller comes from the robot config (AGENTS.md ss3), and nothing
        # downstream ever looked at what is actually on the bus -- so a run with
        # the pico2 robot selected happily tried to write `--family rp2350-arm-s`
        # onto a bare RP2040, having printed "Auto-Detected MCU: Raspberry Pi
        # Pico (RP2040)" in the UI moments earlier. Only decisive evidence stops
        # a run: an RP2 names its own silicon in its vid/pid, but a classic
        # ESP32 answers through a CP2102/CH340/FTDI that says nothing about the
        # chip behind it, so that case must never block. See scripts/mcu_identity.py.
        if not args.skip_mcu_check:
            expected = mcu_identity.env_family(pio_env)
            family, chip, decisive = mcu_identity.identify_target(serial_port)
            if mcu_identity.mismatch(expected, family, decisive):
                exp_label = mcu_identity.FAMILY_LABEL.get(expected, expected)
                got_label = mcu_identity.FAMILY_LABEL.get(family, family)
                print(f"\n{'=' * 80}")
                print(f"❌ [MCU MISMATCH] '{controller}' builds for {exp_label}, "
                      f"but the board on the bus is {got_label}.")
                print(f"{'=' * 80}")
                print(f"   Detected: {chip}")
                print(f"   Requested: robot config '{os.path.basename(params_path)}' "
                      f"-> base_controller '{controller}' (PlatformIO env '{pio_env}')")
                print("")
                print("   Nothing has been written to the board. Flashing an image built for")
                print(f"   {exp_label} onto {got_label} silicon is never what you meant.")
                print("")
                print("   Fix it either way round:")
                print(f"     * pick the robot whose base_controller matches the board, or")
                print(f"     * set base_controller.name in the config to the {got_label} env,")
                print("       or plug in the board this robot is written for.")
                print("   --skip-mcu-check runs anyway, if you know better than the bus.")
                print(f"{'=' * 80}")
                return 1
            if family and decisive:
                print(f"  ✅ Board on the bus ({chip}) matches '{controller}'.")

        print(f"\n[2/6] [PROBE] Asking {serial_port} what it is already running...")
        board = probe_board(pio_env, serial_port, baudrate, params_path, app="base",
                            prebuilt_dir=prebuilt_dir)
        for line in (board.get("_human") or "").splitlines():
            print(f"    {line}")

    # "absent" belongs here with "no_firmware". Both mean the probe cannot
    # account for what is on the board, and the difference between them is only
    # WHY -- no application answered, or nothing was on the bus to answer. In
    # neither case may the run proceed as though the board were already correct:
    # that is how a stale env block survives a release test and the emulator
    # starts the run parked against a wall.
    #
    # If the board really is unplugged the flash fails and the pipeline halts
    # with that as the reason, which is the honest outcome. Silently testing an
    # unknown board is not.
    blank_board = bool(board and board.get("verdict") in ("no_firmware", "absent"))
    stale_board = bool(board and board.get("verdict") in ("stale", "unknown") and not board.get("probe_failed"))
    auto_updating = bool(args.auto_update and stale_board)
    want_firmware = (args.flash or blank_board or auto_updating) and not args.skip_flash
    # Every run writes the env. It used to go only when the probe called it stale,
    # which compares against what THIS host recorded -- so a board carrying an env
    # from an older config, another host, or a --skip-flash run kept it, and the
    # run silently tested a description nobody had chosen. That is exactly how a
    # GenDrv leg "passed" with fake_ld19 off: it inherited an older env with the
    # emulator on. The block is 4 KB and the application image is untouched, so
    # the write is cheaper than the doubt.
    want_env = bool(board) and not args.skip_flash

    if blank_board and not args.flash:
        print("  → No application is running on the board (BOOTSEL / nothing installed), "
              "so this run will install one.")
    if auto_updating:
        what = ("did not identify itself" if board.get("verdict") == "unknown"
                else f"is running {board['installed'].get('git')}"
                     + (f" ({board['installed'].get('distro')})" if board["installed"].get("distro") else ""))
        print(f"  → Auto-update is on and the board {what}; this run will install "
              f"{board['local'].get('git')} ({board['local'].get('distro')}). "
              f"Pass --no-auto-update to run against the board as it is.")
    if args.auto_update and board and board.get("probe_failed"):
        print("  ⚠️  Auto-update is on but the probe did not run, so the board's firmware is "
              "left alone — a failed check is not evidence that the image is stale. "
              "Pass --flash to update it anyway.")
    if board and board.get("firmware_differs") and not want_firmware and not board.get("probe_failed"):
        print(f"  ⚠️  The board is not running this build "
              f"({board['installed'].get('git') or 'unidentified'} vs {board['local'].get('git')}). "
              f"Nothing will be written to the application because auto-update is off. "
              f"Pass --flash to update it.")

    if want_firmware and source == "build" and not args.skip_build:
        print(f"\n[2/6] [BUILD] Compiling firmware for '{controller}' (pio_env: {pio_env})...")
        build_res = run_streamed(["bash", "-c", f"{get_ros_env(args.distro)} && pio run -d "
                                  f"{os.path.join(REPO_ROOT, 'firmware')} -e {pio_env}"],
                                 timeout=args.build_timeout, log_tag="build", prefix="    | ")
        if build_res.returncode != 0:
            if build_res.returncode == 124:
                print(f"❌ Firmware build timed out after {args.build_timeout}s "
                      f"(a first build downloads the toolchain; raise --build-timeout).")
            print(f"❌ Failed to build firmware for '{controller}' ({pio_env}). See logs/build.log.")
            return 1
        print(f"  ✅ Firmware built for {controller} ({pio_env}).")
    elif want_firmware and source == "build":
        print("\n[2/6] [BUILD] Skipped per --skip-build; flashing what is already built.")
    elif want_firmware:
        print(f"\n[2/6] [BUILD] Not needed: flashing the prebuilt release image.")
    else:
        print("\n[2/6] [BUILD] Not building: the board's firmware is not being updated.")

    bg_processes = []
    # The three that make a running robot. Tracked apart from the short-lived
    # drive step, which is always stopped where it is started.
    stack_processes = []
    failures = []   # steps that must not abort the run but must not read as success either
    try:
        if want_firmware:
            why = ("nothing answered on the bus when the board was probed"
                   if board.get("verdict") == "absent"
                   else "no application was running on the board" if blank_board
                   else "auto-update: the board is not running this build" if auto_updating
                   else "requested with --flash")
            print(f"\n[3/6] [FLASH] Updating the firmware on '{controller}' ({serial_port}) — {why}.")
            release_serial_port(serial_port)
            if not flash_firmware(pio_env, serial_port, baudrate, params_path, controller,
                                  source, prebuilt_dir, args):
                print("\n==================================================================")
                print(f"❌ [FLASH FAILED] Microcontroller firmware flash failed for '{controller}'!")
                print("⛔ HALTING PIPELINE: not proceeding to bringup. See logs/flash.log.")
                print("==================================================================")
                return 1
            print(f"  ✅ Firmware flashed and verified for {controller}.")
        elif want_env:
            why = ("the config or the selected application changed" if board.get("verdict") == "env_stale"
                   else "this machine has no record of the env block on the board"
                   if board.get("needs_env_write")
                   else "every run writes it, so the board cannot be running an env nobody chose")
            print(f"\n[3/6] [ENV] Writing the env block only — {why}. The firmware is not touched.")
            release_serial_port(serial_port)
            if not write_env_only(pio_env, serial_port, baudrate, params_path, controller,
                                  timeout=args.flash_timeout, sensors=sensors_for_mode(args.mode)):
                print("  ⚠️  The env block could not be written; the board keeps the one it has.")
                failures.append("env block write")
            else:
                print("  ✅ env block written.")
        elif not args.skip_flash:
            print("\n[3/6] [FLASH] The application is current; the env block was rewritten anyway.")
        else:
            print("\n[3/6] [FLASH] Skipping firmware flash per --skip-flash.")
            print("  ⚠️  --skip-flash also skips the env block, so this run tests whatever")
            print("      description the board is already carrying -- possibly written by an")
            print("      older config, another host, or another run. A GenDrv leg passed this")
            print("      way with fake_ld19 off, inheriting an env that had the emulator on.")
            print("      A release leg must not use it.")

        # Step 4: bringup and the topic gate
        print(f"\n[4/6] [BRINGUP] Launching the bringup stack (controller={controller}, distro={args.distro})...")
        bringup_cmd = (f"ros2 launch linorobot2_cockpit bringup.launch.py controller:={controller} "
                       f"distro:={args.distro} robot:={robot_name} config_file:={params_path}")
        bg_processes.append(launch_bg(bringup_cmd, log_tag="bringup", distro=args.distro))
        stack_processes.append(("bringup", bg_processes[-1]))
        # A serial board is already enumerated when the agent starts, so 30 s is
        # generous. A udp4 board has not even joined the network yet: it boots,
        # associates, takes a DHCP lease and only then finds the agent, and the
        # LiDAR UDP client connects later still. Measured on a NodeMCU over
        # Wi-Fi, from the ldlidar server binding 8889 to "ldlidar communication
        # is normal": 31.8 s -- so the audit ran, found /scan with no messages
        # and aborted the run, while /odom and /imu/data were already at 48 Hz
        # and the scan arrived seconds after everything was torn down. Nothing
        # was wrong with the robot; the gate was tuned for a cable.
        transport = str(controller_cfg.get("transport", "serial") or "serial").lower()
        handshake_wait = 30 if transport.startswith("serial") else 120
        print(f"  Waiting for the micro-ROS agent handshake and /odom/unfiltered"
              f" (transport={transport}, up to {handshake_wait} s)...")
        if not wait_for_topic("/odom/unfiltered", timeout_sec=handshake_wait,
                              require_publisher=True, distro=args.distro):
            print(f"  ⚠️ /odom/unfiltered publisher not detected within {handshake_wait} s, auditing topics...")
        else:
            print("  ✅ micro-ROS connected (/odom/unfiltered has a publisher).")
        time.sleep(2.0)

        # The scan is the LAST thing to arrive, and on udp4 it is not close.
        # /odom and /imu/data come from the micro-ROS session, which is up as
        # soon as the board finds the agent; the LiDAR is a second UDP client
        # that has to connect to the ldlidar server afterwards. Measured on a
        # NodeMCU over Wi-Fi: 31.8 s from the server binding port 8889 to
        # "ldlidar communication is normal".
        #
        # Lengthening the handshake gate did NOT fix this -- /odom came up in
        # seconds, the handshake passed, and the audit still ran into a /scan
        # that had no publisher yet, reporting "NO DATA (0 msgs received in
        # 10.0s)" and aborting a run whose robot was entirely healthy. The scan
        # needs its own wait, because it is not what the handshake measures.
        if has_lidar:
            # WHO publishes /scan decides the wait, not which transport micro-ROS
            # happens to use. This keyed on `transport` and cost the GenDrv's serial
            # leg on both distros: micro-ROS on a cable, but the scan produced by the
            # BOARD's fake_ld19 out GPIO 4 into a second USB bridge, where the real
            # ldlidar driver gives the port ~3 s, dies, and respawns every 2 s while
            # the board is still rebooting from the flash this run just did. 15 s is
            # not enough for that, and the 90 s the udp path gets is not a property of
            # Wi-Fi -- it is a property of the board being the source.
            #
            # The host's virtual room is the only fast case: fake_laser_node publishes
            # as soon as it starts. Mirror bringup.launch.py's own choice of when it
            # stands in, so the two cannot drift apart.
            lidar_cfg = controller_cfg.get("lidar", {}) or {}
            lidar_mode = str(lidar_cfg.get("comm_mode", "serial") or "serial").lower()
            lidar_port_cfg = lidar_cfg.get("serial_port", "/dev/ttyUSB1")
            # Same order as bringup.launch.py: udp/udp_server is the real driver in
            # server mode, whoever produces the frames, and is decided FIRST; only
            # then can the host room stand in for an absent serial port.
            host_room = (lidar_mode not in ("udp", "udp_server")
                         and controller_cfg.get("sensors", {}).get("use_fake_ld19", False)
                         and (lidar_mode != "serial" or not os.path.exists(lidar_port_cfg)))
            scan_wait = 15 if host_room else 90
            print(f"  Waiting for the first /scan (up to {scan_wait} s)...")
            if wait_for_topic("/scan", timeout_sec=scan_wait, distro=args.distro,
                              require_message="header.frame_id"):
                print("  ✅ /scan is publishing.")
            else:
                print(f"  ⚠️ no /scan within {scan_wait} s — the audit below will say what is missing.")

        print("  [CHECK TOPICS] Verifying ROS 2 topic payloads and publish rates...")
        verify_flag = "" if has_lidar else " --no-scan"
        if required_aux:
            verify_flag += " --require " + ",".join(required_aux)
        verify_res = run_ros(f"python3 {os.path.join(REPO_ROOT, 'scripts', 'verify_topics.py')}{verify_flag}",
                             timeout=25, distro=args.distro)
        if verify_res.stdout:
            print(verify_res.stdout)
        if verify_res.returncode != 0:
            if verify_res.stderr:
                print(f"  STDERR:\n{verify_res.stderr}")
            print("❌ Topic verification failed! Aborting before SLAM/Nav2. See logs/bringup.log.")
            return 1
        print("  ✅ Topic verification passed.")

        # Step 4.5: the drive test. Rates prove the board TALKS; only driving
        # proves it MOVES, and moves the way it was told -- the FakeEncoder
        # invert and the PID windup each shipped perfect 50 Hz topics on a base
        # that spun in place or pinned a rail. The six manoeuvres run every pass
        # unless --no-drive-test, and a failure is recorded but not fatal: a base
        # that talks but drives wrong is worth knowing about without throwing the
        # map away. See scripts/drive_suite.py and docs memory "always flash and
        # drive".
        if args.drive_test:
            print("\n[4.5/6] [DRIVE] Six manoeuvres, checked against odometry...")
            stamped_now = wants_stamped_cmd_vel(args.distro, controller_cfg, params)
            tname = "geometry_msgs/msg/TwistStamped" if stamped_now else "geometry_msgs/msg/Twist"
            drive_res = run_ros(f"python3 {os.path.join(REPO_ROOT, 'scripts', 'drive_suite.py')} {tname}",
                                timeout=90, distro=args.distro)
            if drive_res.stdout:
                print(drive_res.stdout)
            if drive_res.returncode == 0:
                print("  ✅ All six manoeuvres correct.")
            else:
                print(f"  ⚠️ Drive suite returned {drive_res.returncode} — see the verdict above.")
                failures.append(f"Drive suite: exit {drive_res.returncode}")

        # Step 4.7: put the simulated robot back at the origin, with a clean EKF.
        #
        # The flash zeroes the pose, and the six manoeuvres then move it inside
        # that same micro-ROS session: two arcs do not cancel, and the residual
        # was measured at 0.46 m on the bare ESP32 and 0.48 m on an RP2350. SLAM
        # then anchors its map wherever the robot happens to be, and a goal at
        # fixed coordinates is either already under the robot (a vacuous pass)
        # or on the far side of the wall (the documented wedge).
        #
        # The firmware resets a simulated pose on every NEW session
        # (createEntities). Ending only the agent gets that session -- and was
        # measured to wreck the EKF: robot_localization keeps predicting through
        # the ~12 s the board takes to reconnect (x ran to -5.5 m on an RP2040
        # from a stale acceleration state), then wobbles for 15 s after the pose
        # snaps back, and SLAM started its map in the middle of that. So the
        # WHOLE bringup is restarted: agent, EKF, state publisher, LiDAR driver.
        # The EKF then meets its first measurement at the origin and there is
        # nothing to settle. Real robots keep their odometry across a restart
        # and are not touched.
        if not is_real and args.pose_reset and not args.skip_flash:
            print("\n[4.7/6] [POSE] Restarting the bringup so the simulated robot starts from the origin...")
            before_raw, before_ekf = _odom_xy(args.distro), _ekf_xy(args.distro)
            fmt = lambda q: "(%.3f, %.3f)" % q if q else "unknown"
            print(f"  after the drive suite: base odometry {fmt(before_raw)}, EKF {fmt(before_ekf)}")
            # The drive suite proved the BASE moved (/odom/unfiltered). Nav2 never
            # reads that topic: it steers by the EKF's /odom and the TF it
            # publishes. An EKF that did not follow the base is the exact fault
            # a goal then reports as "Failed to make progress" with the robot
            # visibly driving -- measured on the GenDrv, EKF pinned at (0, 0)
            # while the base drove 4 m. Catch it here, where the cause is plain.
            if before_raw and before_ekf and math.hypot(*before_raw) > 0.2 \
                    and math.hypot(*before_ekf) < 0.05:
                print(f"  ❌ the EKF did not follow the base: /odom/unfiltered moved "
                      f"{math.hypot(*before_raw):.2f} m, /odom stayed at {fmt(before_ekf)}. "
                      f"Nav2 steers by /odom, so every goal will fail. Check the ekf block "
                      f"(odom0_config, frames) before anything downstream.")
                failures.append("EKF does not follow /odom/unfiltered")
            # Let the base come to rest first. The drive suite ends with a stop,
            # but the simulated wheels decelerate through their acceleration
            # clamp; a new session that zeroes the pose while they still coast
            # leaves the robot a few cm out (measured 0.079 m on the GenDrv) and
            # the origin check below then argues with a robot that is fine.
            t_still = time.time()
            while time.time() - t_still < 10:
                v = _odom_speed(args.distro)
                if v is not None and v < 0.01:
                    break
                time.sleep(0.5)
            bringup_proc = next((proc for tag, proc in stack_processes if tag == "bringup"), None)
            if bringup_proc is None:
                print("  ⚠️ no bringup process to restart; the pose keeps the drive suite's residual.")
            else:
                stop_bg(bringup_proc)
                stack_processes[:] = [(t, q) for t, q in stack_processes if q is not bringup_proc]
                bg_processes[:] = [q for q in bg_processes if q is not bringup_proc]
                # The new session is only new once the old publishers are gone.
                t_gone = time.time()
                while time.time() - t_gone < 20 and wait_for_topic("/odom/unfiltered", timeout_sec=1,
                                                                    require_publisher=True,
                                                                    distro=args.distro):
                    time.sleep(1.0)
                bg_processes.append(launch_bg(bringup_cmd, log_tag="bringup2", distro=args.distro))
                stack_processes.append(("bringup", bg_processes[-1]))
                if not wait_for_topic("/odom/unfiltered", timeout_sec=handshake_wait,
                                      require_publisher=True, distro=args.distro) \
                        or not wait_for_topic("/odom", timeout_sec=30, require_publisher=True,
                                              distro=args.distro):
                    print(f"  ⚠️ the bringup did not come back within {handshake_wait} s; the pose is unverified.")
                    failures.append("pose reset: bringup did not come back")
                else:
                    # Settled: base and EKF both at the origin, and staying there.
                    after_raw = after_ekf = None
                    t_settle = time.time()
                    while time.time() - t_settle < 30:
                        after_raw, after_ekf = _odom_xy(args.distro), _ekf_xy(args.distro)
                        if after_raw and after_ekf and max(abs(after_raw[0]), abs(after_raw[1]),
                                                           abs(after_ekf[0]), abs(after_ekf[1])) < ORIGIN_TOL:
                            break
                        time.sleep(1.0)
                    if after_raw and after_ekf and max(abs(after_raw[0]), abs(after_raw[1]),
                                                       abs(after_ekf[0]), abs(after_ekf[1])) < ORIGIN_TOL:
                        print(f"  ✅ pose {fmt(before_raw)} -> base {fmt(after_raw)}, EKF {fmt(after_ekf)}: "
                              f"back at the origin, EKF fresh.")
                    else:
                        print(f"  ⚠️ pose {fmt(before_raw)} -> base {fmt(after_raw)}, EKF {fmt(after_ekf)}: "
                              f"NOT at the origin. Is this really a simulated base? A real one keeps "
                              f"its odometry.")
                        failures.append("pose reset: robot not at the origin")
                    # The scan has to come back too before SLAM is asked to map it.
                    if has_lidar:
                        wait_for_topic("/scan", timeout_sec=scan_wait, require_publisher=True,
                                       distro=args.distro, require_message="header.frame_id")

        # Step 5: SLAM. A robot with no scan source has nothing to map.
        if has_lidar:
            print(f"\n[5/6] [SLAM] Launching SLAM Toolbox (distro={args.distro})...")
            bg_processes.append(launch_bg(f"ros2 launch linorobot2_cockpit slam.launch.py config_file:={params_path}",
                                          log_tag="slam", distro=args.distro))
            stack_processes.append(("slam", bg_processes[-1]))
            print("  Waiting for /map...")
            if not wait_for_topic("/map", timeout_sec=40, distro=args.distro,
                                  require_message="info.width"):
                # slam_toolbox is a lifecycle node and its own launch file drives
                # the transitions from a launch event handler. When the configure
                # result event is missed the node sits in `inactive` forever: it
                # logs "Configuring", picks its solver, and then nothing. No map
                # is published, the map frame never exists, and Nav2 fails half a
                # minute later with planner_server unable to transform base_link
                # to map -- which reads as a Nav2 fault.
                #
                # The transition is idempotent and cheap, so ask for it directly
                # rather than give up on a race in somebody's event handler.
                #
                # --no-daemon: the ros2 CLI daemon caches the graph, and it was
                # measured answering "Node not found" for 36 s straight about a
                # slam_toolbox that was active at the time, in the same
                # container where --no-daemon answered "active [3]" at once. A
                # recovery that asks the cache can be told the node is not there.
                print("  ⚠️ no map in 40 s — asking slam_toolbox to activate directly...")
                run_ros("ros2 lifecycle set --no-daemon /slam_toolbox activate", timeout=20,
                        distro=args.distro)
                if wait_for_topic("/map", timeout_sec=30, distro=args.distro,
                                  require_message="info.width"):
                    print("  ✅ /map is publishing (slam_toolbox needed a manual activate).")
                else:
                    print("  ⚠️ still no map — check logs/slam.log for 'Activating'.")
                    failures.append("SLAM: no map was published")
            else:
                print("  ✅ /map is publishing.")
        else:
            print(f"\n[5/6] [SLAM] Skipped — {controller} has no scan source (teleop-only robot).")

        stamped_cmd = wants_stamped_cmd_vel(args.distro, controller_cfg, params)
        if stamped_cmd:
            drive_cmd = ("ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "
                         "'{header: {frame_id: \"base_link\"}, twist: {angular: {z: 0.4}}}'")
        else:
            drive_cmd = "ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.4}}'"

        # Step 6: Nav2, or a plain exploration spin
        if not args.no_nav2 and has_lidar:
            print(f"\n[6/6] [NAV2] Launching Nav2 (distro={args.distro})...")
            bg_processes.append(launch_bg(f"ros2 launch linorobot2_cockpit nav2.launch.py autostart:=true "
                                          f"distro:={args.distro} config_file:={params_path}",
                                          log_tag="nav2", distro=args.distro))
            stack_processes.append(("nav2", bg_processes[-1]))
            print("  Waiting for Nav2 lifecycle activation...")
            nav2_ok, nav2_detail = wait_for_nav2_activation()
            if nav2_ok:
                print("  ✅ Nav2 stack active.")
            else:
                print(f"  ❌ Nav2 did not activate: {nav2_detail}\n     See logs/nav2.log.")
                failures.append(f"Nav2: did not activate ({nav2_detail})")
            cmd_vel_type = "twist_stamped" if stamped_cmd else "twist"
            n_legs = 2 * args.goal_round_trips if args.goal_round_trips else 1
            if args.goal_round_trips:
                print(f"  Nav2 goal behind the obstacle wall ({args.goal_x}, {args.goal_y}) and back "
                      f"home, {args.goal_round_trips} round trips ({n_legs} legs, "
                      f"{args.goal_timeout} s each)...")
            else:
                print(f"  Nav2 goal behind the obstacle wall ({args.goal_x}, {args.goal_y})...")
            goal_args = (f"--goal-x {args.goal_x} --goal-y {args.goal_y} "
                         f"--timeout {args.goal_timeout} --cmd-vel-type {cmd_vel_type} "
                         f"--round-trips {args.goal_round_trips}")
            if args.require_goal or args.goal_round_trips:
                goal_args += f" --require-goal --goal-tolerance {args.goal_tolerance}"
            if nav2_ok:
                test_res = run_ros(f"python3 {os.path.join(REPO_ROOT, 'scripts', 'test_nav2_goal.py')} "
                                   + goal_args, timeout=args.goal_timeout * n_legs + 20 * n_legs + 15,
                                   distro=args.distro)
            else:
                # A goal sent to a stack that never activated is rejected on
                # arrival ("Action server is inactive") and the transcript then
                # reads like a navigation failure. The failure is the lifecycle
                # one above; say so and let the drive suite below place it.
                test_res = subprocess.CompletedProcess(
                    args=[], returncode=3, stderr="",
                    stdout=f"❌ NAV2 GOAL NOT SENT: the stack never activated ({nav2_detail}); "
                           f"a goal would only be rejected. See logs/nav2.log.")
            print(test_res.stdout)
            if test_res.returncode == 0:
                print("  🎉 Nav2 planned around the obstacle wall and executed the motion!")
            else:
                print(f"  ⚠️ Nav2 goal test returned {test_res.returncode}; continuing to the mapping spin...")
                if test_res.stderr and test_res.stderr.strip():
                    print("     --- goal test stderr ---")
                    for line in test_res.stderr.strip().splitlines():
                        print(f"     {line}")
                failures.append(f"Nav2: goal test exit {test_res.returncode}")

                # The goal failed. Ask the base directly, right now, in this
                # stack state: six manoeuvres against odometry. It already ran
                # before SLAM/Nav2, but that was minutes and two lifecycle
                # activations ago, and what a reader needs here is which half is
                # at fault. A base that still does 6/6 after a failed goal says
                # the firmware, the transport, the agent and the cmd_vel contract
                # are all fine and the fault is above them -- planner, costmap,
                # footprint, TF. A base that now fails says the opposite, and the
                # goal failure was a symptom.
                # tname only exists when the earlier drive block ran; derive it
                # here so --no-drive-test cannot turn a Nav2 failure into a
                # NameError inside the diagnostic meant to explain it.
                post_tname = ("geometry_msgs/msg/TwistStamped"
                              if wants_stamped_cmd_vel(args.distro, controller_cfg, params)
                              else "geometry_msgs/msg/Twist")
                print("  [DRIVE] The goal failed -- re-running the six manoeuvres to "
                      "see whether the base is still answering...")
                post_res = run_ros(f"python3 {os.path.join(REPO_ROOT, 'scripts', 'drive_suite.py')} {post_tname}",
                                   timeout=90, distro=args.distro)
                if post_res.stdout:
                    print(post_res.stdout)
                if post_res.returncode == 0:
                    print("  ↳ The base still drives 6/6 after the failed goal: the base, the "
                          "transport and the /cmd_vel contract are fine, so the fault is in "
                          "the navigation layer above them.")
                else:
                    print(f"  ↳ The base ALSO fails the manoeuvres now (exit {post_res.returncode}): "
                          f"the goal failure is a symptom, not the cause. Look at the board and "
                          f"the agent before looking at Nav2.")
                    failures.append(f"Drive suite after the failed goal: exit {post_res.returncode}")
                drive = launch_bg(drive_cmd, log_tag="drive", distro=args.distro)
                bg_processes.append(drive)
                time.sleep(args.explore_sec)
                stop_bg(drive)
        else:
            print("\n[6/6] [EXPLORE] Nav2 skipped (per argument or no LiDAR). Simulating a mapping rotation...")
            drive = launch_bg(drive_cmd, log_tag="drive", distro=args.distro)
            bg_processes.append(drive)
            time.sleep(args.explore_sec)
            stop_bg(drive)

        print(f"\n[MAP] Saving the map to '{args.map_output}'...")
        # save_map_timeout=15: /map is transient-local and the saver has to get
        # its subscription matched inside the window; 2 s is a coin toss on a
        # machine that just brought Nav2 up.
        save_res = run_ros(f"ros2 run nav2_map_server map_saver_cli -f {args.map_output} "
                           f"--ros-args -p save_map_timeout:=15.0", timeout=30, distro=args.distro)
        if save_res.returncode == 0:
            print(f"🎉 MAP SAVED: {args.map_output}.yaml / .pgm")
        else:
            print(f"⚠️ Map saver returned {save_res.returncode}: {save_res.stdout} {save_res.stderr}")
            failures.append(f"Map saver: exit {save_res.returncode}")

        print("\n==================================================================")
        if failures:
            print(f"❌ 1-Click Pipeline finished WITH {len(failures)} FAILED STEP(S):")
            for item in failures:
                print(f"   - {item}")
            print("==================================================================")
            return 1
        print("✅ 1-Click Pipeline Finished Successfully!")
        print("==================================================================")
        return 0
    finally:
        keep = getattr(args, "keep_running", False)
        kept = {id(p) for _, p in stack_processes} if keep else set()
        if keep and stack_processes:
            for tag, proc in stack_processes:
                try:
                    robot_stack.record(tag, proc.pid)
                except Exception as exc:
                    print(f"  (could not record {tag} for the Stop buttons: {exc})")
            print("\nLeaving the robot running:")
            for tag, proc in stack_processes:
                print(f"   {tag:<8} pid {proc.pid}   logs/{tag}.log")
            print("   Stop it from the cockpit (Bringup / SLAM / Nav2 Stop), or:")
            print("   python3 scripts/robot_stack.py --stop")
            print("   Re-run with --shutdown-when-done to tear it down here instead.")
        else:
            print("\nCleaning up background ROS 2 launch processes...")
        for p in bg_processes:
            if id(p) in kept:
                continue
            stop_bg(p)
        for f in open_log_files:
            try:
                f.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
