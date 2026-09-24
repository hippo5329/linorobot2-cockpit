"""Named server-side actions — the commands the UI runs, built HERE, not there.

The first review's leading finding: `/api/exec` executed a shell string composed
in the browser, so anyone holding the access token had an interactive shell on
the robot computer rather than the ability to flash and bring up a robot. The
fix the review asked for is this module: the browser names an ACTION and sends
structured DATA; the server owns the command text.

Every builder here takes a small dict of validated arguments and returns the
exact command the frontend used to compose. ROS sourcing that the browser did
inline with envPrefix() is done with runners.ros_setup_shell(distro) instead --
behaviourally the same (it sources the distro, the micro-ROS and the cockpit
workspaces), and now in one place.

Nothing the browser sends reaches the shell unquoted: every argument is checked
against a small vocabulary (an identifier, an int in range, a known enum, a
device node, a package name) and shell-quoted, so a value can never carry a
`;`, a `$(...)`, or a newline into the command.

`build(action, args)` is the whole public surface. An unknown action, or an
argument that fails validation, raises ValueError -- the endpoint turns that
into a 400, and the command is never run.
"""
from __future__ import annotations

import os
import re
import secrets
import shlex
import sys
import time
from typing import Callable, Dict

from runners import ros_setup_shell

# --------------------------------------------------------------------------
# Prepared commands. Some commands are still built by a dedicated endpoint that
# already owns the context (flashing, from the USB probe and the active config;
# the colcon and apt install lines, from the missing-package scan). Those
# endpoints REGISTER the command they built and hand the browser a one-shot
# handle; the exec endpoint runs the command the handle names. The browser never
# sees or supplies the command text, so it cannot substitute one -- the same
# guarantee the named actions give, for commands whose builder lives elsewhere.
_PREPARED: Dict[str, tuple] = {}   # handle -> (command, expiry)
_PREPARED_TTL = 300.0


def prepare(command: str) -> str:
    _prune_prepared()
    handle = secrets.token_urlsafe(18)
    _PREPARED[handle] = (command, time.monotonic() + _PREPARED_TTL)
    return handle


def claim(handle: str) -> str:
    """The command a handle names, spent on use; None if unknown or expired."""
    if not handle:
        return None
    _prune_prepared()
    entry = _PREPARED.pop(handle, None)
    if entry is None or entry[1] < time.monotonic():
        return None
    return entry[0]


def _prune_prepared() -> None:
    now = time.monotonic()
    for h, (_, expiry) in list(_PREPARED.items()):
        if expiry < now:
            _PREPARED.pop(h, None)

# --------------------------------------------------------------------------
# Argument validation. Each returns a safe string/int, or raises ValueError.
# --------------------------------------------------------------------------
_IDENT_RE = re.compile(r"^[A-Za-z0-9_.:+/@-]+$")   # ros pkg/exec, distro, image tags
_DEVICE_RE = re.compile(r"^/dev/[A-Za-z0-9_./-]+$")
_PKG_RE = re.compile(r"^[a-z0-9][a-z0-9.+-]*$")     # apt / ros package names

# A browser field that was never set serialises as these literal strings; treat
# them as "not given" so a default applies, rather than validating the word.
_EMPTY = (None, "", "undefined", "null", "NaN")


def _ident(value, name: str, default: str = None) -> str:
    s = str(value if value not in _EMPTY else (default if default is not None else ""))
    if default is not None and s == "":
        s = default
    if not _IDENT_RE.match(s):
        raise ValueError(f"{name}: {value!r} is not a plain identifier")
    return s


def _enum(value, name: str, allowed, default: str = None) -> str:
    s = str(value) if value not in _EMPTY else default
    if s not in allowed:
        raise ValueError(f"{name}: {value!r} is not one of {sorted(allowed)}")
    return s


def _int(value, name: str, lo: int, hi: int, default: int = None) -> int:
    try:
        n = int(value) if value not in _EMPTY else default
    except (TypeError, ValueError):
        raise ValueError(f"{name}: {value!r} is not an integer")
    if n is None or not (lo <= n <= hi):
        raise ValueError(f"{name}: {value!r} out of range [{lo},{hi}]")
    return n


def _num(value, name: str, lo: float, hi: float, default: float) -> float:
    try:
        x = float(value) if value not in _EMPTY else default
    except (TypeError, ValueError):
        raise ValueError(f"{name}: {value!r} is not a number")
    if not (lo <= x <= hi):
        raise ValueError(f"{name}: {value!r} out of range [{lo},{hi}]")
    return x


def _device(value, name: str, default: str) -> str:
    s = str(value) if value not in _EMPTY else default
    if not _DEVICE_RE.match(s):
        raise ValueError(f"{name}: {value!r} is not a /dev device node")
    return s


def _path(value, name: str) -> str:
    """A filesystem path the command will hand to a tool. Shell-quoted, and with
    the shell-active characters that have no place in a path refused outright."""
    s = str(value or "").strip()
    if not s or any(c in s for c in ";|&`$\n\r<>*?"):
        raise ValueError(f"{name}: {value!r} is not an acceptable path")
    return s


def _bool(value) -> bool:
    return str(value).lower() in ("1", "true", "yes", "on")


def _packages(value, name: str) -> list:
    """A whitespace/comma list of apt or ROS package names."""
    items = re.split(r"[\s,]+", str(value or "").strip())
    out = [p for p in items if p]
    if not out:
        raise ValueError(f"{name}: no packages given")
    for p in out:
        if not _PKG_RE.match(p):
            raise ValueError(f"{name}: {p!r} is not a package name")
    return out


# --------------------------------------------------------------------------
# The agent. Four engines, two transports -- reproduces agentLaunchCommand().
# --------------------------------------------------------------------------
def _agent_start(a: Dict) -> str:
    engine = _enum(a.get("engine"), "engine", {"native", "docker", "podman", "podman_systemd"}, "native")
    transport = _enum(a.get("transport"), "transport", {"serial", "udp4"}, "serial")
    distro = _ident(a.get("distro"), "distro", "jazzy")
    device = _device(a.get("device"), "device", "/dev/ttyACM0")
    port = _int(a.get("port"), "port", 1, 65535, 8888)
    baud = _int(a.get("baud"), "baud", 1200, 6000000, 921600)

    pre_clean = "" if transport == "udp4" else f"fuser -k -TERM {shlex.quote(device)} 2>/dev/null || true; sleep 0.5; "
    dev_flags = "" if transport == "udp4" else f"--device {shlex.quote(device)}"
    agent_args = f"udp4 --port {port}" if transport == "udp4" else f"serial --dev {shlex.quote(device)} -b {baud}"
    img = f"microros/micro-ros-agent:{distro}"
    mode = "udp4" if transport == "udp4" else "serial"

    if engine == "podman_systemd":
        return pre_clean + " && ".join([
            f'IMG="{img}"',
            f'echo ">>> micro-ROS agent: Podman + systemd user service ({distro})"',
            f'podman pull "$IMG" 2>/dev/null || IMG="microros/micro-ros-agent:rolling"',
            f'podman run -d --replace --name "microros_agent_{mode}" --net=host {dev_flags} "$IMG" {agent_args}',
            'mkdir -p "$HOME/.config/systemd/user"',
            f'podman generate systemd --new --name "microros_agent_{mode}" > "$HOME/.config/systemd/user/microros-agent.service" 2>/dev/null || true',
            'systemctl --user daemon-reload 2>/dev/null || true',
            'systemctl --user enable --now microros-agent.service 2>/dev/null || true',
            'loginctl enable-linger "$USER" 2>/dev/null || true',
            'echo ">>> micro-ROS agent running as persistent systemd user service: microros-agent.service"',
        ])
    if engine == "podman":
        return f'{pre_clean}podman run --rm --replace -it --name "uros_agent_{mode}" --net=host --privileged -v /dev:/dev {dev_flags} -e ROS_DOMAIN_ID=0 {img} {agent_args}'
    if engine == "docker":
        return f'{pre_clean}docker run --rm --net=host --privileged -v /dev:/dev {dev_flags} -e ROS_DOMAIN_ID=0 {img} {agent_args}'
    # native
    run_line = f"ros2 run micro_ros_agent micro_ros_agent udp4 -p {port}" if transport == "udp4" \
        else f"ros2 run micro_ros_agent micro_ros_agent serial --dev {shlex.quote(device)} -b {baud}"
    return f"{ros_setup_shell(distro)}; [ -f ~/uros_ws/install/setup.bash ] && source ~/uros_ws/install/setup.bash; {pre_clean}{run_line}"


def _agent_prepare(a: Dict) -> str:
    """find-or-build the agent (native) or pull the image (container).
    reproduces findOrBuildAgentCommand(); the registry host, if any, is a
    validated hostname the operator typed, never anything hardcoded."""
    engine = _enum(a.get("engine"), "engine", {"native", "docker", "podman", "podman_systemd"}, "native")
    distro = _ident(a.get("distro"), "distro", "jazzy")
    img = f"microros/micro-ros-agent:{distro}"
    if engine in ("docker", "podman", "podman_systemd"):
        bin_ = "docker" if engine == "docker" else "podman"
        reg = a.get("registry", "")
        pull_block = (f'echo ">>> Pulling $IMG from Docker Hub directly..."; '
                      f'{bin_} pull "$IMG" 2>/dev/null || {{ echo ">>> no \'$IMG\' tag on Docker Hub, trying \':rolling\'"; '
                      f'IMG="microros/micro-ros-agent:rolling"; {bin_} pull "$IMG" 2>/dev/null || true; }}; ')
        if reg:
            host = _ident(reg, "registry")
            pull_block = (
                f'REG_PULLED=0; for reg in "{host}"; do '
                f'if curl -fsSL -m 2 "https://$reg/v2/" >/dev/null 2>&1 || curl -fsSL -m 2 "http://$reg/v2/" >/dev/null 2>&1; then '
                f'echo ">>> Container registry active at $reg. Pulling $reg/$IMG..."; '
                f'if {bin_} pull "$reg/$IMG" >/dev/null 2>&1; then {bin_} tag "$reg/$IMG" "$IMG"; REG_PULLED=1; break; fi; fi; done; '
                f'if [ "$REG_PULLED" -eq 0 ]; then echo ">>> Pulling $IMG from upstream..."; '
                f'{bin_} pull "$IMG" 2>/dev/null || {{ echo ">>> no \'$IMG\' tag on Docker Hub, trying \':rolling\'"; '
                f'IMG="microros/micro-ros-agent:rolling"; {bin_} pull "$IMG" 2>/dev/null || true; }}; fi; ')
        return (f'echo ">>> micro-ROS agent: using {bin_} container image (skipping build from source)"; '
                f'if ! command -v {bin_} >/dev/null 2>&1; then echo "ERROR: {bin_} is not installed" >&2; exit 1; fi; '
                f'IMG="{img}"; {pull_block}echo AGENT_DOCKER_READY')
    return f"{ros_setup_shell(distro)}; " + "; ".join([
        "[ -f ~/uros_ws/install/setup.bash ] && source ~/uros_ws/install/setup.bash",
        ("if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then echo AGENT_FOUND; "
         "else sudo apt-get install -y ros-$ROS_DISTRO-micro-ros-agent >/dev/null 2>&1; "
         "source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null; "
         "if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then echo AGENT_APT_OK; "
         "else mkdir -p ~/uros_ws/src && cd ~/uros_ws/src && "
         "{ [ -d micro_ros_agent ] || git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ROS-Agent.git micro_ros_agent || git clone -b rolling https://github.com/micro-ROS/micro-ROS-Agent.git micro_ros_agent; } && "
         "{ [ -d micro_ros_msgs ] || git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_msgs.git micro_ros_msgs || git clone -b rolling https://github.com/micro-ROS/micro_ros_msgs.git micro_ros_msgs; } && "
         "cd ~/uros_ws && colcon build && echo AGENT_BUILT; fi; fi"),
    ])


# --------------------------------------------------------------------------
# Bringup / SLAM / Nav2 / teleop -- the ROS launches.
# --------------------------------------------------------------------------
def _bringup(a: Dict) -> str:
    distro = _ident(a.get("distro"), "distro", "jazzy")
    launcher = _path(a.get("launcher"), "launcher")
    cfg = _path(a.get("config_path"), "config_path")
    base = _enum(a.get("base"), "base", {"2wd", "4wd", "mecanum", "ackermann"}, "2wd")
    dev = _device(a.get("device"), "device", "/dev/ttyACM0")
    baud = _int(a.get("baud"), "baud", 1200, 6000000, 1500000)
    madgwick = "true" if _bool(a.get("madgwick")) else "false"
    micro_ros = "true" if _bool(a.get("micro_ros")) else "false"
    return (f"{ros_setup_shell(distro)}; ros2 launch {shlex.quote(launcher)} "
            f"config_file:={shlex.quote(cfg)} base:={base} base_serial_port:={shlex.quote(dev)} "
            f"micro_ros_baudrate:={baud} madgwick:={madgwick} micro_ros:={micro_ros}")


# The compose file is docker-compose.YML and has been since the first commit.
# These four call sites said .yaml from 0d789db (the action registry that replaced
# the browser's raw command strings) until 2026-09-24, so every Docker action here
# named a file that does not exist. tests/test_compose_files_exist.py now asserts
# that every `-f` target actions.py names is either tracked or generated.
def _bringup_docker(a: Dict) -> str:
    compose = _compose_resolve(a)
    flags = "--env-file .env -f docker-compose.yml -f devices.generated.yaml"
    d = _path(a.get("docker_dir"), "docker_dir")
    return f"{compose}cd {shlex.quote(d)} && $COMPOSE {flags} up bringup"


def _slam(a: Dict) -> str:
    distro = _ident(a.get("distro"), "distro", "jazzy")
    launcher = _path(a.get("launcher"), "launcher")
    params = a.get("params_file")
    params_arg = f" params_file:={shlex.quote(_path(params, 'params_file'))}" if params else ""
    depth = "true" if _bool(a.get("depth")) else "false"
    return (f"{ros_setup_shell(distro)}; if [ -f {shlex.quote(launcher)} ]; then "
            f"ros2 launch {shlex.quote(launcher)} slam:=true{params_arg} depth_costmap:={depth} distro:={distro} sim:=false; "
            f"else ros2 launch linorobot2_navigation slam.launch.py; fi")


def _nav2(a: Dict) -> str:
    distro = _ident(a.get("distro"), "distro", "jazzy")
    launcher = _path(a.get("launcher"), "launcher")
    custom = a.get("params_file")
    default_params = a.get("default_params")
    params = custom or default_params
    depth = "true" if _bool(a.get("depth")) else "false"
    map_arg = f" map:={shlex.quote(_path(a.get('map'), 'map'))}" if a.get("map") else ""
    custom_arg = f" params_file:={shlex.quote(_path(custom, 'params_file'))}" if custom else ""
    q_launcher = shlex.quote(launcher)
    # Three ways, in order: the console launcher (auto-resolves its own params);
    # plain nav2_bringup with an explicit params file, when one exists; and the
    # linorobot2_navigation fallback. Matches the frontend's original branches.
    branches = (f"if [ -f {q_launcher} ]; then "
                f"ros2 launch {q_launcher}{map_arg}{custom_arg} depth_costmap:={depth} distro:={distro} sim:=false; ")
    if params:
        qp = shlex.quote(_path(params, "params_file"))
        branches += (f"elif [ -f {qp} ]; then "
                     f"ros2 launch nav2_bringup bringup_launch.py{map_arg} params_file:={qp} use_sim_time:=false; ")
    branches += (f"else ros2 launch linorobot2_navigation navigation.launch.py{map_arg}; fi")
    return f"{ros_setup_shell(distro)}; {branches}"


def _distro_stamps_cmd_vel(distro: str) -> bool:
    """Does this distro put TwistStamped on /cmd_vel?

    Asks gen_firmware_header, which is the file that decided
    USE_STAMPED_CMD_VEL when the firmware was built. Two copies of that list
    would disagree on exactly the distro nobody tested.
    """
    scripts = os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))), "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    try:
        from gen_firmware_header import distro_stamps_cmd_vel
    except Exception:
        # Never let a teleop button fail on an import. Unstamped is the older
        # contract and the one jazzy uses; guessing stamped here would break
        # every jazzy robot to save a post-kilted one.
        return False
    return bool(distro_stamps_cmd_vel(distro))


def _teleop(a: Dict) -> str:
    distro = _ident(a.get("distro"), "distro", "jazzy")
    axis_lin = _int(a.get("axis_linear"), "axis_linear", 0, 31, 1)
    scale_lin = _num(a.get("scale_linear"), "scale_linear", 0.0, 100.0, 0.5)
    axis_ang = _int(a.get("axis_angular"), "axis_angular", 0, 31, 0)
    scale_ang = _num(a.get("scale_angular"), "scale_angular", 0.0, 100.0, 1.0)
    tmp = "/tmp/linorobot2_console_joy.yaml"
    # The same contract the firmware was built to. nav2 1.4 (kilted) flipped
    # TwistPublisher to TwistStamped, so kilted and later stamp /cmd_vel and
    # jazzy and older do not; teleop_twist_joy has to match or the base hears
    # nothing. With USE_STAMPED_CMD_VEL the firmware subscribes TwistStamped on
    # /cmd_vel and moves the plain Twist subscriber to /cmd_vel_unstamped, so a
    # plain Twist arrives as the wrong type on the right topic and is dropped
    # in silence -- the joystick moves and the robot does not.
    stamped = _distro_stamps_cmd_vel(distro)
    stamped_line = f"    publish_stamped_twist: {'true' if stamped else 'false'}\n"
    yaml_body = (f"teleop_twist_joy_node:\n  ros__parameters:\n"
                 f"{stamped_line}"
                 f"    axis_linear:\n      x: {axis_lin}\n"
                 f"    scale_linear:\n      x: {scale_lin}\n"
                 f"    axis_angular:\n      yaw: {axis_ang}\n"
                 f"    scale_angular:\n      yaw: {scale_ang}\n")
    write_yaml = f"cat > {tmp} << 'CONSOLE_JOY_EOF'\n{yaml_body}CONSOLE_JOY_EOF"
    return (f"{ros_setup_shell(distro)}; {write_yaml}\n"
            f"(ros2 run joy_linux joy_linux_node & "
            f"ros2 run teleop_twist_joy teleop_node --ros-args --params-file {tmp}; wait)")


def _map_save(a: Dict) -> str:
    distro = _ident(a.get("distro"), "distro", "jazzy")
    maps_dir = _path(a.get("maps_dir"), "maps_dir")
    name = _ident(a.get("name"), "name")
    return (f"{ros_setup_shell(distro)}; mkdir -p {shlex.quote(maps_dir)} && "
            f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(maps_dir + '/' + name)}")


# --------------------------------------------------------------------------
# Docker/compose helpers and installs.
# --------------------------------------------------------------------------
def _compose_resolve(a: Dict) -> str:
    engine = _enum(a.get("engine"), "engine", {"docker", "podman"}, "docker")
    if engine == "podman":
        return 'if command -v podman-compose >/dev/null 2>&1; then COMPOSE="podman-compose"; else COMPOSE="podman compose"; fi; '
    return 'COMPOSE="docker compose"; '


def _docker_service_up(a: Dict) -> str:
    compose = _compose_resolve(a)
    flags = "--env-file .env -f docker-compose.yml -f devices.generated.yaml"
    d = _path(a.get("docker_dir"), "docker_dir")
    service = _ident(a.get("service"), "service")
    return f"{compose}cd {shlex.quote(d)} && DISPLAY=:200 $COMPOSE {flags} up {service}"


def _clone_distro(repo_url: str, target: str, distro: str) -> str:
    """git clone with the same branch fallbacks the frontend used."""
    return (f"[ -d {target} ] || git clone -b {distro} {repo_url} {target} 2>/dev/null || "
            f"git clone -b main {repo_url} {target} 2>/dev/null || "
            f"git clone -b jazzy {repo_url} {target} 2>/dev/null || "
            f"git clone {repo_url} {target}")


def _docker_build(a: Dict) -> str:
    """Build the console's compose image: clone linorobot2, write the .env and
    the device overlay, and run `$COMPOSE build`. The two files are heredocs, so
    the steps are newline-joined (a heredoc terminator must stand alone) with
    `set -e` for fail-fast."""
    distro = _ident(a.get("distro"), "distro", "jazzy")
    workspace = _path(a.get("workspace"), "workspace")
    docker_dir = _path(a.get("docker_dir"), "docker_dir")
    base_image = _ident(a.get("base_image"), "base_image")
    robot_base = _ident(a.get("robot_base"), "robot_base", "2wd")
    laser = _ident(a.get("laser"), "laser", "none") if a.get("laser") else ""
    depth = _ident(a.get("depth"), "depth", "none") if a.get("depth") else ""
    serial_port = _device(a.get("serial_port"), "serial_port", "/dev/ttyACM0")
    domain_id = _int(a.get("domain_id"), "domain_id", 0, 232, 0)
    gpu_id = _int(a.get("gpu_id"), "gpu_id", 0, 64, 0)
    robot_name = _ident(a.get("robot_name"), "robot_name", "linorobot2")
    engine = _enum(a.get("engine"), "engine", {"docker", "podman"}, "docker")

    env_body = (
        f"DOCKER_ROS_DISTRO={distro}\nBASE_IMAGE={base_image}\nROBOT_BASE={robot_base}\n"
        f"LASER_SENSOR={laser}\nDEPTH_SENSOR={depth}\nBASE_SERIAL_PORT={serial_port}\n"
        f"ODOM_TOPIC=/odom\nROBOT_NAME={robot_name}\nROS_DOMAIN_ID={domain_id}\n"
        f"CUSTOM_ROBOT=false\nLAUNCH_EXTRA=false\nLAUNCH_JOYSTICK=false\n"
        f"GPU_ID={gpu_id}\nVIRTUALGL_VER=3.1.4\n")

    device_lines = [f"      - {serial_port}:{serial_port}"]
    if a.get("laser_device"):
        dev = _device(a.get("laser_device"), "laser_device", "/dev/ldlidar")
        device_lines.append(f"      - {dev}:{dev}")
    override_body = "services:\n  bringup:\n    devices:\n" + "\n".join(device_lines) + "\n"

    compose = _compose_resolve({"engine": engine})
    flags = "--env-file .env -f docker-compose.yml -f devices.generated.yaml"
    ws_q = shlex.quote(workspace)
    dir_q = shlex.quote(docker_dir)
    return "set -e\n" + "\n".join([
        f"mkdir -p {ws_q}/src",
        f"cd {ws_q}/src",
        _clone_distro("https://github.com/linorobot/linorobot2", "linorobot2", distro),
        f"mkdir -p {dir_q}",
        f"cat > {dir_q}/.env << 'CONSOLE_DOCKER_ENV_EOF'\n{env_body}CONSOLE_DOCKER_ENV_EOF",
        f"cat > {dir_q}/devices.generated.yaml << 'CONSOLE_DOCKER_OVERRIDE_EOF'\n{override_body}CONSOLE_DOCKER_OVERRIDE_EOF",
        f"cd {dir_q}",
        f"{compose}HOST_UID=$(id -u) HOST_GID=$(id -g) $COMPOSE {flags} build",
    ])


def _docker_down(a: Dict) -> str:
    compose = _compose_resolve(a)
    flags = "--env-file .env -f docker-compose.yml -f devices.generated.yaml"
    d = _path(a.get("docker_dir"), "docker_dir")
    return f"{compose}cd {shlex.quote(d)} && $COMPOSE {flags} down"


def _apt_install(a: Dict) -> str:
    pkgs = _packages(a.get("packages"), "packages")
    return "sudo apt-get update && sudo apt-get install -y " + " ".join(shlex.quote(p) for p in pkgs)


def _ld_node_params(a: Dict, overrides: Dict) -> str:
    base = {
        "product_name": _ident(a.get("product"), "product", "LDLiDAR_LD19"),
        "topic_name": "scan",
        "frame_id": "laser",
        "laser_scan_dir": "true",
        "bins": str(_int(a.get("bins"), "bins", 1, 100000, 456)),
        "enable_angle_crop_func": "false",
        "angle_crop_min": "135.0",
        "angle_crop_max": "225.0",
    }
    base.update(overrides)
    return " ".join(f"-p {k}:={shlex.quote(str(v))}" for k, v in base.items())


def _laser_driver(a: Dict) -> str:
    distro = _ident(a.get("distro"), "distro", "jazzy")
    prefix = ros_setup_shell(distro)
    if not _bool(a.get("is_ld")):
        code = _ident(a.get("code"), "code")
        cmd = f"ros2 launch linorobot2_bringup lasers.launch.py sensor:={code}"
        if a.get("port"):
            dev = _device(a.get("port"), "port", "/dev/ttyUSB0")
            cmd += f" lidar_transport:=serial lidar_serial_port:={shlex.quote(dev)}"
        return f"{prefix}; {cmd}"

    mode = _enum(a.get("mode"), "mode", {"serial", "udp_bridge", "udp_server", "udp_client"}, "serial")
    baud = _int(a.get("baud"), "baud", 1200, 6000000, 230400)
    node = "ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node"

    if mode == "serial":
        dev = _device(a.get("port") or a.get("symlink"), "port", "/dev/ttyUSB0")
        params = _ld_node_params(a, {"comm_mode": "serial", "port_name": dev, "port_baudrate": str(baud)})
        return f"{prefix}; {node} --ros-args {params}"

    if mode == "udp_bridge":
        udp_port = _int(a.get("udp_port"), "udp_port", 1, 65535, 8889)
        bridge = _device(a.get("bridge_path"), "bridge_path", "/dev/lidar_udp_bridge")
        params = _ld_node_params(a, {"comm_mode": "serial", "port_name": bridge, "port_baudrate": str(baud)})
        # Reclaim the pty by INSPECTED PID (bracketed pgrep), never `pkill -f`
        # -- AGENTS.md Rule 1. The original client code broke that rule.
        qb = shlex.quote(bridge)
        bridge_cmd = (
            "command -v socat >/dev/null 2>&1 || sudo apt-get install -y socat; "
            f"for pid in $(pgrep -f \"[s]ocat.*{bridge}\"); do sudo kill \"$pid\" 2>/dev/null; done; sleep 0.3; "
            f"(socat -d -d UDP-LISTEN:{udp_port},reuseaddr PTY,link={qb},raw,echo=0,mode=666 &) && sleep 1.5")
        return f"{prefix}; {bridge_cmd} && {node} --ros-args {params}"

    # udp_server / udp_client: the driver's own native network modes.
    server_ip = a.get("server_ip", "0.0.0.0")
    if not re.match(r"^[0-9A-Za-z_.:-]+$", str(server_ip)):
        raise ValueError(f"server_ip: {server_ip!r} is not a host")
    server_port = _int(a.get("server_port"), "server_port", 1, 65535, 8889)
    params = _ld_node_params(a, {"comm_mode": mode, "server_ip": str(server_ip),
                                 "server_port": str(server_port), "port_baudrate": str(baud)})
    return f"{prefix}; {node} --ros-args {params}"


def _mag_calibrate(a: Dict) -> str:
    distro = _ident(a.get("distro"), "distro", "jazzy")
    return (f"{ros_setup_shell(distro)}; "
            "dpkg -s ros-$ROS_DISTRO-robot-calibration >/dev/null 2>&1 || "
            "sudo apt-get install -y ros-$ROS_DISTRO-robot-calibration; "
            "ros2 run robot_calibration magnetometer_calibration")


def _rviz_novnc(a: Dict) -> str:
    """RViz on a headless Xvfb, streamed to the browser through x11vnc + noVNC.
    A host-side viewer for a machine with no screen."""
    distro = _ident(a.get("distro"), "distro", "jazzy")
    display = a.get("display", ":99")
    if not re.match(r"^:\d+$", str(display)):
        raise ValueError(f"display: {display!r} is not an X display like :99")
    display_num = str(display)[1:]
    novnc_port = _int(a.get("novnc_port"), "novnc_port", 1, 65535, 6080)
    cfg = a.get("rviz_config")
    # Fall back to a bare rviz2 if the config file is missing, rather than
    # failing to start (matches the original client behaviour).
    rviz = (f"[ -f {shlex.quote(_path(cfg, 'rviz_config'))} ] && "
            f"rviz2 -d {shlex.quote(_path(cfg, 'rviz_config'))} || rviz2") if cfg else "rviz2"
    return f"{ros_setup_shell(distro)}; " + " && ".join([
        # rviz2 itself, guarded like the three tools below it. The robot image
        # carries the rviz LIBRARIES -- rviz_common, rviz_default_plugins,
        # rviz_rendering, nav2_rviz_plugins, all pulled in as dependencies --
        # but not the rviz2 EXECUTABLE, so this action installed Xvfb, x11vnc
        # and noVNC and then died on command-not-found. Verified on a stock
        # image, ROS sourced: rviz2 MISSING.
        ("command -v rviz2 >/dev/null 2>&1 || "
         "{ sudo apt-get install -y ros-$ROS_DISTRO-rviz2 && "
         "source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null; }"),
        "command -v rviz2 >/dev/null 2>&1 || { echo 'rviz2 is not installed and could not be installed'; exit 1; }",
        "command -v Xvfb >/dev/null 2>&1 || sudo apt-get install -y xvfb",
        "command -v x11vnc >/dev/null 2>&1 || sudo apt-get install -y x11vnc",
        "command -v websockify >/dev/null 2>&1 || sudo apt-get install -y novnc websockify",
        # Reclaim the display through its own lock file (a single inspected PID) --
        # never a broad string-matching kill (AGENTS.md).
        (f"if [ -e /tmp/.X{display_num}-lock ]; then XPID=$(cat /tmp/.X{display_num}-lock 2>/dev/null | tr -d ' '); "
         f"if [ -n \"$XPID\" ] && [ -r /proc/$XPID/cmdline ] && tr '\\0' ' ' < /proc/$XPID/cmdline | grep -q Xvfb; "
         f"then kill \"$XPID\" 2>/dev/null; fi; fi; sleep 0.3"),
        f"(Xvfb {display} -screen 0 1280x800x24 &) && sleep 1",
        f"(DISPLAY={display} {rviz} &) && sleep 1",
        f"(x11vnc -display {display} -forever -shared -nopw -quiet -rfbport 5900 &) && sleep 1",
        f"websockify --web=/usr/share/novnc {novnc_port} localhost:5900",
    ])


# --------------------------------------------------------------------------
# Registry.
# --------------------------------------------------------------------------
_ACTIONS: Dict[str, Callable[[Dict], str]] = {
    "agent_start": _agent_start,
    "agent_prepare": _agent_prepare,
    "bringup": _bringup,
    "bringup_docker": _bringup_docker,
    "slam": _slam,
    "nav2": _nav2,
    "teleop": _teleop,
    "map_save": _map_save,
    "docker_build": _docker_build,
    "docker_service_up": _docker_service_up,
    "docker_down": _docker_down,
    "apt_install": _apt_install,
    "mag_calibrate": _mag_calibrate,
    "rviz_novnc": _rviz_novnc,
    "laser_driver": _laser_driver,
}


def known(action: str) -> bool:
    return action in _ACTIONS


def build(action: str, args: Dict) -> str:
    """The command for a named action, built from validated args. Raises
    ValueError for an unknown action or an argument that fails validation."""
    fn = _ACTIONS.get(action)
    if fn is None:
        raise ValueError(f"unknown action {action!r}")
    return fn(args or {})
