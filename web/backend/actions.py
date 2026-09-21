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

import re
import secrets
import shlex
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


def _ident(value, name: str, default: str = None) -> str:
    s = str(value if value not in (None, "") else (default if default is not None else ""))
    if default is not None and s == "":
        s = default
    if not _IDENT_RE.match(s):
        raise ValueError(f"{name}: {value!r} is not a plain identifier")
    return s


def _enum(value, name: str, allowed, default: str = None) -> str:
    s = str(value) if value not in (None, "") else default
    if s not in allowed:
        raise ValueError(f"{name}: {value!r} is not one of {sorted(allowed)}")
    return s


def _int(value, name: str, lo: int, hi: int, default: int = None) -> int:
    try:
        n = int(value) if value not in (None, "") else default
    except (TypeError, ValueError):
        raise ValueError(f"{name}: {value!r} is not an integer")
    if n is None or not (lo <= n <= hi):
        raise ValueError(f"{name}: {value!r} out of range [{lo},{hi}]")
    return n


def _num(value, name: str, lo: float, hi: float, default: float) -> float:
    try:
        x = float(value) if value not in (None, "") else default
    except (TypeError, ValueError):
        raise ValueError(f"{name}: {value!r} is not a number")
    if not (lo <= x <= hi):
        raise ValueError(f"{name}: {value!r} out of range [{lo},{hi}]")
    return x


def _device(value, name: str, default: str) -> str:
    s = str(value) if value not in (None, "") else default
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


def _bringup_docker(a: Dict) -> str:
    compose = _compose_resolve(a)
    flags = "--env-file .env -f docker-compose.yaml -f devices.generated.yaml"
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
    params = a.get("params_file")
    params_arg = f" params_file:={shlex.quote(_path(params, 'params_file'))}" if params else ""
    depth = "true" if _bool(a.get("depth")) else "false"
    map_arg = f" map:={shlex.quote(_path(a.get('map'), 'map'))}" if a.get("map") else ""
    return (f"{ros_setup_shell(distro)}; if [ -f {shlex.quote(launcher)} ]; then "
            f"ros2 launch {shlex.quote(launcher)} slam:=false{params_arg}{map_arg} depth_costmap:={depth} distro:={distro} sim:=false; "
            f"else ros2 launch linorobot2_navigation navigation.launch.py; fi")


def _teleop(a: Dict) -> str:
    distro = _ident(a.get("distro"), "distro", "jazzy")
    axis_lin = _int(a.get("axis_linear"), "axis_linear", 0, 31, 1)
    scale_lin = _num(a.get("scale_linear"), "scale_linear", 0.0, 100.0, 0.5)
    axis_ang = _int(a.get("axis_angular"), "axis_angular", 0, 31, 0)
    scale_ang = _num(a.get("scale_angular"), "scale_angular", 0.0, 100.0, 1.0)
    tmp = "/tmp/linorobot2_console_joy.yaml"
    yaml_body = (f"teleop_twist_joy_node:\n  ros__parameters:\n"
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
    flags = "--env-file .env -f docker-compose.yaml -f devices.generated.yaml"
    d = _path(a.get("docker_dir"), "docker_dir")
    service = _ident(a.get("service"), "service")
    return f"{compose}cd {shlex.quote(d)} && DISPLAY=:200 $COMPOSE {flags} up {service}"


def _docker_down(a: Dict) -> str:
    compose = _compose_resolve(a)
    flags = "--env-file .env -f docker-compose.yaml -f devices.generated.yaml"
    d = _path(a.get("docker_dir"), "docker_dir")
    return f"{compose}cd {shlex.quote(d)} && $COMPOSE {flags} down"


def _apt_install(a: Dict) -> str:
    pkgs = _packages(a.get("packages"), "packages")
    return "sudo apt-get update && sudo apt-get install -y " + " ".join(shlex.quote(p) for p in pkgs)


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
    "docker_service_up": _docker_service_up,
    "docker_down": _docker_down,
    "apt_install": _apt_install,
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
