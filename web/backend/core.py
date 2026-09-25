"""Linorobot2 Cockpit backend core: the FastAPI `app`, middleware, shared
state and the helper functions the route modules build on.

The routes themselves live in routes_*.py; main.py imports core (which creates
`app` and registers the middleware/exception handler) and then imports each
routes_* module, whose @app.<method> handlers register on this same app as an
import side effect. A handler that is ALSO called internally stays here in core
(so callers resolve it) rather than moving to a route module.
"""
#!/usr/bin/env python3
# ==============================================================================
# Linorobot2 Cockpit — FastAPI Supervisor Backend Daemon
#
# Unified backend integrating all capabilities from Linorobot2 Console and
# Robot Config Engine:
# - Multi-threaded UDP Syslog Server (port 5140 by default)
# - ProcessRunners for streaming command execution (main, agent, bringup, laser, tool)
# - Safe serial port checking and PID-based release (AGENTS.md compliant)
# - Nav2 / SLAM / EKF configuration and tuning presets (Single Source of Truth)
# - AI robotics tuning assistant and robot builder
# - Virtual Gamepad teleop publisher
# - Static frontend asset server and SSE streams
# ==============================================================================

import collections
import glob
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
import urllib.error
import urllib.request
from contextlib import asynccontextmanager
from typing import Any, Dict, List, Optional


_DEPS_RETRIED_ENV = "COCKPIT_DEPS_RETRIED"


def _ensure_dependencies():
    """Verify required third-party packages and auto-install on Debian/Ubuntu if permitted."""
    missing_apt = []
    missing_pip = []

    try:
        import yaml
    except ImportError:
        missing_apt.append("python3-yaml")
        missing_pip.append("PyYAML")

    try:
        import fastapi
    except ImportError:
        missing_apt.append("python3-fastapi")
        missing_pip.append("fastapi")

    try:
        import uvicorn
    except ImportError:
        missing_apt.append("python3-uvicorn")
        missing_pip.append("uvicorn")

    try:
        import serial
    except ImportError:
        missing_apt.append("python3-serial")
        missing_pip.append("pyserial")

    if not missing_apt:
        return

    print("=" * 72, file=sys.stderr)
    print("[linorobot2-cockpit] Missing required Python packages:", file=sys.stderr)
    for pkg in missing_apt:
        print(f"  • {pkg}", file=sys.stderr)
    print("=" * 72, file=sys.stderr)

    can_auto_install = False
    sudo_prefix = []
    if shutil.which("apt-get"):
        if os.geteuid() == 0:
            can_auto_install = True
        else:
            try:
                res = subprocess.run(["sudo", "-n", "true"], capture_output=True, timeout=3)
                if res.returncode == 0:
                    can_auto_install = True
                    sudo_prefix = ["sudo", "-n"]
            except Exception:
                pass

    # apt can report success while the import still fails: apt installs into the
    # SYSTEM interpreter's dist-packages, which a venv, pyenv, conda or CI's
    # setup-python cannot see. Re-executing then finds the same packages missing
    # and installs them again -- forever. CI caught it doing exactly that:
    #
    #   Missing required Python packages: • python3-serial
    #   Automatically installing required packages via apt-get...
    #   python3-serial is already the newest version (3.5-2).
    #   All dependencies installed successfully! Resuming startup...
    #   Missing required Python packages: • python3-serial      (round and round)
    #
    # The cockpit never starts and never says why. One retry, then explain.
    already_retried = os.environ.get(_DEPS_RETRIED_ENV) == "1"
    if can_auto_install and already_retried:
        print(f"[linorobot2-cockpit] apt reported success, but "
              f"{', '.join(missing_pip)} still cannot be imported by this "
              f"interpreter:\n    {sys.executable}\n"
              f"  It does not see apt's site-packages -- typical of a venv, pyenv, "
              f"conda, or a tool-cache Python.\n"
              f"  Install into THIS interpreter instead:\n"
              f"    {sys.executable} -m pip install {' '.join(missing_pip)}",
              file=sys.stderr)
        can_auto_install = False

    if can_auto_install:
        print("[linorobot2-cockpit] Automatically installing required packages via apt-get...", file=sys.stderr)
        try:
            proxy_args = []
            if os.environ.get("APT_PROXY"):
                proxy_args = ["-o", f"Acquire::http::Proxy={os.environ['APT_PROXY']}"]

            env = os.environ.copy()
            env["DEBIAN_FRONTEND"] = "noninteractive"
            update_cmd = sudo_prefix + ["apt-get", "-o", "APT::Update::Pre-Invoke::="] + proxy_args + ["update", "-qq"]
            subprocess.run(update_cmd, check=False, env=env)
            install_cmd = (
                sudo_prefix
                + ["apt-get", "install", "-y", "--no-install-recommends", "-o", "DPkg::Lock::Timeout=60"]
                + proxy_args
                + missing_apt
            )
            ret = subprocess.run(install_cmd, env=env)
            if ret.returncode == 0:
                print("[linorobot2-cockpit] All dependencies installed successfully! Resuming startup...\n", file=sys.stderr)
                # Marks the one retry we allow; os.execv keeps the environment.
                os.environ[_DEPS_RETRIED_ENV] = "1"
                os.execv(sys.executable, [sys.executable] + sys.argv)
        except Exception as e:
            print(f"[linorobot2-cockpit] Auto-installation failed: {e}", file=sys.stderr)

    print("\nPlease install the missing dependencies manually and re-run:", file=sys.stderr)
    print(f"  Ubuntu / Debian:\n    sudo apt update && sudo apt install -y {' '.join(missing_apt)}\n", file=sys.stderr)
    print(f"  Virtual Environment (pip):\n    pip install {' '.join(missing_pip)}\n", file=sys.stderr)
    print(f"  Or run helper script:\n    bash scripts/install_deps.sh", file=sys.stderr)
    print("=" * 72 + "\n", file=sys.stderr)
    sys.exit(1)


_ensure_dependencies()

# Compatibility shim for Ubuntu 24.04 (FastAPI 0.101 + Starlette 0.31)
try:
    import starlette.applications
    import starlette.routing

    _orig_router_init = starlette.routing.Router.__init__

    def _compat_router_init(self, *args, on_startup=None, on_shutdown=None, **kwargs):
        return _orig_router_init(self, *args, **kwargs)

    starlette.routing.Router.__init__ = _compat_router_init

    _orig_starlette_call = starlette.applications.Starlette.__call__

    async def _compat_starlette_call(self, scope, receive, send):
        if scope.get("type") == "lifespan":
            await self.router.lifespan(scope, receive, send)
            return
        await _orig_starlette_call(self, scope, receive, send)

    starlette.applications.Starlette.__call__ = _compat_starlette_call
except Exception:
    pass

import yaml
from fastapi import FastAPI, HTTPException, Request, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

# Local helper modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import patcher
import yaml_merge
from runners import (
    BRINGUP_HEALTH_TOPICS,
    BRINGUP_TF_CHAIN,
    GamepadRunner,
    ProcessRunner,
    ros_setup_shell,
    check_agent_port_status,
    check_bringup_health,
    release_agent_port,
)
from hardware_service import (
    detect as hw_detect,
    flash as hw_flash,
    list_serial_ports as hw_list_serial_ports,
    resolve_artifact as hw_resolve_artifact,
    tool_status as hw_tool_status,
)
from syslog_manager import SyslogManager
from system_utils import (
    analyze_robotics_ai,
    build_base_install_cmd,
    build_ros2_install_cmd,
    build_sensor_install_cmd,
    check_container_status,
    disable_autostart,
    enable_autostart,
    generate_custom_robot_specs,
    get_autostart_logs,
    get_autostart_status,
    get_package_install_info,
    get_rootless_info,
    get_sensor_driver_status,
    install_container_engine,
    list_dir,
    nav2_stack_status,
    sensor_registry,
    setup_rootless_docker,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cockpit_paths  # noqa: E402
import access  # noqa: E402  (web/backend/access.py: token + path policy)
import fetch_prebuilt  # noqa: E402
import mcu_identity
import robot_stack  # noqa: E402
import one_click_pipeline  # noqa: E402
import mcu_identity  # noqa: E402
import mcu_probe  # noqa: E402  (the boot-banner parser and the flash stamps)
import pin_catalog  # noqa: E402
import gen_robot_description  # the URDF, generated from the config (scripts/)

# The user's robots live OUTSIDE the repo, in their own git repository
# (~/linorobot2-config, or $COCKPIT_CONFIG_DIR). Seeded from config/reference
# on first start. See scripts/cockpit_paths.py.
CONFIG_DIR = cockpit_paths.ensure_config_dir()
SECRETS_PATH = cockpit_paths.secrets_path()
SECRETS_EXAMPLE_PATH = cockpit_paths.SECRETS_EXAMPLE_PATH
COCKPIT_TOKEN = access.load_or_create_token(CONFIG_DIR)
FRONTEND_DIR = os.path.join(REPO_ROOT, "web", "frontend")


def display_path(path: str) -> str:
    """A path as the user would type it: ~/linorobot2-config/x.yaml, not an absolute one."""
    home = os.path.expanduser("~")
    path = os.path.abspath(path)
    return "~" + path[len(home):] if path.startswith(home + os.sep) else path

# ------------------------------------------------------------------------------
# Global Runners & Managers
# ------------------------------------------------------------------------------
syslog_manager = SyslogManager(REPO_ROOT)
main_runner = ProcessRunner("main")
agent_runner = ProcessRunner("agent")
bringup_runner = ProcessRunner("bringup")
laser_runner = ProcessRunner("laser")
tool_runner = ProcessRunner("tool")
gamepad_runner = GamepadRunner(REPO_ROOT)

RUNNERS = {
    "main": main_runner,
    "agent": agent_runner,
    "bringup": bringup_runner,
    "laser": laser_runner,
    "tool": tool_runner,
}


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Auto-start the UDP syslog receiver on the CONFIGURED port, defaulting to
    # 5140 when the operator has not set one.
    #
    # 5140 rather than 514 because this is a private sink for one robot, not a
    # host syslog daemon: it gains nothing from the privileged well-known port,
    # and an unprivileged port is the one both root and non-root can always
    # bind. The port used to depend on who started the backend (514 as root,
    # 5140 otherwise), which made it a RUNTIME fact two machines could disagree
    # about -- and a board pointed at 514 with a sink on 5140 fails in total
    # silence: the board keeps sending and the syslog tab just stays empty.
    #
    # It is a default, not a fixed value: `telemetry.syslog_port` in
    # config/secrets.yaml still wins, so an operator who wants 514 (or anything
    # else) gets it. The board reads the same key through
    # mcu_env.resolve_syslog_port(), so the two stay in step.
    initial_port = 5140
    try:
        import yaml as _yaml
        with open(SECRETS_PATH) as _f:
            _sec = _yaml.safe_load(_f) or {}
        _cfg = ((_sec.get("telemetry") or {}).get("syslog_port"))
        if _cfg is not None:
            initial_port = int(_cfg)
    except Exception:
        pass
    try:
        syslog_manager.start(initial_port)
    except Exception as e:
        print(f"[syslog_manager] startup notice: {e}")
    port = os.environ.get("COCKPIT_PORT", "8000")
    if access.auth_enabled():
        print(f"[cockpit] access token: open http://<robot-computer>:{port}/?token={COCKPIT_TOKEN} "
              f"once; the browser remembers it. Also in {display_path(access.token_path(CONFIG_DIR))}. "
              f"COCKPIT_AUTH=off disables the check.")
    else:
        print("[cockpit] COCKPIT_AUTH=off: the API accepts writes from anyone on the network")
    yield
    # Shutdown
    syslog_manager.stop()
    for r in RUNNERS.values():
        r.kill()
    gamepad_runner.kill()


app = FastAPI(title="Linorobot2 Cockpit Supervisor", version="1.0.0", lifespan=lifespan)

# The UI is served by this same process, so it needs no CORS at all. The one
# legitimate cross-origin caller is a frontend dev server on 5173; everything
# else on the network -- including a page open in someone's browser on the
# same LAN -- gets the browser's default refusal. It used to be `*` with
# credentials, which let any web page POST to /api/exec.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.middleware("http")
async def require_access_token(request: Request, call_next):
    """Writes to /api/ and the private GETs need the install's token (access.py).

    The EventSource streams are the exception: a one-shot ?ticket= (minted by the
    token-authed GET /api/stream_ticket) authorises them, so the long-lived token
    never rides in a stream URL. A valid token still works for them too.
    """
    if access.auth_enabled() and access.needs_token(request.method, request.url.path):
        ticket_ok = (access.is_stream_path(request.url.path)
                     and access.consume_stream_ticket(request.query_params.get("ticket", "")))
        if not ticket_ok and not access.token_matches(access.presented_token(request), COCKPIT_TOKEN):
            return JSONResponse(
                status_code=401,
                content={
                    "error": "cockpit token required",
                    "detail": "This cockpit needs its access token. Open the ?token= URL the "
                              "supervisor printed at start (docker compose logs), or paste the "
                              f"contents of {display_path(access.token_path(CONFIG_DIR))}.",
                },
            )
    return await call_next(request)


@app.exception_handler(Exception)
async def json_error_handler(request: Request, exc: Exception):
    """Answer an unhandled error with JSON carrying the reason.

    The default handler replies with the plain text "Internal Server Error",
    which every fetch().then(r => r.json()) in the SPA turns into an opaque
    "JSON.parse: unexpected character at line 1 column 1" — the real cause never
    reaches the console.
    """
    traceback.print_exc()
    return JSONResponse(
        status_code=500,
        content={"error": f"{type(exc).__name__}: {exc}", "detail": f"{type(exc).__name__}: {exc}",
                 "path": str(request.url.path)},
    )


# ------------------------------------------------------------------------------
# What a client actually sent
# ------------------------------------------------------------------------------
# A request body is whatever reached the socket, and every endpoint here reads
# it with `await json_body(request)` and then treats the result as a dict of
# strings. Neither is guaranteed. `{"robot": {...}}` is as valid JSON as
# `{"robot": "pico"}`, and `.strip()` on the dict raised AttributeError inside
# the handler -- a 500 with a traceback in the log, where a 400 belongs. The
# API test found it by posting /api/config's own answer back to
# /api/robot/select, which is not even a malicious shape: `robot` is a mapping
# in the config, and a caller reasonably passed it along.
#
# These two say it once for all 36 call sites: a body that is not a JSON object
# reads as empty, and a field that is not a string reads as absent. Each
# endpoint's own validation then answers 400 the way it already does for a
# missing field.


async def json_body(request: Request) -> Dict[str, Any]:
    """The request body as a dict; {} for malformed JSON or a non-object."""
    try:
        data = await request.json()
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def text_field(data: Dict[str, Any], *keys: str, default: str = "") -> str:
    """The first key holding a non-empty string, stripped; `default` otherwise."""
    if not isinstance(data, dict):
        return default
    for key in keys:
        value = data.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return default


import actions  # noqa: E402  (web/backend/actions.py: named server-side commands)

# The browser used to POST the command TEXT to /api/exec, so the token bought a
# shell. Now it names an action (built server-side from validated args) or a
# prepared handle (a command another endpoint already built). A raw command is
# refused unless COCKPIT_ALLOW_RAW_EXEC is set -- an escape hatch for a trusted
# operator debugging, off by default.
def _allow_raw_exec() -> bool:
    # Default OFF: every UI screen now names a server-side action (actions.py) or
    # runs a prepared handle, so the browser never puts command text on the wire.
    # The hole the reviews led with is closed. The env var is an escape hatch for
    # a trusted operator debugging by hand, off unless explicitly set.
    return os.environ.get("COCKPIT_ALLOW_RAW_EXEC", "").strip().lower() in ("1", "true", "yes", "on")


def resolve_command(data: Dict[str, Any], default: str = "") -> str:
    """Turn an exec request body into the command to run.

    {action, args}      -> built server-side (actions.build), or a prepared
                           handle when action == "prepared"
    {command}           -> only when COCKPIT_ALLOW_RAW_EXEC is set
    neither             -> `default` (the endpoint's own safe fallback)
    Raises HTTPException(400) for an unknown/invalid action or a refused raw
    command.
    """
    action = text_field(data, "action")
    if action:
        if action == "prepared":
            cmd = actions.claim(text_field(data, "handle"))
            if not cmd:
                raise HTTPException(status_code=400, detail="prepared command handle is unknown or expired")
            return cmd
        if actions.known(action):
            try:
                return actions.build(action, data.get("args") or {})
            except ValueError as exc:
                raise HTTPException(status_code=400, detail=f"action {action!r}: {exc}")
        # An unknown action is an ERROR, not a request for the endpoint's default.
        # Falling through used to mean a misspelled or renamed action ran the
        # fallback instead: /api/agent/exec started a SERIAL agent when a UDP one
        # was asked for, and /api/bringup/exec brought up an UNPREFIXED robot when
        # a namespaced one was asked for -- the request looked like it worked. The
        # legacy shape (a human label in `action` beside a raw `command`) only ever
        # worked with raw enabled, so it is the one case still allowed through.
        if not (_allow_raw_exec() and text_field(data, "command")):
            raise HTTPException(
                status_code=400,
                detail=f"unknown action {action!r}; the server builds commands from a named "
                       f"action, so a name it does not know cannot be run")
    raw = text_field(data, "command")
    if raw:
        if not _allow_raw_exec():
            raise HTTPException(
                status_code=400,
                detail="raw commands are disabled; the UI names an action instead "
                       "(set COCKPIT_ALLOW_RAW_EXEC=on to allow raw, for debugging only).")
        return raw
    return default


# ------------------------------------------------------------------------------
# Single Source of Truth Helpers (Per-Robot Configuration)
# ------------------------------------------------------------------------------
DEFAULT_ROBOT_NAME = "pico2_mecanum"
ACTIVE_PARAMS_PATH = os.path.join(CONFIG_DIR, f"{DEFAULT_ROBOT_NAME}_config.yaml")
ACTIVE_ROBOT_NAME = DEFAULT_ROBOT_NAME


def active_robot_name() -> str:
    """The current active robot's name.

    A function, not a bare import: this global is reassigned on robot-select
    (set_active_params_path), so a route module that did `from core import
    ACTIVE_ROBOT_NAME` would bind a stale snapshot and, after a switch, show the
    wrong name in status, commit messages and export filenames. Route modules
    call this instead; core's own code, living in this module, may read the
    global directly.
    """
    return ACTIVE_ROBOT_NAME
# The selection outlives the process. It used to live only in these globals, so
# every container restart put the UI back on the default robot; now the config
# file's name is written to <config dir>/.active_robot (gitignored there) and
# read back at start-up, when it still names a config that exists.
ACTIVE_ROBOT_FILE = os.path.join(CONFIG_DIR, ".active_robot")


def _restore_active_robot() -> Optional[str]:
    try:
        with open(ACTIVE_ROBOT_FILE) as fh:
            name = os.path.basename(fh.read().strip())
    except OSError:
        return None
    if not name or name.startswith("secrets") or not name.endswith((".yaml", ".yml")):
        return None
    path = os.path.join(CONFIG_DIR, name)
    return path if os.path.isfile(path) else None


def _remember_active_robot(path: str):
    try:
        with open(ACTIVE_ROBOT_FILE, "w") as fh:
            fh.write(os.path.basename(path) + "\n")
    except OSError as e:
        print(f"[cockpit] could not record the active robot in {ACTIVE_ROBOT_FILE}: {e}")


def list_robot_config_files() -> List[str]:
    """Every per-robot config in the config directory (secrets excluded)."""
    if not os.path.isdir(CONFIG_DIR):
        return []
    return [
        os.path.join(CONFIG_DIR, f)
        for f in sorted(os.listdir(CONFIG_DIR))
        if f.endswith((".yaml", ".yml")) and not f.startswith("secrets")
    ]


def get_active_params_path() -> str:
    """
    Path of the robot being edited. One robot per file, one controller per
    robot -- so the file itself is the selection; there is no active_target to
    resolve inside it any more.
    """
    global ACTIVE_PARAMS_PATH, ACTIVE_ROBOT_NAME
    if ACTIVE_PARAMS_PATH and os.path.isfile(ACTIVE_PARAMS_PATH):
        return ACTIVE_PARAMS_PATH
    saved = _restore_active_robot()
    if saved:
        set_active_params_path(saved, remember=False)
        return ACTIVE_PARAMS_PATH
    default_cfg = os.path.join(CONFIG_DIR, f"{DEFAULT_ROBOT_NAME}_config.yaml")
    if os.path.isfile(default_cfg):
        ACTIVE_PARAMS_PATH = default_cfg
        ACTIVE_ROBOT_NAME = DEFAULT_ROBOT_NAME
        return default_cfg
    # Otherwise fall back to whichever robot config exists.
    others = list_robot_config_files()
    if others:
        ACTIVE_PARAMS_PATH = others[0]
        ACTIVE_ROBOT_NAME = os.path.basename(others[0]).replace("_config.yaml", "").replace(".yaml", "")
        return ACTIVE_PARAMS_PATH
    return ACTIVE_PARAMS_PATH


def set_active_params_path(path: str, remember: bool = True):
    global ACTIVE_PARAMS_PATH, ACTIVE_ROBOT_NAME
    if os.path.isfile(path):
        ACTIVE_PARAMS_PATH = path
        try:
            with open(path, "r") as f:
                d = yaml.safe_load(f) or {}
            ACTIVE_ROBOT_NAME = d.get("robot", {}).get("name") or os.path.basename(path).replace("_config.yaml", "").replace(".yaml", "")
        except Exception:
            pass
        if remember:
            _remember_active_robot(path)


# Start where the last run left off, if that robot's config is still there.
_saved_active = _restore_active_robot()
if _saved_active:
    set_active_params_path(_saved_active, remember=False)
del _saved_active


def load_params(path: Optional[str] = None) -> Dict[str, Any]:
    cfg_path = path or get_active_params_path()
    if not os.path.isfile(cfg_path):
        return {}
    with open(cfg_path, "r") as f:
        return yaml.safe_load(f) or {}


def save_params(data: Dict[str, Any], path: Optional[str] = None):
    target_path = path or get_active_params_path()
    with open(target_path, "w") as f:
        yaml.dump(data, f, sort_keys=False)


def update_robot_name_and_controller(name: str, controller: Optional[str] = None, path: Optional[str] = None):
    """Update robot name and optional base controller in the robot YAML, preserving comments."""
    cfg_path = path or get_active_params_path()
    if not os.path.isfile(cfg_path):
        return
    with open(cfg_path, "r") as f:
        content = f.read()

    # If robot: section exists, update using regex to preserve comments and layout
    if "robot:" in content:
        content = re.sub(
            r'(robot:\s*\n(?:[ \t]*[^\n]*\n)*?[ \t]*name:\s*)[^\n]+',
            f'\\g<1>"{name}"',
            content,
        )
        if controller:
            # base_controller: has its own name: -- anchor on the block header so
            # this cannot walk into robot.name.
            content = re.sub(
                r'(base_controller:\s*\n(?:[ \t]*[^\n]*\n)*?[ \t]*name:\s*)[^\n#]+',
                f'\\g<1>"{controller}" ',
                content,
            )
        with open(cfg_path, "w") as f:
            f.write(content)
    else:
        params = load_params(cfg_path)
        params.setdefault("robot", {})["name"] = name
        if controller:
            params.setdefault("base_controller", {})["name"] = controller
        save_params(params, path=cfg_path)


# The simulated MCU: sim_base_node on this computer stands in for the board.
SIM_MCU = "sim"


def refuse_sim_flash(name: str) -> None:
    """Nothing to flash when the base controller is the simulated MCU."""
    if (name or "").strip().lower() == SIM_MCU:
        raise HTTPException(status_code=400, detail=(
            "The base controller is the simulated MCU (sim_base_node): there is no board "
            "to build for, flash or monitor. Pick the board you plugged in on the Base & MCU "
            "tab to use it."))


def get_controller(params: Dict[str, Any]) -> Dict[str, Any]:
    """The robot's base controller block (linorobot2_hardware firmware)."""
    bc = params.get("base_controller")
    return bc if isinstance(bc, dict) else {}


def get_controller_name(params: Dict[str, Any], default: str = "pico2") -> str:
    """Controller name -- also the PlatformIO env and the firmware variant."""
    return get_controller(params).get("name") or default


def regenerate_firmware_headers(controller: Optional[str] = None, params_path: Optional[str] = None):
    gen_script = os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py")
    active_path = params_path or get_active_params_path()
    # The simulated MCU has no firmware, so there is no header to generate.
    name = controller or get_controller_name(load_params(active_path) if os.path.isfile(active_path) else {})
    if name == SIM_MCU:
        return subprocess.CompletedProcess([], 0, "simulated MCU: no firmware header\n", "")
    cmd = [sys.executable, gen_script, "--params", active_path]
    if controller:
        cmd.extend(["--controller", controller])
    return subprocess.run(cmd, capture_output=True, text=True)


def require_header(res) -> None:
    """Stop if the firmware header could not be regenerated.

    gen_firmware_header.py exits non-zero on a pin error and writes the reason
    to stderr. Every caller here used to discard that, so a rejected config
    left the previous config.h in place and the next `pio run` built from it:
    the board came back reporting success and running firmware for a config
    nobody had asked for. A flash is the one place that must fail loudly.
    """
    if res is None or res.returncode == 0:
        return
    detail = (res.stderr or res.stdout or "").strip() or "firmware header generation failed"
    raise HTTPException(status_code=400, detail=detail)


def regenerate_robot_description(params: Dict[str, Any], params_path: Optional[str] = None) -> Dict[str, Any]:
    """The URDF for this config, written to <config dir>/generated/<robot>.urdf.

    bringup.launch.py regenerates it at every launch as well; doing it on save
    is what lets the Pins tab and /api/robot/urdf show the description the
    next launch will publish, and what surfaces geometry warnings while the
    user is still looking at the numbers.
    """
    path = params_path or get_active_params_path()
    try:
        out = gen_robot_description.write_urdf(
            params, gen_robot_description.default_out_path(params, path, os.path.dirname(path)))
        return {"urdf_ok": True, "urdf_path": display_path(out),
                "geometry_warnings": gen_robot_description.geometry_warnings(params)}
    except Exception as exc:  # a config with no kinematics yet, an unwritable dir
        return {"urdf_ok": False, "urdf_error": str(exc),
                "geometry_warnings": gen_robot_description.geometry_warnings(params)}


SERVER_BOOT_COMMIT = None


def get_git_info_dict() -> Dict[str, Any]:
    global SERVER_BOOT_COMMIT

    def _run(cmd):
        try:
            return subprocess.check_output(
                cmd, cwd=REPO_ROOT, text=True, stderr=subprocess.DEVNULL
            ).strip()
        except Exception:
            return ""

    branch = _run(["git", "branch", "--show-current"]) or _run(
        ["git", "rev-parse", "--abbrev-ref", "HEAD"]
    )
    commit = _run(["git", "rev-parse", "--short", "HEAD"])
    status = _run(["git", "status", "--porcelain"])
    message = _run(["git", "log", "-1", "--pretty=%B"])

    branches = []
    seen_branches = set()
    # Local branches
    for b in _run(["git", "branch", "--format=%(refname:short)"]).splitlines():
        b = b.strip().replace("heads/", "")
        if b and b not in seen_branches:
            seen_branches.add(b)
            branches.append(b)
    # Remote branches
    for b in _run(["git", "branch", "-r", "--format=%(refname:short)"]).splitlines():
        b = b.strip()
        if not b or "HEAD" in b:
            continue
        clean_b = b.split("/", 1)[-1] if "/" in b else b
        if clean_b and clean_b != "origin" and clean_b not in seen_branches:
            seen_branches.add(clean_b)
            branches.append(clean_b)
    if not branches and branch:
        branches = [branch]

    remotes = []
    seen_remotes = set()
    for line in _run(["git", "remote", "-v"]).splitlines():
        parts = line.split()
        if len(parts) >= 2:
            rname, rurl = parts[0], parts[1]
            if rname not in seen_remotes:
                seen_remotes.add(rname)
                remotes.append({"name": rname, "url": rurl})

    commits = []
    log_out = _run(
        [
            "git",
            "log",
            "-n",
            "10",
            "--pretty=format:%h%x09%s%x09%an%x09%ad%x09%ar",
            "--date=short",
        ]
    )
    if log_out:
        for line in log_out.splitlines():
            parts = line.split("\t")
            if len(parts) >= 5:
                commits.append(
                    {
                        "hash": parts[0],
                        "subject": parts[1],
                        "author": parts[2],
                        "date": parts[3],
                        "reldate": parts[4],
                    }
                )

    if not SERVER_BOOT_COMMIT and commit:
        SERVER_BOOT_COMMIT = commit

    version = commit or "unknown"
    version_at_start = SERVER_BOOT_COMMIT or version

    return {
        "version": version,
        "version_at_start": version_at_start,
        "commit": commit,
        "branch": branch or "unknown",
        "branches": branches,
        "remotes": remotes,
        "commits": commits,
        "dirty": bool(status),
        "moved_since_start": bool(
            SERVER_BOOT_COMMIT and commit and commit != SERVER_BOOT_COMMIT
        ),
        "message": message,
    }


def sysfs_device_link(port_path: str) -> str:
    """The sysfs `device` link for a tty, found by its device number.

    Lives in scripts/mcu_identity.py so the pipeline's pre-flash guard resolves
    a port exactly the way this page labels it; the rationale is in that module.
    """
    return mcu_identity.sysfs_device_link(port_path)


def get_port_usb_details(port_path: str) -> Dict[str, Any]:
    port_name = os.path.basename(port_path)
    info = {
        "port": port_path,
        # udev's stable name for this board. /dev/ttyACM0 is enumeration order,
        # not identity: two Picos on one bench swapped numbers overnight and a
        # flash aimed at the config's port went at the other board.
        "by_id": mcu_identity.by_id_for_port(port_path),
        "vid": "",
        "pid": "",
        "product": "",
        "manufacturer": "",
        "chip": "Generic USB Serial",
        "mcu_hint": "",
        "decisive": False,
    }
    sys_device_path = sysfs_device_link(port_path)
    if os.path.exists(sys_device_path):
        curr = os.path.realpath(sys_device_path)
        for _ in range(6):
            vid_path = os.path.join(curr, "idVendor")
            pid_path = os.path.join(curr, "idProduct")
            if os.path.exists(vid_path) and os.path.exists(pid_path):
                try:
                    with open(vid_path, "r") as f:
                        info["vid"] = f.read().strip().lower()
                    with open(pid_path, "r") as f:
                        info["pid"] = f.read().strip().lower()
                    prod_path = os.path.join(curr, "product")
                    mfg_path = os.path.join(curr, "manufacturer")
                    if os.path.exists(prod_path):
                        with open(prod_path, "r") as f:
                            info["product"] = f.read().strip()
                    if os.path.exists(mfg_path):
                        with open(mfg_path, "r") as f:
                            info["manufacturer"] = f.read().strip()
                except Exception:
                    pass
                break
            curr = os.path.dirname(curr)

    # One table for the label shown here and the family the pre-flash guard
    # reasons about (scripts/mcu_identity.py). When these were two copies, the
    # UI could say "Raspberry Pi Pico (RP2040)" while the pipeline flashed an
    # RP2350 image -- which is exactly what happened.
    family, chip, decisive = mcu_identity.classify_usb(
        info["vid"], info["pid"], info["product"])
    if family:
        info["chip"] = chip
        info["mcu_hint"] = family
    # Carried through so the UI can warn only when the bus POSITIVELY
    # contradicts the config. An RP2 names its own silicon; a classic ESP32
    # answers through a CP2102 that says nothing about the chip behind it, and
    # warning there would cry wolf on every ESP32 robot.
    info["decisive"] = decisive

    return info


def list_local_ports() -> List[Dict[str, Any]]:
    """Every USB serial device on THIS machine, with the MCU each one looks like."""
    ports = []
    for p in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        try:
            holders = subprocess.check_output(["lsof", "-t", p], text=True).strip().split()
        except Exception:
            holders = []
        details = get_port_usb_details(p)
        ports.append({
            "path": p,
            "by_id": details.get("by_id", ""),
            "busy": len(holders) > 0,
            "holders": holders,
            "vid": details.get("vid", ""),
            "pid": details.get("pid", ""),
            "product": details.get("product", ""),
            "manufacturer": details.get("manufacturer", ""),
            "chip": details.get("chip", "Generic USB Serial"),
            "mcu_hint": details.get("mcu_hint", ""),
            "decisive": details.get("decisive", False),
        })
    return ports


def host_lan_ip() -> str:
    """This machine's primary LAN address (what the board dials over Wi-Fi)."""
    try:
        return SyslogManager._host_ip()
    except Exception:
        return "127.0.0.1"


def suggest_mcu(ports: List[Dict[str, Any]]) -> Dict[str, Any]:
    """The port a run should use, and the controller env it looks like.

    First device carrying an MCU hint wins; a bench with two boards is not a
    robot (AGENTS.md S6), so there is nothing cleverer to do here than say what
    was found and let the user pick a chip from the list.
    """
    for p in ports:
        if p.get("mcu_hint"):
            return {"port": p.get("path", ""), "mcu": p.get("mcu_hint", ""),
                    "chip": p.get("chip", ""), "busy": bool(p.get("busy"))}
    if ports:
        p = ports[0]
        return {"port": p.get("path", ""), "mcu": "", "chip": p.get("chip", ""),
                "busy": bool(p.get("busy"))}
    return {"port": "", "mcu": "", "chip": "", "busy": False}


def get_serial_ports_dict(params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """The USB serial devices on this machine — the robot computer is where the board is."""
    ports = list_local_ports()
    return {
        "local_ports": ports,
        "host_ip": host_lan_ip(),
        "suggested": suggest_mcu(ports),
    }


# ==============================================================================
# Core Status & Parameters APIs
# ==============================================================================

# The header pills poll /api/status every 4 s, so liveness is probed on a TTL
# and reused in between -- the probe shells out and must not run per request.
_LIVENESS_TTL_SEC = 6.0
_liveness_lock = threading.Lock()
_liveness_cache: Dict[str, Any] = {"ts": 0.0, "agent": False, "bringup": False, "where": "host", "detail": ""}


def get_exec_shell_cmd(cmd_str: str) -> List[str]:
    """A plain shell command on this machine (process probes need no ROS sourcing)."""
    return ["bash", "-lc", cmd_str]


def probe_stack_liveness() -> Dict[str, Any]:
    """
    Report whether the micro-ROS agent and Bringup are up.

    A plain `pgrep` is not enough: the agent is commonly a micro-ros-agent
    container, which does not show up as a process by that name, so the
    container engines are asked too -- the same way check_agent_port_status()
    does for the port-holder popover.
    """
    now = time.time()
    with _liveness_lock:
        fresh = (now - _liveness_cache["ts"]) < _LIVENESS_TTL_SEC
        first_run = _liveness_cache["ts"] == 0.0
        if fresh:
            return dict(_liveness_cache)
        if not first_run:
            # Stale but usable: refresh behind the poll rather than making the
            # header wait on the probe.
            if not _liveness_cache.get("refreshing"):
                _liveness_cache["refreshing"] = True
                threading.Thread(target=_refresh_liveness, daemon=True).start()
            return dict(_liveness_cache)
    return _refresh_liveness()


def _refresh_liveness() -> Dict[str, Any]:
    # The bracketed first letter keeps `pgrep -f` from matching the probe's own
    # shell: this command line contains "[m]icro_ros_agent", which the regex
    # (literal "micro_ros_agent") does not match. Without it every probe finds
    # itself and both pills read "running" forever.
    probe = (
        "echo '---AGENT---'; "
        "pgrep -fa '[m]icro_ros_agent' 2>/dev/null; "
        "{ docker ps --format '{{.Names}}|{{.Image}}|{{.Command}}' 2>/dev/null; "
        "  podman ps --format '{{.Names}}|{{.Image}}|{{.Command}}' 2>/dev/null; } "
        "  | grep -Ei 'micro[-_]ros[-_]agent' 2>/dev/null; "
        "echo '---BRINGUP---'; "
        "pgrep -fa '[b]ringup\\.launch\\.py|[e]kf_node|[r]obot_state_publisher' 2>/dev/null; "
        "true"
    )

    agent = bringup = False
    detail = ""
    where = "host"
    try:
        cmd = get_exec_shell_cmd(probe)
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
        section = ""
        for line in res.stdout.splitlines():
            line = line.strip()
            if line == "---AGENT---":
                section = "agent"
                continue
            if line == "---BRINGUP---":
                section = "bringup"
                continue
            if not line:
                continue
            if section == "agent":
                agent = True
                if not detail:
                    detail = line[:120]
            elif section == "bringup":
                bringup = True
    except Exception as e:
        detail = f"probe failed: {e}"

    with _liveness_lock:
        _liveness_cache.update({
            "ts": time.time(), "agent": agent, "bringup": bringup,
            "where": where, "detail": detail, "refreshing": False,
        })
        return dict(_liveness_cache)
























def pio_present() -> bool:
    """Is PlatformIO on this machine? Decides build-here versus release image."""
    for cand in (shutil.which("pio"), os.path.expanduser("~/.local/bin/pio"),
                 os.path.expanduser("~/.pioenv/bin/pio"),
                 os.path.expanduser("~/.platformio/penv/bin/pio")):
        if cand and os.path.isfile(cand):
            return True
    return False
















def _read_robot_file(fpath: str):
    """(declared_name, description, controller, mcu) from a config, or None."""
    try:
        with open(fpath, "r") as f:
            yd = yaml.safe_load(f) or {}
    except Exception:
        return None
    if not isinstance(yd, dict):
        return None
    r_info = yd.get("robot") or {}
    if not isinstance(r_info, dict):
        r_info = {}
    ctrl = yd.get("base_controller") or {}
    if not isinstance(ctrl, dict):
        ctrl = {}
    controller = ctrl.get("name") or "pico2"
    return (r_info.get("name"), r_info.get("description"),
            controller, ctrl.get("mcu", controller))


def _robot_candidates(config_dir: str):
    """Every file in the config dir that is a robot, in name order.

    A dotfile is not a robot: the cockpit keeps its own state here
    (.active_robot, .cockpit_token) and bench scripts leave things behind, and
    offering one lets a click select a file that was never a config.
    """
    if not os.path.isdir(config_dir):
        return []
    out = []
    for fname in sorted(os.listdir(config_dir)):
        if not (fname.endswith(".yaml") or fname.endswith(".yml")):
            continue
        if fname.startswith(".") or fname in ("secrets.yaml", "secrets.yaml.example"):
            continue
        out.append(fname)
    return out


def get_robots_list(params: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
    """Every robot config in the directory, identified by its CONTENT.

    A robot is what its file SAYS it is: `robot.name` inside the YAML is the
    identity, and the filename is only where that content happens to live. The
    stem is the fallback for a file that declares no name at all.

    Two files can therefore claim one name, and that is a conflict the user has
    to see -- not something for this function to resolve quietly. It used to key
    a `seen` set on the name and `continue` past a repeat: no log, no warning,
    the file simply was not in the list. Measured on a bench box 2026-09-21,
    three of nineteen configs were invisible in the cockpit, including the
    real-hardware config the CLI was driving at the time -- because
    pico2_real_config.yaml, pico2_realhw_config.yaml and pico2w_real_config.yaml
    all declared `robot: {name: pico2_real}`. The CLI resolves a path from
    `--robot <stem>` and never lost them, so the two halves of the product
    disagreed about which robots existed and only the UI came up short.

    So: one entry per FILE, named by its content, with `conflict` listing the
    other files making the same claim. `select` is the handle that unambiguously
    reaches THIS file -- the declared name when it is unique, the filename stem
    otherwise, both of which /api/robot/select accepts.
    """
    config_dir = CONFIG_DIR
    active_path = get_active_params_path()
    active_robot_name = ACTIVE_ROBOT_NAME
    if params:
        active_robot_name = params.get("robot", {}).get("name", active_robot_name)

    files = _robot_candidates(config_dir)
    read = {}
    for fname in files:
        info = _read_robot_file(os.path.join(config_dir, fname))
        if info is not None:
            read[fname] = info

    def stem_of(fname):
        return re.sub(r"_config\.ya?ml$|\.ya?ml$", "", fname)

    # Identity comes from the content; the stem only stands in for a file that
    # declares no name.
    identity = {f: (read[f][0] or stem_of(f)) for f in read}
    claims = {}
    for fname, name in identity.items():
        claims.setdefault(name, []).append(fname)

    robot_list = []
    for fname in files:
        if fname not in read:
            continue
        declared, desc, controller_name, mcu = read[fname]
        fpath = os.path.join(config_dir, fname)
        r_name = identity[fname]
        rivals = [f for f in claims[r_name] if f != fname]

        # The file wins outright when it is the one actually open. Otherwise fall
        # back to the name -- but only when that name is unambiguous, or the
        # wrong file of a colliding pair would light up as active.
        is_active = os.path.realpath(fpath) == os.path.realpath(active_path)
        if not is_active and not rivals and active_robot_name:
            is_active = (r_name == active_robot_name
                         or stem_of(fname) == active_robot_name)

        robot_list.append({
            "name": r_name,
            "description": desc or f"Robot {r_name}",
            "mcu": mcu,
            "controller": controller_name,
            "active": is_active,
            "path": display_path(fpath),
            "filename": fname,
            # The unambiguous handle for THIS file. Equal to `name` in the normal
            # case; a colliding file is reachable only by its stem.
            "select": r_name if not rivals else stem_of(fname),
            # Present only when another file claims the same name, so the UI can
            # show the clash instead of the user losing a robot to it.
            "conflict": rivals or None,
        })

    robot_list.sort(key=lambda r: (not r["active"], r["name"]))
    return robot_list














































































































































def get_ros_exec_cmd(cmd_str: str, distro: str = "jazzy") -> List[str]:
    """A ROS 2 command on this machine, with the installed ROS sourced.

    The PlatformIO venv dirs go last on PATH on purpose: a venv bin dir also
    owns `python3`, and reached through it Python loses the system
    dist-packages (numpy, and with it rclpy.node).
    """
    path_export = ("export PATH=$HOME/.local/bin:/usr/local/bin:$PATH:"
                   "$HOME/.pioenv/bin:$HOME/.platformio/penv/bin")
    return ["bash", "-lc", f"{path_export}; {ros_setup_shell(distro)} && {cmd_str}"]


































# Public surface: names the routes_* modules import from core.
__all__ = [
    "active_robot_name",
    "CONFIG_DIR",
    "REPO_ROOT",
    "RUNNERS",
    "SECRETS_EXAMPLE_PATH",
    "SECRETS_PATH",
    "access",
    "actions",
    "agent_runner",
    "analyze_robotics_ai",
    "app",
    "bringup_runner",
    "build_base_install_cmd",
    "build_ros2_install_cmd",
    "build_sensor_install_cmd",
    "check_agent_port_status",
    "check_bringup_health",
    "check_container_status",
    "cockpit_paths",
    "disable_autostart",
    "display_path",
    "enable_autostart",
    "fetch_prebuilt",
    "gamepad_runner",
    "gen_robot_description",
    "generate_custom_robot_specs",
    "get_active_params_path",
    "get_autostart_logs",
    "get_autostart_status",
    "get_controller",
    "get_controller_name",
    "get_git_info_dict",
    "get_package_install_info",
    "get_robots_list",
    "get_rootless_info",
    "get_ros_exec_cmd",
    "get_sensor_driver_status",
    "get_serial_ports_dict",
    "host_lan_ip",
    "hw_detect",
    "hw_flash",
    "hw_list_serial_ports",
    "hw_resolve_artifact",
    "hw_tool_status",
    "install_container_engine",
    "json_body",
    "laser_runner",
    "list_dir",
    "list_local_ports",
    "list_robot_config_files",
    "load_params",
    "main_runner",
    "mcu_identity",
    "nav2_stack_status",
    "one_click_pipeline",
    "patcher",
    "pin_catalog",
    "pio_present",
    "probe_stack_liveness",
    "regenerate_firmware_headers",
    "regenerate_robot_description",
    "release_agent_port",
    "require_header",
    "resolve_command",
    "robot_stack",
    "ros_setup_shell",
    "save_params",
    "sensor_registry",
    "set_active_params_path",
    "setup_rootless_docker",
    "suggest_mcu",
    "syslog_manager",
    "text_field",
    "yaml_merge",
]
