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
import copy
import glob
import json
import os
import queue
import re
import shlex
import shutil
import signal
import socket
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
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

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
        # An action name that is not in the registry is not a command -- fall
        # through to raw/command handling, which is where the label used to live.
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


@app.get("/api/status")
def get_status(controller: Optional[str] = None):
    params = load_params()
    robot = params.get("robot", {})
    git = get_git_info_dict()

    syslog_stat = syslog_manager.get_status()
    ports_info = get_serial_ports_dict(params)

    controller_hint = (controller or "").strip()
    controller = get_controller(params)
    # Whether anything was actually found on the bus, kept separate from the
    # fallback. The two used to be indistinguishable downstream: with no board
    # detected the UI still printed "Auto-Detected MCU: Controller: pico2" --
    # a configured name presented as a probe result -- above a stale VID:PID
    # line from the previous poll. That happens routinely, because a board in
    # BOOTSEL has no tty, which is exactly when it is being flashed.
    mcu_detected = False
    detected_mcu = controller.get("name") or "pico2"
    detected_chip = "Controller: " + detected_mcu
    for lp in ports_info.get("local_ports", []):
        hint = lp.get("mcu_hint")
        if hint:
            detected_mcu = hint
            detected_chip = lp.get("chip", hint)
            mcu_detected = True
            break

    # Does the bus positively contradict the configured controller? Answered
    # here rather than in the browser so there is one vid/pid table in the
    # project (scripts/mcu_identity.py) and the warning can never disagree with
    # the guard that refuses the flash.
    mcu_mismatch = None
    if mcu_detected:
        # Judge what 1-Click will ACTUALLY run. The browser passes its live
        # selection, which can differ from the config's own base_controller --
        # a Reference Build preset changes the former and not the latter -- and
        # warning about the config while the run uses something else is a false
        # alarm in one direction and silence in the other.
        want_name = (controller_hint or controller.get("name") or "")
        want = mcu_identity.env_family(want_name)
        got = mcu_identity.env_family(detected_mcu)
        for lp in ports_info.get("local_ports", []):
            if lp.get("mcu_hint") == detected_mcu:
                got = got or ""
                decisive = bool(lp.get("decisive"))
                if mcu_identity.mismatch(want, got, decisive):
                    mcu_mismatch = {
                        "expected": mcu_identity.FAMILY_LABEL.get(want, want),
                        "detected": mcu_identity.FAMILY_LABEL.get(got, got),
                        "controller": want_name,
                        "chip": detected_chip,
                    }
                break

    ctrl_sensors = controller.get("sensors", {}) or {}
    fake_mode_active = bool(
        ctrl_sensors.get("use_fake_wheel") or
        ctrl_sensors.get("use_fake_imu") or
        ctrl_sensors.get("use_fake_ld19")
    )

    # Ask the same list the bringup command itself sources, or the two disagree:
    # the container image ships its workspace at /opt/lino_ws/setup.bash, which
    # this check used to miss entirely, so the Bringup tab declared the workspace
    # unbuilt and tried to colcon build a directory that is not there.
    ws_setup = one_click_pipeline.workspace_setup()
    ws_built = bool(ws_setup)
    ws_path = (os.path.dirname(os.path.dirname(ws_setup))
               if ws_setup.endswith(os.path.join("install", "setup.bash"))
               else os.path.dirname(ws_setup)) if ws_setup else \
        os.path.abspath(os.path.join(REPO_ROOT, "..", ".."))

    liveness = probe_stack_liveness()
    agent_external = liveness["agent"]
    bringup_external = liveness["bringup"]

    controller_name = controller.get("name") or "pico2"
    active_path_rel = display_path(get_active_params_path())

    # The installed ROS wins over the configured name, for the same reason the
    # I2C bus wins over the configured IMU (§5): the YAML is a CLAIM about the
    # computer and /opt/ros is the computer. `ros_distro` is not a property of
    # the robot at all -- it belongs to whichever machine runs the stack -- so a
    # config pinning `jazzy` that is opened on a lyrical-only box was announcing
    # "Jazzy (Noble 24.04)" in the header while every node on the box came from
    # /opt/ros/lyrical, and the payload saying so carried the agent's own
    # `/opt/ros/lyrical/bin/ros2` command line two keys further down.
    #
    # That is not cosmetic: "Start 1-Click" reads this selector, so the button
    # would have run a JAZZY pipeline against a box with no jazzy installed --
    # the §12 class of bug where the UI and the stack disagree and nothing says
    # which is right. The configured value is honoured whenever it is actually
    # installed, so a machine carrying both distros still does what the config
    # asks; it is overridden only when it names a distro that is not there, and
    # the override is reported rather than silently applied.
    installed = [d for d in ("jazzy", "lyrical", "rolling")
                 if os.path.exists(os.path.join("/opt/ros", d))]
    configured = params.get("ros_distro")
    distro_override = None
    if configured and configured in installed:
        active_distro = configured
    elif installed:
        active_distro = installed[0]
        if configured:
            distro_override = (f"config says {configured}, which is not installed here; "
                               f"using {active_distro}")
    else:
        active_distro = configured or os.environ.get("ROS_DISTRO", "jazzy")

    return {
        "status": "online",
        "robot_name": robot.get("name", ACTIVE_ROBOT_NAME),
        "controller": controller_name,
        "base_controller": controller,
        "robot_config_path": active_path_rel,
        "robots": get_robots_list(params),
        "detected_mcu": detected_mcu,
        "detected_chip": detected_chip,
        "mcu_detected": mcu_detected,
        "mcu_mismatch": mcu_mismatch,
        "host_ip": ports_info.get("host_ip", ""),
        "config_dir": display_path(CONFIG_DIR),
        "fake_mode_active": fake_mode_active,
        "ros_distro": active_distro,
        "ros_distro_configured": configured,
        "ros_distro_override": distro_override,
        "ros_distro_installed": installed,
        "supported_distros": ["jazzy", "lyrical", "rolling"],
        "ros2_installed": True,
        "workspace_path": ws_path,
        "workspace_built": ws_built,
        "agent_alive_external": agent_external,
        "bringup_alive_external": bringup_external,
        "liveness": liveness,
        "repo_root": REPO_ROOT,
        "web_dir": os.path.join(REPO_ROOT, "web"),
        "git": git,
        "git_branch": git.get("branch", ""),
        "ports": ports_info,
        "config": params,
        "syslog": syslog_stat,
        "is_rootless": syslog_manager.is_rootless,
        "agent_busy_console": agent_runner.is_busy(),
        "bringup_busy_console": bringup_runner.is_busy(),
        "main_busy": main_runner.is_busy(),
        "laser_busy": laser_runner.is_busy(),
        "gamepad_running": gamepad_runner.is_running(),
    }


@app.get("/api/gitinfo")
def api_gitinfo():
    return get_git_info_dict()


@app.get("/api/gitinfo/branch")
@app.post("/api/gitinfo/branch")
async def api_gitinfo_branch(request: Request):
    data = {}
    if request.method == "POST":
        try:
            data = await json_body(request)
        except Exception:
            pass
    branch = text_field(data, "branch") or (request.query_params.get("branch") or "").strip()
    if not branch or not re.match(r"^[A-Za-z0-9._/-]+$", branch):
        raise HTTPException(
            status_code=400,
            detail=f"Invalid branch name: '{branch}'. Use letters, numbers, dot, underscore, slash, hyphen."
        )

    def event_generator():
        try:
            current_branch = subprocess.check_output(
                ["git", "branch", "--show-current"], cwd=REPO_ROOT, text=True, stderr=subprocess.DEVNULL
            ).strip()
        except Exception:
            current_branch = ""

        if current_branch == branch:
            yield f"data: {json.dumps({'line': f'Already on branch {branch}'})}\n\n"
            yield f"data: {json.dumps({'exit_code': 0, 'branch': branch})}\n\n"
            return

        # Check if local branch exists
        check_local = subprocess.run(
            ["git", "rev-parse", "--verify", f"refs/heads/{branch}"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        if check_local.returncode == 0:
            cmd = ["git", "checkout", branch]
        else:
            # Check if remote branch exists
            check_remote = subprocess.run(
                ["git", "rev-parse", "--verify", f"refs/remotes/origin/{branch}"],
                cwd=REPO_ROOT,
                capture_output=True,
                text=True,
            )
            if check_remote.returncode == 0:
                cmd = ["git", "checkout", "-b", branch, "--track", f"origin/{branch}"]
            else:
                # Create a new local branch from current HEAD
                cmd = ["git", "checkout", "-b", branch]

        cmd_str = " ".join(cmd)
        yield f"data: {json.dumps({'line': f'$ {cmd_str}'})}\n\n"

        proc = subprocess.Popen(
            cmd,
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        has_output = False
        if proc.stdout:
            for line in iter(proc.stdout.readline, ""):
                line_str = line.rstrip("\r\n")
                if line_str:
                    has_output = True
                    yield f"data: {json.dumps({'line': line_str})}\n\n"
            proc.stdout.close()
        proc.wait()

        if proc.returncode == 0 and not has_output:
            yield f"data: {json.dumps({'line': f'Switched to branch {branch}'})}\n\n"

        yield f"data: {json.dumps({'exit_code': proc.returncode, 'branch': branch})}\n\n"

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.get("/api/serial_ports")
def api_serial_ports():
    sp = get_serial_ports_dict()
    # Both shapes: the flat list for the chip pickers, the dicts for the rest.
    return {"ports": [p["path"] for p in sp["local_ports"]], **sp}


@app.get("/api/mcu/ports")
def api_mcu_ports():
    """The USB serial devices on this machine, and the MCU each one looks like."""
    ports = list_local_ports()
    return {
        "status": "ok",
        "host_ip": host_lan_ip(),
        "hostname": socket.gethostname(),
        "ports": ports,
        "suggested": suggest_mcu(ports),
    }


@app.get("/api/config")
def api_config():
    return load_params()


@app.post("/api/config")
async def api_save_config(request: Request):
    data = await json_body(request)
    params = load_params()
    if isinstance(data, dict):
        for k, v in data.items():
            params[k] = v
        save_params(params)
    return {"success": True, "config": params}


@app.get("/api/hardware/config")
def api_get_hardware_config():
    params = load_params()
    controller = get_controller(params)
    return {
        "robot_name": params.get("robot", {}).get("name", "linorobot2"),
        "controller": controller.get("name", "pico2"),
        "kinematics": params.get("kinematics", {}),
        # The body, with every gap filled from the kinematics, so the form is
        # never blank; what the user saves is what the URDF is built from.
        "geometry": gen_robot_description.effective_geometry(params),
        "geometry_warnings": gen_robot_description.geometry_warnings(params),
        "base_controller": controller,
        # What the pin catalogue thinks of this config (scripts/pin_catalog.py):
        # the Pin Matrix shows these next to the fields they concern.
        "pin_findings": [{"level": l, "message": m} for l, m in pin_catalog.check_config(params)],
    }


@app.post("/api/hardware/config")
async def api_save_hardware_config(request: Request):
    data = await json_body(request)
    params = load_params()

    ctrl = params.setdefault("base_controller", {})
    # The robot file names exactly one controller; a posted name renames it
    # (e.g. switching this robot from pico2 to esp32), it does not add a second.
    controller_name = data.get("controller") or ctrl.get("name") or "pico2"
    ctrl["name"] = controller_name

    if "kinematics" in data and isinstance(data["kinematics"], dict):
        params.setdefault("kinematics", {}).update(data["kinematics"])
    if "geometry" in data and isinstance(data["geometry"], dict):
        geo = params.setdefault("geometry", {})
        for section, values in data["geometry"].items():
            if isinstance(values, dict):
                geo.setdefault(section, {}).update(values)
            else:
                geo[section] = values

    if "driver_type" in data:
        ctrl["driver_type"] = data["driver_type"]
    if "baudrate" in data:
        ctrl["baudrate"] = int(data["baudrate"])
    if "serial_port" in data:
        ctrl["serial_port"] = str(data["serial_port"])
    # The robot computer's address, entered by the user like `host`/`host_ip`.
    # Stored even when blank, because blank is a meaningful value: it means
    # "this is the rig, resolve the box". Written through str() and stripped so
    # a pasted address with stray whitespace cannot become an unreachable URL.
    if "mcu" in data:
        ctrl["mcu"] = data["mcu"]
    if "sensors" in data and isinstance(data["sensors"], dict):
        ctrl.setdefault("sensors", {}).update(data["sensors"])
    if "pins" in data and isinstance(data["pins"], dict):
        ctrl.setdefault("pins", {}).update(data["pins"])
    if "base_controller" in data and isinstance(data["base_controller"], dict):
        ctrl.update(data["base_controller"])

    save_params(params)
    findings = [{"level": l, "message": m} for l, m in pin_catalog.check_config(params)]
    res = regenerate_firmware_headers(controller_name)
    header_ok = res.returncode == 0
    description = regenerate_robot_description(params)
    errors = sum(1 for f in findings if f["level"] == "error")
    if header_ok:
        message = f"Hardware config saved and header regenerated for [{controller_name}]."
    elif errors:
        message = (f"Hardware config saved, but the header was NOT regenerated: {errors} pin "
                   f"error(s) for [{controller_name}]. Fix them below and save again.")
    else:
        message = f"Hardware config saved, but the header generator failed for [{controller_name}]."
    return {
        "success": True,
        "controller": controller_name,
        "header_ok": header_ok,
        "message": message,
        "header_stdout": res.stdout,
        "header_stderr": res.stderr,
        "pin_findings": findings,
        **description,
    }


@app.get("/api/robot/urdf")
def api_robot_urdf():
    """The description the next bringup will publish, as text."""
    params = load_params()
    return PlainTextResponse(gen_robot_description.build_urdf(params), media_type="application/xml")


@app.post("/api/hardware/fake_mode")
async def api_toggle_fake_mode(request: Request):
    data = await json_body(request)
    enabled = bool(data.get("enabled", False))
    params = load_params()
    ctrl = params.setdefault("base_controller", {})
    controller_name = data.get("controller") or ctrl.get("name") or "pico2"

    sensors = ctrl.setdefault("sensors", {})
    sensors["use_fake_wheel"] = enabled
    sensors["use_fake_imu"] = enabled
    sensors["use_fake_ld19"] = enabled
    sensors["use_fake_mag"] = enabled
    if "use_fake_env" in sensors or not enabled:
        sensors["use_fake_env"] = enabled

    save_params(params)
    res = regenerate_firmware_headers(controller_name)
    return {
        "success": True,
        "enabled": enabled,
        "controller": controller_name,
        "message": f"Fake simulation mode {'enabled' if enabled else 'disabled'} for [{controller_name}].",
        "header_stdout": res.stdout,
    }


def pio_present() -> bool:
    """Is PlatformIO on this machine? Decides build-here versus release image."""
    for cand in (shutil.which("pio"), os.path.expanduser("~/.local/bin/pio"),
                 os.path.expanduser("~/.pioenv/bin/pio"),
                 os.path.expanduser("~/.platformio/penv/bin/pio")):
        if cand and os.path.isfile(cand):
            return True
    return False


@app.post("/api/hardware/test")
async def api_hardware_test(request: Request):
    data = await json_body(request)
    firmware = data.get("firmware") or data.get("target") or "test_sensors"
    port = data.get("port", "/dev/ttyACM0")
    baud = int(data.get("baud", 921600))
    action = data.get("action", "upload")
    # The Sensors tab fills cfg-mcu from the USB probe, which answers with a
    # BOARD ("gendrv" for a CP2102N). Flashing needs a PlatformIO env, and
    # asking for "gendrv" sent fetch_prebuilt after a release artifact that
    # does not exist -- firmware upload 404ed on the one ESP32 board this
    # project ships a reference design for.
    mcu_env = mcu_identity.pio_env_for(data.get("mcu_env"), "pico2")

    if action in ("upload", "monitor"):
        try:
            if agent_runner.is_busy():
                agent_runner.kill()
        except Exception as e:
            print(f"[api_hardware_test] agent stop notice: {e}")
        try:
            release_agent_port(port=port)
        except Exception as e:
            print(f"[api_hardware_test] release_agent_port notice: {e}")

    # One image now carries every application, so there is no per-tool directory
    # to point at: `firmware` is always the project, and which application boots
    # is the `app` key in the env partition.
    firmware_dir = "firmware"
    require_header(regenerate_firmware_headers(mcu_env))
    params_path = get_active_params_path()

    if action == "build":
        # The published robot image deliberately ships no PlatformIO -- it
        # flashes the release artifact instead -- so Build on a stock install
        # ended in a bare `bash: line 1: pio: command not found` and exit 127,
        # which says nothing about why or what to do. Answer the question the
        # user is actually asking.
        if not pio_present():
            raise HTTPException(
                status_code=400,
                detail=("PlatformIO is not installed here, and this image does not ship "
                        "it: Build compiles from source, which a robot does not need. "
                        "Use Flash MCU — it fetches the release image for this board — "
                        "or run the build in the pio container: "
                        "`docker compose run --rm pio pio run -e %s`." % mcu_env))
        cmd = f"pio run -d {firmware_dir} -e {mcu_env}"
    elif action == "upload":
        # Build and flash stay separate: pio only compiles, esptool/picotool
        # writes. With PlatformIO on this machine the image is built first;
        # without it the release image for this env is fetched and flashed.
        # One image carries base AND every diagnostic tool (firmware tools.h,
        # the "swiss knife"): the `app` env key selects which boots, so flashing
        # `adc_calibrate` writes the same base image and sets app=adc_calibrate.
        # No per-tool binary, and no toolchain needed on the robot.
        flash_script = os.path.join(REPO_ROOT, "scripts", "flash_mcu.py")
        if firmware != "base":
            # Switching to a diagnostic tool is the swiss knife's whole point: the
            # installed image already carries every tool, so this is a 4 KB env
            # write (app=<tool>) and a reboot -- no download, no reflash. Flash
            # the base firmware once; then switch blades freely. If the board
            # runs an image too old to have the tool, it boots base with a
            # message rather than the tool (firmware tools.cpp), so this is safe.
            cmd = (
                f"python3 {flash_script} --env {mcu_env} --env-only "
                f"--port {port} "
                f"--params {shlex.quote(params_path)} "
                f"--app {firmware} "
                f"--baud {baud}"
            )
        else:
            image = (f"--firmware-dir {firmware_dir} --env {mcu_env} --build"
                     if pio_present() else
                     f"--prebuilt {fetch_prebuilt.profile_for_env(mcu_env)}")
            fetch = ("" if pio_present() else
                     f"python3 {os.path.join(REPO_ROOT, 'scripts', 'fetch_prebuilt.py')} {mcu_env} && ")
            cmd = (
                f"{fetch}python3 {flash_script} {image} "
                f"--port {port} "
                f"--params {shlex.quote(params_path)} "
                f"--firmware-name {firmware} "
                f"--app {firmware} "
                f"--baud {baud}"
            )
    elif action == "monitor":
        # NOT miniterm. It builds a Console() in its constructor, which calls
        # termios.tcgetattr() on stdin, and every command here runs on a PIPE --
        # so Monitor failed with `termios.error: (25, 'Inappropriate ioctl for
        # device')` every single time, and the diagnostic applications had no
        # way to show their output at all. scripts/serial_monitor.py reads the
        # port and writes lines to stdout, which is what the SSE runner
        # forwards.
        cmd = (
            f"echo '=== [1/2] Releasing serial port {port} ===' && "
            f"lsof -ti {port} 2>/dev/null | xargs -r kill -9 2>/dev/null || true; "
            f"sleep 0.5; "
            f"echo '=== [2/2] Streaming {port} @ {baud} ===' && "
            f"python3 -u {os.path.join(REPO_ROOT, 'scripts', 'serial_monitor.py')} "
            f"{shlex.quote(port)} {baud}"
        )
    else:
        raise HTTPException(status_code=400, detail=f"Unknown action: {action}")

    return {"command": cmd, "handle": actions.prepare(cmd), "firmware": firmware, "port": port, "mcu_env": mcu_env}


@app.get("/api/params")
def api_params():
    return load_params()


@app.get("/api/params/raw", response_class=PlainTextResponse)
def get_params_raw():
    active_path = get_active_params_path()
    if not os.path.isfile(active_path):
        return ""
    with open(active_path, "r") as f:
        return f.read()


@app.post("/api/params/raw")
async def update_params_raw(request: Request):
    body = await request.body()
    text = body.decode("utf-8")
    try:
        yaml.safe_load(text)
        active_path = get_active_params_path()
        with open(active_path, "w") as f:
            f.write(text)
        res = regenerate_firmware_headers(params_path=active_path)
        return {
            "success": True,
            "message": f"Parameters saved to {os.path.basename(active_path)} and firmware header generated.",
            "generator_stdout": res.stdout if res else "",
            "header_ok": bool(res and res.returncode == 0),
            "generator_stderr": (res.stderr if res else "") or "",
            "active_path": display_path(active_path),
        }
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.post("/api/params")
def update_params(data: Dict[str, Any]):
    try:
        save_params(data)
        res = regenerate_firmware_headers()
        return {
            "success": True,
            "message": "Parameters saved and firmware header generated.",
            "generator_stdout": res.stdout,
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/robot_config")
def get_robot_config(distro: str = "jazzy"):
    active_path = get_active_params_path()
    raw_yaml = ""
    if os.path.isfile(active_path):
        with open(active_path, "r") as f:
            raw_yaml = f.read()
    params = load_params(active_path)
    kine = params.get("kinematics", {})
    robot = params.get("robot", {})
    controller = get_controller(params)
    rel_path = display_path(active_path)
    return {
        "success": True,
        "path": rel_path,
        "yaml": raw_yaml,
        "robot": robot,
        "kinematics": kine,
        "base_controller": controller,
        "base": kine.get("base_type", "2wd"),
        "linorobot2": {
            "base": kine.get("base_type", "2wd"),
            "laser_sensor": str((controller.get("lidar") or {}).get("model") or "none"),
            "micro_ros_port": controller.get("serial_port", ""),
            "micro_ros_baudrate": str(controller.get("baudrate", "")),
        }
    }


@app.post("/api/robot_config")
async def save_robot_config(request: Request):
    content_type = request.headers.get("content-type", "")
    if "application/json" in content_type:
        data = await json_body(request)
        raw_yaml = data.get("yaml", "")
    else:
        body = await request.body()
        raw_yaml = body.decode("utf-8")
    if raw_yaml:
        yaml.safe_load(raw_yaml)
        active_path = get_active_params_path()
        with open(active_path, "w") as f:
            f.write(raw_yaml)
        res = regenerate_firmware_headers(params_path=active_path)
        rel_path = display_path(active_path)
        return {"success": True, "status": "saved", "path": rel_path, "message": f"Saved {rel_path}",
                "generator_stdout": res.stdout if res else "",
                "header_ok": bool(res and res.returncode == 0),
                "generator_stderr": (res.stderr if res else "") or ""}
    raise HTTPException(status_code=400, detail="Empty YAML")


def get_robots_list(params: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
    config_dir = CONFIG_DIR
    active_path = get_active_params_path()
    active_robot_name = ACTIVE_ROBOT_NAME
    if params:
        active_robot_name = params.get("robot", {}).get("name", active_robot_name)

    robot_list = []
    seen = set()

    if os.path.isdir(config_dir):
        candidates = sorted(os.listdir(config_dir))
        for fname in candidates:
            if not (fname.endswith(".yaml") or fname.endswith(".yml")):
                continue
            if fname in ("secrets.yaml", "secrets.yaml.example"):
                continue
            fpath = os.path.join(config_dir, fname)
            try:
                with open(fpath, "r") as f:
                    yd = yaml.safe_load(f) or {}
            except Exception:
                continue

            r_info = yd.get("robot", {})
            r_name = r_info.get("name")
            if not r_name:
                r_name = fname.replace("_config.yaml", "").replace("_config.yml", "").replace(".yaml", "").replace(".yml", "")

            if r_name in seen:
                continue
            seen.add(r_name)

            desc = r_info.get("description", f"Robot {r_name}")
            ctrl_info = yd.get("base_controller", {})
            if not isinstance(ctrl_info, dict):
                ctrl_info = {}
            controller_name = ctrl_info.get("name") or "pico2"
            mcu = ctrl_info.get("mcu", controller_name)

            is_active = (os.path.realpath(fpath) == os.path.realpath(active_path)) or (r_name == active_robot_name)

            robot_list.append({
                "name": r_name,
                "description": desc,
                "mcu": mcu,
                "controller": controller_name,
                "active": is_active,
                "path": display_path(fpath),
                "filename": fname,
            })

    robot_list.sort(key=lambda r: (not r["active"], r["name"]))
    return robot_list


@app.get("/api/config/git")
def api_config_git():
    """The config directory's git state, for the Config Studio header.

    Deliberately not folded into /api/status: the browser polls that every couple
    of seconds, and `git status` on a bind-mounted directory is not free. The UI
    asks for this when the tab opens and after a save.
    """
    return {"status": "ok", **cockpit_paths.git_state(CONFIG_DIR),
            "path": display_path(CONFIG_DIR)}


@app.post("/api/config/commit")
async def api_config_commit(request: Request):
    """Commit the user's robot configs. Their repository, their history."""
    try:
        data = await json_body(request)
    except Exception:
        data = {}
    message = text_field(data, "message")
    if not message:
        message = f"Update {ACTIVE_ROBOT_NAME} from the Cockpit"
    res = cockpit_paths.git_commit(message, CONFIG_DIR)
    if res.get("state"):
        res["state"]["path"] = display_path(CONFIG_DIR)
    return res


@app.get("/api/robots")
def get_robots():
    active_path = get_active_params_path()
    params = load_params(active_path)
    robot = params.get("robot", {})
    robot_name = robot.get("name", ACTIVE_ROBOT_NAME)
    return {
        "status": "ok",
        "robots": get_robots_list(params),
        "active": robot_name,
        "path": display_path(active_path),
    }


@app.post("/api/robot/select")
async def select_robot(request: Request):
    data = await json_body(request)
    name = text_field(data, "name", "robot")
    if not name or not re.match(r"^[a-zA-Z0-9_.-]+$", name):
        raise HTTPException(
            status_code=400,
            detail=f"Invalid robot name: '{name}'. Use lowercase, digits, and underscores.",
        )

    config_dir = CONFIG_DIR
    target_yaml = None

    # 1. Match by exact config filename e.g. pico2_mecanum_config.yaml or rover_pico2.yaml
    for candidate in [f"{name}_config.yaml", f"{name}.yaml", f"{name}_config.yml", f"{name}.yml"]:
        p = os.path.join(config_dir, candidate)
        if os.path.isfile(p):
            target_yaml = p
            break

    # 2. Match by robot.name declared inside any YAML in the config dir
    if not target_yaml:
        for fname in os.listdir(config_dir):
            if not (fname.endswith(".yaml") or fname.endswith(".yml")) or fname.startswith("secrets"):
                continue
            fpath = os.path.join(config_dir, fname)
            try:
                with open(fpath, "r") as f:
                    yd = yaml.safe_load(f) or {}
                if yd.get("robot", {}).get("name") == name:
                    target_yaml = fpath
                    break
            except Exception:
                pass

    # 3. If new robot requested, create a dedicated <config dir>/{name}_config.yaml.
    #    The new file is seeded from the robot currently open and keeps exactly
    #    one base controller -- a robot is one controller, by definition.
    if not target_yaml:
        target_yaml = os.path.join(config_dir, f"{name}_config.yaml")
        base_params = load_params()
        new_params = copy.deepcopy(base_params)
        new_params.setdefault("robot", {})["name"] = name
        requested = data.get("controller") or data.get("mcu")
        ctrl = new_params.setdefault("base_controller", {})
        if requested:
            ctrl["name"] = requested
        ctrl.setdefault("name", "pico2")
        save_params(new_params, path=target_yaml)

    set_active_params_path(target_yaml)
    params = load_params(target_yaml)

    controller_name = get_controller_name(params)
    res = regenerate_firmware_headers(controller=controller_name, params_path=target_yaml)
    rel_path = display_path(target_yaml)

    return {
        "status": "ok",
        "active": name,
        "robot_name": name,
        "robot_config_path": rel_path,
        "robots": get_robots_list(params),
        "config": params,
        "message": f"Active robot set to '{name}' ({rel_path})",
        "generator_stdout": res.stdout if res else "",
    }


@app.post("/api/import_config")
async def import_config(request: Request):
    data = await json_body(request)
    raw_text = data.get("text", "")
    file_path = data.get("path", "")
    if file_path and not raw_text and os.path.isfile(file_path):
        with open(file_path, "r") as f:
            raw_text = f.read()

    if not raw_text:
        raise HTTPException(status_code=400, detail="No config text or valid path provided")

    # If full YAML provided
    if "robot:" in raw_text or "kinematics:" in raw_text or "linorobot2:" in raw_text:
        parsed = yaml.safe_load(raw_text)
        params = load_params()
        for k in ["robot", "kinematics", "base_controller", "ekf", "slam", "nav2"]:
            if k in parsed:
                params[k] = parsed[k]
        save_params(params)
        res = regenerate_firmware_headers()
        return {"status": "ok", "type": "yaml", "message": "Imported configuration YAML",
                "generator_stdout": res.stdout, "header_ok": res.returncode == 0,
                "generator_stderr": res.stderr or ""}

    return {"status": "ok", "message": "Imported config successfully"}


# ==============================================================================
# UDP Syslog Telemetry Endpoints (with Rootless Fallback)
# ==============================================================================
@app.get("/api/syslog/status")
def api_syslog_status():
    return syslog_manager.get_status()


@app.post("/api/syslog/start")
async def api_syslog_start(request: Request):
    data = {}
    try:
        data = await json_body(request)
    except Exception:
        pass
    req_port = data.get("port")
    try:
        res = syslog_manager.start(req_port)
        return res
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/syslog/stop")
def api_syslog_stop():
    return syslog_manager.stop()


@app.get("/api/syslog/logs")
def api_syslog_logs(tail: int = 50):
    return syslog_manager.get_logs_data(tail_count=tail)


@app.get("/api/syslog/stream")
def api_syslog_stream():
    q = queue.Queue(maxsize=100)
    syslog_manager.subscribe(q)

    def event_generator():
        try:
            init_status = syslog_manager.get_status()
            yield f"event: status\ndata: {json.dumps(init_status)}\n\n"
            while True:
                try:
                    entry = q.get(timeout=2.0)
                    yield f"event: syslog\ndata: {json.dumps(entry)}\n\n"
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            syslog_manager.unsubscribe(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


# ==============================================================================
# Secrets Management API (WiFi, Telemetry, micro-ROS quarantine)
# ==============================================================================
@app.get("/api/secrets")
def get_secrets():
    exists = os.path.isfile(SECRETS_PATH)
    path_to_read = SECRETS_PATH if exists else SECRETS_EXAMPLE_PATH
    raw_yaml = ""
    secrets_data = {}
    if os.path.isfile(path_to_read):
        with open(path_to_read, "r") as f:
            raw_yaml = f.read()
        try:
            secrets_data = yaml.safe_load(raw_yaml) or {}
        except Exception:
            secrets_data = {}

    # If syslog_port in secrets is 514 and system is rootless, note the active bound port
    bound_syslog = syslog_manager.bound_port
    return {
        "success": True,
        "exists": exists,
        "path": display_path(SECRETS_PATH) if exists else display_path(SECRETS_EXAMPLE_PATH),
        "secrets": secrets_data,
        "raw_yaml": raw_yaml,
        "is_rootless": syslog_manager.is_rootless,
        "bound_syslog_port": bound_syslog,
    }


@app.post("/api/secrets")
async def save_secrets(request: Request):
    content_type = request.headers.get("content-type", "")
    raw_yaml = ""
    secrets_data = {}
    if "application/json" in content_type:
        data = await json_body(request)
        if "raw_yaml" in data and isinstance(data["raw_yaml"], str) and data["raw_yaml"].strip():
            raw_yaml = data["raw_yaml"]
            secrets_data = yaml.safe_load(raw_yaml) or {}
        elif "secrets" in data and isinstance(data["secrets"], dict):
            secrets_data = data["secrets"]
            raw_yaml = yaml.dump(secrets_data, sort_keys=False, default_flow_style=False)
        else:
            secrets_data = data
            raw_yaml = yaml.dump(secrets_data, sort_keys=False, default_flow_style=False)
    else:
        body = await request.body()
        raw_yaml = body.decode("utf-8")
        secrets_data = yaml.safe_load(raw_yaml) or {}

    if not isinstance(secrets_data, dict):
        raise HTTPException(status_code=400, detail="Invalid YAML structure")

    os.makedirs(os.path.dirname(SECRETS_PATH), exist_ok=True)
    with open(SECRETS_PATH, "w") as f:
        f.write(raw_yaml)
    try:
        os.chmod(SECRETS_PATH, 0o600)
    except Exception:
        pass

    res = regenerate_firmware_headers()
    return {
        "success": True,
        "status": "saved",
        "path": display_path(SECRETS_PATH),
        "message": "Saved secrets.yaml and regenerated firmware headers",
        "generator_stdout": res.stdout,
    }


# ==============================================================================
# Process Execution & Slots (SSE Streaming)
# ==============================================================================
@app.post("/api/exec")
async def api_exec(request: Request):
    data = await json_body(request)
    slot = data.get("slot", "main")
    runner = RUNNERS.get(slot, main_runner)
    command = resolve_command(data)
    if not command:
        raise HTTPException(status_code=400, detail="No action or command given")
    if runner.is_busy():
        raise HTTPException(status_code=409, detail=f"Slot '{slot}' is busy")

    def event_generator():
        q = queue.Queue(maxsize=500)
        runner.subscribe(q)
        th = threading.Thread(
            target=runner.start_streaming,
            args=(command, REPO_ROOT),
            daemon=True,
        )
        th.start()
        try:
            while runner.is_busy() or not q.empty():
                try:
                    ev_type, payload = q.get(timeout=1.0)
                    yield f"event: {ev_type}\ndata: {json.dumps(payload)}\n\n"
                    if ev_type == "done":
                        break
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            runner.unsubscribe(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/api/kill")
async def api_kill(request: Request):
    data = await json_body(request)
    slot = data.get("slot", "main")
    runner = RUNNERS.get(slot, main_runner)
    killed = runner.kill()
    return {"status": "ok", "killed": killed, "slot": slot}


# micro-ROS Agent execution
@app.post("/api/agent/exec")
async def api_agent_exec(request: Request):
    data = await json_body(request)
    params = load_params()
    ctrl = get_controller(params)
    port = ctrl.get("serial_port", "/dev/ttyUSB0")
    baud = ctrl.get("baudrate", 1500000)
    default = f"ros2 run micro_ros_agent micro_ros_agent serial --dev {port} -b {baud}"
    command = resolve_command(data, default=default)

    if agent_runner.is_busy():
        raise HTTPException(status_code=409, detail="micro-ROS Agent slot is busy")

    def event_generator():
        q = queue.Queue(maxsize=500)
        agent_runner.subscribe(q)
        th = threading.Thread(
            target=agent_runner.start_streaming,
            args=(command, REPO_ROOT),
            daemon=True,
        )
        th.start()
        try:
            while agent_runner.is_busy() or not q.empty():
                try:
                    ev_type, payload = q.get(timeout=1.0)
                    yield f"event: {ev_type}\ndata: {json.dumps(payload)}\n\n"
                    if ev_type == "done":
                        break
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            agent_runner.unsubscribe(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/api/agent/kill")
def api_agent_kill():
    killed = agent_runner.kill()
    return {"status": "ok", "killed": killed}


@app.get("/api/agent/port_check")
@app.post("/api/agent/port_check")
async def api_agent_port_check(request: Request):
    if request.method == "POST":
        data = await json_body(request)
    else:
        data = dict(request.query_params)
    port = data.get("port", "/dev/ttyUSB0")
    mode = data.get("mode", "serial")
    udp_port = int(data.get("udp_port", 8888))
    host = data.get("host")
    user = data.get("user")
    return check_agent_port_status(port=port, mode=mode, udp_port=udp_port, host=host, user=user)


@app.post("/api/agent/port_release")
async def api_agent_port_release(request: Request):
    data = await json_body(request)
    port = data.get("port", "/dev/ttyUSB0")
    mode = data.get("mode", "serial")
    udp_port = int(data.get("udp_port", 8888))
    host = data.get("host")
    user = data.get("user")
    return release_agent_port(port=port, mode=mode, udp_port=udp_port, host=host, user=user)


# Bringup execution
@app.post("/api/bringup/exec")
async def api_bringup_exec(request: Request):
    data = await json_body(request)
    command = resolve_command(data, default="ros2 launch linorobot2_cockpit bringup.launch.py")

    if bringup_runner.is_busy():
        raise HTTPException(status_code=409, detail="Bringup slot is busy")

    def event_generator():
        q = queue.Queue(maxsize=500)
        bringup_runner.subscribe(q)
        th = threading.Thread(
            target=bringup_runner.start_streaming,
            args=(command, REPO_ROOT),
            daemon=True,
        )
        th.start()
        try:
            while bringup_runner.is_busy() or not q.empty():
                try:
                    ev_type, payload = q.get(timeout=1.0)
                    yield f"event: {ev_type}\ndata: {json.dumps(payload)}\n\n"
                    if ev_type == "done":
                        break
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            bringup_runner.unsubscribe(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/api/bringup/kill")
def api_bringup_kill():
    # Two ways a robot can be running. The bringup RUNNER is this backend's own
    # child, started from the Bringup tab. A 1-Click run is not: the pipeline
    # exits and leaves bringup, SLAM and Nav2 alive on purpose, so Stop has to
    # reach those too or the button lies. scripts/robot_stack.py is the record.
    killed = bringup_runner.kill()
    stopped = []
    try:
        stopped = robot_stack.stop()
    except Exception as exc:
        print(f"[api_bringup_kill] robot_stack notice: {exc}")
    return {"status": "ok", "killed": killed, "stack_stopped": stopped}


@app.get("/api/stack")
def api_stack():
    """What a 1-Click run left running, for the Stop buttons and the header."""
    try:
        entries = robot_stack.load()
    except Exception as exc:
        return {"status": "error", "detail": str(exc), "running": []}
    return {"status": "ok", "running": entries,
            "summary": robot_stack.describe()}


@app.post("/api/stack/stop")
async def api_stack_stop(request: Request):
    """Stop the whole kept-running stack, or one part of it."""
    data = await json_body(request)
    tag = (data.get("tag") or "").strip() or None
    if tag and tag not in ("bringup", "slam", "nav2"):
        raise HTTPException(status_code=400, detail=f"Unknown stack part: {tag}")
    try:
        stopped = robot_stack.stop(tag)
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))
    return {"status": "ok", "stopped": stopped}


@app.get("/api/bringup/stream")
def api_bringup_stream():
    def event_generator():
        q = queue.Queue(maxsize=500)
        bringup_runner.subscribe(q)
        try:
            for line in bringup_runner.get_history():
                yield f"event: output\ndata: {json.dumps({'line': line})}\n\n"
            while True:
                try:
                    ev_type, payload = q.get(timeout=2.0)
                    yield f"event: {ev_type}\ndata: {json.dumps(payload)}\n\n"
                    if ev_type == "done":
                        break
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            bringup_runner.unsubscribe(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.get("/api/bringup/health")
def api_bringup_health(timeout: float = 4.0):
    return check_bringup_health(timeout=timeout)


# ==============================================================================
# Remote 1-Click execution — the robot half of a run driven from a workstation
#
# AGENTS.md §6: the host workstation owns the config and the compiler, the ROBOT
# computer owns the serial bus and the ROS 2 runtime. When "Robot PC Address"
# names another machine, the firmware already travelled here over HTTP; these
# four routes carry everything AFTER the flash the same way, so bringup, SLAM,
# Nav2 and the map saver run on the machine the board is plugged into rather
# than on the workstation driving the run.
#
# HTTP and not ssh, for the reason §6 gives: ssh needs keys, an account and a
# shell on the robot that a user who installed the Cockpit has not necessarily
# set up, while this supervisor is already running, already owns the bus and is
# already the thing the browser talks to. One channel, one set of failure modes.
# ==============================================================================
# ==============================================================================
# Gamepad & Virtual Teleop Publisher API
# ==============================================================================
@app.post("/api/gamepad/start")
def api_gamepad_start():
    started = gamepad_runner.start()
    return {"status": "ok", "running": started}


@app.post("/api/gamepad/cmd")
async def api_gamepad_cmd(request: Request):
    data = await json_body(request)
    lx = float(data.get("linear_x", 0.0))
    ly = float(data.get("linear_y", 0.0))
    az = float(data.get("angular_z", 0.0))
    if not gamepad_runner.is_running():
        gamepad_runner.start()
    sent = gamepad_runner.send(lx, ly, az)
    return {"sent": sent, "running": gamepad_runner.is_running()}


@app.get("/api/gamepad/stall")
def api_gamepad_stall():
    return {"stalled": False, "running": gamepad_runner.is_running()}


@app.post("/api/gamepad/kill")
def api_gamepad_kill():
    killed = gamepad_runner.kill()
    return {"killed": killed}


# ==============================================================================
# Rootless Docker & Container Engine APIs
# ==============================================================================
@app.get("/api/docker/status")
def api_docker_status():
    return check_container_status()


@app.get("/api/docker/rootless_info")
def api_docker_rootless_info():
    return get_rootless_info()


@app.post("/api/docker/setup_rootless")
def api_docker_setup_rootless():
    return setup_rootless_docker()


@app.get("/api/container/install")
@app.post("/api/container/install")
async def api_container_install(request: Request, engine: Optional[str] = None):
    req_engine = engine
    if request.method == "POST":
        try:
            data = await json_body(request)
            if isinstance(data, dict):
                req_engine = data.get("engine") or req_engine
        except Exception:
            pass
    if not req_engine:
        req_engine = request.query_params.get("engine", "docker")
    return install_container_engine(req_engine)


# ==============================================================================
# Autostart APIs (systemd user unit)
# ==============================================================================
@app.get("/api/autostart/status")
def api_autostart_status():
    return get_autostart_status()


@app.post("/api/autostart/enable")
async def api_autostart_enable(request: Request):
    data = {}
    try:
        data = await json_body(request)
    except Exception:
        pass
    return enable_autostart(data)


@app.post("/api/autostart/disable")
def api_autostart_disable():
    return disable_autostart()


@app.get("/api/autostart/logs")
def api_autostart_logs():
    return get_autostart_logs()


# ==============================================================================
# Nav2 / EKF / SLAM Configuration & Presets (Single Source of Truth)
# ==============================================================================
@app.get("/api/presets")
def api_presets():
    return {"presets": patcher.PRESETS}


@app.post("/api/presets/apply")
async def api_presets_apply(request: Request):
    data = await json_body(request)
    preset_name = data.get("preset", "")
    if preset_name not in patcher.PRESETS:
        raise HTTPException(status_code=400, detail=f"Unknown preset '{preset_name}'")
    pinfo = patcher.PRESETS[preset_name]

    with open(get_active_params_path(), "r") as f:
        text = f.read()

    # 1. Nav2 patch
    nav2_out = patcher.patch_nav2_text(text, **patcher.nav2_kwargs(pinfo))
    # 2. EKF patch
    ekf_out = patcher.patch_ekf_text(
        nav2_out,
        base_type=pinfo["base"],
        frequency=pinfo["ekf_frequency"],
        fuse_vy=pinfo["fuse_vy"],
        fuse_imu_yaw=pinfo["fuse_imu_yaw"],
    )
    # 3. SLAM patch
    slam_out = patcher.patch_slam_text(
        ekf_out,
        resolution=pinfo["slam_resolution"],
        max_laser_range=pinfo["slam_max_range"],
    )

    with open(get_active_params_path(), "w") as f:
        f.write(slam_out)

    return {
        "status": "applied",
        "preset": preset_name,
        "label": pinfo["label"],
        "base": pinfo["base"],
    }


@app.get("/api/nav2_config")
def api_get_nav2_config(distro: str = "jazzy"):
    params = load_params()
    kine = params.get("kinematics", {})
    base = kine.get("base_type", "2wd")
    with open(get_active_params_path(), "r") as f:
        full_text = f.read()
    return {
        "distro": distro,
        "base": base,
        "config": yaml.dump(params.get("nav2", {}), sort_keys=False),
        "full_yaml": full_text,
        "path": os.path.relpath(get_active_params_path(), REPO_ROOT),
        "exists": True,
        "supported_distros": ["jazzy", "lyrical", "rolling"],
        "supported_bases": ["2wd", "4wd", "mecanum"],
        "depth_pointcloud_active": patcher.costmap_depth_active(full_text),
    }


@app.post("/api/nav2_config")
async def api_save_nav2_config(request: Request):
    data = await json_body(request)
    cfg_text = data.get("config", "")
    if not cfg_text.strip():
        raise HTTPException(status_code=400, detail="Empty configuration")
    nav2_data = yaml.safe_load(cfg_text)
    params = load_params()
    params["nav2"] = nav2_data
    save_params(params)
    return {"status": "ok", "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/nav2_config/patch")
async def api_nav2_config_patch(request: Request):
    data = await json_body(request)
    base = data.get("base", "2wd")
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_nav2_text(cur_text, **patcher.nav2_kwargs(dict(data, base_type=base)))
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "patched", "base": base, "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/nav2_config/costmap_sources")
async def api_nav2_costmap_sources(request: Request):
    data = await json_body(request)
    depth_enabled = bool(data.get("depth_enabled"))
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_costmap_sources(cur_text, depth_enabled)
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "ok", "depth_pointcloud_active": depth_enabled}


@app.post("/api/nav2_config/reset")
def api_nav2_config_reset():
    # Restore defaults from standard_diff preset
    pinfo = patcher.PRESETS["standard_diff"]
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_nav2_text(cur_text, **patcher.nav2_kwargs(pinfo))
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "reset"}


@app.get("/api/ekf_config")
def api_get_ekf_config(base: str = "2wd"):
    params = load_params()
    return {
        "base": base,
        "config": yaml.dump(params.get("ekf", {}), sort_keys=False),
        "path": os.path.relpath(get_active_params_path(), REPO_ROOT),
        "exists": True,
        "supported_bases": ["2wd", "4wd", "mecanum"],
    }


@app.post("/api/ekf_config")
async def api_save_ekf_config(request: Request):
    data = await json_body(request)
    cfg_text = data.get("config", "")
    if not cfg_text.strip():
        raise HTTPException(status_code=400, detail="Empty configuration")
    ekf_data = yaml.safe_load(cfg_text)
    params = load_params()
    params["ekf"] = ekf_data
    save_params(params)
    return {"status": "ok", "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/ekf_config/patch")
async def api_ekf_config_patch(request: Request):
    data = await json_body(request)
    base = data.get("base", "2wd")
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_ekf_text(cur_text, **patcher.ekf_kwargs(data, base=base))
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "patched", "base": base, "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/ekf_config/reset")
def api_ekf_config_reset():
    pinfo = patcher.PRESETS["standard_diff"]
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_ekf_text(cur_text, **patcher.ekf_kwargs(pinfo, base="2wd"))
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "reset"}


@app.get("/api/slam_config")
def api_get_slam_config():
    params = load_params()
    return {
        "config": yaml.dump(params.get("slam", {}), sort_keys=False),
        "path": os.path.relpath(get_active_params_path(), REPO_ROOT),
        "exists": True,
    }


@app.post("/api/slam_config")
async def api_save_slam_config(request: Request):
    data = await json_body(request)
    cfg_text = data.get("config", "")
    if not cfg_text.strip():
        raise HTTPException(status_code=400, detail="Empty configuration")
    slam_data = yaml.safe_load(cfg_text)
    params = load_params()
    params["slam"] = slam_data
    save_params(params)
    return {"status": "ok", "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/slam_config/patch")
async def api_slam_config_patch(request: Request):
    data = await json_body(request)
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_slam_text(
        cur_text,
        resolution=data.get("resolution"),
        max_laser_range=data.get("max_laser_range"),
        minimum_travel_distance=data.get("minimum_travel_distance"),
        minimum_travel_heading=data.get("minimum_travel_heading"),
    )
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "patched", "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/slam_config/reset")
def api_slam_config_reset():
    pinfo = patcher.PRESETS["standard_diff"]
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_slam_text(cur_text, resolution=pinfo["slam_resolution"], max_laser_range=pinfo["slam_max_range"])
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "reset"}


# ==============================================================================
# AI Assistant & Custom Robot Builder APIs
# ==============================================================================
@app.post("/api/ai/tune")
async def api_ai_tune(request: Request):
    data = await json_body(request)
    prompt = data.get("prompt", "")
    base = data.get("base", "2wd")
    distro = data.get("distro", "jazzy")
    return analyze_robotics_ai(prompt, base=base, distro=distro)


@app.post("/api/ai/apply")
async def api_ai_apply(request: Request):
    data = await json_body(request)
    distro = data.get("distro", "jazzy")
    nav2_p = data.get("nav2_patch") or {}
    ekf_p = data.get("ekf_patch") or {}
    slam_p = data.get("slam_patch") or {}
    resolved_base = data.get("base") or nav2_p.get("base") or ekf_p.get("base") or "2wd"

    with open(get_active_params_path(), "r") as f:
        text = f.read()

    if nav2_p:
        text = patcher.patch_nav2_text(text, **patcher.nav2_kwargs(dict(nav2_p, base_type=resolved_base)))
    if ekf_p:
        text = patcher.patch_ekf_text(text, **patcher.ekf_kwargs(ekf_p, base=resolved_base))
    if slam_p:
        text = patcher.patch_slam_text(
            text,
            resolution=slam_p.get("resolution"),
            max_laser_range=slam_p.get("max_laser_range"),
            minimum_travel_distance=slam_p.get("minimum_travel_distance"),
            minimum_travel_heading=slam_p.get("minimum_travel_heading"),
        )

    with open(get_active_params_path(), "w") as f:
        f.write(text)

    return {
        "status": "ai_applied",
        "distro": distro,
        "nav2_updated": bool(nav2_p),
        "ekf_updated": bool(ekf_p),
        "slam_updated": bool(slam_p),
    }


@app.post("/api/ai/robot_builder")
async def api_ai_robot_builder(request: Request):
    data = await json_body(request)
    description = data.get("description", "")
    return generate_custom_robot_specs(description)


@app.post("/api/ai/deploy_robot")
async def api_ai_deploy_robot(request: Request):
    data = await json_body(request)
    specs = data.get("specs") or {}
    design = specs.get("design") or {}
    tuning = specs.get("tuning") or {}
    base = design.get("base_type", "2wd")

    params = load_params()
    # The design speaks the config's own vocabulary (system_utils
    # generate_custom_robot_specs): the keys the firmware and the URDF read.
    # This used to write `track_width` and `wheelbase`, which nothing read.
    kine = params.setdefault("kinematics", {})
    kine["base_type"] = base
    for key in ("wheel_diameter", "lr_wheels_distance", "fr_wheels_distance"):
        if design.get(key) is not None:
            kine[key] = design[key]
    # A new chassis gets a body sized to it; the user's explicit geometry, if
    # any, is stale for the new kinematics, so it is derived afresh.
    params.pop("geometry", None)
    params["geometry"] = gen_robot_description.effective_geometry(params)

    save_params(params)
    res = regenerate_firmware_headers()
    regenerate_robot_description(params)

    return {
        "status": "deployed",
        "base": base,
        "message": f"Successfully configured and deployed custom {base.upper()} robot!",
        "generator_stdout": res.stdout,
    }


# ==============================================================================
# Sensors & Workspace Diagnostics APIs
# ==============================================================================
@app.get("/api/sensors")
def api_sensors():
    return sensor_registry()


@app.get("/api/sensors/driver_status")
def api_sensor_driver_status(sensor: str = "", ws: str = ""):
    return get_sensor_driver_status(sensor, ws=ws)


@app.get("/api/sensor_install_cmd")
@app.post("/api/sensor_install_cmd")
def api_sensor_install_cmd(sensor: str = "", distro: str = "jazzy", ws: str = ""):
    # A plain apt install -- no ROS sourcing or cd needed, so the handle is the
    # command as-is. The browser runs the handle instead of re-composing it.
    result = build_sensor_install_cmd(sensor, distro=distro, ws=ws)
    if result.get("command"):
        result["handle"] = actions.prepare(result["command"])
    return result


@app.get("/api/package/check")
def api_package_check(pkg: str = "", distro: str = "jazzy", ws: str = ""):
    return get_package_install_info(pkg, distro=distro, ws=ws)


@app.get("/api/workspace/build_cmd")
def api_workspace_build_cmd(ws: str = REPO_ROOT, distro: str = "jazzy"):
    # colcon needs a sourced ROS, so the handle carries the sourcing the browser
    # used to prepend -- the page runs the handle verbatim, adding nothing.
    base_cmd = build_base_install_cmd(ws, distro=distro)
    full = f"{ros_setup_shell(distro)}; {base_cmd}"
    return {"command": base_cmd, "handle": actions.prepare(full), "workspace": ws}


@app.get("/api/ros2/install_cmd")
def api_ros2_install_cmd(distro: str = "jazzy"):
    ros2_cmd = build_ros2_install_cmd(distro)
    return {
        "distro": distro,
        "installed": True,
        "command": ros2_cmd,
        "handle": actions.prepare(ros2_cmd),
    }


@app.get("/api/nav2/stack")
def api_nav2_stack(distro: str = "jazzy", ws: Optional[str] = None):
    return nav2_stack_status(distro=distro, ws=ws)


@app.post("/api/stream_ticket")
def api_stream_ticket():
    """Mint a one-shot, one-minute ticket for a server-sent-event stream.

    This is a POST, so the middleware requires the real token (in the header) to
    reach it; it hands back a nonce the page puts in the stream URL's ?ticket=.
    The token itself then never appears in a URL. Auth-off installs need no
    ticket, but one is returned anyway so the same frontend path works.
    """
    return {"status": "ok", "ticket": access.mint_stream_ticket()}


@app.get("/api/list_dir")
def api_list_dir(path: str = "", only: str = "any", exts: str = ""):
    target = path or REPO_ROOT
    if not access.path_allowed(target, CONFIG_DIR, REPO_ROOT):
        raise HTTPException(status_code=403, detail=f"Not a browsable location: {target}")
    return list_dir(target, only=only, exts=exts)


# ==============================================================================
# Parameter Export & Merge (yaml_merge.py)
# ==============================================================================
@app.post("/api/params/export")
async def api_params_export(request: Request):
    data = await json_body(request)
    dest = text_field(data, "dest_dir")
    if not dest:
        raise HTTPException(status_code=400, detail="dest_dir required")
    if not access.path_allowed(dest, CONFIG_DIR, REPO_ROOT, for_write=True):
        raise HTTPException(status_code=403, detail=f"Not an export location: {dest}")
    os.makedirs(dest, exist_ok=True)
    out_file = os.path.join(dest, f"{ACTIVE_ROBOT_NAME}_config_export.yaml")
    shutil.copyfile(get_active_params_path(), out_file)
    return {"status": "exported", "path": out_file}


@app.post("/api/params/merge")
async def api_params_merge(request: Request):
    data = await json_body(request)
    source_text = data.get("source_text", "")
    with open(get_active_params_path(), "r") as f:
        target_text = f.read()

    merged, report = yaml_merge.merge_yaml(target_text, source_text)
    if not data.get("dry_run"):
        with open(get_active_params_path(), "w") as f:
            f.write(merged)
    return {
        "status": "dry-run" if data.get("dry_run") else "merged",
        "report": report,
        "config": merged,
    }


@app.post("/api/params/promote")
async def api_params_promote(request: Request):
    return {"status": "promoted", "message": f"Single source of truth ({os.path.relpath(get_active_params_path(), REPO_ROOT)}) is active"}


# ==============================================================================
# Boards & Config Engine Diagnostics APIs
# ==============================================================================
@app.get("/api/boards")
def api_boards():
    ini_path = os.path.join(REPO_ROOT, "firmware", "platformio.ini")
    base_ini = os.path.join(REPO_ROOT, "firmware", "common", "platformio_base.ini")
    boards = []
    seen = set()

    # This robot's controller first, then every other robot's, so the selector
    # still offers the full set of known controllers to switch to.
    for cfg_path in [get_active_params_path()] + list_robot_config_files():
        try:
            with open(cfg_path, "r") as f:
                yd = yaml.safe_load(f) or {}
        except Exception:
            continue
        c_cfg = yd.get("base_controller") or {}
        c_name = c_cfg.get("name")
        if not c_name or c_name in seen:
            continue
        seen.add(c_name)
        boards.append({
            "env": c_name,
            "board": c_cfg.get("board", c_name),
            "platform": c_cfg.get("platform", ""),
            "mcu": c_cfg.get("mcu", ""),
            "transport": c_cfg.get("transport", "serial"),
            "baudrate": c_cfg.get("baudrate", 921600),
            "description": c_cfg.get("description", c_name),
            "robot": yd.get("robot", {}).get("name", ""),
        })
    return {"status": "ok", "boards": boards}


@app.get("/api/wiring_table")
def api_wiring_table():
    """The wiring chart for the active robot, as Markdown -- the sheet you want
    at the bench, generated from the same pins the Pin Matrix holds."""
    try:
        import gen_wiring_table
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"gen_wiring_table: {exc}")
    params = load_params()
    return {"status": "ok",
            "robot": (params.get("robot") or {}).get("name", ""),
            "markdown": gen_wiring_table.render(params)}


@app.get("/api/configs")
def api_configs():
    custom_dir = os.path.join(REPO_ROOT, "firmware", "include", "custom")
    configs = []
    if os.path.isdir(custom_dir):
        for f in sorted(os.listdir(custom_dir)):
            if f.endswith(".h"):
                configs.append({
                    "filename": f,
                    "path": os.path.relpath(os.path.join(custom_dir, f), REPO_ROOT),
                })
    return {"status": "ok", "configs": configs}


def get_ros_exec_cmd(cmd_str: str, distro: str = "jazzy") -> List[str]:
    """A ROS 2 command on this machine, with the installed ROS sourced.

    The PlatformIO venv dirs go last on PATH on purpose: a venv bin dir also
    owns `python3`, and reached through it Python loses the system
    dist-packages (numpy, and with it rclpy.node).
    """
    path_export = ("export PATH=$HOME/.local/bin:/usr/local/bin:$PATH:"
                   "$HOME/.pioenv/bin:$HOME/.platformio/penv/bin")
    return ["bash", "-lc", f"{path_export}; {ros_setup_shell(distro)} && {cmd_str}"]


# ==============================================================================
# Topic Monitor & LiDAR SSE APIs
# ==============================================================================
@app.get("/api/ros2/topics")
def api_ros2_topics(distro: str = "jazzy"):
    cmd = get_ros_exec_cmd("ros2 topic list -t", distro=distro)
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        lines = res.stdout.strip().splitlines()
        topics = []
        for line in lines:
            parts = line.split()
            if len(parts) >= 2:
                topics.append({"topic": parts[0], "type": parts[1].strip("[]")})
        return {"success": True, "topics": topics}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/ros2/hz_single")
def api_ros2_hz_single(topic: str, distro: str = "jazzy"):
    cmd = get_ros_exec_cmd(f"timeout 3 ros2 topic hz {shlex.quote(topic)}", distro=distro)
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
        rate = 0.0
        for line in res.stdout.splitlines():
            if "average rate:" in line:
                try:
                    rate = float(line.split("average rate:")[1].strip().split()[0])
                except Exception:
                    pass
        return {"success": True, "topic": topic, "hz": rate}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/lidar_stream")
def api_lidar_stream(distro: str = "jazzy", topic: str = "/scan", max_points: int = 360):
    """
    SSE feed of LaserScan messages for the canvas viewer.

    scripts/scan_stream.py emits one JSON object per line; each is relayed as a
    named `scan` event (plus `status` events for "no publisher yet"), which is
    what the browser listens for. Echoing `--csv` here produced unnamed events
    carrying field-less CSV, so the viewer never drew anything.
    """
    script = os.path.join(REPO_ROOT, "scripts", "scan_stream.py")
    cmd = get_ros_exec_cmd(
        f"python3 -u {shlex.quote(script)} --topic {shlex.quote(topic)} --max-points {int(max_points)}",
        distro=distro,
    )

    def event_generator():
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            for line in iter(proc.stdout.readline, ""):
                line = line.strip()
                if not line:
                    continue
                try:
                    payload = json.loads(line)
                    evt = payload.get("event", "scan")
                except (ValueError, AttributeError):
                    # Anything the ROS setup shell prints on the way up is not
                    # JSON; surface it as status rather than dropping it.
                    payload = {"event": "status", "level": "info", "message": line}
                    evt = "status"
                yield f"event: {evt}\ndata: {json.dumps(payload)}\n\n"
            rc = proc.wait()
            yield f"event: status\ndata: {json.dumps({'level': 'error' if rc else 'info', 'message': f'scan stream ended (exit {rc})'})}\n\n"
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()

    return StreamingResponse(event_generator(), media_type="text/event-stream")


# ------------------------------------------------------------------------------
# Firmware Detection & Native Flashing Engine (robot-computer side)
#
# The host workstation compiles the binary (pio run) and never opens /dev; this
# machine owns the serial bus and writes it with esptool / picotool. Build and
# flash are always separate steps -- no PlatformIO uploader is ever invoked.
# Flashing is never routed through the browser: WebSerial is the debug monitor only.
# ------------------------------------------------------------------------------
@app.get("/api/firmware/tools")
def api_firmware_tools():
    """Which flashing tools this machine can actually run, and what is plugged in."""
    return {"status": "ok", "tools": hw_tool_status(), "ports": hw_list_serial_ports()}


@app.get("/api/firmware/detect")
@app.post("/api/firmware/detect")
async def api_firmware_detect(request: Request):
    """
    Identify the attached MCU via `esptool chip_id` / `picotool info`.

    micro_ros_agent is stopped for the probe and resumed afterwards, because it
    holds the tty open for as long as it runs.
    """
    if request.method == "POST":
        try:
            data = await json_body(request)
        except Exception:
            data = {}
    else:
        data = dict(request.query_params)

    params = load_params()
    ctrl = get_controller(params)
    port = text_field(data, "port") or None
    family = data.get("family", "auto")
    baud = int(data.get("baud") or 115200)
    force = str(data.get("force", "")).lower() in ("1", "true", "yes")
    mode = data.get("mode") or ctrl.get("transport", "serial")
    udp_port = int(data.get("udp_port") or ctrl.get("udp_port", 8888))

    return hw_detect(
        port=port, family=family, baud=baud, force=force,
        agent_runner=agent_runner, mode=mode, udp_port=udp_port,
    )


@app.get("/api/firmware/artifact")
def api_firmware_artifact(firmware_dir: str = "firmware", env: Optional[str] = None):
    """Locate the binary for this environment: a local build, else the prebuilt release image."""
    params = load_params()
    return hw_resolve_artifact(firmware_dir, env or get_controller_name(params, "pico2"))


@app.post("/api/firmware/flash")
async def api_firmware_flash(request: Request):
    """
    Flash the firmware natively, streaming tool output as SSE.

    Stops micro_ros_agent before touching the bus and restarts it afterwards in
    the same execution mode it was running in.
    """
    try:
        data = await json_body(request)
    except Exception:
        data = {}

    params = load_params()
    ctrl = get_controller(params)
    firmware_dir = data.get("firmware_dir") or "firmware"
    env = mcu_identity.pio_env_for(text_field(data, "env") or get_controller_name(params, "pico2"), "pico2")
    port = text_field(data, "port") or ctrl.get("serial_port", "/dev/ttyUSB0")
    baud = int(data.get("baud") or ctrl.get("upload_baudrate") or 921600)
    chip = data.get("chip") or "auto"
    erase = bool(data.get("erase"))
    resume_agent = data.get("resume_agent", True)
    mode = data.get("mode") or ctrl.get("transport", "serial")
    udp_port = int(data.get("udp_port") or ctrl.get("udp_port", 8888))
    timeout = int(data.get("timeout") or 300)

    def event_generator():
        q = queue.Queue(maxsize=2000)
        done = {}

        def emit(line):
            try:
                q.put_nowait(line)
            except queue.Full:
                pass

        def worker():
            try:
                done["result"] = hw_flash(
                    firmware_dir=firmware_dir, env=env, port=port, baud=baud,
                    chip=chip, erase=erase, agent_runner=agent_runner, mode=mode,
                    udp_port=udp_port, resume_agent=resume_agent, emit=emit,
                    timeout=timeout,
                )
            except Exception as exc:
                done["result"] = {"status": "error", "ok": False,
                                  "error": f"{type(exc).__name__}: {exc}"}
                emit(f"[error] {type(exc).__name__}: {exc}")
            finally:
                done["finished"] = True

        threading.Thread(target=worker, daemon=True).start()

        while not done.get("finished") or not q.empty():
            try:
                line = q.get(timeout=1.0)
                yield f"event: output\ndata: {json.dumps({'line': line})}\n\n"
            except queue.Empty:
                yield ": ping\n\n"

        result = done.get("result") or {"status": "error", "ok": False,
                                        "error": "flash worker produced no result"}
        # The log was already streamed line by line; resending it would double it.
        summary = {k: v for k, v in result.items() if k != "log"}
        yield f"event: done\ndata: {json.dumps(summary)}\n\n"

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/api/firmware/generate")
def trigger_firmware_generation(controller: Optional[str] = None):
    gen_script = os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py")
    cmd = [sys.executable, gen_script]
    if controller:
        cmd.extend(["--controller", controller])
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise HTTPException(status_code=500, detail=res.stderr)
    return {"success": True, "output": res.stdout}


@app.post("/api/workflow/one-click")
def trigger_one_click_workflow(controller: Optional[str] = None, explore_sec: int = 15, no_nav2: bool = False, distro: Optional[str] = None, mode: Optional[str] = None, flash_firmware: bool = False, auto_update: bool = True, firmware: str = "auto"):
    params = load_params()
    selected_controller = controller or get_controller_name(params, "gendrv")
    pipeline_script = os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")

    d_str = distro
    if not d_str:
        if os.path.exists("/opt/ros/lyrical"):
            d_str = "lyrical"
        elif os.path.exists("/opt/ros/jazzy"):
            d_str = "jazzy"
        else:
            d_str = os.environ.get("ROS_DISTRO", "jazzy")

    cmd = [
        sys.executable, pipeline_script,
        "--controller", selected_controller,
        "--explore-sec", str(explore_sec),
        "--distro", d_str,
    ]
    if no_nav2:
        cmd.append("--no-nav2")
    if firmware in ("build", "prebuilt"):
        cmd.extend(["--firmware", firmware])
    # Two different questions, and the UI asks both.
    #   flash_firmware  force a write even over an up_to_date board.
    #   auto_update     (default on) write when the probe says the board is stale.
    # Only the negative is passed: the pipeline's own default is on, so a caller
    # that predates this parameter keeps the pipeline default rather than
    # silently pinning the old protective behaviour.
    if flash_firmware:
        cmd.append("--flash")
    if not auto_update:
        cmd.append("--no-auto-update")
    if mode:
        cmd.extend(["--mode", mode])
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise HTTPException(status_code=500, detail=res.stderr or res.stdout)
    return {
        "success": True,
        "controller": selected_controller,
        "output": res.stdout,
    }


@app.get("/api/workflow/one-click/stream")
def stream_one_click_workflow(controller: Optional[str] = None, explore_sec: int = 15, no_nav2: bool = False, distro: Optional[str] = None, mode: Optional[str] = "fake", flash_firmware: bool = False, auto_update: bool = True, firmware: str = "auto", robot: Optional[str] = None):
    # `robot` is not optional decoration: without it the pipeline resolves the
    # config from --controller alone, and where two robots declare the same
    # base_controller it takes the alphabetically first one. Measured with
    # rover_pico2 selected in the UI, on a box whose config dir also holds
    # linorobot2_config.yaml (also `pico2`):
    #
    #   Requested: robot config 'linorobot2_config.yaml' -> base_controller 'pico2'
    #
    # So the run used another robot's kinematics, pins, EKF, SLAM and Nav2
    # parameters while the header displayed "Robot: rover_pico2" throughout.
    # The UI knows the robot; it simply was not sending it.
    params = load_params()
    selected_controller = controller or get_controller_name(params, "pico2")
    pipeline_script = os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")
    m_str = mode or "fake"

    d_str = distro
    if not d_str:
        if os.path.exists("/opt/ros/lyrical"):
            d_str = "lyrical"
        elif os.path.exists("/opt/ros/jazzy"):
            d_str = "jazzy"
        else:
            d_str = os.environ.get("ROS_DISTRO", "jazzy")

    cmd = [
        sys.executable, "-u", pipeline_script,
        "--controller", selected_controller,
        "--explore-sec", str(explore_sec),
        "--mode", m_str,
        "--distro", d_str,
    ]
    if robot:
        cmd.extend(["--robot", robot])
    if no_nav2:
        cmd.append("--no-nav2")
    if firmware in ("build", "prebuilt"):
        cmd.extend(["--firmware", firmware])
    if flash_firmware:
        cmd.append("--flash")
    if not auto_update:
        cmd.append("--no-auto-update")

    def event_generator():
        start_line = (f">>> Starting One-Click Pipeline on robot: {robot or '<resolved from controller>'}, "
                      f"controller: {selected_controller} "
                      f"(distro: {d_str}, mode: {m_str}, "
                      f"auto-update: {'on' if auto_update else 'off'})")
        yield f"event: output\ndata: {json.dumps({'line': start_line})}\n\n"
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in iter(proc.stdout.readline, ""):
            if not line:
                break
            yield f"event: output\ndata: {json.dumps({'line': line.rstrip()})}\n\n"
        proc.wait()
        yield f"event: done\ndata: {json.dumps({'exit_code': proc.returncode})}\n\n"

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.get("/api/topics/verify")
def api_verify_topics(no_scan: bool = False, distro: str = "jazzy"):
    verify_script = os.path.join(REPO_ROOT, "scripts", "verify_topics.py")
    flag = " --no-scan" if no_scan else ""
    cmd = get_ros_exec_cmd(f"python3 {verify_script} --json{flag}", distro=distro)
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
        parsed = None
        for line in reversed(res.stdout.splitlines()):
            line = line.strip()
            if line.startswith("{") and line.endswith("}"):
                try:
                    parsed = json.loads(line)
                    break
                except Exception:
                    pass
        return {
            "success": res.returncode == 0,
            "exit_code": res.returncode,
            "data": parsed,
            "raw": res.stdout or res.stderr,
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/maps")
def list_maps():
    maps_dir = os.path.join(REPO_ROOT, "maps")
    if not os.path.isdir(maps_dir):
        return {"maps": [], "maps_dir": maps_dir}
    files = os.listdir(maps_dir)
    map_list = []
    for f in files:
        if f.endswith(".yaml"):
            base = f[:-5]
            map_list.append({
                "name": base,
                "yaml": f,
                "image": f"{base}.pgm" if f"{base}.pgm" in files else None,
            })
    # maps_dir is part of the contract: Nav2's map_server takes a filesystem
    # path (map:=/abs/path.yaml), not a URL, so the caller cannot build the
    # argument from the file names alone.
    return {"maps": map_list, "maps_dir": maps_dir}


@app.get("/api/maps/{filename}")
def get_map_file(filename: str):
    maps_dir = os.path.join(REPO_ROOT, "maps")
    file_path = os.path.join(maps_dir, filename)
    if not os.path.isfile(file_path):
        raise HTTPException(status_code=404, detail="Map file not found")
    media_type = "application/x-yaml" if filename.endswith(".yaml") else "application/octet-stream"
    return FileResponse(file_path, media_type=media_type)


# Mount static frontend.
#
# No-store on the app's own files. StaticFiles serves them with an ETag and a
# Last-Modified, which lets a browser keep a stale app.js across a cockpit
# update: the backend restarts with new behaviour, the page keeps the old
# script, and the two disagree in ways that look like the feature never
# landed. Measured here on 2026-09-20 -- a fixed Monitor worked through curl
# and did nothing in the browser until a hard reload. These files are a few
# hundred KB served over a LAN; re-fetching them costs nothing next to that.
class _NoStoreStatic(StaticFiles):
    async def get_response(self, path, scope):
        response = await super().get_response(path, scope)
        response.headers["Cache-Control"] = "no-store, must-revalidate"
        return response


if os.path.isdir(FRONTEND_DIR):
    app.mount("/", _NoStoreStatic(directory=FRONTEND_DIR, html=True), name="frontend")


if __name__ == "__main__":
    import argparse
    import uvicorn

    ap = argparse.ArgumentParser(description="Linorobot2 Cockpit supervisor")
    ap.add_argument("--host", default="0.0.0.0")
    # 8000 is the cockpit's own port; a spare one keeps a test instance clear of it.
    ap.add_argument("--port", type=int, default=int(os.environ.get("COCKPIT_PORT", 8000)))
    cli_args = ap.parse_args()
    uvicorn.run(app, host=cli_args.host, port=cli_args.port)
