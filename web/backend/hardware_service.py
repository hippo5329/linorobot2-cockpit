#!/usr/bin/env python3
# ==============================================================================
# hardware_service.py — Native MCU Detection & Flashing Engine
#
# Runs on the robot computer, which owns the serial bus. Detection
# (`esptool chip_id` / `picotool info`) and flashing (`esptool write_flash` /
# `picotool load -f`) are native; the binary comes from a local `pio run` or
# from a prebuilt release image in firmware/prebuilt/<profile>/.
#
# Browsers never flash. WebSerial in the frontend is reserved for a raw serial
# debug monitor only.
#
# Serial safety (AGENTS.md §6, inviolable):
#   * micro_ros_agent is stopped before any bus operation and resumed afterwards.
#   * Termination is by inspected PID only. NEVER pkill / killall.
#   * Ports 8000 (FastAPI), 5173 (Vite) and 9090 (rosbridge) are immune.
# ==============================================================================

import contextlib
import glob
import os
import re
import shlex
import shutil
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Dict, Iterator, List, Optional, Tuple

from runners import IMMUNE_PORTS, check_agent_port_status, release_agent_port


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# One table for what a board IS, shared with the pre-flash guard and the UI
# (scripts/mcu_identity.py). The path is set up here rather than relying on
# main.py having done it, so a test or a tool can import this module alone.
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import mcu_identity  # noqa: E402
PIO_PACKAGES = os.path.expanduser("~/.platformio/packages")

# Ready-to-flash release images, fetched by scripts/fetch_prebuilt.py or built
# by scripts/build_prebuilt.py. Kept out of git (see .gitignore).
PREBUILT_ROOT = os.path.join(REPO_ROOT, "firmware", "prebuilt")

# Only one caller may hold the serial bus at a time. Detection and flashing both
# stop micro_ros_agent, so letting two of them interleave would have one resume
# the agent while the other is mid-write.
_BUS_LOCK = threading.Lock()

# USB vendor IDs that identify the MCU family before any tool is run, so a Pico is
# never probed with esptool (which would hang on it for the full timeout).
RP_VENDOR_IDS = {"2e8a"}                      # Raspberry Pi (RP2040 / RP2350)
ESP_VENDOR_IDS = {
    "303a",   # Espressif native USB-JTAG/serial (ESP32-S3, C3)
    "10c4",   # Silicon Labs CP210x
    "1a86",   # QinHeng CH340 / CH9102
    "0403",   # FTDI
}

# PlatformIO env name -> MCU family. Mirrors flash_mcu.py so the two agree.
ESP_ENV_RE = re.compile(r"esp32", re.I)
RP_ENV_RE = re.compile(r"pico", re.I)

# Standard ESP-IDF flash layout used by the Arduino/PlatformIO builds in firmware/.
# ESP32-S3 / C3 place the bootloader at 0x0; the classic ESP32 at 0x1000.
ESP_BOOTLOADER_OFFSETS = {
    "esp32": "0x1000",
    "esp32s2": "0x1000",
    "esp32s3": "0x0",
    "esp32c3": "0x0",
    "esp32c6": "0x0",
}


def family_for_env(env: str) -> str:
    """'esp' or 'rp' for a PlatformIO environment name."""
    if RP_ENV_RE.search(env or ""):
        return "rp"
    if ESP_ENV_RE.search(env or ""):
        return "esp"
    return "unknown"


# ------------------------------------------------------------------------------
# Tool discovery
# ------------------------------------------------------------------------------
def find_esptool() -> Optional[List[str]]:
    """
    argv prefix that invokes esptool, or None.

    Checked in order: a system `esptool` / `esptool.py`, the importable module,
    then the PlatformIO `tool-esptoolpy` package -- which is where it actually
    lives on a machine that has only ever built firmware through PlatformIO.
    """
    for name in ("esptool", "esptool.py"):
        found = shutil.which(name)
        if found:
            return [found]

    try:
        subprocess.run(
            [sys.executable, "-m", "esptool", "version"],
            capture_output=True, timeout=15, check=True,
        )
        return [sys.executable, "-m", "esptool"]
    except Exception:
        pass

    for candidate in sorted(glob.glob(os.path.join(PIO_PACKAGES, "tool-esptoolpy*", "esptool.py"))):
        if os.path.isfile(candidate):
            return [sys.executable, candidate]
    return None


def find_picotool() -> Optional[str]:
    """Path to a picotool binary, or None. Prefers the system copy."""
    found = shutil.which("picotool")
    if found:
        return found
    patterns = [
        os.path.join(PIO_PACKAGES, "tool-picotool*", "picotool"),
        os.path.join(PIO_PACKAGES, "tool-picotool*", "bin", "picotool"),
        "/usr/bin/picotool",
        "/usr/local/bin/picotool",
    ]
    for pattern in patterns:
        for candidate in sorted(glob.glob(pattern)):
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate
    return None


def tool_status() -> Dict[str, Any]:
    """What the flashing engine can do on this machine right now."""
    esptool_argv = find_esptool()
    picotool_path = find_picotool()
    return {
        "esptool": {
            "available": esptool_argv is not None,
            "command": " ".join(esptool_argv) if esptool_argv else None,
            "version": _tool_version(esptool_argv + ["version"]) if esptool_argv else None,
            "purpose": "ESP32 / ESP32-S3 chip_id and write_flash",
        },
        "picotool": {
            "available": picotool_path is not None,
            "command": picotool_path,
            "version": _tool_version([picotool_path, "version"]) if picotool_path else None,
            "purpose": "RP2040 / RP2350 info and load -f",
        },
        "hint": (
            "Install with: pip install esptool  /  apt-get install picotool. "
            "PlatformIO's bundled copies are used automatically when present."
        ),
    }


def _tool_version(cmd: List[str]) -> Optional[str]:
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
        out = (res.stdout or res.stderr or "").strip().splitlines()
        return out[-1].strip() if out else None
    except Exception:
        return None


# ------------------------------------------------------------------------------
# Serial port enumeration
# ------------------------------------------------------------------------------
def _usb_ids_for_port(port: str) -> Tuple[Optional[str], Optional[str], Optional[str]]:
    """(idVendor, idProduct, product-name) for a tty, read from sysfs.

    Indexed by the device NUMBER (/sys/dev/char/<maj>:<min>), because the name
    is not the identity: the Incus robot box is handed the bench's ttyACM1 as
    /dev/ttyACM0 (AGENTS.md §6), and /sys/class/tty/ttyACM0 inside it still
    describes the HOST's ttyACM0 — a different board.
    """
    device_link = None
    try:
        rdev = os.stat(port).st_rdev
        cand = f"/sys/dev/char/{os.major(rdev)}:{os.minor(rdev)}/device"
        if os.path.exists(cand):
            device_link = cand
    except OSError:
        pass
    if device_link is None:
        device_link = f"/sys/class/tty/{os.path.basename(port)}/device"
    if not os.path.exists(device_link):
        return None, None, None
    path = os.path.realpath(device_link)
    # Walk up until the USB device node carrying idVendor is reached.
    for _ in range(6):
        vendor_file = os.path.join(path, "idVendor")
        if os.path.isfile(vendor_file):
            def _read(fname):
                try:
                    with open(os.path.join(path, fname)) as f:
                        return f.read().strip()
                except Exception:
                    return None
            return _read("idVendor"), _read("idProduct"), _read("product")
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    return None, None, None


def list_serial_ports() -> List[Dict[str, Any]]:
    """Every USB serial device present, with the MCU family guessed from its USB IDs."""
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    out = []
    for port in ports:
        vid, pid, product = _usb_ids_for_port(port)
        if vid in RP_VENDOR_IDS:
            family = "rp"
        elif vid in ESP_VENDOR_IDS:
            family = "esp"
        else:
            family = "unknown"
        out.append({
            "port": port,
            "by_id": mcu_identity.by_id_for_port(port),
            "vid": vid,
            "pid": pid,
            "product": product,
            "family": family,
            "writable": os.access(port, os.W_OK),
        })
    return out


# ------------------------------------------------------------------------------
# micro_ros_agent concurrency guard
# ------------------------------------------------------------------------------
def _verify_port_released(port: str, timeout: float = 6.0) -> bool:
    """Poll lsof until nothing holds the tty. An empty lsof is the release signal."""
    if not port or not os.path.exists(port):
        return True
    if not shutil.which("lsof"):
        return True
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            res = subprocess.run(["lsof", port], capture_output=True, text=True, timeout=5)
            if res.returncode != 0 or not res.stdout.strip():
                return True
        except Exception:
            return True
        time.sleep(0.4)
    return False


@contextlib.contextmanager
def microros_agent_paused(
    port: str,
    agent_runner: Any = None,
    mode: str = "serial",
    udp_port: int = 8888,
    resume: bool = True,
    emit: Optional[Callable[[str], None]] = None,
) -> Iterator[Dict[str, Any]]:
    """
    Stop micro_ros_agent, hand the serial bus to the caller, then put the agent
    back exactly as it was.

    The agent holds /dev/ttyUSB*/ttyACM* open for as long as it runs, so esptool
    and picotool cannot open the port while it is up -- flashing under a live
    agent is the classic "Failed to connect / device busy" on this stack.

    Termination is targeted: the supervisor's own agent slot is killed by PID,
    and any other holder goes through release_agent_port(), which refuses to
    signal a PID bound to an immune port.
    """
    log = emit or (lambda _m: None)
    state = {
        "was_running": False,
        "resumed": False,
        "command": "",
        "released": False,
    }

    if agent_runner is not None and agent_runner.is_busy():
        state["was_running"] = True
        state["command"] = getattr(agent_runner, "command_str", "") or ""
        log(f"[bus] micro_ros_agent is running — stopping it to release {port}")
        agent_runner.kill()
        time.sleep(0.6)
    else:
        log("[bus] supervisor micro_ros_agent slot is idle")

    # Anything else still holding the tty (a stray agent, a serial monitor).
    # Always probed as a serial port, whatever `mode` the robot's runtime transport
    # is: flashing goes over USB even when the firmware talks micro-ROS over Wi-Fi,
    # so a udp4 agent's UDP port is not what has to be free here -- the tty is.
    status = check_agent_port_status(port, "serial", udp_port)
    if status.get("in_use"):
        log(f"[bus] {port} still held by PIDs {status.get('pids')} — releasing (targeted, never pkill)")
        result = release_agent_port(port, mode, udp_port)
        for action in result.get("actions", []):
            log(f"[bus] {action}")
        for err in result.get("errors", []):
            log(f"[bus] {err}")

    state["released"] = _verify_port_released(port)
    if state["released"]:
        log(f"[bus] {port} released")
    else:
        log(f"[bus] WARNING: {port} still appears busy — continuing, the tool will report the real error")

    try:
        yield state
    finally:
        if resume and state["was_running"] and state["command"] and agent_runner is not None:
            # Give the MCU a moment to enumerate again after a reset before the
            # agent reopens the port, otherwise it binds to a tty that is about
            # to disappear.
            time.sleep(2.0)
            log("[bus] resuming micro_ros_agent")
            try:
                threading.Thread(
                    target=agent_runner.start_streaming,
                    args=(state["command"], REPO_ROOT),
                    daemon=True,
                ).start()
                state["resumed"] = True
            except Exception as exc:
                log(f"[bus] failed to resume micro_ros_agent: {exc}")


# ------------------------------------------------------------------------------
# Detection
# ------------------------------------------------------------------------------
_CHIP_RE = re.compile(r"Chip is\s+(?P<chip>[^\r\n(]+?)\s*(?:\(revision\s*(?P<rev>[^)]*)\))?\s*$", re.M)
_MAC_RE = re.compile(r"^MAC:\s*(?P<mac>[0-9a-fA-F:]{17})", re.M)
_FEATURES_RE = re.compile(r"^Features:\s*(?P<features>.+)$", re.M)
_CRYSTAL_RE = re.compile(r"^Crystal is\s*(?P<crystal>.+)$", re.M)
_DETECTING_RE = re.compile(r"^Detecting chip type\.\.\.\s*(?P<chip>.+)$", re.M)


def detect_esp32(port: str, baud: int = 115200, timeout: int = 40) -> Dict[str, Any]:
    """`esptool chip_id` against a port. Assumes the bus is already released."""
    esptool_argv = find_esptool()
    if not esptool_argv:
        return {"ok": False, "family": "esp", "port": port,
                "error": "esptool not found. Install with: pip install esptool"}

    cmd = esptool_argv + ["--port", port, "--baud", str(baud), "chip_id"]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"ok": False, "family": "esp", "port": port, "command": " ".join(cmd),
                "error": f"esptool chip_id timed out after {timeout}s. "
                         "Hold BOOT and tap EN, or check that nothing else holds the port."}
    except Exception as exc:
        return {"ok": False, "family": "esp", "port": port, "command": " ".join(cmd), "error": str(exc)}

    output = (res.stdout or "") + (res.stderr or "")
    info: Dict[str, Any] = {
        "ok": res.returncode == 0,
        "family": "esp",
        "port": port,
        "tool": "esptool",
        "command": " ".join(cmd),
        "returncode": res.returncode,
        "raw": output.strip(),
    }

    chip_match = _CHIP_RE.search(output) or _DETECTING_RE.search(output)
    if chip_match:
        info["chip"] = chip_match.group("chip").strip()
        try:
            info["revision"] = (chip_match.group("rev") or "").strip() or None
        except IndexError:
            info["revision"] = None
    mac_match = _MAC_RE.search(output)
    if mac_match:
        # esptool >=4 answers chip_id with the MAC: the ESP32 has no separate chip ID.
        info["mac"] = mac_match.group("mac")
        info["chip_id"] = mac_match.group("mac").replace(":", "").lower()
    features = _FEATURES_RE.search(output)
    if features:
        info["features"] = [f.strip() for f in features.group("features").split(",")]
    crystal = _CRYSTAL_RE.search(output)
    if crystal:
        info["crystal"] = crystal.group("crystal").strip()

    if not info["ok"] and "error" not in info:
        tail = [ln for ln in output.strip().splitlines() if ln.strip()]
        info["error"] = tail[-1] if tail else f"esptool exited {res.returncode}"
    return info


_RP_TYPE_RE = re.compile(r"^\s*type:\s*(?P<type>RP\d+\w*)", re.M | re.I)
_RP_FLASH_RE = re.compile(r"^\s*flash size:\s*(?P<flash>.+)$", re.M | re.I)
_RP_NAME_RE = re.compile(r"^\s*name:\s*(?P<name>.+)$", re.M | re.I)
_RP_ID_RE = re.compile(r"^\s*(?:flash id|id):\s*(?P<id>\S+)", re.M | re.I)


def detect_rp(force: bool = False, timeout: int = 30) -> Dict[str, Any]:
    """
    `picotool info -a` over every accessible RP2040/RP2350.

    picotool addresses boards over USB, not through a tty, so no port is passed.
    A board running normal firmware is only reachable when it exposes the reset
    interface; `force=True` adds `-f`, which reboots it into BOOTSEL to look --
    intrusive, so it stays opt-in.
    """
    picotool = find_picotool()
    if not picotool:
        return {"ok": False, "family": "rp",
                "error": "picotool not found. Install with: apt-get install picotool"}

    cmd = [picotool, "info", "-a"] + (["-f"] if force else [])
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"ok": False, "family": "rp", "command": " ".join(cmd),
                "error": f"picotool info timed out after {timeout}s"}
    except Exception as exc:
        return {"ok": False, "family": "rp", "command": " ".join(cmd), "error": str(exc)}

    output = (res.stdout or "") + (res.stderr or "")
    info: Dict[str, Any] = {
        "ok": res.returncode == 0,
        "family": "rp",
        "tool": "picotool",
        "command": " ".join(cmd),
        "returncode": res.returncode,
        "raw": output.strip(),
    }
    type_match = _RP_TYPE_RE.search(output)
    if type_match:
        info["chip"] = type_match.group("type").upper()
    name_match = _RP_NAME_RE.search(output)
    if name_match:
        info["program"] = name_match.group("name").strip()
    flash_match = _RP_FLASH_RE.search(output)
    if flash_match:
        info["flash_size"] = flash_match.group("flash").strip()
    id_match = _RP_ID_RE.search(output)
    if id_match:
        info["chip_id"] = id_match.group("id").strip()

    if not info["ok"]:
        if "no accessible" in output.lower():
            info["error"] = (
                "No RP2040/RP2350 in BOOTSEL mode. Hold BOOTSEL while plugging the board in, "
                "or retry with force=true to reboot it into BOOTSEL."
            )
        else:
            tail = [ln for ln in output.strip().splitlines() if ln.strip()]
            info["error"] = tail[-1] if tail else f"picotool exited {res.returncode}"
    return info


def detect(
    port: Optional[str] = None,
    family: str = "auto",
    baud: int = 115200,
    force: bool = False,
    agent_runner: Any = None,
    mode: str = "serial",
    udp_port: int = 8888,
) -> Dict[str, Any]:
    """
    Identify the MCU on `port`, stopping micro_ros_agent for the probe.

    With no port, every USB serial device is enumerated and each is probed with
    the tool its USB vendor ID calls for.
    """
    with _BUS_LOCK:
        available = list_serial_ports()
        result: Dict[str, Any] = {
            "status": "ok",
            "ports": available,
            "tools": tool_status(),
            "detections": [],
            "log": [],
        }
        log = result["log"].append

        targets: List[Dict[str, Any]] = []
        if port:
            match = next((p for p in available if p["port"] == port), None)
            targets = [match or {"port": port, "family": "unknown"}]
        else:
            targets = available

        if not targets:
            result["status"] = "empty"
            result["message"] = "No /dev/ttyUSB* or /dev/ttyACM* devices present."
            # picotool still sees a board sitting in BOOTSEL, which exposes no tty.
            rp_info = detect_rp(force=False)
            if rp_info.get("ok"):
                result["detections"].append(rp_info)
                result["status"] = "ok"
                result["message"] = "No tty devices, but a board was found in BOOTSEL mode."
            return result

        for target in targets:
            target_port = target["port"]
            want = family if family != "auto" else target.get("family", "unknown")

            with microros_agent_paused(
                target_port, agent_runner=agent_runner, mode=mode,
                udp_port=udp_port, emit=log,
            ):
                if want == "rp":
                    log(f"[detect] {target_port}: RP2040/RP2350 vendor ID — running picotool info")
                    info = detect_rp(force=force)
                    info["port"] = target_port
                elif want == "esp":
                    log(f"[detect] {target_port}: ESP vendor ID — running esptool chip_id")
                    info = detect_esp32(target_port, baud=baud)
                else:
                    # Unknown USB IDs: esptool is the safe probe. It fails fast on a
                    # non-ESP board, whereas picotool would not see it at all.
                    log(f"[detect] {target_port}: unknown vendor ID — trying esptool chip_id")
                    info = detect_esp32(target_port, baud=baud)
                    if not info.get("ok"):
                        log(f"[detect] {target_port}: not an ESP — trying picotool info")
                        rp_info = detect_rp(force=force)
                        if rp_info.get("ok"):
                            rp_info["port"] = target_port
                            info = rp_info
                info["usb"] = {k: target.get(k) for k in ("vid", "pid", "product")}
                result["detections"].append(info)

        result["detected"] = any(d.get("ok") for d in result["detections"])
        return result


# ------------------------------------------------------------------------------
# Flash plans
# ------------------------------------------------------------------------------
def esp32_flash_plan(build_dir: str, chip: str = "esp32") -> List[Tuple[str, str]]:
    """
    [(offset, file)] for an ESP32 PlatformIO build.

    A PlatformIO Arduino build emits bootloader.bin, partitions.bin and
    firmware.bin separately; flashing firmware.bin alone at 0x0 produces a board
    that boots into a reset loop. When the build produced a single merged image,
    that one image goes to 0x0 instead.
    """
    merged = os.path.join(build_dir, "firmware-merged.bin")
    if os.path.isfile(merged):
        return [("0x0", merged)]

    chip_key = (chip or "esp32").lower().replace("-", "")
    boot_offset = ESP_BOOTLOADER_OFFSETS.get(chip_key, "0x1000")

    plan: List[Tuple[str, str]] = []
    bootloader = os.path.join(build_dir, "bootloader.bin")
    partitions = os.path.join(build_dir, "partitions.bin")
    firmware = os.path.join(build_dir, "firmware.bin")

    if os.path.isfile(bootloader):
        plan.append((boot_offset, bootloader))
    if os.path.isfile(partitions):
        plan.append(("0x8000", partitions))
    # boot_app0 lives in the framework package, not the build dir; PlatformIO
    # writes it at 0xe000 on dual-OTA layouts.
    for boot_app0 in sorted(glob.glob(os.path.join(
            PIO_PACKAGES, "framework-arduinoespressif32*", "tools", "partitions", "boot_app0.bin"))):
        plan.append(("0xe000", boot_app0))
        break
    if os.path.isfile(firmware):
        plan.append(("0x10000" if plan else "0x0", firmware))
    return plan


def prebuilt_dir_for_env(env: str) -> Optional[str]:
    """firmware/prebuilt/<profile> whose manifest names this PlatformIO env."""
    import json
    if not os.path.isdir(PREBUILT_ROOT):
        return None
    for name in sorted(os.listdir(PREBUILT_ROOT)):
        d = os.path.join(PREBUILT_ROOT, name)
        m = os.path.join(d, "manifest.json")
        if not os.path.isfile(m):
            continue
        try:
            with open(m) as fh:
                if json.load(fh).get("pio_env") == env:
                    return d
        except Exception:
            continue
    return None


def resolve_artifact(firmware_dir: str, env: str) -> Dict[str, Any]:
    """
    Locate the compiled binary for this env.

    A local .pio build wins (someone on this machine compiled for this env);
    otherwise the prebuilt profile whose manifest names this env is used. The
    profile directory is flat (firmware.uf2 / *.bin next to manifest.json).
    """
    if not os.path.isabs(firmware_dir):
        firmware_dir = os.path.join(REPO_ROOT, firmware_dir)
    local_build = os.path.join(firmware_dir, ".pio", "build", env)
    family = family_for_env(env)

    prebuilt = prebuilt_dir_for_env(env)
    if os.path.isdir(local_build) and os.listdir(local_build):
        build_dir, source = local_build, "local-build"
    elif prebuilt:
        build_dir, source = prebuilt, "prebuilt"
    else:
        build_dir, source = local_build, "local-build"

    info: Dict[str, Any] = {
        "firmware_dir": firmware_dir,
        "build_dir": build_dir,
        "prebuilt_dir": prebuilt,
        "local_build_dir": local_build,
        "source": source,
        "env": env,
        "family": family,
        "exists": os.path.isdir(build_dir) and bool(os.listdir(build_dir)),
    }
    if not info["exists"]:
        info["error"] = (
            f"No firmware for env '{env}'. Fetch the release image with "
            f"`python3 scripts/fetch_prebuilt.py {env}` or build it with "
            f"`pio run -d {os.path.relpath(firmware_dir, REPO_ROOT)} -e {env}`."
        )
        return info

    if family == "rp":
        uf2 = os.path.join(build_dir, "firmware.uf2")
        info["artifact"] = uf2 if os.path.isfile(uf2) else None
        if not info["artifact"]:
            info["error"] = f"firmware.uf2 not found in {build_dir}"
    elif family == "esp":
        info["plan"] = esp32_flash_plan(build_dir)
        info["artifact"] = info["plan"][-1][1] if info["plan"] else None
        if not info["plan"]:
            info["error"] = f"No .bin images found in {build_dir}"
    else:
        info["error"] = f"Cannot tell the MCU family from environment '{env}'"
    return info


# ------------------------------------------------------------------------------
# Flashing
# ------------------------------------------------------------------------------
def _stream_tool(cmd: List[str], emit: Callable[[str], None], timeout: int = 300) -> int:
    """Run a flashing tool, forwarding each line as it appears. Returns the exit code."""
    emit(f"$ {' '.join(shlex.quote(c) for c in cmd)}")
    try:
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, preexec_fn=os.setsid,
        )
    except FileNotFoundError:
        emit(f"[error] tool not found: {cmd[0]}")
        return 127
    except Exception as exc:
        emit(f"[error] failed to start {cmd[0]}: {exc}")
        return 1

    deadline = time.monotonic() + timeout
    try:
        for line in iter(proc.stdout.readline, ""):
            if not line:
                break
            emit(line.rstrip("\n"))
            if time.monotonic() > deadline:
                emit(f"[error] timed out after {timeout}s — terminating")
                # Kill the whole group: esptool spawns children that would
                # otherwise keep holding the port and the stdout pipe.
                try:
                    os.killpg(os.getpgid(proc.pid), 15)
                except Exception:
                    pass
                break
    finally:
        try:
            proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), 9)
            except Exception:
                pass
    return proc.returncode if proc.returncode is not None else 1


def flash(
    firmware_dir: str = "firmware",
    env: str = "esp32",
    port: str = "",
    baud: int = 921600,
    chip: str = "auto",
    erase: bool = False,
    agent_runner: Any = None,
    mode: str = "serial",
    udp_port: int = 8888,
    resume_agent: bool = True,
    emit: Optional[Callable[[str], None]] = None,
    timeout: int = 300,
) -> Dict[str, Any]:
    """
    Flash a binary natively, with micro_ros_agent paused around it.

    ESP32  -> esptool write_flash at the standard ESP-IDF offsets
    RP2040 -> picotool load -f (force into BOOTSEL), then reboot into the app
    """
    lines: List[str] = []

    def log(message: str):
        lines.append(message)
        if emit:
            emit(message)

    result: Dict[str, Any] = {"status": "error", "ok": False, "env": env, "port": port, "log": lines}

    artifact = resolve_artifact(firmware_dir, env)
    result["artifact"] = artifact
    if artifact.get("error"):
        log(f"[error] {artifact['error']}")
        result["error"] = artifact["error"]
        return result

    family = artifact["family"]
    log("=" * 68)
    log(f"Flashing {env} ({family.upper()}) from {artifact['build_dir']}")
    log(f"Port: {port or '(usb)'} | Baud: {baud}")
    log("=" * 68)

    with _BUS_LOCK:
        with microros_agent_paused(
            port, agent_runner=agent_runner, mode=mode, udp_port=udp_port,
            resume=resume_agent, emit=log,
        ) as bus:
            # Live reference, not a copy: microros_agent_paused fills in
            # "resumed" on exit, which is after every return below.
            result["agent"] = bus

            if family == "esp":
                esptool_argv = find_esptool()
                if not esptool_argv:
                    result["error"] = "esptool not found. Install with: pip install esptool"
                    log(f"[error] {result['error']}")
                    return result
                if not port:
                    result["error"] = "ESP32 flashing needs a serial port"
                    log(f"[error] {result['error']}")
                    return result

                base = esptool_argv + ["--port", port, "--baud", str(baud)]
                if chip and chip != "auto":
                    base = esptool_argv + ["--chip", chip, "--port", port, "--baud", str(baud)]

                if erase:
                    log("[flash] erasing flash first")
                    if _stream_tool(base + ["erase_flash"], log, timeout=timeout) != 0:
                        result["error"] = "erase_flash failed"
                        log(f"[error] {result['error']}")
                        return result

                write_cmd = base + ["write_flash", "-z"]
                for offset, path in artifact["plan"]:
                    write_cmd += [offset, path]
                    log(f"[flash] {offset} <- {os.path.basename(path)}")
                code = _stream_tool(write_cmd, log, timeout=timeout)

            elif family == "rp":
                picotool = find_picotool()
                if not picotool:
                    result["error"] = "picotool not found. Install with: apt-get install picotool"
                    log(f"[error] {result['error']}")
                    return result

                uf2 = artifact["artifact"]
                # Put the board in BOOTSEL before picotool is asked to find one.
                # picotool's own -f reboots a running board through the SDK reset
                # interface, which a board is only guaranteed to expose if its
                # current firmware compiled it in -- and the firmware on a board
                # about to be reflashed is, by definition, not something this can
                # assume. The 1200-baud CDC pulse is handled by the ROM-backed
                # USB stack instead, so it works regardless of what is running.
                usb_serial = _rp_enter_bootsel(port, picotool, log)
                # Every picotool invocation is pinned to this board. The bench
                # carries more than one RP-series device and picotool otherwise
                # picks whichever it finds first -- it already reported "RP2040
                # device at bus 3" while flashing the RP2350 on bus 1, and -f
                # would have rebooted that one into BOOTSEL and written to it.
                sel = ["--ser", usb_serial] if usb_serial else []

                log(f"[flash] loading {os.path.basename(uf2)}")
                # -x executes the image afterwards, so the board comes straight
                # back as a micro-ROS node rather than sitting in the bootloader.
                # Device selection goes AFTER the filename. picotool's synopsis is
                #   load [opts] <filename> [-t <type>] [-o <offset>] [device-selection]
                # and putting --ser before the file is rejected outright with
                # "unexpected option: --ser" once -f is also present.
                code = _stream_tool([picotool, "load", "-x", uf2] + sel, log, timeout=timeout)
                if code != 0:
                    # Still not in BOOTSEL: fall back to asking picotool to force
                    # the reset itself, which works when the firmware does expose
                    # the reset interface.
                    log("[flash] retrying with -f (asking picotool to force the reset)")
                    code = _stream_tool([picotool, "load", "-f", "-x", uf2] + sel,
                                        log, timeout=timeout)
            else:
                result["error"] = f"Unsupported MCU family for env '{env}'"
                log(f"[error] {result['error']}")
                return result

            result["returncode"] = code
            result["ok"] = code == 0
            result["status"] = "ok" if code == 0 else "error"
            if code == 0:
                log(f"[flash] OK — {env} flashed successfully")
            else:
                result["error"] = f"Flashing failed with exit code {code}"
                log(f"[error] {result['error']}")

    return result



def _usb_serial_for_tty(port: Optional[str]) -> Optional[str]:
    """The USB serial number of the board behind a tty, read from sysfs.

    This has to be captured BEFORE the board is rebooted, because in BOOTSEL it
    has no tty to ask. The number survives the reset -- the RP2 bootloader
    reports the same one the application does -- so it is the one handle that
    identifies a specific board across the reboot.

    Without it picotool operates on whichever RP-series device it happens to
    find first. On a bench with two of them that is a coin toss, and `-f` would
    reboot the wrong board into BOOTSEL and flash it.
    """
    if not port:
        return None
    name = os.path.basename(port)
    try:
        # /sys/class/tty/ttyACM0/device is the USB *interface*; its parent is the
        # device, which is what carries the serial.
        iface = os.path.realpath(f"/sys/class/tty/{name}/device")
        serial_path = os.path.join(os.path.dirname(iface), "serial")
        if os.path.isfile(serial_path):
            with open(serial_path) as fh:
                return fh.read().strip() or None
    except Exception:
        pass
    return None


def _rp_enter_bootsel(port: Optional[str], picotool: str, log) -> Optional[str]:
    """Drop an RP2040/RP2350 into BOOTSEL with the 1200-baud CDC pulse.

    Opening the CDC port at 1200 baud and closing it is the RP2 convention for
    "reboot into the bootloader". The board then drops its tty and re-enumerates
    as a raw USB bootloader device, which is what picotool talks to -- so the
    port disappearing is success, not a fault.

    Returns the board's USB serial when one is known, so the caller can point
    picotool at this board and no other.
    """
    usb_serial = _usb_serial_for_tty(port)
    if usb_serial:
        log(f"[flash] target board USB serial {usb_serial}")
    if _picotool_sees_bootsel(picotool, usb_serial):
        log("[flash] board is already in BOOTSEL")
        if not usb_serial:
            # Already in BOOTSEL means no tty, so there was nothing to read the
            # serial from. picotool will act on whichever RP-series device it
            # finds; that is correct only while this is the only one in BOOTSEL.
            log("[flash] note: board not identified by serial (no tty to read it from)")
        return usb_serial
    if not port or not os.path.exists(port):
        log("[flash] no serial port to pulse — hoping the board is already in BOOTSEL")
        return usb_serial

    log(f"[flash] 1200-baud pulse on {port} to request BOOTSEL")
    try:
        import serial
        # write_timeout as well as timeout: a board whose USB stack has stopped
        # servicing the host makes the OPEN itself block, and one wedged board
        # would otherwise stall the flash for the full tool timeout.
        handle = serial.Serial(port, baudrate=1200, timeout=1, write_timeout=1)
        # DTR must be dropped while the port is still OPEN. The core watches for
        # DTR going low at 1200 baud; simply closing the handle does not present
        # that transition in a form it acts on, and the board carries on running
        # as if nothing happened. This one line is the difference between the
        # pulse working and the flash failing with "Unable to locate reset
        # interface on the device".
        handle.dtr = False
        time.sleep(0.3)
        handle.close()
    except Exception as exc:
        log(f"[flash] pyserial could not pulse ({exc}); falling back to stty")
        try:
            subprocess.run(["stty", "-F", port, "1200", "-hupcl"], capture_output=True, timeout=3)
        except Exception as stty_exc:
            log(f"[flash] stty pulse failed too: {stty_exc}")
            return usb_serial

    # USB re-enumeration is not instant and its timing varies with the hub, so
    # poll for the bootloader rather than sleeping a guessed interval.
    for _ in range(20):
        time.sleep(0.5)
        if _picotool_sees_bootsel(picotool, usb_serial):
            log("[flash] board re-enumerated in BOOTSEL")
            return usb_serial
    log("[flash] no BOOTSEL device appeared within 10s after the pulse")
    return usb_serial


def _picotool_sees_bootsel(picotool: str, usb_serial: Optional[str] = None) -> bool:
    """True when picotool can open THIS board in BOOTSEL mode."""
    cmd = [picotool, "info"]
    if usb_serial:
        cmd += ["--ser", usb_serial]
    try:
        return subprocess.run(cmd, capture_output=True, timeout=10).returncode == 0
    except Exception:
        return False
