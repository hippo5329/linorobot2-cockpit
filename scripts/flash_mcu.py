#!/usr/bin/env python3
# ==============================================================================
# flash_mcu.py — Linorobot2 Intelligent Microcontroller Firmware Flasher
#
# Directives Compliance (AGENTS.md):
# - Strict process safety: targeted PID signaling, no broad pkill/killall (Directive 6).
# - Safe serial port release: stops micro_ros_agent, terminates conflicting port holders.
# - Port immunity: never touch ports 8000, 5173, 9090.
# - Multi-target support: Raspberry Pi Pico / Pico 2 (RP2040/RP2350), ESP32 / ESP32-S3.
# - Robust automatic recovery on flash failure:
#   1. Aggressive port re-release and permission validation.
#   2. RP2040/RP2350: 1200-baud CDC reset pulse to trigger ROM BOOTSEL mode.
#   3. RP2040/RP2350: Mounted UF2 bootloader drive auto-discovery and copy.
#   4. RP2040/RP2350: Unmounted block device detection & temporary auto-mount.
#   5. RP2040/RP2350: Direct picotool discovery (system & PlatformIO toolchains).
#   6. ESP32: Fallback baudrate and manual bootloader mode guidance.
# - Build and flash are ALWAYS separate steps. `pio run -t upload` is never used:
#   PlatformIO only ever compiles here, and every write to the board goes through
#   esptool (ESP32) or picotool (RP2040/RP2350). The two halves normally run on
#   different steps -- `pio run` builds, this script writes the board.
# - Inviolable stopping invariant: If flashing fails, HALT immediately with exit 1.
#   Never proceed to bringup or ROS 2 execution with un-flashed or corrupted firmware.
# ==============================================================================

import hashlib
import json
import tempfile
import argparse
import glob
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Optional, List

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import mcu_identity  # noqa: E402


def log(msg: str):
    print(f"[flash_mcu] {msg}", flush=True)


# Absolute time by which the whole flash attempt (all recovery stages) must end.
# Without it the recovery chain can outlive the caller's own timeout, which kills
# the flasher mid-stage and loses the diagnosis.
_deadline: Optional[float] = None


def set_deadline(total_timeout: int):
    global _deadline
    _deadline = time.monotonic() + total_timeout if total_timeout > 0 else None


def time_left() -> Optional[float]:
    if _deadline is None:
        return None
    return _deadline - time.monotonic()


def out_of_time(stage: str) -> bool:
    left = time_left()
    if left is not None and left <= 0:
        log(f"⏱ Out of time before {stage}; stopping recovery here.")
        return True
    return False


def terminate_group(proc: subprocess.Popen):
    """SIGTERM then SIGKILL the process group started for one flashing tool.

    Targeted at a process group this script created itself (Directive 6) — it is
    never a name-matched kill. Killing only the direct child leaves picotool or
    esptool holding the serial port and the stdout pipe, which is exactly what
    makes a stalled upload look like a hang to the caller.
    """
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
        except ProcessLookupError:
            return
        except Exception:
            pass
        try:
            proc.wait(timeout=3)
            return
        except subprocess.TimeoutExpired:
            continue


def run_tool(cmd: List[str], timeout: int, echo: bool = True, prefix: str = "    | ") -> subprocess.CompletedProcess:
    """Run a flashing tool, streaming its output live and never hanging.

    Output is echoed as it arrives instead of being captured, so a slow upload
    shows progress rather than silence, and a timeout returns 124 with whatever
    the tool managed to say. stderr is folded into stdout; callers that inspect
    .stderr still work because it is set to an empty string.
    """
    if timeout is not None and timeout > 0:
        left = time_left()
        if left is not None:
            timeout = max(5, int(min(timeout, left)))
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1, start_new_session=True)
    except FileNotFoundError as exc:
        log(f"❌ Command not found: {cmd[0]} ({exc})")
        return subprocess.CompletedProcess(cmd, 127, "", str(exc))

    lines: List[str] = []

    def pump():
        try:
            for line in proc.stdout:
                lines.append(line)
                if echo:
                    sys.stdout.write(prefix + line)
                    sys.stdout.flush()
        except Exception:
            pass

    reader = threading.Thread(target=pump, daemon=True)
    reader.start()

    timed_out = False
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
        log(f"⏱ '{os.path.basename(cmd[0])}' exceeded {timeout}s — terminating its process group.")
        terminate_group(proc)
    reader.join(timeout=3)
    try:
        proc.stdout.close()
    except Exception:
        pass

    rc = 124 if timed_out else (proc.returncode if proc.returncode is not None else 1)
    return subprocess.CompletedProcess(cmd, rc, "".join(lines), "")


def is_pico_family(env: str) -> bool:
    env_lower = (env or "").lower()
    return any(k in env_lower for k in ("pico", "rp2040", "rp2350", "rpipico"))


def is_esp_family(env: str) -> bool:
    env_lower = (env or "").lower()
    return any(k in env_lower for k in ("esp32", "esp32s3", "espressif", "gendrv"))



def _ancestry() -> set:
    """This process and every ancestor of it, by pid."""
    pids = set()
    pid = os.getpid()
    while pid > 0 and pid not in pids:
        pids.add(pid)
        try:
            with open(f"/proc/{pid}/stat") as fh:
                # "pid (comm) state ppid ..." -- comm may contain spaces, so split after ')'
                pid = int(fh.read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return pids


def _holders_of(path: str) -> list:
    """Pids with an open fd on `path` (by real path), excluding our own ancestry."""
    try:
        real = os.path.realpath(path)
    except OSError:
        return []
    skip = _ancestry()
    holders = []
    try:
        entries = os.listdir("/proc")
    except OSError:
        return []
    for d in entries:
        if not d.isdigit() or int(d) in skip:
            continue
        fd_dir = f"/proc/{d}/fd"
        try:
            for fd in os.listdir(fd_dir):
                try:
                    if os.readlink(f"{fd_dir}/{fd}") == real:
                        holders.append(int(d))
                        break
                except OSError:
                    continue
        except OSError:
            continue
    return holders

def release_serial_port(serial_port: str):
    """Safely release the serial port from micro_ros_agent or other holders (Directive 6)."""
    if not serial_port or not os.path.exists(serial_port):
        return

    # 1. Safely signal any running micro_ros_agent PID
    try:
        ps_res = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True, timeout=5)
        if ps_res.returncode == 0:
            for line in ps_res.stdout.splitlines():
                if "micro_ros_agent" in line and "grep" not in line and "launch" not in line:
                    parts = line.strip().split()
                    if parts:
                        try:
                            pid = int(parts[0])
                            os.kill(pid, signal.SIGINT)
                            time.sleep(0.2)
                        except Exception:
                            pass
    except Exception:
        pass

    # 2. Signal the processes that actually hold the character device.
    #
    # Not lsof: in a rootless Docker container a passed-through device node
    # resolves to the numbers of /dev/null, so `lsof -t /dev/ttyUSB0` lists
    # EVERY process with /dev/null open -- this one included -- and the SIGINT
    # meant for a stale agent landed on the flasher itself (a
    # KeyboardInterrupt in release_serial_port, seen on a bench host running rootless Docker).
    # /proc says which fds really point at the path, and our own ancestry is
    # never a holder worth signalling.
    try:
        for _ in range(5):
            holders = _holders_of(serial_port)
            if not holders:
                break
            for pid in holders:
                try:
                    os.kill(pid, signal.SIGINT)
                    time.sleep(0.2)
                    os.kill(pid, signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    pass
            time.sleep(0.5)
    except Exception:
        pass


def ensure_port_permissions(serial_port: str):
    """Attempt to ensure read/write permissions on the serial port device."""
    if not serial_port or not os.path.exists(serial_port):
        return
    try:
        os.chmod(serial_port, 0o666)
    except Exception:
        try:
            subprocess.run(["sudo", "-n", "chmod", "666", serial_port], capture_output=True, timeout=2)
        except Exception:
            pass


# The USB port the board we are flashing is plugged into, as sysfs names it
# ("3-2", "7-1.1"). Remembered rather than re-derived, because the tty vanishes
# the moment the board enters BOOTSEL and there is nothing left to derive it
# from afterwards.
_TARGET_USB_PATH: Optional[str] = None


def usb_path_for_tty(port: str) -> Optional[str]:
    """sysfs port path of the USB device behind a tty, or None.

    Resolved through the device NUMBERS, not the name. Inside a container the
    tty is usually a bind mount -- the bench passes the host's /dev/ttyACM1 in
    as /dev/ttyACM0 -- so /sys/class/tty/<basename> is a different board. The
    major:minor pair survives the bind, and identifies exactly one.
    """
    try:
        st = os.stat(port)
    except OSError:
        return None
    want = "%d:%d" % (os.major(st.st_rdev), os.minor(st.st_rdev))
    for dev_file in glob.glob("/sys/class/tty/*/dev"):
        try:
            with open(dev_file) as fh:
                if fh.read().strip() != want:
                    continue
        except OSError:
            continue
        node = os.path.realpath(os.path.join(os.path.dirname(dev_file), "device"))
        # Climb from the CDC interface to the USB device that owns it: the one
        # directory on the way up that has a busnum.
        for _ in range(8):
            if os.path.isfile(os.path.join(node, "busnum")):
                return os.path.basename(node)
            parent = os.path.dirname(node)
            if parent == node or not parent.startswith("/sys"):
                break
            node = parent
    return None


def remember_usb_path(port: str, stamp_path: Optional[str] = None) -> Optional[str]:
    """Pin the flash to ONE board, before anything can make the tty disappear.

    Two RP-series boards on one host is not an exotic setup -- this bench has a
    Pico and a Pico 2 on every test machine -- and picotool refuses to guess:

        ERROR: Command requires a single RP-series device to be targeted.

    It only refuses once BOTH are in BOOTSEL, so serial runs never saw it and
    the first parallel run failed all four RP2 cells at once: each cell touched
    its own board, and then every picotool call in either cell saw two.

    The port path is the right key. Bus and device numbers change when the board
    re-enumerates into BOOTSEL; the physical port it is plugged into does not.
    The USB serial is no good either -- an RP2040 bootrom reports a different
    one from the running application (E0C9125B0D9B vs D665C007DA2A1336 on this
    bench), so --ser would work on RP2350 and silently target nothing on RP2040.
    """
    global _TARGET_USB_PATH
    path = usb_path_for_tty(port) if port else None
    if path is None and stamp_path:
        # The board was already in BOOTSEL when we arrived, so there is no tty
        # to ask. The last flash through this port wrote down where it was.
        path = stamp_path
        log(f"board on {port} is not enumerated as a tty; targeting USB port "
            f"{path} from the last flash's stamp")
    _TARGET_USB_PATH = path
    if path:
        log(f"target board: USB port {path}")
    return path


def stamped_usb_path(env: str, port: str) -> Optional[str]:
    """Where the last flash through this port found the board, if it said."""
    try:
        sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
        import mcu_probe
        return (mcu_probe.read_stamp(env, port) or {}).get("usb_path")
    except Exception:
        return None


def picotool_target(root: str = "/sys/bus/usb/devices") -> List[str]:
    """`--bus`/`--address` for the remembered port, or nothing.

    Read fresh on every call: BOOTSEL is a re-enumeration, so the address
    changes underneath us between the touch and the load.
    """
    if not _TARGET_USB_PATH:
        return []
    dev = os.path.join(root, _TARGET_USB_PATH)
    try:
        with open(os.path.join(dev, "busnum")) as fh:
            bus = fh.read().strip()
        with open(os.path.join(dev, "devnum")) as fh:
            addr = fh.read().strip()
    except OSError:
        return []
    return ["--bus", bus, "--address", addr]


# How long to keep looking for the bootloader after the touch. The board itself
# re-enumerates in well under a second; what takes longer is anything between
# this process and the USB device -- on the bench that is Incus hot-plugging the
# re-enumerated device into the container, measured at ~2 s.
BOOTSEL_WAIT_S = 15.0


def wait_for_bootsel(timeout_s: float = BOOTSEL_WAIT_S) -> bool:
    """Poll until picotool can actually see a board in BOOTSEL, or give up.

    This replaced a flat `time.sleep(2.5)`. A fixed sleep answers the wrong
    question: it guesses how long re-enumeration takes and then tries once. On
    this bench the board reached BOOTSEL correctly every time -- the host logged
    `Product: RP2350 Boot` within a second -- but picotool ran before Incus had
    attached the new device to the container, found nothing, and the flash
    reported "No accessible RP-series devices in BOOTSEL mode were found" about
    a board that was sitting in BOOTSEL. Wait for the condition, not the clock.

    sysfs is not enough to decide this: it can show the bootloader interface
    while /dev/bus/usb still has no entry this process may open. So ask the tool
    that is about to do the work whether it can see the device.
    """
    deadline = time.time() + timeout_s
    binaries = find_picotool_binaries()
    started = time.time()
    while time.time() < deadline:
        for pt in binaries:
            try:
                res = subprocess.run([pt, "info"] + picotool_target(),
                                     capture_output=True, timeout=10)
            except Exception:
                continue
            if res.returncode == 0:
                log(f"bootloader visible to {os.path.basename(pt)} after "
                    f"{time.time() - started:.1f}s")
                return True
        # No picotool at all: fall back to the interface classes, which at least
        # prove the board is there even if this process cannot open it.
        if not binaries and rp2_usb_mode() == "bootsel":
            return True
        time.sleep(0.5)
    log(f"no board in BOOTSEL after {timeout_s:.0f}s")
    return False


# USBDEVFS_RESET, the ioctl behind the `usbreset` one-liner: _IO('U', 20).
_USBDEVFS_RESET = ord("U") << 8 | 20


def usb_reset_target(port: str, settle_s: float = 6.0) -> bool:
    """Reset the target board's USB device, then wait for its tty to return.

    A board whose port was only just released does not reliably answer the
    1200-baud touch: it stays in the application, and the flash then reports a
    BOOTSEL failure about a board that never heard the request. A USB-level
    reset makes it answer every time.

    The bench wrapper had been doing this by hand since 2026-09-19 -- which is
    why the six-cell hardware pass succeeded and the six-cell Nav2 pass, which
    calls the flasher through the pipeline instead, failed every RP2 board with
    `no board in BOOTSEL after 15s`. Knowledge that only lives in a test wrapper
    is knowledge the product does not have.
    """
    sel = picotool_target()
    if len(sel) != 4:
        return False
    bus, addr = int(sel[1]), int(sel[3])
    node = "/dev/bus/usb/%03d/%03d" % (bus, addr)
    try:
        import fcntl
        fd = os.open(node, os.O_WRONLY)
        try:
            fcntl.ioctl(fd, _USBDEVFS_RESET, 0)
        finally:
            os.close(fd)
    except Exception as exc:
        log(f"(usb reset of {node} not possible: {exc})")
        return False
    log(f"usb reset {node}")
    deadline = time.time() + settle_s
    while time.time() < deadline:
        if port and os.path.exists(port):
            time.sleep(0.5)
            return True
        time.sleep(0.2)
    return True


def pulse_1200_baud(port: str) -> bool:
    """Pulse serial port at 1200 baud to signal Pico CDC bootloader reboot.

    Returns once the board is in BOOTSEL and reachable, not after a fixed nap.
    """
    if not port or not os.path.exists(port):
        return False
    usb_reset_target(port)
    log(f"Attempting 1200-baud CDC pulse on {port} to trigger BOOTSEL mode...")
    touched = False
    try:
        import serial
        s = serial.Serial(port, baudrate=1200, timeout=1)
        time.sleep(0.3)
        s.close()
        touched = True
    except Exception as e:
        log(f"pyserial 1200 baud notice: {e}")
        try:
            subprocess.run(["stty", "-F", port, "1200"], capture_output=True, timeout=3)
            touched = True
        except Exception:
            return False
    if not touched:
        return False
    # True even when the wait times out: the touch was sent, and the callers
    # have their own recoveries plus report_failed_bootsel_request() to tell a
    # board that refused BOOTSEL from one that was merely slow to appear.
    wait_for_bootsel()
    return True


# USB interface classes, as sysfs spells them (hex, zero padded).
_IFCLASS_MASS_STORAGE = "08"
_IFCLASS_CDC = ("02", "0a")


def _rp2_mode_from_sysfs(root: str = "/sys/bus/usb/devices") -> Optional[str]:
    """The same answer as lsusb, read straight out of sysfs.

    sysfs is the source lsusb itself reads, minus the binary, the dynamic
    loader and the permissions. It cannot fail to start, which lsusb can and
    does -- see rp2_usb_mode below. Returns None when there is no sysfs to
    read, which is the one case that genuinely needs the fallback.
    """
    if not os.path.isdir(root):
        return None
    classes, found = [], False
    try:
        names = os.listdir(root)
    except OSError:
        return None
    # With the board identified, answer about THAT board. Otherwise a Pico in
    # BOOTSEL next to a Pico 2 running the application reports "bootsel" for
    # both, and report_failed_bootsel_request() -- which only fires on "app" --
    # stays silent about the one that actually refused.
    if _TARGET_USB_PATH and _TARGET_USB_PATH in names:
        names = [_TARGET_USB_PATH]
    for name in names:
        dev = os.path.join(root, name)
        try:
            with open(os.path.join(dev, "idVendor")) as fh:
                if fh.read().strip().lower() != "2e8a":
                    continue
        except OSError:
            continue
        found = True
        try:
            ifaces = os.listdir(dev)
        except OSError:
            continue
        for iface in ifaces:
            path = os.path.join(dev, iface, "bInterfaceClass")
            try:
                with open(path) as fh:
                    classes.append(fh.read().strip().lower())
            except OSError:
                continue
    if not found:
        return "absent"
    if _IFCLASS_MASS_STORAGE in classes:
        return "bootsel"
    if any(c in classes for c in _IFCLASS_CDC):
        return "app"
    return "unknown"


def rp2_usb_mode() -> str:
    """Which mode an RP-series board is in, read from its interface classes.

    The PID is 2e8a:000f in both application mode and BOOTSEL, so it proves
    nothing; the interface classes do. Returns "bootsel", "app", "absent", or
    "unknown" when neither source can answer.

    sysfs first, and lsusb only as a fallback, because lsusb is a binary that
    can fail to start. On an Ubuntu 26.04 bench it did, every time:

        lsusb: error while loading shared libraries: libc.so.6: cannot apply
        additional memory protection after relocation: Permission denied

    -- exit 127, no output. The old code read any non-zero exit as "absent",
    so every probe on that host returned a confident "nothing is on that port"
    about a board that was sitting there running micro-ROS. That verdict
    writes neither firmware nor env block while printing "Nothing to write",
    and it suppressed report_failed_bootsel_request(), which only fires on
    "app" -- so the one message explaining a failed flash never printed. "I
    could not look" is not "there is nothing there", and it must never again
    be reported as though it were.
    """
    mode = _rp2_mode_from_sysfs()
    if mode is not None:
        return mode
    try:
        res = subprocess.run(["lsusb", "-d", "2e8a:", "-v"],
                             capture_output=True, text=True, timeout=10)
    except Exception:
        return "unknown"
    if res.returncode != 0:
        # Could not ask. Say so.
        return "unknown"
    if not res.stdout.strip():
        return "absent"
    classes = [ln.split("bInterfaceClass")[1].strip()
               for ln in res.stdout.splitlines() if "bInterfaceClass" in ln]
    blob = " ".join(classes)
    if "Mass Storage" in blob:
        return "bootsel"
    if "Communications" in blob or "CDC" in blob:
        return "app"
    return "unknown"


def report_failed_bootsel_request(port: str):
    """Say what a board that swallowed the touch and stayed in CDC actually is.

    The touch is handled in the core, in SerialUSB::checkSerialReset(): it tears
    down the USB block and calls reset_usb_boot(), which on RP2350 is a
    rom_reboot() whose failure is not checked -- the core then sits in
    `while (1); // WDT will fire here`. A board there is still enumerated, mute
    at every baud, and answers no control transfer. That is a hang, not a
    missing BOOTSEL, and it does not resolve by retrying from this side.
    """
    log("⚠️ The board answered the 1200-baud touch and did NOT reach BOOTSEL.")
    log("   It is still enumerated as CDC, which means the core's BOOTSEL request")
    log("   failed and it is now parked in SerialUSB::checkSerialReset()'s while(1)")
    log("   with USB torn down. Nothing on this side can reach it.")
    log("   • Firmware built after the RP2 watchdog was armed reboots itself within")
    log("     ~8 s -- wait, then retry.")
    log("   • Otherwise press the RESET button on the board (BOOTSEL is not needed),")
    log("     then run the flash again.")


def find_mounted_uf2_volume(env: str = "pico2") -> Optional[str]:
    """Search for already mounted Raspberry Pi Pico / RP2040 / RP2350 USB mass storage volumes."""
    is_pico2 = "2350" in env.lower() or "pico2" in env.lower()
    preferred_names = ["RP2350", "rp2350"] if is_pico2 else ["RPI-RP2", "rpi-rp2"]
    fallback_names = ["RPI-RP2", "rpi-rp2"] if is_pico2 else ["RP2350", "rp2350"]

    search_dirs = ["/run/media", "/media", "/mnt"]
    for base in search_dirs:
        if not os.path.exists(base):
            continue
        try:
            for root, dirs, files in os.walk(base, followlinks=True):
                depth = root[len(base):].count(os.sep)
                if depth > 3:
                    continue
                if "INFO_UF2.TXT" in files:
                    # Check if folder name matches preferred target family
                    r_base = os.path.basename(root)
                    if any(pn in r_base for pn in preferred_names):
                        log(f"Found mounted UF2 volume ({r_base}) at: {root}")
                        return root
        except Exception:
            pass

    # Glob search with preferred order
    for group in [preferred_names, fallback_names]:
        for pat_name in group:
            patterns = [
                f"/run/media/*/*{pat_name}*",
                f"/media/*/*{pat_name}*",
                f"/media/*/*/*{pat_name}*",
                f"/mnt/*{pat_name}*",
            ]
            for pat in patterns:
                for m in glob.glob(pat):
                    if os.path.isdir(m) and os.path.exists(os.path.join(m, "INFO_UF2.TXT")):
                        log(f"Found mounted UF2 volume via glob: {m}")
                        return m
    return None


def find_and_mount_pico_device(env: str = "pico2") -> Optional[str]:
    """
    Search for unmounted Pico block devices (common in headless Linux and Distrobox)
    and mount temporarily to /tmp/pico_bootloader. Prioritizes the target family.
    """
    is_pico2 = "2350" in env.lower() or "pico2" in env.lower()
    labels = ["RP2350", "rp2350"] if is_pico2 else ["RPI-RP2", "rpi-rp2"]
    # Fallback to general RP labels only if target family not found
    labels += ["RPI-RP2", "rpi-rp2"] if is_pico2 else ["RP2350", "rp2350"]

    by_label_dir = "/dev/disk/by-label"
    target_dev = None

    if os.path.exists(by_label_dir):
        for lbl in labels:
            p = os.path.join(by_label_dir, lbl)
            if os.path.exists(p):
                target_dev = os.path.realpath(p)
                break

    if not target_dev:
        try:
            res = subprocess.run(["lsblk", "-o", "PATH,LABEL,MOUNTPOINT", "-P"], capture_output=True, text=True, timeout=4)
            if res.returncode == 0:
                for line in res.stdout.splitlines():
                    for lbl in labels:
                        if f'LABEL="{lbl}"' in line:
                            m_path = re.search(r'PATH="([^"]+)"', line)
                            m_mount = re.search(r'MOUNTPOINT="([^"]*)"', line)
                            if m_path:
                                dev = m_path.group(1)
                                if m_mount and m_mount.group(1):
                                    return m_mount.group(1)
                                target_dev = dev
                                break
                    if target_dev:
                        break
        except Exception:
            pass

    if target_dev and os.path.exists(target_dev):
        temp_mount = "/tmp/pico_bootloader"
        os.makedirs(temp_mount, exist_ok=True)
        log(f"Found unmounted Pico block device {target_dev}. Attempting temporary mount to {temp_mount}...")
        # Mount with open permissions and current uid/gid
        uid = os.getuid()
        gid = os.getgid()
        res = subprocess.run(["sudo", "-n", "mount", "-o", f"umask=000,uid={uid},gid={gid}", target_dev, temp_mount], capture_output=True, text=True, timeout=5)
        if res.returncode == 0:
            return temp_mount
        # Try simple mount without options
        res2 = subprocess.run(["sudo", "-n", "mount", target_dev, temp_mount], capture_output=True, text=True, timeout=5)
        if res2.returncode == 0:
            return temp_mount
        # Try udisksctl as non-root fallback
        u_res = subprocess.run(["udisksctl", "mount", "-b", target_dev], capture_output=True, text=True, timeout=5)
        if u_res.returncode == 0:
            words = u_res.stdout.strip().split()
            if words and os.path.isdir(words[-1]):
                return words[-1]
    return None


def flash_via_uf2_copy(uf2_path: str, mount_point: str) -> bool:
    """Copy UF2 binary to mounted volume, sync filesystem, and await reboot."""
    if not os.path.exists(uf2_path) or not os.path.isdir(mount_point):
        return False
    try:
        log(f"Copying {os.path.basename(uf2_path)} to {mount_point}...")
        dest = os.path.join(mount_point, os.path.basename(uf2_path))
        try:
            shutil.copy2(uf2_path, dest)
        except (PermissionError, OSError) as e:
            log(f"Standard copy hit permission notice ({e}); retrying with sudo cp...")
            cp_res = subprocess.run(["sudo", "-n", "cp", uf2_path, dest], capture_output=True, text=True, timeout=5)
            if cp_res.returncode != 0:
                raise Exception(f"sudo cp failed: {cp_res.stderr or cp_res.stdout}")
        subprocess.run(["sync"], check=False)
        time.sleep(1.0)
        if mount_point == "/tmp/pico_bootloader":
            subprocess.run(["sudo", "-n", "umount", mount_point], capture_output=True, check=False)
        log("✅ Firmware flashed successfully via UF2 mass storage drive! Waiting for reboot...")
        time.sleep(3.0)
        return True
    except Exception as e:
        log(f"UF2 copy error: {e}")
        return False


def find_picotool_binaries() -> List[str]:
    """Discover all picotool executables across system PATH and PlatformIO package directories."""
    candidates = []
    which_pt = shutil.which("picotool")
    if which_pt:
        candidates.append(which_pt)

    search_roots = [
        os.environ.get("PLATFORMIO_CORE_DIR", os.path.expanduser("~/.platformio")) + "/packages",
        os.path.expanduser("~/.platformio/packages"),
        "/pio/packages",
    ]
    for root in search_roots:
        if os.path.exists(root):
            for pat in [
                os.path.join(root, "tool-picotool*", "picotool"),
                os.path.join(root, "tool-picotool*", "bin", "picotool"),
            ]:
                candidates.extend(glob.glob(pat))

    candidates.extend(["/usr/bin/picotool", "/usr/local/bin/picotool"])
    # Deduplicate while preserving order
    seen = set()
    valid = []
    for c in candidates:
        if c not in seen and os.path.isfile(c) and os.access(c, os.X_OK):
            seen.add(c)
            valid.append(c)
    return valid


def flash_via_picotool(uf2_path: str, env: str = "pico2",
                       env_bin: Optional[str] = None) -> bool:
    """Attempt direct upload via picotool with family filter to prevent device confusion.

    The env block goes down FIRST when one is supplied. Both writes need the
    board in BOOTSEL, and loading the application with `-x` runs it, which leaves
    BOOTSEL -- so the application has to be the last thing written. Doing it here
    rather than at the call site means the recovery paths provision the board too.
    """
    if not os.path.exists(uf2_path):
        return False
    pts = find_picotool_binaries()
    if not pts:
        log("No picotool binaries discovered on system.")
        return False

    if env_bin:
        flash_env_via_picotool(env_bin, env)

    # Ensure open permissions on USB bus
    try:
        subprocess.run(["sudo", "-n", "chmod", "-R", "a+rw", "/dev/bus/usb"], capture_output=True, timeout=2)
    except Exception:
        pass

    is_rp2350 = "2350" in env.lower() or "pico2" in env.lower()
    family_flag = ["--family", "rp2350-arm-s"] if is_rp2350 else ["--family", "rp2040"]
    # Which board. The family only says which CHIP; with two RP2350s, or with a
    # Pico and a Pico 2 both in BOOTSEL, picotool still has a choice to make and
    # refuses to make it. See remember_usb_path().
    target = picotool_target()
    # `-f` (force a RUNNING board to reset) is picotool's OWN way of choosing a
    # device, and it refuses to combine with an explicit one:
    #     ERROR: unexpected option: --bus
    # So the forced variants are only tried when we have nothing better. No
    # loss: forcing never worked on this firmware anyway -- it exposes no
    # picotool reset interface (AGENTS.md §6) -- and a board we can name is a
    # board we have already identified through its tty.
    forced = [] if target else ["-f"]

    for pt in pts:
        log(f"Attempting direct picotool upload with: {pt} ({' '.join(family_flag + target)})...")
        # Standard load with family constraint
        cmd = [pt, "load", "-x"] + family_flag + [uf2_path] + target
        res = run_tool(cmd, timeout=20)
        if res.returncode == 0:
            log(f"✅ Firmware flashed successfully via {pt} ({' '.join(family_flag)})!")
            time.sleep(3.0)
            return True

        # If permission denied, try with sudo -n
        out_combined = ((res.stderr or "") + (res.stdout or "")).lower()
        if "permission" in out_combined or "sudo" in out_combined:
            sudo_cmd = ["sudo", "-n", pt, "load", "-x"] + family_flag + [uf2_path] + target
            sudo_res = run_tool(sudo_cmd, timeout=20)
            if sudo_res.returncode == 0:
                log(f"✅ Firmware flashed successfully via sudo {pt} ({' '.join(family_flag)})!")
                time.sleep(3.0)
                return True

        # Force load, when forcing is still on the table at all.
        res_f = res
        out_f_combined = out_combined
        if forced:
            cmd_f = [pt, "load", "-f", "-x"] + family_flag + [uf2_path]
            res_f = run_tool(cmd_f, timeout=20)
            if res_f.returncode == 0:
                log(f"✅ Firmware flashed successfully via {pt} (-f {' '.join(family_flag)})!")
                time.sleep(3.0)
                return True

            out_f_combined = ((res_f.stderr or "") + (res_f.stdout or "")).lower()
            if "permission" in out_f_combined or "sudo" in out_f_combined:
                sudo_cmd_f = ["sudo", "-n", pt, "load", "-f", "-x"] + family_flag + [uf2_path]
                sudo_res_f = run_tool(sudo_cmd_f, timeout=20)
                if sudo_res_f.returncode == 0:
                    log(f"✅ Firmware flashed successfully via sudo {pt} "
                        f"(-f {' '.join(family_flag)})!")
                    time.sleep(3.0)
                    return True

        # Fallback without family constraint if older picotool didn't support --family
        if "unknown option" in out_combined or "unknown option" in out_f_combined:
            plain_res = run_tool([pt, "load", "-x", uf2_path] + target, timeout=20)
            if plain_res.returncode == 0:
                log(f"✅ Firmware flashed successfully via {pt} (plain)!")
                time.sleep(3.0)
                return True

        if "single rp-series device" in (out_combined + out_f_combined) and not target:
            # Naming the real problem. The generic message that follows this
            # ("not in BOOTSEL") is wrong here and sends the user to hold down
            # a button that is not the issue.
            log("⚠️  more than one RP-series board is in BOOTSEL and this run "
                "could not tell which one to write. Plug the board in while it "
                "is RUNNING (so the tty exists and identifies it), or leave "
                "only one board in BOOTSEL.")
        log(f"picotool attempt ({pt}) failed: {res_f.stderr or res.stderr or res_f.stdout}")

    return False


def flash_env_via_picotool(env_bin: str, env: str = "pico2") -> bool:
    """Write the env block to an RP2 board's top flash sector.

    The counterpart of the `env` partition on ESP32. There is no partition table
    here, so the address comes from scripts/mcu_env.py, which derives it the way
    arduino-pico's builder does: the last 4 KB of flash, above both the sketch
    and the filesystem. Writing it is therefore independent of the application --
    and, for the same reason, flashing the application never disturbs it.

    Written as a separate picotool invocation rather than merged into the UF2 so
    that re-keying a board does not involve the application image at all.
    """
    if not env_bin or not os.path.isfile(env_bin):
        return False
    pts = find_picotool_binaries()
    if not pts:
        log("No picotool binaries discovered; cannot write the env block.")
        return False

    offset = rp2_env_offset(env)
    if offset is None:
        log(f"No env offset known for '{env}'; skipping the env block.")
        return False

    is_rp2350 = "2350" in env.lower() or "pico2" in env.lower()
    family_flag = ["--family", "rp2350-arm-s"] if is_rp2350 else ["--family", "rp2040"]

    for pt in pts:
        # -t bin says the file is raw rather than UF2; -o places it. Both are
        # required: without -t picotool guesses from the extension, and picotool's
        # default load address for a bin IS 0x10000000, so without -o the block
        # would land at the start of flash, over the application. -v verifies --
        # 4 KB is cheap to read back, and a silently bad env block presents later
        # as a board that ignores its configuration.
        #
        # ORDER MATTERS, and getting it wrong looks like a device problem rather
        # than a syntax one. picotool's synopsis is
        #     load [--family <id>] [-v] [-x] <filename> [-t <type>] [-o <offset>]
        # -- `-t` and `-o` are positional-ish options that belong AFTER the
        # filename. Passing them before it (the natural reading) makes picotool
        # 2.1.1 answer `ERROR: unexpected option: -o`, and since this runs inside
        # the recovery path whose next step is a successful application load, the
        # whole flash still reported success while the board kept whatever env it
        # already had. Found on a 2026-09-16 rig run: a board flashed on a
        # fresh box had no env block at all and fell back to the header.
        target = picotool_target()
        base = ([pt, "load"] + family_flag + ["-v", env_bin, "-t", "bin",
                "-o", hex(offset)] + target)
        forms = [base, ["sudo", "-n"] + base]
        # -f only without an explicit device: picotool treats the two as rival
        # ways of saying which board, and rejects the pair outright.
        if not target:
            forms.append(base[:2] + ["-f"] + base[2:])
        for cmd in forms:
            # (base[:2] is [picotool, load], so -f still precedes the filename.)
            res = run_tool(cmd, timeout=20)
            if res.returncode == 0:
                log(f"✅ env block written at {offset:#x} via {pt}")
                time.sleep(1.0)
                return True
        log(f"env write via {pt} failed: {res.stderr or res.stdout}")

    return False


def find_esptool() -> Optional[List[str]]:
    """argv prefix that runs esptool: system binary, importable module, or PlatformIO's copy."""
    for name in ("esptool", "esptool.py"):
        found = shutil.which(name)
        if found:
            return [found]
    try:
        subprocess.run([sys.executable, "-m", "esptool", "version"],
                       capture_output=True, timeout=15, check=True)
        return [sys.executable, "-m", "esptool"]
    except Exception:
        pass
    for candidate in sorted(glob.glob(os.path.expanduser(
            "~/.platformio/packages/tool-esptoolpy*/esptool.py"))):
        if os.path.isfile(candidate):
            return [sys.executable, candidate]
    return None


def esp32_flash_plan(build_dir: str, env: str) -> List[tuple]:
    """
    [(offset, file)] for an ESP32 PlatformIO build.

    firmware.bin alone is not a bootable image -- without bootloader.bin and
    partitions.bin at their ESP-IDF offsets the chip lands in a reset loop.
    ESP32-S3/C3 put the bootloader at 0x0, the classic ESP32 at 0x1000.
    """
    merged = os.path.join(build_dir, "firmware-merged.bin")
    if os.path.isfile(merged):
        return [("0x0", merged)]

    boot_offset = "0x0" if re.search(r"esp32(s3|c3|c6)", env or "", re.I) else "0x1000"
    plan = []
    for name, offset in (("bootloader.bin", boot_offset), ("partitions.bin", "0x8000")):
        path = os.path.join(build_dir, name)
        if os.path.isfile(path):
            plan.append((offset, path))
    for boot_app0 in sorted(glob.glob(os.path.expanduser(
            "~/.platformio/packages/framework-arduinoespressif32*/tools/partitions/boot_app0.bin"))):
        plan.append(("0xe000", boot_app0))
        break
    firmware = os.path.join(build_dir, "firmware.bin")
    if os.path.isfile(firmware):
        plan.append(("0x10000" if plan else "0x0", firmware))
    return plan


ENV_PARTITION_OFFSET = "0x3FF000"   # the `env` row of firmware/common/partitions_lino.csv


def rp2_env_offset(env: str) -> Optional[int]:
    """Top-sector address the env block goes to on an RP2 board, or None.

    Deferred to scripts/mcu_env.py so the address is stated once: the firmware
    reads the sector arduino-pico reserves, and two copies of that arithmetic
    would be two chances to disagree with it.
    """
    sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
    try:
        import mcu_env as _mcu_env
        name = (env or "").replace("_lyrical", "")
        return _mcu_env.RP2_ENV_OFFSETS.get(name)
    except Exception as exc:
        log(f"could not resolve the RP2 env offset: {exc}")
        return None


# Parts with a hardware DAC, the same condition as ADC_LUT_SUPPORTED in
# firmware/common/lib/adc_lut/adc_lut.h: the classic ESP32 and the ESP32-S2.
# The S3, C3, C6 and H2 have none, and neither do the RP2040/RP2350.
_DAC_ENVS = ("esp32", "esp32s2")


def warn_if_app_unsupported(app: str, env: str) -> None:
    """Say so HERE when the board cannot run the application being selected.

    `adc_calibrate` is compiled in only on a part with a hardware DAC; elsewhere
    the name is not in the firmware's tool table at all and `toolSelect()` falls
    back to the robot firmware. That fallback is correct and deliberate -- but
    its only trace is one line on the board's serial at boot, which nobody is
    attached to during a 4 KB env write. So the write "succeeded", the flasher
    reported success, and the board came up running `base` with nothing to
    explain it. Measured on a Pico 2, 2026-09-21. Warn where the choice is made.
    """
    if app != "adc_calibrate":
        return
    name = (env or "").strip().lower()
    if name in _DAC_ENVS:
        return
    log(f"\u26a0\ufe0f  '{app}' needs a hardware DAC; '{env or 'this board'}' has none "
        f"(only {', '.join(_DAC_ENVS)} do).")
    log("    The env key will be written, but the image does not carry that "
        "application:")
    log("    the board will print '[app] ... is not an application this image "
        "carries' and boot `base`.")


def resolve_env_bin(args, prebuilt_dir: Optional[str]) -> Optional[str]:
    """The env block to write alongside an application image, built on demand.

    Used by both families. On ESP32 it lands in the `env` partition; on RP2 in
    the top flash sector arduino-pico reserves (see flash_env_via_picotool).

    The application image carries no Wi-Fi credentials and no agent, syslog or
    LiDAR-UDP address — those are site facts, and an image with them compiled in
    would work on one LAN only. They go into their own flash partition instead,
    so this generates the 4 KB block from the config directory's secrets.yaml at flash time.
    """
    if args.env_bin:
        return args.env_bin
    params = getattr(args, "params", None)
    if not prebuilt_dir and not params:
        return None          # a local build still has its own header values
    if prebuilt_dir:
        manifest_path = os.path.join(prebuilt_dir, "manifest.json")
        with open(manifest_path) as fh:
            manifest = json.load(fh)
        if "env_partition" not in manifest and not params:
            return None      # a serial profile has nothing to provision
        # --params wins. A prebuilt image was compiled for whichever robot the
        # build script happened to name, and its header fallbacks are that
        # robot's pins; for the board actually in front of the user the env is
        # not an override, it is the only true description of the hardware.
        params = params or manifest["config"]

    out = os.path.join(tempfile.gettempdir(), f"lino_env_{os.getpid()}.bin")
    cmd = [sys.executable, os.path.join(REPO_ROOT, "scripts", "mcu_env.py"), "build",
           "--params", os.path.abspath(params),
           "--board", getattr(args, "env", "") or "",
           "--out", out]
    # Which application the unified image boots. Written even when it is "base",
    # so that flashing the robot firmware over a board left in a diagnostic mode
    # actually returns it to the robot firmware.
    app = getattr(args, "app", None) or "base"
    warn_if_app_unsupported(app, getattr(args, "env", "") or "")
    cmd += ["--set", f"app={app}"]
    log("Building the env block (Wi-Fi keys + agent/syslog/lidar addresses)...")
    res = run_tool(cmd, timeout=30)
    if res.returncode != 0 or not os.path.isfile(out):
        log("⚠️  could not build the env block; flashing the application only.")
        log("    The board will report a missing env partition over serial.")
        return None
    return out


def flash_via_esptool(build_dir: str, env: str, port: str, baud: int, timeout: int = 300,
                      env_bin: Optional[str] = None) -> bool:
    """Write an ESP32 build with esptool. The only way this script touches an ESP32."""
    esptool_argv = find_esptool()
    if not esptool_argv:
        log("esptool not found. Install it with: pip install esptool")
        return False

    plan = esp32_flash_plan(build_dir, env)
    if env_bin and os.path.isfile(env_bin):
        plan.append((ENV_PARTITION_OFFSET, env_bin))
    if not plan:
        log(f"No .bin images found in {build_dir} — was the firmware built?")
        return False

    cmd = esptool_argv + ["--port", port, "--baud", str(baud), "write_flash", "-z"]
    for offset, path in plan:
        cmd += [offset, path]
        log(f"    {offset} <- {os.path.basename(path)}")
    res = run_tool(cmd, timeout=timeout)
    return res.returncode == 0


def rp2_reboot_into_app(env: str) -> bool:
    """Leave BOOTSEL and start the application again.

    `picotool load` without `-x` does not run anything, so an --env-only write
    finishes with the board sitting in BOOTSEL: the tty is gone, the agent has
    nothing to talk to, and the robot looks dead for a reason that has nothing to
    do with what was written. The application load path never needed this because
    it passes `-x`.

    `picotool reboot -f` reaches the board here for the same reason the write did
    -- it is in BOOTSEL, which is the one state whose USB interface accepts it.
    This is NOT the `reboot -f -u` that cannot reach a RUNNING board (AGENTS.md
    §6): that fails because our firmware exposes no picotool reset interface, and
    it is the opposite situation.
    """
    # No --family here: `picotool reboot` (2.x) takes only the device-selection
    # options, and the family variant used to be tried first, so every env-only
    # write ended with two `ERROR: unexpected option` blocks before the plain
    # form succeeded. The family filter belongs to `load`, where it stops an
    # image built for one RP2 landing on the other.
    target = picotool_target()
    forced = [] if target else ["-f"]
    for pt in find_picotool_binaries():
        for cmd in ([pt, "reboot"] + forced + target,
                    ["sudo", "-n", pt, "reboot"] + forced + target):
            if run_tool(cmd, timeout=15).returncode == 0:
                log("✅ board rebooted into the application")
                return True
    log("⚠️  could not reboot the board out of BOOTSEL; press RESET or replug it.")
    return False


def esp32_reset_into_app(port: str, baud: int) -> bool:
    """Start the application again after a write. See mcu_probe.esp32_boot_app().

    One implementation, because the failure is the same one in both places: a
    serial open can leave this board in the ROM download mode, and nothing
    downstream can tell that from a board that simply booted too fast to be
    heard. Kept as a named wrapper so the flash path reads like its RP2
    counterpart, rp2_reboot_into_app().
    """
    try:
        sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
        import mcu_probe
    except Exception as exc:
        log(f"⚠️  cannot reset the board ({exc}); press EN or replug it "
            f"if the robot stays silent.")
        return False
    if mcu_probe.esp32_boot_app(port, baud):
        return True
    log(f"⚠️  could not pulse EN on {port}; press EN or replug it "
        f"if the robot stays silent.")
    return False


def flash_env_via_esptool(env_bin: str, env: str, port: str, baud: int,
                          timeout: int = 60) -> bool:
    """Write ONLY the `env` partition on an ESP32, leaving the application alone.

    This is the cheap operation the whole one-image design is built around
    (AGENTS.md §5): changing the robot's pins, its transport, its credentials or
    which application it boots is a 4 KB write here, and the application image is
    not touched -- not rebuilt, not restaged, not rewritten. esptool addresses
    the partition directly, so nothing else on the chip is read or erased.
    """
    esptool_argv = find_esptool()
    if not esptool_argv:
        log("esptool not found. Install it with: pip install esptool")
        return False
    cmd = esptool_argv + ["--port", port, "--baud", str(baud), "write_flash", "-z",
                          ENV_PARTITION_OFFSET, env_bin]
    log(f"    {ENV_PARTITION_OFFSET} <- {os.path.basename(env_bin)} (env partition only)")
    return run_tool(cmd, timeout=timeout).returncode == 0


def print_failure_troubleshooting(target_name: str, env: str, port: str, uf2_path: str):
    """Output comprehensive, prominent troubleshooting guidance to help user resolve hardware flash failures."""
    print("\n" + "=" * 80, flush=True)
    print(f"❌ [FLASH FAILED] Microcontroller Firmware Upload Failed for Target '{target_name}'", flush=True)
    print(f"   Environment: {env} | Port: {port}", flush=True)
    print("=" * 80, flush=True)
    print("⛔ HALTING EXECUTION: Cockpit will NOT proceed to bringup or ROS 2 navigation!", flush=True)
    print("   Starting bringup without valid firmware causes agent timeouts and hardware faults.", flush=True)

    print("-" * 80, flush=True)
    print("📋 REQUIRED USER ACTION TO RESOLVE FLASHING:", flush=True)

    # One line the cockpit can lift into a banner: the single thing to do
    # next. The numbered steps below are the long form.
    if is_pico_family(env):
        print("NEXT ACTION: Unplug the board, hold BOOTSEL, plug it back in, release BOOTSEL, then press Start again.", flush=True)
    elif is_esp_family(env):
        print(f"NEXT ACTION: Unplug and replug the USB cable (or hold BOOT, tap EN, release BOOT), check {port} exists, then press Start again.", flush=True)
    else:
        print(f"NEXT ACTION: Unplug and replug the USB cable, check {port} exists, then press Start again.", flush=True)

    if is_pico_family(env):
        print(f"\n👉 For Raspberry Pi Pico / Pico 2 (RP2040 / RP2350):", flush=True)
        print("   1. UNPLUG the USB cable from the Pico board.", flush=True)
        print("   2. Press and HOLD the white 'BOOTSEL' button on the board.", flush=True)
        print("   3. While holding 'BOOTSEL', PLUG the USB cable back into the computer.", flush=True)
        print("   4. RELEASE the 'BOOTSEL' button.", flush=True)
        print("      • The Pico will enter ROM Bootloader mode and appear as a USB drive", flush=True)
        print("        named 'RPI-RP2' (RP2040) or 'RP2350' (RP2350).", flush=True)
        if os.path.exists(uf2_path):
            print(f"   5. Manual Drag & Drop Option:", flush=True)
            print(f"      • Copy the compiled firmware directly to the mounted drive:", flush=True)
            print(f"        cp {uf2_path} /media/$USER/RPI-RP2/   (or RP2350)", flush=True)
        print("   6. Container / Distrobox Environment Check:", flush=True)
        print("      • Ensure USB devices are passed into Distrobox: verify with 'lsusb' or 'ls /dev/ttyACM*'.", flush=True)
        print("      • Fix device permissions: 'sudo chmod 666 /dev/ttyACM*'.", flush=True)
        print("   7. Confirm which mode the board is in — the PID is the same either way:", flush=True)
        print("      lsusb -d 2e8a: -v | grep bInterfaceClass", flush=True)
        print("      • Communications / CDC  -> running the application, a tty exists", flush=True)
        print("      • Mass Storage / Vendor -> BOOTSEL, picotool can load", flush=True)
        print("   8. Click 'Start 1-Click' or 'Flash MCU' once plugged in!", flush=True)

    elif is_esp_family(env):
        print(f"\n👉 For ESP32 / ESP32-S3 / GenDrv Board:", flush=True)
        print("   1. UNPLUG and RE-PLUG the USB cable to reset the USB-UART bridge.", flush=True)
        print("   2. Manual Download Mode:", flush=True)
        print("      • Press and HOLD the 'BOOT' (GPIO0) button on the board.", flush=True)
        print("      • Press and release the 'EN' (or 'RST') reset button.", flush=True)
        print("      • Release the 'BOOT' button. (The ESP32 is now in bootloader mode).", flush=True)
        print(f"   3. Check Port Existence & Permissions:", flush=True)
        print(f"      • Verify port: ls -l {port} or ls -l /dev/ttyUSB*", flush=True)
        print(f"      • Grant access: sudo chmod 666 {port}", flush=True)
        print("      • Ensure dialout group membership: sudo usermod -aG dialout $USER", flush=True)
        print("   4. Verify the USB cable supports data (not power-only).", flush=True)
        print("   5. Re-run 'Start 1-Click' or 'Flash MCU' in Cockpit!", flush=True)
    else:
        print(f"\n👉 General Microcontroller Recovery:", flush=True)
        print("   1. Unplug and replug the USB cable.", flush=True)
        print("   2. Check serial port connection: 'ls -l /dev/ttyACM* /dev/ttyUSB*'.", flush=True)
        print(f"   3. Ensure permissions: 'sudo chmod 666 {port}'.", flush=True)
        print("   4. Re-run flashing once re-connected.", flush=True)

    print("=" * 80 + "\n", flush=True)



def record_stamp(env: str, port: str, app: Optional[str], env_bin: Optional[str],
                 params: Optional[str], baud: int, app_written: bool,
                 prebuilt_dir: Optional[str] = None):
    """Write down what this host just put on the board, and try to hear it back.

    This is the other half of scripts/mcu_probe.py. The board announces itself at
    boot -- `[fw] linorobot2_hardware app=... built=... git=...` -- and a flash is
    the one moment the host knows a reboot is about to happen, so it is the one
    moment the banner can be caught reliably. Catching it turns the stamp from
    "what I believe I wrote" into "what the board says it is running".

    If the banner is missed (the port takes a moment to re-enumerate and the line
    is printed within milliseconds of boot), the stamp still records what was
    written, flagged as unconfirmed. Recording nothing would send the next run
    back to reflashing blind, which is the behaviour this whole mechanism exists
    to end.
    """
    try:
        sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
        import mcu_probe
        import cockpit_paths
    except Exception as exc:
        log(f"(no stamp recorded: {exc})")
        return

    stamp = dict(mcu_probe.read_stamp(env, port))
    # prebuilt_dir MATTERS. local_build() answers "what would this machine
    # flash", and without the directory it answers for the source tree -- which
    # inside the published Docker image has no .git at all, so it returns
    # git="unknown". Stamping that after writing a release image makes the very
    # next probe compare "unknown" against the manifest's revision, call the
    # board stale, and reflash it. Every run. On an RP2 that is merely a wasted
    # minute (picotool leaves the board running either way); on an ESP32 it is
    # fatal, because the re-flash leaves the board not running and bringup then
    # finds no /odom, no /imu/data and no /scan.
    local = mcu_probe.local_build(env, prebuilt_dir)
    if app_written:
        stamp["git"] = local.get("git")
        # The distro is a property of the IMAGE and only of the image, so it is
        # recorded only when the image was actually written -- an --env-only
        # write must not restamp it (§10).
        stamp["distro"] = local.get("distro")
        stamp["built"] = local.get("artifact_built") or time.strftime("%Y-%m-%d")
        stamp["artifact_sha256"] = local.get("artifact_sha256")
    stamp["app"] = app or stamp.get("app") or "base"
    stamp["env"] = env
    stamp["port"] = port
    # Where this board physically is, so a later run can still name it when the
    # board is sitting in BOOTSEL and has no tty to be resolved through.
    if _TARGET_USB_PATH:
        stamp["usb_path"] = _TARGET_USB_PATH
    if env_bin and os.path.isfile(env_bin):
        stamp["env_sha256"] = hashlib.sha256(open(env_bin, "rb").read()).hexdigest()
    elif params:
        try:
            # The USER's secrets, resolved the one way they are ever resolved
            # (AGENTS.md: never build the config path anywhere else). Hashing
            # the repo's config/secrets.yaml -- a file that does not exist in a
            # clean tree -- recorded a digest no probe could ever match.
            digest, _ = mcu_probe.env_digest(
                os.path.abspath(params), cockpit_paths.secrets_path(),
                app or "base")
            stamp["env_sha256"] = digest
        except Exception:
            pass

    # The board re-enumerates after a flash; wait for the node to come back
    # before opening it, then listen briefly for the banner.
    # The board prints its banner within milliseconds of USB coming up, and the
    # device node appears at enumeration -- so the only way to catch it is to
    # poll hard for the node and open immediately. At 0.5 s the line was always
    # already gone; at 20 ms it is usually there. Missing it is not fatal (the
    # stamp still records what was written) but catching it is the difference
    # between "what I believe I wrote" and "what the board says it is running".
    banner = {}
    deadline = time.time() + 15
    while time.time() < deadline and not os.path.exists(port):
        time.sleep(0.02)
    if os.path.exists(port):
        captured = mcu_probe.listen_for_banner(port, baud, 6.0)
        banner = mcu_probe.parse_banner(captured)
    # A missed banner on an ESP32 is not cosmetic. esptool's own reset does not
    # start the application on every board, and a board that never started is
    # indistinguishable, from here, from a board that merely printed too early.
    # So ask it again the one way that always works, rather than recording a
    # guess and handing bringup a board that is not running.
    if not banner and app_written and is_esp_family(env) and os.path.exists(port):
        log("no banner yet — pulsing EN to start the application...")
        if esp32_reset_into_app(port, baud):
            captured = mcu_probe.listen_for_banner(port, baud, 8.0)
            banner = mcu_probe.parse_banner(captured)
    if banner:
        log(f"Board reports: linorobot2_hardware app={banner['app']} "
            f"distro={banner.get('distro') or 'unstated'} "
            f"built={banner['built']} git={banner['git']}"
            + (f" {banner['id_kind']}={banner['board_id']}"
               if banner.get("board_id") else ""))
        # Everything else the board said while booting -- which sensors answered
        # on the bus, which drivers it enabled, why it last reset. This is the
        # operator's only view of it on a USB-CDC board: the port disappears on
        # reboot, so a monitor cannot be attached across it, and these lines are
        # printed in the first milliseconds after it comes back.
        for line in captured.splitlines():
            line = line.strip()
            if line.startswith("[i2c]") or line.startswith("[boot]") \
                    or line.startswith("[sensors]"):
                log(f"  | {line}")
        stamp.update({"git": banner["git"], "built": banner["built"],
                      "app": banner["app"], "banner_confirmed": True})
        # Only if the board stated one. A board running an image older than the
        # distro field says nothing here, and inventing the value this host
        # happens to build would be the one thing the banner exists to prevent.
        if banner.get("distro"):
            stamp["distro"] = banner["distro"]
        # Which board this is, as the board itself says. Recorded under the key
        # it used: `uid` is the silicon's own id, `flashid` only the external
        # flash chip's (an RP2040 has nothing else). Keeping them apart is the
        # point -- a flash swap must never read as the same chip, nor a chip
        # swap as the same board. Absent when the image predates the field.
        if banner.get("board_id"):
            stamp["board_id"] = banner["board_id"]
            stamp["id_kind"] = banner["id_kind"]
    else:
        stamp["banner_confirmed"] = False
        log("(the board did not print a boot banner in time — the stamp records "
            "what was written, not what was heard)")
    mcu_probe.write_stamp(env, port, stamp)
    log(f"Recorded {mcu_probe.stamp_path(env, port)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Intelligent Microcontroller Firmware Flasher & Recovery Engine")
    parser.add_argument("--firmware-dir", "--target-dir", dest="firmware_dir", default="firmware",
                        help="Path to the firmware project (there is one: `firmware`)")
    parser.add_argument("--env", default="pico2", help="PlatformIO environment (e.g. pico2, esp32)")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port of the base controller")
    parser.add_argument("--baud", type=int, default=921600, help="Upload baudrate")
    parser.add_argument("--app", default=None,
                        help="application the unified image should boot: base, test_sensors, "
                             "test_motors, test_acc, i2c_detect, bno085_cal or adc_calibrate. "
                             "Written as the `app` key of the env block, so switching tools "
                             "rewrites 4 KB of flash instead of rebuilding.")
    # No default board name here. It used to be "pico2", so a hand-run flash of any
    # other board announced "Flashing firmware: pico2" and "flashed successfully to
    # pico2" while writing an ESP32 (seen on the bench, 2026-09-18, flashing the
    # release esp32 image). The env IS the board (AGENTS.md: base_controller.name
    # doubles as the PlatformIO env), and --prebuilt sets it from the manifest, so
    # it is the honest fallback; the pipeline and the supervisor still pass the
    # robot's own controller name.
    parser.add_argument("--firmware-name", "--target-name", dest="firmware_name", default=None,
                        help="Name used in telemetry and diagnostics (base controller or "
                             "sub-firmware; default: the PlatformIO env being flashed)")
    parser.add_argument("--skip-mcu-check", action="store_true",
                        help="Write the image even when the USB bus says the board is "
                             "different silicon than this env builds for")
    parser.add_argument("--prebuilt", metavar="PROFILE",
                        help="Flash the ready-made images in firmware/prebuilt/<PROFILE> "
                             "instead of a local build. No toolchain needed: nothing is "
                             "compiled and PlatformIO is never invoked. Sets --env from "
                             "the profile's manifest.json.")
    parser.add_argument("--env-bin", metavar="PATH",
                        help="U-Boot-style env block to write alongside an ESP32 image "
                             "(scripts/mcu_env.py). Holds the Wi-Fi keys and the agent, "
                             "syslog and lidar_udp addresses, which are deliberately not "
                             "in the application image. Defaults to building one from "
                             "<config dir>/secrets.yaml when the profile needs Wi-Fi.")
    parser.add_argument("--params", metavar="CONFIG",
                        help="Robot config the env block should describe (default: the one "
                             "the image was built from). This matters for a prebuilt image: "
                             "the env carries the pin matrix, so flashing the generic esp32 "
                             "image onto a Waveshare General Driver means naming "
                             "gendrv_config.yaml here — otherwise the board is "
                             "described by somebody else's wiring.")
    parser.add_argument("--build-dir", metavar="DIR",
                        help="Where the artifacts are, when they are not in "
                             "<firmware-dir>/.pio/build/<env> (a directory of files "
                             "built elsewhere, for instance).")
    parser.add_argument("--env-only", action="store_true",
                        help="Write ONLY the env block and leave the application alone. "
                             "This is the config-changed / switch-application case "
                             "(AGENTS.md §5): 4 KB instead of a whole image, and the "
                             "firmware on the board is not touched. Note it still needs "
                             "BOOTSEL on an RP2 board -- every write to RP2 flash does -- "
                             "so it is cheap in time and in risk-to-the-image, but not in "
                             "the 1200-baud touch. On ESP32 it costs nothing at all.")
    parser.add_argument("--build", action="store_true",
                        help="Compile with `pio run` before flashing. Off by default: build and "
                             "flash are separate steps (see docs/flashing.md).")
    parser.add_argument("--no-build", action="store_true",
                        help=argparse.SUPPRESS)  # accepted for backwards compatibility; flashing never builds
    parser.add_argument("--timeout", type=int, default=90, help="Timeout for a single upload attempt, in seconds")
    parser.add_argument("--total-timeout", type=int, default=600,
                        help="Budget for the whole flash including every recovery stage, in seconds")
    args = parser.parse_args()
    set_deadline(args.total_timeout)
    # A /dev/serial/by-id/... path is accepted anywhere a tty is: it is the only
    # name that follows a board across reboots and plug order (two Picos on one
    # bench swapped ttyACM numbers overnight). Everything downstream -- lsof,
    # esptool, picotool, the agent -- gets the node it points at.
    _real_port = mcu_identity.resolve_port(args.port)
    if _real_port != args.port:
        log(f"port {args.port} -> {_real_port}")
        args.port = _real_port

    # Pin the flash to one physical board while the tty still exists to say
    # which one it is. Must happen before anything touches the port.
    if is_pico_family(args.env):
        remember_usb_path(args.port, stamp_path=stamped_usb_path(args.env, args.port))


    firmware_dir = os.path.abspath(os.path.join(REPO_ROOT, args.firmware_dir) if not os.path.isabs(args.firmware_dir) else args.firmware_dir)

    # A prebuilt profile replaces the build directory outright. The artifacts sit
    # flat in firmware/prebuilt/<profile>/ rather than under .pio/build/<env>/,
    # so that a user who has never installed PlatformIO can still flash a board.
    prebuilt_dir = None
    if args.prebuilt:
        prebuilt_dir = os.path.join(REPO_ROOT, "firmware", "prebuilt", args.prebuilt)
        manifest_path = os.path.join(prebuilt_dir, "manifest.json")
        if not os.path.isfile(manifest_path):
            log(f"❌ No prebuilt profile '{args.prebuilt}' — {manifest_path} is missing.")
            root = os.path.join(REPO_ROOT, "firmware", "prebuilt")
            available = sorted(d for d in os.listdir(root)
                               if os.path.isdir(os.path.join(root, d))) if os.path.isdir(root) else []
            log("   Available: " + (", ".join(available) if available else "(none built yet)"))
            return 1
        with open(manifest_path) as fh:
            manifest = json.load(fh)
        args.env = manifest["pio_env"]
        args.build = False
        args.firmware_name = args.firmware_name or args.prebuilt
        log(f"Prebuilt profile '{args.prebuilt}': {manifest.get('description', '')}")
        log(f"   built {manifest.get('built')} from commit {manifest.get('commit')}")
        # The shipped checksums are the only evidence the files are the ones that
        # were built; a truncated download would otherwise be flashed happily.
        for entry in manifest["files"]:
            path = os.path.join(prebuilt_dir, entry["name"])
            if not os.path.isfile(path):
                log(f"❌ {entry['name']} missing from {prebuilt_dir}")
                return 1
            digest = hashlib.sha256(open(path, "rb").read()).hexdigest()
            if digest != entry["sha256"]:
                log(f"❌ {entry['name']} does not match its manifest checksum.")
                return 1
        log(f"   ✅ {len(manifest['files'])} artifacts match their checksums")

    # Nothing is written to a board the USB bus says is different silicon.
    #
    # This lives HERE, not only in one_click_pipeline.py, because the pipeline is
    # not the only way to reach a flash: the Cockpit's "Flash MCU" button builds a
    # flash_mcu.py command directly (web/backend/main.py), and the script is a
    # documented CLI in its own right. A check that guards one entry point is a
    # check that the other entry points walk around.
    #
    # Only decisive evidence stops a flash. An RP2040/RP2350 is itself the USB
    # device, so 2e8a:<pid> names the silicon outright -- in the application and
    # in BOOTSEL alike. A classic ESP32 answers through a CP2102/CH340/FTDI,
    # which identifies the BRIDGE and says nothing about the chip behind it, so
    # that case never blocks. See scripts/mcu_identity.py.
    if not args.skip_mcu_check:
        expected = mcu_identity.env_family(args.env)
        family, chip, decisive = mcu_identity.identify_target(args.port)
        if mcu_identity.mismatch(expected, family, decisive):
            exp = mcu_identity.FAMILY_LABEL.get(expected, expected)
            got = mcu_identity.FAMILY_LABEL.get(family, family)
            log("")
            log("=" * 78)
            log(f"❌ [MCU MISMATCH] '{args.env}' builds for {exp}, but the board is {got}.")
            log("=" * 78)
            log(f"   Detected: {chip}")
            log(f"   Asked to flash: env '{args.env}' on {args.port}")
            log("")
            log(f"   Nothing has been written. An image built for {exp} does not belong on")
            log(f"   {got} silicon.")
            log("   Use the env that matches the board, or --skip-mcu-check to override.")
            log("=" * 78)
            return 1
        if family and decisive:
            log(f"Board on the bus: {chip} — matches '{args.env}'.")

    explicit_build_dir = None
    if args.build_dir:
        explicit_build_dir = (args.build_dir if os.path.isabs(args.build_dir)
                              else os.path.join(REPO_ROOT, args.build_dir))
        if not os.path.isdir(explicit_build_dir):
            log(f"❌ --build-dir {explicit_build_dir} does not exist.")
            return 1
    uf2_path = (os.path.join(prebuilt_dir, "firmware.uf2") if prebuilt_dir
                else os.path.join(explicit_build_dir or
                                  os.path.join(firmware_dir, ".pio", "build", args.env),
                                  "firmware.uf2"))

    log("==================================================================")
    args.firmware_name = args.firmware_name or args.env
    log(f"🚀 Flashing firmware: {args.firmware_name} ({args.env})")
    log(f"   Firmware Dir: {firmware_dir}")
    log(f"   Port: {args.port} | Baud: {args.baud}")
    log(f"   Attempt timeout: {args.timeout}s | Total budget: {args.total_timeout}s")
    log("==================================================================")

    # Step 1: Pre-flash Port Release (Directive 6)
    log(f"Ensuring serial port {args.port} is released from micro-ROS agent and active monitors...")
    release_serial_port(args.port)
    ensure_port_permissions(args.port)

    # Step 2a: Optional compile. `pio run` only ever builds -- it is never given
    # a -t upload target, so PlatformIO never opens the serial port.
    if args.build:
        build_cmd = ["pio", "run", "-d", firmware_dir, "-e", args.env]
        log(f"Building: {' '.join(build_cmd)}")
        build_proc = run_tool(build_cmd, timeout=900)
        if build_proc.returncode != 0:
            log(f"❌ Build failed (exit {build_proc.returncode}); nothing was flashed.")
            return 1
        log("✅ Build succeeded.")
    else:
        log("Flash-only mode: flashing an already-built image.")

    # Step 2b: Primary native flash — esptool for ESP32, picotool for RP2040/RP2350.
    build_dir = prebuilt_dir or explicit_build_dir or os.path.join(firmware_dir, ".pio", "build", args.env)

    # --env-only: the board keeps the firmware it has. Nothing here reads the
    # build directory, so this works on a machine that has never compiled
    # anything (a robot running a prebuilt release image).
    if args.env_only:
        env_bin = resolve_env_bin(args, prebuilt_dir)
        if not env_bin:
            log("❌ --env-only needs an env block: pass --params <robot config> or --env-bin.")
            return 1
        log(f"Writing the env block only — the application on the board is left alone.")
        if is_esp_family(args.env):
            ok = flash_env_via_esptool(env_bin, args.env, args.port, args.baud, args.timeout)
        elif is_pico_family(args.env):
            # Every write to RP2 flash goes through BOOTSEL, so this needs the
            # board there too. It is still the right operation: it does not
            # rebuild, does not restage a binary, and cannot leave a half-written
            # application behind.
            ok = flash_env_via_picotool(env_bin, args.env)
            if not ok and not out_of_time("the 1200-baud BOOTSEL pulse") and pulse_1200_baud(args.port):
                ok = flash_env_via_picotool(env_bin, args.env)
            if not ok and rp2_usb_mode() == "app":
                report_failed_bootsel_request(args.port)
            if ok:
                rp2_reboot_into_app(args.env)
        else:
            log(f"❌ Cannot tell the MCU family from environment '{args.env}'.")
            return 1
        if ok:
            log("✅ env block written.")
            record_stamp(args.env, args.port, args.app, env_bin, args.params,
                         args.baud, app_written=False, prebuilt_dir=prebuilt_dir)
            return 0
        log("❌ The env block could not be written.")
        return 1
    if is_esp_family(args.env):
        env_bin = resolve_env_bin(args, prebuilt_dir)
        log("Flashing via esptool...")
        flashed = flash_via_esptool(build_dir, args.env, args.port, args.baud,
                                    timeout=args.timeout, env_bin=env_bin)
    elif is_pico_family(args.env):
        env_bin = resolve_env_bin(args, prebuilt_dir)
        log("Flashing via picotool...")
        flashed = flash_via_picotool(uf2_path, args.env, env_bin=env_bin)
    else:
        log(f"❌ Cannot tell the MCU family from environment '{args.env}'.")
        return 1

    # Every success below goes through this, because every one of them is a
    # write to the board and the next run's decision depends on knowing about it.
    # Recording only in the primary path was enough to lose the stamp on the very
    # first real run: the board was in application mode, so the flash landed in
    # recovery 4 (1200-baud touch, then picotool), which returned 0 directly --
    # and the next probe then reported a board that had "never been flashed by
    # this host" moments after this host flashed it.
    def flashed_ok(message: str = None) -> int:
        if message:
            log(message)
        time.sleep(2.0)
        record_stamp(args.env, args.port, args.app, env_bin, args.params,
                     args.baud, app_written=True, prebuilt_dir=prebuilt_dir)
        return 0

    if flashed:
        return flashed_ok(f"✅ Firmware flashed successfully to {args.firmware_name} on {args.port}.")

    log("⚠️ Primary flash failed (the board is most likely not in BOOTSEL / bootloader mode).")

    # Step 3: Automatic Resolution & Recovery Attempt
    log("Initiating automatic recovery protocol...")

    if is_pico_family(args.env):
        log("Family detected: Raspberry Pi Pico / RP2040 / RP2350.")

        # Recovery 1: Check already mounted UF2 drive
        mounted_vol = find_mounted_uf2_volume(args.env)
        if mounted_vol and flash_via_uf2_copy(uf2_path, mounted_vol):
            return flashed_ok()

        # Recovery 2: Check unmounted block device and temporarily mount
        dev_mount = find_and_mount_pico_device(args.env)
        if dev_mount and flash_via_uf2_copy(uf2_path, dev_mount):
            return flashed_ok()

        # Recovery 3: Send 1200-baud pulse to kick CDC into BOOTSEL mode
        if not out_of_time("the 1200-baud BOOTSEL pulse") and pulse_1200_baud(args.port):
            log("Checking for mounted bootloader volume after 1200-baud pulse...")
            mounted_vol = find_mounted_uf2_volume(args.env) or find_and_mount_pico_device(args.env)
            if mounted_vol and flash_via_uf2_copy(uf2_path, mounted_vol):
                return flashed_ok()

            # Recovery 4: Try picotool load directly with family constraint
            if flash_via_picotool(uf2_path, args.env, env_bin=env_bin):
                return flashed_ok()

            # Recovery 5: Re-attempt the native picotool flash once more
            release_serial_port(args.port)
            ensure_port_permissions(args.port)
            log("Re-attempting picotool flash after 1200-baud pulse...")
            if flash_via_picotool(uf2_path, args.env, env_bin=env_bin):
                return flashed_ok("✅ Firmware flashed successfully on retry!")

            # Everything above needs BOOTSEL and none of it found it. Read the
            # interface classes once and say which of the two very different
            # states this is, instead of leaving the generic "not in BOOTSEL".
            if rp2_usb_mode() == "app":
                report_failed_bootsel_request(args.port)

        # Recovery 6: Direct picotool fallback even if 1200-baud failed
        if flash_via_picotool(uf2_path, args.env, env_bin=env_bin):
            return flashed_ok()

    elif is_esp_family(args.env) and not out_of_time("the ESP32 fallback baudrates"):
        log("Family detected: ESP32 / ESP32-S3.")
        release_serial_port(args.port)
        ensure_port_permissions(args.port)

        # Retry with lower flash baudrate — a marginal USB-UART bridge often
        # enumerates fine but cannot sustain 921600 through a whole write.
        for safe_baud in [460800, 115200]:
            log(f"Attempting ESP32 flash at fallback baudrate {safe_baud}...")
            if flash_via_esptool(build_dir, args.env, args.port, safe_baud, timeout=args.timeout):
                return flashed_ok(f"✅ Firmware flashed successfully at fallback baudrate {safe_baud}!")

    # Step 4: Complete Failure & Halting Directive
    #
    # There is no blink fallback any more. It built and flashed a second, smaller
    # image to prove the board was alive -- but there is one image now, so the
    # only thing left to flash is the one that just failed. What distinguished a
    # dead board from a bad build was never blink itself, it was whether ANY
    # write succeeded, and the interface-class check below says that directly.
    print_failure_troubleshooting(args.firmware_name, args.env, args.port, uf2_path)
    return 1


if __name__ == "__main__":
    sys.exit(main())
