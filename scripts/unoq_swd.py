#!/usr/bin/env python3
"""The Arduino UNO Q's STM32U585, over SWD, with the board's own OpenOCD.

    unoq_swd.py identify
    unoq_swd.py verify  <firmware.elf>              is the flash this image? (exit 0 = yes)
    unoq_swd.py flash   [--elf firmware.elf] --env env.bin

The UNO Q has no USB to its MCU and no boot banner on its micro-ROS link, so picotool,
esptool and the serial probe do not apply. Its SWD lines are GPIOs of the QRB2210, driven
by the OpenOCD Arduino ships in /opt/openocd (adapter `linuxgpiod`, openocd_gpiod.cfg).

That OpenOCD is built for the board's Debian and links its libgpiod.so.3, which the robot
image (Ubuntu) does not have. So inside the cockpit container it runs through the HOST's
dynamic loader and libraries, mounted at /host-lib (docker-compose.unoq.yml); on the board
itself it runs directly. Either way it needs the GPIO chips (char major 254) and root.

flash writes the application only when given --elf, then the env block at 0x081FE000 (the
last 8 KB page, fw/app.overlay `lino_env_partition`), reads the env back and compares it,
and resets the MCU into the application.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

OPENOCD_DIR = os.environ.get("UNOQ_OPENOCD_DIR", "/opt/openocd")
HOST_LIB = os.environ.get("UNOQ_HOST_LIB", "/host-lib")
ENV_ADDR = 0x081FE000
ENV_SIZE = 0x1000
FLASH_BASE = 0x08000000
DBGMCU_IDCODE = 0xE0044000     # DEV_ID in bits 11:0
UID_ADDR = 0x0BFA0700          # 96-bit unique device ID
STM32U5_DEV_IDS = {0x481, 0x482, 0x455, 0x476}   # U59x/5Ax, U575/585, U535/545, U5Fx/5Gx


class SwdError(RuntimeError):
    pass


def openocd_argv():
    """The OpenOCD command line, through the host's loader when it must be."""
    binary = os.path.join(OPENOCD_DIR, "bin", "openocd")
    cfg = os.path.join(OPENOCD_DIR, "openocd_gpiod.cfg")
    if not (os.path.isfile(binary) and os.path.isfile(cfg)):
        raise SwdError(
            f"no UNO Q OpenOCD at {OPENOCD_DIR} (bin/openocd and openocd_gpiod.cfg). On the board "
            f"it ships in /opt/openocd; in the cockpit container it is mounted by "
            f"docker-compose.unoq.yml.")
    loader = os.path.join(HOST_LIB, "ld-linux-aarch64.so.1")
    prefix = [loader, "--library-path", HOST_LIB] if os.path.isfile(loader) else []
    # Through the loader, OpenOCD cannot find its own install prefix: name its scripts.
    return prefix + [binary, "-s", OPENOCD_DIR,
                     "-s", os.path.join(OPENOCD_DIR, "share", "openocd", "scripts"), "-f", cfg]


def run(commands, timeout=120):
    """Run one OpenOCD session; returns its combined output, raises on failure to start."""
    argv = openocd_argv() + ["-c", "; ".join(commands)]
    try:
        res = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, cwd=OPENOCD_DIR)
    except subprocess.TimeoutExpired:
        raise SwdError(f"OpenOCD did not finish in {timeout} s")
    out = (res.stdout or "") + (res.stderr or "")
    if "Examination succeed" not in out and "examination succeed" not in out.lower():
        tail = "\n".join(out.strip().splitlines()[-6:])
        raise SwdError("OpenOCD did not reach the STM32 over SWD (is this an Arduino UNO Q, and "
                       "does the process have the GPIO chips?):\n" + tail)
    return out


def _words(out, addr):
    """The words OpenOCD's `mdw` printed for addr."""
    m = re.search(rf"0x{addr:08x}:\s+((?:[0-9a-f]{{8}}\s*)+)", out, re.I)
    return [int(w, 16) for w in m.group(1).split()] if m else []


def identify():
    """(dev_id, uid hex string). Raises unless it is an STM32U5."""
    # A command's result is not printed in batch mode (-c); only log lines are. echo it.
    out = run(["init", f"echo [capture {{mdw 0x{DBGMCU_IDCODE:08x}}}]",
               f"echo [capture {{mdw 0x{UID_ADDR:08x} 3}}]", "shutdown"])
    idc = _words(out, DBGMCU_IDCODE)
    uid = _words(out, UID_ADDR)
    if not idc:
        raise SwdError("could not read the MCU's DBGMCU_IDCODE")
    dev_id = idc[0] & 0xFFF
    if dev_id not in STM32U5_DEV_IDS:
        raise SwdError(f"the MCU on SWD is DEV_ID 0x{dev_id:03x}, not an STM32U5 -- not a UNO Q")
    return dev_id, "".join(f"{w:08X}" for w in reversed(uid))


def verify(elf):
    """True when the flash holds this image."""
    out = run(["init", "reset halt",
               f"if {{[catch {{verify_image {elf}}} e]}} {{echo \"UNOQ_VERIFY_FAIL $e\"}} else {{echo UNOQ_VERIFY_OK}}",
               "reset run", "shutdown"], timeout=180)
    if "UNOQ_VERIFY_OK" in out:
        return True
    if "UNOQ_VERIFY_FAIL" in out:
        return False
    raise SwdError("verify_image gave no answer:\n" + "\n".join(out.strip().splitlines()[-6:]))


def flash(env_bin, elf=None):
    """Write the application (when given) and the env block; read the env back; reset into it."""
    if os.path.getsize(env_bin) > ENV_SIZE:
        raise SwdError(f"{env_bin} is larger than the {ENV_SIZE}-byte env page")
    with tempfile.TemporaryDirectory() as tmp:
        back = os.path.join(tmp, "envback.bin")
        cmds = ["reset_config srst_only srst_push_pull", "init", "reset halt"]
        if elf:
            cmds.append(f"program {elf} verify")
        cmds += [f"flash write_image erase {env_bin} 0x{ENV_ADDR:08x} bin",
                 f"dump_image {back} 0x{ENV_ADDR:08x} {os.path.getsize(env_bin)}",
                 "reset run", "shutdown"]
        out = run(cmds, timeout=300)
        if elf and "Verified OK" not in out:
            raise SwdError("the application did not verify after programming:\n"
                           + "\n".join(out.strip().splitlines()[-6:]))
        if not os.path.isfile(back) or open(back, "rb").read() != open(env_bin, "rb").read():
            raise SwdError("the env block read back differs from what was written")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("identify")
    v = sub.add_parser("verify")
    v.add_argument("elf")
    f = sub.add_parser("flash")
    f.add_argument("--elf")
    f.add_argument("--env", required=True)
    a = ap.parse_args()
    try:
        if a.cmd == "identify":
            dev, uid = identify()
            print(f"STM32U5 DEV_ID 0x{dev:03x} uid={uid}")
        elif a.cmd == "verify":
            ok = verify(a.elf)
            print("firmware current" if ok else "firmware differs")
            return 0 if ok else 1
        else:
            flash(a.env, a.elf)
            print(("firmware and " if a.elf else "") + "env block written and verified")
    except SwdError as e:
        print(f"unoq_swd: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
