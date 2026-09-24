#!/usr/bin/env python3
"""What is on the board already — asked before anything is written to it.

Every other step in this project got cheaper as facts moved out of the image and
into the env partition, and one question was left with no answer at all: *is the
firmware on this board the firmware I have here?* With no answer, the only safe
assumption is "no", so every run reflashed — and a flash is the one step that can
brick a robot that is already assembled, needs the agent stopped, needs the bus
free, and on an RP2350 can hang the board outright in the 1200-baud touch
(AGENTS.md §6). Reflashing by default made all of that the normal case.

Three sources, deliberately different in kind, because none of them is complete:

  usb mode   For an RP2 board the interface classes separate "an application is
             running" (Communications/CDC) from "BOOTSEL, nothing is running"
             (Mass Storage) -- AGENTS.md §6. This is the only source that needs
             no cooperation from the firmware, and it is what answers "is
             firmware installed" for a board that has never been flashed.

  banner     `[fw] linorobot2_hardware app=base built=2026-09-16 git=6aa607f`,
             printed at boot by firmware/src/main.cpp. This is the authoritative
             answer and the only one the BOARD itself gives -- but it is printed
             at boot and nothing on the host can reboot a running RP2 safely, so
             it is caught only when the board happens to reboot (which it does
             right after a flash, which is when flash_mcu.py records it).

  stamp      <config dir>/state/flashed/<env>_<port>.json, written by flash_mcu.py
             from the banner the board printed after the last flash, plus the
             CRC of the env block that went with it. This is the host's memory
             of the last write, and it is what makes "has the config changed
             since the board was flashed?" answerable without touching the
             board at all.

             It hangs off the CONFIG directory, not $HOME. The one-click
             pipeline runs as container-root and the cockpit's web backend as
             the container user, so ~/.cache resolved to two different places
             and neither side saw the other: a board flashed seconds earlier
             still probed as "this host has no record of flashing it".
             LINO_STAMP_DIR overrides.

The verdict combines them:

  no_firmware   BOOTSEL / nothing enumerated -- the board has no application.
  up_to_date    the board is running this tree's build and the env matches.
  env_stale     right image, but the config implies a different env block: a
                4 KB write, no reflash (this is the `switch tool` / `edit config`
                case, and it is the only write this project makes without asking).
  stale         the board is running a different build (revision or ROS 2 distro).
  unknown       an application is running and did not identify itself -- treated
                exactly like `stale`, because "cannot tell" must never resolve to
                "assume it is the one I just compiled".

This module only reports; what a `stale` verdict COSTS is the caller's policy.
one_click_pipeline.py updates a stale board by default (`--auto-update`, on) and
leaves it strictly alone under `--no-auto-update`, which is the setting for a
robot in the field -- the flash is still the one step that can brick an assembled
robot, and that has not changed, only who is expected to be holding it.
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cockpit_paths  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mcu_env  # noqa: E402  (same directory, and the env encoder must be the same one)
import mcu_identity  # noqa: E402  (the by-id resolver and the family table)

# Outside the workspace on purpose. Every box in this rig gets a FRESH workspace
# per run (`git archive HEAD`, the old tree deleted first -- AGENTS.md §12), so a
# stamp kept under firmware/ would be destroyed by exactly the re-seed that
# precedes the run that needs to read it, and the rig would reflash every time
# for a reason that has nothing to do with the board. It belongs to the MACHINE
# that did the flashing, which is what ~/.cache is for.
def _default_stamp_dir() -> str:
    """Where flash stamps live, for BOTH the pipeline and the cockpit backend.

    Keyed on the config directory, not on $HOME. The pipeline runs as
    container-root and the backend as the container user, so ~/.cache resolved
    to /root/.cache for one and /home/ubuntu/.cache for the other: a board
    flashed by one path still probed as "this host has no record of flashing
    it" from the other, and the verdict fell back to `unknown` for the whole
    2026-09-19 release matrix.
    """
    try:
        return os.path.join(cockpit_paths.state_dir(), "flashed")
    except Exception:
        # No config directory to hang it off (a bare checkout, a unit test):
        # the old per-user path is still better than failing to import.
        return os.path.expanduser("~/.cache/linorobot2/flashed")


STAMP_DIR = os.environ.get("LINO_STAMP_DIR") or _default_stamp_dir()

# The one place this line is parsed. firmware/src/main.cpp:printBanner() is the
# one place it is written; keep the two together.
# distro= is optional in the pattern on purpose: a board flashed before that
# field existed must still parse, and report no distro rather than a guessed one.
# The identity field is optional, and WHICH key appears is itself information
# (firmware/src/main.cpp:identityField): `uid=` is the silicon's own id (RP2350
# chip info, ESP32 eFuse MAC), `flashid=` is the external flash chip's, which is
# all an RP2040 has. Both name one board on a bench; only `uid` names the part.
# Neither key means an older image, so absence is never "the probe failed".
BANNER_RE = re.compile(
    r"\[fw\]\s+linorobot2_hardware\s+app=(?P<app>\S+)"
    r"(?:\s+distro=(?P<distro>\S+))?"
    r"\s+built=(?P<built>\S+)\s+git=(?P<git>\S+)"
    r"(?:\s+(?P<id_kind>uid|flashid)=(?P<board_id>[0-9A-Fa-f]+))?")


def distro_for_env(env: str) -> str:
    """Which ROS 2 distro a PlatformIO env builds against.

    §10: a micro-ROS image is distro-specific and the release matrix keeps the
    jazzy envs on the bare profile name with the lyrical ones suffixed. The env
    name therefore already carries the answer, and reading it here means the
    probe needs no extra plumbing from its callers to ask the question.
    """
    return "lyrical" if env.endswith("_lyrical") else "jazzy"


def stamp_path(env: str, port: str) -> str:
    return os.path.join(STAMP_DIR, f"{env}_{os.path.basename(port)}.json")


def read_stamp(env: str, port: str) -> dict:
    try:
        with open(stamp_path(env, port)) as fh:
            return json.load(fh)
    except Exception:
        return {}


def write_stamp(env: str, port: str, data: dict):
    os.makedirs(STAMP_DIR, exist_ok=True)
    data = dict(data)
    data["flashed_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    with open(stamp_path(env, port), "w") as fh:
        json.dump(data, fh, indent=2, sort_keys=True)
        fh.write("\n")


def parse_banner(text: str) -> dict:
    """The LAST banner in a capture, so a board that rebooted twice reports the
    boot that is actually running rather than the one before it."""
    matches = list(BANNER_RE.finditer(text or ""))
    return matches[-1].groupdict() if matches else {}


# How long a board gets to come back onto the USB bus before the probe
# calls it absent. Two seconds covers an RP2 re-enumerating after the
# previous run let go of the port; eight leaves room for a slow hub.
PROBE_ENUMERATE_WAIT = float(os.environ.get("LINO_PROBE_ENUMERATE_WAIT", "8"))


def is_pico_family(env: str) -> bool:
    return "pico" in (env or "").lower() or "rp2" in (env or "").lower()


def usb_mode(env: str, port: str) -> str:
    """app / bootsel / absent / unknown.

    RP2 is read from the interface classes -- the PID is 2e8a:000f in both modes
    and proves nothing. For everything else the presence of the tty is the only
    passive signal there is, and an ESP32's USB-UART bridge enumerates whether or
    not the ESP32 behind it is running, so `app` there means "a port exists",
    which is why the banner and the stamp carry the real weight.
    """
    # A board that is mid-re-enumeration is not an absent board. The probe runs
    # moments after the previous ROS stack was torn down and the port released,
    # and on that boundary an RP2 can be off the bus for a second or two. The
    # first version answered "absent" there, and "absent" is the one verdict
    # that makes the pipeline write NOTHING -- not the firmware, not the env --
    # so the run went on to test whatever image and whatever env block the board
    # happened to be carrying.
    #
    # That is how a pico2 bench came to be tested with an env that never had
    # sim_wheel=1 written to it: no pose reset at the agent session, the
    # emulator still parked at the room wall from an earlier run (odom
    # x=4.800, /scan min 0.20 m), Nav2 boxed in and 41 consecutive zero
    # velocity commands. Every symptom pointed at Nav2 or the firmware; the
    # cause was a probe that looked one second too early.
    deadline = time.time() + PROBE_ENUMERATE_WAIT
    while True:
        if is_pico_family(env):
            try:
                sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
                import flash_mcu
                mode = flash_mcu.rp2_usb_mode()
            except Exception:
                mode = "app" if port and os.path.exists(port) else "absent"
        else:
            mode = "app" if port and os.path.exists(port) else "absent"
        if mode != "absent" or time.time() >= deadline:
            return mode
        time.sleep(0.5)


def listen_for_banner(port: str, baud: int, timeout: float, reset: bool = False) -> str:
    """Capture serial output, optionally hard-resetting an ESP32 first.

    The reset is DTR/RTS in the esptool pattern and is offered ONLY for the ESP
    family. There is no equivalent for an RP2: the only software route into a
    reboot is the 1200-baud touch, which is a request to enter BOOTSEL -- it
    stops the application, can hang the board in the core's `while (1)`
    (AGENTS.md §6), and is emphatically not a way to read a version string.
    """
    try:
        import serial
    except ImportError:
        return ""
    try:
        with serial.Serial(port, baud, timeout=0.2) as ser:
            if reset:
                # EN low, IO0 high, release: the classic auto-reset circuit.
                ser.setDTR(False)
                ser.setRTS(True)
                time.sleep(0.12)
                ser.setRTS(False)
                time.sleep(0.05)
            deadline = time.time() + timeout
            chunks = []
            while time.time() < deadline:
                data = ser.read(4096)
                if data:
                    chunks.append(data.decode("utf-8", "replace"))
                    if BANNER_RE.search("".join(chunks)):
                        break
            return "".join(chunks)
    except Exception as exc:
        return f"[mcu_probe] serial: {exc}"


def is_esp_family(env: str) -> bool:
    return str(env).lower().startswith("esp32")


def esp32_boot_app(port: str, baud: int = 115200) -> bool:
    """Leave an ESP32 running its application after we have touched its port.

    Opening a tty asserts DTR and RTS, and on the two-transistor auto-reset
    circuit those are EN and IO0. Which state the board lands in therefore
    depends on the order the kernel and the driver happen to move the two lines
    -- and on the Waveshare General Driver's CP2102N it lands, often, in the ROM
    download mode: the board stops dead, says nothing on serial, never joins
    Wi-Fi, and answers no ping. Measured on that bench: 12 s of reading
    /dev/ttyUSB0 returned ZERO bytes after a probe, and the run that followed
    reported

        ❌ /odom       : NO DATA (0 msgs received in 10.0s)
        ❌ /imu/data   : NO DATA (0 msgs received in 10.0s)
        ❌ /scan       : NO DATA (0 msgs received in 10.0s)

    -- a board that was neither broken nor misflashed, merely not started. It is
    worse on a Wi-Fi robot than on a serial one, because there is no second
    symptom to notice: the whole link is the radio, and the radio never comes up.

    So every serial interaction with an ESP32 ends here. DTR low holds IO0 high
    (run the application, do not enter the bootloader); RTS high pulls EN low;
    releasing it boots. The board then prints its banner and associates in about
    9 s. This is the ESP32 counterpart of flash_mcu's rp2_reboot_into_app().
    """
    try:
        import serial
    except ImportError:
        return False
    try:
        with serial.Serial(port, baud, timeout=0.2) as ser:
            ser.setDTR(False)     # IO0 high: boot the application
            ser.setRTS(True)      # EN low: assert reset
            time.sleep(0.12)
            ser.setRTS(False)     # EN high: run
        return True
    except Exception:
        return False


def local_build(env: str, prebuilt_dir: str = None) -> dict:
    """What this machine would flash: its git revision, and the artifact.

    Two sources. A tree build reports the tree's revision, computed the same way
    build_stamp.py computes the one it compiles in, or the comparison would be
    between two different questions. A prebuilt release image (`prebuilt_dir`)
    reports the revision its manifest was built from -- that is the image that
    would be written, so a checkout a few commits ahead of the release does not
    make every board look stale.
    """
    if prebuilt_dir:
        try:
            with open(os.path.join(prebuilt_dir, "manifest.json")) as fh:
                manifest = json.load(fh)
        except Exception:
            manifest = {}
        info = {"git": (manifest.get("commit") or "unknown")[:8],
                "distro": manifest.get("ros_distro") or distro_for_env(env),
                "source": "prebuilt", "prebuilt_dir": prebuilt_dir}
        for entry in manifest.get("files", []):
            if entry.get("name") in ("firmware.uf2", "firmware.bin"):
                info["artifact"] = os.path.join(prebuilt_dir, entry["name"])
                info["artifact_sha256"] = entry.get("sha256")
                info["artifact_built"] = (manifest.get("built") or "")[:10]
        return info
    def git(*args):
        try:
            out = subprocess.run(["git", "-C", REPO_ROOT, *args],
                                 capture_output=True, text=True, timeout=10)
            return out.stdout.strip() if out.returncode == 0 else ""
        except Exception:
            return ""

    rev = git("rev-parse", "--short=7", "HEAD")
    if rev and git("status", "--porcelain"):
        rev += "+"
    if not rev:
        stamp = os.path.join(REPO_ROOT, "firmware", ".git_rev")
        if os.path.isfile(stamp):
            words = open(stamp).read().split()
            rev = words[0][:8] if words else "unknown"
    info = {"git": rev or "unknown", "distro": distro_for_env(env), "source": "build"}

    build_dir = os.path.join(REPO_ROOT, "firmware", ".pio", "build", env)
    for name in ("firmware.uf2", "firmware.bin"):
        path = os.path.join(build_dir, name)
        if os.path.isfile(path):
            info["artifact"] = path
            info["artifact_sha256"] = hashlib.sha256(open(path, "rb").read()).hexdigest()
            info["artifact_built"] = time.strftime("%Y-%m-%d",
                                                   time.localtime(os.path.getmtime(path)))
            break
    return info


def env_digest(params_path: str, secrets_path: str, app: str = None,
               host_ip: str = None) -> tuple:
    """(sha256, dict) of the env block this config implies.

    The digest is taken over the ENCODED block, not over the dict: that is the
    4 KB that would actually be written, so two configs that differ only in key
    order -- or in a value the encoder normalises -- correctly compare equal and
    do not provoke a pointless write.
    """
    env = mcu_env.env_from_config(params_path, secrets_path, host_ip)
    if app:
        env["app"] = app
    blob = mcu_env.encode(env)
    return hashlib.sha256(blob).hexdigest(), env


def probe(env_name: str, port: str, baud: int, params: str = None, secrets: str = None,
          app: str = None, listen: float = 0.0, reset: bool = False,
          prebuilt_dir: str = None) -> dict:
    result = {
        "env": env_name,
        "port": port,
        "usb_mode": usb_mode(env_name, port),
        "stamp": read_stamp(env_name, port),
        "local": local_build(env_name, prebuilt_dir),
        "banner": {},
    }

    if listen > 0 and os.path.exists(port):
        captured = listen_for_banner(port, baud, listen, reset=reset)
        result["banner"] = parse_banner(captured)

    # Hand an ESP32 back running. Reading a board must not stop it, and on this
    # wiring merely opening the port can -- see esp32_boot_app(). The cost is one
    # reboot of a board we have just interrupted anyway; the cost of skipping it
    # is a robot that is silent for the rest of the run.
    if is_esp_family(env_name) and os.path.exists(port):
        esp32_boot_app(port, baud)

    if params:
        secrets = secrets or cockpit_paths.secrets_path()
        digest, env = env_digest(params, secrets, app)
        result["env_sha256"] = digest
        result["env_app"] = env.get("app", app or "base")

    # --- the verdict, cheapest evidence first
    running = result["banner"] or {}
    stamp = result["stamp"] or {}
    installed_git = running.get("git") or stamp.get("git")
    installed_app = running.get("app") or stamp.get("app")
    installed_distro = running.get("distro") or stamp.get("distro")

    if result["usb_mode"] == "bootsel":
        # Mass Storage means no application is running. It can also mean the user
        # is holding BOOTSEL, which is the same thing for our purposes.
        verdict = "no_firmware"
    elif result["usb_mode"] == "absent":
        verdict = "absent"
    elif not installed_git:
        # Something enumerated as CDC, so an application IS running -- we just
        # cannot say which. Never guess it is the right one.
        verdict = "unknown"
    elif installed_git != result["local"]["git"]:
        verdict = "stale"
    elif installed_distro and installed_distro != result["local"]["distro"]:
        # Same revision, different half of the release matrix. The env partition
        # cannot carry the distro (§10) and nothing else distinguishes the two,
        # so without this a lyrical run against a jazzy board reads as
        # up_to_date -- and the failure that follows is an agent and a board
        # that simply never discover each other.
        verdict = "stale"
    elif result.get("env_sha256") and stamp.get("env_sha256") \
            and stamp["env_sha256"] != result["env_sha256"]:
        verdict = "env_stale"
    elif result.get("env_app") and installed_app and result["env_app"] != installed_app:
        # Switching application is an env write, not a build (AGENTS.md §5).
        verdict = "env_stale"
    elif not stamp.get("env_sha256"):
        # Right image, but this host has never recorded what env went on it, so
        # "the config has not changed" is not something it can claim.
        verdict = "env_unknown"
    else:
        verdict = "up_to_date"

    result["installed"] = {"git": installed_git, "app": installed_app,
                           "distro": installed_distro}
    result["verdict"] = verdict
    result["needs_env_write"] = verdict in ("env_stale", "env_unknown")
    # Reported, not acted on here -- see the policy note in the module docstring.
    result["firmware_differs"] = verdict in ("stale", "unknown", "no_firmware")
    return result


def human(result: dict) -> str:
    lines = [f"port {result['port']}  ({result['env']})",
             f"  usb            {result['usb_mode']}"]
    inst = result.get("installed") or {}
    if inst.get("git"):
        src = "board" if result.get("banner") else "last flash on this host"
        lines.append(f"  installed      app={inst.get('app')} "
                     f"distro={inst.get('distro') or 'unstated'} "
                     f"git={inst.get('git')}   [{src}]")
        # Only a board that answered carries one; a stamp from a previous flash
        # never does. The label follows the key, because "chip" and "flash" are
        # different claims about what is being identified.
        if inst.get("board_id"):
            label = "chip uid" if inst.get("id_kind") == "uid" else "flash id"
            lines.append(f"  {label:<14} {inst['board_id']}")
    else:
        lines.append("  installed      unknown — the board has not said, and this "
                     "host has no record of flashing it")
    lines.append(f"  this tree      git={result['local']['git']} "
                 f"distro={result['local']['distro']}")
    if result.get("env_sha256"):
        lines.append(f"  env block      {result['env_sha256'][:12]}  "
                     f"(recorded: {(result['stamp'] or {}).get('env_sha256', 'none')[:12]})")
    lines.append(f"  verdict        {result['verdict']}")
    explain = {
        "no_firmware": "no application is running (BOOTSEL / mass storage). "
                       "Flash one with: flash_mcu.py --env %s --port %s"
                       % (result["env"], result["port"]),
        "absent": "nothing is on that port.",
        "unknown": "an application is running but did not identify itself. "
                   "Either it predates the boot banner or it was flashed elsewhere; "
                   "pass --listen and reboot the board to catch its banner.",
        "stale": "the board is running a different build (revision or ROS 2 "
                 "distro). The pipeline updates it when auto-update is on "
                 "(the default) and leaves it alone under --no-auto-update.",
        "env_stale": "the image is current; the config or the selected application "
                     "is not. That is a 4 KB env write, no reflash.",
        "env_unknown": "the image is current; this host has no record of the env on "
                       "the board, so the env will be written to be sure.",
        "up_to_date": "the board is running this tree's build with this config.",
    }
    lines.append(f"                 {explain.get(result['verdict'], '')}")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--env", default="pico2", help="PlatformIO env name (pico2, esp32, ...)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--params", default=None,
                    help="robot config, to compare the env block the board should carry")
    ap.add_argument("--secrets", default=None)
    # "base" and "the caller said nothing" have to encode to the SAME env block,
    # or a bare probe compares a blob with no app key against a stamp written
    # with one and reports env_stale on a board that is perfectly current
    # (seen on the bench, 2026-09-18: `mcu_probe.py --params <cfg>` said
    # env_stale immediately after a green pipeline run). The pipeline always
    # passes --app; a human reading the board rarely does.
    ap.add_argument("--app", default="base",
                    help="application the env should select (default: base)")
    ap.add_argument("--listen", type=float, default=0.0,
                    help="seconds to listen for the boot banner (the board must reboot "
                         "in that window; use --reset on an ESP32 to make it)")
    ap.add_argument("--reset", action="store_true",
                    help="ESP32 only: pulse DTR/RTS to reset the board before listening. "
                         "There is no safe equivalent on RP2 — the 1200-baud touch is a "
                         "BOOTSEL request, not a reboot.")
    ap.add_argument("--prebuilt-dir", default=None,
                    help="compare against this prebuilt release image (its manifest's "
                         "commit) instead of the tree's own revision")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    # A /dev/serial/by-id/... path is accepted anywhere a tty is: it is the only
    # name that follows a board across reboots and plug order (two Picos on one
    # bench swapped ttyACM numbers overnight). Everything downstream -- lsof,
    # esptool, picotool, the agent -- gets the node it points at.
    _real_port = mcu_identity.resolve_port(args.port)
    if _real_port != args.port:
        print(f"[mcu_probe] port {args.port} -> {_real_port}", file=sys.stderr)
        args.port = _real_port


    if args.reset and is_pico_family(args.env):
        print("[mcu_probe] --reset is ESP32 only; ignoring it for an RP2 board.",
              file=sys.stderr)
        args.reset = False

    result = probe(args.env, args.port, args.baud, params=args.params,
                   secrets=args.secrets, app=args.app, listen=args.listen,
                   reset=args.reset, prebuilt_dir=args.prebuilt_dir)
    print(json.dumps(result, indent=2, sort_keys=True) if args.json else human(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
