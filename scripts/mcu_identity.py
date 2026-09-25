#!/usr/bin/env python3
"""What the USB bus says a board is, and what a PlatformIO env expects.

One table, two callers: the supervisor labels ports with it
(`web/backend/main.py`) and the pipeline refuses to flash the wrong silicon
with it (`scripts/one_click_pipeline.py`). Two copies of a vid/pid table drift,
and the drift is only discovered by writing an image to the wrong chip.

The distinction that matters here is between a MCU that speaks USB itself and
one sitting behind a serial bridge:

  * An RP2040/RP2350 IS the USB device, so 2e8a:<pid> names the silicon exactly
    -- in the application (CDC) and in BOOTSEL alike. Evidence: DECISIVE.
  * An ESP32-S3 with native USB is likewise 303a:<pid>. DECISIVE.
  * A classic ESP32 reaches the computer through a CP2102, CH340 or FTDI. That
    vid/pid identifies the BRIDGE, and the same bridge is soldered next to
    every kind of chip. Evidence: a HINT, never grounds to refuse a flash.

`decisive` is what keeps the guard honest: it blocks a genuine mismatch and
stays quiet whenever the bus cannot actually tell.
"""
import os

# PlatformIO env / base_controller name -> the silicon it builds for.
_ENV_FAMILY = {
    "pico": "pico", "picow": "pico",
    "pico2": "pico2", "pico2w": "pico2",
    "esp32": "esp32", "gendrv": "esp32",
    "esp32s3": "esp32s3", "yb_eet01": "esp32s3",   # Yahboom YB-EET01: an ESP32-S3 board
}

# Families that cannot be told apart from the bus alone, so a mismatch between
# them is not evidence of anything. A CP2102 fronts an ESP32 and an ESP32-S3
# equally well.
_BRIDGE_AMBIGUOUS = {"esp32", "esp32s3", "esp32s2", "gendrv"}

FAMILY_LABEL = {
    "pico": "RP2040", "pico2": "RP2350",
    "esp32": "ESP32", "esp32s3": "ESP32-S3", "esp32s2": "ESP32-S2",
    "gendrv": "ESP32",
}


def classify_usb(vid: str, pid: str, product: str = "") -> tuple:
    """(family, chip label, decisive) for a vid/pid/product.

    Branch order mirrors what the supervisor has always shown, so the label in
    the UI and the family the guard reasons about can never disagree.
    Returns (None, "", False) for a vendor we do not recognise.
    """
    vid = (vid or "").strip().lower()
    pid = (pid or "").strip().lower()
    prod = (product or "").lower()

    if vid == "2e8a":
        # The RP2 is the USB device itself, in the application and in BOOTSEL
        # alike, so the pid names the silicon outright.
        if pid in ("000f", "0005", "f00f") or "pico 2" in prod or "rp2350" in prod:
            return ("pico2", "Raspberry Pi Pico 2 (RP2350)", True)
        if pid in ("0003", "000a") or "pico" in prod or "rp2040" in prod:
            return ("pico", "Raspberry Pi Pico (RP2040)", True)
        # An RP2 we do not have a pid for: still an RP2 board, but which one is
        # a guess, so it must not be grounds to refuse a flash.
        return ("pico2", "Raspberry Pi RP2", False)

    if vid == "303a":
        if pid in ("1001", "1002") or "esp32-s3" in prod:
            return ("esp32s3", "ESP32-S3 (Native USB CDC)", True)
        if pid == "0002":
            return ("esp32s2", "ESP32-S2 (Native USB CDC)", True)
        return ("esp32", "Espressif USB Serial", False)

    # Everything below is a serial BRIDGE. It identifies the bridge, and the
    # same bridge sits next to every kind of chip -- never decisive.
    if vid == "10c4" and pid == "ea60":
        # A GenDrv and a bare ESP32 DevKit are indistinguishable over USB: both
        # are an ESP32 behind a Silicon Labs bridge, same vid:pid. The ONE thing
        # that differs is the bridge variant, and it is a usable signal because
        # no other ESP32 module on the market ships the CP2102**N** -- the
        # Waveshare General Driver does. So N is taken to mean GenDrv.
        #
        # It is a population argument, not a measurement, which is exactly why
        # this returns False for `decisive`: it picks a better default in the UI
        # and never refuses a flash, so a user holding the other board simply
        # corrects the selection and nothing has been lost.
        if "cp2102n" in prod or "general driver" in prod:
            return ("gendrv", "CP2102N USB Bridge (ESP32/GenDrv)", False)
        return ("esp32", "CP2102 USB Bridge (ESP32)", False)
    if vid == "1a86":
        return ("esp32", "CH340/CH341 USB Bridge (Arduino/ESP32)", False)
    if vid == "0403":
        return ("esp32", "FTDI USB Serial Bridge", False)

    return (None, "", False)


def env_family(pio_env: str) -> str:
    """The silicon a PlatformIO env targets ('' when we do not know)."""
    name = (pio_env or "").strip().lower()
    # `<env>_lyrical` is the same board against another micro-ROS build.
    if name.endswith("_lyrical"):
        name = name[: -len("_lyrical")]
    return _ENV_FAMILY.get(name, "")


def read_usb_ids(sys_device_path: str) -> tuple:
    """Walk up a sysfs path to the USB device node and read (vid, pid, product)."""
    curr = os.path.realpath(sys_device_path)
    for _ in range(6):
        vid_path = os.path.join(curr, "idVendor")
        pid_path = os.path.join(curr, "idProduct")
        if os.path.exists(vid_path) and os.path.exists(pid_path):
            def _read(p):
                try:
                    with open(p) as f:
                        return f.read().strip()
                except Exception:
                    return ""
            return (_read(vid_path).lower(), _read(pid_path).lower(),
                    _read(os.path.join(curr, "product")))
        curr = os.path.dirname(curr)
    return ("", "", "")


def sysfs_device_link(port_path: str) -> str:
    """The sysfs `device` link for a tty, found by its DEVICE NUMBER not its name.

    `/sys/class/tty/<basename>` is wrong wherever the node has been renamed, and
    in the Incus rig it is renamed on purpose: the robot box is given the bench's
    /dev/ttyACM1 as `/dev/ttyACM0` (AGENTS.md ss6 -- one MCU reaches the robot box,
    always at the same path), while /sys inside the container still shows the
    HOST's ttyACM0. Reading sysfs by name there described the wrong board
    entirely -- the Cockpit reported "Raspberry Pi Pico (RP2040)" for a box whose
    only device was a Pico 2, which is the ss6 "identify by what the device IS,
    never by the number it happens to carry" rule in a new place.

    st_rdev is the identity the kernel itself uses, and /sys/dev/char/<maj>:<min>
    is indexed by it, so it survives any renaming. The name-based path stays as
    the fallback for a node that exists but has no /sys/dev/char entry.

    A port that is NOT THERE must not reach that fallback. /sys is the host's
    inside a container, so `/sys/class/tty/ttyACM0` resolves whether or not this
    box has a ttyACM0 -- and on a bench with more than one board it resolves to
    somebody else's. A board in BOOTSEL has no tty at all, which is exactly when
    an image is about to be written: a pico box whose own RP2040 sat in BOOTSEL
    asked about its absent /dev/ttyACM0, was told "Raspberry Pi Pico 2 (RP2350)"
    decisively -- the OTHER box's board, on the other side of the machine -- and
    the run was refused with an MCU mismatch against a board that was correct and
    ready. Absent is not evidence about anything, so say nothing.
    """
    if not port_path or not os.path.exists(port_path):
        return ""
    try:
        rdev = os.stat(port_path).st_rdev
        by_number = f"/sys/dev/char/{os.major(rdev)}:{os.minor(rdev)}/device"
        if os.path.exists(by_number):
            return by_number
    except OSError:
        pass
    return f"/sys/class/tty/{os.path.basename(port_path)}/device"


BY_ID_DIR = "/dev/serial/by-id"


def by_id_map() -> dict:
    """Every udev by-id name on this machine, keyed by the tty it points at.

    `/dev/ttyACM0` is not a board's name, it is the order the kernel happened to
    enumerate in. Two Raspberry Pi Picos on one bench swapped numbers between
    2026-09-18 and 2026-09-19 with nothing touched but a reboot, and a flash
    aimed at the config's `/dev/ttyACM0` went at the other board (the pre-flash
    guard caught it: "the board is RP2350"). udev's by-id name carries the USB
    serial -- `usb-Raspberry_Pi_Pico_D665C007DA2A1336-if00` -- so it follows the
    board across reboots, hubs and plug order.
    """
    out = {}
    try:
        names = os.listdir(BY_ID_DIR)
    except OSError:
        return out
    for name in sorted(names):
        link = os.path.join(BY_ID_DIR, name)
        # Symlinks only: a by-id entry IS a link to a tty, and anything else in
        # the directory is not a stable name for anything.
        if not os.path.islink(link):
            continue
        try:
            target = os.path.realpath(link)
        except OSError:
            continue
        # First name wins: udev can make several for one port (-if00, -if00-port0).
        out.setdefault(target, link)
    return out


def by_id_for_port(port_path: str) -> str:
    """The stable by-id path for a tty, or "" when udev made none."""
    if not port_path:
        return ""
    try:
        return by_id_map().get(os.path.realpath(port_path), "")
    except OSError:
        return ""


def resolve_port(port_path: str) -> str:
    """The real tty behind a port, so a by-id path can be used anywhere a tty is.

    Everything downstream (lsof, esptool, picotool, the agent) is happier with
    the node itself, and a by-id path that no longer resolves must stay as it is
    so the caller reports the name the user actually configured.
    """
    if not port_path:
        return port_path
    try:
        real = os.path.realpath(port_path)
    except OSError:
        return port_path
    return real if os.path.exists(real) else port_path


def identify_port(port_path: str) -> tuple:
    """(family, chip, decisive) for a tty, by walking sysfs to its USB parent.

    Resolved by device number, not by name: a guard that reads the wrong board's
    sysfs is worse than no guard, and this rig renames tty nodes on purpose.
    """
    if not port_path:
        return (None, "", False)
    link = sysfs_device_link(port_path)
    if not os.path.exists(link):
        return (None, "", False)
    vid, pid, product = read_usb_ids(link)
    if not vid:
        return (None, "", False)
    return classify_usb(vid, pid, product)


def _reachable(sys_dev: str) -> bool:
    """Is this USB device ours to open -- is its node in THIS /dev?

    sysfs is not namespaced: in a container it lists the HOST's whole bus. An
    container test box with no USB passed through saw the Picos handed to the other
    cells on its host, called a board present, and so never fell back to the
    simulated MCU; a flash would have gone at a device it cannot reach. A board
    is present when its /dev/bus/usb node is, which is also what picotool needs.
    """
    try:
        bus = int(open(os.path.join(sys_dev, "busnum")).read().strip())
        num = int(open(os.path.join(sys_dev, "devnum")).read().strip())
    except (OSError, ValueError):
        return True     # no numbers to check against: do not hide it
    return os.path.exists(f"/dev/bus/usb/{bus:03d}/{num:03d}")


def identify_bus() -> list:
    """Every RP2 / Espressif device on the USB bus, tty or not.

    A board in BOOTSEL has no tty at all, and BOOTSEL is exactly when an image
    is about to be written -- so the port-based lookup goes blind in the one
    moment the answer matters most. This reads the bus directly.
    """
    found = []
    root = "/sys/bus/usb/devices"
    try:
        entries = sorted(os.listdir(root))
    except OSError:
        return found
    for entry in entries:
        dev = os.path.join(root, entry)
        vid_path = os.path.join(dev, "idVendor")
        if not os.path.exists(vid_path):
            continue
        vid, pid, product = read_usb_ids(dev)
        if vid not in ("2e8a", "303a"):
            continue
        if not _reachable(dev):
            continue
        family, chip, decisive = classify_usb(vid, pid, product)
        if family:
            found.append({"vid": vid, "pid": pid, "product": product,
                          "family": family, "chip": chip, "decisive": decisive})
    return found


def identify_target(port_path: str) -> tuple:
    """(family, chip, decisive) for the board a flash is about to be written to.

    The tty is the trustworthy answer: it is the device the flasher will open,
    and sysfs resolves it by device number. Only when there is no tty -- a board
    in BOOTSEL, which is exactly when an image gets written -- does this fall
    back to the bus.

    That fallback must be able to say "I cannot tell". A bench can easily have
    more than one RP2 attached (this rig passes USB through by vendor id, so a
    Pico and a Pico 2 both appear), and picking the first would be a coin flip
    deciding whether a flash is allowed. Several boards that disagree is not
    evidence, so it reports not-decisive and the guard stays out of the way.
    """
    family, chip, decisive = identify_port(port_path)
    if family:
        return (family, chip, decisive)

    devices = identify_bus()
    if not devices:
        return (None, "", False)
    families = {d["family"] for d in devices}
    if len(families) == 1 and all(d["decisive"] for d in devices):
        d = devices[0]
        return (d["family"], d["chip"], True)
    names = ", ".join(sorted(d["chip"] for d in devices))
    return (None, f"{len(devices)} boards on the bus ({names})", False)


def mismatch(expected_family: str, detected_family: str, decisive: bool) -> bool:
    """True only when the bus positively contradicts the configured board."""
    if not expected_family or not detected_family or not decisive:
        return False
    if expected_family == detected_family:
        return False
    # Anything fronted by a bridge is indistinguishable from its siblings.
    if expected_family in _BRIDGE_AMBIGUOUS and detected_family in _BRIDGE_AMBIGUOUS:
        return False
    return True


# PlatformIO env names, read from firmware/platformio.ini once. A name that is
# not in here is not something `pio run -e` or a release artifact can be asked
# for, however sensible it looks.
def _pio_envs() -> set:
    import re
    ini = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "firmware", "platformio.ini")
    try:
        with open(ini) as fh:
            return set(re.findall(r"^\[env:([^\]]+)\]", fh.read(), re.M))
    except OSError:
        return set()


def pio_env_for(name: str, default: str = "esp32") -> str:
    """Normalise a board/controller/detection name to a PlatformIO env.

    The USB probe answers with a BOARD -- `gendrv` for a CP2102N, because that
    is what the bridge tells you. `gendrv` is not a PlatformIO env and never was:
    asking to flash it made the cockpit fetch
    `linorobot2-firmware-gendrv-jazzy.tar.gz`, which is in no release and never
    will be, so firmware upload was simply broken on the one ESP32 board the
    project ships a reference design for. The env is a property of the silicon;
    the board name only picks which silicon.
    """
    key = (name or "").strip().lower()
    envs = _pio_envs()
    if key in envs:
        return key
    family = _ENV_FAMILY.get(key)
    if family and family in envs:
        return family
    return default
