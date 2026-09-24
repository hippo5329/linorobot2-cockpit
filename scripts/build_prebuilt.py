#!/usr/bin/env python3
"""
build_prebuilt.py — produce the ready-to-flash images in firmware/prebuilt/.

Sim-mode images, one per MCU per ROS 2 distribution, so a user can bring a
board up without installing a toolchain at all:

    pico2    RP2350, micro-ROS over USB serial
    pico     RP2040, micro-ROS over USB serial
    esp32    ESP32, serial or udp4 — whichever the env partition asks for
    esp32s3  ESP32-S3, native USB CDC, serial or udp4

One per MCU, not one per robot. A Waveshare General Driver board and a bare
DevKit run the SAME esp32 image; what differs between them -- pin matrix, LiDAR
wiring, transport, which IMU is fitted -- is in the env partition. That is the
whole point of the env: a robot is a configuration, not a build.

Distro, however, is NOT a configuration. board_microros_distro selects the
precompiled micro_ros library the firmware links against, and a jazzy image will
not talk to a lyrical agent, so each board ships twice -- and every profile says
which one it is, rather than the jazzy half being spelled as the bare board name:

    pico2-jazzy    pico-jazzy    esp32-jazzy    esp32s3-jazzy
    pico2-lyrical  pico-lyrical  esp32-lyrical  esp32s3-lyrical

Sim mode throughout: the firmware simulates the IMU, magnetometer and wheels,
so a bare board with nothing wired to it still produces odometry and, where the
profile has a scan source, a /scan. That is what makes a prebuilt image useful
without knowing anything about the user's hardware.

The esp32 image contains NO credentials. Its Wi-Fi keys and the agent,
syslog and LiDAR-UDP addresses live in the separate `env` flash partition
(scripts/mcu_env.py), because an image with an SSID compiled into it works on
one LAN and could not be shipped to anyone. Flashing writes the env block
alongside the application; re-keying later rewrites only that 4 KB.

Needs PlatformIO. Run it where `pio` is installed, or through the build image:

    docker compose run --rm pio python3 scripts/build_prebuilt.py

`--dist DIR` additionally writes one `linorobot2-firmware-<profile>.tar.gz` per
profile, which is what a release attaches (scripts/fetch_prebuilt.py downloads
and verifies them).
"""
import argparse
import glob
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tarfile
from datetime import datetime, timezone

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cockpit_paths  # noqa: E402
PREBUILT_DIR = os.path.join(REPO_ROOT, "firmware", "prebuilt")
BASE_DIR = os.path.join(REPO_ROOT, "firmware")

# board -> (mcu, base pio env, description)
#
# Every release image is built from ITS SILICON AND NOTHING ELSE: no config
# file is read, no robot is described. gen_firmware_header.py --mcu.
#
# Twice now a design has leaked in through a path that looked clean:
#
#   2026-09-20, directly. It built from pico2_mecanum and gendrv, and two bare
#   Picos came up printing
#       [range] HC-SR04 trigger=27 echo=28 (interrupt driven)
#   and publishing /sonar, because the mecanum reference wires a sonar there
#   and the header's TRIG_PIN/ECHO_PIN are the fallback when a config is silent.
#
#   2026-09-23, through the generated bare config. "Generated, not read from
#   config/reference" was true of the pins and false of everything else:
#   gen_bare_config donates geometry/ekf/slam/nav2 from gendrv_config.yaml, so
#   that design's costmap radius shipped in every image as SIM_ROBOT_RADIUS.
#
# Both times the wrong robot's dimensions ended up in someone else's image. A
# released image must describe NO robot: the env partition is what turns it
# into one. The wired designs are still tested -- they are what the bench
# flashes for the sonar and real-IMU cases -- but they are not what ships.
BOARDS = {
    "pico2":   ("pico2",   "pico2w",  "RP2350, micro-ROS over USB serial (runs on Pico 2 and Pico 2 W)"),
    "pico":    ("pico",    "picow",   "RP2040, micro-ROS over USB serial (runs on Pico and Pico W)"),
    "esp32":   ("esp32",   "esp32",   "ESP32, serial or udp4 — chosen by the env partition"),
    "esp32s3": ("esp32s3", "esp32s3", "ESP32-S3, native USB CDC, serial or udp4"),
}

# The distro the BARE env names in firmware/platformio.ini are pinned to; the
# others get an explicit `<env>_<distro>` env there. That asymmetry is a
# PlatformIO detail and it used to leak out here, where the jazzy images were
# published under the bare board name and only the lyrical ones were suffixed --
# so `pico2` and `pico2-lyrical` sat side by side in a release and only one of
# them said what it was. Every profile now carries its distro.
DEFAULT_DISTRO = "jazzy"
DISTROS = ("jazzy", "lyrical")


def _profiles():
    """profile -> (config stem, pio env, distro, description).

    The profile name always ends in `-<distro>`; the PlatformIO env still does
    not, because [env:pico2] is pinned to jazzy in platformio.ini and renaming it
    would mean re-pointing every reference config's `pio_env`.
    """
    out = {}
    for distro in DISTROS:
        for board, (stem, env, desc) in BOARDS.items():
            pio_env = env if distro == DEFAULT_DISTRO else f"{env}_{distro}"
            out[f"{board}-{distro}"] = (stem, pio_env, distro, f"{desc} [{distro}]")
    return out


PROFILES = _profiles()

# Must match firmware/common/partitions_lino.csv and scripts/mcu_env.py.
# The ESP32 family's env address. RP2 boards keep theirs in the last page of
# their own flash (0x101FF000 on a 2 MB part, 0x103FF000 on 4 MB), so the
# manifest must be told PER PROFILE -- mcu_env.env_offset() is the one place
# that knows, and writing the ESP32 constant into a pico manifest would send
# a flasher at an address that is not on the chip.
ENV_OFFSET = "0x3FF000"


def env_offset_for(pio_env: str) -> str:
    import mcu_env
    return f"0x{mcu_env.env_offset(pio_env):X}"


def sh(cmd, **kwargs):
    print(f"  $ {' '.join(cmd)}", flush=True)
    return subprocess.run(cmd, check=True, **kwargs)


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_commit():
    """The revision the firmware banner will carry, spelled the way
    firmware/common/build_stamp.py spells it (7 chars, '+' when dirty), so
    mcu_probe.py can compare the manifest with the board."""
    def git(*args):
        try:
            res = subprocess.run(["git", "-C", REPO_ROOT, *args],
                                 capture_output=True, text=True, timeout=10)
            return res.stdout.strip() if res.returncode == 0 else ""
        except Exception:
            return ""
    rev = git("rev-parse", "--short=7", "HEAD")
    if not rev:
        try:
            return open(os.path.join(REPO_ROOT, "firmware", ".git_rev")).read().split()[0]
        except Exception:
            return "unknown"
    return rev + ("+" if git("status", "--porcelain") else "")


def collect(profile, env, build_dir, out_dir):
    """Copy the flashable artifacts, with the offset each one is written at."""
    files = []

    uf2 = os.path.join(build_dir, "firmware.uf2")
    if os.path.isfile(uf2):
        # RP2040/RP2350: one self-describing image. The UF2 carries its own load
        # addresses, so picotool needs no offset from us.
        shutil.copy2(uf2, out_dir)
        files.append({"name": "firmware.uf2", "offset": None, "tool": "picotool"})
        return files

    boot_offset = "0x0" if env.startswith(("esp32s3", "esp32c3", "esp32c6")) else "0x1000"
    for name, offset in (("bootloader.bin", boot_offset),
                         ("partitions.bin", "0x8000"),
                         ("firmware.bin", "0x10000")):
        path = os.path.join(build_dir, name)
        if not os.path.isfile(path):
            raise SystemExit(f"build_prebuilt: {profile} is missing {name} in {build_dir}")
        shutil.copy2(path, out_dir)
        files.append({"name": name, "offset": offset, "tool": "esptool"})

    # boot_app0 lives in the framework package, not the build directory. It is
    # copied in so that flashing a prebuilt image needs no PlatformIO install at
    # all -- which is the entire point of shipping one.
    for src in sorted(glob.glob(os.path.join(
            os.environ.get("PLATFORMIO_CORE_DIR", os.path.expanduser("~/.platformio")),
            "packages", "framework-arduinoespressif32*", "tools", "partitions", "boot_app0.bin"))):
        shutil.copy2(src, out_dir)
        files.append({"name": "boot_app0.bin", "offset": "0xe000", "tool": "esptool"})
        break
    else:
        print("  ! boot_app0.bin not found — OTA slot selection will be unset", file=sys.stderr)
    return files


def header_cmd(mcu, distro):
    """The gen_firmware_header.py invocation for one profile.

    --mcu, never --params: the image is built for a silicon and describes no
    robot. It used to be built from a generated bare config, which sounds the
    same and was not -- gen_bare_config donates its geometry/ekf/slam/nav2 from
    config/reference/gendrv_config.yaml, so that design's costmap radius was
    reaching every released image as #define SIM_ROBOT_RADIUS. A reference
    design is a robot someone can own, not a build input; every robot fact now
    arrives through the env partition at flash time.

    A function rather than three lines inline, because the --distro argument is
    the entire /cmd_vel contract of the image and nothing downstream can detect
    it being wrong. gen_firmware_header.py resolves `stamped_cmd_vel: auto` --
    which is what EVERY robot config says -- from this flag alone, falling
    back to $ROS_DISTRO; the release runner that builds these images is a bare
    ubuntu with pip platformio and no ROS, so that fallback is the empty string.
    Omitting the flag shipped all four -lyrical images subscribing to plain
    Twist while nav2 >= kilted publishes /cmd_vel as TwistStamped: a board that
    links, flashes, enumerates, publishes odometry at 50 Hz, and never moves
    under autonomy. The pio env chooses the micro-ROS library; only this
    chooses the contract.

    --no-embed-secrets unconditionally, for every image without exception. The
    first build of these did leak, from the profile least expected to: a board
    running micro-ROS over SERIAL still brings Wi-Fi up for syslog and OTA, so
    its header embedded the SSID and PSK exactly as a udp4 one did. A profile's
    transport says nothing about whether it has credentials to leak.
    """
    return [sys.executable, os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py"),
            "--mcu", mcu, "--distro", distro, "--no-embed-secrets"]


def build(profile, keep_going=False):
    mcu, env, distro, description = PROFILES[profile]
    out_dir = os.path.join(PREBUILT_DIR, profile)

    print(f"\n=== {profile}  (bare {mcu} -> pio env {env})", flush=True)

    # No config file at all -- see header_cmd(). The silicon is the only input.
    sh(header_cmd(mcu, distro))

    # Every image reads the env partition -- that is the whole configuration
    # model -- so every manifest records where it is.
    #
    # This used to test the header for `#define USE_MCU_ENV`, which was exactly
    # right while that macro gated the env reader. The gate is gone (an image
    # that cannot read its env cannot be configured, so there was nothing left
    # to choose), and the test silently became false for every profile: the
    # release went out with no `env_partition` in any manifest, and
    # flash_mcu.py checks for that key before writing one.
    uses_env = True

    # A release image describes no robot, so it is neither sim nor real until
    # an env is written to it. There is no config to read this from any more.
    sim_mode = False
    sh(["pio", "run", "-d", BASE_DIR, "-e", env])

    build_dir = os.path.join(BASE_DIR, ".pio", "build", env)
    if os.path.isdir(out_dir):
        # Rename aside rather than delete in place: the one-click pipeline runs
        # as container-root and the cockpit's backend as the container user, so
        # this directory is routinely owned by the other of the two, and
        # removing entries inside it needs write permission on IT, not on the
        # parent. fetch_prebuilt hit the same wall and showed the user a
        # traceback. Renaming needs the parent only, which both can write.
        stale = f"{out_dir}.stale.{os.getpid()}"
        shutil.rmtree(stale, ignore_errors=True)
        try:
            os.rename(out_dir, stale)
        except OSError:
            shutil.rmtree(out_dir, ignore_errors=True)
        else:
            shutil.rmtree(stale, ignore_errors=True)
    os.makedirs(out_dir, exist_ok=True)
    files = collect(profile, env, build_dir, out_dir)

    for entry in files:
        path = os.path.join(out_dir, entry["name"])
        entry["size"] = os.path.getsize(path)
        entry["sha256"] = sha256(path)

    # The /cmd_vel contract, checked on the ARTIFACT before it is published.
    #
    # A jazzy image that subscribes TwistStamped enumerates, publishes odometry
    # and never moves: nav2 on jazzy publishes plain Twist and nothing is
    # delivered. It is silent on both sides. This happened -- a pico2-jazzy
    # image went onto the bench stamped, and the Nav2 goal test reported "494
    # cmd_vel msgs, base moved 0.002 m" with no other symptom.
    #
    # release.yml has a version of this check, but it only fires one way: it
    # catches a lyrical image MISSING TwistStamped, not a jazzy image that has
    # it. Both directions are failures and both are checked here, in the script
    # that writes the artifact, so a locally cut release is checked too.
    img = next((os.path.join(out_dir, n) for n in ("firmware.uf2", "firmware.bin")
                if os.path.isfile(os.path.join(out_dir, n))), None)
    if img:
        try:
            found = subprocess.run(["strings", "-a", img], capture_output=True,
                                   text=True, timeout=120).stdout.count("TwistStamped")
        except Exception:
            found = None
        if found is not None:
            wants_stamped = distro not in ("humble", "iron", "jazzy")
            if wants_stamped and found == 0:
                raise SystemExit(
                    f"{profile}: built for {distro}, which publishes /cmd_vel as "
                    f"TwistStamped, but the image contains none. It would enumerate "
                    f"and never move.")
            if not wants_stamped and found:
                raise SystemExit(
                    f"{profile}: built for {distro}, which publishes /cmd_vel as "
                    f"plain Twist, but the image contains {found} TwistStamped "
                    f"references. nav2 would publish Twist, the board would "
                    f"subscribe TwistStamped, and nothing would be delivered.")
            print(f"  /cmd_vel contract ok ({distro}: "
                  f"{'TwistStamped' if wants_stamped else 'Twist'})", flush=True)

    manifest = {
        "profile": profile,
        "description": description,
        "config": f"generated bare module ({mcu})",
        "pio_env": env,
        "ros_distro": distro,
        # False because the image commits to nothing: `sim_wheel`, `sim_ld19`
        # and the rest are env keys, so any of these images is sim or real at
        # run time depending only on what was written to its env partition.
        # (It was once hardcoded True, then read from whichever config built
        # the image -- both were answering a question the image cannot have an
        # opinion on.)
        "sim_mode": sim_mode,
        "built": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "commit": git_commit(),
        "files": files,
    }
    if uses_env:
        manifest["env_partition"] = {
            "offset": env_offset_for(env),
            "note": "Wi-Fi keys and the agent / syslog / lidar_udp addresses are NOT in "
                    "this image. Build the env block with scripts/mcu_env.py and write "
                    f"it at {env_offset_for(env)}; reflashing the application never disturbs it.",
        }
    with open(os.path.join(out_dir, "manifest.json"), "w") as fh:
        json.dump(manifest, fh, indent=2)
        fh.write("\n")

    total = sum(entry["size"] for entry in files)
    print(f"  -> {out_dir}  ({len(files)} files, {total:,} bytes)")
    return manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("profiles", nargs="*", default=list(PROFILES),
                    help=f"which to build (default: all of {', '.join(PROFILES)})")
    ap.add_argument("--distro", choices=DISTROS,
                    help="build only this ROS 2 distribution's half of the matrix")
    ap.add_argument("--dist", metavar="DIR", default=None,
                    help="also write linorobot2-firmware-<profile>.tar.gz release archives here")
    a = ap.parse_args()

    unknown = [p for p in a.profiles if p not in PROFILES]
    if unknown:
        sys.exit(f"build_prebuilt: unknown profile(s): {', '.join(unknown)}")
    if a.distro:
        a.profiles = [p for p in a.profiles if PROFILES[p][2] == a.distro]
        if not a.profiles:
            sys.exit(f"build_prebuilt: no profiles for distro '{a.distro}'")

    os.makedirs(PREBUILT_DIR, exist_ok=True)
    built = [build(p) for p in a.profiles]
    if a.dist:
        os.makedirs(a.dist, exist_ok=True)
        for manifest in built:
            profile = manifest["profile"]
            archive = os.path.join(a.dist, f"linorobot2-firmware-{profile}.tar.gz")
            with tarfile.open(archive, "w:gz") as tar:
                src = os.path.join(PREBUILT_DIR, profile)
                for name in sorted(os.listdir(src)):
                    tar.add(os.path.join(src, name), arcname=name)
            print(f"  -> {archive}")

    print("\n=== prebuilt images")
    for manifest in built:
        size = sum(entry["size"] for entry in manifest["files"])
        print(f"  {manifest['profile']:18} {manifest['pio_env']:18} "
              f"{manifest['ros_distro']:8} {size:>10,} bytes")


if __name__ == "__main__":
    main()
