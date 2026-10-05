#!/usr/bin/env python3
"""Package an Arduino UNO Q firmware build as a release archive, like build_prebuilt.py's.

    package_unoq.py <zephyr build dir> <distro> [--dist DIR]

The UNO Q's STM32U585 firmware is built with Zephyr (firmware/zephyr/unoq/build.sh,
MICROROS_DISTRO=<distro>), not PlatformIO, so build_prebuilt.py cannot make it. This
takes that build's zephyr.elf and zephyr.bin and writes firmware/prebuilt/unoq-<distro>/
with the same manifest shape the other profiles have, and with --dist the
linorobot2-firmware-unoq-<distro>.tar.gz a release attaches.

The image is checked before it is packaged, on its bytes, by the same function the other
profiles go through: both /cmd_vel types linked, and its FW_ROS_DISTRO stamp equal to
<distro>. A lyrical UNO Q image once said "jazzy" (a fixed define in its CMakeLists), so
it listened for Twist while lyrical's Nav2 sent TwistStamped.
"""
import argparse
import json
import os
import shutil
import sys
import tarfile
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build_prebuilt as bp  # noqa: E402

ENV_OFFSET = "0x081FE000"   # the last 8 KB page of the 2 MB flash (fw/app.overlay lino_env_partition)
DISTROS = ("jazzy", "lyrical")


def package(build_dir, distro, dist=None):
    if distro not in DISTROS:
        raise SystemExit(f"package_unoq: distro must be one of {', '.join(DISTROS)}")
    zdir = os.path.join(build_dir, "zephyr")
    elf, binf = os.path.join(zdir, "zephyr.elf"), os.path.join(zdir, "zephyr.bin")
    for p in (elf, binf):
        if not os.path.isfile(p):
            raise SystemExit(f"package_unoq: {p} is missing -- build with firmware/zephyr/unoq/build.sh first")

    profile = f"unoq-{distro}"
    bp.check_cmd_vel_contract(profile, binf, distro)

    out_dir = os.path.join(bp.PREBUILT_DIR, profile)
    shutil.rmtree(out_dir, ignore_errors=True)
    os.makedirs(out_dir)
    files = []
    for src, name in ((binf, "firmware.bin"), (elf, "firmware.elf")):
        dst = os.path.join(out_dir, name)
        shutil.copyfile(src, dst)
        files.append({"name": name, "size": os.path.getsize(dst), "sha256": bp.sha256(dst)})
    files[0]["offset"] = "0x08000000"

    manifest = {
        "profile": profile,
        "description": f"Arduino UNO Q STM32U585, Zephyr, micro-ROS over the internal UART (4 Mbaud) [{distro}]",
        "config": "generated bare module (unoq)",
        "pio_env": "unoq",
        "build": f"MICROROS_DISTRO={distro} firmware/zephyr/unoq/build.sh",
        "ros_distro": distro,
        "sim_mode": False,
        "built": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "commit": bp.git_commit(),
        "files": files,
        "flash": "From the UNO Q's own Linux, over SWD with its OpenOCD (/opt/openocd): "
                 "program firmware.elf, then write the env block at " + ENV_OFFSET + ".",
        "env_partition": {
            "offset": ENV_OFFSET,
            "note": "Wi-Fi keys and addresses are NOT in this image. Build the env block with "
                    f"scripts/mcu_env.py and write it at {ENV_OFFSET}; reflashing the application "
                    "never disturbs it.",
        },
    }
    with open(os.path.join(out_dir, "manifest.json"), "w") as fh:
        json.dump(manifest, fh, indent=2)
        fh.write("\n")
    print(f"  -> {out_dir}  ({sum(f['size'] for f in files):,} bytes)")

    if dist:
        os.makedirs(dist, exist_ok=True)
        archive = os.path.join(dist, f"linorobot2-firmware-{profile}.tar.gz")
        with tarfile.open(archive, "w:gz") as tar:
            for name in sorted(os.listdir(out_dir)):
                tar.add(os.path.join(out_dir, name), arcname=name)
        print(f"  -> {archive}")
    return manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("build_dir", help="the Zephyr build directory, e.g. <workdir>/build-fw")
    ap.add_argument("distro", choices=DISTROS)
    ap.add_argument("--dist", help="also write linorobot2-firmware-unoq-<distro>.tar.gz here")
    a = ap.parse_args()
    package(a.build_dir, a.distro, a.dist)


if __name__ == "__main__":
    main()
