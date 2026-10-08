#!/usr/bin/env python3
"""Install Orbbec's OpenNI2 driver, which lets openni2_camera see an Orbbec Astra Pro.

    python3 scripts/install_orbbec_openni2.py [--zip <OpenNI SDK zip>] [--config-dir DIR]

Run once on the robot computer, in the cockpit container. ros-<distro>-openni2-camera
(in the robot image) drives the camera through OpenNI2, but Debian's OpenNI2 carries
only PrimeSense's drivers: an Astra needs Orbbec's own (liborbbec.so). The image does
not redistribute it; this fetches Orbbec's published OpenNI SDK release from Orbbec's
GitHub, checks it against the pinned hashes below, and keeps two files from it:

    <config dir>/drivers/openni2/liborbbec.so.0   the driver, for this computer's CPU
    <config dir>/drivers/openni2/orbbec.ini        its settings

The container's entrypoint copies them into OpenNI2's Drivers directory at every start
(that directory is part of the image). If this runs as root it installs them there at
once as well; otherwise restart the cockpit.

Why `.so.0`: Debian's OpenNI2 loads only versioned driver files. With the file named
liborbbec.so it took libPS1080.so.0 and skipped Orbbec's (checked 2026-10-08).

Orbbec's SDK states no license in its repository; it is Orbbec's software, fetched
from Orbbec on the user's machine -- nothing of it is in this project or its images.
"""
import argparse
import glob
import hashlib
import os
import platform
import shutil
import sys
import tempfile
import urllib.request
import zipfile

# Orbbec's OpenNI SDK, release v2.3.0.86-beat6 (2022-10, published 2025-01-13). The Linux
# x64 archive carries the aarch64 driver too, so one download serves both CPUs.
SDK_URL = ("https://github.com/orbbec/OpenNI_SDK/releases/download/v2.3.0.86-beat6/"
           "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux_x64.zip")
SDK_SHA256 = "2240d645f3127bfcefbd2909be35bb35a446661e6a76d97dead414a4c5d684ad"
_ROOT = "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/"
# CPU -> (driver in the archive, its sha256), and the settings file both share.
DRIVERS = {
    "x86_64": (_ROOT + "sdk/libs/OpenNI2/Drivers/liborbbec.so",
               "baa27b0abbaa0c20a972a93e7f94cbc50c3a23e4a8f700f85b633591c1691f3e"),
    "aarch64": (_ROOT + "samples/samples/ThirdParty/OpenNI2/arm/Arm64/OpenNI2/Drivers/liborbbec.so",
                "79e1c24d2fa82955eac1158bb4b13ea4ca19bbe3b069c0d158585762f846d275"),
}
INI = (_ROOT + "sdk/libs/OpenNI2/Drivers/orbbec.ini",
       "b6a1cc4ed8ee62c7c4b2596e43727362ee489bd1733f59633794b005224d1e9b")
DRIVER_NAME = "liborbbec.so.0"      # depth_camera.ORBBEC_DRIVER
SUBDIR = os.path.join("drivers", "openni2")


def _sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _download(url: str, dest: str) -> None:
    print(f"[orbbec] downloading Orbbec's OpenNI SDK (about 220 MB) from {url}", flush=True)
    with urllib.request.urlopen(url, timeout=60) as r, open(dest, "wb") as out:
        shutil.copyfileobj(r, out, 1 << 20)


def extract(zip_path: str, arch: str) -> dict:
    """{file name: bytes} for this CPU, each checked against its pinned hash."""
    if arch not in DRIVERS:
        raise SystemExit(f"[orbbec] no Orbbec OpenNI2 driver for CPU {arch!r} "
                         f"(the SDK has {', '.join(DRIVERS)})")
    member, want = DRIVERS[arch]
    out = {}
    with zipfile.ZipFile(zip_path) as z:
        for name, (m, h) in ((DRIVER_NAME, (member, want)), ("orbbec.ini", INI)):
            data = z.read(m)
            got = hashlib.sha256(data).hexdigest()
            if got != h:
                raise SystemExit(f"[orbbec] {m}: sha256 {got} is not the pinned {h} -- refusing it")
            out[name] = data
    return out


def install(files: dict, config_dir: str) -> list:
    """Write the files to the config dir and, when writable, to every OpenNI2 Drivers dir."""
    dest = os.path.join(config_dir, SUBDIR)
    os.makedirs(dest, exist_ok=True)
    written = []
    for name, data in files.items():
        with open(os.path.join(dest, name), "wb") as fh:
            fh.write(data)
        written.append(os.path.join(dest, name))
    for d in glob.glob("/usr/lib/*/OpenNI2/Drivers"):
        if os.access(d, os.W_OK):
            for name, data in files.items():
                with open(os.path.join(d, name), "wb") as fh:
                    fh.write(data)
            written.append(d + "/")
    return written


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--zip", help="a copy of Orbbec's OpenNI SDK archive already on disk")
    ap.add_argument("--config-dir", default=os.environ.get("COCKPIT_CONFIG_DIR", "/config"))
    ap.add_argument("--arch", default=platform.machine())
    a = ap.parse_args(argv)
    with tempfile.TemporaryDirectory() as tmp:
        zip_path = a.zip
        if not zip_path:
            zip_path = os.path.join(tmp, "openni_sdk.zip")
            _download(SDK_URL, zip_path)
        got = _sha256_file(zip_path)
        if got != SDK_SHA256:
            raise SystemExit(f"[orbbec] {zip_path}: sha256 {got} is not the pinned {SDK_SHA256} -- "
                             f"not Orbbec's v2.3.0.86 archive, refusing it")
        files = extract(zip_path, a.arch)
    written = install(files, a.config_dir)
    for path in written:
        print(f"[orbbec] installed {path}")
    if not any(p.endswith("/") for p in written):
        print("[orbbec] OpenNI2's Drivers directory is not writable here: restart the cockpit, "
              "and its entrypoint installs the driver")
    return 0


if __name__ == "__main__":
    sys.exit(main())
