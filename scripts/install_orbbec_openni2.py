#!/usr/bin/env python3
"""Install Orbbec's OpenNI2, which lets openni2_camera see an Orbbec Astra Pro.

    python3 scripts/install_orbbec_openni2.py [--zip <OpenNI SDK zip>] [--config-dir DIR]

Run once on the robot computer, in the cockpit container. ros-<distro>-openni2-camera
(in the robot image) drives the camera through OpenNI2, but Debian's OpenNI2 (2.2.0.33)
carries only PrimeSense's drivers, and Orbbec's driver (liborbbec.so, built for OpenNI
2.3) crashes inside it: the device open segfaults in XnOniDriver::deviceOpen (gdb,
2026-10-10). On the OpenNI2 library of Orbbec's own SDK the same driver opens the camera
and streams depth at 30 Hz. The image does not redistribute either; this fetches Orbbec's
published OpenNI SDK release from Orbbec's GitHub, checks it against the pinned hashes
below, and keeps, for this computer's CPU:

    <config dir>/drivers/openni2/sdk/libOpenNI2.so.0              Orbbec's OpenNI2
    <config dir>/drivers/openni2/sdk/libOpenNI2.so                 -> libOpenNI2.so.0
    <config dir>/drivers/openni2/sdk/OpenNI.ini                    its settings
    <config dir>/drivers/openni2/sdk/OpenNI2/Drivers/liborbbec.so  the Astra driver
    <config dir>/drivers/openni2/sdk/OpenNI2/Drivers/orbbec.ini    its settings

launchers/openni2_depth.launch.py puts that directory first on the driver node's
LD_LIBRARY_PATH, so openni2_camera loads Orbbec's libOpenNI2.so.0 in place of Debian's,
and that library finds its drivers next to itself (OpenNI.ini's default Repository).
Nothing is written under /usr/lib, and the rest of the image keeps Debian's OpenNI2.

Orbbec's SDK states no license in its repository; it is Orbbec's software, fetched
from Orbbec on the user's machine -- nothing of it is in this project or its images.
"""
import argparse
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
# CPU -> {file under SDK_DIR: (its member in the archive, sha256)}. The settings files
# are text and shared; the libraries are per CPU.
_X64 = _ROOT + "sdk/libs/"
_ARM64 = _ROOT + "samples/samples/ThirdParty/OpenNI2/arm/Arm64/"
_INIS = {
    "OpenNI.ini": (_X64 + "OpenNI.ini",
                   "3896e8f831c6a56576e9e93a2e5b3e1d3fa72041e7d01e1d7f23b7cd907e7eff"),
    "OpenNI2/Drivers/orbbec.ini": (_X64 + "OpenNI2/Drivers/orbbec.ini",
                                   "b6a1cc4ed8ee62c7c4b2596e43727362ee489bd1733f59633794b005224d1e9b"),
}
FILES = {
    "x86_64": {
        "libOpenNI2.so.0": (_X64 + "libOpenNI2.so",
                            "958c38fb4e8724df2ed81b3fcf83ea9d6ab0ab6c65fa2862ed8f3ecf61ac08f2"),
        "OpenNI2/Drivers/liborbbec.so": (_X64 + "OpenNI2/Drivers/liborbbec.so",
                                         "baa27b0abbaa0c20a972a93e7f94cbc50c3a23e4a8f700f85b633591c1691f3e"),
        **_INIS,
    },
    "aarch64": {
        "libOpenNI2.so.0": (_ARM64 + "libOpenNI2.so",
                            "89ed05df0d72531756a4019561c5a6ce39fa60d6761ca23eca7ba0ba75a44447"),
        "OpenNI2/Drivers/liborbbec.so": (_ARM64 + "OpenNI2/Drivers/liborbbec.so",
                                         "79e1c24d2fa82955eac1158bb4b13ea4ca19bbe3b069c0d158585762f846d275"),
        **_INIS,
    },
}
SDK_DIR = os.path.join("drivers", "openni2", "sdk")    # depth_camera.ORBBEC_SDK_DIR


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
    """{path under SDK_DIR: bytes} for this CPU, each checked against its pinned hash."""
    if arch not in FILES:
        raise SystemExit(f"[orbbec] no Orbbec OpenNI2 for CPU {arch!r} "
                         f"(the SDK has {', '.join(FILES)})")
    out = {}
    with zipfile.ZipFile(zip_path) as z:
        for name, (member, want) in FILES[arch].items():
            data = z.read(member)
            got = hashlib.sha256(data).hexdigest()
            if got != want:
                raise SystemExit(f"[orbbec] {member}: sha256 {got} is not the pinned {want} -- refusing it")
            out[name] = data
    return out


def install(files: dict, config_dir: str) -> list:
    """Write the files under <config dir>/SDK_DIR, plus the unversioned library link."""
    dest = os.path.join(config_dir, SDK_DIR)
    written = []
    for name, data in files.items():
        path = os.path.join(dest, name)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path + ".new", "wb") as fh:
            fh.write(data)
        os.replace(path + ".new", path)
        written.append(path)
    link = os.path.join(dest, "libOpenNI2.so")
    if os.path.lexists(link):
        os.remove(link)
    os.symlink("libOpenNI2.so.0", link)
    written.append(link)
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
    print("[orbbec] the next bringup with depth_camera.model: astra_pro_openni2 runs openni2_camera on it")
    return 0


if __name__ == "__main__":
    sys.exit(main())
