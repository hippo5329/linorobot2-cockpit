"""The host target reads the SAME env image a board is flashed with.

`firmware/host/` runs the robot computer as a micro-ROS client so that the one
layer `scripts/fake_base_node.py` cannot exercise -- micro-ROS over UDP4 -- is
under test without silicon. That is only worth anything if the host is configured
the way a board is, which means `mcu_env.cpp` must read a real 4096-byte image
from `scripts/mcu_env.py`, CRC and 0xFF padding and all.

Two files therefore have to agree about a binary layout, and a contract between
two files that nothing asserts is the shape this project keeps paying for. So this
does not inspect either side's source for the spelling of a `#define`: it BUILDS
an image with the Python writer, compiles the firmware's own C++ reader against
the host shim, runs it, and requires the keys to come back. If either end changes
its layout, this goes red; if both change together, it stays green, which is the
correct behaviour for a contract test.

The compile also keeps the shim honest. `mcu_env.cpp` is firmware, compiled here
unmodified, so anything it starts needing from Arduino.h fails HERE -- at desk
speed, with no board and no micro-ROS workspace -- rather than in the container
leg that needs both.
"""
import os
import shutil
import subprocess
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(REPO_ROOT, "firmware")
SHIM = os.path.join(FW, "host", "shim")
MCU_ENV_CPP = os.path.join(FW, "common", "lib", "mcu_env", "mcu_env.cpp")
MCU_ENV_PY = os.path.join(REPO_ROOT, "scripts", "mcu_env.py")

# A harness, not firmware: it prints what the firmware's accessors return so the
# assertions below can be about values rather than about source text.
READER = r"""
#include <Arduino.h>
#include "mcu_env.h"
_LinoHostSerial Serial;
int main(void)
{
    initMcuEnv();
    if (!mcuEnvValid()) { printf("INVALID\n"); return 1; }
    printf("agent_ip=%s\n", envGet("agent_ip", "<absent>"));
    printf("agent_port=%u\n", (unsigned)envU16("agent_port", 0));
    printf("transport=%s\n", envGet("transport", "<absent>"));
    printf("base=%s\n", envGet("base", "<absent>"));
    printf("wheel_d=%.6f\n", envFloat("wheel_d", -1.0f));
    printf("fake_wheel=%d\n", (int)envFlag("fake_wheel", false));
    printf("missing=%s\n", envGet("no_such_key", "<fallback>"));
    printf("envIP=%s\n", envIP("agent_ip", IPAddress(0, 0, 0, 0)).c_str());
    return 0;
}
"""


@pytest.fixture(scope="module")
def reader(tmp_path_factory):
    """The firmware's own env parser, compiled for this machine."""
    cxx = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no C++ compiler: cannot compile the firmware's env parser")
    d = tmp_path_factory.mktemp("hostenv")
    # The host target has no generated robot header; the shim supplies config.h.
    (d / "reader.cpp").write_text(READER)
    out = d / "reader"
    # -Werror: the host target compiles firmware sources unmodified, so a new
    # warning here is a real signal about the firmware and not about this test.
    cmd = [cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-o", str(out),
           "-I", SHIM, "-I", os.path.dirname(MCU_ENV_CPP),
           str(d / "reader.cpp"), MCU_ENV_CPP]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    assert proc.returncode == 0, (
        "the firmware's mcu_env.cpp no longer compiles against firmware/host/shim.\n"
        "Whatever it now needs from Arduino.h has to be added to the shim, or the\n"
        "host micro-ROS target cannot be built at all:\n" + proc.stderr)
    return str(out)


def _build_image(path, **overrides):
    """An env image, written by the same script that flashes a board."""
    cmd = [sys.executable, MCU_ENV_PY, "build",
           "--params", os.path.join(REPO_ROOT, "config", "reference", "yb_eet01_config.yaml"),
           # A path that does not exist: mcu_env.py falls back to the example
           # placeholders, so this test never depends on a real secrets.yaml.
           "--secrets", os.path.join(str(path), "no-secrets.yaml"),
           "--host-ip", "10.11.12.13",
           "--out", str(path / "env.bin")]
    for key, value in overrides.items():
        cmd += ["--set", f"{key}={value}"]
    proc = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO_ROOT)
    assert proc.returncode == 0, proc.stderr + proc.stdout
    return path / "env.bin"


def _read(reader, image, cwd):
    proc = subprocess.run([reader], capture_output=True, text=True,
                          cwd=str(cwd), env={**os.environ, "LINO_ENV_BIN": str(image)})
    return proc.returncode, proc.stdout


def test_the_firmware_reads_the_flashers_image(reader, tmp_path):
    """The whole contract, in one pass: writer -> 4096 bytes -> reader."""
    image = _build_image(tmp_path, transport="udp4", agent_port="8877")
    assert image.stat().st_size == 4096, "the env image is one flash sector"

    rc, out = _read(reader, image, tmp_path)
    assert rc == 0, f"the firmware rejected the flasher's own image:\n{out}"
    got = dict(line.split("=", 1) for line in out.strip().splitlines() if "=" in line)

    # Values the CLI set, straight through the binary layout.
    assert got["agent_ip"] == "10.11.12.13"
    assert got["agent_port"] == "8877"
    assert got["transport"] == "udp4"
    # envIP goes through IPAddress::fromString, the path uros_transport.cpp uses.
    assert got["envIP"] == "10.11.12.13"
    # A value from the config rather than the CLI, so the config path is covered too.
    assert got["base"] == "2wd"
    # Typed accessors, not just envGet: a float and a flag.
    assert abs(float(got["wheel_d"]) - 0.1) < 1e-6
    assert got["fake_wheel"] in ("0", "1")
    # An absent key must yield the caller's fallback, never an empty string --
    # every accessor is documented as safe to call without checking validity.
    assert got["missing"] == "<fallback>"


def test_the_python_writer_and_cxx_reader_agree_on_every_key(reader, tmp_path):
    """Not a sample of keys: every key the writer emits must be readable.

    `mcu_env.py print` decodes the image with the Python reader. Comparing that to
    what the C++ walker finds catches a divergence in the layout itself -- an entry
    the writer packs in a way the firmware's walk cannot reach would otherwise only
    surface as one missing sensor on a real robot.
    """
    image = _build_image(tmp_path, transport="udp4")
    proc = subprocess.run([sys.executable, MCU_ENV_PY, "print", str(image), "--show-secrets"],
                          capture_output=True, text=True, cwd=REPO_ROOT)
    assert proc.returncode == 0, proc.stderr
    expected = dict(line.split("=", 1) for line in proc.stdout.strip().splitlines()
                    if "=" in line)
    assert len(expected) > 20, "the reference config should produce a full env"

    # Ask the C++ side for each key by name, through the firmware's own envGet.
    lister = tmp_path / "lister.cpp"
    lister.write_text(r"""
#include <Arduino.h>
#include "mcu_env.h"
_LinoHostSerial Serial;
int main(int argc, char **argv)
{
    initMcuEnv();
    if (!mcuEnvValid()) return 1;
    for (int i = 1; i < argc; i++)
        printf("%s=%s\n", argv[i], envGet(argv[i], "<ABSENT>"));
    return 0;
}
""")
    cxx = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
    out = tmp_path / "lister"
    proc = subprocess.run([cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-o", str(out),
                           "-I", SHIM, "-I", os.path.dirname(MCU_ENV_CPP),
                           str(lister), MCU_ENV_CPP], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr

    proc = subprocess.run([str(out), *sorted(expected)], capture_output=True, text=True,
                          env={**os.environ, "LINO_ENV_BIN": str(image)})
    assert proc.returncode == 0, proc.stdout
    got = dict(line.split("=", 1) for line in proc.stdout.strip().splitlines() if "=" in line)

    missing = sorted(k for k in expected if got.get(k) == "<ABSENT>")
    assert not missing, (
        "the firmware's env walk cannot reach keys scripts/mcu_env.py wrote: "
        f"{missing}")
    differing = {k: (expected[k], got[k]) for k in expected if got.get(k) != expected[k]}
    assert not differing, f"writer and reader disagree on values: {differing}"


def test_a_short_or_corrupt_image_is_refused(reader, tmp_path):
    """Neither failure may look like "no keys set".

    A truncated file would mmap with a zero-filled tail, and a zero reads as the
    empty entry that ends the list -- so half an env would parse as an empty one
    and the board would come up on compiled-in defaults, quietly. Both faults have
    to be loud and distinguishable, because only one of them is a flashing mistake.
    """
    image = _build_image(tmp_path, transport="udp4")
    blob = image.read_bytes()

    short = tmp_path / "short.bin"
    short.write_bytes(blob[:2048])
    rc, out = _read(reader, short, tmp_path)
    assert rc != 0 and "INVALID" in out, "a half-sector image must be refused"

    corrupt = tmp_path / "corrupt.bin"
    corrupt.write_bytes(blob[:8] + b"\x00\x00\x00\x00" + blob[12:])
    rc, out = _read(reader, corrupt, tmp_path)
    assert rc != 0 and "INVALID" in out, "a CRC32 mismatch must be refused"

    absent = tmp_path / "does-not-exist.bin"
    rc, out = _read(reader, absent, tmp_path)
    assert rc != 0 and "INVALID" in out, "a missing image must be refused"
