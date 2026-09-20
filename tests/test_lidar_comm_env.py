"""Which sink the synthetic scan leaves by is a run-time choice, not a build one.

`comm_mode` used to be compiled in three ways at once: it zeroed LIDAR_RXD for
the topic and udp modes, it defined USE_LIDAR_UDP, and main.cpp derived
USE_FAKE_LD19_RAW_SCAN from both. So the transport was a property of the IMAGE.

The released `esp32` image used to be built from esp32_wifi_config.yaml, whose
comm_mode is `udp`.
Flashing it onto the gendrv bench -- wired for a serial LD19 into a USB-serial
bridge -- gave a board that could not be told to use the UART however its env
was keyed. It emitted at roughly the right average byte rate through the
`lidar_rx` override and ldlidar_stl_ros2 still rejected it, because the UDP
build's per-step budget is a whole revolution rather than eight packets. The env
key made it look configurable right up to the point where it did not work.

Now every image compiles all three sinks and picks at boot from `lidar_comm`,
falling back to LIDAR_COMM_DEFAULT -- the build's own comm_mode -- so a board
whose env lacks the key behaves exactly as it did before.
"""
import os
import re
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
REF = os.path.join(REPO_ROOT, "config", "reference")
FW = os.path.join(REPO_ROOT, "firmware")


def _header(cfg_stem, tmp_path, distro="jazzy"):
    import subprocess
    env = dict(os.environ)
    env["COCKPIT_CONFIG_DIR"] = str(tmp_path / "cfg")
    env.pop("ROS_DISTRO", None)
    hdr = os.path.join(FW, "include", "custom", "lino_base_config.h")
    saved = open(hdr).read() if os.path.exists(hdr) else None
    try:
        subprocess.run([sys.executable, os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py"),
                        "--params", os.path.join(REF, f"{cfg_stem}_config.yaml"),
                        "--distro", distro, "--no-embed-secrets"],
                       check=True, capture_output=True, env=env)
        return open(hdr).read()
    finally:
        if saved is not None:
            open(hdr, "w").write(saved)


def _fake_udp_config(tmp_path):
    """gendrv turned into what esp32_wifi_config.yaml used to be.

    That file -- the fake-mode, udp-sink ESP32 reference -- was deleted on
    2026-09-20 along with esp32_config.yaml: same silicon as gendrv, differing
    only in keys the env partition decides at boot. The COMBINATION it supplied
    is still worth testing, so it is built here instead of stored.
    """
    import yaml as _yaml
    src = _yaml.safe_load(open(os.path.join(REF, "gendrv_config.yaml")))
    ctrl = src["base_controller"]
    ctrl["transport"] = "udp4"
    ctrl["sensors"]["use_fake_ld19"] = True
    ctrl.setdefault("lidar", {})["comm_mode"] = "udp"
    cfg = tmp_path / "fake_udp_config.yaml"
    cfg.write_text(_yaml.safe_dump(src))
    return cfg, src


def _comm_mode(cfg_stem):
    d = yaml.safe_load(open(os.path.join(REF, f"{cfg_stem}_config.yaml")))
    lidar = ((d.get("hardware") or {}).get("lidar")
             or (d.get("controller") or {}).get("lidar") or {})
    if not lidar:
        for v in d.values():
            if isinstance(v, dict) and "lidar" in v:
                lidar = v["lidar"] or {}
                break
    return (lidar or {}).get("comm_mode")


@pytest.mark.parametrize("cfg_stem", ["gendrv"])
def test_the_header_names_the_build_default_mode(cfg_stem, tmp_path):
    text = _header(cfg_stem, tmp_path)
    m = re.search(r'#define LIDAR_COMM_DEFAULT\s+"(\w+)"', text)
    assert m, f"{cfg_stem}: no LIDAR_COMM_DEFAULT; the firmware has no fallback to resolve"
    want = _comm_mode(cfg_stem)
    want = "udp" if want in ("udp", "udp_server") else (want or "serial")
    assert m.group(1) == want, f"{cfg_stem}: default is {m.group(1)}, config says {want}"


def test_the_configured_rx_pin_survives_a_non_serial_build(tmp_path):
    """The pin is wiring; the mode is transport. Zeroing LIDAR_RXD because the
    build's comm_mode is udp is precisely what welded the transport into the image.

    No reference config combines a udp comm_mode with an rx_pin -- gendrv has the
    pin and is serial -- so this builds that combination rather than asserting
    against one that cannot fail.
    """
    import subprocess
    src = yaml.safe_load(open(os.path.join(REF, "gendrv_config.yaml")))

    def find_lidar(node):
        if isinstance(node, dict):
            if "lidar" in node and isinstance(node["lidar"], dict):
                return node["lidar"]
            for v in node.values():
                got = find_lidar(v)
                if got is not None:
                    return got
        return None

    lidar = find_lidar(src)
    assert lidar and lidar.get("rx_pin") == 4, "gendrv_config no longer wires rx_pin 4"
    lidar["comm_mode"] = "udp"
    cfg = tmp_path / "udp_with_pin_config.yaml"
    cfg.write_text(yaml.safe_dump(src))

    hdr = os.path.join(FW, "include", "custom", "lino_base_config.h")
    saved = open(hdr).read() if os.path.exists(hdr) else None
    env = dict(os.environ)
    env["COCKPIT_CONFIG_DIR"] = str(tmp_path / "cfg")
    env.pop("ROS_DISTRO", None)
    try:
        subprocess.run([sys.executable, os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py"),
                        "--params", str(cfg), "--distro", "jazzy", "--no-embed-secrets"],
                       check=True, capture_output=True, env=env)
        text = open(hdr).read()
    finally:
        if saved is not None:
            open(hdr, "w").write(saved)
    assert re.search(r"#define LIDAR_RXD 4\b", text), (
        "a udp-mode build zeroed the configured rx_pin, so that image can never be "
        "told to drive a serial LD19 however its env is keyed.")
    assert re.search(r'#define LIDAR_COMM_DEFAULT\s+"udp"', text)


def test_the_udp_destination_exists_even_in_a_serial_build(tmp_path):
    """A serial-default image told `lidar_comm=udp` still has to know where to send.
    Both are env lookups with a compiled fallback, so an image that never streams
    pays nothing for them."""
    text = _header("gendrv", tmp_path)
    assert "#define LIDAR_SERVER " in text and "#define LIDAR_PORT " in text


def test_use_lidar_udp_is_only_the_real_lidar_forwarder(tmp_path):
    """lidar.cpp's path is gated `USE_LIDAR_UDP && !USE_FAKE_LD19` -- forwarding a
    PHYSICAL LiDAR's bytes over UDP. A fake-mode udp robot must not define it, or
    the emulator's sink goes back to being chosen by the build."""
    import subprocess
    cfg, _src = _fake_udp_config(tmp_path)
    hdr = os.path.join(FW, "include", "custom", "lino_base_config.h")
    saved = open(hdr).read() if os.path.exists(hdr) else None
    env = dict(os.environ)
    env["COCKPIT_CONFIG_DIR"] = str(tmp_path / "cfg")
    env.pop("ROS_DISTRO", None)
    try:
        subprocess.run([sys.executable, os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py"),
                        "--params", str(cfg), "--distro", "jazzy", "--no-embed-secrets"],
                       check=True, capture_output=True, env=env)
        text = open(hdr).read()
    finally:
        if saved is not None:
            open(hdr, "w").write(saved)
    # Both sinks are compiled into every image now, so the header no longer
    # decides. What it still carries is the DEFAULT the board falls back to,
    # and lidar.cpp forwards a real LiDAR only when the env says the emulator
    # is off -- checked here rather than asserting on a macro that is gone.
    assert 'FAKE_LD19_DEFAULT true' in text, "the built config is a fake-mode one"
    assert "#define USE_LIDAR_UDP" not in text, (
        "USE_LIDAR_UDP is gone: forwarding a real LiDAR over UDP is a run-time "
        "decision from lidar_comm + fake_ld19, not a build.")


@pytest.mark.parametrize("cfg_stem", ["gendrv"])
def test_the_env_block_carries_the_mode(cfg_stem):
    """Without this key the firmware falls back to the image's own default, which is
    exactly the behaviour being fixed -- so mcu_env.py has to write it."""
    import mcu_env
    params = yaml.safe_load(open(os.path.join(REF, f"{cfg_stem}_config.yaml")))
    env = mcu_env.hardware_env(params)
    want = _comm_mode(cfg_stem)
    if want is None:
        pytest.skip(f"{cfg_stem} has no comm_mode")
    want = "udp" if want in ("udp", "udp_server") else want
    assert env.get("lidar_comm") == want, (
        f"{cfg_stem}: env has lidar_comm={env.get('lidar_comm')!r}, config says {want!r}")


def test_the_serial_lidar_driver_respawns():
    """A LiDAR that is merely late must not cost the robot /scan for the session.

    ldlidar_stl_ros2_node waits about three seconds for a valid frame on the
    serial port, then logs "ldlidar communication is abnormal", exits 1 and
    stays dead -- launch does not restart it unless told to. SLAM and Nav2 then
    block on a topic that will never appear.

    Late is the ordinary case: the driver and whatever produces the packets come
    up together. The esp32-jazzy release test failed exactly this way -- /scan
    NO DATA from a freshly flashed, still-booting ESP32 that was emitting clean
    47-byte LD19 packets at 15.3 kB/s when asked a minute later, and against
    which the very same driver then reported "communication is normal". A real
    LD19 needs its motor at speed before a frame validates, so this is not a
    bench artifact.
    """
    import re
    path = os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")
    text = open(path).read()

    # The serial driver is the last ldlidar Node in the lidar chain: the one
    # that is handed port_name and port_baudrate rather than a UDP server port.
    blocks = [m.start() for m in re.finditer(r'executable="ldlidar_stl_ros2_node"', text)]
    assert blocks, "no ldlidar node left in bringup.launch.py"
    serial = [b for b in blocks if "port_baudrate" in text[b:b + 3000]]
    assert serial, "no ldlidar node is configured with a serial port any more"
    for start in serial:
        node = text[start:start + 3000]
        assert "respawn=True" in node, (
            "the serial ldlidar node does not respawn; it dies ~3s after launch "
            "if its producer has not started yet, and never comes back"
        )
