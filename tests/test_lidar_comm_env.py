"""Which sink the synthetic scan leaves by is a run-time choice, not a build one.

`comm_mode` used to be compiled in three ways at once: it zeroed LIDAR_RXD for
the topic and udp modes, it defined USE_LIDAR_UDP, and main.cpp derived
USE_SIM_LD19_RAW_SCAN from both. So the transport was a property of the IMAGE.

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


def _stub_udp_config(tmp_path):
    """gendrv turned into what esp32_wifi_config.yaml used to be.

    That file -- the simulation-mode, udp-sink ESP32 reference -- was deleted on
    2026-09-20 along with esp32_config.yaml: same silicon as gendrv, differing
    only in keys the env partition decides at boot. The COMBINATION it supplied
    is still worth testing, so it is built here instead of stored.
    """
    import yaml as _yaml
    src = _yaml.safe_load(open(os.path.join(REF, "gendrv_config.yaml")))
    ctrl = src["base_controller"]
    ctrl["transport"] = "udp4"
    ctrl["sensors"]["use_sim_ld19"] = True
    ctrl.setdefault("lidar", {})["comm_mode"] = "udp"
    cfg = tmp_path / "stub_udp_config.yaml"
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
    """lidar.cpp's path is gated `USE_LIDAR_UDP && !USE_SIM_LD19` -- forwarding a
    PHYSICAL LiDAR's bytes over UDP. A simulation-mode udp robot must not define it, or
    the emulator's sink goes back to being chosen by the build."""
    import subprocess
    cfg, _src = _stub_udp_config(tmp_path)
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
    assert 'SIM_LD19_DEFAULT true' in text, "the built config is a simulation-mode one"
    assert "#define USE_LIDAR_UDP" not in text, (
        "USE_LIDAR_UDP is gone: forwarding a real LiDAR over UDP is a run-time "
        "decision from lidar_comm + sim_ld19, not a build.")


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


def test_the_host_raycasts_only_in_its_special_cases():
    """A board's simulated LD19 goes through the LD driver (user, 2026-10-06: "let topic
    /raw_scan go through stl driver to generate /scan"). sim_laser_node raycasts on the robot
    computer only with no board (sim_base), when asked (host_laser: an ESP32 / ESP32-S3 UART at
    921600 has no link for its scan -- "update test suit to use sim_laser_node with
    esp32/esp32s3 serial transport uart baud 921600"), or on a saved-map world on a board
    ("world map support use sim_laser_node").

    Until then any comm mode but serial went to the host, so a `topic` board's raw_scan was
    read by nothing (bringup logs: lidar_mode='topic', sim_laser_node, no ld19)."""
    text = open(os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")).read()
    assert 'effective_lidar_comm_mode != "serial"' not in text
    assert 'host_laser_why = "no board (sim_base)"' in text
    assert "host_laser_why = \"asked for (host_laser)" in text
    assert 'host_laser_why = f"a board cannot hold the saved map {world_map_path}"' in text
    assert "use_host_sim_laser = bool(host_laser_why)" in text
    assert "[HOST LASER REFUSED]" in text
    assert "[bringup] NO /scan: {no_scan_why}" in text
    assert "if (not robot_has_lidar or no_scan_why)" in text
    assert '"comm_mode": lidar_comm_mode,' in text and '"raw_scan_topic": lidar_raw_topic,' in text
    pipe = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert 'parser.add_argument("--host-laser"' in pipe and '" host_laser:=true" if host_laser' in pipe

def test_scan_wait_follows_the_scan_source_not_the_transport():
    """The pipeline's /scan gate must key on who PRODUCES the scan.

    It keyed on the micro-ROS transport, so a board with micro-ROS on a cable and
    sim_ld19 out a UART into a second bridge -- the GenDrv -- got 15 s while the
    real ldlidar driver was still respawning against a board rebooting from the
    flash the same run had just done. Both distros failed with /scan NO DATA on a
    healthy board, while the udp leg beside it passed with 90 s.
    """
    src = open(os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")).read()
    assert "scan_wait = 15 if transport.startswith" not in src, \
        "the scan gate must not key on the micro-ROS transport"
    assert "scan_wait = 15 if host_room else 90" in src
    # and it must decide host_room the same way bringup decides to launch the
    # virtual room, or the gate and the stack disagree about what will publish.
    assert 'use_sim_ld19' in src and 'lidar_mode != "serial"' in src
    assert "os.path.exists(lidar_port_cfg)" in src
