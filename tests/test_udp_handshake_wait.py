"""A Wi-Fi board has not joined the network when the agent starts.

A serial board is already enumerated by the time the micro-ROS agent runs, so
30 s to first /odom/unfiltered is generous. A udp4 board boots, associates,
takes a DHCP lease and only then finds the agent, and the LiDAR UDP client
connects later still.

Measured on a NodeMCU over Wi-Fi: from the ldlidar server binding port 8889 to
"ldlidar communication is normal" was 31.8 s. The audit ran first, found /scan
with no messages and aborted -- while /odom and /imu/data were already at 48 Hz
and the scan arrived seconds after the stack was torn down. Nothing was wrong
with the robot; the gate was tuned for a cable.
"""
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PIPELINE = os.path.join(REPO_ROOT, "scripts", "one_click_pipeline.py")


def _handshake_block():
    text = open(PIPELINE).read()
    m = re.search(r"transport = .*?wait_for_topic\(\"/odom/unfiltered\".*?\)", text, re.S)
    assert m, "the /odom/unfiltered handshake gate is gone or no longer reads the transport"
    return m.group(0)


def test_the_handshake_wait_is_not_a_constant():
    block = _handshake_block()
    assert "timeout_sec=handshake_wait" in block, (
        "the handshake gate is back to a fixed timeout, which aborts every "
        "Wi-Fi robot before it has associated"
    )


def test_udp_gets_materially_longer_than_serial():
    block = _handshake_block()
    m = re.search(r"handshake_wait = (\d+) if transport\.startswith\(\"serial\"\) else (\d+)", block)
    assert m, "the serial/udp split is gone"
    serial_s, udp_s = int(m.group(1)), int(m.group(2))
    # The measured first-contact was 31.8 s; anything at or under that is a gate
    # that fails the run it is supposed to be waiting for.
    assert udp_s >= 60, f"udp wait is {udp_s}s, not enough for association + DHCP + agent"
    assert udp_s > serial_s, "udp must wait longer than serial, not the same"


def test_the_transport_defaults_to_serial():
    """A config with no transport key is a cabled board; do not slow it down."""
    block = _handshake_block()
    assert 'controller_cfg.get("transport", "serial")' in block, (
        "a missing transport key must mean serial, not udp"
    )


def _scan_block():
    text = open(PIPELINE).read()
    m = re.search(r"if has_lidar:\s*\n\s*scan_wait = .*?wait_for_topic\(\"/scan\".*?\)", text, re.S)
    assert m, "the /scan gate before the topic audit is gone"
    return m.group(0)


def test_the_scan_gets_its_own_wait_before_the_audit():
    """Lengthening the handshake did not help: /odom is up long before /scan.

    The micro-ROS session carries /odom and /imu/data, and is live as soon as
    the board finds the agent. The LiDAR is a SECOND UDP client that connects
    to the ldlidar server afterwards -- 31.8 s later, measured. The handshake
    gate passed in seconds and the audit still ran into a /scan with no
    publisher and aborted a healthy run.
    """
    block = _scan_block()
    assert 'require_message="header.frame_id"' in block, (
        "the /scan gate must read a message; a topic that is merely listed is "
        "what the audit already fails on"
    )


def test_the_scan_wait_covers_a_udp_lidar():
    block = _scan_block()
    m = re.search(r"scan_wait = (\d+) if transport\.startswith\(\"serial\"\) else (\d+)", block)
    assert m, "the /scan wait is a constant again"
    serial_s, udp_s = int(m.group(1)), int(m.group(2))
    assert udp_s >= 60, (
        f"udp /scan wait is {udp_s}s; the measured first scan was 31.8 s after bind, "
        "so anything near that fails the run it is waiting for"
    )
    assert udp_s > serial_s


def test_a_robot_with_no_lidar_does_not_wait():
    block = _scan_block()
    assert block.startswith("if has_lidar:"), (
        "a robot with no scan source must not sit through the /scan wait"
    )
