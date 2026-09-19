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
