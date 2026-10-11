"""The Yahboom YB-EET01 design runs with no robot computer, and /imu/data is held to 20 Hz.

The board has no robot computer on it, so micro-ROS runs over Wi-Fi (udp4) and its LiDAR,
on the board's own header (UART1 RX GPIO 18), is forwarded to the LD driver over UDP --
the design used to read the LiDAR on a robot computer's serial port, which this robot does
not have. /imu/data comes at 50 Hz from every board; the topic gate used to pass 7 Hz.
"""
import os
import re

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _bc():
    with open(os.path.join(ROOT, "config", "reference", "yb_eet01_config.yaml")) as fh:
        return yaml.safe_load(fh)["base_controller"]


def test_the_yahboom_runs_micro_ros_over_wifi():
    bc = _bc()
    assert bc["transport"] == "udp4"
    assert bc["wifi"]["enabled"] is True
    assert bc["telemetry"]["ota"] is True and bc["telemetry"]["syslog"] is True


def test_the_yahboom_lidar_is_on_its_own_header_and_goes_out_over_udp():
    lidar = _bc()["lidar"]
    assert lidar["comm_mode"] == "udp"
    assert lidar["rx_pin"] == 18          # UART1 RX, the MS200 connector
    assert lidar["model"] == "ms200"


def test_imu_data_must_reach_20_hz():
    src = open(os.path.join(ROOT, "scripts", "verify_topics.py")).read()
    m = re.search(r'"/imu/data":\s*([0-9.]+),', src)
    assert m and float(m.group(1)) == 20.0
    assert "/imu/data  >= 20.0 Hz" in src
