"""The LiDAR link follows the micro-ROS transport and the robot computer's ports.

User, 2026-10-08: "do we have option to select lidar serial/udp/topic in web ui? it should
match wifi, number of serial ports." Measured the same day on a GenDrv with a real LD19:
serial 1.5 Mbaud + topic -- /raw_scan 37.5 Hz, /scan 9.98 Hz, the IMU 40 Hz; Wi-Fi + topic --
/scan 0.77 Hz and the IMU starved at 3.4 Hz; Wi-Fi + udp -- 3798 of 3798 packets arrived.
"""
import glob
import os
import re
import sys

import yaml

HERE = os.path.dirname(__file__)
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import depth_camera  # noqa: E402
import gen_bare_config  # noqa: E402
import lidar_link  # noqa: E402

# A GenDrv with a real LD19 on GPIO 4 -- pins assigned, so a real robot.
GENDRV = {"name": "gendrv", "mcu": "esp32", "serial_port": "/dev/ttyUSB0", "baudrate": 1500000,
          "lidar": {"model": "ld19", "comm_mode": "topic", "rx_pin": 4},
          "pins": {"motor1": {"pwm": 25, "in_a": 17, "in_b": 21}},
          "sensors": {"use_sim_ld19": False}}


def _ok(controller):
    return {o["mode"] for o in lidar_link.options(controller) if o["ok"]}


def test_a_cable_robot_at_1_5_mbaud_may_use_serial_or_topic():
    assert _ok(dict(GENDRV, transport="serial")) == {"serial", "topic"}


def test_a_921600_baud_link_has_no_room_for_the_scan():
    c = dict(GENDRV, transport="serial", baudrate=921600)
    assert _ok(c) == {"serial"}
    assert "921600" in lidar_link.why_not(c, "topic")


def test_native_usb_carries_the_scan_whatever_the_nominal_baud():
    pico = {"name": "pico2", "mcu": "pico2", "transport": "serial", "baudrate": 115200,
            "lidar": {"model": "ld19", "comm_mode": "topic"}}
    assert "topic" in _ok(pico)


def test_a_wifi_robot_sends_its_scan_as_udp_only():
    c = dict(GENDRV, transport="udp4")
    assert _ok(c) == {"udp"}
    assert "Wi-Fi" in lidar_link.why_not(c, "topic")
    assert "no cable" in lidar_link.why_not(c, "serial")
    assert "Wi-Fi robot" in lidar_link.why_not(dict(GENDRV, transport="serial"), "udp")


def test_bringup_refuses_topic_over_wifi():
    assert depth_camera.raw_scan_over_uart(dict(GENDRV, transport="udp4"))
    assert not depth_camera.raw_scan_over_uart(dict(GENDRV, transport="serial"))


def test_a_serial_lidar_needs_a_second_port():
    c = dict(GENDRV, transport="serial", lidar={"model": "ld19", "comm_mode": "serial",
                                                "serial_port": "/dev/ttyUSB0"})
    assert any("second port" in p for p in lidar_link.problems(c))
    c["lidar"]["serial_port"] = "/dev/ttyUSB1"
    assert lidar_link.problems(c) == []


def test_a_real_lidar_on_the_mcu_needs_its_pin():
    c = dict(GENDRV, transport="serial", lidar={"model": "ld19", "comm_mode": "topic", "rx_pin": -1})
    assert any("rx_pin" in p for p in lidar_link.problems(c))


def test_udp_server_is_udp():
    assert lidar_link.normal("udp_server") == "udp"
    c = dict(GENDRV, transport="udp4", lidar={"model": "ld19", "comm_mode": "udp_server", "rx_pin": 4})
    assert lidar_link.problems(c) == []


def test_every_shipped_design_and_bare_config_passes():
    configs = glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml"))
    bad = []
    for f in configs:
        bc = (yaml.safe_load(open(f)) or {}).get("base_controller") or {}
        bad += [(os.path.basename(f), p) for p in lidar_link.problems(bc)]
    for mcu in ("pico", "pico2", "picow", "pico2w", "esp32", "esp32s3", "xrp", "unoq"):
        bad += [(f"bare_{mcu}", p) for p in lidar_link.problems(gen_bare_config.bare_config(mcu)["base_controller"])]
    assert bad == []


def test_the_ui_offers_exactly_these_modes_and_saves_only_an_edit():
    html = open(os.path.join(ROOT, "web", "frontend", "index.html")).read()
    block = html[html.index('id="cfg-lidar-comm"'):]
    block = block[:block.index("</select>")]
    assert tuple(re.findall(r'<option value="([a-z]+)"', block)) == lidar_link.MODES
    js = open(os.path.join(ROOT, "web", "frontend", "app-hardware.js")).read()
    assert "...(lidarLinkTouched ? { lidar: lidarLinkForm() } : {})" in js
    assert "/api/hardware/lidar_link" in js


def test_the_backend_refuses_a_mode_the_link_cannot_carry():
    src = open(os.path.join(ROOT, "web", "backend", "routes_config.py")).read()
    assert '@app.post("/api/hardware/lidar_link")' in src
    assert "lidar_link.why_not(ctrl, mode)" in src and "status_code=400" in src
    assert '"lidar_link": _lidar_link(controller)' in src
