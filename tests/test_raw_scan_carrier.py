"""Which link may carry the scan as raw_scan, and what a board with no carrier gets (2026-10-06).

User: "esp32 serial transport and raw scan topic will exhaust 921600 baud bandwidth";
"esp32s3 cdc can support raw scan topic. esp32s3 uart cannot"; "esp32s3 uart like yahboom";
"esp32 scan legs on gendrv only"; "We use raw scan topic only when the serial link is fast
enough (like cdc or 1.5M/4M) as in pico/pico2/esp32s3 cdc/unoq".
"""
import copy
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import depth_camera  # noqa: E402
import gen_bare_config  # noqa: E402


def bare(mcu):
    return gen_bare_config.bare_config(mcu)["base_controller"]


def test_native_usb_carries_raw_scan():
    for mcu in ("pico", "pico2", "picow", "pico2w", "xrp", "esp32s3"):
        bc = bare(mcu)
        assert bc["lidar"]["comm_mode"] == "topic", mcu
        assert depth_camera.raw_scan_over_uart(bc) == "", mcu
    assert bare("esp32s3")["console"] == "usb"


def test_an_esp32_uart_is_refused_raw_scan():
    bc = bare("esp32")
    assert bc["lidar"]["comm_mode"] == "serial"          # its bare default: no topic
    bc["lidar"]["comm_mode"] = "topic"
    assert "921600" in depth_camera.raw_scan_over_uart(bc)


def test_an_esp32s3_on_its_uart_is_refused_raw_scan_like_the_yahboom():
    with open(os.path.join(ROOT, "config", "reference", "yb_eet01_config.yaml")) as fh:
        yb = yaml.safe_load(fh)["base_controller"]
    assert str(yb.get("console")).lower() == "uart0"
    yb = copy.deepcopy(yb)
    yb.setdefault("lidar", {}).update(model="ld19", comm_mode="topic")
    yb["transport"] = "serial"
    assert depth_camera.raw_scan_over_uart(yb)
    s3 = bare("esp32s3")
    s3["console"] = "uart0"
    assert depth_camera.raw_scan_over_uart(s3)


def test_a_fast_uart_carries_raw_scan_the_gendrv_and_the_uno_q():
    with open(os.path.join(ROOT, "config", "reference", "gendrv_config.yaml")) as fh:
        gd = copy.deepcopy(yaml.safe_load(fh)["base_controller"])
    assert int(gd["baudrate"]) == 1500000
    gd["lidar"]["comm_mode"] = "topic"
    assert depth_camera.raw_scan_over_uart(gd) == ""
    uq = bare("unoq")
    assert int(uq["baudrate"]) == 4000000 and uq["lidar"]["comm_mode"] == "topic"
    assert depth_camera.raw_scan_over_uart(uq) == ""
    slow = copy.deepcopy(gd)
    slow["baudrate"] = 921600
    assert "921600" in depth_camera.raw_scan_over_uart(slow)


def test_wifi_micro_ros_cannot_carry_the_scan_either():
    # It was allowed as "not a UART". Measured 2026-10-08 on a GenDrv with a real LD19:
    # raw_scan over Wi-Fi came at 3.3 Hz, /scan at 0.77 Hz, the IMU starved at 3.4 Hz --
    # the same board on its 1.5 Mbaud cable held 37.5 / 9.98 / 40 Hz. Over Wi-Fi: udp.
    bc = bare("esp32")
    bc.update(transport="udp4")
    bc["lidar"]["comm_mode"] = "topic"
    reason = depth_camera.raw_scan_over_uart(bc)
    assert "Wi-Fi" in reason and "udp" in reason


def test_a_serial_scan_with_no_port_reaches_nothing(tmp_path):
    bc = bare("esp32")
    assert depth_camera.sim_scan_unreachable(bc, str(tmp_path / "ttyUSB9"))
    port = tmp_path / "ttyUSB1"
    port.write_text("")
    assert depth_camera.sim_scan_unreachable(bc, str(port)) == ""
    assert depth_camera.sim_scan_unreachable(bare("pico"), str(tmp_path / "none")) == ""


def test_every_entry_point_applies_the_rules():
    launch = open(os.path.join(ROOT, "launchers", "bringup.launch.py")).read()
    assert "depth_camera.sim_scan_unreachable(controller, lidar_port)" in launch
    assert "[RAW_SCAN OVER UART REFUSED]" in launch
    pipe = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()
    assert "depth_camera.raw_scan_over_uart(controller_cfg)" in pipe
    assert "depth_camera.sim_scan_unreachable(controller_cfg, lidar_port)" in pipe
    assert "args.topics_only = True" in pipe
    env = open(os.path.join(ROOT, "scripts", "mcu_env.py")).read()
    assert "depth_camera.raw_scan_over_uart(whole.get(\"base_controller\") or {})" in env
