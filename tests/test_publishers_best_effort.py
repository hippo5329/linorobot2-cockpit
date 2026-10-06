"""Every publisher on the board takes the env's QoS, best effort by default (2026-10-07).

A reliable rmw_publish blocks the board's loop until the agent acknowledges the message, up
to RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT (1 s). On the S3's USB link one lost frame stopped
/odom for 1.007 s, through the reliable battery and sonar publishers, dozens of times per
Nav2 run. Each topic republishes its state, and every host consumer subscribes with
sensor-data QoS, so a lost message is replaced by the next one.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN = open(os.path.join(ROOT, "firmware", "src", "main.cpp")).read()
CREATE = MAIN[MAIN.index("bool createEntities()\n{"):MAIN.index("bool destroyEntities()\n{")]


def test_no_publisher_keeps_rclcs_reliable_default():
    # only the env opt-out (best_effort=0, for a consumer that insists) may name it
    switch = "best_effort ? rclc_publisher_init_best_effort : rclc_publisher_init_default"
    assert "rclc_publisher_init_default" not in CREATE.replace(switch, "")


def test_every_publisher_goes_through_the_env_switch_or_is_best_effort():
    calls = re.findall(r"RCCHECK\((\w+)\(\s*&(\w+_publisher)", CREATE)
    pubs = {p for _, p in calls}
    for p in ("odom_publisher", "imu_publisher", "mag_publisher", "battery_publisher",
              "safety_stop_publisher", "range_publisher", "pressure_publisher",
              "temperature_publisher", "humidity_publisher", "raw_scan_publisher"):
        assert p in pubs, p
    for fn, p in calls:
        assert fn in ("init_pub", "rclc_publisher_init_best_effort"), (fn, p)
    assert "best_effort ? rclc_publisher_init_best_effort : rclc_publisher_init_default" in CREATE


def test_best_effort_is_the_default():
    assert re.search(r'best_effort = envFlag\("best_effort", true\);', MAIN)


def test_safety_stop_is_republished_every_pass():
    # best effort is only safe for a state that is sent again: not on change alone
    i = MAIN.index("rcl_publish(&safety_stop_publisher")
    assert "changed" not in MAIN[i - 200:i]
