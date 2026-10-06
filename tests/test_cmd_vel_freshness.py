"""/cmd_vel freshness on a board: subscription QoS, the UDP read, and spin_all (2026-10-06).

Traced through rclc and rmw_microxrcedds: with RMW_UXRCE_MAX_HISTORY=1 the client holds ONE
received message, shared by every subscription; rclc's default subscription (reliable,
keep-last 10) then drops each command that arrives while one is buffered, keeping the
oldest. The UDP read tested the clock before the socket, so a zero-timeout read never read a
datagram. rclc_executor_spin_some takes one message per handle per call. Measured on the Sim
MCU at an ESP32's 8 Hz loop with 20 Hz /cmd_vel: step to motion 825 ms -> 645 ms.
"""
import json
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN = open(os.path.join(ROOT, "firmware", "src", "main.cpp")).read()


def test_cmd_vel_is_best_effort_keep_last_one():
    helper = MAIN[MAIN.index("static rcl_ret_t cmdSubscriptionInit("):MAIN.index("bool createEntities()\n{")]
    assert "qos.depth = 1;" in helper
    assert "RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT" in helper
    assert "RMW_QOS_POLICY_HISTORY_KEEP_LAST" in helper
    # every /cmd_vel subscription goes through it; none keeps rclc's default
    create = MAIN[MAIN.index("bool createEntities()\n{"):MAIN.index("bool destroyEntities()\n{")]
    assert "rclc_subscription_init_default" not in create
    assert create.count("cmdSubscriptionInit(") >= 2


def test_a_zero_timeout_udp_read_still_reads():
    src = open(os.path.join(ROOT, "firmware", "common", "lib", "uros_transport", "uros_transport.cpp")).read()
    udp = src[src.index("if (uros_use_udp) {", src.index("platformio_transport_read")):]
    udp = udp[:udp.index("return n;")]
    # the socket is asked before the clock is checked
    assert udp.index("parsePacket()") < udp.index(">= (int64_t)timeout")
    assert "while ((uxr_millis() - start) < (int64_t)timeout && uros_udp.parsePacket() == 0)" not in src


def test_spin_all_follows_spin_some_where_rclc_has_it():
    connected = MAIN[MAIN.index("case AGENT_CONNECTED:"):MAIN.index("case AGENT_DISCONNECTED:")]
    assert "rclc_executor_spin_some(executor, RCL_MS_TO_NS(100))" in connected
    assert "#ifdef RCLC_EXECUTOR_HAS_SPIN_ALL" in connected
    assert "rclc_executor_spin_all(executor, RCL_MS_TO_NS(SPIN_ALL_MAX_MS))" in connected


def test_rclc_is_upstream_everywhere():
    """spin_all was measured and dropped (2026-10-07): at MAX_HISTORY=1 there is one message
    to drain, so it changed nothing (Sim MCU 833 -> 823 ms), and history 4 + spin_all on the
    GenDrv over Wi-Fi was within noise (median 283 -> 264 ms, worse tail). The call stays
    behind RCLC_EXECUTOR_HAS_SPIN_ALL, dormant until upstream rclc has it; no fork is built."""
    repos = open(os.path.join(ROOT, "firmware", "host", "client.repos.txt")).read()
    line = next(l for l in repos.splitlines() if l.startswith("uros/rclc "))
    assert "github.com/ros2/rclc.git" in line and "spin-all" not in line
    ini = open(os.path.join(ROOT, "firmware", "common", "platformio_base.ini")).read()
    assert "#spin-all" not in ini
    unoq = open(os.path.join(ROOT, "firmware", "zephyr", "unoq", "build.sh")).read()
    assert "hippo5329/rclc" not in unoq


def test_board_metas_hold_one_input_buffer():
    for meta in ("esp32.meta", "atomic.meta"):
        text = "".join(l for l in open(os.path.join(ROOT, "firmware", meta)) if not l.lstrip().startswith("#"))
        args = json.loads(text)["names"]["rmw_microxrcedds"]["cmake-args"]
        assert "-DRMW_UXRCE_MAX_HISTORY=1" in args, meta
