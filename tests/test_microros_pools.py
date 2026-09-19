"""The micro-ROS entity pools must cover what the firmware can actually create.

rmw_microxrcedds sizes its entity storage STATICALLY at library-build time, so
every slot is .bss whether or not anything uses it. firmware/*.meta trims those
pools from micro_ros_platformio's defaults, which matters because esp32_lyrical
misses dram0_0_seg by 14504 bytes and the smaller boards have no use for the
slack either.

The trim has a sharp edge, and that is what these tests guard. Running out of
publisher slots is not a build error: rclc_publisher_init returns non-zero,
RCCHECK reboots the board, and you get a boot loop on whichever robot config
happened to enable the tenth publisher. Nine of the ten publishers are behind
#ifdefs, so the maximum is a property of the SOURCE, not of any one config --
which is why these read main.cpp and compare, instead of pinning a number
somebody would later "optimise".
"""
import json
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE = os.path.join(REPO_ROOT, "firmware")
METAS = ("esp32.meta", "atomic.meta")


def _pools(name):
    """The -DRMW_UXRCE_* settings from a user meta, as {KEY: value}."""
    raw = open(os.path.join(FIRMWARE, name)).read()
    body = "\n".join(l for l in raw.splitlines() if not l.lstrip().startswith("#"))
    args = json.loads(body)["names"]["rmw_microxrcedds"]["cmake-args"]
    out = {}
    for a in args:
        if a.startswith("-DRMW_UXRCE_") and "=" in a:
            k, v = a[len("-DRMW_UXRCE_"):].split("=", 1)
            out[k] = v
    return out


def _declared(kind):
    """File-scope `rcl_<kind>_t name;` declarations in main.cpp -- the true maximum."""
    src = open(os.path.join(FIRMWARE, "src", "main.cpp")).read()
    return len(re.findall(rf"^rcl_{kind}_t\s+\w+\s*;", src, re.M))


def test_every_board_meta_declares_the_pools():
    for name in METAS:
        p = _pools(name)
        for key in ("MAX_NODES", "MAX_PUBLISHERS", "MAX_SUBSCRIPTIONS",
                    "MAX_SERVICES", "MAX_CLIENTS", "MAX_HISTORY"):
            assert key in p, f"{name} does not set RMW_UXRCE_{key}"


def test_the_two_metas_agree():
    """esp32.meta covers esp32/esp32s3 and atomic.meta the pico family; a board
    that silently got different pools would be a link that fits on one bench
    and not the other."""
    a, b = (_pools(n) for n in METAS)
    assert a == b, f"{METAS[0]} and {METAS[1]} disagree: {a} vs {b}"


def test_publisher_pool_covers_every_declared_publisher():
    declared = _declared("publisher")
    assert declared >= 10, "main.cpp should declare ten publishers; did one go away?"
    for name in METAS:
        got = int(_pools(name)["MAX_PUBLISHERS"])
        assert got >= declared, (
            f"{name}: MAX_PUBLISHERS={got} but main.cpp declares {declared}. Nine of them "
            f"are conditional, so a config CAN enable them all, and the overrun is a "
            f"runtime RCCHECK reboot, not a build failure.")


def test_subscription_pool_covers_every_declared_subscription():
    declared = _declared("subscription")
    for name in METAS:
        got = int(_pools(name)["MAX_SUBSCRIPTIONS"])
        assert got >= declared, f"{name}: MAX_SUBSCRIPTIONS={got} < {declared} declared"


def test_services_and_clients_stay_at_zero_while_none_are_created():
    """Zero is only correct while the firmware creates neither. If a parameter
    server or any service ever appears, these pools have to grow with it."""
    src = open(os.path.join(FIRMWARE, "src", "main.cpp")).read()
    uses_services = bool(re.search(r"rclc_service_init|rclc_parameter_server_init", src))
    uses_clients = bool(re.search(r"rclc_client_init", src))
    for name in METAS:
        p = _pools(name)
        if not uses_services:
            assert p["MAX_SERVICES"] == "0", f"{name}: no services are created; pool should be 0"
        else:
            assert int(p["MAX_SERVICES"]) >= 1, f"{name}: firmware creates services but the pool is 0"
        if not uses_clients:
            assert p["MAX_CLIENTS"] == "0", f"{name}: no clients are created; pool should be 0"
        else:
            assert int(p["MAX_CLIENTS"]) >= 1, f"{name}: firmware creates clients but the pool is 0"


def test_mtu_survives_the_trim():
    """A best-effort XRCE stream cannot fragment, and nav_msgs/Odometry is ~720
    bytes: the 1024 MTU is why /odom/unfiltered works at all. Trimming history
    must not have taken the MTU with it."""
    for name in METAS:
        raw = open(os.path.join(FIRMWARE, name)).read()
        body = "\n".join(l for l in raw.splitlines() if not l.lstrip().startswith("#"))
        args = json.loads(body)["names"]["microxrcedds_client"]["cmake-args"]
        mtu = [int(a.split("=")[1]) for a in args if "UCLIENT_CUSTOM_TRANSPORT_MTU" in a]
        assert mtu and mtu[0] >= 1024, f"{name}: transport MTU must stay >= 1024, got {mtu}"
