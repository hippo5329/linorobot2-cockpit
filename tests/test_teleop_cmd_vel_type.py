"""Teleop has to speak the contract the firmware was built to.

The boundary is KILTED, not lyrical: nav2 1.4 (kilted) flipped
nav2_util::TwistPublisher to TwistStamped, so jazzy and older drive /cmd_vel
plain and kilted and everything after it drive it stamped.
`stamped_cmd_vel: auto` follows that and the firmware follows `auto` -- with
USE_STAMPED_CMD_VEL it subscribes TwistStamped on /cmd_vel and moves the plain
Twist subscriber to **/cmd_vel_unstamped**. Nothing outside
firmware/src/main.cpp had ever heard of that topic.

So both teleop paths published plain Twist on /cmd_vel unconditionally, which
on any post-kilted robot is the wrong type on the right topic: dropped by the
middleware, with nothing logged on either side. The gamepad moved and the base
did not. Nav2 was unaffected -- it stamps -- so the matrix stayed green while
teleop was dead on every distro from kilted on.
"""
import os
import sys
import types

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))

import actions  # noqa: E402


def _load_gamepad():
    stubs = {
        "rclpy": ["init", "spin", "shutdown"],
        "rclpy.node": ["Node"],
        "geometry_msgs": [],
        "geometry_msgs.msg": ["Twist", "TwistStamped"],
    }
    saved = {n: sys.modules.get(n) for n in stubs}
    for n, attrs in stubs.items():
        m = types.ModuleType(n)
        for a in attrs:
            setattr(m, a, type(a, (object,), {}))
        sys.modules[n] = m
    try:
        sys.modules.pop("gamepad_publisher", None)
        import gamepad_publisher
        return gamepad_publisher
    finally:
        for n, m in saved.items():
            if m is None:
                sys.modules.pop(n, None)
            else:
                sys.modules[n] = m


GP = _load_gamepad()


def test_auto_follows_the_distro_that_built_the_firmware():
    """Kilted is the boundary, and it is the one that must not be missed.

    lyrical passing proves nothing on its own -- an allow-list naming only
    lyrical would drop kilted back to the jazzy contract in silence.
    """
    for d in ("kilted", "lyrical"):
        assert GP.resolve_cmd_vel_type("auto", d) is True, d
    for d in ("jazzy", "humble", "iron"):
        assert GP.resolve_cmd_vel_type("auto", d) is False, d


def test_a_distro_after_lyrical_stays_stamped():
    """The rule is "newer than the last unstamped release", not a list of
    stamped ones, so the release after lyrical does not silently fall back."""
    assert GP.resolve_cmd_vel_type("auto", "zesty") is True


def test_an_unknown_or_missing_distro_stays_on_the_older_contract():
    """Guessing stamped on an unknown distro would break every jazzy robot."""
    assert GP.resolve_cmd_vel_type("auto", "") is False


def test_the_type_can_be_forced_either_way():
    assert GP.resolve_cmd_vel_type("twist", "lyrical") is False
    assert GP.resolve_cmd_vel_type("twist_stamped", "jazzy") is True


def test_the_rule_is_imported_not_copied():
    """One list of unstamped distros, in the file that decides the #define.

    A second copy would disagree on exactly the distro nobody tested.
    """
    src = open(os.path.join(SCRIPTS, "gamepad_publisher.py"), encoding="utf-8").read()
    assert "from gen_firmware_header import distro_stamps_cmd_vel" in src
    # the list itself lives in exactly one file, and it is not this one
    assert "UNSTAMPED_CMD_VEL_DISTROS =" not in src
    gen = open(os.path.join(SCRIPTS, "gen_firmware_header.py"), encoding="utf-8").read()
    assert "UNSTAMPED_CMD_VEL_DISTROS =" in gen


def test_the_joystick_path_stamps_on_lyrical_too():
    for d in ("kilted", "lyrical"):
        assert "publish_stamped_twist: true" in actions.build("teleop", {"distro": d}), d
    cmd = actions.build("teleop", {"distro": "jazzy"})
    assert "publish_stamped_twist: false" in cmd


def test_the_stop_on_shutdown_is_the_same_type_as_the_driving():
    """The final zero has to be heard too, or the base coasts on the last
    command it did hear."""
    src = open(os.path.join(SCRIPTS, "gamepad_publisher.py"), encoding="utf-8").read()
    assert "node.pub.publish(node._msg(0.0, 0.0, 0.0))" in src
    assert "node.pub.publish(Twist())" not in src
