"""A prebuilt image must be built for the /cmd_vel contract of its own distro.

Every reference config says `stamped_cmd_vel: auto`, and gen_firmware_header.py
resolves `auto` from its --distro flag, which defaults to $ROS_DISTRO. The
release runner that builds the prebuilt images is a bare ubuntu-24.04 with pip
platformio and no ROS at all, so $ROS_DISTRO is empty there and `auto` resolved
to False for every profile -- including the four -lyrical ones.

The result was an image that passes every check a build can make. It links, it
flashes, it enumerates with the agent, it publishes odometry at 50 Hz. It simply
subscribes to geometry_msgs/Twist while nav2 >= kilted publishes /cmd_vel as
TwistStamped, so the robot never moves under autonomy and nothing anywhere says
why. The pio env (esp32_lyrical vs esp32) selects which micro-ROS library is
linked; it has no bearing on this, which is why the mismatch was survivable all
the way to a published release.

The first test pins the call. The second is the one that would have caught it:
it generates the header the way build_prebuilt.py does and asserts the macro
follows the profile's distro.
"""
import os
import re
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))


def test_the_header_command_names_each_profile_own_distro():
    """Not `--distro` appearing somewhere: the VALUE, per profile. Checking only that
    the flag is present passes just as happily when it is hardcoded to one distro,
    which is the same bug with an extra step."""
    import build_prebuilt
    for profile, (_cfg_stem, _env, distro, _desc) in build_prebuilt.PROFILES.items():
        cmd = build_prebuilt.header_cmd("pico2", distro)
        assert "--distro" in cmd, f"{profile}: header_cmd() dropped --distro"
        assert cmd[cmd.index("--distro") + 1] == distro, (
            f"{profile} is built for {distro} but header_cmd() asks for "
            f"{cmd[cmd.index('--distro') + 1]!r}.")
        assert "--no-embed-secrets" in cmd, f"{profile}: a prebuilt image must carry no keys"


def test_the_builder_uses_that_command_rather_than_its_own():
    """header_cmd() is only a guard if build() actually calls it."""
    src = open(os.path.join(REPO_ROOT, "scripts", "build_prebuilt.py")).read()
    body = src[src.index("def build(profile"):]
    assert "header_cmd(mcu, distro)" in body, (
        "build() no longer calls header_cmd(mcu, distro); the tests above are then "
        "checking a function nothing runs.")
    assert "gen_firmware_header.py" not in body, (
        "build() invokes gen_firmware_header.py directly again, bypassing header_cmd().")


def test_each_profile_image_defaults_to_its_distro_cmd_vel_contract():
    """Each release image must answer /cmd_vel with its distro's type.

    It was USE_STAMPED_CMD_VEL in the generated header. Now the firmware picks
    the type at boot from FW_ROS_DISTRO (the distro its image was linked for)
    and the env overrides it -- so what must hold is that the firmware's list of
    unstamped distros IS the host's, for every profile that ships.
    """
    import build_prebuilt
    from gen_firmware_header import distro_stamps_cmd_vel, UNSTAMPED_CMD_VEL_DISTROS
    main = open(os.path.join(REPO_ROOT, "firmware", "src", "main.cpp")).read()
    m = re.search(r"const bool unstamped = ([^;]+);", main)
    assert m, "main.cpp no longer derives the /cmd_vel type from its distro"
    fw_unstamped = set(re.findall(r'!strcmp\(d, "([a-z]+)"\)', m.group(1)))
    assert fw_unstamped == set(UNSTAMPED_CMD_VEL_DISTROS), (fw_unstamped, UNSTAMPED_CMD_VEL_DISTROS)
    for profile, (_mcu, _pio_env, distro, _desc) in build_prebuilt.PROFILES.items():
        assert (distro not in fw_unstamped) == distro_stamps_cmd_vel(distro), profile
