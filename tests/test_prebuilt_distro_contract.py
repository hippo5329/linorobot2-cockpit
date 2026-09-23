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


def test_each_profile_header_matches_its_distro_cmd_vel_contract(tmp_path):
    import build_prebuilt
    from gen_firmware_header import distro_stamps_cmd_vel
    import cockpit_paths

    header = os.path.join(REPO_ROOT, "firmware", "include", "custom", "lino_base_config.h")
    saved = open(header).read() if os.path.exists(header) else None
    env = dict(os.environ)
    env.pop("ROS_DISTRO", None)                       # as on the release runner
    env["COCKPIT_CONFIG_DIR"] = str(tmp_path / "cfg")
    try:
        # The release images are built from the GENERATED bare module, not from
        # a reference config -- a released image describes no robot, because the
        # env partition is what turns it into one. Generate the same file
        # build_prebuilt does, so this checks what actually ships.
        import gen_bare_config
        import yaml as _yaml
        for profile, (mcu, _pio_env, distro, _desc) in build_prebuilt.PROFILES.items():
            cfg = str(tmp_path / f"bare_{mcu}_config.yaml")
            with open(cfg, "w") as fh:
                _yaml.safe_dump(gen_bare_config.bare_config(mcu), fh, sort_keys=False)
            subprocess.run(
                [sys.executable, os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py"),
                 "--params", cfg, "--distro", distro, "--no-embed-secrets"],
                check=True, capture_output=True, env=env)
            got = "#define USE_STAMPED_CMD_VEL" in open(header).read()
            want = distro_stamps_cmd_vel(distro)
            assert got == want, (
                f"profile {profile} is built for {distro}, which "
                f"{'stamps' if want else 'does not stamp'} /cmd_vel, but its header "
                f"{'defines' if got else 'does not define'} USE_STAMPED_CMD_VEL. A board "
                f"flashed with this image would not respond to nav2.")
    finally:
        if saved is not None:
            open(header, "w").write(saved)
