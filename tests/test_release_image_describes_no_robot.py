"""A release image is built for a silicon, not for a robot.

Every robot fact -- pins, sensors, baud rate, kinematics, the simulated room,
the robot's own radius -- reaches a board through the env partition at flash
time. None of it may be compiled in, because one image is flashed to every
robot and an image that describes one of them is wrong for all the others.

This has leaked twice, both times through something that looked clean:

  2026-09-20  built from pico2_mecanum and gendrv directly, so two bare Picos
              came up printing "[range] HC-SR04 trigger=27 echo=28".
  2026-09-23  built from the GENERATED bare config, which sounds design-free
              and is not: gen_bare_config donates geometry/ekf/slam/nav2 from
              config/reference/gendrv_config.yaml, so that design's costmap
              radius shipped in every image as #define SIM_ROBOT_RADIUS.

So the test is not "does the build mention a reference" -- both leaks would
have passed that. It generates the header the release actually builds with and
asserts the design values are absent from it.
"""
import ast
import glob
import os
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import gen_firmware_header  # noqa: E402

MCUS = ("pico", "pico2", "esp32", "esp32s3")


def _bare_header(mcu, tmp_path):
    out = os.path.join(str(tmp_path), f"{mcu}.h")
    subprocess.run(
        [sys.executable, os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py"),
         "--mcu", mcu, "--distro", "jazzy", "--no-embed-secrets", "--out", out],
        check=True, capture_output=True, text=True)
    with open(out, encoding="utf-8") as fh:
        return fh.read()


def test_the_build_passes_no_config_file_at_all():
    """--mcu, never --params. The 2026-09-23 leak was a --params that pointed
    at a generated file, which is why this checks the flag and not the path."""
    src = open(os.path.join(REPO_ROOT, "scripts", "build_prebuilt.py"), encoding="utf-8").read()
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "header_cmd")
    flags = [n.value for n in ast.walk(fn) if isinstance(n, ast.Constant) and isinstance(n.value, str)]
    assert "--mcu" in flags, "the release header must be generated from the silicon"
    assert "--params" not in flags, "a release image may not be built from a config file"


def test_no_reference_design_value_reaches_the_image(tmp_path):
    """The concrete leak: a robot radius from someone else's costmap."""
    import yaml
    radii = set()
    for f in glob.glob(os.path.join(REPO_ROOT, "config", "reference", "*_config.yaml")):
        with open(f, encoding="utf-8") as fh:
            radii.add(f"{gen_firmware_header.nav2_robot_radius(yaml.safe_load(fh)):.4f}")
    assert radii, "no reference designs to check against"
    for mcu in MCUS:
        hdr = _bare_header(mcu, tmp_path)
        assert "SIM_ROBOT_RADIUS" not in hdr, (
            f"{mcu}: the image carries a robot's dimensions; it is flashed to every robot")
        for r in radii:
            assert r not in hdr, f"{mcu}: a reference design's radius {r} reached the image"


def test_the_radius_reaches_the_board_as_an_env_key_instead(tmp_path):
    """Removing it from the image is only safe because the env carries it --
    the emulator's own 0.30f fallback against a config planning with 0.26 m is
    the lethal-cell failure that started all this."""
    import mcu_env
    secrets = os.path.join(REPO_ROOT, "config", "secrets.yaml.example")
    for f in sorted(glob.glob(os.path.join(REPO_ROOT, "config", "reference", "*_config.yaml"))):
        env = mcu_env.env_from_config(f, secrets, "192.0.2.1")
        assert "sim_radius" in env, os.path.basename(f)
        assert float(env["sim_radius"]) > 0, os.path.basename(f)


def test_the_emulator_reads_that_key_rather_than_the_macro():
    src = open(os.path.join(REPO_ROOT, "firmware", "common", "lib", "lidar", "sim_ld19.h"),
               encoding="utf-8").read()
    assert 'robot_radius_ = envFloat("sim_radius", robot_radius_);' in src
    # The macro survives as the member's initialiser -- the fallback for a
    # board with a blank env -- and nowhere else: every use of the radius must
    # read the member, or the env key silently does nothing.
    decl = 'float robot_radius_ = (float)SIM_ROBOT_RADIUS;'
    assert decl in src
    assert src.count("SIM_ROBOT_RADIUS") == 3, (
        "expected the #ifndef, the #define and the initialiser only; "
        "a clamp still reads the macro")


def test_the_bare_params_describe_silicon_only():
    """No ekf/slam/nav2/geometry -- those are a robot, and none of them belong
    to an MCU. This is what makes the header above design-free."""
    for mcu in MCUS:
        params = gen_firmware_header.bare_mcu_params(mcu)
        assert set(params) <= {"robot", "base_controller", "kinematics"}, (mcu, sorted(params))
        pins = params["base_controller"]["pins"]
        for name, pin in pins.items():
            if name == "led":
                continue                      # the board's own, and an MCU fact
            values = pin.values() if isinstance(pin, dict) else [pin]
            for v in values:
                assert v in (-1, False), f"{mcu}: {name} claims a wire ({v})"
