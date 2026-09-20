#!/usr/bin/env python3
"""
verify_nav2_params.py — Validates Nav2 configuration adaptation for both Jazzy and Lyrical.
"""

import copy
import importlib.util
import os
import sys
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(REPO_ROOT, "launchers", "nav2.launch.py")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cockpit_paths  # noqa: E402
try:
    import launch  # noqa: F401  the launch file under test imports it
except ImportError:
    sys.exit("verify_nav2_params.py needs a sourced ROS 2 (python module 'launch' not found); "
             "run it on the robot computer or in the robot container.")
CONFIG_DIR = cockpit_paths.ensure_config_dir(quiet=True)


def robot_configs():
    """Every per-robot config; one robot per file, one base controller each."""
    return [
        os.path.join(CONFIG_DIR, f)
        for f in sorted(os.listdir(CONFIG_DIR))
        if f.endswith("_config.yaml") and not f.startswith("secrets")
    ]


CONFIG_FILE = (robot_configs() or [os.path.join(CONFIG_DIR, "pico2_mecanum_config.yaml")])[0]


class DummyContext:
    def __init__(self, distro="jazzy"):
        self.launch_configurations = {
            "config_file": CONFIG_FILE,
            "distro": distro,
            "use_sim_time": "false",
            "autostart": "true",
            "map": "",
        }


def main():
    if not os.path.isfile(CONFIG_FILE):
        print(f"Error: Config file not found: {CONFIG_FILE}", file=sys.stderr)
        return 1

    with open(CONFIG_FILE, "r") as f:
        cfg = yaml.safe_load(f)

    assert "nav2" in cfg, f"Missing 'nav2' section in {os.path.basename(CONFIG_FILE)}"

    spec = importlib.util.spec_from_file_location("nav2_launcher", LAUNCH_FILE)
    nav2_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(nav2_mod)

    def get_nav2_params(actions):
        for k, v in actions[1].launch_arguments:
            if k == "params_file":
                with open(v, "r") as f:
                    return yaml.safe_load(f)
        raise RuntimeError("params_file not found in launch arguments")

    # 1. Test Jazzy default: enable_stamped_cmd_vel must be False (Jazzy Nav2 default)
    ctx_j = DummyContext("jazzy")
    actions_j = nav2_mod.launch_setup(ctx_j)
    params_j = get_nav2_params(actions_j)
    assert params_j["controller_server"]["ros__parameters"]["enable_stamped_cmd_vel"] is False, "Jazzy should default to enable_stamped_cmd_vel: false"
    assert params_j["velocity_smoother"]["ros__parameters"]["enable_stamped_cmd_vel"] is False

    # 2. Test Lyrical default: enable_stamped_cmd_vel must be True (Lyrical Nav2 default)
    ctx_l = DummyContext("lyrical")
    actions_l = nav2_mod.launch_setup(ctx_l)
    params_l = get_nav2_params(actions_l)
    assert params_l["controller_server"]["ros__parameters"]["enable_stamped_cmd_vel"] is True, "Lyrical should default to enable_stamped_cmd_vel: true"
    assert params_l["velocity_smoother"]["ros__parameters"]["enable_stamped_cmd_vel"] is True

    # 3. Test explicit CLI override on Jazzy: stamped_cmd_vel:=true
    ctx_j_override = DummyContext("jazzy")
    ctx_j_override.launch_configurations["stamped_cmd_vel"] = "true"
    actions_jo = nav2_mod.launch_setup(ctx_j_override)
    params_jo = get_nav2_params(actions_jo)
    assert params_jo["controller_server"]["ros__parameters"]["enable_stamped_cmd_vel"] is True, "Explicit override on Jazzy should enable stamped_cmd_vel"

    # 4. Test explicit CLI override on Lyrical: stamped_cmd_vel:=false
    ctx_l_override = DummyContext("lyrical")
    ctx_l_override.launch_configurations["stamped_cmd_vel"] = "false"
    actions_lo = nav2_mod.launch_setup(ctx_l_override)
    params_lo = get_nav2_params(actions_lo)
    assert params_lo["controller_server"]["ros__parameters"]["enable_stamped_cmd_vel"] is False, "Explicit override on Lyrical should disable stamped_cmd_vel"

    print("PASS: Nav2 distro adaptation cleanly verified (Jazzy: unstamped Twist, Lyrical: stamped TwistStamped).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
