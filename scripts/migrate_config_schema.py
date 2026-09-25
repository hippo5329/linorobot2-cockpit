#!/usr/bin/env python3
# ==============================================================================
# migrate_config_schema.py — one config file per robot, one controller per file
#
# Old shape (multi-controller, indexed):
#     robot:
#       active_target: pico2
#     targets:
#       pico2: {mcu: ..., pins: ...}
#       gendrv: {...}
#
# New shape (single controller, flat):
#     robot:
#       name: pico2_mecanum
#     base_controller:
#       name: pico2          # firmware variant == PlatformIO env
#       mcu: pico2
#       pins: {...}
#
# "target" is retired as a noun: it reads as a navigation goal, while what it
# actually names is the linorobot2_hardware base controller. With one robot per
# file there is nothing left to index, so active_target has nothing to select
# and the targets: layer has nothing to hold.
#
# A multi-controller file is split into one file per controller, each a robot of
# its own, so the header's robot dropdown lists them all.
# ==============================================================================

import argparse
import os
import re
import sys
from typing import Any, Dict, List

try:
    import yaml
except ImportError:
    print("PyYAML is required: sudo apt install -y python3-yaml", file=sys.stderr)
    sys.exit(1)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cockpit_paths  # noqa: E402
CONFIG_DIR = cockpit_paths.config_dir()

# Everything that is not the controller block is robot-wide and is copied as-is
# into each file a split produces.
SHARED_SECTIONS = ["kinematics", "ekf", "slam", "nav2", "ros_distro", "agent", "laser", "network"]


def is_legacy(params: Dict[str, Any]) -> bool:
    return "targets" in params or "active_target" in params.get("robot", {})


def build_controller(name: str, block: Dict[str, Any]) -> Dict[str, Any]:
    """Flatten one targets[<name>] entry into a base_controller block."""
    controller: Dict[str, Any] = {"name": name}
    # description first if present, then the rest in the original order
    if "description" in block:
        controller["description"] = block["description"]
    for key, value in block.items():
        if key == "description":
            continue
        controller[key] = value
    return controller


def convert(params: Dict[str, Any], controller_name: str, robot_name: str) -> Dict[str, Any]:
    robot = dict(params.get("robot", {}))
    robot.pop("active_target", None)
    robot.pop("target", None)
    robot["name"] = robot_name

    out: Dict[str, Any] = {"robot": robot}
    out["base_controller"] = build_controller(controller_name, params.get("targets", {}).get(controller_name, {}))
    for section in SHARED_SECTIONS:
        if section in params:
            out[section] = params[section]
    # anything unexpected survives the migration rather than being dropped
    for key, value in params.items():
        if key not in out and key not in ("targets", "robot"):
            out[key] = value
    return out


def write_yaml(path: str, data: Dict[str, Any], header: str) -> None:
    with open(path, "w") as fh:
        fh.write(header)
        yaml.safe_dump(data, fh, sort_keys=False, default_flow_style=False, width=100)


# ------------------------------------------------------------------ retunes
# Schema migrations move keys. A retune changes a VALUE in a config that is
# already the right shape. These are textual on purpose: a robot config lives
# in the user's own git repo, and yaml.safe_dump would take every comment with
# it.
#
# There is no dual-core retune any more. One flipped `use_dual_core: true` to
# false (the critical section it installs disables interrupts on the Wi-Fi
# core), and it kept flipping every run -- including the shipped `true` that
# the serial ESP32 references carry now that dual core is the measured win for
# a serial robot (43 -> 49 Hz) and the firmware itself refuses it when the
# radio is on (`[core] dual_core=1 ignored`). The firmware holds that rule;
# a migration that overrides the user's value every time it runs is not one.
TOP_KEY_RE = re.compile(r"^[A-Za-z_]")
DEAD_KEY_RE = re.compile(r"^\s*(motor_power_measured_voltage|backend_host|backend_port|frontend_port):")


def add_geometry(path: str, dry_run: bool) -> bool:
    """A config from before `geometry:` gets one derived from its kinematics.

    The URDF is generated from the config now (scripts/gen_robot_description.py),
    and the derivation is written into the file rather than applied silently at
    every launch, so the numbers are the user's to see and correct. Inserted as
    text after the kinematics block, so comments survive. Dead keys that nothing
    ever read (motor_power_measured_voltage; the supervisor's host and ports,
    which docker-compose and --port own) are dropped on the way.
    """
    with open(path) as fh:
        lines = fh.readlines()
    params = yaml.safe_load("".join(lines)) or {}
    hit = False
    if "geometry" not in params and "kinematics" in params:
        import gen_robot_description
        g = gen_robot_description.effective_geometry(params)
        base = str(params["kinematics"].get("base_type", "2wd")).lower()
        block = [
            "# The robot's body, for the generated URDF (scripts/gen_robot_description.py). Derived\n",
            "# from the kinematics by migrate_config_schema.py; metres, kilograms, radians. Correct it.\n",
            "geometry:\n",
            f"  body: {{length: {g['body']['length']}, width: {g['body']['width']}, height: {g['body']['height']}, mass: {g['body']['mass']}}}\n",
            f"  wheel: {{width: {g['wheel']['width']}, mass: {g['wheel']['mass']}, z: {g['wheel']['z']}}}\n",
        ]
        if base == "2wd":
            block.append("  casters: {front: true, rear: true}\n")
        block += [
            f"  laser: {{x: {g['laser']['x']}, y: {g['laser']['y']}, z: {g['laser']['z']}, roll: 0.0, pitch: 0.0, yaw: 0.0, frame: laser}}\n",
            "  imu: {x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}\n",
            "  mesh: {base: '', wheel: ''}\n",
        ]
        k = next(n for n, ln in enumerate(lines) if ln.startswith("kinematics:"))
        end = next((n for n in range(k + 1, len(lines)) if TOP_KEY_RE.match(lines[n])), len(lines))
        lines[end:end] = block
        print(f"  {os.path.basename(path)}: geometry block added (derived from kinematics)")
        hit = True
    kept = [ln for ln in lines if not DEAD_KEY_RE.match(ln)]
    if len(kept) != len(lines):
        print(f"  {os.path.basename(path)}: {len(lines) - len(kept)} dead key(s) removed")
        lines, hit = kept, True
    if hit and not dry_run:
        with open(path, "w") as fh:
            fh.writelines(lines)
    return hit


FAKE_KEY_RE = re.compile(r"\buse_fake_([A-Za-z0-9_]+)(\s*):")


def rename_fake_keys(path: str, dry_run: bool) -> bool:
    """use_fake_* -> use_sim_*, the 2026-09-24 rename, in place and as TEXT.

    mcu_env.py refuses a config that still says use_fake_* -- an absent use_sim_*
    flag is a compiled-in default, so reading it would flash a bare module to
    expect hardware it does not have -- and said "rename them". This migrator is
    the tool for renames, and did not know this one: on 2026-09-25 a stale
    pico_config.yaml stopped the UI's 1-Click at "Flashing or verification
    failed". Only KEYS are rewritten; comments and layout survive.
    """
    with open(path) as fh:
        text = fh.read()
    new, n = FAKE_KEY_RE.subn(r"use_sim_\1\2:", text)
    if not n:
        return False
    print(f"  {os.path.basename(path)}: {n} use_fake_* key(s) -> use_sim_*")
    if not dry_run:
        with open(path, "w") as fh:
            fh.write(new)
    return True


# Values that were right once and now break a run without saying so. Each is
# rewritten as TEXT (comments survive) and refused by the pipeline until it is.
# Measured 2026-09-25 on the UI's Start 1-Click, against configs from before
# these conventions:
#   * ekf base_link_frame: base_footprint -- the EKF parents a frame the URDF
#     already parents, the tree splits, Nav2's controller never configures.
#   * imu0_remove_gravitational_acceleration: true -- the board removes gravity
#     since the AHRS moved on board; a second subtraction fabricates 9.81 m/s2.
#   * sensors.imu/mag: FAKE -- the pre-rename name of SIM. Outside --mode sim it
#     reaches the firmware as an unknown driver.
STALE_VALUE_RULES = [
    (re.compile(r"^(\s*base_link_frame:\s*)(['\"]?)(?!base_link\b)[A-Za-z0-9_/]+\2(\s*(#.*)?)$", re.M),
     r"\1base_link\3", "ekf base_link_frame -> base_link"),
    (re.compile(r"^(\s*imu0_remove_gravitational_acceleration:\s*)(true|True|TRUE|yes)(\s*(#.*)?)$", re.M),
     r"\1false\3", "imu0_remove_gravitational_acceleration -> false"),
    (re.compile(r"^(\s*(?:imu|mag):\s*)(['\"]?)FAKE\2(\s*(#.*)?)$", re.M | re.I),
     r"\1SIM\3", "sensors FAKE -> SIM"),
]


def _ekf_params(params: Dict[str, Any]) -> Dict[str, Any]:
    ekf = params.get("ekf") or {}
    return ((ekf.get("ekf_filter_node") or {}).get("ros__parameters")) or ekf


def _all_keys(node):
    if isinstance(node, dict):
        for k, v in node.items():
            yield str(k)
            yield from _all_keys(v)
    elif isinstance(node, list):
        for item in node:
            yield from _all_keys(item)


def stale_faults(params: Dict[str, Any]) -> List[str]:
    """What in this config breaks a run today, one line each; [] when nothing."""
    faults = []
    fake = sorted({k for k in _all_keys(params) if k.startswith("use_fake_")})
    if fake:
        faults.append(f"{', '.join(fake)}: renamed use_sim_* on 2026-09-24")
    ekf = _ekf_params(params)
    frame = ekf.get("base_link_frame")
    if frame is not None and str(frame) != "base_link":
        faults.append(f"ekf base_link_frame is {frame!r}: the EKF must publish odom -> base_link, "
                      f"or the TF tree splits and Nav2 never configures")
    if ekf.get("imu0_remove_gravitational_acceleration") is True:
        faults.append("ekf imu0_remove_gravitational_acceleration is true: the board already "
                      "removes gravity, a second subtraction fabricates 9.81 m/s2")
    sensors = ((params.get("base_controller") or {}).get("sensors")) or {}
    for field in ("imu", "mag"):
        if str(sensors.get(field, "")).strip().upper() == "FAKE":
            faults.append(f"sensors.{field} is FAKE: renamed SIM")
    return faults


def fix_stale_values(path: str, dry_run: bool) -> bool:
    with open(path) as fh:
        text = fh.read()
    new, done = text, []
    for rx, repl, what in STALE_VALUE_RULES:
        new, n = rx.subn(repl, new)
        if n:
            done.append(f"{what} ({n}x)" if n > 1 else what)
    if not done:
        return False
    print(f"  {os.path.basename(path)}: " + "; ".join(done))
    if not dry_run:
        with open(path, "w") as fh:
            fh.write(new)
    return True


def migrate_file(path: str, dry_run: bool) -> List[str]:
    renamed = rename_fake_keys(path, dry_run)
    renamed = fix_stale_values(path, dry_run) or renamed
    with open(path) as fh:
        params = yaml.safe_load(fh) or {}
    if renamed and dry_run:
        # a dry run did not write the rename, so read the keys as they will be
        params = yaml.safe_load(FAKE_KEY_RE.sub(r"use_sim_\1\2:", open(path).read())) or {}

    if not is_legacy(params):
        if add_geometry(path, dry_run) or renamed:
            return [path]
        print(f"  {os.path.basename(path)}: already migrated, skipped")
        return []

    targets = params.get("targets", {}) or {}
    active = params.get("robot", {}).get("active_target") or params.get("robot", {}).get("target")
    robot_name = params.get("robot", {}).get("name", "linorobot2")
    written: List[str] = []

    if len(targets) <= 1:
        name = (list(targets.keys()) or [active or "pico2"])[0]
        out = convert(params, name, robot_name)
        header = (
            f"# {robot_name} — single source of truth for this robot.\n"
            f"# One robot per file, one base controller per robot.\n"
        )
        print(f"  {os.path.basename(path)}: targets.{name} -> base_controller (in place)")
        if not dry_run:
            write_yaml(path, out, header)
            add_geometry(path, dry_run)
        written.append(path)
        return written

    # Multi-controller file: one robot per controller. The controller that was
    # active keeps the original robot name; the rest are named after themselves.
    for name in targets:
        new_robot = robot_name if name == active else name
        dest = os.path.join(CONFIG_DIR, f"{new_robot}_config.yaml")
        if os.path.exists(dest):
            print(f"  skip {os.path.basename(dest)}: already exists")
            continue
        out = convert(params, name, new_robot)
        desc = targets[name].get("description", "")
        header = (
            f"# {new_robot} — single source of truth for this robot.\n"
            f"# One robot per file, one base controller per robot.\n"
            + (f"# {desc}\n" if desc else "")
        )
        print(f"  {os.path.basename(path)}: targets.{name} -> {os.path.basename(dest)}")
        if not dry_run:
            write_yaml(dest, out, header)
            add_geometry(dest, dry_run)
        written.append(dest)
    return written


def main() -> int:
    ap = argparse.ArgumentParser(description="Migrate robot configs to the one-controller-per-file schema.")
    ap.add_argument("files", nargs="*", help="config YAMLs (default: every <config dir>/*.yaml except secrets)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    files = args.files
    if not files:
        files = [
            os.path.join(CONFIG_DIR, f)
            for f in sorted(os.listdir(CONFIG_DIR))
            if f.endswith((".yaml", ".yml")) and not f.startswith("secrets")
        ]

    print(f"Migrating {len(files)} config file(s){' (dry run)' if args.dry_run else ''}:")
    total = []
    for path in files:
        total.extend(migrate_file(path, args.dry_run))
    verb = "would be changed" if args.dry_run else "written"
    print(f"\n{len(total)} file(s) {verb}.")
    # The exit code is the answer, so no caller has to read the prose. CI used
    # to decide by grepping the output for "would" -- which this summary line
    # contains even when nothing would change ("0 file(s) would be changed."),
    # so the step could never pass and the whole job had been red since the
    # release. A dry run that found pending changes is a failure; doing the
    # migration is not.
    return 1 if (args.dry_run and total) else 0


if __name__ == "__main__":
    sys.exit(main())
