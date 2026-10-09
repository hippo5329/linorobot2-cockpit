"""One DDS domain per robot: the board, the Sim MCU and every stack process agree.

The micro-ROS agent puts a client's participant in the domain the CLIENT asks for
and never reads ROS_DOMAIN_ID (only a request for 255 is redirected, to
XRCE_DOMAIN_ID_OVERRIDE). So the board's domain is whatever its env says, and a
board flashed with 0 under a stack started on another domain shows nothing at all.
Measured on Jazzy and Lyrical alike: a client asking for 73 publishes on 73 with the
agent on 0; two clients on one agent in domains 5 and 6 each see only their own
robot; 232 works and 233 is refused by the agent ("Calculated port number is too
high").

`base_controller.domain_id` decides for a robot; a robot that names none takes the
process's ROS_DOMAIN_ID, then 0 -- for the flash (mcu_env), the Sim MCU
(host_firmware), the 1-Click (one_click_pipeline) and every command the Web UI
starts (runners.ros_setup_shell).
"""
import os

import pytest
import yaml

import cockpit_paths
import mcu_env

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read(*parts):
    with open(os.path.join(REPO, *parts), encoding="utf-8") as fh:
        return fh.read()


@pytest.mark.parametrize("raw,want", [
    (0, 0), (7, 7), ("42", 42), (" 9 ", 9), ('"12"', 12), (232, 232),
    (233, None), (-1, None), ("-1", None), ("", None), (None, None),
    ("abc", None), ("1.5", None), (True, None), (False, None),
])
def test_valid_domain_id(raw, want):
    assert cockpit_paths.valid_domain_id(raw) == want


def test_the_robot_decides_then_the_environment_then_zero():
    rd = cockpit_paths.robot_domain_id
    assert rd({"base_controller": {"domain_id": 12}}, {"ROS_DOMAIN_ID": "75"}) == 12
    assert rd({"base_controller": {}}, {"ROS_DOMAIN_ID": "75"}) == 75
    assert rd({}, {}) == 0
    # a robot naming 0 keeps 0 even under a stack started elsewhere: the config is explicit
    assert rd({"base_controller": {"domain_id": 0}}, {"ROS_DOMAIN_ID": "75"}) == 0
    # garbage on either side counts as unset rather than as a domain
    assert rd({"base_controller": {"domain_id": "x"}}, {"ROS_DOMAIN_ID": "75"}) == 75
    assert rd({}, {"ROS_DOMAIN_ID": "999"}) == 0


def test_set_domain_id_keeps_clears_and_refuses():
    bc = {}
    cockpit_paths.set_domain_id(bc, "42")
    assert bc == {"domain_id": 42}
    cockpit_paths.set_domain_id(bc, " ")
    assert "domain_id" not in bc
    cockpit_paths.set_domain_id(bc, 0)
    assert bc == {"domain_id": 0}
    cockpit_paths.set_domain_id(bc, None)
    assert "domain_id" not in bc
    for bad in ("233", "-3", "ten", "4.0"):
        with pytest.raises(ValueError, match=r"0\.\.232"):
            cockpit_paths.set_domain_id(bc, bad)
    assert "domain_id" not in bc


def test_the_flash_writes_the_robots_domain_or_the_stacks(reference, monkeypatch):
    params = reference("gendrv")
    params["base_controller"].pop("domain_id", None)

    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    assert "domain_id" not in mcu_env.hardware_env(params), "0 stays the absent default"

    monkeypatch.setenv("ROS_DOMAIN_ID", "9")
    assert mcu_env.hardware_env(params)["domain_id"] == 9, \
        "a robot naming no domain is flashed for the stack that flashes it"

    params["base_controller"]["domain_id"] = 21
    assert mcu_env.hardware_env(params)["domain_id"] == 21, "the robot's own wins"

    params["base_controller"]["domain_id"] = 233
    with pytest.raises(ValueError, match="domain_id"):
        mcu_env.hardware_env(params)


def test_the_sim_mcu_asks_for_the_robots_domain(tmp_path, monkeypatch):
    import gen_bare_config
    import host_firmware
    monkeypatch.setattr(cockpit_paths, "secrets_path", lambda: str(tmp_path / "none.yaml"))
    monkeypatch.setenv("ROS_DOMAIN_ID", "75")
    cfg = gen_bare_config.bare_config("sim")
    cfg["base_controller"]["domain_id"] = 12
    path = tmp_path / "bare_sim_config.yaml"
    path.write_text(yaml.safe_dump(cfg))
    assert host_firmware.env(str(path))["domain_id"] == 12
    cfg["base_controller"].pop("domain_id")
    path.write_text(yaml.safe_dump(cfg))
    assert host_firmware.env(str(path))["domain_id"] == 75


def test_every_web_ui_command_exports_the_robots_domain(monkeypatch):
    import runners
    monkeypatch.setattr(runners, "DOMAIN_RESOLVER", lambda: 42)
    assert "export ROS_DOMAIN_ID=42; true" in runners.ros_setup_shell("jazzy")

    def broken():
        raise RuntimeError("config unreadable")
    monkeypatch.setattr(runners, "DOMAIN_RESOLVER", broken)
    shell = runners.ros_setup_shell("jazzy")
    assert "ROS_DOMAIN_ID" not in shell, "a broken config keeps the inherited domain"
    assert shell.endswith("true")

    monkeypatch.setattr(runners, "DOMAIN_RESOLVER", None)
    assert "ROS_DOMAIN_ID" not in runners.ros_setup_shell("jazzy")


def test_the_backend_installs_the_resolver_and_the_route_saves_through_the_helper():
    core = read("web", "backend", "core.py")
    assert "runners.DOMAIN_RESOLVER = active_domain_id" in core
    route = read("web", "backend", "routes_config.py")
    assert 'cockpit_paths.set_domain_id(ctrl, data["network"].get("domain_id"))' in route
    assert '"domain": {"effective": cockpit_paths.robot_domain_id(params),' in route


def test_the_pipeline_runs_every_child_in_the_robots_domain():
    src = read("scripts", "one_click_pipeline.py")
    i_params = src.index("params_path = select_robot_config(args.robot, args.controller)")
    i_domain = src.index('os.environ["ROS_DOMAIN_ID"] = str(domain)')
    i_flash = src.index("[3/6] [FLASH] Updating the firmware")
    assert i_params < i_domain < i_flash, "set before the flash and the first launch"


def test_the_ui_field_reads_and_writes_base_controller_domain_id():
    html = read("web", "frontend", "index.html")
    assert 'id="cfg-domain-id"' in html and 'min="0" max="232"' in html
    js = read("web", "frontend", "app-hardware.js")
    assert 'domain_id: (document.getElementById("cfg-domain-id")?.value || "").trim(),' in js
    assert "elDomain.value = tgt.domain_id ??" in js


def test_the_firmware_takes_the_domain_from_the_env_on_both_builds():
    assert 'rcl_init_options_set_domain_id(&init_options, (size_t)envInt("domain_id", 0))' in \
        read("firmware", "src", "main.cpp")
    assert 'rcl_init_options_set_domain_id(&opts, (size_t)envInt("domain_id", 0))' in \
        read("firmware", "zephyr", "unoq", "base", "src", "main.cpp")
