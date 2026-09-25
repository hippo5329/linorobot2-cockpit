"""The Sim MCU is the firmware, compiled for the robot computer, speaking micro-ROS.

User, 2026-09-26: "hold for sim mcu rework, it is important to verify the
micro-ros work flow." The Sim MCU was scripts/sim_base_node.py, an rclpy node
that publishes straight into DDS -- no micro-ROS, no agent, no XRCE session, and
a host node raycasting /scan without the LiDAR driver. It is now
firmware/host/app: src/main.cpp and every firmware library, unmodified, as a
UDP4 client of micro_ros_agent, its own LD19 emulator streaming to ldlidar's
udp_server. Its first run measured /odom/unfiltered and /imu/data at 50 Hz,
/scan at 10 Hz through the driver, and found two defects on the way: the host
client's 512-byte MTU dropped every Odometry (the boards had raised theirs to
1024), and /battery was published every 2 s on EVERY board, not at 1 Hz.
"""
import json
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
sys.path.insert(0, os.path.join(REPO_ROOT, "firmware", "host"))


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


def test_the_host_env_is_the_flashers_with_the_hosts_facts(tmp_path, monkeypatch):
    """A board's own config (serial transport, serial LiDAR) run on the Sim MCU:
    the env must still say udp4 to the local agent and the LiDAR over UDP to the
    local driver, or the firmware boots unable to reach either."""
    import cockpit_paths
    import host_firmware
    import mcu_env
    monkeypatch.setattr(cockpit_paths, "secrets_path", lambda: str(tmp_path / "none.yaml"))
    cfg = yaml.safe_load(read("config", "reference", "gendrv_config.yaml"))
    assert cfg["base_controller"].get("transport", "serial") != "udp4" or \
        cfg["base_controller"]["lidar"].get("comm_mode") != "udp", "pick a config that says otherwise"
    path = tmp_path / "gendrv_config.yaml"
    path.write_text(yaml.safe_dump(cfg))
    out = host_firmware.write_env(str(path), str(tmp_path / "gen" / "env.bin"), 8877, 8878)
    blob = open(out, "rb").read()
    assert len(blob) == mcu_env.ENV_SIZE
    env = mcu_env.decode(blob)
    assert env["transport"] == "udp4" and env["agent_ip"] == "127.0.0.1" and env["agent_port"] == "8877"
    assert env["lidar_comm"] == "udp" and env["lidar_ip"] == "127.0.0.1" and env["lidar_port"] == "8878"
    assert env["sim_wheel"] == "1" and env["sim_ld19"] == "1", "the Sim MCU simulates every device"


def test_no_host_build_means_no_host_firmware(monkeypatch, tmp_path):
    import host_firmware
    monkeypatch.setattr(host_firmware, "HOST_WS", str(tmp_path))
    assert host_firmware.binary() is None


def test_the_host_client_is_configured_like_a_board():
    """The MTU is the one that bit: a best-effort stream cannot fragment, and
    Odometry is ~720 bytes."""
    import client_meta
    m = client_meta.merged(client_meta.read_meta(os.path.join(REPO_ROOT, "firmware", "host", "host.meta")),
                           client_meta.read_meta(os.path.join(REPO_ROOT, "firmware", "esp32.meta")))["names"]
    board = client_meta.read_meta(os.path.join(REPO_ROOT, "firmware", "esp32.meta"))["names"]
    assert "-DUCLIENT_CUSTOM_TRANSPORT_MTU=1024" in m["microxrcedds_client"]["cmake-args"]
    rmw = m["rmw_microxrcedds"]["cmake-args"]
    for a in board["rmw_microxrcedds"]["cmake-args"]:
        assert a in rmw, f"the host client differs from the board's in {a}"
    assert "-DRMW_UXRCE_TRANSPORT=custom" in rmw
    assert not any(a.startswith(("-DRMW_UXRCE_DEFAULT_UDP_", "-DRMW_UXRCE_STREAM_HISTORY=")) for a in rmw)


def test_the_host_takes_the_boards_library_list():
    import fetch_libdeps
    deps = fetch_libdeps.lib_deps()
    assert "jrowberg/I2Cdevlib-Core" in deps
    assert not any(d.endswith(("micro_ros_platformio", "SPI", "Wire")) for d in deps)


def test_bringup_and_the_pipeline_make_one_decision():
    launch = read("launchers", "bringup.launch.py")
    assert "host_firmware.binary()" in launch and "host_firmware.write_env(" in launch
    assert 'no_board = sim_base_arg or (controller_name == "sim" and host_fw_bin is None)' in launch
    assert 'name="sim_mcu_firmware"' in launch
    assert "RMW_IMPLEMENTATION=rmw_microxrcedds" in launch, "rcl refuses a Fast DDS stack's RMW name"
    pipe = read("scripts", "one_click_pipeline.py")
    assert "host_firmware.binary()" in pipe
    assert 'sim_base:=true" if sim_mcu' not in pipe, "the pipeline must not force the rclpy base"


def test_the_image_builds_the_client_where_it_is_used():
    df = read("docker", "Dockerfile")
    stage = df[df.index("AS hostfw"):df.index("Stage 2")]
    assert "build_client_ws.sh /opt/hosturos_ws" in stage
    assert "/uros_ws" not in stage.replace("never at /uros_ws", ""), "the agent's workspace is not the client's"
    assert "COPY --from=hostfw /opt/hosturos_ws /opt/hosturos_ws" in df
    assert "LINO_GIT_REV=${{ github.sha }}" in read(".github", "workflows", "release.yml")


def test_the_sim_mcu_config_says_what_runs():
    import gen_bare_config
    bc = gen_bare_config.bare_config("sim")["base_controller"]
    assert bc["mcu"] == "host" and bc["transport"] == "udp4" and bc["agent_ip"] == "127.0.0.1"
    assert bc["lidar"]["comm_mode"] == "udp"


def test_the_browser_leaves_the_sim_mcus_agent_to_bringup():
    """Start SLAM on bare_sim started a SERIAL agent on the Settings device, and
    bringup, seeing an agent alive, skipped the udp4 one the firmware needs."""
    js = read("web", "frontend", "app-agent-bringup.js")
    assert "if (!robotIsSimMcu() && !isAgentAlive())" in js
    assert 'robotIsSimMcu() || (nativeAgent && !isAgentAlive())' in js


def test_the_domain_is_the_envs_on_every_distro(tmp_path, monkeypatch):
    """Jazzy's rcl took ROS_DOMAIN_ID from the process and Lyrical's does not:
    the same host firmware published on the stack's domain on one and on 0 on
    the other. The firmware reads `domain_id` from the env (0 when unset, as
    every board has always been), and the host writes the stack's in."""
    src = read("firmware", "src", "main.cpp")
    assert 'rcl_init_options_set_domain_id(&init_options, (size_t)envInt("domain_id", 0))' in src
    import cockpit_paths
    import host_firmware
    import gen_bare_config
    monkeypatch.setattr(cockpit_paths, "secrets_path", lambda: str(tmp_path / "none.yaml"))
    monkeypatch.setenv("ROS_DOMAIN_ID", "75")
    path = tmp_path / "bare_sim_config.yaml"
    path.write_text(yaml.safe_dump(gen_bare_config.bare_config("sim")))
    assert host_firmware.env(str(path))["domain_id"] == 75
