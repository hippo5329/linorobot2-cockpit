"""The robot computer keeps the stack's shared memory when its user logs out.

systemd-logind's RemoveIPC=yes (the default) deleted the running stack's Fast DDS segments in
/dev/shm when the last session of the uid the stack runs as ended: the topics stayed listed,
and every reader started afterwards received nothing (the tracked robot, 2026-10-17: 62
segments to 6 on the last logout, a scan-and-odometry look then silent; with RemoveIPC=no,
57 -> 58 and data). scripts/install_keep_ipc.sh installs the drop-in on the host.
"""
import configparser
import os
import subprocess

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DROPIN = os.path.join(REPO, "systemd", "logind.conf.d", "50-linorobot2-keep-ipc.conf")
SCRIPT = os.path.join(REPO, "scripts", "install_keep_ipc.sh")


def test_the_drop_in_turns_remove_ipc_off():
    c = configparser.ConfigParser(comment_prefixes=("#",))
    c.read(DROPIN)
    assert c.get("Login", "RemoveIPC") == "no"


def test_the_installer_puts_it_where_logind_reads_drop_ins_and_restarts_logind():
    src = open(SCRIPT).read()
    assert "/etc/systemd/logind.conf.d/50-linorobot2-keep-ipc.conf" in src
    assert "systemctl restart systemd-logind" in src
    assert 'id -u)" = 0' in src                       # refuses to run as a user
    assert os.access(SCRIPT, os.X_OK)
    subprocess.run(["bash", "-n", SCRIPT], check=True)


def test_compose_and_the_docs_send_the_robot_computer_there():
    assert "install_keep_ipc.sh" in open(os.path.join(REPO, "docker-compose.yml")).read()
    assert "install_keep_ipc.sh" in open(os.path.join(REPO, "docs", "docker.md")).read()
