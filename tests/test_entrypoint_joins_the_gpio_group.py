"""The app user joins the group of every GPIO chip the container was given.

On the UNO Q the SWD lines to the STM32 are /dev/gpiochip*, group gpiod (1001). The backend
runs as the app user (setpriv --init-groups), which had dialout but not that group, so every
1-Click started from the web UI failed its SWD env write; a root `docker exec` hid it (2026-10-15).
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_the_entrypoint_adds_each_gpiochip_group_before_dropping_privileges():
    src = open(os.path.join(ROOT, "docker", "entrypoint.sh")).read()
    loop = src.index("for dev in /dev/gpiochip*; do")
    drop = src.index('exec setpriv --reuid "$HOST_UID" --regid "$HOST_GID" --init-groups')
    assert loop < drop, "the group must be joined before setpriv reads the user's groups"
    body = src[loop:src.index("done", loop)]
    assert 'stat -c %g "$dev"' in body and 'usermod -aG "$g" "$APP_USER"' in body
    assert re.search(r'case "\$g" in ""\|0\) continue', body), "root-owned chips are skipped"


def test_the_entrypoint_hands_the_config_subdirs_back_to_the_app_user():
    """A root run left /config/generated 0:0 and the UI's Bringup (the app user) could not write
    the robot's URDF there (2026-10-15)."""
    src = open(os.path.join(ROOT, "docker", "entrypoint.sh")).read()
    block = src[src.index('CFG="${COCKPIT_CONFIG_DIR:-/config}"'):]
    block = block[:block.index("# (An empty config dir")]
    assert "for sub in generated state live; do" in block
    assert 'chown -R "$HOST_UID:$HOST_GID" "$CFG/$sub"' in block
