#!/usr/bin/env bash
# Keep the robot stack's shared memory when you log out: install the cockpit's logind drop-in
# on the HOST (the robot computer). Run once, as root:
#     sudo scripts/install_keep_ipc.sh
# systemd-logind deletes all of a user's /dev/shm files when that user's last session ends
# (RemoveIPC=yes, the default); the stack's Fast DDS segments are owned by that same uid, so
# logging out broke every reader started afterwards. See systemd/logind.conf.d/.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
src="$here/systemd/logind.conf.d/50-linorobot2-keep-ipc.conf"
dst=/etc/systemd/logind.conf.d/50-linorobot2-keep-ipc.conf
[ "$(id -u)" = 0 ] || { echo "run as root: sudo $0"; exit 1; }
install -d -m 0755 /etc/systemd/logind.conf.d
install -m 0644 "$src" "$dst"
# logind reads its configuration at start; restarting it keeps the sessions it manages.
systemctl restart systemd-logind
now=$(busctl get-property org.freedesktop.login1 /org/freedesktop/login1 org.freedesktop.login1.Manager RemoveIPC 2>/dev/null || true)
echo "installed $dst (logind RemoveIPC: ${now#b })"
[ "${now#b }" = false ] || { echo "logind still removes IPC: check for another RemoveIPC= setting in /etc/systemd/logind.conf*"; exit 1; }
