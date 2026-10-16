#!/usr/bin/env bash
# Install the cockpit's udev rule on the HOST, so a cockpit running as a user can flash an RP2
# board (picotool talks to its bootloader over raw USB). Run once, as root:
#     sudo scripts/install_udev_rules.sh
# then unplug and replug the board (or it takes effect the next time the board enumerates).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
src="$here/udev/60-linorobot2-usb.rules"
dst=/etc/udev/rules.d/60-linorobot2-usb.rules
[ "$(id -u)" = 0 ] || { echo "run as root: sudo $0"; exit 1; }
install -m 0644 "$src" "$dst"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --attr-match=idVendor=2e8a || true
echo "installed $dst"
getent group dialout >/dev/null || echo "note: this host has no 'dialout' group; create it or edit GROUP= in $dst"
