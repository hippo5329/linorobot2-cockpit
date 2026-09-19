#!/usr/bin/env bash
# install_docker.sh — give the robot computer a container engine, if it has none.
#
#   bash scripts/install_docker.sh
#
# Runs before `docker compose up -d` and does NOTHING when the machine already
# has podman or docker. Only a machine with neither gets an install, and what it
# gets is Docker Engine from docs.docker.com set up ROOTLESS, so the cockpit
# container runs as you and the files it writes — maps, logs, fetched firmware —
# come back owned by you.
#
# Why not Ubuntu's docker.io: it has no rootless support at all. Not "partial" —
# `dpkg -L docker.io` matches nothing for "rootless" and
# dockerd-rootless-setuptool.sh is simply absent (checked on
# 29.1.3-0ubuntu3~24.04.2). That tool ships only in Docker Engine's own
# docker-ce-rootless-extras.
#
# An existing engine is never touched, replaced or "upgraded". docker.io and
# docker-ce conflict, so swapping one for the other would uninstall a working
# daemon to change how it is packaged — not something a setup script should do
# behind your back. If you have rootful docker.io and want rootless, remove it
# yourself first and then run this.
set -euo pipefail

say() { printf '[install_docker] %s\n' "$*"; }
die() { printf '[install_docker] %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- already ok?
if command -v podman >/dev/null 2>&1; then
    say "podman is installed ($(podman --version 2>/dev/null | head -1)) — nothing to do."
    say "podman is rootless by default; run the compose file with podman-compose or"
    say "\`podman compose\`, or enable the podman socket for the docker CLI."
    exit 0
fi

if command -v docker >/dev/null 2>&1; then
    say "docker is installed ($(docker --version 2>/dev/null | head -1))."
    # `docker info` prints a Server section and exits 0 even when it cannot
    # reach a daemon, so grepping it for "rootless" answers "no" in two very
    # different situations: a rootful daemon, and no daemon at all. Saying
    # "It is running ROOTFUL" for the second is wrong and sends the user off to
    # debug the cockpit instead of their engine. Ask whether the daemon
    # ANSWERS first -- `docker version --format {{.Server.Version}}` fails when
    # it does not.
    if ! docker version --format '{{.Server.Version}}' >/dev/null 2>&1; then
        say "…but no daemon is answering. The docker CLI is installed and the engine is not"
        say "running, or your user cannot reach its socket. This script will not guess which,"
        say "and it will not reconfigure an engine that is already installed. Either:"
        say "  * start the system daemon:  sudo systemctl enable --now docker"
        say "    (and add yourself to the docker group: sudo usermod -aG docker \$(id -un))"
        say "  * or set up the rootless one: dockerd-rootless-setuptool.sh install"
        say "Then run \`docker run --rm hello-world\` and start the cockpit."
        exit 1
    fi
    if docker info 2>/dev/null | grep -qi rootless; then
        say "It is running rootless. Nothing else is needed."
    else
        say "It is running ROOTFUL, which works: the container starts as root and drops"
        say "to the user who owns this checkout, so what it writes stays yours."
        say "For rootless you would need Docker Engine from docs.docker.com; this script"
        say "will not replace an engine that is already installed."
    fi
    exit 0
fi

say "no podman and no docker on this machine — installing Docker Engine, rootless."

# ---------------------------------------------------------------- preconditions
[ "$(uname -s)" = "Linux" ] || die "Linux only. On macOS or Windows install Docker Desktop by hand."

if [ "$(id -u)" -eq 0 ]; then
    die "Run this as the user that will own the robot, not as root. Rootless Docker
                   belongs to a normal user account; installing it as root gives you the
                   thing it exists to avoid."
fi

command -v sudo >/dev/null 2>&1 || die "sudo is required to install packages."

# HOME, if whatever started us did not set one. A login shell always has it;
# a provisioning run often does not -- `incus exec`, `ssh host <cmd>`,
# cloud-init, ansible -- and dockerd-rootless-setuptool.sh stops dead with
# "[ERROR] HOME needs to be set" after the packages are already installed,
# which looks like the install failed when it had almost finished. Same class
# of problem as the linger note below: a robot is provisioned far more often
# than it is logged into.
if [ -z "${HOME:-}" ] || [ ! -d "${HOME:-}" ]; then
    HOME="$(getent passwd "$(id -u)" | cut -d: -f6)"
    [ -n "$HOME" ] && [ -d "$HOME" ] || die "cannot determine a home directory for $(id -un)."
    export HOME
    say "HOME was not set; using $HOME"
fi
sudo -n true 2>/dev/null || say "sudo will prompt for your password."

. /etc/os-release 2>/dev/null || die "cannot read /etc/os-release; install Docker by hand: https://docs.docker.com/engine/install/"
case "${ID:-}${ID_LIKE:-}" in
    *debian*|*ubuntu*) : ;;
    *) die "This script only automates Debian/Ubuntu. For ${PRETTY_NAME:-this distro} follow
                   https://docs.docker.com/engine/install/ and then run
                   dockerd-rootless-setuptool.sh install" ;;
esac

# ---------------------------------------------------------------- the repository
# The apt repository rather than the curl|sh convenience script: the packages are
# signed, the source is auditable afterwards, and upgrades arrive with the rest
# of the system.
say "adding Docker's apt repository"
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -qq
sudo apt-get install -y -qq ca-certificates curl gnupg

sudo install -m 0755 -d /etc/apt/keyrings
if [ ! -s /etc/apt/keyrings/docker.asc ]; then
    curl -fsSL "https://download.docker.com/linux/${ID}/gpg" -o /tmp/docker.asc
    sudo install -m 0644 /tmp/docker.asc /etc/apt/keyrings/docker.asc
    rm -f /tmp/docker.asc
fi

CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
[ -n "$CODENAME" ] || die "cannot determine the distro codename."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/${ID} ${CODENAME} stable" \
    | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
sudo apt-get update -qq

say "installing docker-ce and the rootless extras"
sudo apt-get install -y -qq \
    docker-ce docker-ce-cli containerd.io \
    docker-buildx-plugin docker-compose-plugin \
    docker-ce-rootless-extras \
    uidmap dbus-user-session slirp4netns

# The system-wide daemon is not wanted: rootless runs a per-user one. Leaving the
# root daemon enabled means two daemons racing for the same container names, and
# the cockpit would talk to whichever DOCKER_HOST happened to point at.
say "disabling the system-wide daemon (rootless uses a per-user one)"
sudo systemctl disable --now docker.service docker.socket 2>/dev/null || true

# ---------------------------------------------------------------- rootless setup
command -v dockerd-rootless-setuptool.sh >/dev/null 2>&1 \
    || die "docker-ce-rootless-extras installed but dockerd-rootless-setuptool.sh is missing."

# Linger FIRST, before the setup tool runs. Two reasons, and the order matters:
#
#   * it is what a robot needs anyway -- the machine boots with nobody logged
#     in, and the per-user daemon has to come up regardless;
#   * it is what CREATES the user's systemd instance and /run/user/<uid>. Run
#     the setup tool without one (a bare `incus exec`, `ssh host <cmd>`, a
#     provisioning run) and it reports "systemd not detected", installs the
#     manual dockerd-rootless.sh path instead, and puts the socket somewhere
#     else entirely -- ~/.docker/run/docker.sock rather than the runtime dir.
#     The install then looks like it worked and no daemon ever starts at boot.
say "enabling linger (creates the user's systemd instance; a robot boots logged out)"
sudo loginctl enable-linger "$(id -un)"

UID_N="$(id -u)"
for _ in $(seq 1 20); do
    [ -S "/run/user/$UID_N/bus" ] && break
    sleep 1
done
if [ -S "/run/user/$UID_N/bus" ]; then
    export XDG_RUNTIME_DIR="/run/user/$UID_N"
    export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$UID_N/bus"
else
    say "warning: no user systemd bus appeared at /run/user/$UID_N/bus."
    say "The daemon will be installed, but starting it at boot needs systemd --user;"
    say "log in normally on this machine and re-run if the verification below fails."
fi

say "setting up the rootless daemon for $(id -un)"
dockerd-rootless-setuptool.sh install

if [ -S "/run/user/$UID_N/bus" ]; then
    systemctl --user daemon-reload || true
    systemctl --user enable --now docker.service || true
fi

# Ask the setup tool where it actually put the socket rather than assuming.
# It picks ~/.docker/run/docker.sock when there is no systemd and
# $XDG_RUNTIME_DIR/docker.sock when there is, and a guess that disagrees leaves
# DOCKER_HOST pointing at nothing.
SOCK="$(docker context inspect rootless --format '{{.Endpoints.docker.Host}}' 2>/dev/null || true)"
[ -n "$SOCK" ] || SOCK="unix://${XDG_RUNTIME_DIR:-/run/user/$UID_N}/docker.sock"
if ! grep -qs "DOCKER_HOST=$SOCK" "$HOME/.bashrc"; then
    {
        echo ""
        echo "# rootless Docker, added by linorobot2-cockpit scripts/install_docker.sh"
        echo "export DOCKER_HOST=$SOCK"
    } >> "$HOME/.bashrc"
    say "added DOCKER_HOST=$SOCK to ~/.bashrc"
fi
export DOCKER_HOST="$SOCK"

# ---------------------------------------------------------------- verify
say "verifying"
if docker info >/dev/null 2>&1 && docker info 2>/dev/null | grep -qi rootless; then
    say "rootless Docker is up: $(docker --version)"
    # The setup tool switched the CLI to its "rootless" context, which lives in
    # ~/.docker/config.json and applies to every shell immediately -- so there
    # is nothing to re-source and no new shell to open. The DOCKER_HOST line in
    # ~/.bashrc is belt and braces for anything that ignores contexts.
    # Read the context WITHOUT our own DOCKER_HOST in scope: when DOCKER_HOST is
    # set it overrides the context and `docker context show` answers "default",
    # so this line reported the opposite of what it was explaining.
    say "The CLI is on the '$(env -u DOCKER_HOST docker context show 2>/dev/null)' context, so carry straight on:"
    say "    docker compose up -d"
else
    die "the daemon did not come up rootless. Check: systemctl --user status docker"
fi
