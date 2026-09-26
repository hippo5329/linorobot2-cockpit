# Docker on the robot computer

What the README's Quick Start leaves out on purpose: which engine, rootless or rootful, and
why the board goes in first. Nothing here is required reading — `scripts/install_docker.sh`
makes these choices for you.

**Which Linux does not matter.** ROS 2 Jazzy, the whole stack and every tool run *inside the
container*, so the distribution and release on the robot computer are not part of the
equation: Raspberry Pi OS, Debian, an older Ubuntu, Fedora, Arch all work. You are not
installing ROS 2 and you do not need Ubuntu 24.04 to run Jazzy — the container brings its
own userland. What the machine has to provide is only the kernel and the USB bus: a 64-bit
arm64 or x86-64 CPU, a container engine, and permission to reach the serial port. That is
also why `ROS_DISTRO=lyrical` is a one-word change rather than a reinstall.

**A note on which Docker.** `scripts/install_docker.sh` handles a machine that has neither.
If you would rather do it yourself: install Docker Engine from
[docs.docker.com](https://docs.docker.com/engine/install/), or use Podman. Ubuntu's
`docker.io` package runs the cockpit perfectly well as a *rootful* daemon, but it cannot run
rootless: it ships no rootless support at all — `dpkg -L docker.io` has nothing matching
"rootless", and `dockerd-rootless-setuptool.sh` is simply absent (checked on
`29.1.3-0ubuntu3~24.04.2`). That tool comes from Docker Engine's own
`docker-ce-rootless-extras`. Podman is rootless by default and needs nothing extra.

Rootless matters because a rootful daemon runs the container as root, so maps, logs and
fetched firmware would come back owned by root. You do not have to do anything about that:
the container works out whose files they are on its own, by looking at who owns the
checkout you are about to make. (`COCKPIT_UID`/`COCKPIT_GID` override it if you ever need
to; you should not.)

**Why the board goes in first.** The cockpit reads the USB bus when it starts and again
whenever you load the page, so it will find a board plugged in later too — but starting
with it attached means the first page you see already names your MCU and has pre-selected
the matching reference build, instead of an empty **Base** panel you have to re-scan.

`install_docker.sh` does nothing at all when the machine already has podman or docker — it
prints what it found and exits. On a machine with neither it installs Docker Engine from
Docker's apt repository and sets it up **rootless**, so the container runs as you. It never
replaces an engine you already have.

