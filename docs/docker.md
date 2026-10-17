# Docker on the robot computer

What the README's Quick Start leaves out on purpose: which engine, rootless or rootful, and
why the board goes in first. Nothing here is required reading — `scripts/install_docker.sh`
makes these choices for you.

**Which Linux does not matter.** ROS 2 (Lyrical by default, or Jazzy), the whole stack and
every tool run *inside the container*, so the distribution and release on the robot computer
are not part of the equation: Raspberry Pi OS, Debian, an older Ubuntu, Fedora, Arch all
work. You are not installing ROS 2 and you do not need the Ubuntu release a ROS distro is
built for — the container brings its own userland. What the machine has to provide is only
the kernel and the USB bus: a 64-bit arm64 or x86-64 CPU, a container engine, and permission
to reach the serial port. That is also why `ROS_DISTRO=jazzy` is a one-word change rather
than a reinstall.

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


**Run `sudo scripts/install_keep_ipc.sh` once.** The stack keeps its DDS shared memory in the
host's `/dev/shm`, owned by the same uid as you, and systemd-logind by default deletes every
`/dev/shm` file a user owns when that user's last session ends. So logging out of your last
SSH session took the running stack's shared memory with it: the topics were still listed, but
anything that started reading afterwards -- the Map Viewer reconnecting, a test, Nav2
restarting -- received nothing until bringup was restarted. The script installs
`systemd/logind.conf.d/50-linorobot2-keep-ipc.conf` (`RemoveIPC=no`) and restarts logind,
which keeps the sessions it manages. A robot computer that only ever runs the cockpit and is
never logged into is not affected, but most are.
