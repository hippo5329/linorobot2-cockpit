#!/usr/bin/env bash
# =============================================================================
# entrypoint.sh — UID/GID reconciliation, then drop privileges
#
# The robot config directory is bind-mounted at /config and is the user's own
# git repository: the Web Cockpit edits those files and the user diffs and
# commits them. If the container wrote them as root, the user would be locked
# out of their own repo -- a `git checkout` away from needing sudo to recover.
#
# So: start as root only long enough to align ownership with the invoking host
# user, then exec the real process as that user. Named volumes are chowned too,
# because Docker creates them root-owned and an unprivileged process could not
# write to them.
# =============================================================================
set -e

APP_USER="cockpit"

# ---------------------------------------------------------------------------
# Which host user? Docker will not say, so look at something of theirs.
#
# The daemon runs as root and does not carry the client's identity into the
# container, and compose cannot compute `id -u` either -- its interpolation
# reads the environment and .env and runs no commands. So this used to be a
# COCKPIT_UID=$(id -u) prefix on the start command, which worked and which
# people forgot, and a robot that writes its maps as root is a bad first hour.
#
# docker-compose.yml binds itself read-only at /run/host-anchor. It is owned by
# whoever cloned the repo, it cannot be missing (compose could not have started
# us without it), and a bind mount carries the numeric owner through unchanged.
# One stat and we know. Explicit HOST_UID/HOST_GID still win.
#
# Only the numbers are used -- the file's CONTENT is never read.
# ---------------------------------------------------------------------------
ANCHOR="${COCKPIT_HOST_ANCHOR:-/run/host-anchor}"
ANCHOR_UID=""
ANCHOR_GID=""
if [ -e "$ANCHOR" ]; then
    ANCHOR_UID="$(stat -c %u "$ANCHOR" 2>/dev/null || true)"
    ANCHOR_GID="$(stat -c %g "$ANCHOR" 2>/dev/null || true)"
fi

if [ -z "${HOST_UID:-}" ] && [ -n "$ANCHOR_UID" ] && [ "$ANCHOR_UID" != "0" ]; then
    HOST_UID="$ANCHOR_UID"
    HOST_GID="${HOST_GID:-$ANCHOR_GID}"
    echo "[cockpit] host user ${HOST_UID}:${HOST_GID} (owner of $ANCHOR)"
fi
HOST_UID="${HOST_UID:-1000}"
HOST_GID="${HOST_GID:-1000}"

# ---------------------------------------------------------------------------
# Rootless vs rootful Docker changes which UID must own the writes.
#
#   rootless: container root IS the host user already. Staying root makes every
#             file land as them. Dropping to uid 1000 would map to host 100999
#             (subuid base + 999) and lock the host user out of their own
#             config/ -- the exact failure this is meant to prevent.
#   rootful:  container root IS host root, so privileges must be dropped.
#
# The anchor answers this too, and it is the only signal here that cannot be
# fooled. Under rootless the user's own file arrives folded onto container uid
# 0, because that is precisely what the rootless mapping does; under rootful it
# arrives with its real numeric owner.
#
# It replaces reading /proc/self/uid_map, which was wrong whenever Docker runs
# inside another user namespace. Measured on an Incus container running Ubuntu's
# rootful docker.io 29.1.3: uid_map inside the Docker container reads
# "0 1000000 1000000000" -- inherited from Incus, nothing to do with Docker --
# so the old test saw a non-zero mapping, concluded "rootless", stayed root, and
# handed the user a ~/linorobot2-config owned by 0:0. `docker info` listed no
# rootless security option at all. Nested namespaces are not exotic: LXD/Incus,
# a dev container, and rootless Podman running rootful Docker all hit it.
#
# The anchor read 1000 on that same box, which is the right answer.
#
# COCKPIT_DROP_PRIVS=always|never overrides the detection.
# ---------------------------------------------------------------------------
detect_drop_privs() {
    case "${COCKPIT_DROP_PRIVS:-auto}" in
        always) echo yes; return ;;
        never)  echo no;  return ;;
    esac
    if [ -n "$ANCHOR_UID" ]; then
        # Non-zero: the mount was not folded, so this is a rootful daemon and
        # container root is host root. Zero: either rootless (folded) or a
        # genuinely root-owned checkout -- and staying root is right for both.
        [ "$ANCHOR_UID" = "0" ] && echo no || echo yes
        return
    fi
    # No anchor (the image run without compose). Fall back to the namespace
    # reading, with its nested-userns blind spot, rather than guessing.
    if [ -r /proc/self/uid_map ]; then
        host_uid_of_root="$(awk 'NR==1 {print $2}' /proc/self/uid_map)"
        if [ -n "$host_uid_of_root" ] && [ "$host_uid_of_root" != "0" ]; then
            echo no
            return
        fi
    fi
    echo yes           # rootful (or unknown): be safe and drop
}

DROP_PRIVS="$(detect_drop_privs)"

if [ "$(id -u)" = "0" ] && [ "$DROP_PRIVS" = "yes" ]; then
    getent group "$HOST_GID" >/dev/null 2>&1 || groupadd -g "$HOST_GID" "$APP_USER"
    if ! getent passwd "$HOST_UID" >/dev/null 2>&1; then
        useradd -u "$HOST_UID" -g "$HOST_GID" -M -d "/home/$APP_USER" -s /bin/bash "$APP_USER"
    fi
    APP_USER="$(getent passwd "$HOST_UID" | cut -d: -f1)"

    # A HOME the user can actually write to. setpriv changes the uid and NOTHING
    # else: HOME stays whatever the container started with, which is /root, and
    # /root is not writable by uid 1000. Everything that keeps state under ~
    # then fails after doing its work --
    #
    #   [flash_mcu] ✅ Firmware flashed successfully via picotool (--family rp2040)!
    #   PermissionError: [Errno 13] Permission denied: '/root/.cache'
    #   ❌ [FLASH FAILED] Microcontroller firmware flash failed for 'pico'!
    #
    # -- the board had the image, the run was halted, and the message named the
    # flash. ~/.ros/log is the same trap one step later.
    APP_HOME="$(getent passwd "$HOST_UID" | cut -d: -f6)"
    case "$APP_HOME" in
        ""|/|/root) APP_HOME="/home/$APP_USER" ;;
    esac
    mkdir -p "$APP_HOME" 2>/dev/null || true
    chown "$HOST_UID:$HOST_GID" "$APP_HOME" 2>/dev/null || true
    export HOME="$APP_HOME"

    # Serial access on the robot: the tty nodes are group dialout, and the
    # group's GID inside the image will not match the host's, so align it.
    if [ -n "${SERIAL_GID:-}" ]; then
        groupadd -g "$SERIAL_GID" hostdialout 2>/dev/null || true
        usermod -aG "$SERIAL_GID" "$APP_USER" 2>/dev/null || true
    fi

    # Writable paths only. Never a blanket chown of /ws: that would rewrite
    # every source file's ownership on a bind-mounted checkout.
    # Only reached in rootful mode; under rootless the volumes are already
    # owned by the host user via the namespace mapping.
    #
    # The list is every directory the stack WRITES INTO, not just the named
    # volume mount points -- a distinction that cost a working 1-Click. The
    # first step of the pipeline is gen_firmware_header.py, whose output is
    # firmware/include/custom/lino_base_config.h; that directory does not exist
    # in the image (it is gitignored) and its parent, firmware/include, is baked
    # root-owned like the rest of the source tree. So under a ROOTFUL daemon --
    # Ubuntu's docker.io, the common case -- the very first thing the big Start
    # button does was:
    #
    #   [1/6] [CONFIG] Generating firmware header for base controller 'pico2'...
    #   PermissionError: [Errno 13] Permission denied: '/ws/firmware/include/custom'
    #   [pipeline] Mission finished with exit code 1
    #
    # mkdir -p, so a directory that is only ever created at run time is created
    # here, as root, and handed over -- rather than being created later by a
    # process that cannot.
    #
    # Two lists, and the split is about cost, not about safety.
    #
    # CONTENTS TOO. Everything here is rewritten in place by a process running
    # as the user -- and `open(path, "w")` truncates, which needs write
    # permission on the FILE, not just on the directory holding it. A named
    # volume that an earlier root-run container wrote into keeps those files
    # root-owned forever, so chowning only the directory fixes the first run and
    # nothing after it:
    #
    #   PermissionError: [Errno 13] Permission denied: '/ws/logs/bringup.log'
    #
    # after a run that had already flashed the board. These are small -- a
    # handful of logs, maps and release images -- so -R costs nothing.
    for path in /ws/logs /ws/maps /ws/firmware/prebuilt /ws/firmware/_staged \
                /ws/firmware/include/custom; do
        mkdir -p "$path" 2>/dev/null || true
        chown -R "$HOST_UID:$HOST_GID" "$path" 2>/dev/null || true
    done
    # DIRECTORY ONLY. Package and build caches: new files are created by the
    # user who owns the directory, so the entry point is all that matters, and a
    # `chown -R` over a populated PlatformIO build tree takes minutes on
    # millions of small object files -- which looks exactly like a hung
    # container.
    for path in /pio /ws/firmware/.pio; do
        mkdir -p "$path" 2>/dev/null || true
        chown "$HOST_UID:$HOST_GID" "$path" 2>/dev/null || true
    done
    # The config dir is the bind mount. Chowning to the host user is a no-op in
    # the normal case and repairs it if an earlier root-run container claimed it.
    CFG="${COCKPIT_CONFIG_DIR:-/config}"
    if [ -d "$CFG" ]; then
        chown "$HOST_UID:$HOST_GID" "$CFG" 2>/dev/null || true
        find "$CFG" -maxdepth 1 -type f -exec chown "$HOST_UID:$HOST_GID" {} + 2>/dev/null || true
        # .git as well. The config dir IS a git repository -- that is the whole
        # point of it, and the reason this block exists -- and git refuses to
        # work in a tree owned by somebody else:
        #
        #   fatal: detected dubious ownership in repository at
        #   '/home/ubuntu/linorobot2-config'
        #
        # Repairing the YAML files but not .git leaves the user able to edit
        # their robot configs and unable to diff, commit or revert them, which
        # is precisely the lock-out this is meant to undo. Measured on a box
        # whose config dir had been created by an older root-running image: the
        # files came back owned, .git stayed 0:0, and the Cockpit header showed
        # "Branch: unknown" because its own `git` call failed the same way. A
        # config repo is ten small YAML files and their objects, so -R is free.
        if [ -d "$CFG/.git" ]; then
            chown -R "$HOST_UID:$HOST_GID" "$CFG/.git" 2>/dev/null || true
        fi
    fi
fi
# (An empty config dir is seeded from config/reference by the supervisor itself.)

if [ "$DROP_PRIVS" = "yes" ]; then
    echo "[cockpit] rootful docker — dropping to uid=${HOST_UID} gid=${HOST_GID} (HOME=${HOME})"
else
    echo "[cockpit] rootless docker — staying root (maps to the host user)"
fi

if [ "$(id -u)" = "0" ] && [ "$DROP_PRIVS" = "yes" ]; then
    # setpriv over su/gosu: it is in util-linux, already present, and does not
    # fork an extra supervising process that would swallow signals.
    exec setpriv --reuid "$HOST_UID" --regid "$HOST_GID" --init-groups -- "$@"
fi
exec "$@"
