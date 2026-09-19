#!/usr/bin/env bash
# ==============================================================================
# prepare_docker_vendor.sh — stage the sibling ROS packages into the build context
#
# bringup needs two packages the cockpit repo does not contain:
#
#   ldlidar_stl_ros2        LD19 driver, used whenever the scan is not faked
#   micro_ros_agent         pulled from the registry image inside the Dockerfile
#
# (linorobot2_description used to be the third. It is gone: the robot
# description is generated from the config by scripts/gen_robot_description.py,
# plain URDF, so nothing at run time needs that package or xacro.)
#
# The LD19 driver lives beside this repo in ~/cockpit_ws/src, which is OUTSIDE
# the Docker build context -- and COPY cannot reach outside it. Widening the context
# to the parent directory is not an option either: .dockerignore is read from
# the context root, so the parent's .git and .pio trees (several GB) would all
# be sent to the daemon, which AGENTS.md §12 calls out as looking exactly like a
# hung build.
#
# So they are staged into docker/vendor/ instead, which is gitignored. Sources
# only: `git archive` where the package is a repo, and an exclude list where it
# is not, so no build residue is carried in.
# ==============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_ROOT="${SRC_ROOT:-$(dirname "$REPO_ROOT")}"
VENDOR="${REPO_ROOT}/docker/vendor"
PACKAGES=(ldlidar_stl_ros2)

rm -rf "$VENDOR"
mkdir -p "$VENDOR"

# Where each package comes from when it is not already beside this repo.
#
# The LD19 driver is OUR FORK, not ldrobot's, and the difference is not cosmetic:
# ldrobot's node declares only the serial parameters, while every non-serial
# LiDAR path in this project needs the ones the fork adds -- `comm_mode`,
# `server_ip`/`server_port` (the board's fake_ld19 streaming a scan over UDP) and
# `raw_scan_topic`/`bins`. Vendoring ldrobot's meant `bringup.launch.py` passed
# `comm_mode: udp_server` to a node that had never heard of it: it fell through to
# the serial path, died on `input serial param error` with an empty port name, and
# esp32_wifi published no /scan at all (GenDrv bench, 2026-09-18).
# GIT_TERMINAL_PROMPT=0 makes a bad URL fail instead of asking for a username.
export GIT_TERMINAL_PROMPT=0 GIT_ASKPASS=/bin/true
upstream_of() {
    case "$1" in
        ldlidar_stl_ros2)       echo "https://github.com/hippo5329/ldlidar_stl_ros2.git ." ;;
    esac
}

for pkg in "${PACKAGES[@]}"; do
    src="${SRC_ROOT}/${pkg}"
    if [ ! -d "$src" ]; then
        read -r url subdir <<<"$(upstream_of "$pkg")"
        echo "[vendor] ${src} not found — cloning ${url}"
        tmp="$(mktemp -d)"
        git clone --depth 1 "$url" "$tmp/src"
        src="$tmp/src/$subdir"
        [ -d "$src" ] || { echo "[vendor] ❌ ${pkg} not found in ${url}" >&2; exit 1; }
    fi
    mkdir -p "${VENDOR}/${pkg}"
    if git -C "$src" rev-parse --git-dir >/dev/null 2>&1; then
        git -C "$src" archive HEAD | tar -x -C "${VENDOR}/${pkg}"
        echo "[vendor] ${pkg}: git archive HEAD"
    else
        tar -C "$src" --exclude=.git --exclude=build --exclude=install \
            --exclude=log --exclude='.pio' -cf - . | tar -x -C "${VENDOR}/${pkg}"
        echo "[vendor] ${pkg}: copied (not a git repo)"
    fi
done

# ldlidar_stl_ros2 as shipped calls pthread_mutex_lock without <pthread.h> (fails
# on GCC 13+) and uses ament_target_dependencies(), which lyrical's ament_cmake
# removed. Patch the staged copy; the same two patches belong in a fork.
LOG="${VENDOR}/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp"
if [ -f "$LOG" ] && ! grep -q "<pthread.h>" "$LOG"; then
    sed -i '1i #include <pthread.h>' "$LOG"
    echo "[vendor] ldlidar_stl_ros2: added #include <pthread.h>"
fi
CM="${VENDOR}/ldlidar_stl_ros2/CMakeLists.txt"
if [ -f "$CM" ] && grep -q "ament_target_dependencies" "$CM"; then
    python3 - "$CM" <<'PY'
import re, sys
p = sys.argv[1]; s = open(p).read()
s = re.sub(r"ament_target_dependencies\(\s*(\S+)[^)]*\)",
           r"target_link_libraries(\1 rclcpp::rclcpp ${sensor_msgs_TARGETS})", s)
open(p, "w").write(s)
PY
    echo "[vendor] ldlidar_stl_ros2: ament_target_dependencies -> target_link_libraries"
fi

for pkg in "${PACKAGES[@]}"; do
    [ -f "${VENDOR}/${pkg}/package.xml" ] || { echo "[vendor] ❌ ${pkg} is not staged" >&2; exit 1; }
done
du -sh "$VENDOR"
