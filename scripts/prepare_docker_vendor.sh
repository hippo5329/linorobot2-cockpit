#!/usr/bin/env bash
# ==============================================================================
# prepare_docker_vendor.sh — stage the sibling ROS packages into the build context
#
# bringup needs two packages the cockpit repo does not contain:
#
#   ldlidar_stl_ros2        LD19 driver, used whenever the scan is not simd
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
# `server_ip`/`server_port` (the board's sim_ld19 streaming a scan over UDP) and
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

# Four defects, and ALL FOUR ARE NOW FIXED IN THE FORK (2026-09-20): the
# missing <pthread.h> behind pthread_mutex_lock (fails on GCC 13+),
# ament_target_dependencies(), which lyrical's ament_cmake removed, the near
# filter throwing away any revolution with no gap in it, and the network
# Start() that never marked the driver started. Every block below is therefore
# a no-op against the current fork.
#
# They are kept because SRC_ROOT wins over the clone: a developer with an older
# sibling checkout of the driver stages THAT, and each block is idempotent and
# costs a grep. The two behavioural ones are covered by
# tests/test_ldlidar_scan_assembly.py and tests/test_ldlidar_network_start.py,
# which compile whatever was staged and run it -- so a stale checkout that
# somehow defeats a guard fails the suite rather than a robot.
LOG="${VENDOR}/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp"
if [ -f "$LOG" ] && ! grep -q "<pthread.h>" "$LOG"; then
    sed -i '1i #include <pthread.h>' "$LOG"
    echo "[vendor] ldlidar_stl_ros2: added #include <pthread.h>"
fi
# A scan with no gaps in it comes back EMPTY, and the node then publishes nothing
# for ever without saying why. Tofbf::NearFilter groups the revolution by
# continuity, then "connects 0 and 359 degrees" by moving the last group onto the
# front one and erasing it. When the whole revolution is ONE group -- which it is
# whenever the surface is continuous all the way round -- front() and back() are
# the same vector: it is inserted into itself (undefined behaviour, the iterators
# are invalidated by the reallocation) and then erased as a duplicate, so the
# filter returns nothing at all.
#
# Downstream that is silent. LiPkg::AssemblePacket publishes only `if
# (tmp.size() > 0)`, and it erases the revolution from its buffer only inside
# that same branch -- so the points pile up until the overrun bail throws them
# away, and demo.cpp prints nothing for the DATA_WAIT that results.
#
# Found 2026-09-20 by driving the driver from a harness: our simulated LD19 room is
# geometrically perfect, so all 456 points of a revolution form one group and
# every scan after the first was discarded. The first survives only because the
# driver drops the very first packet to seed its timestamp, which leaves an arc
# that does not close. A real LD19 is saved by its own noise, not by anything in
# the code -- point one at a smooth cylinder and it has the same bug.
FILT="${VENDOR}/ldlidar_stl_ros2/ldlidar_driver/src/filter/tofbf.cpp"
if [ -f "$FILT" ] && ! grep -q "group.size() > 1" "$FILT"; then
    python3 - "$FILT" <<'TOFBF_PY'
import sys
p = sys.argv[1]
s = open(p).read()
old = "  if (fabs(first_item.angle + 360.f - last_item.angle) < angle_delta_up_limit &&"
new = ("  // One group means front() and back() are the same vector: merging it into\n"
       "  // itself is undefined behaviour, and erasing it loses the whole scan.\n"
       "  if (group.size() > 1 &&\n"
       "      fabs(first_item.angle + 360.f - last_item.angle) < angle_delta_up_limit &&")
assert old in s, "tofbf.cpp: wrap-merge condition not found -- has upstream changed?"
open(p, "w").write(s.replace(old, new, 1))
TOFBF_PY
    echo "[vendor] ldlidar_stl_ros2: the wrap merge no longer eats a single-group scan"
fi

# Every network comm mode starts "successfully" and then reports STOP for ever.
# The serial LDLidarDriver::Start() ends with `is_start_flag_ = true;
# SetIsOkStatus(true);`; the network overload -- TCP client, TCP server, UDP
# client, UDP server -- ends with a bare `return true;` and sets neither. So
# GetLaserScanData() takes its `if (!is_start_flag_) return LidarStatus::STOP;`
# branch on every call, and demo.cpp's switch has no case for STOP: it falls
# into `default: break`. The node publishes nothing, for ever, in silence.
#
# Nothing upstream of that looks wrong, which is what made it expensive to
# find. "ldlidar node start is success" comes from Start()'s return value;
# "ldlidar communication is normal" comes from is_poweron_comm_normal_, set by
# the parser when it accepts a packet -- so the socket is open, the bytes
# arrive, the CRC passes and full 456-point revolutions are assembled. They are
# then never collected. Instrumenting demo.cpp's loop on the GenDrv bench,
# 2026-09-20:
#
#     DBG loops=400 normal=0 wait=0 timeout=0 other=399 freq=0.00 pts=0
#
# `other` is STOP. GetLidarScanFreq() returns early on the same flag, which is
# the freq=0.00 -- and ToLaserscanMessagePublish() skips the publish entirely
# when the frequency is not positive, so even reaching it would not have helped.
DRV="${VENDOR}/ldlidar_stl_ros2/ldlidar_driver/src/core/ldlidar_driver.cpp"
if [ -f "$DRV" ] && [ "$(grep -c 'is_start_flag_ = true;' "$DRV")" -lt 3 ]; then
    python3 - "$DRV" <<'DRIVER_PY'
import sys
p = sys.argv[1]
s = open(p).read()
old = """      break;
  }
  
  return true;
}

bool LDLidarDriver::Start(LDType product_name, CommunicationModeTypeDef comm_mode) {"""
new = """      break;
  }

  // The serial overload sets both of these; this one used to set neither, so
  // every network mode reported LidarStatus::STOP for ever after starting
  // "successfully".
  is_start_flag_ = true;

  SetIsOkStatus(true);

  return true;
}

bool LDLidarDriver::Start(LDType product_name, CommunicationModeTypeDef comm_mode) {"""
assert old in s, "ldlidar_driver.cpp: network Start() tail not found -- has upstream changed?"
open(p, "w").write(s.replace(old, new, 1))
DRIVER_PY
    echo "[vendor] ldlidar_stl_ros2: network Start() now marks the driver started"
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
