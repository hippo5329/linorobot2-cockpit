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
# The LiDAR drivers bringup can start (scripts/lidar_drivers.py): LDRobot through
# our fork, and each other vendor's own ROS 2 package, pinned to the commit the
# image was verified with -- a vendor's default branch is not a version.
# YDLidar-SDK is plain CMake (no package.xml): the Dockerfile builds and installs
# it before colcon, and a COLCON_IGNORE keeps colcon from building it twice.
# m-explore-ros2 is frontier exploration (explore_lite's ROS 2 port): it drives
# Nav2 to the edge of the known map until none is left, which is how a robot
# whose sensor does not see all round -- a camera, a masked LiDAR -- builds a
# map of a multi-room space. Nav2 has no exploration of its own.
PACKAGES=(ldlidar_stl_ros2 sllidar_ros2 ydlidar_ros2_driver xv_11_driver YDLidar-SDK m-explore-ros2)

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
        sllidar_ros2)           echo "https://github.com/Slamtec/sllidar_ros2.git ." ;;
        ydlidar_ros2_driver)    echo "https://github.com/YDLIDAR/ydlidar_ros2_driver.git ." ;;
        xv_11_driver)           echo "https://github.com/mjstn/xv_11_driver.git ." ;;
        YDLidar-SDK)            echo "https://github.com/YDLIDAR/YDLidar-SDK.git ." ;;
        m-explore-ros2)         echo "https://github.com/robo-friends/m-explore-ros2.git ." ;;
    esac
}
# Commit pins (2026-09-26). Pinned packages are always fetched at the pin, never
# taken from a local checkout, so the image cannot drift with a developer's tree.
pin_of() {
    case "$1" in
        sllidar_ros2)           echo "34300099fadfc772965962dec837bf436706188f" ;;  # main
        ydlidar_ros2_driver)    echo "4ef70d3f32a85704ade0be54b214f3763b1ab3e8" ;;  # humble (the ROS 2 branch)
        xv_11_driver)           echo "82cce0fcb54f9edc61fe365df95f97aaf7bdc8c0" ;;  # main
        YDLidar-SDK)            echo "42a82ed10d2304094c111fc63dee8e4a229b79b7" ;;  # master
        m-explore-ros2)         echo "326cf8a0b487c34246bb8f3326afbcd69576dc60" ;;  # main
    esac
}

for pkg in "${PACKAGES[@]}"; do
    src="${SRC_ROOT}/${pkg}"
    pin="$(pin_of "$pkg")"
    if [ -n "$pin" ]; then
        read -r url subdir <<<"$(upstream_of "$pkg")"
        tmp="$(mktemp -d)"
        git init -q "$tmp/src"
        git -C "$tmp/src" fetch -q --depth 1 "$url" "$pin"
        git -C "$tmp/src" checkout -q FETCH_HEAD
        echo "[vendor] ${pkg}: ${url} @ ${pin:0:12}"
        src="$tmp/src/$subdir"
    elif [ ! -d "$src" ]; then
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

# xv_11_driver (2020) declares its parameters with no default, which Humble and
# later refuse to compile ("no matching function for call to
# declare_parameter(const char [5])"). Its own XV11_*_DEFAULT macros give each
# a default and so a type; the values are unchanged.
XV="${VENDOR}/xv_11_driver/src/xv_11_driver.cpp"
if [ -f "$XV" ] && grep -q 'declare_parameter("port");' "$XV"; then
    sed -i -e 's/declare_parameter("port");/declare_parameter("port", std::string(XV11_PORT_DEFAULT));/' \
           -e 's/declare_parameter("baud_rate");/declare_parameter("baud_rate", XV11_BAUD_RATE_DEFAULT);/' \
           -e 's/declare_parameter("frame_id");/declare_parameter("frame_id", std::string(XV11_FRAME_ID_DEFAULT));/' \
           -e 's/declare_parameter("firmware_version");/declare_parameter("firmware_version", XV11_FIRMWARE_VERSION_DEFAULT);/' "$XV"
    echo "[vendor] xv_11_driver: typed parameter declarations"
fi
# YDLidar-SDK sets CMP0053, CMP0037 and CMP0043 to OLD. CMake 4 (lyrical's
# resolute ships 4.2) refuses OLD for all three and stops configuring: "Policy
# CMP0053 may not be set to OLD behavior because this version of CMake no longer
# supports it". NEW is what every current CMake does anyway; CMP0037's OLD only
# allowed a target named "test", and the SDK builds none with tests off.
SDKCM="${VENDOR}/YDLidar-SDK/CMakeLists.txt"
if [ -f "$SDKCM" ] && grep -qE 'cmake_policy\(SET CMP00(53|37|43) OLD\)' "$SDKCM"; then
    sed -i -E 's/^([[:space:]]*)cmake_policy\(SET (CMP00(53|37|43)) OLD\)/\1# \2 left NEW: CMake 4 refuses OLD (prepare_docker_vendor.sh)/' "$SDKCM"
    echo "[vendor] YDLidar-SDK: dropped three OLD policies CMake 4 refuses"
fi
touch "${VENDOR}/YDLidar-SDK/COLCON_IGNORE"
# explore_lite, two fixes (measured 2026-09-26 on the Sim MCU in the rooms world):
# 1. Frontiers are searched on Nav2's global costmap (launchers/explore.launch.py
#    says why), and the search accepted only cost-0 cells as free and only walked
#    "downhill" in cost from the robot. Inflation gives every cell of a doorway a
#    cost, so the search stopped at the first door: "No frontiers found" with
#    three rooms unmapped. A cell Nav2 will drive through is free enough to explore
#    from: below INSCRIBED_INFLATED_OBSTACLE.
# 2. stop() fired cancel-all and then sent the go-home goal at once; the cancel
#    landed 13 ms later and cancelled the go-home goal too, leaving the robot
#    wherever exploration ended. Home is now sent from the cancel's own callback.
FS="${VENDOR}/m-explore-ros2/explore/src/frontier_search.cpp"
if [ -f "$FS" ] && grep -q "map_\[nbr\] == FREE_SPACE" "$FS"; then
    python3 - "$FS" <<'PYFS'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("using nav2_costmap_2d::FREE_SPACE;",
              "using nav2_costmap_2d::FREE_SPACE;\nusing nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;", 1)
s = s.replace("if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {",
              "if (map_[nbr] < INSCRIBED_INFLATED_OBSTACLE && !visited_flag[nbr]) {", 1)
s = s.replace("if (map_[nbr] == FREE_SPACE) {", "if (map_[nbr] < INSCRIBED_INFLATED_OBSTACLE) {", 1)
open(p, "w").write(s)
PYFS
    echo "[vendor] explore_lite: frontiers searched through navigable (inflated) cells"
fi
EX="${VENDOR}/m-explore-ros2/explore/src/explore.cpp"
if [ -f "$EX" ] && grep -q "  move_base_client_->async_cancel_all_goals();\n  exploring_timer_->cancel();" "$EX" 2>/dev/null || grep -q "^  move_base_client_->async_cancel_all_goals();$" "$EX"; then
    python3 - "$EX" <<'PYEX'
import sys
p = sys.argv[1]; s = open(p).read()
old = """  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }"""
new = """  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    // Home only once the cancel has been answered: sent at once, the late
    // cancel-all cancelled the go-home goal as well (prepare_docker_vendor.sh).
    move_base_client_->async_cancel_all_goals(
        [this](auto) { returnToInitialPose(); });
  } else {
    move_base_client_->async_cancel_all_goals();
  }"""
assert s.count(old) == 1, "explore.cpp stop() changed upstream"
open(p, "w").write(s.replace(old, new))
PYEX
    echo "[vendor] explore_lite: go home after the cancel is answered"
fi
# 3. No goal replaces a goal in progress. explore_lite re-targets every replan
#    (planner_frequency) by sending a new goal over the running one, and Nav2
#    1.5.1 refuses that -- "another navigator is processing, rejecting request",
#    and with allow_navigator_preemption "Timed out waiting for current
#    navigator to stop": every replanned goal on lyrical, measured 2026-09-26.
#    Planning now waits for the goal in progress to end (reached or aborted --
#    Nav2's own progress checker aborts a stuck one, and an aborted frontier is
#    blacklisted); the result callback plans the next. Exploration therefore
#    also ends with no goal running, so going home never races a cancel.
if [ -f "$EX" ] && ! grep -q "Never replace a goal in progress" "$EX"; then
    python3 - "$EX" <<'PYMP'
import sys
p = sys.argv[1]; s = open(p).read()
old = """void Explore::makePlan()
{
"""
new = """void Explore::makePlan()
{
  // Never replace a goal in progress (prepare_docker_vendor.sh): Nav2 1.5.1
  // refuses a goal sent over a running one. The result callback plans next.
  if (goal_active_) {
    return;
  }
"""
assert s.count(old) == 1, "explore.cpp makePlan() changed upstream"
open(p, "w").write(s.replace(old, new))
PYMP
    echo "[vendor] explore_lite: plan the next frontier only when the goal in progress has ended"
fi
# 4. Outside the map is not "explored". While SLAM's map (and the costmap sized
#    to it) does not yet contain the robot -- a camera's first view starts past
#    its near limit -- the frontier search logs "Robot out of costmap bounds"
#    and returns nothing, and explore_lite took that as "No frontiers found,
#    stopping": a camera run declared the rooms world explored after 2 s with
#    2 m2 mapped. It now waits for the map to reach the robot.
# 5. Home is retried. A go-home goal that fails (a planner timeout, a spin
#    refused for "Collision Ahead") was dropped and the robot left where it
#    stopped; it is now sent again, up to three times, 5 s apart.
if [ -f "$EX" ] && ! grep -q "outside the costmap yet" "$EX"; then
    python3 - "$EX" <<'PYWAIT'
import sys
p = sys.argv[1]; s = open(p).read()
old = """  if (goal_active_) {
    return;
  }
"""
new = """  if (goal_active_) {
    return;
  }
  {
    // Outside the map is not "explored" (prepare_docker_vendor.sh): wait for it.
    unsigned int mx, my;
    auto here = costmap_client_.getRobotPose();
    if (!costmap_client_.getCostmap()->worldToMap(here.position.x, here.position.y, mx, my)) {
      RCLCPP_WARN(logger_, "Robot is outside the costmap yet; waiting for the map to reach it");
      return;
    }
  }
"""
assert s.count(old) == 1, "makePlan() guard missing"
s = s.replace(old, new)
old = """        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }"""
new = """        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        } else if (home_attempts_ < 3) {
          // Home is retried (prepare_docker_vendor.sh), 5 s apart.
          home_attempts_++;
          RCLCPP_WARN(logger_, "Return to initial pose failed; retry %d of 3 in 5 s", home_attempts_);
          home_retry_timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {
            home_retry_timer_->cancel();
            returnToInitialPose();
          });
        } else {
          RCLCPP_ERROR(logger_, "Return to initial pose failed three times; giving up.");
        }"""
assert s.count(old) == 1, "returnToInitialPose() result callback changed upstream"
s = s.replace(old, new)
open(p, "w").write(s)
H = p.replace("src/explore.cpp", "include/explore/explore.h")
h = open(H).read()
old = "  bool return_to_init_;"
assert h.count(old) == 1, "explore.h changed upstream"
h = h.replace(old, old + "\n  int home_attempts_ = 0;                       // prepare_docker_vendor.sh\n"
              "  rclcpp::TimerBase::SharedPtr home_retry_timer_;")
open(H, "w").write(h)
PYWAIT
    echo "[vendor] explore_lite: wait for the map to reach the robot; retry going home"
fi
# 6. Three faults found by the gate's exploration case (2026-10-03, rooms world):
#    a. Every frontier goal was sent with yaw 0. A base that must turn to reach a
#       point (differential, skid) looks at the frontier on the way; a mecanum base
#       strafes or backs up to it and never does. With the rear 150 deg of the scan
#       masked, the nearest frontier sat 0.3 m BEHIND the robot, within the goal
#       tolerance: 8636 goals to the same point, each "reached" at once, map stuck
#       at 3.6 m2 for 900 s. A goal within 1 m -- one the robot reaches without
#       driving, so nothing turns it -- now faces from the robot to the frontier.
#       Farther goals keep upstream's yaw 0: facing EVERY goal cost an 87 deg depth
#       camera the sweep that turning to yaw 0 after each frontier gave it, and its
#       explorations ended 4-9 m2 short (2026-10-03, both distros).
#    b. That repeat was never blacklisted: fix 3 returns early while a goal is
#       active, so the no-progress check never ran, and an instantly-reached goal
#       is never "active" for long. A frontier reached three times running without
#       disappearing is now blacklisted -- counting only goals within 0.5 m of the
#       robot when sent, the reached-without-driving case, and only once the
#       frontier has outlived 15 s since it was first reached. SLAM adds a scan
#       only after 0.5 m of travel, so at the start a frontier 0.8 m ahead is
#       still on the map after three quick visits: counting those blacklisted the
#       depot's only frontier and exploration ended at 5 m2 (its map was 185 m2
#       by 21 s). The masked-mecanum loop repeats for minutes, so 15 s costs it
#       nothing.
#    c. One empty frontier search ended the run. Just after the map first reaches
#       the robot (fix 4) the costmap can lag SLAM by a cycle: a camera run declared
#       the world explored 3 s in, at 34.2 m2 of 50. Exploration now ends only after
#       five consecutive empty searches (the planner runs at 1 Hz).
if [ -f "$EX" ] && ! grep -q "faces the frontier" "$EX"; then
    python3 - "$EX" <<'PYFACE'
import sys
p = sys.argv[1]; s = open(p).read()
old = """  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");"""
new = """  if (frontiers.empty()) {
    // One empty search is not "explored" (prepare_docker_vendor.sh): five in a row.
    if (++empty_searches_ < 5) {
      RCLCPP_INFO(logger_, "No frontiers found (%d of 5 before stopping)", empty_searches_);
      return;
    }
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");"""
assert s.count(old) == 1, "makePlan() empty-frontier branch changed upstream"
s = s.replace(old, new)
old = """  // publish frontiers as visualization markers
  if (visualize_) {"""
new = """  empty_searches_ = 0;
  // publish frontiers as visualization markers
  if (visualize_) {"""
assert s.count(old) == 1, "makePlan() marker block changed upstream"
s = s.replace(old, new)
old = """  // we don't need to do anything if we still pursuing the same goal
  if (same_goal && goal_active_) {
    return;
  }
"""
new = """  // we don't need to do anything if we still pursuing the same goal
  if (same_goal && goal_active_) {
    return;
  }
  // A frontier "reached" again and again without disappearing is unreachable to
  // see from here (prepare_docker_vendor.sh): blacklist it after three.
  {
    auto here = costmap_client_.getRobotPose().position;
    double ddx = target_position.x - here.x, ddy = target_position.y - here.y;
    bool near = ddx * ddx + ddy * ddy < 0.25;   // within 0.5 m: reached without driving
    same_goal_reached_ = (same_goal && near) ? same_goal_reached_ + 1 : 0;
    if (same_goal_reached_ == 1) {
      first_reached_ = this->now();
    }
  }
  if (same_goal_reached_ >= 3 && (this->now() - first_reached_).seconds() >= 15.0) {
    RCLCPP_WARN(logger_, "Frontier (%.2f, %.2f) reached 3 times without being explored; blacklisting it",
                target_position.x, target_position.y);
    frontier_blacklist_.push_back(target_position);
    same_goal_reached_ = 0;
    makePlan();
    return;
  }
"""
assert s.count(old) == 1, "makePlan() same-goal check changed upstream"
s = s.replace(old, new)
old = """  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.;"""
new = """  goal.pose.pose.position = target_position;
  {
    // A goal within 1 m faces the frontier (prepare_docker_vendor.sh): reaching it
    // needs no driving, so nothing else turns a sensor that does not see all round
    // towards it -- yaw 0 let a mecanum base back up to a frontier behind it and
    // never look. Farther goals keep upstream's yaw 0.
    auto here = costmap_client_.getRobotPose().position;
    double dx = target_position.x - here.x, dy = target_position.y - here.y;
    double d2 = dx * dx + dy * dy;
    double yaw = (d2 > 1e-6 && d2 < 1.0) ? std::atan2(dy, dx) : 0.0;
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);
  }"""
assert s.count(old) == 1, "makePlan() goal orientation changed upstream"
s = s.replace(old, new)
old = "#include <thread>\n"
assert s.count(old) == 1
s = s.replace(old, "#include <cmath>\n#include <thread>\n")
open(p, "w").write(s)
H = p.replace("src/explore.cpp", "include/explore/explore.h")
h = open(H).read()
old = "  int home_attempts_ = 0;                       // prepare_docker_vendor.sh\n"
assert h.count(old) == 1, "explore.h: fix 5 member missing"
h = h.replace(old, old + "  int empty_searches_ = 0;                      // prepare_docker_vendor.sh, fix 6c\n"
              "  int same_goal_reached_ = 0;                   // prepare_docker_vendor.sh, fix 6b\n"
              "  rclcpp::Time first_reached_{0, 0, RCL_ROS_TIME};  // fix 6b\n")
open(H, "w").write(h)
PYFACE
    echo "[vendor] explore_lite: a near goal faces the frontier; blacklist a frontier reached thrice; stop after five empty searches"
fi
# 7. Look around once before declaring the space explored. A depth camera's first
#    map is a narrow wedge, and the frontier search -- which only sees unknown cells
#    inside the costmap, through cells Nav2 can drive -- can come back empty from a
#    robot standing still: the rooms world was "explored" 31 s in at 34.5 m2 of 50
#    after five empty searches (2026-10-03; the same 34 m2 stop happened before any
#    of fix 6). Waiting changes nothing a stationary 87 deg camera can see. On the
#    first empty search of an episode Nav2's spin behaviour turns the robot a full
#    circle, and the five empty searches are counted afresh after it; the look-around
#    is re-armed whenever frontiers reappear.
if [ -f "$EX" ] && ! grep -q "look around before stopping" "$EX"; then
    python3 - "$EX" <<'PYLOOK'
import sys
p = sys.argv[1]; s = open(p).read()
old = """  if (goal_active_) {
    return;
  }
"""
new = """  if (goal_active_ || spinning_) {
    return;
  }
"""
assert s.count(old) == 1, "makePlan() goal_active_ guard missing"
s = s.replace(old, new)
old = """    // One empty search is not "explored" (prepare_docker_vendor.sh): five in a row.
    if (++empty_searches_ < 5) {"""
new = """    // Turn once to look around before stopping (prepare_docker_vendor.sh, fix 7).
    if (!looked_around_ && spin_client_->action_server_is_ready()) {
      looked_around_ = true;
      spinning_ = true;
      RCLCPP_INFO(logger_, "No frontiers found; turning once to look around before stopping");
      auto spin_goal = nav2_msgs::action::Spin::Goal();
      spin_goal.target_yaw = 6.2;
      spin_goal.time_allowance.sec = 60;
      auto spin_options = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
      spin_options.goal_response_callback =
          [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr& h) {
            if (!h) { spinning_ = false; }
          };
      spin_options.result_callback =
          [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult&) {
            spinning_ = false;
            empty_searches_ = 0;
          };
      spin_client_->async_send_goal(spin_goal, spin_options);
      return;
    }
    // One empty search is not "explored" (prepare_docker_vendor.sh): five in a row.
    if (++empty_searches_ < 5) {"""
assert s.count(old) == 1, "fix 6c empty-search block missing"
s = s.replace(old, new)
old = "  empty_searches_ = 0;\n  // publish frontiers as visualization markers\n"
assert s.count(old) == 1, "fix 6c reset missing"
s = s.replace(old, "  empty_searches_ = 0;\n  looked_around_ = false;   // fix 7: frontiers again, so look around again next time\n  // publish frontiers as visualization markers\n")
old = """  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);
"""
assert s.count(old) == 1, "constructor action client changed upstream"
s = s.replace(old, old + """  spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(this, "spin");  // fix 7
""")
open(p, "w").write(s)
H = p.replace("src/explore.cpp", "include/explore/explore.h")
h = open(H).read()
old = '#include "nav2_msgs/action/navigate_to_pose.hpp"\n'
assert h.count(old) == 1
h = h.replace(old, old + '#include "nav2_msgs/action/spin.hpp"\n')
old = "  int same_goal_reached_ = 0;                   // prepare_docker_vendor.sh, fix 6b\n"
assert h.count(old) == 1, "fix 6b member missing"
h = h.replace(old, old + "  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;  // fix 7\n"
              "  bool spinning_ = false;                       // fix 7\n"
              "  bool looked_around_ = false;                  // fix 7\n")
open(H, "w").write(h)
PYLOOK
    echo "[vendor] explore_lite: look around once (Nav2 spin) before declaring the space explored"
fi
# map_merge is multi-robot map merging, not exploration; it is not built.
touch "${VENDOR}/m-explore-ros2/map_merge/COLCON_IGNORE"

# xv_11_driver on Lyrical's Boost: boost::asio::io_service is gone (renamed
# io_context in 1.66, which Jazzy's 1.83 has too), and xv11_laser.cpp uses M_PI
# without <cmath>, which the newer toolchain no longer pulls in by accident.
if [ -d "${VENDOR}/xv_11_driver" ] && grep -rq "io_service" "${VENDOR}/xv_11_driver/src" "${VENDOR}/xv_11_driver/include"; then
    grep -rl "io_service" "${VENDOR}/xv_11_driver/src" "${VENDOR}/xv_11_driver/include" \
        | xargs sed -i 's/boost::asio::io_service/boost::asio::io_context/g'
    sed -i '1i #include <cmath>' "${VENDOR}/xv_11_driver/src/xv11_laser.cpp"
    echo "[vendor] xv_11_driver: io_service -> io_context, <cmath>"
fi

# ament_target_dependencies is gone in Lyrical (ament_cmake 2.8): the three
# drivers stop configuring at it. The same rewrite as ldlidar_stl_ros2 above,
# generalised: rclcpp -> rclcpp::rclcpp, an interface package -> its
# ${pkg_TARGETS}. Both exist on Jazzy too, so one source builds on both.
for pkg in sllidar_ros2 ydlidar_ros2_driver xv_11_driver; do
    CM="${VENDOR}/${pkg}/CMakeLists.txt"
    [ -f "$CM" ] && grep -q "ament_target_dependencies" "$CM" || continue
    python3 - "$CM" <<'ATD_PY'
import re, sys
p = sys.argv[1]; s = open(p).read()
def repl(m):
    words = [w.strip('"') for w in m.group(1).split()]
    target, deps = words[0], words[1:]
    libs = ["rclcpp::rclcpp" if d == "rclcpp" else "${%s_TARGETS}" % d for d in deps]
    return "target_link_libraries(%s %s)" % (target, " ".join(libs))
s = re.sub(r"ament_target_dependencies\(\s*([^)]*)\)", repl, s)
open(p, "w").write(s)
ATD_PY
    echo "[vendor] ${pkg}: ament_target_dependencies -> target_link_libraries"
done

for pkg in "${PACKAGES[@]}"; do
    marker="package.xml"; [ "$pkg" = "YDLidar-SDK" ] && marker="CMakeLists.txt"
    [ "$pkg" = "m-explore-ros2" ] && marker="explore/package.xml"
    [ -f "${VENDOR}/${pkg}/${marker}" ] || { echo "[vendor] ❌ ${pkg} is not staged" >&2; exit 1; }
done
du -sh "$VENDOR"
