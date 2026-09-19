# =============================================================================
# Dockerfile — the Linorobot2 Cockpit runtime for the ROBOT COMPUTER
#
# Runs on the computer the microcontroller is plugged into (RPi 5 / Jetson /
# any Linux box): the web supervisor, micro_ros_agent, the ROS 2 stack and the
# flashing tools (esptool, picotool). Firmware is normally a prebuilt release
# image fetched at run time; building needs docker/Dockerfile.pio instead.
#
# LINUX ONLY, and must run with `network_mode: host`:
#   * flashing is over USB, which Docker Desktop cannot pass through
#   * Wi-Fi transport needs the board to reach micro_ros_agent's UDP port on
#     the LAN, which a NAT'd Docker Desktop VM cannot offer
# =============================================================================
ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------
# Stage 1 — picotool, built from source.
#
# picotool is in neither noble nor resolute, universe included, so there is no
# package to install: without it the RP2040/RP2350 half of the bench cannot be
# flashed at all. It is built here and only the resulting binary is copied into
# the runtime stage, so cmake and the toolchain never reach the shipped image.
#
# This must be a real build rather than the silent `|| echo "not available"`
# fallback it replaces. That fallback turned a missing flasher into a message
# nobody reads until a board fails to flash halfway through a test run.
# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS picotool-build
ARG PICOTOOL_VERSION=2.1.1
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/01proxy; fi; \
    apt-get update && apt-get install -y --no-install-recommends \
        git cmake build-essential pkg-config libusb-1.0-0-dev ca-certificates \
    && rm -rf /var/lib/apt/lists/*
# picotool builds against the SDK for headers only, so a shallow clone with no
# submodules is enough and saves most of the download.
RUN git clone --depth 1 -b "${PICOTOOL_VERSION}" https://github.com/raspberrypi/pico-sdk /opt/pico-sdk \
 && git clone --depth 1 -b "${PICOTOOL_VERSION}" https://github.com/raspberrypi/picotool /tmp/picotool \
 && cmake -S /tmp/picotool -B /tmp/picotool/build \
          -DPICO_SDK_PATH=/opt/pico-sdk -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /tmp/picotool/build -j"$(nproc)" \
 && /tmp/picotool/build/picotool version

# ---------------------------------------------------------------------------
# Stage 1b — micro_ros_agent, built from source against THIS image's ROS 2.
#
# The published image was tried first and does not survive contact with the
# serial transport. Its binary is linked against a different ROS 2 build than
# ros:<distro>-ros-base, and the mismatch surfaces as
#
#     *** stack smashing detected ***: terminated
#
# the instant the agent opens a tty -- immediately after "logger setup", inside
# TermiosAgentLinux.cpp. So it is built from source, against this image's ROS 2.
#
# Worth recording: the udp4 probe PASSED on the prebuilt binary. Only the serial
# path crashes, so a transport-agnostic smoke test is not evidence of anything.
# Both transports are probed below.
#
# Built at /uros_ws so the install prefix colcon bakes into its setup scripts is
# the path the runtime stage mounts it at.
# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS uros
ARG ROS_DISTRO
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/01proxy; fi; \
    apt-get update && apt-get install -y --no-install-recommends \
        git build-essential cmake python3-colcon-common-extensions \
        libasio-dev libtinyxml2-dev libssl-dev \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /uros_ws/src \
 && git clone --depth 1 -b ${ROS_DISTRO} https://github.com/micro-ROS/micro-ROS-Agent.git \
        /uros_ws/src/micro-ROS-Agent \
 && git clone --depth 1 -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_msgs.git \
        /uros_ws/src/micro_ros_msgs \
 && . /opt/ros/${ROS_DISTRO}/setup.sh \
 && cd /uros_ws \
 && colcon build --merge-install --packages-up-to micro_ros_agent 2>&1 | tail -20 \
 && rm -rf /uros_ws/build /uros_ws/log /uros_ws/src

# ---------------------------------------------------------------------------
# Stage 2 — the robot runtime.
# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base

ENV PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_BREAK_SYSTEM_PACKAGES=1

# Optional apt cache for sites that run one: --build-arg APT_PROXY=http://cache:3142
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/01proxy; fi

# Bring the base image's ROS packages up to the CURRENT snapshot before adding
# any. This is not hygiene, it is a correctness fix. On a rolling-snapshot
# distro the `ros:<distro>-ros-base` image lags the archive by weeks, and every
# package installed below comes from today's snapshot -- so the two disagree
# about the rosidl ABI. rc-20260919's lyrical image shipped std_msgs from
# 2026-08-11, whose typesupport library defines no `has_buffer_fields_*`
# symbols at all, under geographic_msgs / rosbridge_msgs / robot_localization
# from 2026-09-15, which need them. The result was three nodes dying at
# startup with `undefined symbol: has_buffer_fields_std_msgs__msg__Header`:
# ekf_node (so no /odom), and rosbridge_websocket + rosapi (so the browser
# canvas on :9090 had nothing to talk to) -- on an image that otherwise looked
# perfect, because nothing fails until a node is actually launched.
RUN apt-get update \
 && apt-get upgrade -y --no-install-recommends \
 && rm -rf /var/lib/apt/lists/*

# Everything the launch trees name. The set is not guesswork: it is what
# `FindPackageShare` / `package=` resolve to across launchers/ (the robot
# description is generated as plain URDF by scripts/gen_robot_description.py,
# so robot_state_publisher and joint_state_publisher are needed, xacro and the
# linorobot2_description package are not). A launch file
# that names a missing package throws while the description is being BUILT, so
# one absent package takes the whole bringup down rather than degrading it --
# which is how "package 'joint_state_publisher' not found" left every topic
# empty while the board itself was running perfectly.
#
# rosbridge + rosapi expose the topic graph to the browser canvas on :9090.
# lsof is not optional: the serial-release check reads it to confirm the tty
# was freed before esptool opens it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-yaml python3-serial \
        ros-${ROS_DISTRO}-rosbridge-server \
        ros-${ROS_DISTRO}-rosapi \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-imu-tools \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-slam-toolbox \
        lsof usbutils udev curl ca-certificates util-linux \
    && rm -rf /var/lib/apt/lists/*

# Nav2. The line this replaces installed `navigation2` and `nav2_bringup` and
# shrugged if either was missing, which on lyrical meant an image with no
# navigation at all -- announced once in the build log and then forgotten.
#
# Measured on 2026-09-19: lyrical publishes 41 matching packages, `nav2_bringup`
# and `opennav_docking` among them, but no `navigation2` metapackage. So the
# metapackage is the only thing actually absent, and installing by pattern gets
# the whole stack. Do not trust that on the next distro -- rolling has neither
# nav2 nor slam_toolbox -- hence the source fallback below. Recipe from the
# console project, which solved this first:
#
#   1. apt-install every published ros-<distro>-(nav2|opennav)-* package. A
#      pattern, not a hand-kept list that drifts with each nav2 release.
#      opennav_* is not optional: nav2_bringup launches opennav_docking, and a
#      ^ros-<distro>-nav2- pattern alone leaves the launch failing on
#      "package 'opennav_docking' not found".
#   2. If nav2_bringup is still absent, build it and the `navigation2`
#      metapackage from source into /opt/nav2_ws. nav2_bringup's CMakeLists
#      find_package()s navigation2, so building it alone stops at "Could not
#      find a package configuration file provided by navigation2".
#      COLCON_IGNORE hides the ~40 sibling packages in the monorepo, which
#      colcon would otherwise insist on rebuilding from source instead of
#      taking from /opt/ros. rosdep must skip `navigation2` (no binary -- that
#      is the whole reason we are here) and `nav2_smac_planner` (not published
#      for lyrical; the cockpit configures nav2_navfn_planner::NavfnPlanner, so
#      nothing we launch needs smac).
#
# This step is fatal on failure, unlike the line it replaces. rc-20260919 is
# the argument: an image that is quietly missing a package ships, gets
# released, and fails on the user's bench instead of in CI.
RUN apt-get update \
 && NAV2_PKGS=$(apt-cache --names-only search "^ros-${ROS_DISTRO}-(nav2|opennav)-" \
        | awk '{print $1}' | grep -v -- '-dbgsym$' | tr '\n' ' ') \
 && if [ -n "$NAV2_PKGS" ]; then \
        echo "[build] nav2 components from apt: $NAV2_PKGS"; \
        apt-get install -y --no-install-recommends $NAV2_PKGS; \
    else \
        echo "[build] no ros-${ROS_DISTRO}-nav2-* packages in apt"; \
    fi \
 && (apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
     || echo "[build] no navigation2/nav2_bringup binary for ${ROS_DISTRO}; building from source next") \
 && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && if [ -d "/opt/ros/${ROS_DISTRO}/share/nav2_bringup" ]; then \
        echo "[build] nav2_bringup came from apt; no source build needed"; \
    else \
        apt-get update \
     && mkdir -p /opt/nav2_ws/src \
     && cd /opt/nav2_ws/src \
     && (git clone --depth 1 -b ${ROS_DISTRO} https://github.com/ros-navigation/navigation2.git navigation2 \
         || git clone --depth 1 -b main https://github.com/ros-navigation/navigation2.git navigation2) \
     && KEEP='/(nav2_bringup|navigation2)$' \
     && find /opt/nav2_ws/src/navigation2 -name package.xml -printf '%h\n' \
            | grep -vE "$KEEP" | xargs -r -I@ touch @/COLCON_IGNORE \
     && cd /opt/nav2_ws \
     && (rosdep update --rosdistro ${ROS_DISTRO} 2>/dev/null || true) \
     && (rosdep install --from-paths $(find /opt/nav2_ws/src/navigation2 -name package.xml \
                -printf '%h\n' | grep -E "$KEEP" | tr '\n' ' ') \
             --ignore-src -y --rosdistro ${ROS_DISTRO} \
             --skip-keys 'navigation2 nav2_smac_planner microxrcedds_agent' || true) \
     && if ! colcon build --merge-install --packages-select nav2_bringup navigation2 \
                > /tmp/nav2.log 2>&1; then \
            echo "[build] Nav2 source build FAILED:"; tail -40 /tmp/nav2.log; exit 1; \
        fi \
     && tail -5 /tmp/nav2.log \
     && test -f /opt/nav2_ws/install/share/nav2_bringup/local_setup.bash \
     && rm -rf /opt/nav2_ws/build /opt/nav2_ws/log /tmp/nav2.log /var/lib/apt/lists/*; \
    fi

# esptool from pip rather than the archive: the packaged version lags behind
# current ESP32-S3 silicon revisions.
#
# --ignore-installed is load-bearing, and it applies to the whole set rather
# than to one named package. These wheels want newer versions of things the ros
# base image already carries from dpkg, and a dpkg-installed distribution has no
# RECORD file, so pip cannot uninstall it: the install aborts with
# "uninstall-no-record-file / Cannot uninstall <pkg>". Which package trips it
# varies by distro -- cryptography on jazzy, typing_extensions on lyrical -- so
# naming them one at a time only moves the failure. Installing over the dpkg
# copies instead of replacing them keeps the layer buildable on every distro.
RUN pip install --no-cache-dir --ignore-installed \
        "fastapi>=0.110" "uvicorn[standard]>=0.27" "esptool>=4.7"
# libusb is the runtime half of picotool: RP2040/RP2350 flashing speaks raw USB
# to the bootloader, not a serial protocol, so there is no tty involved at all.
RUN apt-get update && apt-get install -y --no-install-recommends libusb-1.0-0 \
    && rm -f /etc/apt/apt.conf.d/01proxy && rm -rf /var/lib/apt/lists/*
COPY --from=picotool-build /tmp/picotool/build/picotool /usr/local/bin/picotool

# micro_ros_agent from the build stage, copied back to the SAME path it was built at. colcon's generated setup scripts
# bake their build-time prefix in and refuse to source from anywhere else:
# "The build time path /uros_ws/install doesn't exist." Relocating it would mean
# setting COLCON_CURRENT_PREFIX at every source site, which is a trap waiting for
# the next person; keeping the path is free.
COPY --from=uros /uros_ws/install /uros_ws/install

WORKDIR /ws
COPY . /ws

# The ROS workspace. bringup needs two packages beyond this repo:
# ldlidar_stl_ros2 for a real LD19, and micro_ros_agent above. The driver is
# staged into docker/vendor by scripts/prepare_docker_vendor.sh (which clones it
# when it is not beside this repo), because COPY cannot reach outside the build
# context.
# --base-paths rather than a src/ tree of symlinks: colcon discovers packages by
# recursing the given paths, and /ws already contains both this package and the
# staged vendor ones. --merge-install gives a single setup.bash to source.
# Two base paths, not one. /ws is itself a package -- package.xml sits at its
# root -- and colcon stops descending as soon as it identifies one, so the
# vendor package nested inside it is never seen. Naming the directory
# explicitly is what makes them build.
# The build must NOT be piped into tail: a pipeline exits with the status of its
# last command, so `colcon ... | tail` reported success for a failed build, and
# the old `test -f /opt/lino_ws/setup.bash` guard passed anyway because colcon
# writes that file even when every package fails. rc-20260919 shipped that way
# on both distros -- no local_setup.bash for this package and no
# ldlidar_stl_ros2 at all -- so bringup could not start on any released image.
# Redirect instead, show the tail only on failure, and assert the two artefacts
# that actually prove the overlay is usable.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && bases="/ws"; [ -d /ws/docker/vendor ] && bases="$bases /ws/docker/vendor"; \
    if ! colcon build --base-paths $bases --merge-install \
            --install-base /opt/lino_ws --build-base /tmp/colcon_build \
            > /tmp/colcon.log 2>&1; then \
        echo "[build] colcon build FAILED:"; tail -40 /tmp/colcon.log; exit 1; \
    fi; \
    tail -8 /tmp/colcon.log \
 && rm -rf /tmp/colcon_build /ws/log /tmp/colcon.log \
 && test -f /opt/lino_ws/setup.bash \
 && test -f /opt/lino_ws/share/linorobot2_cockpit/local_setup.bash \
 && test -x /opt/lino_ws/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node

# ...and prove the overlay is usable, not merely present: ros2 must RESOLVE
# both packages and the bringup launch file must be where a launch resolves it.
# Sourcing is the part that actually broke -- share/<pkg>/local_setup.bash is
# named by package.dsv, so a missing one makes `source setup.bash` fail with
# `not found: ".../local_setup.bash"` and every launch after it.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && . /opt/lino_ws/setup.sh \
 && ros2 pkg prefix linorobot2_cockpit \
 && ros2 pkg prefix ldlidar_stl_ros2 \
 && test -f "$(ros2 pkg prefix linorobot2_cockpit)/share/linorobot2_cockpit/launchers/bringup.launch.py" \
 && echo "[build] ROS overlay OK"

# The overlay check above proves the packages are THERE. It does not prove they
# can be loaded together, and on a rolling-snapshot distro that is a different
# question: every rosidl typesupport library is dlopened lazily, the first time
# something publishes that type, so an ABI mismatch is invisible until a node
# runs. rc-20260919's lyrical image passed every structural check and then lost
# ekf_node, rosbridge_websocket and rosapi at startup to
# `undefined symbol: has_buffer_fields_std_msgs__msg__Header`.
#
# Creating one publisher per message package is what forces the dlopen. These
# are the packages whose absence or breakage takes the stack down: the three
# that actually broke, plus the types bringup itself publishes.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && . /opt/lino_ws/setup.sh \
 && python3 /ws/scripts/check_ros_abi.py \
 && echo "[build] ROS ABI OK"

# Prove the agent starts on BOTH transports, not just one. The prebuilt binary
# this replaced passed a udp4 probe and then died with "*** stack smashing
# detected ***" the moment it opened a tty, so probing one transport proves
# nothing about the other. socat gives the serial probe a pty to open; without a
# device the serial agent exits immediately for the wrong reason.
# timeout -k because the agent ignores SIGTERM.
RUN apt-get update && apt-get install -y --no-install-recommends socat \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && . /uros_ws/install/setup.sh \
 && socat -d -d pty,raw,echo=0,link=/tmp/probe_tty pty,raw,echo=0 & \
    sleep 2; \
    for probe in "udp4 --port 18888" "serial --dev /tmp/probe_tty -b 921600"; do \
        out=$(timeout -k 2 6 ros2 run micro_ros_agent micro_ros_agent $probe 2>&1 || true); \
        echo "[probe] $probe"; echo "$out" | tail -2; \
        if echo "$out" | grep -qi "stack smashing\|error while loading\|Aborted"; then \
            echo "[probe] FAILED: the agent does not survive '$probe'"; exit 1; \
        fi; \
    done

ENV ROS_WS_SETUP=/opt/lino_ws/setup.bash \
    UROS_SETUP=/uros_ws/install/setup.bash

# The user's robot configs are a bind mount at /config, seeded from
# config/reference by scripts/cockpit_paths.py on first start.
RUN mkdir -p /config /ws/firmware/prebuilt /ws/logs /ws/maps

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENV COCKPIT_CONFIG_DIR=/config
EXPOSE 8000 9090
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["python3", "web/backend/main.py"]
