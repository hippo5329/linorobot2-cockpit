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

# Nav2 separately and best-effort: it is not published for every distro yet --
# there is no ros-lyrical-navigation2 at the time of writing -- and bringup and
# topic verification do not touch it. Failing the whole image over a package
# only nav2.launch.py needs would block the lyrical half of the test matrix for
# no reason. The absence is announced rather than silent.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
    || echo "[build] Nav2 is not in the ${ROS_DISTRO} archive — bringup and SLAM work, nav2.launch.py will not" \
    ; rm -rf /var/lib/apt/lists/*

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
