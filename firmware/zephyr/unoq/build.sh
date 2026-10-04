#!/usr/bin/env bash
# Build the micro-ROS app for the Arduino UNO Q's STM32U585 (Zephyr, experiment).
#
#   firmware/zephyr/unoq/build.sh [workdir]      default workdir: ./zephyr-work
#
# Everything runs in the Zephyr CI image (it carries SDK 1.0.1). The workdir is mounted
# as /work, which is the path the module patch and the app CMakeLists name.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORK=$(mkdir -p "${1:-$PWD/zephyr-work}" && cd "${1:-$PWD/zephyr-work}" && pwd)
IMAGE=zephyrprojectrtos/ci:v0.29.4
ZEPHYR_REV=v4.4.2                 # first releases with boards/arduino/uno_q
MODULE_REV=87dbe3a                # micro_ros_zephyr_module, jazzy branch (tested upstream on Zephyr 4.0/4.1 only)

rm -rf "$WORK/unoq_uros" "$WORK/compat"
cp -r "$HERE/app" "$WORK/unoq_uros"
cp -r "$HERE/compat" "$WORK/compat"
if [ ! -d "$WORK/micro_ros_zephyr_module" ]; then
  git clone -q -b jazzy https://github.com/micro-ROS/micro_ros_zephyr_module.git "$WORK/micro_ros_zephyr_module"
fi
( cd "$WORK/micro_ros_zephyr_module" && git checkout -q -- . && git checkout -q "$MODULE_REV" \
  && git apply "$HERE/patches/micro_ros_zephyr_module-zephyr44-unoq.patch" \
  && rm -rf modules/libmicroros/micro_ros_src/build modules/libmicroros/micro_ros_src/install modules/libmicroros/micro_ros_src/log )

docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ZEPHYR_SDK_INSTALL_DIR=/opt/toolchains/zephyr-sdk-1.0.1 \
  -v "$WORK:/work" -w /work "$IMAGE" bash -c "
set -e
[ -d zephyrproject/.west ] || west init -m https://github.com/zephyrproject-rtos/zephyr --mr $ZEPHYR_REV zephyrproject
cd zephyrproject
west config manifest.project-filter -- '-.*,+cmsis.*,+hal_stm32'
west update --narrow -o=--depth=1 >/dev/null
[ -d /work/pyvenv ] || python3 -m venv --system-site-packages /work/pyvenv
. /work/pyvenv/bin/activate
pip install -q catkin_pkg lark empy==3.3.4 colcon-common-extensions
west build -p always -b arduino_uno_q -d /work/build-uros /work/unoq_uros
"
ls -la "$WORK/build-uros/zephyr/zephyr.bin"
