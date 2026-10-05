#!/usr/bin/env bash
# Build a micro-ROS app for the Arduino UNO Q's STM32U585 (Zephyr, experiment).
#
#   firmware/zephyr/unoq/build.sh [workdir] [app]   default workdir: ./zephyr-work
#
#   app  fw    the linorobot2 firmware itself, unmodified, on a shim (default)
#        base  the hand-written 2WD base controller (the stepping stone to fw)
#        app   the module's int32 publisher sample, the link test
#
#   fw also needs LINO_LIBDEPS=<dir> from firmware/host/fetch_libdeps.py (I2Cdevlib,
#   SparkFun BNO080): it is fetched with pio, which this build does not carry.
#
# Everything runs in the Zephyr CI image (it carries SDK 1.0.1). The workdir is mounted
# as /work, which is the path the module patch and the app CMakeLists name.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORK=$(mkdir -p "${1:-$PWD/zephyr-work}" && cd "${1:-$PWD/zephyr-work}" && pwd)
IMAGE=zephyrprojectrtos/ci:v0.29.4
ZEPHYR_REV=v4.4.2                 # first releases with boards/arduino/uno_q
MODULE_REV=87dbe3a                # micro_ros_zephyr_module, jazzy branch (tested upstream on Zephyr 4.0/4.1 only)

APP=${2:-fw}
[ -d "$HERE/$APP" ] || { echo "no app '$APP' in $HERE" >&2; exit 2; }
SRC="$WORK/unoq_$APP"
rm -rf "$SRC" "$WORK/compat"
cp -r "$HERE/$APP" "$SRC"
cp -r "$HERE/compat" "$WORK/compat"
EXTRA_CMAKE=""
if [ "$APP" = fw ]; then
  : "${LINO_LIBDEPS:?fw needs LINO_LIBDEPS=<fetch_libdeps.py directory>}"
  rm -rf "$WORK/firmware" "$WORK/libdeps"
  mkdir -p "$WORK/firmware"
  ( cd "$HERE/../.." && tar -c --exclude=zephyr --exclude=prebuilt --exclude=.pio . ) | tar -x -C "$WORK/firmware"
  cp -r "$LINO_LIBDEPS" "$WORK/libdeps"
  python3 "$HERE/../../../scripts/gen_firmware_header.py" --mcu unoq --no-embed-secrets \
      --out "$WORK/unoq_header.h" >/dev/null
  REV=${FW_GIT_REV:-$(git -C "$HERE" rev-parse --short=7 HEAD 2>/dev/null || echo unknown)}   # from the caller when the tree is a copy
  EXTRA_CMAKE="-DFIRMWARE_ROOT=/work/firmware -DLINO_HEADER=/work/unoq_header.h -DLINO_LIBDEPS=/work/libdeps -DFW_GIT_REV=$REV"
fi
# The base controller compiles the Arduino firmware's kinematics and PID as they are.
if [ "$APP" = base ]; then
  mkdir -p "$SRC/common"
  cp -r "$HERE/../../common/lib/kinematics" "$HERE/../../common/lib/pid" "$SRC/common/"
fi
if [ ! -d "$WORK/micro_ros_zephyr_module" ]; then
  git clone -q -b jazzy https://github.com/micro-ROS/micro_ros_zephyr_module.git "$WORK/micro_ros_zephyr_module"
fi
( cd "$WORK/micro_ros_zephyr_module" && git checkout -q -- . && git checkout -q "$MODULE_REV" \
  && git apply "$HERE/patches/micro_ros_zephyr_module-zephyr44-unoq.patch" \
  && rm -rf modules/libmicroros/micro_ros_src/build modules/libmicroros/micro_ros_src/install modules/libmicroros/micro_ros_src/log \
  && rm -rf modules/libmicroros/include modules/libmicroros/libmicroros.a )
# ^ include/ too: libmicroros.mk does `cp -R install/include include`, which copies INTO an
#   existing include/ (as include/include) and leaves the previous build's headers in force.
#   With the MTU changed that is a layout mismatch -- uxrCustomTransport embeds an MTU-sized
#   buffer -- and the transport's args pointer is read from the wrong offset (a bus fault at
#   the first ping, 2026-10-05).

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
west build -p always -b arduino_uno_q -d /work/build-$APP /work/unoq_$APP -- $EXTRA_CMAKE
"
ls -la "$WORK/build-$APP/zephyr/zephyr.bin" "$WORK/build-$APP/zephyr/zephyr.elf"
