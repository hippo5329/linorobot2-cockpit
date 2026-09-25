#!/usr/bin/env bash
# Build the micro-ROS client workspace the host target links (firmware/host).
#
#   build_client_ws.sh <workspace> [distro]      e.g. /opt/hosturos_ws jazzy
#
# The workspace must be built AT the path it will be used from -- colcon bakes
# its prefix into the setup scripts -- and it must not be /uros_ws, where the
# image keeps micro_ros_agent (see README.md). Needs ROS sourced, git, colcon.
#
# Same steps as micro_ros_setup's host platform (config/host/generic/build.sh),
# without micro_ros_setup itself, which has no lyrical branch: the typesupports
# first, then the foundational message packages so every later interface
# package finds THEIR microxrcedds typesupport rather than /opt/ros's, then the
# rest. The meta is the board's (client_meta.py): a host client configured
# differently from a board is not an instrument for the board.
set -euo pipefail
WS=${1:?workspace path}
DISTRO=${2:-${ROS_DISTRO:?ROS_DISTRO not set}}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$WS/src"
cd "$WS"
grep -vE '^\s*(#|$)' "$HERE/client.repos.txt" | while read -r dir url branch fallback; do
  [ -d "src/$dir/.git" ] && continue
  want=$branch; [ "$branch" = DISTRO ] && want=$DISTRO
  if ! git ls-remote --exit-code --heads "$url" "$want" >/dev/null 2>&1; then
    [ -n "${fallback:-}" ] || { echo "no branch $want in $url"; exit 1; }
    echo "[client] $dir: no $want branch, using $fallback"
    want=$fallback
  fi
  git clone -q --depth 1 -b "$want" "$url" "src/$dir"
done
touch src/uros/rclc/rclc_examples/COLCON_IGNORE 2>/dev/null || true
touch src/uros/rclc/rclc_lifecycle/COLCON_IGNORE 2>/dev/null || true
python3 "$HERE/client_meta.py" "$HERE/host.meta" "$HERE/../esp32.meta" > src/colcon.meta
ARGS=(--metas src --cmake-args -DBUILD_TESTING=OFF -DBUILD_SHARED_LIBS=ON)
colcon build --packages-up-to rosidl_typesupport_microxrcedds_c "${ARGS[@]}"
colcon build --packages-up-to rosidl_typesupport_microxrcedds_cpp "${ARGS[@]}"
set +u; . install/local_setup.bash; set -u
colcon build --packages-select builtin_interfaces unique_identifier_msgs service_msgs action_msgs "${ARGS[@]}"
set +u; . install/local_setup.bash; set -u
colcon build "${ARGS[@]}"
grep -q "^#define RMW_UXRCE_TRANSPORT_CUSTOM" install/rmw_microxrcedds/include/rmw_microxrcedds_c/config.h \
  || { echo "rmw_microxrcedds is not RMW_UXRCE_TRANSPORT=custom"; exit 1; }
echo "[client] $WS built for $DISTRO"
