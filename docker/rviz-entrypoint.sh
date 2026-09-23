#!/usr/bin/env bash
# RViz on the host's display, against the containerised stack.
#
#   --config <name>   one of the configs this repo ships (teleop, slam,
#                     navigation), or a path. Anything else is passed to rviz2
#                     unchanged, so `docker compose run rviz -d /rviz/mine.rviz`
#                     still works.
set -euo pipefail

# set +u around the source, and only around it. ROS's setup.bash reads
# AMENT_TRACE_SETUP_FILES and friends without defaulting them, so `set -u`
# makes sourcing it a fatal "unbound variable" before the script does anything
# -- the entrypoint died on its own first line.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

# The config directory is a volume so a layout saved from inside RViz survives
# the container and never lands in the host's $HOME -- the cockpit does not
# write to the machine it runs on. Seed it once from the baked-in copies.
CONFIG_DIR=${COCKPIT_RVIZ_DIR:-/rviz}
if [ -d /opt/cockpit-rviz ] && [ -w "$CONFIG_DIR" ]; then
  for f in /opt/cockpit-rviz/*.rviz; do
    [ -e "$f" ] || continue
    [ -e "$CONFIG_DIR/$(basename "$f")" ] || cp "$f" "$CONFIG_DIR/"
  done
fi

if [ "${1:-}" = "--config" ]; then
  name=${2:-navigation}
  shift 2
  case "$name" in
    */*|*.rviz) cfg="$name" ;;
    *)          cfg="$CONFIG_DIR/$name.rviz" ;;
  esac
  if [ ! -f "$cfg" ]; then
    echo "rviz config not found: $cfg" >&2
    echo "available: $(ls "$CONFIG_DIR"/*.rviz 2>/dev/null | xargs -r -n1 basename | tr '\n' ' ')" >&2
    exit 1
  fi
  set -- -d "$cfg" "$@"
fi

# Say what it will try to attach to before it tries. A viewer that comes up
# empty is the normal failure here, and the three things that cause it --
# wrong domain, wrong RMW, missing Fast DDS profile -- are all invisible
# unless printed. See measure-from-inside-the-stack-env.
echo "[rviz] DISPLAY=${DISPLAY:-unset} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
     "RMW=${RMW_IMPLEMENTATION:-default}" \
     "FASTDDS_PROFILES=${FASTDDS_DEFAULT_PROFILES_FILE:-none}"
if [ -n "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ] && [ ! -r "${FASTDDS_DEFAULT_PROFILES_FILE}" ]; then
  echo "[rviz] WARNING: the Fast DDS profile is set but not readable here." \
       "This viewer will not see a stack that uses it, and will simply stay empty." >&2
fi

exec rviz2 "$@"
