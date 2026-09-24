#!/usr/bin/env bash
# The host target as a micro-ROS MCU: the firmware's OWN env parser and its OWN
# four transport functions, on Linux, over UDP4 to micro_ros_agent.
#
# What makes this different from the previous proof: the rmw is rebuilt
# RMW_UXRCE_TRANSPORT=custom. With RMW_UXRCE_TRANSPORT=udp the rmw owns a UDP
# socket of its own and rmw_uros_options_set_udp_address() configures it -- the
# firmware's four functions would be installed and NEVER CALLED, and the run
# would pass while testing nothing of ours. `custom` removes that socket, so
# every byte must go through uros_transport.cpp.
#
# Three legs, and all three must hold:
#   CONTROL-DEAD    env names a port with no agent -> no session
#   CONTROL-SERIAL  env says transport=serial      -> refused, rc=2 (the env decides)
#   POSITIVE        env names the live agent        -> session + topic on the ROS 2 side
# Both controls are driven by a real env image from scripts/mcu_env.py, so the
# env path is exercised whichever way the leg is meant to go.
set -uo pipefail
# Overridable so this runs from a clone as well as from the bench.
#   SRC  the repo to compile the firmware sources from (default: this checkout)
#   WS   where the micro-ROS host client workspace lives. It must be BUILT at the
#        path it is used at -- colcon bakes its prefix into the setup scripts, so a
#        workspace cannot be relocated by remounting -- and it must NOT be
#        /uros_ws, which is where the image already keeps micro_ros_agent.
IMG=${IMG:-hippo5329/linorobot2-cockpit-robot:jazzy}
NET=${NET:-urosnet_hostmcu}
WS=${WS:-/srv/hosturos_ws}
SRC=${SRC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}
echo "=== repo $SRC"
[ -f "$SRC/scripts/mcu_env.py" ] || { echo "SRC=$SRC is not the repo root"; exit 1; }
[ -d "$WS/install/rmw_microxrcedds" ] || {
  echo "no micro-ROS host client workspace at $WS -- see firmware/host/README.md"; exit 1; }
echo "=== $(date -u +%H:%M:%S) isolated NAT subnet"
docker network create --driver bridge "$NET" >/dev/null 2>&1 || true
echo "subnet $(docker network inspect -f '{{range .IPAM.Config}}{{.Subnet}}{{end}}' "$NET")"

docker run --rm --name uros_hostmcu --network "$NET" \
  -e ROS_DOMAIN_ID=73 --user 0:0 --entrypoint bash \
  -v "$WS":/hosturos_ws -v "$SRC":/src \
  "$IMG" -lc '
set -eo pipefail
L=/hosturos_ws/proof-logs; mkdir -p $L
source /opt/ros/jazzy/setup.bash

RMWCFG=/hosturos_ws/install/rmw_microxrcedds/include/rmw_microxrcedds_c/config.h
echo "=== rmw transport BEFORE:"
grep -E "^#define RMW_UXRCE_TRANSPORT_(UDP|CUSTOM)|undef RMW_UXRCE_TRANSPORT_CUSTOM" $RMWCFG || true

if ! grep -q "^#define RMW_UXRCE_TRANSPORT_CUSTOM" $RMWCFG; then
  echo "=== switching rmw_microxrcedds to RMW_UXRCE_TRANSPORT=custom and rebuilding"
  python3 - <<PY
import json
p = "/hosturos_ws/src/colcon.meta"
m = json.load(open(p))
a = m["names"]["rmw_microxrcedds"]["cmake-args"]
a = [x for x in a if not x.startswith("-DRMW_UXRCE_TRANSPORT=")]
a.insert(0, "-DRMW_UXRCE_TRANSPORT=custom")
# The rmw ships a test calling rmw_uros_discover_agent(), which is agent
# autodiscovery over UDP multicast and does not exist under custom. That is an
# upstream test of a feature we deliberately remove, not a test of ours, and it
# fails to COMPILE -- so the package cannot build with testing left on.
if "-DBUILD_TESTING=OFF" not in a:
    a.append("-DBUILD_TESTING=OFF")
m["names"]["rmw_microxrcedds"]["cmake-args"] = a
json.dump(m, open(p, "w"), indent=4)
print("colcon.meta rmw args:", a[:4])
PY
  cd /hosturos_ws
  source install/local_setup.bash
  # --cmake-clean-cache: the transport is a cached CMake variable, so a plain
  # rebuild would keep udp and print a success that changed nothing.
  colcon build --packages-select rmw_microxrcedds --cmake-clean-cache \
      --metas /hosturos_ws/src/colcon.meta 2>&1 | tail -25
fi

echo "=== rmw transport AFTER:"
grep -E "^#define RMW_UXRCE_TRANSPORT_(UDP|CUSTOM|IPV4)" $RMWCFG || true
grep -q "^#define RMW_UXRCE_TRANSPORT_CUSTOM" $RMWCFG || { echo "RMW IS NOT CUSTOM - stop"; exit 1; }

# ---------- build the probe against the FIRMWARE tree ----------
echo
echo "=== building the probe from the firmware sources"
rm -rf /hosturos_ws/mcu_ws && mkdir -p /hosturos_ws/mcu_ws/src
cp -r /src/firmware/host/probe /hosturos_ws/mcu_ws/src/lino_host_probe
cd /hosturos_ws/mcu_ws
source /hosturos_ws/install/local_setup.bash
colcon build --packages-select lino_host_probe \
    --cmake-args -DFIRMWARE_ROOT=/src/firmware 2>&1 | tail -20
PROBE=/hosturos_ws/mcu_ws/install/lino_host_probe/lib/lino_host_probe/lino_host_probe
[ -x "$PROBE" ] || { echo "NO PROBE BINARY"; exit 1; }
echo "=== rmw the probe links:"
ldd "$PROBE" | grep -iE "microxrce|rmw" | head -4

# ---------- the env images, from the real flasher ----------
echo
echo "=== env images from scripts/mcu_env.py (the same 4096 bytes a board is flashed)"
cd /src
for spec in "live:8888:udp4" "dead:8899:udp4" "serialcfg:8888:serial" "altport:8877:udp4"; do
  name=${spec%%:*}; rest=${spec#*:}; port=${rest%%:*}; tr=${rest##*:}
  python3 scripts/mcu_env.py build --params config/reference/yb_eet01_config.yaml \
      --secrets /nonexistent.yaml --host-ip 127.0.0.1 --sensors sim \
      --set transport=$tr --set agent_port=$port \
      --out $L/env_$name.bin >/dev/null 2>&1
  echo "  env_$name.bin  transport=$tr agent_port=$port  $(stat -c%s $L/env_$name.bin) bytes"
done

echo
echo "################ CONTROL-SERIAL: env says transport=serial ################"
rc=0
timeout 30 "$PROBE" $L/env_serialcfg.bin > $L/c_serial.log 2>&1 || rc=$?
cat $L/c_serial.log | head -6
if [ $rc -eq 2 ]; then echo "CONTROL-SERIAL OK: refused (rc=2) - the ENV chose the branch"
else echo "CONTROL-SERIAL FAILED: rc=$rc, expected 2"; fi

echo
echo "################ CONTROL-DEAD: udp4 at a port with no agent ################"
timeout 40 "$PROBE" $L/env_dead.bin > $L/c_dead.log 2>&1 || true
head -8 $L/c_dead.log
if grep -aq "SESSION ESTABLISHED" $L/c_dead.log; then
  echo "CONTROL-DEAD FAILED: a session was established with NO agent"
else
  echo "CONTROL-DEAD OK: no agent, no session (the far end is required)"
fi

echo
echo "################ POSITIVE: agent on udp4 :8888 ################"
bash -lc "source /opt/ros/jazzy/setup.bash; source /uros_ws/install/local_setup.bash; \
          exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6" \
  > $L/agent.log 2>&1 &
AGENT=$!
sleep 8
timeout 60 "$PROBE" $L/env_live.bin > $L/probe.log 2>&1 &
PROBE_PID=$!

# The echo has to OVERLAP the probe. A datawriter exists only while its client
# does, so echoing after the publish loop ends finds a topic with no publisher and
# blocks -- which looks exactly like a transport that never worked.
sleep 4
bash -lc "source /opt/ros/jazzy/setup.bash; source /uros_ws/install/local_setup.bash; \
          timeout 20 ros2 topic echo --once /lino_host_probe" > $L/echo.log 2>&1 &
ECHO_PID=$!
wait $PROBE_PID 2>/dev/null || true
wait $ECHO_PID 2>/dev/null || true

echo "--- probe output:"; head -10 $L/probe.log
echo "--- agent saw:"
grep -aE "establish_session|create_participant|create_topic|create_publisher|create_datawriter" \
    $L/agent.log | head -8 || echo "  (none)"
echo "--- the topic, received on the ROS 2 side:"
head -4 $L/echo.log || true
if grep -aq "^data:" $L/echo.log; then
  echo "POSITIVE OK: a ROS 2 subscriber received the message"
else
  echo "POSITIVE INCOMPLETE: the agent saw the client but no ROS 2 subscriber got data"
fi
kill $AGENT 2>/dev/null || true

echo
echo "########### POSITIVE-ALTPORT: the env decides the port, not the rmw ###########"
# THE DISCRIMINATING LEG. rmw_microxrcedds was compiled
# RMW_UXRCE_DEFAULT_UDP_PORT=8888 while its transport was udp, and host_probe.cpp never
# calls rmw_uros_options_set_udp_address(). So a socket owned by the rmw could only
# ever reach 8888, and an agent on 8877 would be unreachable. A session here proves
# the port came out of the ENV IMAGE and was used by uros_transport.cpp -- not that
# a custom callback merely exists.
#
# And under custom the rmw has NO UDP configuration left at all: switching the
# transport #undefs RMW_UXRCE_TRANSPORT_UDP, which is what guards the DEFAULT_UDP_IP
# and DEFAULT_UDP_PORT definitions. Assert that, because it is the structural half
# of the same claim -- there is no second socket to be confused with ours.
if grep -qE "^ *#define RMW_UXRCE_DEFAULT_UDP_(IP|PORT)" $RMWCFG; then
  echo "  UNEXPECTED: the rmw still defines a default UDP address"
  grep -nE "RMW_UXRCE_DEFAULT_UDP_(IP|PORT)" $RMWCFG
else
  echo "  rmw has no default UDP address compiled in (transport is custom) - good"
fi
bash -lc "source /opt/ros/jazzy/setup.bash; source /uros_ws/install/local_setup.bash; \
          exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 8877 -v6" \
  > $L/agent_alt.log 2>&1 &
AGENT2=$!
sleep 8
timeout 40 "$PROBE" $L/env_altport.bin > $L/probe_alt.log 2>&1 || true
head -5 $L/probe_alt.log
if grep -aq "SESSION ESTABLISHED" $L/probe_alt.log && \
   grep -aq "establish_session" $L/agent_alt.log; then
  echo "ALTPORT OK: session on 8877 - the env supplied the port through uros_transport.cpp"
else
  echo "ALTPORT FAILED: no session on 8877"
fi
kill $AGENT2 2>/dev/null || true

echo "HOST_MCU_PROOF_DONE"
' 2>&1
echo "HOST_MCU_EXIT rc=$? $(date -u +%H:%M:%S)"
docker network rm "$NET" >/dev/null 2>&1 || true
