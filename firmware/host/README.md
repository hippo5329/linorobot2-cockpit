# The host target: the robot computer as a micro-ROS client

A board is a configuration, not a build — and this takes that one step further. The
**host target** runs the firmware's own model and its own micro-ROS transport on the robot
computer, speaking **UDP4 to `micro_ros_agent` exactly as an ESP32 does over Wi-Fi**. No
microcontroller, no USB cable, and — unlike `scripts/fake_base_node.py` — no bypassing of
micro-ROS.

## Why, given `fake_base_node.py` already exists

`fake_base_node.py` is an rclpy node: it publishes `odom/unfiltered` and `imu/data` straight
into DDS. Its own skill documentation is explicit about the cost — it removes *"micro-ROS,
the serial and Wi-Fi transports, the board's timing, its 4 KB env partition"* — which is why
a green boardless matrix cannot gate a cut, and why a boardless pass beside a red bench leg
only tells you the fault is in the transport or the board.

The host target closes the largest of those gaps: the XRCE session, the UDP4 datagrams, the
512-byte MTU and its fragmentation, the agent round trip. What it still does **not** cover is
the board's loop timing, the real flash, and the serial link. So it is a promotion, not a
replacement: the gate stays on hardware.

## What is shimmed, and what is not

The firmware sources are compiled **unmodified**. That is the whole point — a second copy of
the wheel model is the one thing that would make the instrument lie, and
`tests/test_fake_base_node.py` exists to prevent exactly that. Rather than port the firmware
to Linux, `shim/` emulates the small Arduino surface it actually uses, measured rather than
guessed:

| firmware source | what it needs from Arduino |
| --- | --- |
| `kinematics.h` | nothing (includes `Arduino.h`, calls none of it) |
| `PID.h` | nothing |
| `odometry.h` | nothing — its `micro_ros_utilities`/`nav_msgs` includes come from the micro-ROS client library |
| `fake_wheel.h` | `micros()`, `random()`, `map()` |
| `uros_transport.cpp` | `Serial.print*`, and nine `WiFiUDP` methods |

`shim/WiFiUdp.h` implements those nine over a POSIX socket, so
`firmware/common/lib/uros_transport/uros_transport.cpp` compiles as it is and the host takes
the **same `transport=udp4` branch a board takes**, reading the same `agent_ip` / `agent_port`
keys. Anything a future firmware change needs will fail to *compile* here, which is the right
failure — a silent divergence between board and host is what this target exists to avoid.

The env comes from the same place too: `scripts/mcu_env.py` generates the 4 KB blob from
`config.yaml`, the firmware parses it in place from a memory-mapped pointer, and on the host
that pointer is an `mmap()`ed file instead of a flash partition. Same generator, same keys,
same parser, and `tests/test_env_contract.py` keeps guarding both.

## `probe/` — and why it is here rather than in a scratch directory

`probe/` is a minimal micro-ROS UDP4 client. It exists because the obvious shortcut is a trap.

**`micro_ros_setup`'s `host` platform does not build a micro-ROS client.** It builds the demo
*sources* against the host's ordinary ROS 2 stack: `ldd` on its `int32_publisher` shows
`librmw_implementation.so` from `/opt/ros`, and the binary published a topic that
`ros2 topic list` and `ros2 topic echo` both saw **with no agent running at all**. The
`RMW_UXRCE_TRANSPORT_UDP` in that workspace's `config.h` belongs to the separately built
`rmw_microxrcedds` package, which that binary never loads. Measured 2026-09-24: a completely
convincing green that said nothing whatsoever about micro-ROS.

So the probe links `rmw_microxrcedds` and `microxrcedds_client` explicitly, and it ships with
its control:

```
CONTROL  no agent  -> rclc_init must FAIL       (no far end, no session)
POSITIVE agent up  -> session established, publishes, a ROS 2 subscriber receives
```

**A transport test that passes without the far end is not a transport test.** Run both halves
or neither.

Verified 2026-09-24 on an isolated NAT subnet (a docker bridge, `172.18.0.0/16`):

```
create_client      | client_key: 0x419E940C, session_id: 0x81
establish_session  | session established | address: 127.0.0.1:58497
create_participant / create_topic / create_publisher / create_datawriter
ros2 topic echo /lino_host_probe  ->  data: 9, 10, 11, 12
```

## Building it

The client library is not in the image; it is built once into its own workspace.

**Not `/uros_ws`.** The robot image already ships a micro-ROS workspace there — that is where
`micro_ros_agent` lives, and `docker/Dockerfile` keeps that exact path on purpose because
colcon bakes its build prefix into the generated setup scripts. Mounting over `/uros_ws`
shadows the agent, and for the same baked-prefix reason a workspace cannot be relocated by
remounting: it has to be **built** at the path it will be used at. Use `/hosturos_ws`.

Give the agent, the client and the `ros2` CLI **separate shells**. The host client workspace
carries micro-ROS builds of the message packages and `rmw_microxrcedds`; sourcing it into the
agent's or the CLI's environment hands them the wrong middleware.

## Status

Landed: the shim, the probe, and the verified UDP4 transport. Still to do: back `mcu_env` with
an `mmap`ed blob, compile the model headers against the shim, and rebuild `rmw_microxrcedds`
with `RMW_UXRCE_TRANSPORT=custom` so the firmware's own four transport functions are the ones
under test rather than the rmw's built-in UDP socket.
