# The host target: the robot computer as a micro-ROS client

A board is a configuration, not a build — and this takes that one step further. The
**host target** runs the firmware's own model and its own micro-ROS transport on the robot
computer, speaking **UDP4 to `micro_ros_agent` exactly as an ESP32 does over Wi-Fi**. No
microcontroller, no USB cable, and — unlike `scripts/sim_base_node.py` — no bypassing of
micro-ROS.

## Why, given `sim_base_node.py` already exists

`sim_base_node.py` is an rclpy node: it publishes `odom/unfiltered` and `imu/data` straight
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
`tests/test_sim_base_node.py` exists to prevent exactly that. Rather than port the firmware
to Linux, `shim/` emulates the small Arduino surface it actually uses, measured rather than
guessed:

| firmware source | what it needs from Arduino |
| --- | --- |
| `kinematics.h` | nothing (includes `Arduino.h`, calls none of it) |
| `PID.h` | nothing |
| `odometry.h` | nothing — its `micro_ros_utilities`/`nav_msgs` includes come from the micro-ROS client library |
| `sim_wheel.h` | `micros()`, `random()`, `map()` |
| `uros_transport.cpp` | the `Print`/`Stream` hierarchy, `Serial`, `IPAddress`, and nine `WiFiUDP` methods |
| `mcu_env.cpp` | `Serial.printf`, `IPAddress::fromString` |
| `diag.cpp` | `Stream *` with `print`/`printf`, `millis()` |

`shim/WiFiUdp.h` implements those nine over a POSIX socket, so
`firmware/common/lib/uros_transport/uros_transport.cpp` compiles as it is and the host takes
the **same `transport=udp4` branch a board takes**, reading the same `agent_ip` / `agent_port`
keys. Anything a future firmware change needs will fail to *compile* here, which is the right
failure — a silent divergence between board and host is what this target exists to avoid.

Two details of the shim are load-bearing rather than cosmetic. It reproduces Arduino's real
`Print` → `Stream` class hierarchy, because the firmware passes `Stream *` around (`diag.cpp`
holds its output as one; `uros_transport.cpp`'s serial branch casts `transport->args` back to
one) — a shim offering only a concrete `Serial` would have forced a host `#ifdef` into each of
those files, and an `#ifdef` in the firmware is precisely the divergence this target exists to
*detect* rather than create. And `IPAddress` lives in `Arduino.h`, not in `WiFiUdp.h`, because
that is where the real cores keep it and `mcu_env.h` relies on exactly that: it declares
`envIP()` while including only `<Arduino.h>`.

### The env is the same 4096 bytes a board is flashed

`scripts/mcu_env.py build --out env.bin` already writes the exact image an ESP32 `env`
partition or an RP2 top sector receives. The host target **maps that file**: `mcu_env.cpp`
gains a third loader beside the ESP32 and RP2040 ones — `open` + `fstat` + `mmap(PROT_READ)`,
at `$LINO_ENV_BIN` (default `./env.bin`) — and the CRC32 check, the entry walk and every
accessor are the ones the board runs.

`mmap` rather than `read()` is deliberate: both other backends parse in place out of a
read-only mapping, and a heap copy here would differ in the ways that matter to the parser. A
copy can be NUL-terminated and sized to its content, where flash is `0xFF`-padded to the full
sector and is walked against `ENV_DATA_LEN`. The mapping keeps the host on the same code path
rather than a kinder one.

That shared blob is the point. A host client with a YAML reader of its own would agree with the
board only for as long as nobody edited either. `tests/test_host_env_blob.py` asserts the two
ends still meet — it writes an image with the Python writer, compiles the firmware's C++ reader
against this shim, runs it, and requires every key back, plus the three refusals (missing,
short, bad CRC). A short file matters more than it looks: it would map with a zero-filled tail,
and a zero reads as the empty entry that ends the list, so half an env would parse as an empty
one and the board would come up on compiled-in defaults, quietly.

## `probe/` — and why it is here rather than in a scratch directory

`probe/` is a minimal micro-ROS UDP4 client. It exists because the obvious shortcut is a trap.

**`micro_ros_setup`'s `host` platform does not build a micro-ROS client.** It builds the demo
*sources* against the host's ordinary ROS 2 stack: `ldd` on its `int32_publisher` shows
`librmw_implementation.so` from `/opt/ros`, and the binary published a topic that
`ros2 topic list` and `ros2 topic echo` both saw **with no agent running at all**. The
`RMW_UXRCE_TRANSPORT_UDP` in that workspace's `config.h` belongs to the separately built
`rmw_microxrcedds` package, which that binary never loads. Measured 2026-09-24: a completely
convincing green that said nothing whatsoever about micro-ROS.

So the probe links `rmw_microxrcedds` and `microxrcedds_client` explicitly — and, critically,
**the rmw is rebuilt `RMW_UXRCE_TRANSPORT=custom`**. That is what puts the firmware's code
under test rather than beside it. With the default `RMW_UXRCE_TRANSPORT=udp` the rmw owns a UDP
socket of its own and `rmw_uros_options_set_udp_address()` configures it: the four functions in
`uros_transport.cpp` would be *installed and never called*, and a `transport=udp4` regression in
the firmware would pass. Under `custom` there is no second socket — switching the transport also
`#undef`s `RMW_UXRCE_TRANSPORT_UDP`, which is what guards `RMW_UXRCE_DEFAULT_UDP_IP` and
`_PORT`, so the rmw has no UDP address compiled in at all. Every byte must go through ours.

(One upstream wrinkle: `rmw_microxrcedds` ships a test calling `rmw_uros_discover_agent()`,
agent autodiscovery over UDP multicast, which does not exist under `custom` and fails to
*compile*. It is upstream's test of a feature we deliberately remove, so the package is built
`-DBUILD_TESTING=OFF`.)

`probe/proof.sh` runs the whole thing on an isolated NAT subnet (a docker bridge per leg, plus
its own `ROS_DOMAIN_ID`), and it has **four legs, none of which is optional**:

| leg | env image says | must |
| --- | --- | --- |
| `CONTROL-SERIAL` | `transport=serial` | be refused, `rc=2` — the **env** picks the branch |
| `CONTROL-DEAD` | `udp4`, port with no agent | fail `rclc_init` — no far end, no session |
| `POSITIVE` | `udp4`, the live agent | establish a session, publish, and a ROS 2 subscriber receives |
| `POSITIVE-ALTPORT` | `udp4`, port `8877` | establish a session — see below |

**A transport test that passes without the far end is not a transport test.** Both controls are
driven by real images from `scripts/mcu_env.py`, so the env path is exercised whichever way the
leg is meant to go.

`POSITIVE-ALTPORT` is the discriminating leg. The rmw was compiled
`RMW_UXRCE_DEFAULT_UDP_PORT=8888` back when its transport was `udp`, and `host_probe.cpp` never
calls `rmw_uros_options_set_udp_address()` — so a socket owned by the rmw could only ever reach
8888, and an agent on 8877 would be unreachable. A session there proves the port came *out of
the env image* and was used by `uros_transport.cpp`, not merely that a custom callback exists.

Verified 2026-09-24 on `172.18.0.0/16`:

```
rmw transport                      RMW_UXRCE_TRANSPORT_CUSTOM (no default UDP address)
probe links                        librmw_microxrcedds.so, libmicroxrcedds_client.so.2.4
CONTROL-SERIAL   rc=2              [uros] env says transport 'serial', host target is udp4 only
CONTROL-DEAD                       [rclc_init] Error in rcl_init  -> FAILED rc=1
POSITIVE         [env] 4096 bytes, CRC32 a9ffbfd2 OK -> [uros] transport udp4 -> 127.0.0.1:8888
                 establish_session / create_participant / create_topic / create_publisher
                 / create_datawriter        ros2 topic echo -> data: 9
POSITIVE-ALTPORT [uros] transport udp4 -> 127.0.0.1:8877  -> SESSION ESTABLISHED
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

**Landed and verified.** The env (`mcu_env.cpp`'s `mmap` loader on the flasher's own image),
the transport (the firmware's four functions, under `RMW_UXRCE_TRANSPORT=custom`), the shim,
`probe/proof.sh` with all four legs, and `tests/test_host_env_blob.py` +
`tests/test_host_target_is_not_a_second_copy.py` guarding both from the desk — no container, no
board.

**The application landed 2026-09-26, and it is the Sim MCU.** `app/` compiles `src/main.cpp`,
every `common/lib` source and the tools, unmodified, plus the boards' own third-party
libraries (`fetch_libdeps.py` reads `lib_deps` from `platformio_base.ini`), against a shim that
grew the Arduino surface a whole firmware needs: GPIO that keeps its level, an I2C and SPI bus
with nothing on it (a NACK, so the sensor factory falls back to the simulated drivers exactly as
a bare board's does), `Servo`, and a `WiFi.h` whose link is up. Two firmware lines name the
host, both architecture branches beside the ESP32 and RP2 ones that were already there: the
encoder wrapper (no quadrature hardware) and `sim_ld19.h`'s UDP sink (a POSIX socket, the same
reason `uros_transport.cpp` gives it udp4). `scripts/host_firmware.py` writes its env with the
flasher's own functions and `bringup.launch.py` runs it for the base controller `sim`, beside
`micro_ros_agent udp4` and `ldlidar_stl_ros2` in `udp_server` mode.

The client workspace is now built by `build_client_ws.sh`, from `client.repos.txt` with the
boards' branch rule (the distro's branch, else `rolling`) and the **board's own meta**
(`client_meta.py` merges `host.meta` with `../esp32.meta`). That was not optional: the first
whole-firmware run lost every `/odom/unfiltered`, because the workspace kept the client's
512-byte custom-transport MTU and a best-effort stream cannot fragment a ~720-byte Odometry -- the
exact fault `esp32.meta` fixed on the boards. The same run measured `/battery` at 0.495 Hz: it
had been published every 2 s on every board since the first commit.

First run, on a bench host in an isolated network: agent session with 7 publishers and 1 subscriber,
`/odom/unfiltered` and `/imu/data` 50.0 Hz, `/scan` 10.0 Hz through the driver, `/sonar` 8.3 Hz,
`/battery` 0.98 Hz, and `/cmd_vel` 0.2 m/s + 0.5 rad/s read back as 0.204 / 0.493. It still does
not cover the board's loop timing, the real flash, or the serial link, so **the gate stays on
hardware**.
