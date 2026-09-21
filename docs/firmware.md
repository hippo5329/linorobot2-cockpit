# Firmware: one image, several applications

Design notes and hard-won rules for `firmware/`.

**There is exactly one firmware project: `firmware/`.** The diagnostics are not separate builds any
more — they are applications inside that image, selected at boot by the `app` key of the env
partition (`base`, `test_sensors`, `test_motors`, `test_acc`, `i2c_detect`, `bno085_cal`,
`adc_calibrate`). Switching between them rewrites 4 KB of flash; it does not rebuild and, on ESP32,
does not reflash the application at all. Do **not** recreate a per-tool PlatformIO project.

Two rules make this free for the robot firmware, and both are load-bearing:
- **Each application is its own translation unit in its own namespace**, exposing nothing but a
  setup/loop pair wired into `firmware/src/tools/tools.cpp`. Several tools and `base` independently
  defined `imu_msg`, `battery_msg`, `setLed` and motor objects at file scope; linked together those
  are duplicate symbols.
- **No application owns hardware.** `Motor`/`Encoder` come from `hw_factory`, built in `setup()`.
  Constructed at file scope they would run at boot in *every* mode, touching motor pins before
  `initBoard()` in a robot whose user asked for a sensor scan. It also means a tool drives the
  wiring the env describes rather than whatever the header was generated for.

Measured cost of folding six applications in: **+8.9 KB RAM and +21 KB flash** over `base` alone
(esp32: 98,008 B RAM, 79% of the 1,310,720 B app slot).

**`blink` was deleted, not merged.** It existed as a standalone image so it could be flashed when the
main one would not run. With a single image there is nothing smaller to fall back TO, and flashing
the image that just failed is not a recovery — so the role was already gone. Keeping it as an app
would have kept the maintenance and lost the only thing that justified it: it answers nothing this
image does not, because `[app] <name>` goes out over serial at boot and `ledInit()`/`setLed` drive
the LED in every mode. A board that will not run the firmware is recovered through BOOTSEL / the
1200-baud touch and a reflash, which is what actually recovered one before.

### The board says what it is running, in one line, at boot
Everything that describes the ROBOT moved into the env partition and can be rewritten without a
compiler (see *The ROS 2 distro is a BUILD property* below). What the env deliberately cannot describe is the IMAGE — the ROS 2 distro is fixed
at link time and so is every line of code — so "which build is on this board?" had no answer from
the outside, and the only safe assumption was "not the one I just compiled". That assumption is what
made a reflash the first step of every run, and a reflash is the step that can brick an assembled
robot, needs the agent stopped and the bus free, and on an RP2350 can hang the board in the
1200-baud touch (docs/flashing.md).

So `firmware/src/main.cpp:printBanner()` prints, before anything else:

```text
[fw] linorobot2_hardware app=base distro=lyrical built=2026-09-16 git=6aa607f
```

`app` is the sub-firmware this boot selected, `distro` the ROS 2 distro it links against, `built` the
compile date, `git` the 7-character revision of the tree it came from — with a trailing `+` when that
tree had uncommitted edits, so a dirty build never reads as a clean revision. The last three arrive as
`-DFW_ROS_DISTRO` / `-DFW_GIT_REV` / `-DFW_BUILD_DATE` from **`firmware/common/build_stamp.py`**, a
`pre:` extra_script in `common/platformio_base.ini`.

**`distro` is in the banner because the revision alone cannot identify an image.** The release matrix
is four boards x two distros (see *The ROS 2 distro is a BUILD property* below), the two halves are built from the *same* source at the *same*
revision, and `board_microros_distro` is fixed at link time — so `git=` is identical on both and the
env partition cannot carry the difference. Without this field `mcu_probe.py` answered `up_to_date`
for a board carrying the jazzy image when a lyrical run asked for it, and the failure that follows
names nothing at all: the agent starts, the board boots, both look healthy, and the two never
discover each other because they do not speak the same micro-ROS. `build_stamp.py` reads the value
from `env.GetProjectOption("board_microros_distro")`; `distro_for_env()` in `mcu_probe.py` derives
the host's side of the comparison from the env name (bare = jazzy, `_lyrical` suffix = lyrical),
which is the same convention `one_click_pipeline.py:resolve_pio_env()` uses.

The field is **optional in the parser and never invented**. A board flashed before it existed prints
no `distro=`, parses fine, and reports `unstated` — the comparison is simply skipped for it. Guessing
the value this host happens to build would recreate exactly the false match the banner exists to
prevent. For the same reason `flash_mcu.py` records `distro` in the stamp **only when the image was
written**: an `--env-only` write must not restamp a property of an image it did not touch.

`build_stamp.py` reads the revision from `git rev-parse`, and failing that from **`firmware/.git_rev`**,
a one-line file the box seeders write. That fallback is not optional: every box gets its workspace as
`git archive HEAD`, which carries no `.git`, so without it every board built in a box would report
`git=unknown` and could never be recognised as carrying the current tree. The file must carry the same
`+` marker the host would compute, or a dirty tree makes every board look stale forever.

`scripts/mcu_probe.py` is the only parser of that line; `printBanner()` is the only writer. Keep the
`key=value` shape and keep the two together.

### A stale board is updated by default; `--no-auto-update` is what protects a robot in the field
`scripts/mcu_probe.py` asks three independent questions before anything is written — the RP2 USB
interface classes (is an application running at all?), the boot banner (which build?), and
`<config dir>/state/flashed/<env>_<port>.json`, this machine's record of its last flash and of
the env block that went with it. It lives with the config rather than in `$HOME` because the pipeline runs
as container-root and the cockpit's web backend as the container user: keyed on `~`, the two wrote
to different places and neither saw the other, so a board flashed seconds earlier still probed
as "this host has no record of flashing it". `LINO_STAMP_DIR` overrides. The verdict decides the cost:

| verdict | what a run does |
|---|---|
| `up_to_date` | nothing is written |
| `env_stale` / `env_unknown` | the 4 KB env block only — `flash_mcu.py --env-only`. The image is not touched |
| `stale` / `unknown` | **auto-update on** (the default): build and write the image. **Off**: nothing, and the run says so |
| `no_firmware` | image **and** env are written without being asked, under either setting |

**Auto-update is on by default because a bench board is supposed to track the tree.** The earlier
default was the opposite, and it cost a run every single time: the pipeline probed, printed "pass
`--flash`", and then brought up the whole stack against the OLD image — so the thing under test was
not the thing just edited, and nothing but the topic rates hinted at it. `--flash` had to be passed
on the re-run to reach the only outcome anyone wanted.

**`--no-auto-update` restores the old behaviour verbatim, and the reason it exists is unchanged:** a
flash is still the one step that can brick an assembled robot, needs the agent stopped and the bus
free, and on an RP2350 can hang the board in the 1200-baud touch (docs/flashing.md). What changed is the
assumption about who is holding the board. On a bench it is the developer who just compiled;
in the field it is a robot, and that is where `--no-auto-update` belongs. The Cockpit exposes both
as checkboxes: **"Auto-update firmware when stale"** (ticked) and **"Force firmware update"**
(unticked, `--flash`, which writes even over an `up_to_date` verdict).

Two things auto-update deliberately does **not** do:

- **It does not act on a probe that failed to run.** `probe_board()`'s fallback reports
  `verdict: unknown, firmware_differs: true` so a broken probe can never read as "nothing to do" —
  but it now also sets **`probe_failed`**, and auto-update skips it. Merged, the two would mean a box
  that failed to start, or any stderr from that command, silently rewrites a working image: an
  unrequested flash caused by the failure of the very check that exists to prevent unrequested
  flashes. A board that ANSWERED and answered differently is the only thing auto-update acts on.
- **It does not change the env block's rules.** That write is 4 KB and reversible and is decided by
  `needs_env_write` alone, under either setting.

`no_firmware` remains outside the question entirely. A board in BOOTSEL with nothing running has
nothing to protect and cannot do anything until it is written, so requiring a flag there is pure
friction — the run would stop, print an instruction, and the user would re-run it to reach the only
outcome that was ever available.

Editing a robot config or switching application is therefore a 4 KB write, which is the whole point
of *One Image, Several Applications* — and re-flashing an image that is already on the board is not something a pipeline decides.
The stamp lives in `~/.cache`, **not** under `firmware/`: every box gets a fresh workspace per run, so
a stamp inside the tree would be destroyed by the re-seed that precedes the run that needs to read it.
On RP2 an `--env-only` write still needs BOOTSEL — every write to RP2 flash does — so it is cheap in
time and in risk to the image, but not in the touch; on ESP32 it costs nothing at all.

**The env digest compares blobs, so `app` has to be written even when it is `base`.** The verdict
`env_stale` is `sha256(encoded env)` against the digest in the stamp, and an env built without an
`app` key does not encode to the same 4 KB as one built with `app=base`. `flash_mcu.py` only set the
key when `--app` was passed and `mcu_probe.py --app` defaulted to `None`, so the two agreed only
because the pipeline always passes `--app base`: run either by hand and the board reported
`env_stale` forever, one pointless 4 KB write per run (bench, 2026-09-18, immediately after a green
prebuilt run). Both now default to `base` — the default application, and the one the comment in
`resolve_env_bin()` already claimed was always written. Same family of bug, same fix: the stamp's
fallback digest hashed `config/secrets.yaml` **inside the repo**, a file that does not exist in a
clean tree, instead of `cockpit_paths.secrets_path()`. Every path to the user's config goes through
`cockpit_paths`, with no exceptions for a fallback branch.

### `base` reads the I2C bus before it believes the config
The scan and the WHO_AM_I table live in **`firmware/common/lib/i2c_probe`**, not inside the
`i2c_detect` tool, because both need them. On a real robot (`app=base`, wheels not fake — override
with the `i2c_scan` env key) `setup()` probes the bus, prints every address that answered, and hands
the detected driver names to the sensor factories.

Detection wins over the configured name, and that is deliberate: the YAML is a claim about the
hardware and the bus is the hardware. A wrong claim used to produce the worst failure this firmware
has — `imu->init()` returns false, `setup()` enters the fatal `flashLED(3)` loop, the board never
reaches micro-ROS, and the Cockpit sees a board that will not connect and nothing saying why — while
the `i2c_detect` application, two flash pages away in the same image, could have read the right
answer in 30 ms. Every override is printed and syslogged, so a config that disagrees with the bus is
visible rather than silently routed around. A chip this image has no driver for (a BNO055, say) gets
an empty `driver` field rather than a plausible substitute, and the configured name is kept.

### PlatformIO Inheritance Rules
Every env MUST inherit from `firmware/common/platformio_base.ini` via `extra_configs` + `extends`.
That file is authoritative; the table below records intent, not a copy to keep in sync.

| Base section | platform | board | notes |
|---|---|---|---|
| `base_esp32` | `espressif32` | `esp32doit-devkit-v1` | `upload_speed 921600` |
| `base_esp32s3` | `espressif32` | `esp32-s3-devkitc-1` | `upload_speed 921600` |
| `base_pico` | `maxgerhardt/platform-raspberrypi` | `rpipico` | `upload_protocol picotool` |
| `base_pico2` | same | `rpipico2` | `upload_protocol picotool` |
| `base_picow` | same | `rpipicow` | + `board_build.filesystem_size 1m` |
| `base_pico2w` | same | `rpipico2w` | + `board_build.filesystem_size 1m` |

Shared `[env]`: `framework = arduino`, `monitor_speed`, `lib_extra_dirs = common/lib`.
Each env is one `extends` plus its build flags, e.g.:
```ini
[platformio]
extra_configs = ../common/platformio_base.ini

[env:pico2]
extends = base_pico2
build_flags = -Iinclude -DCONFIG_PATH=\"custom/lino_base_config.h\"
```
(The `upload_*` keys are inherited board metadata; invariant 6 still forbids using the upload target.)

---

### The ROS 2 distro is a BUILD property — the one thing the env cannot carry
`board_microros_distro` selects the precompiled micro_ros library the firmware links against, so a jazzy
image will not talk to a lyrical agent. It cannot move into the `env` partition the way pins, transport and
credentials did: it is fixed at link time. `[env] board_microros_distro = jazzy` in
`firmware/common/platformio_base.ini` is the default, and `firmware/platformio.ini` carries an explicit
`<env>_lyrical` for each released board. **Do not try to drive it from `${sysenv.VAR}`** — PlatformIO 6.2
has no default for an unset variable, it interpolates to the empty string, and the build breaks in a way
that does not name the cause.

Releases therefore ship **4 boards x 2 distros = 8 firmware images** (`scripts/build_prebuilt.py`), every
one of them named `<board>-<distro>`: `pico2-jazzy` and `pico2-lyrical`, not `pico2` and `pico2-lyrical`.
The two are not interchangeable and a board flashed with the wrong one enumerates and then does nothing
useful, so the distro is in the name rather than implied by its absence. The PlatformIO envs stay
asymmetric — `[env:pico2]` is pinned to jazzy — and `fetch_prebuilt.py` maps an env to its profile, so
`fetch_prebuilt.py pico2` still resolves to `pico2-jazzy`.

### The env block is MCU-independent; only where it lives is not
The block itself — CRC32 then NUL-separated `key=value` — is the same on every board.
What differs is the region:

| MCU | region | reached by |
|---|---|---|
| ESP32 / S3 | `env` partition, 0x3ff000, 4 KB | `esp_partition_read` into RAM |
| RP2040 / RP2350 | last 4 KB of flash (`_EEPROM_start`) | XIP-mapped, parsed in place |

On RP2 there is no partition table, so the block goes in the sector arduino-pico already
reserves for EEPROM emulation. Its builder computes `eeprom_start = 0x10000000 + flash_size
- 4096` and `maximum_sketch_size = flash_size - 4096 - filesystem_size`, so that sector sits
above **both** the sketch and the filesystem on every RP2 board, whatever the flash size and
whether or not a filesystem is configured. Addresses: `0x101FF000` (pico/picow, 2 MB),
`0x103FF000` (pico2/pico2w, 4 MB), stated once in `scripts/mcu_env.py:RP2_ENV_OFFSETS`.

**That placement is what preserves the keys through an update.** `picotool load firmware.uf2`
writes only the blocks the UF2 carries, and a sketch UF2 carries none above
`maximum_sketch_size`; arduino-pico's OTA stages the new image in the filesystem area and the
bootloader copies it down over the sketch region. Neither reaches the top sector. Only a
full-chip erase (`picotool erase` with no range, a flash_nuke UF2) clears it.

Write it with `picotool load -t bin -o <offset>` — **both flags are required**: without `-t`
picotool guesses the format from the extension, and without `-o` a raw file lands at the start
of flash, over the application. `flash_mcu.py` writes the env **before** the application,
because loading the application with `-x` runs it and leaves BOOTSEL.

### The ADC LUT is flash data, not source to paste
`firmware/adc_calibrate` used to print `const int16_t ADC_LUT[4096]` over serial for the user
to paste into a config header and rebuild. Under tool mode there is nothing to paste into, so
the table goes to its own `adclut` partition (0x3f0000, 12 KB: header sector + 8 KB of int16),
carved out of the front of the unused `spiffs` area so `env` and both app slots keep their
offsets and no already-flashed board is disturbed. The header is written **last** and acts as
the commit flag, so an interrupted calibration leaves an invalid table rather than a
half-written one that reads as good. The firmware reads it back through `esp_partition_mmap`,
so a lookup costs no RAM at all.

**A tool's working set belongs on the heap.** Under tool mode every tool in the image pays its
static cost in *every* mode, including the robot firmware that will never calibrate anything.
`adc_calibrate` went from 98,308 bytes of file-scope arrays to a 1,028-byte buffer allocated on
entry and freed on exit (measured: 119,944 -> 25,768 bytes of the image's RAM, and the residue
is the Arduino/WiFi baseline, not the tool).

**Do not try to reuse the serial or LiDAR 4 KB buffers as scratch.** They are not arrays:
`setRxBufferSize()` is a size hint that `HardwareSerial::begin()` hands to the IDF UART driver,
which allocates the ring internally and exposes no pointer. The lever that does work is not
allocating them for modes that do not use them — `adc_calibrate` needs neither the micro-ROS
serial rings (4+4 KB) nor the LiDAR UART (4 KB), which is 12 KB not spent rather than 4 KB
recycled.

**Inverting a monotone curve is a walk, not a search.** The calibration measures 257 knots and
the curve between them is piecewise linear, so both the 4096-point curve and its 20480-point
upsampling are pure functions of those knots and neither needs to exist. The original
materialised both (98 KB) and scanned all 20480 entries for each of 4096 targets; the inverse
is one ascending pass over the knots, and exact rather than quantised to a 1/5-step grid. It
also fixes a factor-of-two inconsistency in the original, which interpolated sub-samples at
`j/10` of each interval but reported their positions at `j/5` — **so the emitted table differs
from the legacy one wherever the ADC curve is non-linear.**

### The RP2 family is excluded from the Wi-Fi micro-ROS transport
`transport: udp4` is **ESP32-only**. This is not a capability check and picow/pico2w are not radioless:
their CYW43 sits behind an SPI/PIO link, and that link cannot sustain the **50 Hz control loop** once
micro-ROS traffic rides over it. Wi-Fi on those boards exists for **syslog and OTA** — bursty traffic
that tolerates the bandwidth — and that stays enabled; only the transport is excluded, so
`gen_firmware_header.py` still emits `WIFI_AP_LIST` for them.

Enforced in three places, deliberately: `uros_transport.cpp` gates `UROS_HAVE_UDP` on
`defined(ESP32) || defined(ARDUINO_ARCH_ESP32)` and falls back to serial at boot with a printed reason;
`scripts/gen_firmware_header.py:check_pico_family_transport()` rejects the config outright. The runtime
fallback alone is not enough — it is discovered on hardware, one build and one flash after the mistake.

### Pair the config with its own PlatformIO env
A robot config carries `pio_env:`; build the env it names and no other. This used to be sharp —
`esp32_wifi` and `esp32` were separate envs and crossing them failed on
`micro_ros_agent_locator has incomplete type`. There is now **one ESP32 env**: the transport is installed
at boot by `initUrosTransport()` from the env partition's `transport` key. That is also why there is now
one ESP32 *config* — `config/reference/gendrv_config.yaml`. The serial and udp4 DevKit references that
used to sit beside it described the same silicon and differed only in keys the env decides at boot, so
they were deleted; `-e esp32` builds gendrv, and `transport=` in the env picks the rest.

### A board is a configuration, not a build
Pin matrix, I2C bus and clock, boot-time output pins, which IMU is fitted, transport, credentials and
addresses all reach the firmware through the `env` flash partition (`scripts/mcu_env.py`, read by
`firmware/common/lib/mcu_env`). The generated header supplies the fallback for every one of them, so a
blank env boots as the config describes. Do **not** reintroduce a `BOARD_INIT`-style macro that pastes C
statements into `setup()` from a config header — `initBoard()` (`firmware/common/lib/board_init`) reads the
same facts as data.

---

### `%f` prints nothing on RP2 without `-Wl,-u,_printf_float`
The RP2 toolchain links newlib-nano, whose `printf` has no floating-point conversion unless the
linker is told to pull the real one in. It fails in the quietest possible way: the literal text of
the format string appears and every number is simply absent. `test_sensors` printed
`ACC [m/s^2] X: Y: Z:` on a board whose MPU6050 was answering perfectly. Every `%.2f` in the image
is affected, syslog lines included. The flag belongs in each RP2 `[env:*]` **build_flags** in
`firmware/platformio.ini`, not in the base section — an env's `build_flags` replaces the parent's
rather than extending it, so a value in `[base_pico2]` is silently dropped.

### A tool must take its sensors from the bus, not from the build
`test_sensors` declared `IMU imu; MAG mag;` — the macro types the config header chose — so in a
shared image built for a robot whose config says `imu: FAKE` it reported the fake driver's zeros on
a board with a real MPU6050 answering at 0x68. Any application that reads a sensor goes through
`createIMU()`/`createMAG()` and `i2cProbeSelect()`, the same two calls `base` makes. A diagnostic
that takes its sensor from the build is diagnosing the build.

**`test_acc` was the second offender and the worse one** (fixed 2026-09-17). It declared the same
`IMU imu; MAG mag;` while its entire output is the IMU's measured acceleration set against the wheel
odometry's. One image serves every board of a family and the generated header is whatever robot was
last generated, so the tool read through whichever driver that header happened to name: the motors
spin, the encoders read, the velocity and acceleration columns fill with real numbers, and the run
looks like a completed measurement in which the one column the tool exists to produce came from the
wrong chip. When auditing a tool for this, grep for the macro types **`IMU `/`MAG `** at file scope
in `firmware/src/tools/*.cpp` — the declaration is the whole bug, and it reads as ordinary until you
know the rule.

**Fake mode is a `base` feature and stops there.** `use_fake_*` exists so a bare module with nothing
wired can still bring a ROS 2 stack up; every other application in the image is a bench diagnostic
for a REAL robot, and a fake sensor or a simulated wheel is worth nothing to it. So do not "add fake
support" to a tool, and do not read a tool's output as evidence about fake mode — if a tool reports
zeros, the question is what is on the bus, not which simulation is selected.

### Generate the header for the robot you are about to flash
The generated `firmware/include/custom/lino_base_config.h` is whatever robot was last generated, and
it is a single file for the whole tree. Flashing a Pico 2 with an image built from an ESP32 Wi-Fi
header — real motor pins, `udp4` transport, a board with neither — left the board not enumerating at
all (`device descriptor read/64, error -110`) and needing a physical BOOTSEL replug. The pipeline
generates it per run and now pushes it to **every** box of the pair, the build box included; a
manual `pio run` must do the same by hand.

### The `/cmd_vel` wire contract is decided at BUILD time, so build and launch must be told the same thing
nav2 1.4 (kilted) flipped `nav2_util::TwistPublisher` to `TwistStamped`, so **lyrical drives
`/cmd_vel` stamped and jazzy drives it plain**. The firmware's subscriber type is compiled in behind
`USE_STAMPED_CMD_VEL`, so this cannot be decided at run time the way pins and transport are (see *The ROS 2 distro is a BUILD property* below).

`stamped_cmd_vel: auto` in a robot config resolves from the distro, and
`scripts/gen_firmware_header.py --distro <d>` is what resolves it — before this, `auto` fell through
every truth test and silently meant *off*, so a lyrical image always listened for plain `Twist`.
`one_click_pipeline.py` passes the same `--distro` to the header generator, the launcher and the goal
test. Getting it wrong is invisible: the type hashes differ, nav2 publishes into a topic nothing is
subscribed to with that type, the robot never moves, and nav2 reports "Failed to make progress" while
nothing anywhere says `cmd_vel`. linorobot2's console branch hit this from the other side (`6e27659`)
and forced nav2 back to unstamped, because its firmware has no `TwistStamped` subscriber; this
firmware has one, so **the cockpit keeps nav2's own default per distro** rather than copying that
workaround.

### Everything the board dials is the ROBOT computer
Over Wi-Fi (`transport: udp4`) the board stops being reachable only through a wire and starts dialling
addresses, so "which machine?" becomes a configuration value that can be wrong in silence. It is the
**robot computer**, for all three:

| what the board dials | key (`config/secrets.yaml`) | default port | why |
|---|---|---|---|
| micro-ROS agent | `micro_ros.agent_ip` | 8888 | `micro_ros_agent` is a ROS 2 node, and docs/flashing.md puts the ROS 2 runtime on the robot |
| fake/real LiDAR | inherits `agent_ip` | 8889 | the LiDAR driver is a ROS 2 node too |
| syslog receiver | `telemetry.syslog_server` | 5140 / 514 | could be either side — but put it with the others |

The LiDAR needs no key of its own: `mcu_env.py:254` is
`lidar_ip = lidar.get("server_ip") or agent_ip`, so it follows the agent unless a config sets
`server_ip` explicitly. Syslog is the only genuinely free choice, and it goes on the robot for
consistency: then **every address the board dials is one machine**, and a wrong one is a single
mistake rather than three independent ones.

Two ways to get this wrong, both of which produce a board that boots perfectly and is never heard
from: pointing the keys at the laptop you happen to be browsing from, or at any machine other than
the one running `micro_ros_agent` — the agent the board is dialling is not running there.

**The syslog port is `5140` by default, and it is a default rather than a fixed value.** It used to
be 514-with-a-5140-fallback, decided by whether the process could bind a privileged port — which made
it a RUNTIME fact that two machines running the same config could disagree about, and a board pointed
at 514 with a sink on 5140 fails in total silence: the board keeps sending, the syslog tab stays
empty, and there is no RSSI to read. A private sink for one robot gains nothing from the well-known
port, so 5140 is now the unconditional default on both ends. `telemetry.syslog_port` still wins when
an operator sets it, and the board reads the same key through `mcu_env.resolve_syslog_port()`, so the
two cannot drift. `/api/syslog/status` reports the port actually bound.

The receiver itself needs no starting: `web/backend/main.py:lifespan()` launches it when the Cockpit
backend starts, and `SyslogManager._host_ip()` already advertises the machine's own primary LAN IP,
so the backend reports the robot computer's own address without being told.

### Everything that is skipped at init must be skipped in `loop()` too
`initWifis()` returns at once when the env says `wifi=0` on a serial robot. `runWifis()` did not:
it called `wifiMulti.run()` from every `loop()` iteration, and the framework's `WiFiMulti::run()`
on a radio that is not connected does not look at its (empty) AP list first — it calls the
**blocking** `WiFi.scanNetworks()`. Measured on the GenDrv, that was **one `loop()` per ~7 s**, one
control-timer callback per loop, `/imu/data_raw` at 0.14 Hz; with `wifi=1` the same binary did
41 loops/s. From the agent's side the board looked healthy: one session, six datawriters, no
cycling — because the keepalive ping is also once per loop, and once per 7 s is enough to hold a
session. `ros2 topic hz` said `NO DATA` and nothing said why.

The rule: a service that `loop()` calls every iteration is guarded by the same question its init
was (`wifiWanted()`, cached — it is a boot-time fact and the loop runs at 50 Hz), and the answer is
measured on the board, not inferred from the agent log. Which is what the next section is for.

### Dual core is for a serial robot, and the radio overrides it
`use_dual_core` pins `moveBase()` to core 0 so the micro-ROS core only reads sensors and
publishes. On the GenDrv with QMI8658 + AK09918 + INA219 + BMP280 over 1.5 Mbaud serial that is
**43 -> 49 Hz** (`/imu/data_raw` 48.8, `/odom/unfiltered` 48.9). It is not a Wi-Fi feature and
must not run with the radio on: `controlTask` holds `portENTER_CRITICAL` around `moveBase()`,
which disables interrupts on core 0 — the core the Wi-Fi driver and lwIP run on. Measured with
`wifi=1`, the same binary froze inside a publish for 6 s (`pubs=6015466us fail=6` on the UART1
line) and `/imu/data_raw` fell to 4 Hz. The firmware therefore ignores `dual_core=1` whenever
`wifiWanted()` — `wifi=1` or `transport=udp4` — and prints `[core] dual_core=1 ignored` at boot.

What the remaining ~24 ms of a 20 ms cycle is: `moveBase` ≈ 2 ms, the four sensor reads ≈ 1.8 ms,
and the six `rcl_publish` calls ≈ 24 ms. Every publisher is `rclc_publisher_init_default`, i.e.
reliable, and a reliable `rmw_publish` blocks in `uxr_run_session_until_confirm_delivery` — one
agent round trip over the USB bridge per message. Best-effort streams for the 50 Hz topics would
remove that wait, but a best-effort writer does not match a reliable subscriber, so the EKF's
`/odom/unfiltered` subscription has to be checked before that topic changes. Not done.

**Best effort is the default, and the MTU it needs.** The three 50 Hz publishers are
best-effort (`qos: reliable` in the config → env `best_effort=0` turns it back), which removes
the per-message ACK wait. Measured on one core, radio off: 1.5 Mbaud 42.3 → **50.0 Hz**;
**921 600 baud 25.1 / 33.0 → 50.1 / 50.0 Hz** — the rate most ESP32 modules can run at. The
launch tree's `imu_filter_madgwick` and `robot_localization` subscribe best-effort already:
with the board best-effort, `/imu/data` 49.97 Hz and `/odom` 49.0 Hz. The first attempt lost `/odom/unfiltered` entirely (`fail=50` per second on the
UART1 line): a best-effort XRCE stream cannot fragment, so a message must fit one transport MTU,
and the client's default for a custom transport is **512 bytes** — `nav_msgs/Odometry` with its
two 6×6 covariances is ~720. `UCLIENT_CUSTOM_TRANSPORT_MTU=1024` in the user metas
(`firmware/atomic.meta` for the pico family, `firmware/esp32.meta` for the ESP32s) is what makes
a best-effort odometry possible; it costs a few KB of stream buffers. A best-effort writer never
matches a reliable subscriber, so the launch tree has to agree before this becomes a default.

**A meta change needs the micro-ROS library cleaned.** `micro_ros_platformio` builds
`libmicroros` once per env and then links the cached copy; editing a `.meta` does not rebuild it.
Before `pio run` after a meta change: `rm -rf firmware/.pio/libdeps/<env>/micro_ros_platformio/{libmicroros,build}`,
then check `grep CUSTOM_TRANSPORT_MTU firmware/.pio/libdeps/<env>/micro_ros_platformio/build/mcu/install/include/uxr/client/config.h`.
CI and the release build start from a clean tree, so they pick the metas up on their own.

**Default: on for ESP32 / ESP32-S3** (`gen_firmware_header.py` defines `USE_DUAL_CORE` unless the
config says `use_dual_core: false`; `mcu_env.py` writes `dual_core` only when the config sets it).
It had been switched off by default because of the Wi-Fi hazard above, "for a benefit nobody
measured"; the benefit is measured now and the hazard is handled at boot, so the serial robot
gets its best setting without asking for it. Open: in ~2.5 min of dual-core, radio-off
observation there was one 4 s freeze inside the publish path (`fail=4`, four 1 s reliable-publish
timeouts) that did not recur in a further 90 s (89 ticks, `loop=50`, no failed publish). Not
understood; the UART1 line is the instrument to catch it with.

### The pin catalogue: a config is checked against the silicon before anything is written
`scripts/pin_catalog.py` knows, per MCU, which GPIOs exist, which must never be driven (the
ESP32's flash bus 6–11, the S3's 26–32), which are input-only or strapping, which are ADC (and
which ADC the Wi-Fi driver owns), and which pin pairs an RP2 I2C block accepts.
`gen_firmware_header.py` runs it on every config: an **error** stops the generation (fix the
config or pass `--no-pin-check`), a **warn** is printed for a human. The first run found
`gendrv_config.yaml` still carrying the ESP32-S3 matrix — GPIO 20, 6, 8, 11 and 1 on a WROOM —
the same set that had boot-looped the GenDrv on the bench; only `gendrv_real` had been fixed.
That is the point of the catalogue: the mistake is named at header time, with the pin and the
reason, instead of arriving as an interrupt-watchdog reset.

Alongside it, `kinematics` may give the encoder as parts — `encoder_ppr`, `gear_ratio`,
`quadrature` (4 unless stated) — and `counts_per_rev` is their product, computed in one place
for both the header and the env. A mecanum config is also checked for the ROS-side values a
differential template leaves behind (EKF `vy` fusion, `min_y_velocity_threshold`, the velocity
smoother's `y` limits).

### UART1 is the board's own diagnostic channel (`diag_tx`)
micro-ROS owns UART0, so once the session is up the board has no console. The ESP32 has a second
UART whose TX can go out any pin; on the GenDrv `LIDAR_SERIAL 1` on GPIO 4 is already wired to a
USB bridge on the bench, and when no LiDAR is using that UART it is free. `--set diag_tx=4`
(`diag_baud`, default 230400 — any rate works, this is not the LiDAR's wire) makes the firmware
print one line a second on it:

```
D t=11598 dt=1006 st=2 loop=41 tmr=41 pub=41 fail=0 tx=51012 rx=3571/401(41 empty) spin=41 rc=0 max=28963us ping=5/0 heap=178760/176280
```

`loop()` iterations, control-timer callbacks, `publishData()` completions, failed `rcl_*` calls,
transport bytes written and read (and how many read calls returned nothing — those are the idle
waits), `rclc_executor_spin_some()` calls with its last error and longest duration, pings, heap.
Every counter is a delta over `dt`. The three numbers to read first are `loop`, `tmr` and `pub`: a
healthy serial board shows all three at the control rate; the stall above showed `loop=1
dt=7224`. The library is `firmware/common/lib/diag/`, costs nothing when `diag_tx` is unset, and
is a bench tool — it is set on the command line with `--set`, never from a robot config, and it
must not be enabled on a board whose LiDAR really is on that UART.

### Wi-Fi rides along on a serial-transport board, and OTA is flashing
`transport: serial` does not mean the radio is off. Wi-Fi stays up for **syslog and OTA** on every
board that has it — that is exactly why the RP2 family keeps Wi-Fi while being excluded from the
`udp4` transport (see *The ROS 2 distro is a BUILD property* below): bursty telemetry tolerates the CYW43's SPI link, a 50 Hz control loop does
not. So a serial board still dials the two endpoints above, and they are still the robot computer.

**ArduinoOTA is a flashing path, so it belongs to the robot computer, exactly like USB flashing.**
The medium changes, the role does not: docs/flashing.md gives the robot the bus and the flasher and says the host
workstation never opens a serial port — and it must not open an OTA session either. A host that can
flash over the air has quietly reacquired the job the split exists to take away from it, and it does
so on the one operation that can brick an assembled robot. The host builds; the robot writes,
whether the bytes arrive over USB or over the air.

This is **not implemented yet** — nothing in `scripts/` or `web/backend/` invokes `espota`, and the
firmware side is only `USE_ARDUINO_OTA` plus `telemetry.ota_port` (3232). When it is built, it takes
the same shape as the USB path: the robot computer fetches or builds the image, verifies its
SHA-256, and runs the uploader against the board itself. Do not add an OTA target to
the host role, and do not drive it from PlatformIO's `upload` (invariant 6 forbids that for the same
reason it forbids `pio run -t upload`: it welds build and flash together and hides which binary
reached the board).

### Every env key has a writer and a reader, and a test says so
A key the config side writes under one name and the firmware reads under another fails
silently: the value never arrives and the compiled-in fallback answers instead, which is
usually the right number on the bench and the wrong one on somebody else's robot. One
afternoon's audit (2026-09-19) found, in this repo:

- `pwm_min` / `pwm_max` read by `createPID()` and written by nothing. An image built for
  10-bit PWM clamped the PID at ±1023 on a robot whose env said `pwm_bits=8`; the motor was
  told 1023 on a 255-wide channel. The limits are now derived from `pwm_bits` on the board.
- `pins.battery.r1` / `r2` saved by Config Studio for years and read by nothing; the firmware
  used a `BATTERY_ADJUST` macro no file defined, so a config with a battery pin did not build.
  `battery_pin bat_r1 bat_r2 bat_min bat_max bat_cap` are env keys now, read by `battery.cpp`.
- The LiDAR emulator was a build macro only (`USE_FAKE_LD19`). A prebuilt image is built from
  a fake-mode reference, so every real robot that flashed it raycast a room and streamed it.
  `fake_ld19=0` in the env (from `sensors.use_fake_ld19`) switches it off; `lidar_x` tells it
  where on the robot to raycast from (`geometry.laser.x`, the same number the URDF uses).
- `telemetry.ota_port` had a field in the UI and no reader; `ota_port` is read now.
- `/api/ai/deploy_robot` wrote `kinematics.track_width` and `wheelbase`; the firmware reads
  `lr_wheels_distance` and `fr_wheels_distance`. The browser read `wheel_diameter_m` from a
  reply that carried `wheel_diameter`. Both sides speak the config's vocabulary now.
- `motor_power_measured_voltage` was documented upstream and read by nothing; removed. So
  were `supervisor.backend_host/backend_port/frontend_port`.

`tests/test_env_contract.py` closes the loop: every key `mcu_env.py` can write must appear as
an `env*("key")` read in the firmware sources (the generated header's macro reads count), and
every key the firmware reads must have a writer or be on the short bench-only list
(`diag_tx`, `diag_baud`, `app`, `dac_pin`, `boot_serial_wait`). Add a key on one side only
and CI fails with its name.


### The diagnostic UART and the LiDAR emulator cannot both have UART1
`diagBegin()` and `FakeLD19::begin()` both do `new HardwareSerial(LIDAR_SERIAL)`. On an ESP32
that is one peripheral behind two objects, and the second `begin()` simply re-points it: diag
starts first (`setup()`, before the env's LiDAR pin is read), the emulator starts second, so the
emulator wins the pin and the diagnostic stream goes nowhere — with nothing said either way.
Every diagnostic run so far happened to have `fake_ld19=0`, which is why it never showed.

`diagBegin()` now refuses instead: with the emulator compiled in, enabled, and holding a UART
sink (`lidar_rx >= 0`), it prints `diag_tx=<n> ignored: the LiDAR emulator owns UART<n>` and
leaves the emulator alone. Turn one off to use the other. If both are ever needed at once, the
fix is a distinct UART for diag (the classic ESP32 and the S3 both have a third), not an
ordering tweak — two owners of one peripheral is the bug.

### Anything that sends UDP must re-check the radio, every time
`syslog()` has always ended with "no radio, no log" because `WiFiUDP::beginPacket()` calls into
lwIP and lwIP has no tcpip thread until the Wi-Fi stack starts one: without the guard it is
`assert failed: tcpip_send_msg_wait_sem ... (Invalid mbox)` and an abort, not a dropped line.
Two other senders had no such guard and were only safe by accident, because `initWifis()` used
to block until the radio was up, so nothing after it ever ran without one.

Both are reachable now. `FakeLD19::flushUdp()` and `lidar.cpp`'s UART receive callback check
`WiFi.status()` on every call and drop the buffer when it is down — checked at send time, not at
`begin()`, because the radio can arrive long after `setup()`. This is not hypothetical on a
released image: the published `esp32` firmware is built from the Wi-Fi profile
(`build_prebuilt.py`), so `USE_LIDAR_UDP` is compiled into the image that a serial ESP32 robot
runs with `wifi=0`. The emulator's counters carry `udp_noradio` so a bench can see it happening.

### An AP that does not answer must not stop the boot
`initWifis()` ended in `while (wifiMulti.run() != WL_CONNECTED) delay(500);` — no timeout, no
message. A board whose AP refused it stopped inside `setup()`: the console ended at
`[wifi] using SSID '<x>' from the env partition` and stayed there, no I2C scan, no banner, no
micro-ROS, no syslog (which needs that radio), and no reset either — so it looked bricked while
it was in fact waiting. Measured on the GenDrv on 2026-09-19: 0 resets in 25 s, 0 datagrams, 0
datawriters, while the same board had been on that AP half an hour earlier.

The wait is bounded now (`WIFI_CONNECT_TIMEOUT_MS`, 20 s) and says which way it went; the robot
boots either way and `runWifis()` keeps trying. That retry had to be rate-limited in the same
change: `WiFiMulti::run()` on a *disconnected* radio calls the blocking `WiFi.scanNetworks()`
first, and until now a board only reached that state briefly between dropouts. It can now be in
it for its whole life, so the disconnected branch runs once every `WIFI_RETRY_INTERVAL_MS` (5 s)
instead of once per `loop()` — which is the same stall that once gave one `loop()` per 7 s and
`/imu/data_raw` at 0.14 Hz.

### Covariance and the simulated world are env keys, not build constants

Ported from linorobot2_hardware's `robot_config_engine`, which carries these in
its spec and emits compile-time macros. Here they go in the env block, for the
reason everything else does: one released image has to serve a board with an
MPU6050 and a board with a BNO085, and their accelerometer variances differ by
a factor of seven. The firmware keeps its `#ifndef` defaults as the fallback —
`envFloatVec()` leaves them alone when a key is absent — so a blank env still
boots with sane values.

```yaml
base_controller:
  bmp280_addr: "0x76"        # 0x77 on the Waveshare board, 0x76 on breakouts
  imu_tuning:
    accel_cov: 0.0015        # a scalar expands to all three axes
    gyro_cov:  3e-06
    pose_cov:  [1, 2, 3, 4, 5, 6]   # ...or give the whole diagonal
    twist_cov: 0.001
    mag_bias:  [1.5, -2.25, 0.75]   # hard-iron offsets, three axes or nothing
  simulation:                # key names are the config engine's schema.json
    map_width: 10.0
    map_height: 6.0
    wall_obstacle: true
    wall_x1: 2.0
    wall_y1: -1.5
    robot_mass: 3.5
    wheel_noise_rpm: 1.0
```

Two rules are worth stating because both protect the EKF from a config mistake:

- **A scalar expands, a full list is used as-is, and anything in between is
  refused.** Silently zero-filling the axes a user forgot would tell the EKF
  the robot is perfectly certain about them.
- **An all-zero `mag_bias` is not a calibration**, so it is not written and the
  firmware subtracts nothing. Subtracting a bias nobody measured is worse than
  subtracting none.

When the config names a sensor but gives no covariance, mcu_env fills in a
datasheet-derived variance (roughly `(noise_density·√100 Hz)²`) from the same
table the config engine uses. Without it a config that names its IMU still
ships the firmware's `1e-5` placeholder, which reads as "this sensor is
nearly perfect".

The simulated room is configuration because **fake mode is the default here**:
a Nav2 test wants the obstacle wall somewhere else without rebuilding, and a
12 kg robot does not accelerate like a 3.5 kg one.

### `topic_prefix` is an env key too, so the names are built at run time

It used to be the one remaining compile-time fact about a robot: `TOPIC_PREFIX`
was pasted onto each topic literal by the preprocessor, so putting two robots on
one DDS domain meant a build per robot -- and the published images, built from
the generated bare config, could not do it at all. It is `topic_prefix` in the
env now, which means the names themselves have to be assembled at run time.

`topicName()` in `main.cpp` does that, and the two rules it follows are the
interesting part:

- **Cache by the suffix's address, not by its text.** Every caller passes a
  string literal, so the pointer is stable and a reconnect
  (`destroyEntities()` -> `createEntities()`) hands back the same buffer instead
  of building a second copy. Twenty slots and a 640-byte arena cover every topic
  the firmware publishes with room to spare.
- **A prefix ROS 2 would reject is worse than no prefix.** rclc just fails the
  entity, so an illegal character would leave the board with no topics at all
  and nothing in the log to say why. Only `[A-Za-z0-9_/]` gets through; anything
  else means no prefix and a line on the console. A missing trailing slash is
  added, so `robot1` and `robot1/` mean the same thing -- the same rule
  `scripts/mcu_env.py` and `cockpit_paths.robot_namespace()` apply on the host
  side, which is what makes the board and the stack agree without either being
  told about the other.

The compiled-in macro is still the fallback, so a board with a blank env behaves
exactly as it always did, and with no prefix set the prefixer returns the
literal and allocates nothing at all.

**The prefix covers the frame_ids too, and it has to.** Prefixing only the topic
names gets you a robot that publishes `/lino1/odom/unfiltered` with
`frame_id: odom` inside it -- naming a frame that does not exist in its own TF
tree, where `robot_state_publisher` has published `lino1/odom`. The EKF finds
nothing relating the two, ignores every message, and publishes nothing; the
board looks perfect and the robot has no odometry. That is what the two-robot
bench found on 2026-09-21, and it is invisible with one robot because a
single-robot stack is usually run without a prefix at all.

So `envPrefixed()` lives in `mcu_env`, beside the env reader it depends on, and
every frame the firmware stamps goes through it: `odom` and `base_footprint` in
the odometry, `imu_link` in the IMU, the magnetometer and the fake wheels,
`sonar_link` in the range driver, `base_link` on the environmental sensors.
`topicName()` is now just its old name.

**When it is applied matters as much as that it is applied.** Several of those
frames are stamped in constructors, and a constructor runs during static
initialisation -- before the flash partition API is usable, where the env reads
back empty. This is the same trap `initSyslog()` and `applyEnvCovariance()`
document. The constructors keep stamping the plain name, which is a valid frame
for an unprefixed robot, and `applyEnvFrames()` re-stamps it from `setup()` once
the env is readable, next to where the covariances are read. A test
(`tests/test_frame_prefix.py`) reads the firmware source and fails if a frame is
ever stamped with a literal that nothing re-stamps.

Auditing those sites turned up an older bug with nothing to do with namespaces.
The **simulated** sonar fills `range_msg` field by field in `main.cpp` and never
touched the header, so every fake-mode board -- which is the default -- published
`/sonar` with an **empty** `frame_id`, a Range message no consumer can place
anywhere. The real path assigns the whole message from `getRange()`, which
carries the frame the range driver set, and was never affected. It went unseen
because nothing in the pipeline subscribes to `/sonar` yet: a topic with no
consumer is a topic with no one to notice it is malformed.

### DRAM is the ESP32's scarce budget, and a static is paid by every board

The `esp32_lyrical` release build failed to link with `dram0_0_seg overflowed by
96 bytes`. Nothing in this repo had grown: the platform, framework and library
SHAs were identical to the run before it. What changed was upstream -- the
lyrical micro-ROS branches drifted and the precompiled library's own `.bss` grew
into the space the firmware had been living in.

Two lessons, and both were worth having:

**The margin was never real.** `dram0_0_seg` is 124,580 bytes on the ESP32 and
holds `.dram0.data` + `.dram0.bss`; PlatformIO's headline "RAM %" is a different
number and will happily read 60% while the link is about to fail. The budget to
watch is the segment, and the way to read it is
`xtensa-esp32-elf-size -A .pio/build/esp32_lyrical/firmware.elf` after a build,
not the percentage in the build summary.

**Allocate when it is needed, not at file scope.** A static buffer is paid by
every board in every application: an `esp32` robot that will never calibrate an
ADC, never prefix a topic and never run `test_sensors` was still carrying all
three working sets in `.bss`. Moving them to the heap is nearly free -- the
allocation happens once, in `setup()` or on first use, out of the ~300 KB heap
that the robot firmware barely touches, and a never-freed block taken at startup
cannot fragment anything. What moved:

| what | was | now |
|---|---|---|
| `topicName()` arena | 640 B of `.bss` | `malloc` on the first *prefixed* name; nothing when `topic_prefix` is unset |
| `FakeLD19 fake_ld19` | a static instance | `new` only when `fake_lidar_on` |
| the micro-ROS messages, executor and `Odometry` | static objects | allocated in `setup()` under `if (micro_ros)`, `rclErrorLoop()` if the allocation fails |
| `test_sensors`, `test_acc`, `bno085_cal` working sets | file-scope buffers and driver objects | `calloc` / `new` in each tool's `setup_()`, with a null guard in `loop_()` |

That took `esp32_lyrical` from **96 bytes over** to **4,148 bytes of margin**
(120,432 of 124,580 used, as the `rc-20260922` release build reports it), and
left `esp32-jazzy` at 19,692 bytes of margin (104,888 used) -- about 4.2 KB
freed on every ESP32 profile. Every path was then re-run on the bench: the prefixed board published
`/lino1/odom/unfiltered`, `/lino1/imu/data_raw`, `/lino1/raw_scan` and
`/lino1/sonar` with no `[topic] no room` warning, and the tools were switched by
env write and read back over serial.

An allocation that can fail has to say so. `topicName()` prints `[topic] no room
to prefix "<suffix>" -- publishing it unprefixed` rather than dropping the
prefix silently, because a silent drop splits one robot's graph across two
namespaces; the message allocations in `setup()` go to `rclErrorLoop()`, because
a board that cannot build its publishers is not a robot.

### Pin the fork's micro-ROS repos to the distro's own branch

The build inputs came from `hippo5329/micro_ros_platformio`, whose
`microros_utils/repositories.py` lists, per distro, the repositories the
micro-ROS library is compiled from. The `lyrical` entry had four of them
overridden to `rolling` -- `micro_ros_utilities`, `micro_ros_msgs`,
`rmw-microxrcedds`, `rosidl_typesupport_microxrcedds` -- because those repos had
no lyrical branch when the entry was written. They have one now, and `rolling`
kept moving, so every build pulled whatever `rolling` happened to be that day.
That is the drift that ate the 96 bytes.

The rule: **a repository that has a branch for our distro is pinned to it; only
one that genuinely has no such branch stays on `rolling`.** After the fix every
micro-ROS repo in the lyrical set builds from `lyrical`; `ros2/rclc` stays on
`rolling` because it has no lyrical branch, and the eProsima repos stay on their
`ros2` branch, which is where they release from. Upstream is not developing this
package any more, so the fork is ours to keep correct -- and unpinned branches
in a build recipe are a failure waiting for a date, not a version.
