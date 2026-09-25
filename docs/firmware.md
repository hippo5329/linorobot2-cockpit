# Firmware: one image, several applications

Design notes and hard-won rules for `firmware/`.

**There is exactly one firmware project: `firmware/`.** The diagnostics are not separate builds any
more — they are applications inside that image, selected at boot by the `app` key of the env
partition (`base`, `test_sensors`, `test_motors`, `test_acc`, `i2c_detect`,
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
[fw] linorobot2_hardware app=base distro=lyrical built=2026-09-16 git=6aa607f uid=C3AF89DC55525350
```

`app` is the sub-firmware this boot selected, `distro` the ROS 2 distro it links against, `built` the
compile date, `git` the 7-character revision of the tree it came from — with a trailing `+` when that
tree had uncommitted edits, so a dirty build never reads as a clean revision. Those four arrive as
`-DFW_ROS_DISTRO` / `-DFW_GIT_REV` / `-DFW_BUILD_DATE` from **`firmware/common/build_stamp.py`**, a
`pre:` extra_script in `common/platformio_base.ini`.

**The base application repeats the banner before it starts micro-ROS.** Printed once, in the first
milliseconds, it was heard by nobody: an RP2's port re-enumerates on every boot and a bench cell only
gets the node back 2–3 s later, past `boot_serial_wait`, so every flash logged
`banner_confirmed: false` — the flasher had verified the bytes it wrote, not what the board runs. So
`setup()` holds the transport for `banner_hold` ms (env, default 4000, 0 disables), printing the banner
every 500 ms, and only then calls `initUrosTransport()`: once XRCE frames own the port, text would be
noise in the agent's stream. It runs before the watchdog is armed and only for `app=base`.

### The last field names the BOARD, and the key says what it identified

`git=` answers "which build is this?". It cannot answer "which board is this?" — and two identical
boards on one bench are otherwise indistinguishable from their own output. So the banner ends with an
identity field. Only two of the three families can name their own silicon, so **the key is the
claim**, and the two are never merged:

| key | identifies | where it comes from |
|---|---|---|
| `uid=` | the **silicon** | RP2350: chip info from ROM (`SYS_INFO_CHIP_INFO`). ESP32: the 48-bit eFuse MAC, burned per chip. |
| `flashid=` | the external **flash chip** | RP2040 only, and it is all an RP2040 has: `pico_get_unique_board_id()` there returns `flash_get_unique_id()`. |

An RP2040 has no chip id of its own. Its value still names that particular board, which is what a
bench needs — but it moves if the flash is replaced, and two RP2040s cannot be told apart by silicon
at all. Reporting it under `uid` would be a lie that looks exactly like the truth, and the trap is
sharper than it sounds: **the Pico's USB serial number *is* that same flash id**
(`usb-Raspberry_Pi_Pico_W_D665C007DA2A1336`), so the mistake would appear confirmed from two
independent directions. Measured on all three families, each matching its USB descriptor:

```text
RP2350  Pico 2 W      uid=C3AF89DC55525350
RP2040  Pico W        flashid=D665C007DA2A1336
ESP32   bare WROOM    uid=E435A8286F24
```

The field is **appended after `git=`**, so a host parsing an older board still matches and a board
running an image from before the field simply has neither key — absence means "older image", never
"the read failed". `firmware/src/main.cpp:identityField()` is the one place that decides which key a
family gets; `mcu_probe.py` parses both, `flash_mcu.py` reports and records them in the flash stamp,
and the cockpit header shows `chip <id>` or `flash <id>` beside the detected MCU — different words,
on purpose.

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
as "this host has no record of flashing it". `LINO_STAMP_DIR` overrides. The `<env>` is whatever
the flasher wrote, which for a prebuilt image is the profile's own `pio_env`: `pico2-jazzy` is the
`pico2w` image (it runs on both boards), so the record is `pico2w_<port>` while the pipeline probes
as `pico2`. `stamp_for()` falls back to the manifest's `pio_env`. Without that fallback, every
Start 1-Click read "no record" a minute after writing one, and reflashed a board already running
the build (2026-09-25). The verdict decides the cost:

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

### The simulated robot stops OUTSIDE Nav2's footprint, and its magnetometer is fused
A soak found two defects that a single-goal gate structurally cannot see, both in the bare-module
simulation, both with the same shape: two numbers describing one robot were set independently and
disagreed.

**The wall clamp.** `sim_ld19.h:clampToRoom()` holds the simulated robot's centre
`SIM_ROBOT_RADIUS` from any wall — it has always collided, and corrects odometry to match. But that
radius was a hardcoded `0.20f` while the shipped configs plan with `robot_radius` 0.22–0.26 m, so
the clamp parked the robot **inside Nav2's own footprint**: a lethal costmap cell the planner will
not plan out of. Measured: the planner replans 48 times in 80 s, every plan a clean route away from
the wall, and the controller executes none — 1.2 mm of travel; one board sat there for 154
consecutive goals. `Spin` cannot help because rotating does not change which cell the robot
occupies, and the base itself was fine — commanded directly it rotated and reversed out.
`scripts/gen_firmware_header.py` now derives `SIM_ROBOT_RADIUS` from the largest `robot_radius`
either costmap plans with, plus 5 cm so the robot never stops exactly on the boundary; the firmware
fallback is 0.30f, above every shipped radius. Verified by driving into the room boundary: it stops
at exactly `5.0 − 0.31` and `3.0 − 0.31`.

**The heading.** `SimIMUFromWheels::applyMag` rotates a world field into the body frame by the
wheel heading for one purpose — to give Madgwick an absolute heading that agrees with the room.
Two separate decisions then silenced it: `bringup.launch.py` excluded the simulated mag from fusion
(`... and not use_sim_mag`, a leftover from when the field pointed +X), and `mcu_env.py` derived
`pub_mag` from `mag: NONE` so the firmware never published it. Madgwick therefore integrated the
gyro alone, the simulated gyro's bias walked onto its ±0.004 rad/s clamp and stayed there — 13.7°/min —
and the EKF, which takes Madgwick's yaw as *absolute* and only the wheels' yaw *rate*, inherited it.
(That last clause described the intent, not the code: `imu0_config` did not fuse absolute yaw in any
shipped config until 2026-09-23. It does now — `docs/ros2-stack.md`, "The heading needs an anchor".)
Measured at rest, read from inside the stack's own DDS environment: wheel yaw 59.4°, EKF 7.2° after
an hour. The body follows the wheels and Nav2 steers by the EKF, so goals veer (a held heading of
0.2° moved at 67°) and the robot eventually finds the wall. Both halves now agree: `use_mag` is true
for a real mag *or* the sim one, and `use_sim_mag` sets `pub_mag=1`. `NONE` means "no chip"; it
must not also mean "silence the simulation of one". Fixing only the launcher half made it *worse* —
Madgwick with a magnetometer waits for `imu/data_raw` and `imu/mag` as a synchronised pair, so with
nothing on `/imu/mag` it published nothing and `/imu/data` went to 0 Hz — which is why the two ship
together and why the fix was measured before it was cut.

**That pairing is gone as of 2026-09-25: the board fuses its own orientation.** Everything above
describes a real fault and a real fix, and the fix was right — but it left `/imu/data` depending on
two best-effort messages both surviving the link. `imu_filter_madgwick` matches `imu/data_raw` with
`imu/mag` through a `message_filters` `ApproximateTime` synchroniser five deep, so `/imu/data` was
the rate of *matched pairs*, and a dropped magnetometer message cost an IMU sample. Measured on two
bench legs where the board slowed down: `/odom`, which needs no partner, held 33 Hz while
`/imu/data` fell to 10 — a 1.5x loss in what was sent becoming a 5x loss in what the EKF received.

The board holds gyro, accel and field in the same 50 Hz cycle, from the same trigger, with nothing
to synchronise. So `firmware/common/lib/imu/ahrs.h` — a port of that node's own `ImuFilter`, held to
it numerically by `tests/test_ahrs_is_the_filter_it_replaces.py` — does the fusion on the board,
publishes `imu/data` directly with an orientation and an honest covariance, and removes gravity at
the source as the node's `remove_gravity_vector` did. `imu/mag` is still published, because
`magnetometer_calibration` needs it. A part with on-chip fusion (BNO085) does its own AHRS and the
filter stands aside: `hasFusedOrientation()` is the seam.

Cost, measured by compiling one update for each target: **988 instructions and 422 hardware FPU ops
on an ESP32** (~5 us, 0.025% of a 20 ms cycle), and on the RP2040 — the only target with no FPU —
922 instructions and 188 soft-float helper calls, about 0.7% of the cycle. There is no
`imu/data_raw` and no filter node any more: the launch has nothing to start for the IMU.

### `base` reads the I2C bus before it believes the config
The scan and the WHO_AM_I table live in **`firmware/common/lib/i2c_probe`**, not inside the
`i2c_detect` tool, because both need them. On a real robot (`app=base`, wheels not simulated — override
with the `i2c_scan` env key) `setup()` probes the bus, prints every address that answered, and hands
the detected driver names to the sensor factories.

Detection wins over the configured name, and that is deliberate: the YAML is a claim about the
hardware and the bus is the hardware. Trusting a wrong claim produces the worst failure this firmware
has — `imu->init()` returns false, `setup()` enters the fatal `flashLED(3)` loop, the board never
reaches micro-ROS, and the Cockpit sees a board that will not connect and nothing saying why — while
the `i2c_detect` application, two flash pages away in the same image, could have read the right
answer in 30 ms. Every override is printed and syslogged, so a config that disagrees with the bus is
visible rather than silently routed around. A chip this image has no driver for (a BNO055, say) gets
an empty `driver` field rather than a plausible substitute, and the configured name is kept.

### Which sensors this image carries, and what has been proven on silicon

Every driver below is compiled into every release image -- the table is the *sensor
factory* (`sensor_factory.cpp`), and which one runs is decided at boot by the I2C probe,
not at build time. So "supported" means the image can drive it; the other two columns are
narrower claims and are kept separate on purpose.

**Read the columns exactly.** *Bench* means a real chip of that part answered on a bare
module and its numbers were read -- not that a robot drove with it. Nothing in this
project has run on a real robot yet (first is October 2026), and a bare-module reading is
the strongest evidence any row here has.

Two separate claims, two columns. *Identified by* is the register read that
tells this part from the others sharing its address. *Read on* is where a real
chip of that part answered and its numbers were taken -- a much narrower claim
than "the driver exists", and the only one backed by a measurement.

| IMU | I2C addr | identified by | read on |
|---|---|---|---|
| MPU6050 / MPU6500 / MPU9150 | 0x68, 0x69 | `WHO_AM_I` 0x75 = 0x68 / 0x70 | bench Pico 2, GP0/GP1 |
| MPU9250 | 0x68, 0x69 | `WHO_AM_I` 0x75 = 0x71 / 0x73 | — |
| ICM-42670-P | 0x68, 0x69 | `WHO_AM_I` 0x75 = 0x67 | Yahboom YB-EET01 |
| ICM-20948 | 0x68, 0x69 | reg 0x00 = 0xEA | bench Pico 2, 2026-09-23 |
| QMI8658 | 0x6A, 0x6B | `WHO_AM_I` | GenDrv |
| LSM6DSOX | 0x6A, 0x6B | reg 0x0F = 0x6C | bench Pico 2, 2026-09-23 |
| GY85 (ADXL345 + ITG3200) | 0x53 / 0x68 | reg 0x00 = 0xE5 / 0x68 | — |
| BNO085 | 0x4A, 0x4B | — | — |

**Read the last column as the honest state.** A dash means the driver exists
and no real chip of that part has answered on a bench yet. The GenDrv's QMI8658
is read at 50 Hz, like every other part here.

| Magnetometer | I2C addr | notes | bench |
|---|---|---|---|
| AK09918 | 0x0C | standalone, or inside an ICM-20948 (see below) | yes — GenDrv |
| AK8963 / AK8975 | 0x0C | `WIA` 0x00 = 0x48 | no |
| AK09916 | via ICM-20948 | reached through the IMU's bypass at 0x0C | yes — bench Pico 2 |
| QMC5883L | 0x0D | | no |
| HMC5883L | 0x1E | | no |

**Every part is read by polling**, at the control loop's rate. Where a chip has a FIFO with
its own timestamp counter, that carries the sample time with the sample and needs no wiring.

**One chip, two roles.** The ICM-20948 carries an AK09918 on its internal auxiliary bus.
`ICM20948IMU::startSensor()` sets `INT_PIN_CFG.BYPASS_EN`, which puts that magnetometer on
the *main* bus at 0x0C -- and leaves it there until the part is power-cycled. So a scan of
a board that has run this firmware once sees 0x0C answer and cannot tell it from a
standalone AK09918. `i2cProbeFoldComposites()` attributes it to the part that owns it,
keyed on the ICM-20948 being present, so a genuine standalone AK09918 is untouched. Two
consequences worth knowing: the magnetometer only works because IMU init runs before MAG
init, and after a power cycle 0x0C is silent until it does.


**The QMI8658 driver is the base's own.** The QST/Waveshare reference copy it replaced carried
a 2.2 s on-chip calibration at every boot that `calibrateGyro()` then repeated, a hard-coded
0x6B, three bus transactions per sample, and a `getData()` override that the `IMUInterface*`
the sketch holds never reached -- `getData()` is not virtual, so its bias handling was dead
code. The driver in `default_imu.h` is 200 lines: WHO_AM_I at 0x6B then 0x6A, a soft reset,
±8 g and ±1024 dps at 224 Hz with the on-chip low-pass at ~30 Hz (the base publishes at 50 Hz,
so the filter sits at its Nyquist), and **one 20-byte burst per sample** -- STATUSINT, STATUS0,
the 24-bit sample counter, temperature, six axes. `readGyroscope()` does the burst and
`readAccelerometer()` hands back the other half of it, which is the order `getData()` calls
them in. `tests/test_qmi8658_driver.py` pins each of these.

**Simulated wheels do not imply a simulated IMU.** Skipping the I2C sensors whenever `use_sim_wheel: true`
would be right for a bare module with nothing on the bus and wrong for a board with no encoders and
a real IMU: `/imu/data` would be the simulation at 50 Hz and the driver you meant to test would
never run. Only the sensors that are themselves sim are synthesised from the simulated wheels (`imu_from_wheels = sim_wheels && imu_is_sim`, likewise
the magnetometer): a real IMU the config or the bus names is initialised with simulated wheels
too, and one that fails to init on such a board falls back to the
simulation with `[imu] init FAILED on a simulated-wheel board - falling back to the simulated IMU`
rather than the fatal LED loop. The bus probe also prints reg 0x00 / 0x0F / 0x75 for a device
it cannot name, so an unfamiliar chip is identified from the boot log.

**The S3 image is built for a 4 MB flash, the smallest S3 module.** The bootloader trusts the
image header's flash size. Built for the DevKitC-1 board file's 8 MB, the same image put the
Yahboom's 4 MB module into a reboot loop before `setup()` ever ran -- `Detected size(4096k)
smaller than the size in the binary image header(8192k). Probe failed`, an assert in
`do_core_init`, `Rebooting...` -- while the flash itself had reported success. One image per MCU
means building for the smallest module (`board_upload.flash_size = 4MB` in `[base_esp32s3]`): a
larger flash goes unused, and the env partition at 0x3FF000 is the last sector of 4 MB either
way, the layout the 4 MB ESP32 DevKit has always had. The flasher now reads the ROM at 115200
when no application banner arrives and puts those lines in the log, so a boot loop is named as
one instead of "no banner".

**`console: uart0` -- the S3 image talks where the board's USB actually is.** The S3 images
are built with `ARDUINO_USB_CDC_ON_BOOT=1`, so `Serial` is the chip's native USB (HW CDC/JTAG on
GPIO 19/20) and UART0 (GPIO 43/44) is `Serial0`, which nothing used. That fits the DevKit and not
a board whose only USB is a bridge on UART0 -- the Yahboom's is a CP2102 on 43/44, and the
released image printed its banner, and ran micro-ROS, into a port the board does not have. The
MCU is the same, so the choice is the env's: `base_controller.console` (`usb`, the default, or
`uart0`; Config Studio's "ESP32-S3 Console Port") is read at the top of `setup()`, before the
first print, and every `Serial.` in this tree goes through `lino_console`
(`common/lib/mcu_env/lino_console.h`), a Stream that forwards to `Serial` or `Serial0`. The
`#define Serial lino_console` that makes that happen is scoped to the translation units that
include it -- all of ours, via `mcu_env.h`; the core and the vendored libraries keep the real
object -- and it defines nothing on a board without CDC-on-boot. A bridge never waits for a host,
so `boot_serial_wait` is skipped on UART0. `tests/test_console_uart0.py` walks the tree for a
translation unit that prints without the wrapper.

**The Yahboom's IMU is a TDK ICM-42670-P, not the QMI8658 its documentation names.** The bus
probe now prints reg 0x00 / 0x0F / 0x75 for a device it cannot name, and the V2.0 board on the
bench answered at 0x68 with WHO_AM_I (0x75) = 0x67. `ICM42670IMU` in `default_imu.h` is the
driver: ±8 g and ±1000 dps at 200 Hz low-noise with the 25 Hz UI filter, and one 14-byte
burst (temperature, six axes) in the byte order `INTF_CONFIG0` declares.

The window is measured rather than assumed, because the report rides in the publish path and that
path does not start until the micro-ROS agent connects. On a serial leg the agent is already
waiting and the window is 5.0 s; over Wi-Fi the board must join an AP first and the same board at
the same ODR reported 1576 edges in 7.9 s. Both are 200 Hz. A fixed "in the first 5 s" label read
correctly only in the first case.

The init follows TDK's own driver, because the first attempt did not and never got past
WHO_AM_I: the probe had just read 0x67 from 0x68, and the driver's own read of the same register
came back empty. A raw-transaction trace showed the **first I2C transaction after the boot-time
bus scan returns nothing and every later one answers**. The part has I3C on by default
(`INTF_CONFIG1` bits 3:2), and the scan ran to 0x7E -- the I3C broadcast address. So:
WHO_AM_I is tried three times, 1 ms apart, before the part is declared absent (the success line
says which try answered); the bank selects are zeroed and I3C switched off before the soft reset
and again after it, since the reset restores the defaults; `INT_STATUS` must then show
`RESET_DONE`; and the bus scan covers the assignable addresses 0x08..0x77 only. A still board
publishes exactly 0 rad/s: the base class zeroes any axis inside ±0.01 rad/s after the bias is
removed, which is older than this driver and applies to every IMU.

**The Yahboom microROS control board** (`config/reference/yb_eet01_config.yaml`,
YB-EET01 V2.0, ESP32-S3) is the first reference design with the line wired: ICM-42670-P at 0x68 on
SDA 40 / SCL 39 with INT on GPIO 41, dual-input drivers on M1 4/5 and M2 15/16 (the `pwm` enable
is `-1`, tied high on the board), encoders 6/7 and 47/48, battery ADC on GPIO 3 for a 2S pack,
LED on 45 -- a strapping pin, which the inspector warns about and nothing else. The board's own
firmware inverts the right motor; so does the config.

### A diagnostic must read the env, not the macro it was compiled with
`initBoard()` opens the bus with `envInt("i2c_sda", SDA_PIN)` — the env partition first, the header
macro only as a fallback. `i2c_detect`'s banner printed `SDA_PIN` *alone*, so on a prebuilt release
image — built from a generated bare config, where both pins are `-1` — it announced
`Scanning I2C bus (SDA:-1, SCL:-1)...` and then found the chip anyway. Every user of the shipped
image was told their wiring was unread while the scan was using it correctly. It now resolves the
pins the same way the bus does and names the source:

```text
Scanning I2C bus (SDA:0, SCL:1, from the env)...
```

The general rule: **when a tool's output disagrees with its own behaviour, suspect the macro/env
split before the wiring.** The code that acts and the code that reports must read the same source.

### `adc_calibrate` exists only where there is a DAC, and the fallback is nearly silent
It is compiled in only where `ADC_LUT_SUPPORTED` is 1 — the classic ESP32 and the ESP32-S2, the only
parts with a hardware DAC to sweep into an ADC pin. Everywhere else the name is not in `TOOLS[]` at
all, `toolSelect()` falls back to `APP_BASE`, and the entire explanation is one line printed at boot:

```text
[app] 'adc_calibrate' is not an application this image carries - starting the robot firmware instead.
```

Nobody is attached to the port during a 4 KB env write, so what an operator actually saw was a
successful write and a board reporting `app=base`. `flash_mcu.py` now warns at the point of choice,
from the same list of parts as the firmware's guard. Either way the rule is the same one the whole
env mechanism rests on: **`Board reports: … app=<name>` is the only proof the switch took** — a
successful write is not.

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
`firmware/adc_calibrate` does not print `const int16_t ADC_LUT[4096]` over serial to be pasted into
a config header and rebuilt: under tool mode there is nothing to paste into, so
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
A robot config carries `pio_env:`; build the env it names and no other. There is **one ESP32 env**:
the transport is installed at boot by `initUrosTransport()` from the env partition's `transport` key.
That is also why there is one ESP32 *config* — `config/reference/gendrv_config.yaml`. A serial and a
udp4 DevKit reference beside it would describe the same silicon and differ only in keys the env
decides at boot; `-e esp32` builds gendrv, and `transport=` in the env picks the rest.

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
shared image built for a robot whose config says `imu: SIM` it reported the simulated driver's zeros on
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

**Simulation mode is a `base` feature and stops there.** `use_sim_*` exists so a bare module with nothing
wired can still bring a ROS 2 stack up; every other application in the image is a bench diagnostic
for a REAL robot, and a simulated sensor or a simulated wheel is worth nothing to it. So do not "add sim
support" to a tool, and do not read a tool's output as evidence about simulation mode — if a tool reports
zeros, the question is what is on the bus, not which simulation is selected.

### Generate the header for the robot you are about to flash
The generated `firmware/include/custom/lino_base_config.h` is whatever robot was last generated, and
it is a single file for the whole tree. Flashing a Pico 2 with an image built from an ESP32 Wi-Fi
header — real motor pins, `udp4` transport, a board with neither — left the board not enumerating at
all (`device descriptor read/64, error -110`) and needing a physical BOOTSEL replug. The pipeline
generates it per run and now pushes it to **every** box of the pair, the build box included; a
manual `pio run` must do the same by hand.

### The `/cmd_vel` wire contract follows the image's distro, and the env can override it
nav2 1.4 (kilted) flipped `nav2_util::TwistPublisher` to `TwistStamped`, so **lyrical drives
`/cmd_vel` stamped and jazzy drives it plain**. Both subscriptions are compiled into every image.
At boot `main.cpp` picks one from `FW_ROS_DISTRO`, the distro the image was linked for: plain
through humble, iron and jazzy, stamped after that, the same rule as
`gen_firmware_header.distro_stamps_cmd_vel`, which a test holds equal. The env key
`stamped_cmd_vel` overrides it, and `mcu_env.py` writes it only when the config says `true` or
`false`; `auto` leaves the image's default. Until 2026-09-25 this was the `USE_STAMPED_CMD_VEL`
macro, and before that `auto` fell through every truth test and silently meant *off*, so a lyrical
image always listened for plain `Twist`.
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
| sim/real LiDAR | inherits `agent_ip` | 8889 | the LiDAR driver is a ROS 2 node too |
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
launch tree's `robot_localization` subscribes best-effort already:
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

**Default: off; the env key `dual_core` turns it on** (`mcu_env.py` writes it from `use_dual_core`).
Only ESP32 silicon has the second core (`HAS_DUAL_CORE` in `main.cpp`); an RP2 ignores the key.
It was the `USE_DUAL_CORE` macro until 2026-09-25.
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
firmware side is the OTA responder, compiled in wherever there is a radio (`HAS_WIFI`), plus
`telemetry.ota_port` (3232). When it is built, it takes
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
  `battery_pin bat_r1 bat_r2 bat_min bat_max bat_cap` are env keys now, read by `battery.cpp`;
  so are `sim_battery` (the simulated pack) and `bat_dip` (the sag detector's threshold).

### /battery is 1 Hz and carries the sag
The pack is sampled at 10 Hz and published at exactly 1 Hz. The message's voltage is the **lowest**
reading of that second, which is the sag under load; the percentage, and the voltage
`rpm_track_voltage` feeds the kinematics, come from a running average. A sample more than `bat_dip`
percent (default 2, config `pins.battery.dip_pct`) below the average is logged to syslog as a dip,
at most once a second. Two faults this replaced: the detector sat behind `#ifdef BATTERY_DIP`,
which nothing defined, so no image had it; and it published an extra message per dip, so the
topic's rate followed the load.

`sensors.use_sim_battery` is a simulated battery voltage sensor (`battery.cpp`, env `sim_battery`),
on in every bare robot. Open-circuit voltage falls from `max_v` to `min_v` as the charge drains
(a 3S 9.9-12.6 V, 5 Ah pack when the config names none). The wheel model's developed pack sag
divides it under load, so it dips when the simulated robot accelerates. A 0.3 A idle draw stands
for the electronics, and an empty pack is swapped for a full one, so a soak runs forever.
The Sim MCU runs this same firmware on the robot computer (`firmware/host/`), so it publishes the same model.

### No robot feature is a build macro; only silicon is
A released image is built per MCU and describes no robot, so every choice a robot makes is an env
key read at boot. The conditionals left in the firmware name the silicon: `ESP32`,
`ARDUINO_ARCH_RP2040`, `HAS_WIFI` (a radio: every ESP32, and the W boards via `-D HAS_WIFI`),
`HAS_DUAL_CORE`, `CONFIG_IDF_TARGET_*` and `ADC_LUT_SUPPORTED`. The rest are the `#ifndef X`
fallbacks a blank env boots with. Removed on 2026-09-25: `USE_STAMPED_CMD_VEL` (env
`stamped_cmd_vel`), `USE_DUAL_CORE` (env `dual_core`), `USE_SYSLOG` / `USE_ARDUINO_OTA` /
`WIFI_AP_LIST` as gates (a radio compiles them in; `syslog_ip`, `ota_port` and `wifi` decide),
`WIFI_MONITOR` (env `wifi_monitor`, minutes, default 2), `USE_WIFI_TRANSPORT`, `USE_LIDAR_UDP`,
`USE_<driver>_MOTOR_DRIVER`, `ENV_COV` (env `env_cov`), the `IMU_TWEAK` / `MAG_TWEAK` hooks, the
legacy `MAG` fallback, and `BATTERY_DIP`.

Where an external library did something trivial, the firmware does it itself now: syslog (one UDP
datagram in the same RFC 5424 form), the INA219 (config `0x1807`, shunt/bus reads, current = shunt
voltage / 0.01 ohm), the QMC5883L (two register writes and a 6-byte read), and the i2cdetect grid
(`i2cScanTable()` in `i2c_probe`). The QMC5883L library spun forever on its data-ready bit, so a
silent chip hung the control loop. It, and ten other sensor drivers, also called `Wire.begin()` in
`startSensor()`, which on an RP2 reset the bus clock `initBoard()` had set from the env. Adafruit
BusIO, the Adafruit PWM Servo Driver and I2Cdevlib-HMC5843 were listed and never used. The
I2Cdevlib core and its drivers stay: the vendored MPU9250/AK8963 code is built on it. The
`bno085_cal` tool is gone: it only tared a BNO085 (a mower's level reference), and the orientation
this stack fuses needs no tare.
- The LiDAR emulator was a build macro only (`USE_SIM_LD19`). A prebuilt image is built from
  a simulation-mode reference, so every real robot that flashed it raycast a room and streamed it.
  `sim_ld19=0` in the env (from `sensors.use_sim_ld19`) switches it off; `lidar_x` tells it
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
(`diag_tx`, `diag_baud`, `app`, `dac_pin`, `boot_serial_wait`, `banner_hold`). Add a key on one side only
and CI fails with its name.


### The diagnostic UART and the LiDAR emulator cannot both have UART1
`diagBegin()` and `SimLD19::begin()` both do `new HardwareSerial(LIDAR_SERIAL)`. On an ESP32
that is one peripheral behind two objects, and the second `begin()` simply re-points it: diag
starts first (`setup()`, before the env's LiDAR pin is read), the emulator starts second, so the
emulator wins the pin and the diagnostic stream goes nowhere — with nothing said either way.
Every diagnostic run so far happened to have `sim_ld19=0`, which is why it never showed.

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

Both are reachable now. `SimLD19::flushUdp()` and `lidar.cpp`'s UART receive callback check
`WiFi.status()` on every call and drop the buffer when it is down — checked at send time, not at
`begin()`, because the radio can arrive long after `setup()`. This is not hypothetical on a
released image: the published `esp32` firmware is built from the Wi-Fi profile
(`build_prebuilt.py`), so `USE_LIDAR_UDP` is compiled into the image that a serial ESP32 robot
runs with `wifi=0`. The emulator's counters carry `udp_noradio` so a bench can see it happening.

### An AP that does not answer must not stop the boot
An unbounded `while (wifiMulti.run() != WL_CONNECTED) delay(500);` stops a board whose AP refuses
it inside `setup()`: the console ends at `[wifi] using SSID '<x>' from the env partition` and stays
there — no I2C scan, no banner, no micro-ROS, no syslog (which needs that radio), and no reset
either, so it looks bricked while it is in fact waiting.

The wait is bounded (`WIFI_CONNECT_TIMEOUT_MS`, 20 s) and says which way it went; the robot
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
    map_width: 10.0          # the room, metres, centred on the origin
    map_height: 6.0
    wall_obstacle: true      # the obstacle the Nav2 goal sits behind
    wall_x1: 2.0
    wall_y1: -1.5
    wall_x2: 2.0
    wall_y2: 1.5
    robot_radius: 0.30       # else the largest robot_radius the costmaps plan with
    robot_mass: 3.5          # kg; sets the spin-up time constant
    wheel_noise_rpm: 1.0     # +/- peak white noise on the reported RPM
    gear_efficiency: 0.75    # fraction of motor torque the reduction returns
    gear_drag_rpm: 12.0      # constant (Coulomb) drag, RPM/s, while turning
    battery_sag: 0.25        # fraction of V_oc lost at four-wheel stall current
    battery_sag_tau_ms: 400  # how long the pack takes to sag, and to recover
    driver_drop: 0.03        # bridge's fixed voltage floor, any current
    driver_resistance: 0.10  # bridge's current-proportional loss, instant
```

**Every key here is optional, and every key is worth writing down anyway.** A
key that is absent from the env is not written, and the firmware keeps its
`#ifndef` default — the values above *are* those defaults, so this block sets
nothing it would not already do. That is deliberate in both directions: a blank
env boots, and a config a person reads tells them the whole simulated robot
rather than only the parts someone chose to override. The generated configs
therefore carry the block in full.

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

The simulated room is configuration because **simulation mode is the default here**:
a Nav2 test wants the obstacle wall somewhere else without rebuilding, and a
12 kg robot does not accelerate like a 3.5 kg one.

### The simulated drivetrain is a brushed DC gear motor, not a ramp

`sim_wheel.h` used to move the wheel toward its commanded RPM with a first-order
lag, which made every robot equally capable: four driven wheels accelerated
exactly like two, and a 12 kg base like a 3.5 kg one. That is not a motor, and
the difference matters because **simulation mode gates the release** — a Nav2 limit that
a real robot cannot meet has to fail on the bench, not in October.

So the model is the motor's actual torque–speed line, and each term is separate
because each behaves differently:

| term | env key | what it is |
| --- | --- | --- |
| torque–speed | — | `accel ∝ (no_load_rpm − wheel_rpm)`: a brushed DC motor's torque falls linearly from stall to no-load, so acceleration dies as the wheel approaches its commanded speed and is greatest from rest. |
| gearbox efficiency | `gear_efficiency` | a spur reduction returns 70–80% of the torque put in, so it scales the driving term. Reflected inertia (`N²·J_motor`) stays lumped into `SIM_WHEEL_TAU_MS`, where it belongs. |
| Coulomb drag | `gear_drag_rpm` | a gear train's loss is roughly **constant**, not proportional to speed. It is why an unpowered gear motor stops instead of coasting, and why a small duty produces no motion at all. Guarded so it can never push the wheel backwards through zero. |
| viscous drag | — | `SIM_WHEEL_FRICTION`, proportional to current RPM; bearings and the motor's own windage. |
| pack sag | `battery_sag`, `battery_sag_tau_ms` | `V_bus = V_oc − I·R_internal`, and for a brushed motor the current is proportional to the same `(no_load − ω)` term the torque uses — so a hard acceleration browns out its own supply. One pack, four wheels: **this is the only coupling between the simulated wheels.** It is also *lagged*: internal resistance drops the voltage at once but the chemistry polarises over hundreds of ms, so a **held** load sags deeper than a brief one. That is the real reason a heavy robot suffers more than its peak current suggests — being heavy means drawing that current for longer. |
| driver loss | `driver_drop`, `driver_resistance` | the bridge keeps some of the voltage. A fixed fraction (body-diode / V_ce floor, worst at low duty) plus a current-proportional one (R_ds(on) and the shunt) that follows the current **instantly**, unlike the pack. Two terms rather than one fudge factor because their time constants differ. |
| current limiter | `motor_stall_amps`, `driver_current_limit` | many small drivers chop at a fixed current — a TB6612 around 1.2 A per channel, an AT8236 or DRV8871 set by a sense resistor. It caps **torque**, not speed, so it bites hardest from rest at full duty and is barely engaged near top speed: a limited robot is *sluggish, not slow*. `0` means none fitted. |
| supply voltage | `motor_v` / `power_v` | scales no-load RPM: a motor rated at 12 V on a 2S pack does not reach `MOTOR_MAX_RPM`. |

Four consequences are worth stating, because each was a mistake first:

- **`MOTOR_MAX_RPM` and `max_rpm_ratio` are different ceilings.** 140 rpm is what
  the motor can spin; `max_rpm_ratio` derates *commands*, so the controller asks
  for less. A report that quotes one where it means the other either flatters the
  robot or accuses a healthy bench measurement of exceeding the motor.
- **`max_rpm_ratio` is the driver margin, and the default is 80%.** It was 0.85 on
  every robot — a 15% derating somebody picked. What it expresses is how much of
  the motor's no-load speed the controller may ask for, and the answer is
  deliberately *not* "as much as it can reach". `suggest_max_rpm_ratio()` measures
  what the motors do reach on `test_acc`'s 1 s profile — 0.91 on this chassis —
  and then takes the **smaller** of that and `DEFAULT_MARGIN` (0.80). So the
  measurement can only make a robot more conservative, never less: a 15 kg base
  that reaches 55% gets 0.55, and a 1.4 kg one that reaches 93% still gets 0.80.

  The reason is on the bench. With the smoother at `[0.3, 0, 1.23]` the 2wd slice
  came back **7/10 twice**; at `[0.3, 0, 1.2]` it came back **10/10** — same
  firmware, same boards, a 2.5% difference in the yaw ceiling. skid_steer and
  mecanum absorbed the identical rise at 10/10, so they are nowhere near their
  cliff and 2wd is sitting on its. A margin is headroom by definition, and
  spending it because a measurement says you could is how you find the edge.
- **Only the *ratio* of limit to stall current matters.** The model carries the
  motor's torque capability in `SIM_WHEEL_TAU_MS`; the amps only say at what
  current that torque arrives. So a motor drawing twice the current for the same
  torque is hurt exactly twice as much by the same driver. The tempting reading —
  "a bigger motor behind a small driver changes nothing" — is *not* what this
  model says, because a bigger motor is a different `max_rpm` and `tau` too.
- **Stall is where every loss bites at once.** At stall the current proxy is 1.0,
  so the motor sees `(1 − driver_drop) / (1 + battery_sag + driver_resistance)` of
  the pack, times whatever the limiter allows — which is why a real stalled motor
  never produces its datasheet stall torque.

Nothing here needs a rebuild: every term above is an env key, so a sweep is a
4 KB env write per value. That is the point — it is how you find out *which* loss
a navigation failure was sensitive to.

#### The model is *run*, not only solved — so `test_acc` is not how you read it

`performance()` solves the model in closed form, which answers "what can this
chassis do". That is not what `test_acc` reports, and the two differ in ways that
matter: the tool samples at 20 ms and differentiates the samples, it drives raw
PWM in 1 s phases so the pack's sag has a specific amount of time to develop, and
it gets the stop distance by integrating the coast, which no closed form gives.

So `drivetrain_report.py` transcribes `SimEncoder::integrate()` and `busScale()`
— same terms, same order, same clamps, same shared pack — and runs them on
`test_acc.cpp`'s own profile, printing the same four lines the tool prints. It is
noiseless on purpose: `getRPM()` adds ±`SIM_WHEEL_NOISE_RPM` and the tool
differentiates it, which is about 0.4 m/s² of pure instrument error in the board's
`MAX ACC` column.

Given that, **flashing `test_acc` is no longer how these numbers are obtained.**
On a simulated-wheel board the tool now refuses and points at the host script, because
what it would otherwise print is a measurement *of the simulator*, taken over a
serial line after a flash, unable to vary mass or gearing without another one. It
remains the right tool for a robot with motors on it — and then it is how this
transcription gets checked, which is a different and much rarer job.

Two numbers come out, and the gap between them is real rather than an error:

```
max speed           0.69 m/s      <- the asymptote, given longer than a second
MAX VEL             0.64 m/s      <- what test_acc's 1 s phase actually reaches
```

Torque falls as the wheel speeds up, so the last few percent arrive slowly.
**Tune a velocity smoother from the measured block**, because that is what the
robot does in the second a manoeuvre lasts.

### `drivetrain_report.py` — enter a config, get what the motors can deliver

```
python3 scripts/drivetrain_report.py --params config/reference/gendrv_config.yaml
```

It answers the question nothing in this repo asked before: **are the Nav2 limits
in this config asking for more than these motors can give?** They were. The
shipped limits wanted 103% of a differential base's motors and 171% at the
velocity smoother's ceiling; on mecanum — which turns on `(lr + fr)/2` rather
than `lr/2`, so the same `angular.z` costs 66% more wheel speed — 129%. Nav2 then
commands what it cannot get, `Kinematics` scales the entire request down to fit,
tracking degrades, and three mecanum legs left the room on 2026-09-23 before
anyone looked at the motors.

The report prints, for the drivetrain the config declares:

```
motor 140 rpm no-load at the wheel, controller will ask at most 119
max speed           0.69 m/s       5.09 rad/s
accel, first kick   3.17 m/s2     23.39 rad/s2   (stiff pack, from rest)
accel, sag settled  2.57 m/s2     18.97 rad/s2   (what a held manoeuvre gets)
```

then checks each Nav2 and velocity-smoother limit against the right one of those
— a *held* manoeuvre gets the settled figure, not the first kick — and adds the
translate-plus-rotate demand at the outer wheel, which is where a combined
request actually saturates.

Two rules keep it honest:

- **The model constants are parsed from `sim_wheel.h`, never copied.** A tool
  that restates another file's numbers drifts from it silently and then describes
  a robot that does not exist. If a macro is renamed the report exits rather than
  guessing.
- **Capability and budget are reported separately** (`motor_rpm` vs
  `command_rpm`). Conflating them once produced "max speed 0.60 m/s" for a bench
  that had just measured 0.71 m/s.

Because the whole model is env-driven, the report also works as a **design tool**:
put a candidate chassis, gearing, mass and pack into a config and read off the
speed and acceleration before ordering anything.

### The same numbers live in the Config Studio, and Nav2's limits follow them

The Kinematics HUD on the **Base & MCU** tab shows all of it, and recomputes on
every keystroke: achievable top speed, first-kick and settled acceleration, the
simulated `test_acc` columns, the rotation radius *with the rule that produced
it*, the wheel-speed budget, and each Nav2 limit checked against the right one of
those. Mass, gearbox efficiency, gear drag, pack sag and its time constant, the
two driver losses and the current limiter are editable fields right below it.

It is computed server-side (`POST /api/drivetrain/performance`, which calls
`drivetrain_report.py`) rather than in the browser. A JavaScript reimplementation
would be a second opinion about the robot that drifts from `sim_wheel.h`
silently — the same rule the report itself follows by parsing its constants out
of the header instead of restating them.

**Nav2's limits are then derived from the motors on save, by default.** The
alternative is what shipped: limits asking 103% of a differential base's motors,
171% at the smoother's ceiling and 129% on mecanum, with nothing checking.
`kinematics.auto_nav2_limits: false` hands control back to somebody tuning by
hand, and then nothing is rewritten — the HUD still shows what the motors
suggest, which is the point of showing it separately from what the file says.

What gets written, and the rule behind each fraction:

| key | rule |
| --- | --- |
| `velocity_smoother.max_velocity` | 47% of the measured top speed; 26% of the measured yaw rate. Rotation gets the *smaller* share because that is where tracking error grows fastest and, on a mecanum, where wheel speed is most expensive. |
| `velocity_smoother.max_accel` | 31% of the **settled** acceleration, not the first kick: a rate limit the base meets only while the pack is still stiff is one it misses for the rest of the manoeuvre. Angular: whatever reaches the angular ceiling in 0.8 s, capped at the same 31% of settled angular acceleration. |
| `FollowPath.desired_linear_vel`, `rotate_to_heading_angular_vel` | 83% of the smoother's ceiling, so the controller asks for something the smoother can pass through. |
| `behavior_server` spin limits | brisker than path following — a recovery spin happens when the robot is stuck, with nothing to track. |
| `kinematics.max_rpm_ratio` | the driver margin, above. |

The pair is then checked against the wheel-speed budget and **both** scaled down
together if it does not fit, because a translation and a rotation add at the
outer wheel and each looked survivable alone. Shaving only one would silently
change the robot's character into a base that turns but will not drive.

The fractions reproduce the limits this project settled on for its default
chassis on 2026-09-23, then scale with the model — so switching the feature on
does not re-tune a robot that was already right, and a mecanum finally gets the
lower yaw ceiling its `(lr + fr)/2` radius always required (1.23 → 0.74 rad/s).

### `topic_prefix` is an env key too, so the names are built at run time

Pasting `TOPIC_PREFIX` onto each topic literal with the preprocessor would mean
a build per robot to put two robots on one DDS domain -- and the published
images, built from the generated bare config, could not do it at all. It is `topic_prefix` in the
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
board looks perfect and the robot has no odometry. It is invisible with one
robot, because a single-robot stack is usually run without a prefix at all.

So `envPrefixed()` lives in `mcu_env`, beside the env reader it depends on, and
every frame the firmware stamps goes through it: `odom` and `base_footprint` in
the odometry, `imu_link` in the IMU, the magnetometer and the simulated wheels,
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
touched the header, so every simulation-mode board -- which is the default -- published
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
| `SimLD19 sim_ld19` | a static instance | `new` only when `sim_lidar_on` |
| the micro-ROS messages, executor and `Odometry` | static objects | allocated in `setup()` under `if (micro_ros)`, `rclErrorLoop()` if the allocation fails |
| `test_sensors`, `test_acc` working sets | file-scope buffers and driver objects | `calloc` / `new` in each tool's `setup_()`, with a null guard in `loop_()` |

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
