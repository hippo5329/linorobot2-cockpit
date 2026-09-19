# Flashing and serial-bus rules

Everything here runs on the robot computer, which owns the USB bus.

### Build and Flash Are Always Separate (never `pio run -t upload`)
`pio run` compiles and nothing else. Every write to a board goes through `esptool write_flash` (ESP32) or
`picotool load -f` (RP2040/RP2350), invoked natively.
- Rationale: the two halves normally run on different machines. `pio run -t upload` welds them together,
  hides which binary actually reached the board, and bypasses the recovery paths.
- `-t nobuild -t upload` is equally banned. `scripts/flash_mcu.py` flashes only; pass `--build` for the
  single-machine case where the same box compiles and flashes.
- There is no host-to-robot transfer any more: the robot computer fetches a release image
  (`scripts/fetch_prebuilt.py`, every file checked against the manifest's SHA-256) or builds one
  itself, and flashes it. The old `deploy_firmware.py` / `POST /api/firmware/upload` staging path
  went with the host role.

### RP2350 BOOTSEL: identify by interface class, never by USB PID
The Pico 2 enumerates as `2e8a:000f` in **both** application mode and BOOTSEL, so the PID proves nothing.
Read the interface classes instead — `lsusb -d 2e8a: -v | grep bInterfaceClass`:

| interfaces | meaning |
|---|---|
| `Communications`, `CDC` | application mode, a tty exists |
| `Mass Storage`, `Vendor Specific` | BOOTSEL, picotool can load |

The firmware exposes no picotool reset interface, so `picotool reboot -f -u` can never reach a running
board; the only software route into BOOTSEL is the **1200-baud touch** (open at 1200, DTR high 100 ms,
DTR low 300 ms, close), which lands in under 1.5 s on a healthy board.

**`Errno 110` on the 1200-baud touch is not a diagnosis — it is the normal outcome of a touch that
worked.** The DTR-low ioctl is a `SET_CONTROL_LINE_STATE` control transfer, and the device is *supposed*
to reboot inside it and never answer; the host then times out (`ETIMEDOUT`). Success and failure produce
the identical errno. The discriminator is what is on the bus a second later (table above): `Mass Storage`
is BOOTSEL and the touch worked; still `Communications`/`CDC` **and mute at every baud** is the hang
described next. **There is no `firmware/blink` any more** — see docs/firmware.md.

### The 1200-baud touch can hang an RP2350 outright, and only RESET recovers it
The touch is handled in the core, not in our firmware: arduino-pico's
`SerialUSB::checkSerialReset()` (`cores/rp2040/SerialUSB.cpp`) disables `USBCTRL_IRQ`, resets the USB
block, calls `reset_usb_boot(0, 0)` and then executes `while (1); // WDT will fire here`. On RP2350
`reset_usb_boot()` is no longer the RP2040 ROM entry point — the SDK maps it to
`rom_reboot(REBOOT2_FLAG_REBOOT_TYPE_BOOTSEL | REBOOT2_FLAG_NO_RETURN_ON_SUCCESS, 10, …)`, declared
`noreturn`, **with its result discarded**. When that reboot does not take, the board is left in that
`while (1)` with USB torn down and its interrupt off: still enumerated, mute at every baud, answering no
control transfer. That is the state previously recorded as "wedged".

Three consequences, all load-bearing:
- **picotool is not the culprit.** It reported "No accessible RP-series devices in BOOTSEL mode" — which
  was true — and our own pyserial touch reached the same state. The failure is device-side, in the core.
- **The core's comment assumes a watchdog that nothing armed.** `firmware/src/main.cpp` now arms one on
  RP2 (`rp2040.wdt_begin(8000)`, fed at the end of `loop()`), so a failed BOOTSEL request costs a reboot
  instead of a trip to the bench. `rclErrorLoop()` deliberately does **not** feed it: on ESP32 that loop
  is an OTA recovery path and keeps feeding, on RP2 it now reboots and retries.
- **A board flashed before that change still needs the RESET button** — not a BOOTSEL replug, just RESET.
  `scripts/flash_mcu.py:rp2_usb_mode()` reads the interface classes after a failed touch and says so by
  name rather than repeating "not in BOOTSEL".

**We do not depend on that fix.** This firmware arms its own watchdog, so the core's `while (1)` is
already bounded on every board we ship; the upstream change only helps sketches that never call
`rp2040.wdt_begin()`. Treat it as a contribution, not a dependency, and never as a build prerequisite.

Per AGENTS.md the durable fix belongs upstream all the same, and it is forked:
**`github.com/hippo5329/arduino-pico`, branch `fix/rp2350-bootsel-touch-hang`** (based on upstream
`master` c82d1d55) — arms the watchdog immediately before `reset_usb_boot()` so the `while (1)` is
bounded for every sketch. Opened as
[earlephilhower/arduino-pico#3532](https://github.com/earlephilhower/arduino-pico/pull/3532);
**changes requested**, the maintainer asking for an MCVE and observing that "Pico2 uploads should fail
99% of the time because most sketches don't ever use the WDT".

That objection has an answer the PR has not yet made: the `while (1)` is reached **only when the reboot
does not take**. `rom_reboot()` is `noreturn` on success, so the overwhelming majority of uploads never
execute that line at all — which is why uploads are not failing 99% of the time, and equally why an MCVE
is awkward: the path is intermittent by construction. What the patch changes is the cost of the rare
failure, from a board that is mute at every baud until someone presses RESET to one that reboots in
8 s.

### WebSerial Is a Monitor, Never a Flasher
The browser's Web Serial API is reserved for the raw serial debug terminal. Only the native path can stop
`micro_ros_agent`, confirm the tty was released, and restart it; a half-written flash bricks the board.

### `/dev/ttyACM0` is not a board — name it by udev's by-id path
The tty number is the order the kernel enumerated in, and it moves. Two Raspberry Pi Picos on
one bench swapped numbers between 2026-09-18 and 2026-09-19 with nothing touched but a reboot:
a config saying `/dev/ttyACM0` now pointed at the RP2350 instead of the RP2040, and the flash
was refused by the pre-flash guard (`'pico' builds for RP2040, but the board is RP2350`) rather
than writing the wrong image. The guard is the safety net; the fix is to stop using the number.

udev publishes a name that carries the USB serial:

```
/dev/serial/by-id/usb-Raspberry_Pi_Pico_D665C007DA2A1336-if00 -> ../../ttyACM1
```

`base_controller.serial_port` accepts that path, and so do `--port` on `flash_mcu.py` and
`mcu_probe.py`. Each resolves it once, up front, and passes the real node to everything
downstream (lsof, esptool, picotool, the agent), printing `port <by-id> -> /dev/ttyACM1` so the
log says which board it means. A by-id path that no longer resolves is left alone, so the error
names what the user configured instead of a guess. The port chips in the UI show the tty with a
`·id` marker when a stable name exists, and clicking one puts that stable name in the config.

It is not a substitute for the identity check. A by-id path is only as good as the last time
the board was plugged in; the guard still reads the silicon before every write.

### Serial Port Safety & Micro-ROS Release Protocol
- Before flashing (`esptool write_flash`, `picotool load -f`) or opening any serial monitor
  (`pio device monitor`, `minicom`), **ALWAYS stop the running `micro_ros_agent`** to release
  `/dev/ttyUSB*` / `/dev/ttyACM*`. Never probe or flash a port the agent still holds.
- Identify holders: `lsof /dev/ttyUSB* 2>/dev/null`, or `ps aux | grep micro_ros_agent | grep -v grep`.
- Stop it by targeted PID (`kill -SIGINT <PID>`) or `systemctl --user stop micro_ros_agent`.
  **NEVER `pkill` / `killall`.**
- Verify release (`lsof <port>` empty) before starting the upload or monitor session.

### picotool takes `-t` and `-o` AFTER the filename
`picotool load [--family <id>] [-v] [-x] <filename> [-t <type>] [-o <offset>]`. Written the natural
way — options first — picotool 2.1.1 answers `ERROR: unexpected option: -o`, and because the env
write sits inside the flash recovery path whose next step is a successful application load, **the
whole flash still reported success while the board kept whatever env it had**. A board flashed on a
fresh box then ran on the header fallbacks with no env block at all.

And an `--env-only` write must reboot the board itself: `picotool load` without `-x` leaves it
sitting in BOOTSEL, where the tty is gone and the robot looks dead for a reason unrelated to what
was written. `picotool reboot -f` reaches it there (that is not the `reboot -f -u` which cannot
reach a *running* board).

### Record every flash, from every path
`flash_mcu.py` has six success paths — the primary flash and five recovery routes. Recording the
stamp in only the primary one lost it on the first real run: the board was in application mode, so
the flash landed in recovery 4 (1200-baud touch, then picotool), which returned 0 directly, and the
next probe reported a board that had "never been flashed by this host" moments after this host
flashed it. One `flashed_ok()` exit point, always.
