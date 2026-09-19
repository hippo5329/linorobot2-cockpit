# Contributing

Thanks for looking at this. The cockpit drives real hardware, so a few rules here are not
style preferences: breaking them can brick a board mid-flash or leave a robot driving with
no way to stop it. They are short, and each one exists because it was learned the hard way.

## The invariants

These hold everywhere: scripts, the web supervisor, launch files, CI and anything you add.

1. **No broad process kills.** Never `pkill`, `killall` or `pkill -f`. Signal a PID or a
   process group you started yourself. Bracket any `pgrep` pattern (`'[m]icro_ros_agent'`)
   so the search does not match itself.
2. **Port immunity.** Ports 8000 (the supervisor), 5173 and 9090 (rosbridge) belong to the
   cockpit. Never terminate whatever owns them.
3. **One robot per config file.** Each `<robot>_config.yaml` carries exactly one
   `base_controller:` block with its own kinematics, EKF, SLAM and Nav2 settings. There is
   no global parameters file and no map of targets. Credentials live only in
   `secrets.yaml`, which is gitignored.
4. **Build and flash are separate steps.** `pio run` compiles. `scripts/flash_mcu.py`
   writes, through esptool or picotool. Never `pio run -t upload`, and never
   `-t nobuild -t upload`.
5. **Browser WebSerial is a monitor, never a flasher.**
6. **The bus wins over the config.** A YAML file is a claim about the hardware; the USB
   device and the I2C bus are the hardware. When they disagree, stop and say so rather than
   writing an image built for the wrong silicon.
7. **Free the serial port before touching a board.** Stop `micro_ros_agent` by PID or unit
   and confirm `lsof <port>` is empty before any flash or monitor.

## Running the checks

Everything here runs on a workstation with no robot attached.

```bash
python3 -m pytest tests                 # unit tests: env block, header, URDF, access, pins
python3 -m py_compile scripts/*.py launchers/*.py web/backend/*.py
node --check web/frontend/app.js && node --check web/frontend/rosviz.js
python3 scripts/migrate_config_schema.py --dry-run     # exits 1 if a shipped config would change
for c in config/reference/*_config.yaml; do
    python3 scripts/gen_firmware_header.py --params "$c" --distro jazzy
    python3 scripts/mcu_env.py build --params "$c" --out /tmp/env.bin
done
```

Firmware and the ROS package need their toolchains:

```bash
pio run -d firmware -e pico2            # esp32 esp32s3 pico picow pico2w, and <env>_lyrical
colcon build --symlink-install --packages-select linorobot2_cockpit
ros2 launch --print-description linorobot2_cockpit bringup.launch.py
```

`--print-description` goes **before** the launch file, not after: `ros2 launch` builds a
positional list and an option in the middle of it splits the arguments.

Do not run ROS 2 or PlatformIO on a machine you care about unless it is disposable. A
container or a VM is the right place.

## Hardware claims need evidence

If a change says a board behaves a certain way, the pull request should show it: the topic
rates, the boot console, the flash log, whatever demonstrates the claim. "Should work" is
not a test result, and a board that was never started looks exactly like a board that
published nothing. Say plainly what you ran, on which board, and what you did not run.

## Pull requests

- Branch off `main`. Keep a pull request to one subject.
- A commit message says what changed and **why**, with the failure that motivated it. The
  history here is the project's memory; a one-word message throws that away.
- Update the documentation next to the rule you changed. Long-form knowledge lives in
  `docs/firmware.md`, `docs/flashing.md` and `docs/ros2-stack.md`.
- Add or adjust a test when the behaviour is testable without hardware.
- Nothing private in this repository: no hostnames, LAN or VPN addresses, Wi-Fi names,
  registries, credentials or bench topology. Configs and docs describe boards, not benches.

## Releases

Release candidates are `rc-YYYYMMDD` branches and tags with a two-week freeze; releases are
datestamped `YYYYMMDD` tags. A release publishes the firmware archives and the container
images, so CI must be green before one is cut. See `docs/` and the release workflow.

## Where things are

`README.md` has the quick start, the repository layout and the configuration reference.
`docs/firmware.md` covers the one-image-per-board design and the env partition,
`docs/flashing.md` the flashing and BOOTSEL rules, and `docs/ros2-stack.md` the launch tree,
QoS and the generated robot description. Each is written as lessons with the failure that
taught them, so read the one nearest what you are about to change.
