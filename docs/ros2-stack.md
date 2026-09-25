# ROS 2 stack: launchers, Nav2, SLAM and the agent

- **Colcon cleanliness:** must build via
  `colcon build --symlink-install --packages-select linorobot2_cockpit`.
- **No ROS 1 hallucinations:** never `roscpp`, `ros::init`, `ros::NodeHandle`, `moveit_commander`, or
  legacy parameter servers.
- **Lifecycle & QoS:** use `rclcpp::Node` / `rclcpp_lifecycle::LifecycleNode`. Use `SensorDataQoS`
  (best-effort, volatile) for continuous high-frequency topics like `/scan` and `/imu/data` — a Reliable
  subscriber silently receives nothing from a `SensorDataQoS` publisher.
- **Frames:** strictly REP-103 / REP-105 (`map` -> `odom` -> `base_link` -> sensor frames).
- **Docker IPC:** container runs include `--ipc=host -v /dev/shm:/dev/shm` (or `--shm-size=2g`) to avoid
  DDS costmap memory drops.

---

### 1-Click leaves the robot running, and something has to remember it

The pipeline leaves bringup, SLAM and Nav2 running when it finishes. Stopping them would be right
for an automated run and wrong for a person: pressing **Start 1-Click** is how you *get* a robot,
and a stack that is switched off on the way out gives you nothing on `/scan`, nothing to drive, and
a map saved from a stack that no longer exists. Pass `--shutdown-when-done` for the automated case.

The stack now stays up until it is stopped. `--shutdown-when-done` restores the old behaviour and is
what automation should pass: a bench run that leaves a stack behind floods the DDS domain for whatever
runs next.

The consequence is that **the pipeline exits while its children keep running**, so the Stop buttons
would have nothing to signal. `scripts/robot_stack.py` is the record — tag, pid and process group per
part — kept in the shared state directory, because the pipeline runs as container-root and the backend
as the container user and anything under `$HOME` is two different files (see
`cockpit_paths.state_dir`). Signals go to the process GROUP, never a name match: every launch starts
with `os.setsid()`, so one signal reaches `ros2 launch` and everything it spawned, and a pgid cannot
hit the wrong process the way `pkill ros2` can.

    python3 scripts/robot_stack.py            # what is still running
    python3 scripts/robot_stack.py --stop     # all of it
    python3 scripts/robot_stack.py --stop nav2

In the cockpit, the Bringup / SLAM / Nav2 **Stop** buttons reach it through `/api/stack/stop`, and
stopping *bringup* stops the whole stack — SLAM and Nav2 on top of a dead robot are not worth keeping.
An entry whose group has gone is dropped on read rather than reported as running, so the UI never
claims a robot that is not there and never signals a pgid that now belongs to somebody else.

---

### The installed ROS wins over the configured distro, for the same reason the bus wins over the config
`ros_distro` is **not a property of the robot**. It belongs to whichever machine runs the stack, so a
robot config that pins it is making a claim about a computer — and `/opt/ros` *is* that computer, exactly
as the I2C bus is the hardware (docs/firmware.md). `config/rover_pico2_config.yaml` declares `ros_distro: jazzy`, and
the supervisor took it unconditionally: opened on a lyrical-only box the Cockpit header announced
**"Jazzy (Noble 24.04)"** while every node on that box came from `/opt/ros/lyrical` — with the agent's own
`/opt/ros/lyrical/bin/ros2` command line sitting two keys further down *in the same `/api/status`
payload*. Nothing on screen resolved the contradiction.

It is not cosmetic, because **"Start 1-Click" reads that selector**: the button would have run a jazzy
pipeline against a box with no jazzy installed. `web/backend/main.py` now honours the configured value
whenever that distro is actually present (a machine carrying both still does what the config asks) and
overrides it otherwise, reporting the override in `ros_distro_override` rather than applying it
silently — the same contract `i2cProbeSelect()` keeps when it disagrees with the YAML.

### A controller name must select a config, never just rename one
`--controller` is the only selector the Cockpit backend passes to
`one_click_pipeline.py`, but the pipeline resolved `config/<robot>_config.yaml` from `--robot` alone.
With `--robot` absent it loaded the **default** robot and then overwrote nothing but the name, so
`--controller esp32_wifi` announced esp32_wifi while compiling `pio_env: pico2` and flashing
`/dev/ttyACM0` — one robot's controller name bolted onto another robot's hardware, which is AGENTS.md read
backwards. Resolution order is now: `--robot`, then `config/<controller>_config.yaml`, then the config
whose `base_controller.name` matches. Two robots may legitimately declare the same controller
(`linorobot2` and `rover_pico2` both declare `pico2`), so the first by filename wins **and the run says
which it picked**; `--robot` disambiguates.

### The bringup health card measures by subscribing, not by `ros2 topic hz`
The card drew two red rows — `/odom/unfiltered` and `/imu/data`, "no messages" — on a robot whose
own 1-Click gate had just measured both at 50 Hz. The probe shelled out to `timeout 3 ros2 topic hz`
per topic. That CLI first looks the publisher up in the graph to copy its QoS, and under
`config/fastdds_service_qos.xml` (`type_propagation=registration_only`, the Fast DDS 3.x TypeLookup
fix) that lookup does not find the micro-ROS agent's publishers: `ros2 topic list` shows them,
`ros2 topic hz` says "does not appear to be published yet", and the 3 s kill left nothing to parse.
Isolated one variable at a time as the backend's own user: profile exported → "does not appear";
profile unset → 40 Hz. The pipeline's gate never had the problem because `verify_topics.py`
subscribes directly with the sensor-data QoS, which matches best-effort and reliable alike and
needs no graph lookup. The probe now does the same in one rclpy process for every advertised
topic, and waits up to 8 s but returns the moment every topic has rated — the agent's endpoints
take several seconds to match a new participant under this profile where host nodes match at once.

Two bench facts that came out of the same investigation, both easy to mistake for product bugs:
a stack launched as **root** (a `docker exec` without `-u`) leaves root-owned `/dev/shm` segments
the backend's user cannot read, and the card then shows *everything* red; and a reader without the
profile exported cannot see a stack that has it, and vice versa. Measure from inside the stack's
own environment or the numbers are about your shell, not the robot.

### A robot is what its file SAYS it is, and no file may be dropped
`robot.name` inside the YAML is the identity; the filename is only where that content lives. Two
files can therefore claim one name — and that is a conflict the user has to see, not one for the
listing to resolve quietly. `get_robots_list()` keyed a `seen` set on the name and skipped a repeat
with no log and no warning: on a bench box **three of nineteen configs were invisible in the
cockpit**, because three files all declared `robot: {name: pico2_real}` and a fourth duplicated
`bare_pico2`. One of the three was the real-hardware config being driven from the CLI at that very
moment — and the CLI never lost it, because `robot_config_path()` resolves `<dir>/<robot>_config.yaml`
by filename. The two halves of the product disagreed about which robots existed and only the UI came
up short, which is the worst direction for that disagreement to run.

Now there is one entry per **file**, named by its content, carrying `conflict` (the other files making
the same claim) and `select` — the handle that unambiguously reaches *this* file: the declared name
when it is unique, the filename stem when it is not, both of which `/api/robot/select` already
accepts. The picker renders a colliding entry as `name (filename)`. A dotfile is never a robot: the
cockpit keeps `.active_robot` and `.cockpit_token` in that directory, and one was being offered as
selectable.

Two UI-side corollaries, both of which produced the same silent mismatch:
- **The base-controller select must follow the robot selector.** 1-Click reads its controller from
  `#cockpit-target-select`, not from the robot dropdown, and `selectRobot()` synced distro, install
  mode, agent engine and registry but not that one. Switching robots and pressing Start therefore ran
  the *previous* robot's controller against the previous robot's port, with nothing in the UI saying so.
- **Every robot config declares `serial_port`.** `esp32_wifi_config.yaml` (since deleted) was the only one that did not,
  and the pipeline's fallback is `/dev/ttyACM0` — so a Wi-Fi-transport ESP32 would have had esptool
  pointed at whatever RP2 board was on the bench. Wi-Fi is the ROS 2 transport; USB is still the
  flashing bus.

### A ROS 2 params file is `<node>: ros__parameters:`, and the robot config is flat
The robot config stores `slam:` as a flat key/value block because that is the readable shape for a
single source of truth. `slam.launch.py` dumped it straight into the temp params file, whose first line
is then a scalar — rcl rejects the whole file (`Cannot have a value before ros__parameters at line 1`)
and `async_slam_toolbox_node` aborts on an uncaught `RCLInvalidROSArgsError`, exit `-6`, before reading
one scan. Wrap at the launcher, not in the config. This was not a new lesson: `bringup.launch.py` already wrapped
`ekf` as `ekf_filter_node: ros__parameters:` and carried a comment quoting the identical rcl error.
`nav2.launch.py` builds `<node>: ros__parameters:` natively. `slam.launch.py` was the one launcher that
never got the treatment — so when adding a launcher, check this first, and check the other two when one
of them fails this way.

### Nav2's `collision_monitor` has no usable default, and it takes the whole stack down
`nav2_bringup` hands our params file to every node it starts, so a node the robot config does not
describe gets no parameters at all. Most Nav2 servers have code defaults; `collision_monitor` reads
`observation_sources` during `on_configure` and errors out without it. Because `lifecycle_manager`
brings the set up as a unit, that one node aborts everything — *after* planner, controller, behavior
and smoother have all configured cleanly, so the log's last healthy lines are misleading. The launcher
now merges a working `collision_monitor` block **per key** (the `stamped_cmd_vel` loop has already
created `collision_monitor.ros__parameters`, so a `"collision_monitor" not in nav2_data` guard can
never fire and a wholesale assignment would drop `enable_stamped_cmd_vel`); key off
`observation_sources`, and take `base_frame_id` from the config rather than Nav2's `base_footprint`.

**`docking_server` is the second node of that class**, and it surfaced only once `bt_navigator` stopped
aborting the stack two nodes earlier: it reads `dock_plugins` during `on_configure` and fails with
*"Charging dock plugins not given!"*. Same per-key merge, keyed off `dock_plugins`. No dock *instance*
is supplied — Nav2's own defaults leave `docks` commented out, and inventing a home dock at the map
origin would make `dock_robot` drive at a fiction. On jazzy these are the only two of the ten servers
`lifecycle_manager_navigation` brings up that have no usable code default; the configure order ends
`… waypoint_follower, docking_server`, so a stack that reaches `docking_server` has cleared the rest.


### "Launched" is not "active", and Nav2 will happily look launched
`lifecycle_manager` brings the Nav2 servers up as a unit, so one node that fails to configure aborts the
whole set — and it aborts *after* the healthy ones have logged a clean configure, so the tail of
`logs/nav2.log` looks fine. So a fixed wait and an unconditional "✅ Nav2 stack launched" will cheerfully announce a stack that
has already died, and the goal driven into nothing then reads as a navigation failure.
`one_click_pipeline.py` calls `wait_for_nav2_activation()` instead, which watches the log for
`Managed nodes are active` versus `Failed to bring up all requested nodes` and names the node that
failed.

The node that failed was `bt_navigator`: `config/rover_pico2_config.yaml` was the only config still
naming its navigators in the **slash** form (`nav2_bt_navigator/NavigateToPoseNavigator`), which
pluginlib rejects — jazzy declares `nav2_bt_navigator::NavigateToPoseNavigator`, and every other config
(and every other plugin in that same file: costmap layers, progress checker, DWB) already used `::`.
**When a Nav2 plugin will not load, diff the plugin strings across `config/*.yaml` before reading Nav2's
source** — one file drifting from nine is the likelier story, and `grep -rn "nav2_[a-z_]*/" config/`
finds it in one command.

### `slam_toolbox` is a lifecycle node too, and "no map" is three different faults
`slam_toolbox`'s own launch file drives its transitions from a launch event handler, so when the
configure result event is missed the node sits in `inactive` forever: it logs `Configuring`, picks its
solver, and then says nothing. No map is published, the `map` frame never comes into existence, and
half a minute later Nav2 fails with `planner_server` unable to transform `base_link` to `map` — 121
identical `Invalid frame ID "map" ... frame does not exist` complaints downstream of a fault that is
not Nav2's. The transition is idempotent and cheap, so `one_click_pipeline.py` asks for it directly
(`ros2 lifecycle set --no-daemon /slam_toolbox activate`) rather than give up on somebody's race.

Use `--no-daemon`. The `ros2` CLI daemon caches the graph and was measured answering "Node not found"
for 36 s straight about a `slam_toolbox` that was active at the time, in the same container where
`--no-daemon` answered `active [3]` immediately. A recovery that asks the cache can be told the node
is not there.

**`SLAM: no map was published` is produced by three different faults**, and they are three different
repairs: the lifecycle node stuck in `inactive`; a solver plugin that would not load; or scans
dropped because `odom`→`laser` was not in the TF buffer yet, which looks identical from outside
because `/scan` is publishing at its full rate the whole time. So the pipeline no longer tells you to
read `logs/slam.log` — on a bench that file lives inside a container the next run destroys. It reports
the **last lifecycle transition reached** (no `Activating` is the stuck-in-`inactive` case by itself)
and the distinct complaints, with numbers normalised so one dropped-scan fault carrying a moving
timestamp is a single counted row instead of a page of near-identical ones:

```
     --- slam_toolbox reached: Creating -> Configuring
     it never activated: the lifecycle node is stuck in `inactive`, so no map is
     published and the map frame never exists
     --- what slam_toolbox complained about (distinct, most frequent first) ---
        12x Message Filter dropping message: frame 'laser' ... queue is full
```

Same reason, and the same shape, as the Nav2 complaint report: **extract the evidence while the log
still exists.** A failure whose explanation dies with its container is a failure you get to have twice.

### A node may subscribe to a topic with exactly one type, and a swallowed stderr hides why
`test_nav2_goal.py` subscribed to `/cmd_vel` as **both** `Twist` and `TwistStamped`, meaning to accept
whichever the run used. rcl rejects the second — *"create_subscription() called for existing topic name
rt/cmd_vel with incompatible type"* — and raises in the **constructor**, so the test aborted before
sending a goal on every run it had ever made. Two things kept that invisible for as long as it was:
`one_click_pipeline.py` printed only the test's stdout (empty on that path) and not its stderr, and the
Nav2 stack was genuinely down for unrelated reasons, so `returned 1` always looked adequately explained.
Pass the type in (`--cmd-vel-type`, which the pipeline already computes for the launcher) or read it off
the graph; **and print stderr whenever a step fails**, not just stdout.

Corollary, learned the same day: a pipeline must not report a run as successful because it reached the
last line. `one_click_pipeline.py` printed "✅ 1-Click Pipeline Finished Successfully!" and returned 0
after a red X on Nav2 and a warning from the map saver. Steps that deliberately do not abort the run
(the map is still worth saving when Nav2 is down) accumulate into `failures`, and the banner is either
that list with exit 1 or the success line.

### Lyrical is a different Nav2, not the same Nav2 on a newer Ubuntu
The jazzy leg passing proves nothing about lyrical. Four independent things differ, each of which
takes the stack down in a way that does not name itself; all four were found on a Pico 2 bench on
2026-09-16 and all four are already solved on `linorobot2`'s **`console` branch** — read it before
re-deriving any of this (AGENTS.md).

- **There is no `ros-lyrical-nav2-bringup` binary.** `apt-cache madison` lists it under *Sources*
  only and `apt-cache policy` reports no candidate, because its exec-depends reach Gazebo simulation
  packages resolute does not publish. Build it from `github.com/ros-navigation/navigation2` branch
  `<distro>`, **together with the `navigation2` metapackage** — nav2_bringup's CMakeLists
  `find_package()`s it and it has no binary either — with `COLCON_IGNORE` on the other ~40 packages
  in the monorepo so colcon does not rebuild what apt just installed.
- **Install the nav2 runtime by apt name pattern, never a hand-kept list.** On jazzy
  `ros-jazzy-navigation2` pulls the stack in one dependency; lyrical publishes the parts and not the
  metapackage. `^ros-<distro>-(nav2|opennav)-` gets all 39. The `opennav` half is not optional:
  lyrical's `navigation_launch.py` brings up `docking_server` **and `following_server`**, and neither
  name starts with `nav2_`.
- **`navigation_launch.py` composes but does not contain, and on 1.5.1 it manages nothing.**
  With `use_composition:=True` it only issues `LoadComposableNodes(target_container='nav2_container')`;
  creating that container belongs to `bringup_launch.py`, which is what console includes. Include
  navigation_launch.py by itself and the load requests go to a container that does not exist, in
  total silence — `logs/nav2.log` holds the launch banner and nothing else. And nav2 1.5.1 moved the
  lifecycle manager out of that file into `bringup_launch.py`'s single `lifecycle_manager_nav2`, so
  even once the container exists all eleven servers load and sit in state 1 (unconfigured) with
  nobody to transition them: eleven `Loaded node` lines, not one `Configuring`, and
  `wait_for_nav2_activation()` correctly reporting neither success nor failure.
  `launchers/nav2.launch.py` therefore starts **both** the container and a manager — but only where
  the installed launch file has no manager of its own, since jazzy's still does and a second one
  would fight it for the same nodes. **Ask the installed file, never the distro name**, and read the
  managed-node list out of it too: it is spelled `lifecycle_nodes = [...]` on jazzy and
  `def get_lifecycle_nodes(): return (...)` on lyrical, and lyrical's set grew by three.
- **Composition is gated on that same check, deliberately.** nav2 defaults `use_composition` to
  `False` on both distros, so False is what the green jazzy hardware run actually used. Composition
  is applied where the problem is (console measured composed 2 passes / 2 runs vs uncomposed 1 / 2 on
  lyrical) and jazzy is left byte-for-byte as it was.

### rmw_fastrtps drops service replies, and nav2 hangs with no error of its own
`rmw_fastrtps` will not send a service response until the server's response writer has matched the
client's response reader, and it waits at most the writer's `max_blocking_time` — which Fast DDS
defaults to **100 ms**. Bringing nav2 up starts a crowd of participants at once, that match misses the
window, and the reply is **dropped, not delayed**: `lifecycle_manager` then waits forever for a
`change_state` answer from a server that configured perfectly, and a different node loses the race
every run. `load_node` is a service too, so composition fails the same way.

`config/fastdds_service_qos.xml` raises it to 10 s for endpoints matching the profile named `service`
and leaves ordinary topics alone. It is a ceiling, not a delay — a healthy machine pays nothing.
Diagnosed upstream on console (`eae75c8`, `0c6c779`) after a campaign of timeout raises that could
not have helped; do not raise a nav2 timeout to chase this symptom again.

The file reaches every process the same way: the robot image exports `FASTDDS_DEFAULT_PROFILES_FILE`
container-wide (`docker-compose.yml`, `docker/Dockerfile`), and `runners.py` / `one_click_pipeline.py`
export it for native runs, so a stack started from the browser and one started from the CLI get the
same profiles.

### Fast DDS 3.x can leave one endpoint unmatched forever, and nav2 never activates
On lyrical (Fast DDS 3.6) `bt_navigator` failed its *activate* transition on 8 of 13 runs with
`"compute_path_to_pose" action server not available after waiting for 30.00s` — or `spin`, or
`is_path_valid`; the endpoint changed, the failure did not — while a bystander participant listed
both the server and the client in the graph the whole time. Fast DDS 3.x can gate an endpoint match
on a TypeLookup exchange: rmw registers a type when the *local* endpoint is created, nav2's servers
announce themselves before `bt_navigator` has built the clients that need them, so a 33-node bringup
performs those lookups routinely, and `TypeLookupManager` has no retry and no timeout. One lost reply
and that endpoint never matches; `rmw_fastrtps` then reports the server unavailable (its check wants
the client's request-writer and response-reader match counts equal and non-zero) for as long as
anyone waits. Fast DDS 2.14 on jazzy matches by type name and never showed it.

The same XML sets `fastdds.type_propagation` to `registration_only` on the default participant
profile: types are still registered locally, the announcement carries no type information, matching
falls back to the name, and the lookup path is never entered. 8 of 8 with it, on both bench machines.
It is not shared memory (fails with `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` and on a freshly cleaned
`/dev/shm`), not CPU (fails on a tuned, idle machine), not config (both robot configs fail), and not
a nav2 bug (`1.5.1..1.5.2` changes nothing here).

### Never add the ROS 2 apt source twice
The `ros2-apt-source` .deb writes `/etc/apt/sources.list.d/ros2-apt-source.**sources**` with the key
embedded; the classic recipe writes `ros2.list` with `Signed-By` pointing at a keyring. Apply both
and apt sees one source described twice with different `Signed-By`, prints *"Conflicting values set
for option Signed-By"* and then refuses to read **the whole sources list** — so `apt-get install git`
fails too, and a per-package fallback loop reports every package as unavailable. Check for either
file (the deb writes `.sources`, not `.list`), and check `apt-get update` itself rather than
inferring from what installed.

### The description is generated from the config, not picked from a shelf
Including `linorobot2_description`'s `description.launch.py` with a xacro chosen by
`kinematics.base_type` alone does not work: those files carry a 90 mm wheel on a 260 mm
track, while the reference GenDrv robot has 152 mm on 271 mm. Odometry would be right (it comes from
the env) and the TF tree wrong, so the robot sits at the wrong height and its wheels turn at
the wrong scale in every viewer -- a mismatch nothing reports, because SLAM and Nav2 never
consult wheel geometry.

Now `scripts/gen_robot_description.py` writes `<config dir>/generated/<robot>.urdf` from the
config at every launch (and the supervisor rewrites it on every save), and bringup starts
`robot_state_publisher` + `joint_state_publisher` on that text. Plain URDF, not xacro: nothing
at run time has to find a package. Wheel radius and axle spacing come from `kinematics`; the
body box, tyre width, axle height, casters, LiDAR and IMU poses from `geometry:`; a config with
no `geometry:` gets one derived from its kinematics by `migrate_config_schema.py`, written into
the file. Frames are fixed by the rest of the stack and are not config keys: `base_footprint`
(the firmware's odometry child), `base_link` (the EKF's), `imu_link` (what the firmware stamps
on `/imu/data`); only the laser frame is a key (`geometry.laser.frame`), because the LiDAR
driver is told it too. The LD driver's `product_name` and `bins` follow `lidar.model`.
`linorobot2_description` is no longer vendored into the image and `xacro` is no longer installed;
the fresh image was proven on the GenDrv bench first (generated description published, the
`base_link`->`laser` transform equal to `geometry.laser`) before either was removed.


### The heading needs an anchor, and the EKF has to be told to use it
The chain is: magnetometer -> the board's own AHRS -> `/imu/data` orientation -> the EKF's
**absolute yaw**. Every link has to be on, and each one is set in a different file.

Until 2026-09-25 the middle link was `imu_filter_madgwick`, running on the robot computer and
pairing `imu/data_raw` with `imu/mag`. It is now `firmware/common/lib/imu/ahrs.h`, the same filter
ported onto the board, because the pairing made `/imu/data` the rate of *matched pairs* across a
best-effort link -- see `docs/firmware.md`. Nothing else in this section changes: the frame
convention, the anchoring, and the EKF's `imu0_config[5]` rule are what they were, and the
covariance the board publishes (1e-4 with a field) is the variance the node's
`orientation_stddev: 0.01` implied.

* The board fuses the field whenever a magnetometer is fitted *or* simulated, in ENU — x east,
  **y north**, z up, which is the frame the firmware's simulated field points along. It is fixed
  in `ahrs.h`, not a parameter. Without a field the board fuses gyro and accel alone, its yaw is
  the gyro's own integral, and `bringup.launch.py` clears `imu0_config[5]` so the EKF does not
  take that drift as absolute. A real robot may have no magnetometer; that is the path it takes.
* The robot config's `ekf.imu0_config` must fuse index **5**, absolute yaw — the 15-element vector
  is `[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`. Every shipped
  config now fuses `yaw, vyaw, ax, ay` from the IMU and `vx, vy, vyaw` from the wheels, which is
  what the upstream linorobot2_hardware wiki specifies.

Without index 5 nothing looks broken. The robot drives, the map builds, and the heading error is
absorbed into `map -> odom` — a transform nothing prints. It reaches you as a map view drawing the
live scan at an angle to the walls it has just built, and it grows all run, because this EKF fuses
velocities and has no absolute reference of its own. `gen_firmware_header.py`'s `config_warnings()`
now says so when an IMU is fitted and index 5 is false, and when two sources both claim absolute
yaw — two headings that disagree make the filter split the difference.

`odom0_config` fuses **vy** as well as vx. On a differential base the firmware publishes vy as an
exact zero with a real covariance, which is the non-holonomic constraint stated as a measurement,
not missing data; on a mecanum base it is a velocity in its own right.

The board also removes gravity before publishing, because the EKF fuses `ax` and `ay` and
`two_d_mode` forces the filter level: on any real slope gravity would otherwise leak into the
horizontal axes and be read as acceleration. It is removed once, from whatever quaternion the
message carries — the AHRS's, or a BNO085's own — and `bringup.launch.py` sets
`robot_localization`'s `imu0_remove_gravitational_acceleration` to false, so nothing subtracts it
twice. The topic gate (`verify_topics.py`) adds the same gravity back to prove a real
accelerometer is reading: a dead one leaves a specific force of zero.

Both the IMU and the magnetometer must be **calibrated** or the pose rotates. The simulated
magnetometer carries a hard-iron offset on purpose, so `robot_calibration`'s
`magnetometer_calibration` has something to find, and the firmware removes it by default — a
simulated robot starts where a real one does after the routine has been run. Uncorrected that
offset is worth about 7.4 deg of heading at rest.


### The IMU filter's `dt` is the interval that passed, not a constant
The board's AHRS integrates over the interval `micros()` actually measured, not over
`CONTROL_TIMER`, and skips an interval longer than a second as a stalled loop rather than
integrating it as rotation. A nominal period standing in for a measured one is the mistake that
once let `imu_filter_madgwick`, run with `constant_dt: 0.02`, integrate 20 ms per message it
*received* -- so every message a best-effort link dropped was time that never happened -- and the
one that let a simulated wheel model report 4x its motor's speed when a loop stalled.

### One `topic_prefix`, consumed on both sides, so two robots share a DDS domain
`base_controller.topic_prefix` is a single key with two readers. The board prefixes every
topic name it publishes (`topicName()` in the firmware, from the env block); the host stack
joins the board by putting itself in the **same namespace** and prefixing its **TF frames** to
match. `cockpit_paths.robot_namespace()` normalises the key exactly the way `scripts/mcu_env.py`
does before writing it — letters, digits, underscore and `/`, no surrounding slashes, an
invalid value read as unset — so neither side has to be told what the other did.

Unset is the default and every branch is then a no-op, which is why the single-robot layout is
byte-identical to what it was before the feature existed.

Three things are worth knowing if you touch this:

- **TF stays on the global `/tf`.** Namespacing a node would give it `/<prefix>/tf`, and then
  no other robot — and no global RViz — could see the tree. The frames carry the prefix
  instead (`robot_state_publisher`'s `frame_prefix`, the EKF's frame parameters, the LiDAR's
  `frame_id`), and the usual `/tf` remap is dropped under a namespace so it resolves globally.
- **Nav2's parameter file names frames and topics in full, and would ignore the namespace.**
  `_prefix_nav2_namespace()` rewrites those values under `/<prefix>/` before the params are
  handed over; a relative name would otherwise resolve against the node and a `/`-absolute one
  would escape the namespace entirely.
- **A lifecycle manager drives nodes by name.** Under a namespace the servers it manages have
  full paths, and anything that is handed a node path — the SLAM node, the manager's
  `node_names` — must be told the full one, not wrapped a second time by `PushRosNamespace`.
- **A params file is matched by the node's fully-qualified name**, and a section that matches
  nothing is not an error. See below; it is the subtlest of the three by a distance.

### A params file the node does not match is not an error — it is defaults, silently

`launchers/*.launch.py` build their params files by hand, because the robot config holds these
settings flat and a ROS 2 params file must be `<node>: ros__parameters: <keys>`. rcl matches
each section against the node's **fully-qualified** name. Under a namespace the node is
`/<prefix>/ekf_filter_node`, so a file keyed `ekf_filter_node:` matches nothing at all — and
nothing is exactly what happens: no warning, no failure, no clue. The node starts with its own
defaults and reports them as its configuration.

For `robot_localization` that is close to the worst case, because its defaults include *no
inputs*:

```
$ ros2 param get /lino1/ekf_filter_node frequency     # the config asked for 50.0
Double value is: 30.0
$ ros2 param get /lino1/ekf_filter_node odom0
Parameter not set
```

So the EKF subscribed to nothing, published nothing, and produced no `odom -> base_footprint`
transform, while `ros2 node list` showed it running and `/lino1/odom` showed an advertised
publisher. Everything upstream of it — 50 Hz odometry from the board, prefixed topics, prefixed
URDF frames — was perfect, which is what made it hard to see.

`cockpit_paths.namespace_params()` re-keys every section to `/<ns>/<node>` before the file is
written, which is the job nav2's own `RewrittenYaml(root_key=…)` does for the nav2 tree. All
three launchers pass their file through it. `/**` would also match, and is the usual shorthand,
but it hands **every** section's parameters to **every** node — wrong the moment the file
describes more than one, as nav2's does. An unset prefix returns the dict unchanged, so the
single-robot path is byte-identical to what it was.

The general form is worth keeping in mind: **when you namespace a node, everything that refers
to it by name has to be told.** Its params sections, its lifecycle manager's `node_names`, and
any full topic or frame name written into a config file.

### A bare module has no LiDAR tty, so the virtual room runs on the robot computer
`use_sim_ld19` says a scan is simulated; it does not say *where*. Two benches want different
answers, and picking the wrong one either bypasses the code under test or leaves the stack with
no `/scan` at all:

- **A board with a real serial bridge** (the GenDrv emits its emulated LD19 out `LIDAR_RXD`
  into a USB-serial adapter) must be read by the **real `ldlidar` driver**. Routing that to a
  host node would bypass the very driver path the bench exists to exercise.
- **A bare module** has one USB, for micro-ROS, and nothing else. The LiDAR tty never appears,
  the serial driver dies on the missing port, `/scan` never comes, and SLAM and Nav2 sit there
  waiting — on the configuration that is supposed to be the easiest one to run.

So the host emulator (`scripts/sim_laser_node.py`) takes over only when the scan is simd
**and** either the mode is not serial or the configured port does not exist:

```python
use_host_sim_laser = (
    controller.get("sensors", {}).get("use_sim_ld19", False)
    and (effective_lidar_comm_mode != "serial" or not os.path.exists(lidar_port))
)
```

Testing the port rather than the config is what keeps the bench-with-a-bridge case unchanged
while letting a module with nothing soldered to it run the whole pipeline. Both emulators
raycast the same room from `geometry.laser.x`, so the scan agrees with the transform either
way.
