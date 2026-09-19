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

Two UI-side corollaries, both of which produced the same silent mismatch:
- **The base-controller select must follow the robot selector.** 1-Click reads its controller from
  `#cockpit-target-select`, not from the robot dropdown, and `selectRobot()` synced distro, install
  mode, agent engine and registry but not that one. Switching robots and pressing Start therefore ran
  the *previous* robot's controller against the previous robot's port, with nothing in the UI saying so.
- **Every robot config declares `serial_port`.** `esp32_wifi_config.yaml` was the only one that did not,
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
`logs/nav2.log` looks fine. `one_click_pipeline.py` used to `time.sleep(8)` and print "✅ Nav2 stack
launched" unconditionally; on 2026-09-16 it printed that 24 s after the stack had died, then drove a goal
into nothing and reported `Nav2 obstacle test returned 1` — blaming the goal for a stack that was never
there. It now calls `wait_for_nav2_activation()`, which watches the log for `Managed nodes are active`
versus `Failed to bring up all requested nodes` and names the node that failed.

The node that failed was `bt_navigator`: `config/rover_pico2_config.yaml` was the only config still
naming its navigators in the **slash** form (`nav2_bt_navigator/NavigateToPoseNavigator`), which
pluginlib rejects — jazzy declares `nav2_bt_navigator::NavigateToPoseNavigator`, and every other config
(and every other plugin in that same file: costmap layers, progress checker, DWB) already used `::`.
**When a Nav2 plugin will not load, diff the plugin strings across `config/*.yaml` before reading Nav2's
source** — one file drifting from nine is the likelier story, and `grep -rn "nav2_[a-z_]*/" config/`
finds it in one command.

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
export it for native runs. Until 2026-09-19 only the CLI pipeline did, so anything started from the
browser ran without these profiles.

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
`bringup.launch.py` used to include `linorobot2_description`'s `description.launch.py` with a
xacro chosen by `kinematics.base_type` alone. Those files carry a 90 mm wheel on a 260 mm
track; the reference GenDrv robot has 152 mm on 271 mm. Odometry was right (it comes from the
env) and the TF tree was wrong, so the robot sat at the wrong height and its wheels turned at
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
on `/imu/data_raw`); only the laser frame is a key (`geometry.laser.frame`), because the LiDAR
driver is told it too. The LD driver's `product_name` and `bins` follow `lidar.model`.
`linorobot2_description` is no longer vendored into the image and `xacro` is no longer installed;
the fresh image was proven on the GenDrv bench first (generated description published, the
`base_link`->`laser` transform equal to `geometry.laser`) before either was removed.


### The IMU filter's `dt` comes from the stamps, not from a constant copied out of the firmware
`imu_filter_madgwick` used to run with `constant_dt: 0.02`, mirroring the firmware's
`CONTROL_TIMER` (20 ms). The firmware does publish `/imu/data_raw` once per control period,
but what *arrives* is not always 50 Hz: an RP2040 at 921600 delivers ~40 Hz, and best-effort
QoS drops a message rather than stall the link. With a constant `dt` the filter integrates the
gyro for 20 ms per message it *receives*, so every dropped message is time that never happened
and the heading drifts short under rotation. The firmware stamps every message with the
agent-synced epoch (`getTime()` after `syncTime()`, the same stamp `/odom` carries and the EKF
already trusts), so `constant_dt: 0.0` — the package default, "use the header stamps" — is the
right setting on every board and at every baud rate. Do not put the constant back to make a
bench number look steadier.
