// Linorobot2 Cockpit frontend -- micro-ROS agent lifecycle, ROS 2 / workspace / lidar auto-install, robot bringup.
// Part of the app.js split: a classic script sharing global scope. See app-core.js.

// ---------- micro-ROS agent: find-or-build, then launch ----------
// Blank means "follow the workflow". Defaulting to "docker" here is what made a
// fresh native install on a box without Docker announce "using docker container
// image" and then die with "ERROR: docker is not installed" -- the user had
// chosen nothing, and the selects ship with docker pre-selected, so the
// configured value never got a say. Derive it from install_mode instead.
function defaultAgentEngine() {
  const el = document.getElementById("hdr-install-mode")
    || document.getElementById("install-mode");
  const mode = (el && el.value)
    || (state.config && state.config.install_mode)
    || localStorage.getItem("linorobot2_install_mode")
    || "native";
  if (mode === "docker") return "docker";
  if (mode === "podman") return "podman";
  return "native";
}

function getAgentEngine() {
  const hdr = document.getElementById("hdr-agent-engine");
  if (hdr && hdr.value) return hdr.value;
  const cfg = document.getElementById("cfg-agent-engine");
  if (cfg && cfg.value) return cfg.value;
  return (state.config && state.config.agent_engine) || defaultAgentEngine();
}

function getContainerRegistry() {
  const hdr = document.getElementById("hdr-container-registry");
  const hdrCustom = document.getElementById("hdr-custom-registry");
  if (hdr && hdr.value === "custom" && hdrCustom && hdrCustom.value.trim()) {
    return hdrCustom.value.trim();
  }
  if (hdr && hdr.value) return hdr.value;
  const cfg = document.getElementById("cfg-container-registry");
  const cfgCustom = document.getElementById("cfg-custom-registry");
  if (cfg && cfg.value === "custom" && cfgCustom && cfgCustom.value.trim()) {
    return cfgCustom.value.trim();
  }
  if (cfg && cfg.value) return cfg.value;
  return (state.config && state.config.container_registry) || "auto";
}

function findOrBuildAgentCommand() {
  const engine = getAgentEngine();
  if (engine === "docker" || engine === "podman" || engine === "podman_systemd") {
    const bin = (engine === "docker") ? "docker" : "podman";
    const regMode = getContainerRegistry();
    const customReg = (document.getElementById("hdr-custom-registry")?.value || document.getElementById("cfg-custom-registry")?.value || (state.config && state.config.custom_registry) || "").trim();

    // Every mode probes whatever host the user configured -- nothing is
    // hardcoded. A shipped hostname would be a private address to everyone
    // except the machine it was written on, and it leaks that machine's name
    // into a public repository.
    let probeList = [];
    if (regMode === "dockerhub") {
      probeList = [];
    } else if (regMode === "custom" || regMode === "cluster" || regMode === "auto") {
      probeList = customReg ? [`"${customReg}"`] : [];
    } else {
      // An explicit host:port typed into the selector.
      probeList = [`"${regMode}"`];
    }

    let pullBlock = "";
    if (probeList.length > 0) {
      pullBlock = `REG_PULLED=0; ` +
        `for reg in ${probeList.join(" ")}; do ` +
        `  if curl -fsSL -m 2 "https://$reg/v2/" >/dev/null 2>&1 || curl -fsSL -m 2 "http://$reg/v2/" >/dev/null 2>&1; then ` +
        `    echo ">>> Container registry active at $reg. Pulling $reg/$IMG..."; ` +
        `    if ${bin} pull "$reg/$IMG" >/dev/null 2>&1; then ` +
        `      ${bin} tag "$reg/$IMG" "$IMG"; REG_PULLED=1; break; ` +
        `    fi; ` +
        `  fi; ` +
        `done; ` +
        `if [ "$REG_PULLED" -eq 0 ]; then ` +
        `  echo ">>> Pulling $IMG from upstream..."; ` +
        `  ${bin} pull "$IMG" 2>/dev/null || { echo ">>> no '$IMG' tag on Docker Hub, trying ':rolling'"; IMG="microros/micro-ros-agent:rolling"; ${bin} pull "$IMG" 2>/dev/null || true; }; ` +
        `fi; `;
    } else {
      pullBlock = `echo ">>> Pulling $IMG from Docker Hub directly..."; ` +
        `${bin} pull "$IMG" 2>/dev/null || { echo ">>> no '$IMG' tag on Docker Hub, trying ':rolling'"; IMG="microros/micro-ros-agent:rolling"; ${bin} pull "$IMG" 2>/dev/null || true; }; `;
    }

    return `echo ">>> micro-ROS agent: using ${bin} container image (skipping build from source)"; ` +
      `if ! command -v ${bin} >/dev/null 2>&1; then echo "ERROR: ${bin} is not installed" >&2; exit 1; fi; ` +
      `IMG="microros/micro-ros-agent:${state.status?.ros_distro || "jazzy"}"; ` +
      pullBlock +
      `echo AGENT_DOCKER_READY`;
  }
  return envPrefix() + [
    `[ -f ~/uros_ws/install/setup.bash ] && source ~/uros_ws/install/setup.bash`,
    `if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then echo AGENT_FOUND; ` +
      `else ` +
        `sudo apt-get install -y ros-$ROS_DISTRO-micro-ros-agent >/dev/null 2>&1; ` +
        `source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null; ` +
        `if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then echo AGENT_APT_OK; ` +
        `else ` +
          `mkdir -p ~/uros_ws/src && cd ~/uros_ws/src && ` +
          `{ [ -d micro_ros_agent ] || git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ROS-Agent.git micro_ros_agent || git clone -b rolling https://github.com/micro-ROS/micro-ROS-Agent.git micro_ros_agent; } && ` +
          `{ [ -d micro_ros_msgs ] || git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_msgs.git micro_ros_msgs || git clone -b rolling https://github.com/micro-ROS/micro_ros_msgs.git micro_ros_msgs; } && ` +
          `cd ~/uros_ws && colcon build && echo AGENT_BUILT; ` +
        `fi; ` +
      `fi`,
  ].join("; ");
}

function agentLaunchCommand() {
  const c = state.config || {};
  const transport = document.getElementById("cfg-agent-transport").value || c.agent_transport || "serial";
  const device = document.getElementById("cfg-agent-device").value || c.agent_device || "/dev/ttyACM0";
  const port = document.getElementById("cfg-agent-port").value || c.agent_port || "8888";
  const baud = document.getElementById("cfg-agent-baud").value || c.agent_baud || "921600";
  const engine = getAgentEngine();
  const distro = state.status?.ros_distro || "jazzy";

  const preClean = transport === "udp4" ? "" : `fuser -k -TERM ${device} 2>/dev/null || true; sleep 0.5; `;
  const devFlags = transport === "udp4" ? "" : `--device ${device}`;
  const agentArgs = transport === "udp4"
    ? `udp4 --port ${port}`
    : `serial --dev ${device} -b ${baud}`;

  if (engine === "podman_systemd") {
    const mode = transport === "udp4" ? "udp4" : "serial";
    return preClean + [
      `IMG="microros/micro-ros-agent:${distro}"`,
      `echo ">>> micro-ROS agent: Podman + systemd user service (${distro})"`,
      `podman pull "$IMG" 2>/dev/null || IMG="microros/micro-ros-agent:rolling"`,
      `podman run -d --replace --name "microros_agent_${mode}" --net=host ${devFlags} "$IMG" ${agentArgs}`,
      `mkdir -p "$HOME/.config/systemd/user"`,
      `podman generate systemd --new --name "microros_agent_${mode}" > "$HOME/.config/systemd/user/microros-agent.service" 2>/dev/null || true`,
      `systemctl --user daemon-reload 2>/dev/null || true`,
      `systemctl --user enable --now microros-agent.service 2>/dev/null || true`,
      `loginctl enable-linger "$USER" 2>/dev/null || true`,
      `echo ">>> micro-ROS agent running as persistent systemd user service: microros-agent.service"`
    ].join(" && ");
  }

  if (engine === "podman") {
    const mode = transport === "udp4" ? "udp4" : "serial";
    return `${preClean}podman run --rm --replace -it --name "uros_agent_${mode}" --net=host --privileged -v /dev:/dev ${devFlags} -e ROS_DOMAIN_ID=0 microros/micro-ros-agent:${distro} ${agentArgs}`;
  }

  if (engine === "docker") {
    const mode = transport === "udp4" ? "udp4" : "serial";
    return `${preClean}docker run --rm --net=host --privileged -v /dev:/dev ${devFlags} -e ROS_DOMAIN_ID=0 microros/micro-ros-agent:${distro} ${agentArgs}`;
  }

  // native
  const runLine = transport === "udp4"
    ? `ros2 run micro_ros_agent micro_ros_agent udp4 -p ${port}`
    : `ros2 run micro_ros_agent micro_ros_agent serial --dev ${device} -b ${baud}`;
  return envPrefix() + `[ -f ~/uros_ws/install/setup.bash ] && source ~/uros_ws/install/setup.bash; ` + preClean + runLine;
}

function _unused_old_agentLaunchCommand() {
  const c = state.config || {};
  const transport = document.getElementById("cfg-agent-transport").value || c.agent_transport || "serial";
  const device = document.getElementById("cfg-agent-device").value || c.agent_device || "/dev/ttyUSB0";
  const port = document.getElementById("cfg-agent-port").value || c.agent_port || "8888";
  const baud = document.getElementById("cfg-agent-baud").value || c.agent_baud || "921600";
  const isDocker = document.getElementById("cfg-agent-use-docker") ? document.getElementById("cfg-agent-use-docker").checked : (installModeSel?.value !== "native");

  if (isDocker) {
    const engine = (installModeSel?.value === "podman") ? "podman" : "docker";
    const devFlags = transport === "udp4" ? "" : `--device ${device}`;
    const agentArgs = transport === "udp4"
      ? `udp4 --port ${port}`
      : `serial --dev ${device} -b ${baud}`;
    return `${engine} run --rm --net=host --privileged -v /dev:/dev ${devFlags} -e ROS_DOMAIN_ID=0 microros/micro-ros-agent:${state.status?.ros_distro || "jazzy"} ${agentArgs}`;
  }
  const runLine = transport === "udp4"
    ? `ros2 run micro_ros_agent micro_ros_agent udp4 -p ${port}`
    : `ros2 run micro_ros_agent micro_ros_agent serial --dev ${device} -b ${baud}`;
  return envPrefix() + `[ -f ~/uros_ws/install/setup.bash ] && source ~/uros_ws/install/setup.bash; ` + runLine;
}

// The command that finds-or-builds / pulls the agent, and the one that runs it,
// are built SERVER-SIDE now (web/backend/actions.py). Here we only gather the
// operator's choices -- engine, registry, transport, device, baud -- and name
// the action. The registry host, if any, is whatever the operator typed.
function agentPrepareSpec() {
  const engine = getAgentEngine();
  const regMode = getContainerRegistry();
  const customReg = (document.getElementById("hdr-custom-registry")?.value
    || document.getElementById("cfg-custom-registry")?.value
    || (state.config && state.config.custom_registry) || "").trim();
  let registry = "";
  if (regMode === "custom" || regMode === "cluster" || regMode === "auto") registry = customReg;
  else if (regMode && regMode !== "dockerhub") registry = regMode;
  return { action: "agent_prepare", args: { engine, registry, distro: state.status?.ros_distro || getDistro() } };
}

function agentStartSpec() {
  const c = state.config || {};
  return { action: "agent_start", args: {
    engine: getAgentEngine(),
    transport: document.getElementById("cfg-agent-transport")?.value || c.agent_transport || "serial",
    device: document.getElementById("cfg-agent-device")?.value || c.agent_device || "/dev/ttyACM0",
    port: document.getElementById("cfg-agent-port")?.value || c.agent_port || "8888",
    baud: document.getElementById("cfg-agent-baud")?.value || c.agent_baud || "921600",
    distro: state.status?.ros_distro || getDistro(),
  } };
}

function ensureAgentRunning() {
  if (state.status && (state.status.agent_alive_external || state.status.agent_busy_console)) {
    return Promise.resolve();
  }
  return new Promise((resolve) => {
    runCommand(agentPrepareSpec(), {
      title: "Preparing micro-ROS agent",
      onDone: (code) => {
        if (code !== 0) {
          resolve();
          return;
        }
        const stopBtn = document.getElementById("btn-agent-stop");
        if (stopBtn) stopBtn.disabled = false;
        runCommand(agentStartSpec(), { slot: "agent", title: "micro-ROS agent" });
        setTimeout(resolve, 2500); // give the agent a moment to bind before the caller launches
      },
    });
  });
}

document.getElementById("btn-agent-ensure")?.addEventListener("click", () => ensureAgentRunning());
document.getElementById("btn-agent-stop")?.addEventListener("click", () => killSlot("agent"));

// ---------- auto-bringup & bringup launch helpers ----------
function isAutoBringupEnabled() {
  const el = document.getElementById("cfg-auto-bringup");
  return el ? el.checked : true;
}


function isAgentAlive() {
  return Boolean(state.status && (state.status.agent_alive_external || state.status.agent_busy_console));
}

// Install mode decides where things actually run. In Docker/Podman mode the
// workspace, the ROS packages and the sensor drivers all live inside the image,
// so the native pre-flight checks -- "is the workspace built", "is the LiDAR
// driver installed", "is nav2_bringup present" -- do not apply, and running
// them would try to build and apt-install on the host instead.
//
// The header select and the Install tab select mirror each other; fall back to
// the persisted config, then localStorage, then native.
function isDockerMode() {
  const el = document.getElementById("hdr-install-mode")
    || document.getElementById("install-mode");
  const mode = (el && el.value)
    || (state.config && state.config.install_mode)
    || localStorage.getItem("linorobot2_install_mode")
    || "native";
  return mode === "docker" || mode === "podman";
}

// The launch files need more than one package each, and finding out one at a
// time -- launch, read the "package 'x' not found" exception, install x, launch
// again -- is exactly what 1-Click is supposed to spare the user. Check the
// whole set and install what is missing in a single apt call.
//
// linorobot2_bringup pulls in robot_localization (ekf_node), imu_filter_madgwick
// (only when madgwick is on, but it is by default), robot_state_publisher and
// xacro. slam.launch.py pulls in slam_toolbox *and* nav2_bringup; navigation
// needs nav2_bringup. rosdep is supposed to cover these during the workspace
// build, but its failures are non-fatal there, so a missing one only surfaces
// as a launch exception much later.
const BRINGUP_PACKAGES = [
  "robot_localization",
  "imu_filter_madgwick",
  "robot_state_publisher",
  "joint_state_publisher",
  "xacro",
];
const SLAM_PACKAGES = ["slam_toolbox", "nav2_bringup"];
const NAV_PACKAGES = ["nav2_bringup"];

async function ensureRosPackages(pkgs, label) {
  if (isDockerMode()) return true;
  const distro = getDistro();
  const apt = [];      // available as ros-<distro>-<pkg>: one batched apt call
  const fromSource = []; // not published for this distro: build into the workspace
  for (const pkg of pkgs) {
    try {
      const res = await fetch(
        `/api/package/check?pkg=${encodeURIComponent(pkg)}` +
        `&distro=${encodeURIComponent(distro)}&ws=${encodeURIComponent(ws())}`
      );
      if (!res.ok) continue;
      const data = await res.json();
      if (data.installed) continue;
      if (data.source === "source" && data.install_cmd) fromSource.push(data);
      else apt.push(data.apt_package || `ros-${distro}-${pkg.replace(/_/g, "-")}`);
    } catch (err) {
      console.warn(`package check failed for ${pkg}:`, err);
    }
  }
  if (apt.length === 0 && fromSource.length === 0) return true;

  openTerminal(`Installing ${label} prerequisites`);
  logLine("[console] -------------------------------------------------------------");
  logLine(`[console] [1-Click] ${label} is missing: ${[...apt, ...fromSource.map((d) => d.package)].join(", ")}`);
  logLine("[console] -------------------------------------------------------------");

  let ok = true;
  if (apt.length) {
    ok = await new Promise((resolve) => {
      runCommand({ action: "apt_install", args: { packages: apt.join(" ") } }, {
        title: `Install ${label} prerequisites`,
        action: `install ${label} prerequisites`,
        onDone: (exitCode) => resolve(exitCode === 0),
      });
    });
    if (!ok) logLine(`[console] ⚠ apt install of ${label} prerequisites exited with an error.`);
  }
  for (const d of fromSource) {
    logLine(`[console] '${d.package}' has no binary package on ${distro} -- building it from source.`);
    const built = await new Promise((resolve) => {
      runCommand({ action: "apt_install", args: { packages: d.apt_package || `ros-${distro}-${(d.package||"").replace(/_/g, "-")}` } }, {
        title: `Build ${d.package} from source`,
        action: `build ${d.package}`,
        onDone: (exitCode) => resolve(exitCode === 0),
      });
    });
    if (!built) {
      ok = false;
      logLine(`[console] ⚠ Source build of '${d.package}' failed.`);
    }
  }
  // Trusting exit codes is not enough: apt can exit non-zero for one package
  // while the rest installed, and a source build can exit 0 having built
  // nothing ("Summary: 0 packages finished"). Ask again what is actually on
  // disk, and report the packages by name.
  const stillMissing = [];
  for (const pkg of pkgs) {
    try {
      const res = await fetch(
        `/api/package/check?pkg=${encodeURIComponent(pkg)}` +
        `&distro=${encodeURIComponent(distro)}&ws=${encodeURIComponent(ws())}`
      );
      if (!res.ok) continue;
      const data = await res.json();
      if (!data.installed) stillMissing.push(pkg);
    } catch (err) {
      console.warn(`re-check failed for ${pkg}:`, err);
    }
  }
  if (stillMissing.length) {
    logLine(`[console] ✖ ${label} prerequisites are still missing: ${stillMissing.join(", ")}`);
    return false;
  }
  return true;
}

// nav2_bringup only launches the rest of the stack, so having it says nothing
// about whether the nodes it starts exist. On Lyrical they are published one by
// one with no ros-lyrical-navigation2 metapackage to pull them in, and each
// missing one costs another failed launch ("package 'nav2_waypoint_follower'
// not found", then the next). Install the whole stack in one go instead.
async function ensureNav2Stack(label) {
  if (isDockerMode()) return true;
  let data = null;
  try {
    const res = await fetch(
      `/api/nav2/stack?distro=${encodeURIComponent(getDistro())}&ws=${encodeURIComponent(ws())}`
    );
    if (res.ok) data = await res.json();
  } catch (err) {
    console.warn("nav2 stack check failed:", err);
  }
  if (!data || data.installed || !data.command) return true;

  openTerminal(`Installing the nav2 stack for ${label}`);
  logLine("[console] -------------------------------------------------------------");
  logLine(`[console] [1-Click] nav2 runtime packages missing: ${data.missing.join(", ")}`);
  logLine("[console] Installing the published nav2 packages for this distro...");
  logLine("[console] -------------------------------------------------------------");
  const ok = await new Promise((resolve) => {
    runCommand({ action: "apt_install", args: { packages: (data.missing || []).join(" ") } }, {
      title: "Install nav2 stack",
      action: "install nav2 stack",
      onDone: (exitCode) => resolve(exitCode === 0),
    });
  });
  // Not fatal on its own and not a decision point: this is a bulk apt install
  // of whatever the distro publishes, and what matters is which packages end
  // up on disk, which ensureRosPackages re-checks by name afterwards.
  if (!ok) logLine("[console] ⚠ nav2 stack install exited with an error -- checking what actually installed...");
  return ok;
}

async function ensureNav2Prerequisites(targetTitle = "Navigation") {
  if (isDockerMode()) return true;
  const isSlam = (targetTitle || "").toLowerCase().includes("slam");
  const label = isSlam ? "SLAM" : "Nav2";
  await ensureNav2Stack(label);
  const ok = await ensureRosPackages(isSlam ? SLAM_PACKAGES : NAV_PACKAGES, label);
  if (!ok) {
    // Launching regardless produced a stack with no mapper in it and no error
    // to explain it: on Rolling/Ubuntu 26.04 the ROS index carries 2205
    // packages of which exactly three are nav2-* (all TurtleBot sim assets),
    // and neither slam_toolbox nor nav2_bringup exists at all. Say so and stop
    // rather than bringing up something that cannot work.
    logLine(`[console] ✖ Cannot start ${label}: the packages above are not published`);
    logLine(`[console]   for ${getDistro()} on this Ubuntu, and could not be built from source.`);
    logLine("[console]   This is a ROS index gap, not a fault in your setup --");
    logLine("[console]   a distro whose nav2/slam_toolbox packages are released will work.");
    throw new Error(`${label} prerequisites unavailable for ${getDistro()}`);
  }
  return ok;
}

function isBringupAlive() {
  return Boolean(state.status && (state.status.bringup_alive_external || state.status.bringup_busy_console));
}

// Returns an action SPEC (built server-side, actions.py), not a command string.
function bringupLaunchCommand() {
  if (isDockerMode()) {
    return { action: "bringup_docker", args: { engine: getAgentEngine() === "podman" ? "podman" : "docker",
                                               docker_dir: dockerDir() } };
  }
  const launcher = `${state.status?.web_dir || "."}/../launch_bringup.py`;
  const cfgPath = (state.robot_config && state.robot_config.path) || "~/.config/linorobot2/robot_config.yaml";
  const base = document.getElementById("bringup-base-type")?.value || (state.config && state.config.base_type) || "2wd";
  const dev = document.getElementById("bringup-agent-device")?.value || (state.config && state.config.agent_device) || "/dev/ttyACM0";
  const baud = document.getElementById("bringup-agent-baud")?.value || (state.config && state.config.agent_baud) || "1500000";
  const madgwick = document.getElementById("bringup-madgwick-toggle")?.checked ? "true" : "false";

  // Bringup would otherwise start its own native micro_ros_agent. Skip that
  // when an agent is already up, or when the agent engine is a container --
  // in container mode the package is usually not installed natively at all, so
  // including it fails the whole launch while the real agent runs happily.
  const nativeAgent = getAgentEngine() === "native";
  const microRos = (nativeAgent && !isAgentAlive()) ? "true" : "false";

  return { action: "bringup", args: {
    launcher, config_path: cfgPath, base, device: dev, baud,
    madgwick, micro_ros: microRos, distro: getDistro(),
  } };
}

async function ensureBringupRunning(targetTitle = "requested action") {
  if (!isAutoBringupEnabled()) return;
  await refreshStatus();

  // 1-Click Intermediate Step 0: ROS 2 itself
  if (!isDockerMode() && state.status && state.status.ros2_installed === false) {
    logLine(`[console] [1-Click] Step 0: ROS 2 ${getDistro()} not installed -- installing it first...`);
    const rosOk = await ensureRos2Installed();
    if (!rosOk) {
      logLine("[console] \u2716 [1-Click] ROS 2 install failed. Aborting " + targetTitle + ".");
      throw new Error("ROS 2 install failed");
    }
  }

  // 1-Click Intermediate Step 1: Ensure micro-ROS agent is active
  if (!isAgentAlive()) {
    logLine("[console] [1-Click] Step 1: micro-ROS Agent is down -- auto-starting agent...");
    await ensureAgentRunning();
  }

  // 1-Click Intermediate Step 2: Auto-build base workspace if not built yet
  if (!isDockerMode() && state.status && state.status.workspace_built === false) {
    logLine("[console] [1-Click] Step 2: Workspace not built -- auto-building linorobot2 base...");
    const buildOk = await checkAndBuildWorkspace();
    if (!buildOk) {
      logLine("[console] ✖ [1-Click] Workspace build failed. Aborting " + targetTitle + ".");
      throw new Error("Workspace build failed");
    }
  }

  // 1-Click Intermediate Step 3: Auto-detect and auto-install missing LiDAR driver
  const laser = document.getElementById("bringup-laser-sensor")?.value || (state.config && state.config.laser_sensor);
  if (laser && !isDockerMode()) {
    logLine(`[console] [1-Click] Step 3: Checking LiDAR driver for '${laser}'...`);
    const driverOk = await checkAndInstallLidarDriver(laser);
    if (!driverOk) {
      logLine(`[console] ⚠ LiDAR driver setup failed for '${laser}'. Continuing bringup...`);
    }
  }

  // 1-Click Intermediate Step 3b: packages linorobot2_bringup itself launches
  if (!isDockerMode()) {
    await ensureRosPackages(BRINGUP_PACKAGES, "Bringup");
  }

  // 1-Click Intermediate Step 3c: the LiDAR driver itself.
  // bringup does not start it -- it is a separate node in its own slot -- so
  // after a 1-Click SLAM the whole stack came up healthy with no /scan at all,
  // and slam_toolbox sat there producing nothing with no error to explain it.
  if (laser && !isDockerMode() && !isLaserRunning()) {
    logLine("[console] [1-Click] Step 3c: starting the LiDAR driver...");
    await startLaserDriver();
  }

  // 1-Click Intermediate Step 4: Check Nav2 / SLAM packages if launching Nav2/SLAM
  const lowerTitle = (targetTitle || "").toLowerCase();
  if (lowerTitle.includes("nav") || lowerTitle.includes("slam")) {
    await ensureNav2Prerequisites(targetTitle);
  }

  // If Bringup is already running, we are ready -- but only if it is actually
  // publishing. A live process is not a live robot: an orphaned `ros2 launch`
  // wrapper whose children have died still matches BRINGUP_PROC_PATTERN, so
  // bringup_alive_external stays true while nothing publishes. Console then
  // skipped bringup and started SLAM with no odometry and no odom->base_link
  // TF, and nav2 failed 60s later with
  //   "Failed to activate local_costmap because transform from base_link to
  //    odom did not become available before timeout"
  // which says nothing about bringup at all. server.py already probes the
  // graph for exactly this (/api/bringup/health, ready = odom + TF chain), so
  // ask it before trusting the process.
  if (isBringupAlive()) {
    let health = null;
    try {
      health = await fetch("/api/bringup/health?timeout=4").then((r) => r.json());
    } catch (e) {
      health = null;
    }
    if (health && health.ready) {
      logLine("[console] ✓ Robot Bringup is already active. Ready for " + targetTitle + ".");
      return;
    }
    const why = health && health.summary ? health.summary : "health probe failed";
    logLine(`[console] ⚠ A bringup process is running but the robot is not publishing: ${why}`);
    logLine("[console] Restarting bringup rather than launching " + targetTitle + " blind...");
    try {
      await killSlot("bringup");
      await new Promise((r) => setTimeout(r, 3000));
      await refreshStatus();
    } catch (e) {
      logLine(`[console] ⚠ Could not stop the stale bringup: ${e}`);
    }
  }

  // 1-Click Intermediate Step 5: Start Bringup in background and stream logs
  logLine(`[console] [1-Click] Step 5: Automatically launching Robot Bringup in background for ${targetTitle}...`);
  openTerminal("Robot Bringup (1-Click Auto-Started) [streaming]");
  attachBringupStream();

  const cmd = bringupLaunchCommand();
  return new Promise((resolve) => {
    const startBtn = document.getElementById("btn-bringup-start");
    const stopBtn = document.getElementById("btn-bringup-stop");
    if (startBtn) startBtn.disabled = true;
    if (stopBtn) stopBtn.disabled = false;

    runCommand(cmd, {
      slot: "bringup",
      title: `Robot Bringup (${targetTitle})`,
      onDone: (exitCode) => {
        if (startBtn) startBtn.disabled = false;
        if (stopBtn) stopBtn.disabled = true;
        if (state.status) state.status.bringup_busy_console = false;
        refreshStatus();
        if (exitCode !== 0) {
          openTerminal("Robot Bringup [Exited with error]");
          logLine(`[console] ✖ Bringup process exited with error code ${exitCode}. Check output above.`);
        }
      },
    });

    if (state.status) state.status.bringup_busy_console = true;
    const pill = document.getElementById("hdr-bringup-pill");
    if (pill) {
      pill.textContent = "starting...";
      pill.className = "pill pill-starting";
    }

    const startedAt = Date.now();
    const DEADLINE_MS = 40000;
    const poll = async () => {
      let alive = false;
      try {
        const s = await fetch("/api/status").then((r) => r.json());
        state.status = s;
        alive = Boolean(s.bringup_alive_external || s.bringup_busy_console);
      } catch (e) {
        /* keep polling */
      }
      const waited = Date.now() - startedAt;
      if (alive || waited >= DEADLINE_MS) {
        refreshStatus();
        logLine(
          alive
            ? `[console] ✓ Robot Bringup confirmed active after ${(waited / 1000).toFixed(1)}s. Launching ${targetTitle}...`
            : `[console] ⚠ Bringup not confirmed after ${(DEADLINE_MS / 1000)}s -- proceeding with ${targetTitle} anyway. Check logs above.`
        );
        // Settle delay so nodes/agent/TF tree bind before caller launches
        setTimeout(resolve, 2500);
        return;
      }
      setTimeout(poll, 1000);
    };
    setTimeout(poll, 1200);
  });
}


// ---------- ROS 2 auto-install ----------
// Everything else in the 1-Click chain assumes /opt/ros/<distro> exists. On a
// fresh machine it does not, and the chain went straight to `colcon build`,
// which failed with "colcon: command not found" -- a message that says nothing
// about the real problem. Install ROS 2 first, then continue.
async function ensureRos2Installed() {
  if (isDockerMode()) return true;
  await refreshStatus();
  if (state.status && state.status.ros2_installed) return true;

  const distro = getDistro();
  let cmd = null;
  try {
    const res = await fetch(`/api/ros2/install_cmd?distro=${encodeURIComponent(distro)}`);
    if (res.ok) {
      const data = await res.json();
      if (data.installed) return true;
      cmd = data.handle;
    }
  } catch (err) {
    console.warn("ROS 2 install command lookup failed:", err);
  }
  if (!cmd) {
    logLine(`[console] ✖ Could not work out how to install ROS 2 ${distro}.`);
    return false;
  }

  openTerminal(`Installing ROS 2 ${distro}`);
  logLine("[console] -------------------------------------------------------------");
  logLine(`[console] ⚠ ROS 2 ${distro} is not installed (/opt/ros/${distro}/setup.bash missing).`);
  logLine("[console] Installing it now -- this takes several minutes on a fresh machine.");
  logLine("[console] -------------------------------------------------------------");
  const ok = await new Promise((resolve) => {
    runCommand({ handle: cmd }, {
      title: `Install ROS 2 ${distro}`,
      onDone: (exitCode) => resolve(exitCode === 0),
    });
  });
  await refreshStatus();
  if (!ok) logLine(`[console] ✖ ROS 2 ${distro} install failed. See the output above.`);
  return ok;
}

// ---------- workspace auto-build & lidar driver auto-install ----------
async function checkAndBuildWorkspace() {
  if (isDockerMode()) return true;
  await refreshStatus();
  if (state.status && state.status.workspace_built) {
    return true;
  }
  // colcon and the compiler come from the ROS 2 install, so a bare machine has
  // to get that first. "Run base install" used to go straight to the clone and
  // then die on "colcon: command not found" (exit 127) -- a message that names
  // the missing tool but not the missing distro, so it reads like a broken box.
  // The 1-Click chain already guards this way at its Step 0; the Install tab
  // button reached the same command without it.
  if (state.status && state.status.ros2_installed === false) {
    logLine(`[console] Base install needs ROS 2 ${getDistro()} for colcon -- installing it first...`);
    if (!(await ensureRos2Installed())) {
      logLine("[console] ✖ Cannot run base install: ROS 2 is not installed.");
      return false;
    }
  }

  const workspace = ws();
  openTerminal("Base Install & Workspace Build");
  logLine("[console] -------------------------------------------------------------");
  logLine(`[console] ⚠ Native workspace is not built yet (${workspace}/install/setup.bash missing).`);
  logLine("[console] linorobot2_bringup and base packages must be built before bringup can run.");
  logLine("[console] Automatically running Base Install & colcon build now...");
  logLine("[console] -------------------------------------------------------------");

  let handle = null;
  try {
    const res = await fetch(`/api/workspace/build_cmd?ws=${encodeURIComponent(workspace)}&distro=${encodeURIComponent(getDistro())}`);
    if (res.ok) handle = (await res.json()).handle;
  } catch (_) {}

  if (!handle) {
    logLine("[console] ✖ Base install: the server did not return a build command.");
    return false;
  }

  const success = await new Promise((resolve) => {
    runCommand({ handle }, {
      title: "Base Install & colcon build",
      onDone: (exitCode) => {
        if (exitCode === 0) {
          logLine("[console] ✓ Base workspace installed and built successfully!");
          resolve(true);
        } else {
          logLine(`[console] ✖ Base install failed with exit code ${exitCode}. Check output above.`);
          resolve(false);
        }
      }
    });
  });

  if (success) {
    await refreshStatus();
  }
  return success;
}

// ---------- bringup log streaming & lidar driver auto-install ----------
let bringupEventSource = null;

function attachBringupStream() {
  if (bringupEventSource) return;
  openTerminal("Robot Bringup [streaming]");
  try {
    bringupEventSource = new EventSource("/api/bringup/stream");
    bringupEventSource.addEventListener("init", (e) => {
      try {
        const data = JSON.parse(e.data);
        if (data.status === "running") {
          openTerminal(`Robot Bringup [${data.source || 'running'}]`);
        }
      } catch (_) {}
    });
    bringupEventSource.addEventListener("output", (e) => {
      try {
        const data = JSON.parse(e.data);
        if (data.line != null) {
          logLine(`[bringup] ${data.line}`);
        }
      } catch (_) {}
    });
    bringupEventSource.addEventListener("done", (e) => {
      try {
        const data = JSON.parse(e.data);
        logLine(`[bringup] exited with code ${data.exit_code}`);
      } catch (_) {}
      if (bringupEventSource) {
        bringupEventSource.close();
        bringupEventSource = null;
      }
      refreshStatus();
    });
    bringupEventSource.addEventListener("idle", () => {
      if (bringupEventSource) {
        bringupEventSource.close();
        bringupEventSource = null;
      }
    });
    bringupEventSource.onerror = () => {
      if (bringupEventSource) {
        bringupEventSource.close();
        bringupEventSource = null;
      }
    };
  } catch (err) {
    console.error("Error attaching bringup stream:", err);
  }
}

async function checkAndInstallLidarDriver(laserSensor) {
  if (!laserSensor || isDockerMode()) return true;
  try {
    const res = await fetch(`/api/sensors/driver_status?sensor=${encodeURIComponent(laserSensor)}&ws=${encodeURIComponent(ws())}`);
    if (!res.ok) return true;
    const info = await res.json();
    if (!info.installed && info.pkg) {
      openTerminal(`Installing LiDAR Driver (${info.pkg})`);
      logLine(`[console] -------------------------------------------------------------`);
      logLine(`[console] LiDAR '${laserSensor}' requires ROS 2 package '${info.pkg}'.`);
      logLine(`[console] Driver package was not found in ROS 2 or workspace ${ws()}.`);
      logLine(`[console] Automatically installing and building driver before bringup...`);
      logLine(`[console] -------------------------------------------------------------`);
      
      const success = await new Promise((resolve) => {
        runCommand({ action: "apt_install", args: {
          packages: `ros-${getDistro()}-${info.pkg.replace(/_/g, "-")}`,
        } }, {
          title: `Install LiDAR Driver: ${info.pkg}`,
          onDone: (exitCode) => {
            if (exitCode === 0) {
              logLine(`[console] ✓ LiDAR driver '${info.pkg}' installed successfully!`);
              resolve(true);
            } else {
              logLine(`[console] ⚠ LiDAR driver install returned exit code ${exitCode}.`);
              resolve(false);
            }
          }
        });
      });
      return success;
    }
  } catch (e) {
    console.warn("Driver pre-flight check failed:", e);
  }
  return true;
}

// ---------- bringup ----------
const btnBringupLogs = document.getElementById("btn-bringup-logs");
if (btnBringupLogs) {
  btnBringupLogs.addEventListener("click", () => attachBringupStream());
}

wireStartStop({
  startBtn: document.getElementById("btn-bringup-start"),
  stopBtn: document.getElementById("btn-bringup-stop"),
  stackTag: "bringup",
  slot: "bringup",
  title: "Bringup",
  buildCommand: async () => {
    if (!isDockerMode() && state.status && state.status.ros2_installed === false) {
      const rosOk = await ensureRos2Installed();
      if (!rosOk) {
        logLine("[console] ✖ Cannot start Bringup: ROS 2 is not installed.");
        throw new Error("ROS 2 install failed");
      }
    }
    if (!isDockerMode() && state.status && state.status.workspace_built === false) {
      const buildOk = await checkAndBuildWorkspace();
      if (!buildOk) {
        logLine("[console] ✖ Cannot start Bringup: workspace build did not succeed.");
        throw new Error("Workspace build failed");
      }
    }
    const laser = document.getElementById("bringup-laser-sensor")?.value || (state.config && state.config.laser_sensor);
    if (laser && !isDockerMode()) {
      await checkAndInstallLidarDriver(laser);
    }
    openTerminal("Robot Bringup [streaming]");
    attachBringupStream();
    return bringupLaunchCommand();
  },
});

// ---------- bringup health (topic + TF level, not just pgrep) ----------
// `bringup_alive_external` in /api/status only says a process exists. This asks
// the ROS graph whether odometry, the IMU, the LiDAR and the TF chain are
// actually live -- the thing SLAM/Nav2 will silently fail on otherwise.
const btnBringupHealth = document.getElementById("btn-bringup-health");
if (btnBringupHealth) {
  btnBringupHealth.addEventListener("click", async () => {
    const summaryEl = document.getElementById("bringup-health-summary");
    const tableEl = document.getElementById("bringup-health-table");
    btnBringupHealth.disabled = true;
    summaryEl.textContent = "Probing the ROS graph (up to ~30 s)…";
    tableEl.innerHTML = "";
    try {
      const h = await fetch("/api/bringup/health?timeout=4").then((r) => r.json());
      const mark = (ok) => (ok ? "🟢" : "🔴");
      const rows = Object.values(h.topics || {}).map((t) => `
        <tr>
          <td>${mark(t.ok)}</td>
          <td><code>${escapeHtml(t.topic)}</code></td>
          <td>${escapeHtml(t.what)}</td>
          <td>${t.hz == null ? (t.advertised ? "no messages" : "not advertised")
                             : t.hz.toFixed(1) + " Hz"}</td>
          <td>&ge; ${t.min_hz} Hz</td>
        </tr>`).join("");
      const tfRows = (h.tf || []).map((l) => `
        <tr>
          <td>${mark(l.ok)}</td>
          <td colspan="2"><code>TF ${escapeHtml(l.parent)} &rarr; ${escapeHtml(l.child)}</code></td>
          <td colspan="2">${escapeHtml(l.ok ? "transform resolves" : l.detail)}</td>
        </tr>`).join("");
      tableEl.innerHTML =
        `<table class="health-table"><tbody>${rows}${tfRows}</tbody></table>`;
      summaryEl.innerHTML =
        `<span style="color: var(--${h.ready ? "accent-ok" : "accent-danger"});">` +
        `${h.ready ? "✓" : "✗"} ${escapeHtml(h.summary)}</span>`;
      logLine(`[console] bringup health: ${h.summary}`);
    } catch (e) {
      summaryEl.textContent = `health check failed: ${e}`;
    } finally {
      btnBringupHealth.disabled = false;
    }
  });
}

// ---------- teleop ----------
wireStartStop({
  startBtn: document.getElementById("btn-teleop-start"),
  stopBtn: document.getElementById("btn-teleop-stop"),
  slot: "main",
  title: "Gamepad teleop",
  needsBringup: true,
  buildCommand: async () => ({ action: "teleop", args: {
    axis_linear: document.getElementById("joy-axis-linear").value || 1,
    scale_linear: document.getElementById("joy-scale-linear").value || 0.5,
    axis_angular: document.getElementById("joy-axis-angular").value || 0,
    scale_angular: document.getElementById("joy-scale-angular").value || 1.0,
    distro: getDistro(),
  } }),
});

