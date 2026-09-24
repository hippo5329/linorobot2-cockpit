#!/usr/bin/env python3
# ==============================================================================
# drivetrain_report.py — what a robot's motors can actually deliver
#
# Enter a robot config; get the speed and acceleration it can reach, and whether
# the Nav2 limits in that same config are asking for more than that.
#
# This exists because nothing checked. The shipped limits asked 103% of the
# motors on a differential base and 171% at the velocity smoother's ceiling, and
# on mecanum -- which turns on (lr + fr)/2 rather than lr/2, so the same angular
# velocity costs it 66% more wheel speed -- 129%. Nav2 then commands what it
# cannot get, Kinematics scales the whole request down to fit, and tracking
# degrades: three mecanum legs left the room on 2026-09-23 before anyone looked
# at the motors. The arithmetic is not hard; it was simply never written down.
#
# The model is the firmware's own (firmware/common/lib/encoder/sim_wheel.h): a
# brushed DC gear motor whose torque falls linearly from stall to no-load, a
# gearbox that returns part of it, constant gear drag, viscous friction, and a
# pack that sags under the current all the wheels draw together.
#
# Its constants are PARSED from that header rather than copied here. A tool that
# restates the model's numbers drifts from it silently, and then reports a robot
# that does not exist -- which is the same class of fault as the table of sensor
# support that had to be checked against the drivers.
#
#   python3 scripts/drivetrain_report.py --params <robot>_config.yaml
# ==============================================================================
import argparse
import datetime
import math
import os
import re
import sys

try:
    import yaml
except ImportError:
    print("PyYAML is required: sudo apt install -y python3-yaml", file=sys.stderr)
    sys.exit(1)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM_WHEEL_H = os.path.join(REPO_ROOT, "firmware", "common", "lib", "encoder",
                            "sim_wheel.h")


def model_defaults(path=SIM_WHEEL_H):
    """The wheel model's constants, read from the firmware that implements it."""
    want = {
        "SIM_WHEEL_TAU_MS": "tau_ms",
        "SIM_WHEEL_REF_MASS": "ref_mass",
        "SIM_WHEEL_MAX_ACCEL_RPM": "accel_clamp",
        "SIM_WHEEL_FRICTION": "viscous",
        "SIM_WHEEL_STALL_DUTY": "stall_duty",
        "SIM_ROBOT_MASS": "mass",
        "SIM_GEAR_EFFICIENCY": "gear_eff",
        "SIM_WHEEL_COULOMB_RPM": "coulomb",
        "SIM_BATT_SAG": "sag",
        "SIM_BATT_SAG_TAU_MS": "sag_tau_ms",
        "SIM_DRV_DROP": "drv_drop",
        "SIM_DRV_R": "drv_r",
        "SIM_MOTOR_STALL_A": "stall_a",
        "SIM_DRV_ILIMIT_A": "ilimit_a",
        # The simulated SENSORS. Not used by this report, which is noiseless on
        # purpose, but sim_base_node has to imitate the board's output and not
        # just its motion -- a perfect IMU is a different robot to fuse.
        "SIM_WHEEL_NOISE_RPM": "noise_rpm",
        "SIM_IMU_GYRO_BIAS": "gyro_bias",
        "SIM_IMU_GYRO_DRIFT": "gyro_drift",
        "SIM_IMU_GYRO_NOISE": "gyro_noise",
        "SIM_IMU_ACCEL_NOISE": "accel_noise",
        "SIM_IMU_SCALE_ERROR": "scale_error",
    }
    out = {}
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            m = re.match(r"\s*#define\s+(\w+)\s+([0-9.]+)", line)
            if m and m.group(1) in want:
                out.setdefault(want[m.group(1)], float(m.group(2)))
    missing = set(want.values()) - set(out)
    if missing:
        raise SystemExit(f"{path}: cannot find {sorted(missing)} -- the model moved, "
                         f"and this report would be describing a robot that does not exist")
    return out


def _dig(node, key, depth=0):
    """Nav2 params are nested <node>: <node>: ros__parameters: ..., and which
    level a key sits at differs between distros (the RotationShim nests
    FollowPath on some). Search rather than assume a path."""
    if depth > 7 or not isinstance(node, dict):
        return None
    if key in node:
        return node[key]
    for v in node.values():
        got = _dig(v, key, depth + 1)
        if got is not None:
            return got
    return None


def drivetrain(params):
    """Everything the report needs, resolved from the config."""
    kine = params.get("kinematics", {}) or {}
    bc = params.get("base_controller", {}) or {}
    sim = bc.get("simulation", {}) or {}
    d = model_defaults()

    # The env overrides the compiled-in defaults, so the config's simulation
    # block is what a real run would use.
    for cfg_key, name in (("robot_mass", "mass"), ("gear_efficiency", "gear_eff"),
                          ("gear_drag_rpm", "coulomb"), ("battery_sag", "sag"),
                          ("battery_sag_tau_ms", "sag_tau_ms"),
                          ("driver_drop", "drv_drop"), ("driver_resistance", "drv_r"),
                          ("motor_stall_amps", "stall_a"),
                          ("driver_current_limit", "ilimit_a")):
        if sim.get(cfg_key) is not None:
            d[name] = float(sim[cfg_key])

    base = str(kine.get("base_type", "2wd")).lower()
    lr = float(kine.get("lr_wheels_distance", 0.0))
    fr = float(kine.get("fr_wheels_distance", 0.0))
    wheel_d = float(kine.get("wheel_diameter", 0.0))
    # The same radius Kinematics::rotationRadius() uses, and for the same reason:
    # a mecanum's rollers put the wheelbase into the yaw term.
    if base == "mecanum":
        radius = (lr + fr) / 2.0
    elif base in ("4wd", "skid_steer"):
        radius = (lr / 2.0) * float(kine.get("angular_scale", 1.0))
    else:
        radius = lr / 2.0
    # TWO different ceilings, and conflating them was a real mistake in the
    # first version of this report.
    #
    #   motor_rpm    the motor's own no-load speed, derated only by the voltage
    #                it is fed. What the machine CAN do. sim_wheel.h uses this.
    #   command_rpm  that, times max_rpm_ratio -- the DRIVER MARGIN, which is
    #                the share of no-load speed the losses take back before the
    #                wheel ever turns at the rated figure. What the controller
    #                WILL ask for, and therefore what a demand must fit inside.
    #                It used to be a hand-picked 0.85; suggest_max_rpm_ratio()
    #                measures it off the model instead, so it follows the load.
    #
    # The report said "max speed 0.60 m/s" using the command cap while the bench
    # measured 0.71 m/s, because test_acc drives raw PWM and bypasses Kinematics
    # entirely (135.6 rpm at 3.5 kg, 2026-09-23). Capability and budget are not
    # the same number.
    op_v = float(kine.get("motor_operating_voltage", 0) or 0)
    pw_v = float(kine.get("motor_power_max_voltage", 0) or 0)
    volt_ratio = min(pw_v / op_v, 1.0) if op_v > 0 and pw_v > 0 else 1.0
    motor_rpm = float(kine.get("max_rpm", 0)) * volt_ratio
    # The loop's output range, because a plant gain in "rpm per PWM count" needs
    # to know how many counts a full step is: 10-bit and 12-bit boards would
    # otherwise be handed gains differing by a factor of four.
    pwm_bits = int(kine.get("pwm_bits", 10) or 10)
    d["pwm_max"] = float((1 << pwm_bits) - 1)
    d.update(base=base, radius=radius, circ=math.pi * wheel_d,
             volt_ratio=volt_ratio, motor_rpm=motor_rpm,
             command_rpm=motor_rpm * float(kine.get("max_rpm_ratio", 1.0)),
             max_rpm=motor_rpm,
             wheels=4 if base in ("4wd", "skid_steer", "mecanum") else 2)
    return d


def performance(d):
    """Achievable speed and acceleration, from the model.

    tau scales with mass: a heavier robot takes longer to reach the same speed
    through the same gearbox. Sag is applied at the demand the manoeuvre makes --
    full stall current from rest, almost none at terminal speed -- so the two
    numbers are not computed under the same voltage, because the robot is not.
    """
    tau = (d["tau_ms"] / 1000.0) * (d["mass"] / d["ref_mass"])
    tau = max(tau, 0.001)

    # The pack's sag LAGS (SIM_BATT_SAG_TAU_MS), so the first instant of an
    # acceleration sees a stiff pack and the sag develops underneath it. That
    # gives two different accelerations and the difference is the lunge a real
    # robot has: bridge losses bite immediately, the pack gives way after.
    drv_now = (1.0 - d["drv_drop"]) / (1.0 + d["drv_r"])          # stall current, no sag yet
    drv_held = (1.0 - d["drv_drop"]) / (1.0 + d["sag"] + d["drv_r"])   # sag fully developed

    # From rest the speed error is the whole no-load speed, so the current
    # demand is at stall -- which is exactly where a driver's limiter bites.
    ilim = 1.0
    if d["ilimit_a"] > 0.0 and d["stall_a"] > 0.0:
        ilim = min(d["ilimit_a"] / d["stall_a"], 1.0)

    def _accel(scale):
        a = ilim * d["gear_eff"] * (d["motor_rpm"] * scale) / tau - d["coulomb"]
        return max(min(a, d["accel_clamp"]), 0.0) * d["circ"] / 60.0

    lin_acc = _accel(drv_now)          # peak, in the first instant
    lin_acc_held = _accel(drv_held)    # once the pack has sagged

    # Terminal speed: accel = 0, and at that point the current -- and so the sag
    # -- has almost gone, so the no-load speed is back to full.
    #   eff*(no_load - w)/tau = w*viscous + coulomb
    # At terminal speed the current has almost gone, so the pack recovers and
    # only the bridge's fixed drop remains.
    no_load_terminal = d["motor_rpm"] * (1.0 - d["drv_drop"])
    w = (d["gear_eff"] * no_load_terminal / tau - d["coulomb"]) / \
        (d["gear_eff"] / tau + d["viscous"])
    w = max(w, 0.0)
    lin_vel = w * d["circ"] / 60.0

    return {
        "tau": tau,
        "lin_vel": lin_vel,
        "lin_acc": lin_acc,
        "lin_acc_held": lin_acc_held,
        "ang_acc_held": lin_acc_held / d["radius"] if d["radius"] > 0 else 0.0,
        "ang_vel": lin_vel / d["radius"] if d["radius"] > 0 else 0.0,
        "ang_acc": lin_acc / d["radius"] if d["radius"] > 0 else 0.0,
        "t_to_90": 2.3 * tau,
        "stop_dist": lin_vel * tau / d["gear_eff"] if d["gear_eff"] > 0 else 0.0,
        "wheel_rpm_terminal": w,
    }


# ==============================================================================
# The simulated test_acc.
#
# performance() above solves the model in closed form. That is the right shape
# for "what can this chassis do", but it is NOT what test_acc reports, and the
# two differ in ways that matter: test_acc samples at 20 ms and differentiates
# the samples, so it sees a first-difference acceleration rather than the
# instantaneous one; it drives raw PWM in 1 s phases, so the pack's sag has a
# specific amount of time to develop; and it measures the stop distance by
# integrating the coast, which no closed form gives.
#
# So the numbers are produced the way the bench produces them: by running the
# model. This class is a transcription of SimEncoder::integrate() and
# busScale() -- same terms, same order, same clamps, same shared pack -- and
# run_test_acc() is a transcription of test_acc.cpp's loop_() and dump_record().
# Given that, flashing test_acc and driving a board is no longer how these
# numbers are obtained. It is how this transcription is CHECKED, which is a
# different job and a much rarer one.
#
# Deliberately noiseless. getRPM() adds +/-SIM_WHEEL_NOISE_RPM, and test_acc
# differentiates it: 1 rpm of white noise across a 20 ms tick is about 0.4 m/s2
# of pure instrument error in the MAX ACC column, which is why the bench figure
# reads slightly high and why a report should not reproduce it.
# ==============================================================================
class _Pack:
    """The one battery all four wheels share, with its lagging sag."""

    def __init__(self, d):
        self.d = d
        self.demand = [0.0, 0.0, 0.0, 0.0]
        # Starts UNSAGGED, because on the board it does.
        #
        # busScale() seeds sagState() from the present demand on its very first
        # call (sagClock() == 0), rather than stepping with a bogus dt. On a real
        # board that first call happens at boot with the wheels at rest and duty
        # zero, so it seeds to 0 and the seeding never matters again. Starting
        # this simulation at the first tick of test_acc's drive phase instead
        # seeded it at FULL stall demand -- the pack fully sagged before the
        # wheel had turned once, which reversed the whole point of the lag: a
        # long tau then held that worst case for longer and a "stiffer" pack
        # looked quicker. Seeding at rest is the faithful transcription.
        self.state = 0.0

    def scale(self, dt):
        inst = min(max(sum(self.demand) * 0.25, 0.0), 1.0)
        if dt > 0:
            tau_s = self.d["sag_tau_ms"] / 1000.0
            k = 1.0 if tau_s <= 0 else min(dt / tau_s, 1.0)
            self.state += (inst - self.state) * k
        scale = (1.0 - self.d["drv_drop"]) / \
                (1.0 + self.d["sag"] * self.state + self.d["drv_r"] * inst)
        return max(scale, 0.0)


class _Wheel:
    """One simulated wheel. Mirrors SimEncoder; see sim_wheel.h."""

    def __init__(self, d, pack, slot):
        self.d, self.pack, self.slot = d, pack, slot
        self.rpm = 0.0
        self.duty = 0.0
        self.tau = max((d["tau_ms"] / 1000.0) * (d["mass"] / d["ref_mass"]), 0.001)

    def step(self, dt, first_in_tick):
        d = self.d
        no_load = self.duty * d["motor_rpm"]      # motor_rpm already carries volt_ratio
        if abs(self.duty) < d["stall_duty"]:
            no_load = 0.0
        # Recorded BEFORE the bus scale, or the sag feeds back on itself.
        stall = d["motor_rpm"]
        ifrac = abs(no_load - self.rpm) / stall if stall > 0 else 0.0

        # The driver's current limiter caps TORQUE, not speed, so it scales the
        # driving term rather than the speed the motor aims for. It also means
        # the pack sees less current than the motor asked for.
        ilim_scale = 1.0
        if d["ilimit_a"] > 0.0:
            want_a = ifrac * d["stall_a"]
            if want_a > d["ilimit_a"]:
                ilim_scale = d["ilimit_a"] / want_a

        self.pack.demand[self.slot] = ifrac * ilim_scale
        # Only the first wheel of a tick advances the pack's filter: in the
        # firmware the other three call busScale() in the same microsecond and
        # its dt is zero, so four wheels advance it by one cycle in total.
        no_load *= self.pack.scale(dt if first_in_tick else 0.0)

        accel = ilim_scale * d["gear_eff"] * (no_load - self.rpm) / self.tau
        accel -= self.rpm * d["viscous"]
        if self.rpm > 0.0:
            accel -= d["coulomb"]
        elif self.rpm < 0.0:
            accel += d["coulomb"]
        accel = min(max(accel, -d["accel_clamp"]), d["accel_clamp"])

        was = self.rpm
        self.rpm += accel * dt
        # Coulomb drag brakes, it must not become a motor.
        if no_load == 0.0 and was != 0.0 and (was > 0.0) != (self.rpm > 0.0):
            self.rpm = 0.0
        if no_load == 0.0 and abs(self.rpm) < 0.5:
            self.rpm = 0.0


def _velocities(d, rpm):
    """Kinematics::getVelocities(), for the base this config declares."""
    r1, r2, r3, r4 = rpm
    if d["base"] not in ("4wd", "skid_steer", "mecanum"):
        r3 = r4 = 0.0
    n = float(d["wheels"])
    lin_x = ((r1 + r2 + r3 + r4) / n / 60.0) * d["circ"]
    ang_z = ((-r1 + r2 - r3 + r4) / n / 60.0) * d["circ"] / d["radius"] if d["radius"] > 0 else 0.0
    return lin_x, ang_z


# test_acc's own timing: `const unsigned ticks = 20`, `run_time = 1000`, and four
# phases per run (drive, coast, reverse, coast) -> buf_size = 200 samples.
TICK_S = 0.020
PHASE_SAMPLES = 50


def run_test_acc(d, rotate=False):
    """What test_acc would print for this config, without flashing anything.

    `rotate` picks the run parity: test_acc alternates a straight run (all four
    wheels driven the same way) with a rotation (wheels 1 and 3 reversed), and
    reports the linear columns for one and the angular for the other.
    """
    pack = _Pack(d)
    wheels = [_Wheel(d, pack, i) for i in range(4)]
    # Full PWM. test_acc halves it every fourth run; the first run, which is the
    # one quoted, is at the top of the range.
    #
    # Only the DRIVEN wheels are driven. On a differential base getRPM() for
    # motors 3 and 4 feeds nothing -- Kinematics asks them for ~0 rpm and the PID
    # holds them there -- so they draw no current and do not load the shared
    # pack. Driving all four here made a 2wd base sag like a 4wd one and come out
    # 4% slower than it is; the Nav2 limits derived from that were sized for a
    # robot with twice the load on its battery.
    n_driven = int(d.get("wheels", 4))
    duty = [(-1.0 if rotate and i in (0, 2) else 1.0) if i < n_driven else 0.0
            for i in range(4)]

    trace = []
    for phase, sign in enumerate((1.0, 0.0, -1.0, 0.0)):
        for i, w in enumerate(wheels):
            w.duty = duty[i] * sign
        for _ in range(PHASE_SAMPLES):
            # SAMPLE FIRST, then advance. record() reads getRPM() and only then
            # delay()s, so the first sample of a phase is taken with no elapsed
            # time -- it still shows the previous phase's final speed -- and
            # sample i shows i ticks of the new duty. Stepping first put every
            # number one tick early, which is a 20 ms error in "time to 0.9x"
            # and reads as a robot that accelerates faster than it does.
            trace.append(_velocities(d, [w.rpm for w in wheels]))
            # feed() integrates once and record()'s four getRPM() calls
            # integrate again, but only the first call after the delay sees a
            # non-zero dt -- so one step per wheel per tick is the whole of it.
            for i, w in enumerate(wheels):
                w.step(TICK_S, first_in_tick=(i == 0))

    lin = [t[0] for t in trace]
    ang = [t[1] for t in trace]
    series = ang if rotate else lin

    def _extremes(v):
        acc = [(v[i] - v[i - 1 if i else 0]) / TICK_S for i in range(len(v))]
        return max(max(v), 0.0), min(min(v), 0.0), max(max(acc), 0.0), min(min(acc), 0.0)

    max_v, min_v, max_a, min_a = _extremes(series)
    # dump_record() integrates the SECOND quarter -- the coast after the first
    # drive phase -- which is the distance the robot takes to stop.
    stop = sum(series[PHASE_SAMPLES:2 * PHASE_SAMPLES]) * TICK_S
    # ...and reports the first sample of the first quarter that passed 0.9x max.
    t_to_90 = PHASE_SAMPLES * TICK_S
    for i in range(PHASE_SAMPLES):
        if series[i] > max_v * 0.9:
            t_to_90 = i * TICK_S
            break

    return {"max_vel": max_v, "min_vel": min_v, "max_acc": max_a, "min_acc": min_a,
            "t_to_90": t_to_90, "stop": stop,
            "max_vel_lin": max(max(lin), 0.0), "max_vel_ang": max(max(ang), 0.0),
            "trace": trace}


def demand_rpm(d, vx, wz):
    """Wheel speed a combined translate-and-rotate asks for, in rpm.

    They ADD: the outer wheel carries the linear component plus the rotational
    one. This is the number the old limits got wrong -- each looked survivable
    alone."""
    if d["circ"] <= 0:
        return 0.0
    return (abs(vx) * 60.0 / d["circ"]) + (abs(wz) * d["radius"] * 60.0 / d["circ"])


# ==============================================================================
# The step response, read as a plant.
#
# test_acc is a STEP RESPONSE, by design: full PWM applied at t=0 to a wheel at
# rest, sampled every 20 ms -- which is exactly the control loop's own period
# (CONTROL_TIMER, 50 Hz). That is not a coincidence in how it was built, and it
# is what makes the output usable for control design rather than just a
# datasheet: from a step you get the plant, and from the plant you get gains
# that are stable by construction instead of by trial.
#
# The plant here is PWM counts in, wheel RPM out. To a good approximation it is
# first order with a dead zone:
#
#     G(s) = K / (tau*s + 1),   with no output at all below SIM_WHEEL_STALL_DUTY
#
# K and tau are MEASURED off the simulated response rather than taken from the
# model's constants, for the same reason you would measure them on a bench: the
# model is not purely first order -- Coulomb drag, the lagging pack and the
# driver's limiter all bend it -- and what a controller sees is the bent curve,
# not the ideal one it was assembled from.
# ==============================================================================
def identify_plant(d, measured=None):
    """Fit the first-order plant the control loop actually faces."""
    measured = measured or run_test_acc(d, rotate=False)
    drive = [v for v, _ in measured["trace"][:PHASE_SAMPLES]]
    if d["circ"] <= 0 or not drive:
        return None
    # Wheel rpm, which is what the PID regulates -- not the body's m/s.
    rpm = [v * 60.0 / d["circ"] for v in drive]
    steady = rpm[-1]
    if steady <= 0:
        return None

    pwm_max = float(d["pwm_max"])

    def _cross(frac):
        """First sample at or past `frac` of steady, linearly interpolated."""
        target = steady * frac
        for i, v in enumerate(rpm):
            if v >= target:
                if i == 0:
                    return 0.0
                span = rpm[i] - rpm[i - 1]
                k = (target - rpm[i - 1]) / span if span > 0 else 0.0
                return (i - 1 + k) * TICK_S
        return PHASE_SAMPLES * TICK_S

    # 63.2% is the definition of the time constant for a first-order step.
    tau = _cross(0.632)
    return {
        "steady_rpm": steady,
        "pwm_step": pwm_max,
        # DC gain: rpm per PWM count. The number the loop gain is the inverse of.
        "gain": steady / pwm_max if pwm_max > 0 else 0.0,
        "tau": tau,
        "t_rise": _cross(0.9) - _cross(0.1),
        # 2% settling. Within a 1 s step the model does not always get there,
        # and saying so is more useful than extrapolating.
        "t_settle": _cross(0.98),
        "settled": rpm[-1] >= steady * 0.98,
        # The dead zone is a duty, and the loop's output is counts.
        "dead_counts": d["stall_duty"] * pwm_max,
        # A first-order plant cannot overshoot; anything here means the model
        # has picked up a resonance and the gains below would not be valid.
        "overshoot": max(0.0, (max(rpm) - steady) / steady),
        "ts": TICK_S,
    }


def simulate_closed_loop(d, gains, setpoint_rpm=None, seconds=2.0):
    """Close the PID round the simulated wheel and step it.

    This is the part the bench could never do. A step response measured open
    loop tells you the plant; it does not tell you whether the gains in the
    config make a STABLE loop, and sim mode used to be no help because the
    wheels did not respond to PWM at all -- they tracked the command, so every
    set of gains looked perfect. Now the wheel is a plant driven by PWM, so the
    loop is a real loop and a bad gain oscillates here exactly as it would on a
    robot.

    PID::compute() is transcribed, not approximated: positional form, integral
    as a plain sum, derivative as a plain difference, and the same anti-windup
    clamp on the integral's CONTRIBUTION that the firmware applies -- which is
    the part that decides how a saturated loop recovers, and therefore most of
    what "stable" means here.
    """
    plant = identify_plant(d)
    if not plant:
        return None
    if setpoint_rpm is None:
        setpoint_rpm = plant["steady_rpm"] * 0.5      # a normal cruise, not the rail
    pwm_max = d["pwm_max"]
    kp, ki, kd = float(gains["kp"]), float(gains["ki"]), float(gains["kd"])

    pack = _Pack(d)
    wheel = _Wheel(d, pack, 0)
    integral = 0.0
    prev_error = 0.0
    trace = []
    for _ in range(int(seconds / TICK_S)):
        error = setpoint_rpm - wheel.rpm
        integral += error
        derivative = error - prev_error
        if ki != 0.0:
            i_max = pwm_max / abs(ki)             # the firmware's anti-windup
            integral = min(max(integral, -i_max), i_max)
        if setpoint_rpm == 0.0 and abs(error) < 0.5:
            integral = derivative = 0.0
        u = kp * error + ki * integral + kd * derivative
        u = min(max(u, -pwm_max), pwm_max)
        prev_error = error
        wheel.duty = u / pwm_max
        wheel.step(TICK_S, True)
        trace.append(wheel.rpm)

    peak = max(trace) if trace else 0.0
    overshoot = (peak - setpoint_rpm) / setpoint_rpm if setpoint_rpm > 0 else 0.0
    # Settled = inside +/-2% and STAYING there, so a curve passing through the
    # band on its way to an oscillation is not counted as settled.
    band = setpoint_rpm * 0.02
    settle = None
    for i in range(len(trace)):
        if all(abs(v - setpoint_rpm) <= band for v in trace[i:]):
            settle = i * TICK_S
            break
    final = trace[-1] if trace else 0.0
    # Sign changes of the error, after the first crossing: a loop that keeps
    # crossing is ringing, whatever its overshoot figure says.
    crossings = 0
    for a, b in zip(trace, trace[1:]):
        if (a - setpoint_rpm) * (b - setpoint_rpm) < 0:
            crossings += 1
    return {"setpoint": setpoint_rpm, "overshoot": overshoot, "settle": settle,
            "final": final, "steady_error": setpoint_rpm - final,
            "crossings": crossings, "trace": trace}


# How much slower than the plant the closed loop is asked to be.
#
# IMC/lambda tuning: lambda is the closed-loop time constant, and the ratio to
# the open-loop tau is the whole trade. 1 is as fast as the plant and lively; 3
# is sluggish and very forgiving. 2 is the usual starting point for a mechanical
# loop with a noisy sensor, and a wheel encoder at 50 Hz is exactly that.
LAMBDA_RATIO = 2.0


def suggest_pid(d, plant=None, lambda_ratio=LAMBDA_RATIO):
    """PI gains for THIS firmware's PID, from the identified plant.

    The discretisation matters and is easy to get wrong, so it is written out.
    PID::compute() is positional and NOT time-normalised:

        u = kp*e + ki*sum(e) + kd*(e - e_prev)

    -- `integral_ += error`, a plain sum with no dt, so ki carries the sample
    period inside it. For a continuous PI Kc*(e + (1/Ti)*integral(e dt)) sampled
    at Ts, the discrete equivalents are kp = Kc and ki = Kc*Ts/Ti.

    IMC for a first-order plant gives Kc = tau / (K*lambda) and Ti = tau, so:

        kp = tau / (K * lambda)
        ki = kp * Ts / tau

    kd is zero on purpose. A first-order plant needs no derivative term, and
    this one differentiates a 50 Hz encoder reading with no filter -- on a real
    robot that is an amplifier for quantisation noise, which is where the
    chattering comes from.
    """
    plant = plant or identify_plant(d)
    if not plant or plant["gain"] <= 0 or plant["tau"] <= 0:
        return None
    lam = plant["tau"] * lambda_ratio
    kp = plant["tau"] / (plant["gain"] * lam)
    ki = kp * plant["ts"] / plant["tau"]
    return {"kp": round(kp, 3), "ki": round(ki, 3), "kd": 0.0,
            "lambda_s": lam, "lambda_ratio": lambda_ratio}


# What "stable" has to mean before a gain is written into a robot's config.
#
# Overshoot on a wheel loop is not cosmetic: the base surges past the commanded
# speed and the controller above it is tracking a plant that argues back. And a
# loop that keeps crossing the setpoint is ringing however small each excursion
# is, so the crossing count is a separate test from the overshoot figure.
PID_MAX_OVERSHOOT = 0.02
PID_MAX_CROSSINGS = 0
# Fast to slow. The first one that passes wins, so the order IS the preference:
# the tightest loop that is still calm.
LAMBDA_CANDIDATES = (1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0, 6.0)


def auto_tune_pid(d, plant=None):
    """Tune, then PROVE it on the closed loop before offering the numbers.

    IMC gives gains from the plant; it does not know about the dead zone, the
    anti-windup clamp, the PWM rail, the Coulomb drag that makes the plant
    non-linear near zero, or the current limiter. So the candidate is simulated
    against the real loop -- the transcribed PID driving the transcribed wheel --
    and rejected if it overshoots or rings.

    Returns the gains plus the closed-loop evidence for them, or None if nothing
    passed, which is a legitimate answer: it means this chassis should not have
    its gains rewritten unattended.
    """
    plant = plant or identify_plant(d)
    if not plant:
        return None
    best = None
    for ratio in LAMBDA_CANDIDATES:
        gains = suggest_pid(d, plant, lambda_ratio=ratio)
        if not gains:
            continue
        # Two setpoints, because a wheel loop is not linear: the dead zone and
        # the Coulomb drag dominate at low speed, the PWM rail and the pack's
        # sag at high speed. Gains that are calm at cruise and ring at a crawl
        # are not calm.
        runs = [simulate_closed_loop(d, gains, setpoint_rpm=plant["steady_rpm"] * f)
                for f in (0.25, 0.9)]
        if any(r is None for r in runs):
            continue
        if any(r["overshoot"] > PID_MAX_OVERSHOOT or
               r["crossings"] > PID_MAX_CROSSINGS for r in runs):
            continue
        settle = max((r["settle"] if r["settle"] is not None else 1e9) for r in runs)
        gains = dict(gains, settle=settle, checked=runs)
        if best is None or settle < best["settle"]:
            best = gains
    return best


def auto_pid_enabled(params):
    """On when the robot has been MEASURED; off when only the model has spoken.

    Separate from the Nav2 flag, and defaulted differently, because they are
    different kinds of decision. A Nav2 limit is a BOUND: get it wrong and the
    robot is slower than it needed to be, and the velocity smoother clamps the
    consequences either way. A loop gain is STABILITY: get it wrong and the
    wheel oscillates, and nothing downstream saves it.

    So the default follows the evidence. With `kinematics.step_response`
    recorded -- a real test_acc run off a real board -- the gains are derived
    from a measurement of this robot and are rewritten. Without it the only
    plant available is the simulated one, and no motor has ever turned: the
    report still prints what it would suggest, loudly, but it does not go and
    change a stability-critical number on the strength of a model.

    `kinematics.auto_pid: true` overrides that for somebody who wants the
    model's gains anyway, and `false` turns it off even on a measured robot.
    """
    kine = params.get("kinematics") or {}
    if "auto_pid" in kine:
        flag = kine["auto_pid"]
        if isinstance(flag, bool):
            return flag
        return str(flag).strip().lower() not in ("false", "0", "no", "off")
    step = kine.get("step_response") or {}
    return bool(isinstance(step, dict) and step.get("max_vel") and step.get("t_to_90"))


# ==============================================================================
# Identifying the plant from a REAL test_acc run.
#
# Everything above reads the SIMULATED step. On a robot with motors the same two
# numbers come off the board, and they are the two test_acc already prints:
#
#     MAX VEL   0.71   0.00 m/s    0.03 rad/s
#     time to 0.9x max vel   0.42 sec
#
# For a first-order step, 0.9 of final is reached at ln(10)*tau = 2.303*tau, so
# those two lines are a complete identification -- steady state and time
# constant -- in the loop's own units. Which is why the tool prints them.
# ==============================================================================
T90_OVER_TAU = math.log(10.0)          # 2.3026


# ------------------------------------------------------------------------------
# The richer identification a real board can now produce.
#
# test_acc's `IDENT` lines are per WHEEL and include a closed-loop pass, which
# the aggregate MAX VEL / time-to-0.9x table cannot give:
#
#   per wheel   four real wheels have different friction, different gearboxes
#               and, on a used robot, different wear. One set of gains is then
#               wrong for three of them, and the aggregate table -- which
#               averages four wheels into one velocity -- cannot say so.
#   closed loop a plant fit says what the motor does when shoved. Whether the
#               PID around it is stable is a different question, and the
#               simulated check here cannot answer it for a real robot: it knows
#               nothing about that robot's backlash, encoder quantisation or
#               load.
# ------------------------------------------------------------------------------
IDENT_LINE = re.compile(r"^IDENT (\w+) (.*)$", re.MULTILINE)


def parse_ident(text):
    """The IDENT block from a real test_acc run, as plain dicts.

    Returns None when the transcript has no IDENT lines at all -- an older
    firmware, which is a fact worth reporting rather than an error.
    """
    out = {"gains": {}, "robot": {}, "deadzone": {}, "plant": {}, "loop": {}}
    found = False
    for kind, rest in IDENT_LINE.findall(text):
        fields = {}
        for token in rest.split():
            if "=" not in token:
                continue
            k, _, v = token.partition("=")
            try:
                fields[k] = float(v)
            except ValueError:
                fields[k] = v
        if kind in ("gains", "robot"):
            out[kind] = fields
            found = True
        elif kind in ("deadzone", "plant"):
            wheel = int(fields.get("wheel", 0))
            out[kind][wheel] = fields
            found = True
        elif kind == "loop":
            wheel = int(fields.get("wheel", 0))
            out["loop"].setdefault(wheel, []).append(fields)
            found = True
    return out if found else None


def wheels_disagree(ident, tol=0.15):
    """Which wheels differ from the median by more than `tol`, and in what.

    The question the aggregate table cannot ask. A wheel whose gain or time
    constant is 15% off its siblings will not be well served by their gains, and
    on a real robot that shows up as a base that pulls to one side under
    acceleration -- which reads like a kinematics error and is not one.
    """
    findings = []
    for field, label in (("K", "plant gain"), ("tau_ms", "time constant"),
                         ("steady_rpm", "top speed")):
        vals = {w: p.get(field) for w, p in (ident.get("plant") or {}).items()
                if isinstance(p.get(field), float) and p[field] > 0}
        if len(vals) < 2:
            continue
        ordered = sorted(vals.values())
        mid = ordered[len(ordered) // 2]
        if mid <= 0:
            continue
        for w, v in sorted(vals.items()):
            if abs(v - mid) / mid > tol:
                findings.append((w, label, v, mid, (v - mid) / mid))
    return findings


def ident_report(ident, d=None):
    """What a real board's IDENT block says, and what to do about it."""
    out = ["--- measured on the robot (test_acc IDENT)"]
    g = ident.get("gains") or {}
    if g:
        out.append(f"    running gains kp {g.get('kp')} ki {g.get('ki')} "
                   f"kd {g.get('kd')}, {g.get('rate_hz', 50):.0f} Hz, "
                   f"{g.get('pwm_max', 0):.0f} counts full scale")

    for w in sorted(ident.get("plant") or {}):
        pl = ident["plant"][w]
        dz = (ident.get("deadzone") or {}).get(w, {})
        dz_txt = (f", dead below {dz['pwm']:.0f} counts ({dz['duty'] * 100:.1f}%)"
                  if dz.get("pwm", -1) >= 0 else ", dead zone not found")
        out.append(f"    wheel {w}: {pl.get('steady_rpm', 0):.1f} rpm, "
                   f"tau {pl.get('tau_ms', 0):.0f} ms, K {pl.get('K', 0):.5f}{dz_txt}")

    odd = wheels_disagree(ident)
    if odd:
        out.append("    ** the wheels do not match, so one set of gains cannot suit "
                   "all of them:")
        for w, label, v, mid, frac in odd:
            out.append(f"       wheel {w} {label} {v:.4g} vs {mid:.4g} median "
                       f"({frac * 100:+.0f}%)")
    elif len(ident.get("plant") or {}) > 1:
        out.append("    the wheels agree within 15%, so one set of gains suits them all")

    loops = ident.get("loop") or {}
    if loops:
        out.append("    closed loop, as configured:")
        for w in sorted(loops):
            for run in loops[w]:
                verdict = ("calm" if run.get("overshoot", 1) <= PID_MAX_OVERSHOOT
                           and run.get("crossings", 9) <= PID_MAX_CROSSINGS else "RINGS")
                settle = run.get("settle_ms", -1)
                settle_txt = "never    " if settle < 0 else f"{settle:.0f} ms"
                if settle < 0:
                    verdict = "RINGS"      # it never got inside the band at all
                out.append(f"       wheel {w} at {run.get('sp', 0):5.1f} rpm: "
                           f"overshoot {run.get('overshoot', 0) * 100:5.1f}%  "
                           f"crossings {run.get('crossings', 0):.0f}  "
                           f"settle {settle_txt}  "
                           f"steady err {run.get('err', 0):+.2f} rpm   {verdict}")
    return out


def plant_from_ident(d, ident):
    """One plant for the tuner, from the per-wheel measurements.

    The MEDIAN wheel, not the mean: a single seized or miswired wheel would drag
    an average and produce gains that suit no wheel at all, where the median
    still describes a real one.
    """
    plants = [p for p in (ident.get("plant") or {}).values()
              if isinstance(p.get("K"), float) and p["K"] > 0 and p.get("tau_ms", 0) > 0]
    if not plants:
        return None
    ks = sorted(p["K"] for p in plants)
    taus = sorted(p["tau_ms"] for p in plants)
    steadies = sorted(p["steady_rpm"] for p in plants)
    mid = len(plants) // 2
    pwm_max = float((ident.get("gains") or {}).get("pwm_max") or d["pwm_max"])
    return {
        "steady_rpm": steadies[mid],
        "pwm_step": pwm_max,
        "gain": ks[mid],
        "tau": taus[mid] / 1000.0,
        "t_rise": math.log(9.0) * taus[mid] / 1000.0,
        "t_settle": math.log(50.0) * taus[mid] / 1000.0,
        "settled": True,
        "dead_counts": ((ident.get("deadzone") or {}).get(1, {}) or {}).get("pwm", 0.0),
        "overshoot": 0.0,
        "ts": TICK_S,
        "measured": True,
        "per_wheel": True,
    }


def plant_from_measurements(d, max_vel, t_to_90, pwm_step=None):
    """The same plant dict, from a real robot's numbers instead of the model."""
    if d["circ"] <= 0 or max_vel <= 0 or t_to_90 <= 0:
        return None
    pwm_max = float(pwm_step or d["pwm_max"])
    steady = max_vel * 60.0 / d["circ"]
    tau = t_to_90 / T90_OVER_TAU
    return {
        "steady_rpm": steady,
        "pwm_step": pwm_max,
        "gain": steady / pwm_max if pwm_max > 0 else 0.0,
        "tau": tau,
        # The board reports one crossing time, so the rest of the curve's shape
        # is inferred from the first-order fit rather than measured. Said here
        # rather than silently presented as if it had been.
        "t_rise": (math.log(9.0)) * tau,        # 10% -> 90% of a first-order step
        "t_settle": math.log(50.0) * tau,       # 2%
        "settled": True,
        "dead_counts": d["stall_duty"] * pwm_max,
        "overshoot": 0.0,
        "ts": TICK_S,
        "measured": True,
    }


TEST_ACC_VEL = re.compile(r"MAX VEL\s+(-?[\d.]+)")
TEST_ACC_T90 = re.compile(r"time to 0\.9x max vel\s+(-?[\d.]+)")
TEST_ACC_PWM = re.compile(r"MAX PWM\s+(-?[\d.]+)")


def parse_test_acc(text):
    """Pull the step response out of a real test_acc transcript.

    The FIRST run is the one used: test_acc halves the PWM every fourth run, so
    later blocks are steps of a different size and mixing them would fit a plant
    to two different inputs.
    """
    vel = TEST_ACC_VEL.search(text)
    t90 = TEST_ACC_T90.search(text)
    pwm = TEST_ACC_PWM.search(text)
    if not vel or not t90:
        return None
    return {"max_vel": float(vel.group(1)), "t_to_90": float(t90.group(1)),
            "pwm_step": float(pwm.group(1)) if pwm else None}


# ==============================================================================
# Nav2 limits, derived from the motors rather than typed in.
#
# The shipped limits asked for 103% of a differential base's motors, 171% at the
# velocity smoother's ceiling and 129% on mecanum, and nothing noticed until
# three mecanum legs left the room. The fix at the time was to lower them by
# hand, which fixes one chassis: the next robot with different wheels, a heavier
# load or a weaker pack starts the same way, because the numbers are a judgement
# about a robot nobody re-measured.
#
# So they are computed. This is ON by default -- `kinematics.auto_nav2_limits:
# false` turns it off for someone who wants to tune by hand, and then nothing
# here touches their values.
#
# The fractions below are not arbitrary: they reproduce the limits this project
# settled on for its default chassis on 2026-09-23 (smoother [0.3, ., 1.2] /
# [0.8, ., 1.5], desired_linear_vel 0.25, rotate_to_heading 1.0) and then scale
# with the model, so a heavier robot or a softer pack gets proportionally lower
# limits without anyone re-deciding.
#
#   speed        47% of what the base reaches in test_acc's 1 s phase. Half the
#                capability, and the other half is what tracking a curve needs.
#   rotation     26% of the achievable yaw rate -- LOWER than the linear
#                fraction, because rotation is where tracking error grows
#                fastest and, on a mecanum, where wheel speed is most expensive.
#   accel        31% of the SETTLED acceleration (not the first kick): a rate
#                limit the base only meets while the pack is still stiff is one
#                it misses for the rest of the manoeuvre.
#   ang accel    whatever reaches the angular ceiling in 0.8 s, capped at 31% of
#                the settled angular acceleration so a weak base cannot be given
#                a figure its motors cannot produce.
#   targets      83% of the smoother's ceiling, so the controller asks for
#                something the smoother can actually pass through.
#
# Every one of them is then checked against the wheel-speed budget, because a
# translation and a rotation ADD at the outer wheel and each looked survivable
# alone. That check is the reason this exists, so it is applied to the OUTPUT and
# not merely offered as advice.
# ==============================================================================
# Re-calibrated 2026-09-24 after the pack-load fix: a differential base drives
# two wheels, not four, so it sags less and the model's measured speed rose ~4%.
# The fractions came down by the same amount so the derived limits land where
# they did before -- the calibration point is the values this project settled on
# for its default chassis, and a bug fix in the model must not become a quiet
# re-tune of every robot. Especially not upward, and especially not on 2wd,
# which is the slice failing 3/10 on hardware while the other two pass 10/10.
SPEED_FRAC = 0.45
ROT_FRAC = 0.244
ACCEL_FRAC = 0.31
ANG_ACCEL_SECONDS = 0.8
TARGET_FRAC = 0.83

# The default driver margin: the controller may ask for 80% of no-load speed.
#
# Set on 2026-09-24, straight after the 2wd cliff. The derivation below measures
# what the motors CAN deliver (0.87-0.91 on the default chassis), and asking for
# all of it leaves the loop nothing to correct with -- which is how a 2.5% rise
# in the yaw ceiling turned a green slice into 7/10, twice. A margin is headroom
# by definition; spending it because the measurement says you could is the same
# mistake in a different place.
#
# So the measurement can only make this MORE conservative, never less: a heavy
# robot that reaches half its no-load speed gets 0.50, and a light one that
# reaches 93% still gets 0.80.
DEFAULT_MARGIN = 0.80


def suggest_max_rpm_ratio(d, measured=None, raw=False):
    """The driver margin, derived instead of guessed.

    `max_rpm_ratio` has always been 0.85 -- a 15% derating somebody picked, with
    nothing behind the number. What it expresses IS the margin: the gap between
    the motor's no-load speed and the speed the wheel actually reaches once the
    gearbox, the gear drag, the viscous friction, the pack's sag and the bridge's
    losses have taken their share. That gap is not a constant. It is small on a
    light robot with a stiff pack and large on a heavy one -- and the heavy case
    is precisely where commanding 85% of no-load asks for rpm the motor cannot
    give.

    So it is measured off the model, on test_acc's own 1 s profile: the speed a
    manoeuvre actually gets, not the asymptote it would reach given longer.

    Nothing is shaded off it and it is not clamped into a "sensible" band. The
    velocity smoother is what bounds what the robot is asked to do, and its
    envelope is derived from the same measurement -- a second fudge here would
    only derate the same physics twice, and a floor or ceiling would quietly
    replace the measurement with a constant on exactly the unusual robots this
    exists to describe.

    It is capped at DEFAULT_MARGIN. The measurement says what the motors reach;
    the margin says how much of that the controller is allowed to spend, and the
    answer is not "all of it" -- see the 2wd cliff. On the default chassis the
    motors reach 0.91 and the controller is given 0.80; on a 15 kg robot the
    motors reach 0.50 and the controller is given 0.50, because there the
    measurement is the binding constraint and the margin is not.
    """
    measured = measured or run_test_acc(d, rotate=False)
    if d["circ"] <= 0 or d["motor_rpm"] <= 0:
        return None
    reached_rpm = measured["max_vel"] * 60.0 / d["circ"]
    # Whichever is SMALLER: what the motors reach, or the standing margin. The
    # measurement exists to catch the robot that cannot even manage 80%.
    ratio = min(reached_rpm / d["motor_rpm"], DEFAULT_MARGIN)
    # Rounded for WRITING only. The budget the velocity limits are clamped
    # against is computed from the unrounded value (see derived_limits), because
    # feeding the written two-decimal ratio back in makes the whole derivation a
    # function of its own last output: the mecanum reference then oscillated
    # between two answers a hundredth apart, and every save produced a diff.
    return round(ratio, 2) if not raw else ratio


def suggest_nav2_limits(d, p=None, measured=None):
    """The Nav2 limits this drivetrain can actually meet, as dotted paths."""
    p = p or performance(d)
    measured = measured or run_test_acc(d, rotate=False)
    rot = run_test_acc(d, rotate=True)

    # What the base MEASURES over the second a manoeuvre lasts, not the
    # asymptote it converges on given longer.
    v = measured["max_vel"] * SPEED_FRAC
    w = rot["max_vel"] * ROT_FRAC

    # ...and then the budget, because the two add at the outer wheel. If the
    # pair does not fit, both are scaled by the same factor: shaving only one
    # would silently change the robot's character (a base that turns but will
    # not drive, or the reverse).
    if d["command_rpm"] > 0:
        need = demand_rpm(d, v, w)
        if need > d["command_rpm"]:
            shrink = d["command_rpm"] / need
            v *= shrink
            w *= shrink

    a = p["lin_acc_held"] * ACCEL_FRAC
    ang_a = min(w / ANG_ACCEL_SECONDS, p["ang_acc_held"] * ACCEL_FRAC)

    def r2(x):
        # +0.0, never -0.0: it round-trips through YAML as "-0.0", which reads
        # like a deliberate negative zero and is just noise.
        return round(x, 2) + 0.0

    v, w, a, ang_a = r2(v), r2(w), r2(a), r2(ang_a)
    # A mecanum is the only base that can command y at all; for the others the
    # smoother's y limits must stay 0 or it will pass through a velocity the
    # kinematics cannot produce.
    vy = r2(v) if d["base"] == "mecanum" else 0.0

    return {
        "nav2.velocity_smoother.ros__parameters.max_velocity": [v, vy, w],
        "nav2.velocity_smoother.ros__parameters.min_velocity": [-v, -vy + 0.0, -w],
        "nav2.velocity_smoother.ros__parameters.max_accel": [a, r2(a) if vy else 0.0, ang_a],
        "nav2.velocity_smoother.ros__parameters.max_decel":
            [-a, -(r2(a)) if vy else 0.0, -ang_a],
        "nav2.controller_server.ros__parameters.FollowPath.desired_linear_vel":
            r2(v * TARGET_FRAC),
        "nav2.controller_server.ros__parameters.FollowPath.rotate_to_heading_angular_vel":
            r2(w * TARGET_FRAC),
        "nav2.controller_server.ros__parameters.FollowPath.max_angular_accel": ang_a,
        # Recovery spins are allowed to be brisker than path following: they
        # happen when the robot is stuck, with nothing to track.
        "nav2.behavior_server.ros__parameters.max_rotational_vel": r2(w * 0.67),
        "nav2.behavior_server.ros__parameters.min_rotational_vel": r2(w * 0.33),
        "nav2.behavior_server.ros__parameters.rotational_acc_lim": r2(ang_a * 2.0),
    }


def auto_limits_enabled(params):
    """Default ON. `kinematics.auto_nav2_limits: false` hands control back.

    The flag lives in `kinematics` beside `stamped_cmd_vel` rather than in `nav2`
    because the nav2 block is dumped whole into a ROS params file, where every
    top-level key has to be a node name.
    """
    flag = (params.get("kinematics") or {}).get("auto_nav2_limits", True)
    if isinstance(flag, bool):
        return flag
    return str(flag).strip().lower() not in ("false", "0", "no", "off")


def config_plant(params, d=None):
    """The plant to tune against: a REAL measurement if the config records one.

    `kinematics.step_response` is where a robot's own test_acc run is kept:

        step_response:
          max_vel: 0.71       # m/s, the MAX VEL line
          t_to_90: 0.42       # s, the "time to 0.9x max vel" line
          pwm: 1023           # the MAX PWM line, if the step was not full scale
          measured: 2026-09-24

    It is stored rather than re-derived because it is evidence: the gains that
    come out of it are only reproducible if the two numbers they came from are
    written down, and a robot that has been measured should not silently fall
    back to the model the next time somebody saves its config.

    Absent, the simulated step is used, which is the right answer for a robot
    that does not exist yet -- which is most of them, at design time.
    """
    d = d or drivetrain(params)
    step = (params.get("kinematics") or {}).get("step_response") or {}
    if isinstance(step, dict) and step.get("max_vel") and step.get("t_to_90"):
        got = plant_from_measurements(d, float(step["max_vel"]),
                                      float(step["t_to_90"]), step.get("pwm"))
        if got:
            return got
    return identify_plant(d)


def derived_limits(params):
    """Everything auto-tuning would write, as dotted paths -> values.

    The driver margin comes FIRST and the Nav2 limits are then derived against
    it, because max_rpm_ratio is what sets the wheel-speed budget those limits
    are checked against. Deriving them in the other order would clamp the
    envelope to the budget the config happened to ship with, and then move the
    budget out from under it.

    There is no circularity: the margin is measured from the motor's no-load
    speed and the model's losses, neither of which depends on the margin.
    """
    d = drivetrain(params)
    if d["max_rpm"] <= 0 or d["circ"] <= 0:
        return {}
    out = {}
    measured = run_test_acc(d, rotate=False)
    ratio = suggest_max_rpm_ratio(d, measured)
    if ratio is not None:
        out["kinematics.max_rpm_ratio"] = ratio
        # The UNROUNDED ratio sets the budget, so the limits below depend only on
        # the measurement and not on what was written to the config last time.
        d["command_rpm"] = d["motor_rpm"] * suggest_max_rpm_ratio(d, measured, raw=True)
    out.update(suggest_nav2_limits(d, measured=measured))

    # The wheel loop's gains, from the step response -- the robot's own if it
    # has been measured, the model's otherwise. Only written when the closed-loop
    # check passed: auto_tune_pid() returns None when no candidate was calm at
    # both a crawl and near full speed, and "leave the gains alone" is the right
    # answer then.
    if auto_pid_enabled(params):
        gains = auto_tune_pid(d, config_plant(params, d))
        if gains:
            for key in ("kp", "ki", "kd"):
                out[f"kinematics.pid.{key}"] = gains[key]
    return out


def apply_derived_limits(params):
    """Write the derived limits into `params` in place. Returns what changed."""
    if not auto_limits_enabled(params):
        return {}
    changed = {}
    for path, value in derived_limits(params).items():
        node = params
        parts = path.split(".")
        for part in parts[:-1]:
            nxt = node.get(part)
            if not isinstance(nxt, dict):
                nxt = {}
                node[part] = nxt
            node = nxt
        if node.get(parts[-1]) != value:
            changed[path] = {"from": node.get(parts[-1]), "to": value}
            node[parts[-1]] = value
    return changed


# The old name, from when this only wrote Nav2 keys. Kept because the backend
# and the bench both call it, and because "nav2 limits" is still what a person
# means when they turn the feature off.
apply_nav2_limits = apply_derived_limits


def report(params, name=""):
    d = drivetrain(params)
    if d["max_rpm"] <= 0 or d["circ"] <= 0:
        raise SystemExit("kinematics.max_rpm and wheel_diameter must be positive")
    p = performance(d)
    nav = params.get("nav2", {}) or {}
    vx = _dig(nav, "desired_linear_vel")
    wz = _dig(nav, "rotate_to_heading_angular_vel")
    smoother_v = _dig(nav, "max_velocity")
    smoother_a = _dig(nav, "max_accel")

    out = []
    out.append(f"=== {name or params.get('robot', {}).get('name', 'robot')}"
               f"   base={d['base']}  {d['wheels']} driven wheels")
    out.append(f"    motor {d['motor_rpm']:.0f} rpm no-load at the wheel"
               + (f" (derated to {d['volt_ratio'] * 100:.0f}% by the pack)" if d['volt_ratio'] < 0.999 else "")
               + f", controller will ask at most {d['command_rpm']:.0f}")
    out.append(f"    {d['circ'] / math.pi * 1000:.0f} mm wheels, {d['mass']:.2f} kg")
    out.append(f"    gearbox {d['gear_eff'] * 100:.0f}% efficient, drag {d['coulomb']:.0f} rpm/s")
    out.append(f"    pack sag {d['sag'] * 100:.0f}% at full stall over {d['sag_tau_ms']:.0f} ms, "
               f"driver {d['drv_drop'] * 100:.0f}% fixed + {d['drv_r'] * 100:.0f}% at stall")
    if d["ilimit_a"] > 0.0:
        out.append(f"    driver limits at {d['ilimit_a']:.1f} A of the motor's "
                   f"{d['stall_a']:.1f} A stall -- torque capped to "
                   f"{min(d['ilimit_a'] / d['stall_a'], 1.0) * 100:.0f}% from rest")
    else:
        out.append(f"    no driver current limiter (motor stalls at {d['stall_a']:.1f} A)")
    out.append(f"    turns on {d['radius']:.4f} m"
               + ("  (mecanum: (lr+fr)/2)" if d["base"] == "mecanum" else "  (lr/2)"))
    out.append("")
    out.append("--- what it can do")
    out.append(f"    max speed          {p['lin_vel']:5.2f} m/s      {p['ang_vel']:5.2f} rad/s")
    # FROM REST, which is the peak: torque falls linearly as the wheel speeds
    # up, so this is the most the base will ever manage and it manages less at
    # every speed above zero. Comparing the smoother's rate limit against the
    # peak is therefore generous -- a limit that only just fits here will not be
    # met near top speed.
    out.append(f"    accel, first kick  {p['lin_acc']:5.2f} m/s2     {p['ang_acc']:5.2f} rad/s2"
               f"   (stiff pack, from rest)")
    out.append(f"    accel, sag settled {p['lin_acc_held']:5.2f} m/s2     {p['ang_acc_held']:5.2f} rad/s2"
               f"   (what a held manoeuvre gets)")
    out.append(f"    time to 0.9x max   {p['t_to_90']:5.2f} s        (tau {p['tau'] * 1000:.0f} ms)")
    out.append("    (the asymptote: the speed the wheel converges on, and the")
    out.append("     acceleration at the instant it starts)")
    out.append("")

    # The same model, run rather than solved, following test_acc's own profile.
    # This is the column to compare against a bench run, because it is the same
    # measurement: 20 ms samples, differentiated, 1 s phases, stop distance from
    # the integrated coast. The closed form above answers "what can it do"; this
    # answers "what will test_acc print", and the two differ by the sampling.
    lin_run = run_test_acc(d, rotate=False)
    rot_run = run_test_acc(d, rotate=True)
    out.append("--- what test_acc would print (simulated, no flashing)")
    out.append(f"    MAX VEL            {lin_run['max_vel']:5.2f} m/s     "
               f"{rot_run['max_vel']:6.2f} rad/s")
    out.append(f"    MAX ACC            {lin_run['max_acc']:5.2f} m/s2    "
               f"{rot_run['max_acc']:6.2f} rad/s2")
    out.append(f"    time to 0.9x max   {lin_run['t_to_90']:5.2f} s       "
               f"{rot_run['t_to_90']:6.2f} s")
    out.append(f"    distance to stop   {lin_run['stop']:5.3f} m       "
               f"{rot_run['stop']:6.3f} rad")
    out.append("    (20 ms samples, full PWM, noiseless -- the board's MAX ACC reads")
    out.append("     ~0.4 m/s2 high because it differentiates encoder noise)")
    # Why the two blocks disagree, said once rather than left to be guessed at.
    # test_acc drives for 1 s; the asymptote needs several tau and the last few
    # percent arrive slowly, because torque falls as the wheel speeds up.
    if p["lin_vel"] > 0:
        shortfall = (1.0 - lin_run["max_vel"] / p["lin_vel"]) * 100.0
        out.append(f"    test_acc's 1 s phase reaches {100 - shortfall:.0f}% of the asymptote:")
        out.append("     the last few percent arrive slowly, since torque falls with speed.")
        out.append("     Tune a velocity smoother from THIS block -- it is what the robot does")
        out.append("     in the second a manoeuvre lasts.")
    out.append("")

    # The step response as a PLANT, which is what test_acc was built to produce:
    # a step into a wheel at rest, sampled at the control loop's own 20 ms, so
    # the loop can be designed rather than guessed at.
    plant = config_plant(params, d)
    if plant:
        src = "measured on the robot" if plant.get("measured") else "from the model"
        out.append(f"--- the step response, as a plant ({src})")
        out.append(f"    K  {plant['gain']:.4f} rpm per PWM count "
                   f"({plant['steady_rpm']:.1f} rpm at a {plant['pwm_step']:.0f}-count step)")
        out.append(f"    tau {plant['tau'] * 1000:.0f} ms      rise (10-90%) "
                   f"{plant['t_rise'] * 1000:.0f} ms      2% settle "
                   f"{plant['t_settle'] * 1000:.0f} ms")
        out.append(f"    dead zone below {plant['dead_counts']:.0f} counts "
                   f"({d['stall_duty'] * 100:.0f}% duty), overshoot "
                   f"{plant['overshoot'] * 100:.1f}%")
        shipped = (params.get("kinematics") or {}).get("pid") or {}
        tuned = auto_tune_pid(d, plant)
        out.append("")
        out.append("--- the wheel loop, closed on that plant")
        for label, gains in (("config", shipped), ("auto-tuned", tuned)):
            if not gains or "kp" not in gains:
                continue
            runs = [simulate_closed_loop(d, gains, setpoint_rpm=plant["steady_rpm"] * f)
                    for f in (0.25, 0.9)]
            runs = [r for r in runs if r]
            if not runs:
                continue
            over = max(r["overshoot"] for r in runs) * 100.0
            ring = max(r["crossings"] for r in runs)
            settle = max((r["settle"] if r["settle"] is not None else float("inf"))
                         for r in runs)
            settle_s = f"{settle:.2f} s" if settle != float("inf") else "never"
            verdict = "calm" if (over <= PID_MAX_OVERSHOOT * 100 and
                                 ring <= PID_MAX_CROSSINGS) else "RINGS"
            out.append(f"    {label:11} kp {gains['kp']:<6} ki {gains['ki']:<6} "
                       f"kd {gains['kd']:<5} -> overshoot {over:5.1f}%  "
                       f"crossings {ring}  settle {settle_s}   {verdict}")
        if tuned:
            out.append(f"    (IMC, lambda = {tuned['lambda_ratio']:g} x tau, checked at a "
                       f"crawl and near full speed)")
        else:
            out.append("    (no candidate was calm at both speeds -- tune this one by hand)")
        out.append("")

    out.append("--- what the config asks for")

    verdict_lines, over = [], False
    for label, v, w in (("controller target", vx, wz),
                        ("smoother envelope",
                         smoother_v[0] if isinstance(smoother_v, list) else None,
                         smoother_v[2] if isinstance(smoother_v, list) and len(smoother_v) > 2 else None)):
        if v is None or w is None:
            verdict_lines.append(f"    {label:18} not set in this config")
            continue
        need = demand_rpm(d, v, w)
        cap = d["command_rpm"]
        pct = need / cap * 100.0
        flag = "OVER BUDGET" if need > cap else "ok"
        if need > cap:
            over = True
        verdict_lines.append(f"    {label:18} {v:.2f} m/s + {w:.2f} rad/s "
                             f"-> {need:6.1f} rpm = {pct:5.1f}% of {cap:.0f}   {flag}")
    out += verdict_lines

    if isinstance(smoother_a, list) and smoother_a:
        # Judged against the SETTLED acceleration, not the first kick: a rate
        # limit the base can only meet while the pack is still stiff is a limit
        # it misses for the rest of the manoeuvre.
        asked = float(smoother_a[0])
        have = p["lin_acc_held"]
        pct = asked / have * 100.0 if have > 0 else float("inf")
        flag = "OVER BUDGET" if asked > have else "ok"
        if asked > have:
            over = True
        out.append(f"    smoother accel     {asked:.2f} m/s2 "
                   f"-> {pct:5.1f}% of the {have:.2f} m/s2 it holds   {flag}")

    out.append("")
    out.append("    VERDICT: the config asks for more than the motors can give"
               if over else "    VERDICT: within the motors' budget")
    return "\n".join(out), over


def dr_drivetrain_safe(params):
    """drivetrain() for the CLI, where a half-written config must not traceback."""
    try:
        return drivetrain(params)
    except (TypeError, ValueError, KeyError):
        return {"pwm_max": 1023.0, "circ": 0.0, "stall_duty": 0.04}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", action="append", required=True,
                    help="robot config YAML; repeat to compare several")
    # A real robot's own step response. Everything else in this report is the
    # model; these two numbers are the board's, and they win when present --
    # a measured plant beats a simulated one every time.
    ap.add_argument("--from-test-acc", metavar="LOG",
                    help="a real test_acc transcript: identify the plant from its "
                         "MAX VEL and 'time to 0.9x max vel' lines instead of the model")
    ap.add_argument("--max-vel", type=float,
                    help="the MAX VEL figure (m/s), if you are typing it rather than "
                         "pasting a log")
    ap.add_argument("--t90", type=float,
                    help="the 'time to 0.9x max vel' figure (s)")
    ap.add_argument("--write", action="store_true",
                    help="write the derived limits and gains back into the config(s), "
                         "and record the measurement under kinematics.step_response")
    args = ap.parse_args()

    step = None
    ident = None
    if args.from_test_acc:
        with open(args.from_test_acc, encoding="utf-8") as fh:
            transcript = fh.read()
        # The IDENT block wins when it is there: per wheel, and with the loop
        # closed. The two summary lines are the fallback for a board running
        # firmware from before test_acc learned to identify itself.
        ident = parse_ident(transcript)
        step = parse_test_acc(transcript)
        if not ident and not step:
            raise SystemExit(f"{args.from_test_acc}: no 'MAX VEL' and 'time to 0.9x max "
                             f"vel' lines -- is this a test_acc transcript?")
    elif args.max_vel and args.t90:
        step = {"max_vel": args.max_vel, "t_to_90": args.t90, "pwm_step": None}
    elif bool(args.max_vel) != bool(args.t90):
        raise SystemExit("--max-vel and --t90 identify the plant together; "
                         "one without the other says nothing")

    any_over = False
    for path in args.params:
        with open(path, encoding="utf-8") as fh:
            params = yaml.safe_load(fh) or {}
        if ident:
            plant = plant_from_ident(dr_drivetrain_safe(params), ident)
            if plant:
                # The median wheel's plant, recorded the same way the two-number
                # fallback is -- so a later save re-derives the same gains
                # without the board being present.
                params.setdefault("kinematics", {})["step_response"] = {
                    "max_vel": round(plant["steady_rpm"] * math.pi
                                     * float((params.get("kinematics") or {})
                                             .get("wheel_diameter", 0)) / 60.0, 4),
                    "t_to_90": round(plant["tau"] * T90_OVER_TAU, 4),
                    "pwm": plant["pwm_step"],
                    "measured": datetime.date.today().isoformat(),
                    "source": "test_acc IDENT, median wheel",
                }
        elif step:
            # Recorded in the config, not just used: gains are only reproducible
            # if the two numbers behind them are written down.
            rec = {"max_vel": step["max_vel"], "t_to_90": step["t_to_90"],
                   "measured": datetime.date.today().isoformat()}
            if step.get("pwm_step"):
                rec["pwm"] = step["pwm_step"]
            params.setdefault("kinematics", {})["step_response"] = rec
        if args.write:
            changed = apply_derived_limits(params)
            with open(path, "w", encoding="utf-8") as fh:
                yaml.safe_dump(params, fh, sort_keys=False)
            print(f"=== {path}: wrote {len(changed)} value(s)")
            for k, v in changed.items():
                print(f"    {k}: {v['from']} -> {v['to']}")
            print()
        text, over = report(params, os.path.basename(path))
        print(text)
        if ident:
            print()
            print("\n".join(ident_report(ident)))
        print()
        any_over = any_over or over
    return 1 if any_over else 0


if __name__ == "__main__":
    sys.exit(main())
