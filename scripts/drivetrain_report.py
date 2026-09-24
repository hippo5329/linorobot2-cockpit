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
# The model is the firmware's own (firmware/common/lib/encoder/fake_wheel.h): a
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
FAKE_WHEEL_H = os.path.join(REPO_ROOT, "firmware", "common", "lib", "encoder",
                            "fake_wheel.h")


def model_defaults(path=FAKE_WHEEL_H):
    """The wheel model's constants, read from the firmware that implements it."""
    want = {
        "FAKE_WHEEL_TAU_MS": "tau_ms",
        "FAKE_WHEEL_REF_MASS": "ref_mass",
        "FAKE_WHEEL_MAX_ACCEL_RPM": "accel_clamp",
        "FAKE_WHEEL_FRICTION": "viscous",
        "FAKE_WHEEL_STALL_DUTY": "stall_duty",
        "FAKE_ROBOT_MASS": "mass",
        "FAKE_GEAR_EFFICIENCY": "gear_eff",
        "FAKE_WHEEL_COULOMB_RPM": "coulomb",
        "FAKE_BATT_SAG": "sag",
        "FAKE_BATT_SAG_TAU_MS": "sag_tau_ms",
        "FAKE_DRV_DROP": "drv_drop",
        "FAKE_DRV_R": "drv_r",
        "FAKE_MOTOR_STALL_A": "stall_a",
        "FAKE_DRV_ILIMIT_A": "ilimit_a",
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
    #                it is fed. What the machine CAN do. fake_wheel.h uses this.
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

    # The pack's sag LAGS (FAKE_BATT_SAG_TAU_MS), so the first instant of an
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
# model. This class is a transcription of FakeEncoder::integrate() and
# busScale() -- same terms, same order, same clamps, same shared pack -- and
# run_test_acc() is a transcription of test_acc.cpp's loop_() and dump_record().
# Given that, flashing test_acc and driving a board is no longer how these
# numbers are obtained. It is how this transcription is CHECKED, which is a
# different job and a much rarer one.
#
# Deliberately noiseless. getRPM() adds +/-FAKE_WHEEL_NOISE_RPM, and test_acc
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
    """One simulated wheel. Mirrors FakeEncoder; see fake_wheel.h."""

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
    duty = [-1.0 if rotate and i in (0, 2) else 1.0 for i in range(4)]

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
SPEED_FRAC = 0.47
ROT_FRAC = 0.26
ACCEL_FRAC = 0.31
ANG_ACCEL_SECONDS = 0.8
TARGET_FRAC = 0.83

def suggest_max_rpm_ratio(d, measured=None):
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

    It lands at 0.87 for the default chassis, near the hand-picked 0.85. That is
    the point: the guess was about right for the robot it was guessed on, and
    wrong for every other one.
    """
    measured = measured or run_test_acc(d, rotate=False)
    if d["circ"] <= 0 or d["motor_rpm"] <= 0:
        return None
    reached_rpm = measured["max_vel"] * 60.0 / d["circ"]
    return round(reached_rpm / d["motor_rpm"], 2)


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
    ratio = suggest_max_rpm_ratio(d)
    if ratio is not None:
        out["kinematics.max_rpm_ratio"] = ratio
        d["command_rpm"] = d["motor_rpm"] * ratio
    out.update(suggest_nav2_limits(d))
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


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", action="append", required=True,
                    help="robot config YAML; repeat to compare several")
    args = ap.parse_args()
    any_over = False
    for path in args.params:
        with open(path, encoding="utf-8") as fh:
            params = yaml.safe_load(fh) or {}
        text, over = report(params, os.path.basename(path))
        print(text)
        print()
        any_over = any_over or over
    return 1 if any_over else 0


if __name__ == "__main__":
    sys.exit(main())
