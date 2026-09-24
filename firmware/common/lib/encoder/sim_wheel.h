// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIM_WHEEL_H
#define SIM_WHEEL_H

#include <Arduino.h>
#include "mcu_env.h"
#include <micro_ros_utilities/string_utilities.h>
#include "encoder_interface.h"
// Self-contained rather than relying on main.cpp to have included these first.
// SimEncoder is now built by hw_factory, which has no reason to know about
// ROS messages, so this header can no longer assume a ROS-aware includer.
#include "imu_interface.h"
#include "mag_interface.h"

// Simulated drivetrain for a bare module with no motors or encoders attached.
//
// Each wheel is modelled as a DC motor driving a share of the robot's mass:
//
//   duty      = pwm / PWM_MAX                 commanded power
//   no_load   = duty * MOTOR_MAX_RPM          speed the motor would settle at
//   torque   ~= no_load - rpm                 back-EMF: torque falls as it spins up
//   d(rpm)/dt = torque / tau_eff              tau_eff scales with robot mass
//
// so the wheel accelerates hard when the error is large, tails off as it
// approaches speed, coasts down against friction when power is cut, and takes
// longer to do all of it on a heavier robot. Acceleration is clamped to a
// traction/current limit, and a stall band keeps a weak duty from creeping.
// The reported RPM carries white noise, so the PID has something real to
// correct against and the odometry drifts the way it does on real hardware.

// The robot's total weight is optional robot data from the config engine, which
// emits it as ROBOT_WEIGHT. Use it when it is there, so the simulated robot has
// the inertia of the one that was configured.
#ifndef SIM_ROBOT_MASS
    #ifdef ROBOT_WEIGHT
        #define SIM_ROBOT_MASS ROBOT_WEIGHT
    #else
        #define SIM_ROBOT_MASS 3.5     // simulated robot mass (kg)
    #endif
#endif

#ifndef SIM_WHEEL_TAU_MS
#define SIM_WHEEL_TAU_MS 150       // spin-up time constant (ms) at SIM_WHEEL_REF_MASS
#endif

#ifndef SIM_WHEEL_REF_MASS
#define SIM_WHEEL_REF_MASS 3.5     // mass SIM_WHEEL_TAU_MS was measured at (kg)
#endif

#ifndef SIM_WHEEL_MAX_ACCEL_RPM
#define SIM_WHEEL_MAX_ACCEL_RPM 900.0  // traction/current limit (RPM per second)
#endif

#ifndef SIM_WHEEL_FRICTION
#define SIM_WHEEL_FRICTION 0.06    // viscous drag, fraction of current RPM per second
#endif

#ifndef SIM_WHEEL_STALL_DUTY
#define SIM_WHEEL_STALL_DUTY 0.04  // duty below which the motor cannot break static friction
#endif

// --- the gearbox -----------------------------------------------------------
//
// MOTOR_MAX_RPM is the OUTPUT speed, so the reduction itself is already folded
// in and the model does not need the ratio. What it was missing is what a
// gearbox costs:
//
//   efficiency   a spur reduction returns 70-80% of the torque put into it, so
//                the driving term is scaled. Reflected inertia (N^2 * J_motor)
//                stays lumped into SIM_WHEEL_TAU_MS, which is where it belongs.
//   Coulomb drag a gear train has a roughly CONSTANT breakaway/running torque
//                loss, unlike the viscous term already here. It is why an
//                unpowered gear motor stops quickly instead of coasting, and why
//                a small duty produces no motion at all.
#ifndef SIM_GEAR_EFFICIENCY
#define SIM_GEAR_EFFICIENCY 0.75
#endif
#ifndef SIM_WHEEL_COULOMB_RPM
#define SIM_WHEEL_COULOMB_RPM 12.0  // constant drag (RPM per second) while turning
#endif

// --- battery sag -----------------------------------------------------------
//
// V_bus = V_oc - I * R_internal, and for a brushed motor the current is
// proportional to (no-load speed - current speed) -- the same term the driving
// torque uses. So a hard acceleration browns out its own supply, and every
// wheel shares one pack: this is the ONLY coupling between the four simulated
// wheels, and without it four driven wheels cost nothing extra, which is why a
// 4WD base used to accelerate exactly like a 2WD one.
//
// Expressed as the fraction of open-circuit voltage lost when all four wheels
// are demanding full stall current, so it needs no pack chemistry: 0.25 means a
// 20% sag at that worst case (1 / 1.25). Kinematics::setMeasuredVoltage() is the
// real-robot counterpart; this is its simulated twin.
#ifndef SIM_BATT_SAG
#define SIM_BATT_SAG 0.25
#endif

// How long the pack takes to sag, and to come back.
//
// Sag is not instantaneous. Internal resistance drops the voltage at once, but
// the chemistry behind it polarises over hundreds of milliseconds, so a load
// that is held gets DEEPER sag than the same load applied briefly -- and the
// recovery is just as slow. That is why a heavy robot suffers more than its peak
// current alone suggests: being heavy means drawing that current for longer, and
// the sag has time to develop.
//
// Modelled as a first-order lag on the demand the pack sees. 0 makes the sag
// instantaneous, which is the old behaviour.
#ifndef SIM_BATT_SAG_TAU_MS
#define SIM_BATT_SAG_TAU_MS 400
#endif

// --- the motor driver ------------------------------------------------------
//
// The bridge is between the pack and the motor and keeps some of the voltage for
// itself. Two parts, because they behave differently:
//
//   SIM_DRV_DROP  a fixed fraction, the body-diode / Vce floor. Present at any
//                  current, so it costs most at low duty.
//   SIM_DRV_R     a fraction proportional to the current drawn -- Rds(on) and
//                  the shunt. This one follows the current INSTANTLY, unlike the
//                  pack's sag, which is why the two are separate terms rather
//                  than one fudge factor.
//
// Stall is where both bite hardest: the current proxy is at 1.0, so the motor
// sees (1 - DROP) / (1 + SAG + DRV_R) of the pack -- which is the real reason a
// stalled motor produces less torque than its datasheet stall figure.
#ifndef SIM_DRV_DROP
#define SIM_DRV_DROP 0.03
#endif
#ifndef SIM_DRV_R
#define SIM_DRV_R 0.10
#endif

// --- the driver's current limiter ------------------------------------------
//
// Many small motor drivers chop at a fixed current -- a DRV8871 at 3.6 A, a
// TB6612 at 1.2 A per channel, plenty of boards set 2.5 A with a sense
// resistor. Below that limit the driver is transparent and the torque-speed
// line above is the whole story; at or above it the driver simply refuses to
// pass more current, and the motor's torque is pinned at whatever that current
// buys no matter how much speed error there is.
//
// That is a different shape of limit from everything else here, and it bites
// exactly where a robot is judged: from rest, at full duty, where the current
// demand is at its highest. It is why two robots with the same motors and the
// same pack can have very different acceleration, and why fitting a bigger
// motor to a board whose driver limits at 1.2 A changes nothing.
//
// Expressed in amps, because that is what a datasheet gives:
//
//   SIM_MOTOR_STALL_A   the motor's own stall current at its rated voltage.
//                        The current scale: a brushed motor draws stall current
//                        in proportion to (no-load speed - present speed), which
//                        is the same term the driving torque uses, so the model
//                        already computes it -- as a fraction, in demand().
//   SIM_DRV_ILIMIT_A    what the driver will actually pass. ZERO MEANS NO
//                        LIMITER, which is the default: a limit nobody entered
//                        must not quietly throttle every existing robot.
#ifndef SIM_MOTOR_STALL_A
#define SIM_MOTOR_STALL_A 2.5      // motor stall current (A) at rated voltage
#endif
#ifndef SIM_DRV_ILIMIT_A
#define SIM_DRV_ILIMIT_A 0.0       // driver current limit (A); 0 = none fitted
#endif

// Mass and encoder noise are properties of THIS robot, not of the image, so
// they come from the env with the macros as the fallback. Function-local
// statics rather than globals: these are read inside a class used before
// setup() finishes, and a global would be initialised before the flash
// partition API is usable.
static inline float simRobotMass()
{
    static float mass = -1.0f;
    if (mass < 0.0f)
        mass = envFloat("sim_mass", (float)SIM_ROBOT_MASS);
    return mass;
}

// The drivetrain's losses, from the env.
//
// A sweep across gear efficiency or pack stiffness is a legitimate test -- it is
// how you find out which of them a navigation failure was actually sensitive to
// -- and it must not cost eight firmware builds. Same reasoning as sim_mass
// beside it: these describe THIS robot, not the image.
//
// Cached behind a sentinel like the others, because integrate() reads them on
// every wheel on every control cycle.
//
// -1 is the "unset" marker, so the sentinel cannot collide with a real value:
// none of the three is meaningfully negative. A gear that returns negative
// torque, drag that accelerates, or a pack that gains voltage under load are all
// nonsense, and clamping here is cheaper than three checks in the hot path.
static inline float simGearEfficiency()
{
    static float eff = -1.0f;
    if (eff < 0.0f) {
        eff = envFloat("sim_gear_eff", (float)SIM_GEAR_EFFICIENCY);
        if (eff < 0.0f) eff = 0.0f;
        if (eff > 1.0f) eff = 1.0f;     // a gearbox cannot return more than it is given
    }
    return eff;
}

static inline float simCoulombRpm()
{
    static float drag = -1.0f;
    if (drag < 0.0f) {
        drag = envFloat("sim_coulomb", (float)SIM_WHEEL_COULOMB_RPM);
        if (drag < 0.0f) drag = 0.0f;
    }
    return drag;
}

static inline float simBattSag()
{
    static float sag = -1.0f;
    if (sag < 0.0f) {
        sag = envFloat("sim_sag", (float)SIM_BATT_SAG);
        if (sag < 0.0f) sag = 0.0f;
    }
    return sag;
}

// The fraction of its rated speed the motor can reach on the pack it is given.
// 1.0 when the pack matches the motor, which is every shipped reference.
static inline float simVoltageRatio()
{
    static float ratio = -1.0f;
    if (ratio < 0.0f) {
        const float op = envFloat("motor_v", (float)MOTOR_OPERATING_VOLTAGE);
        const float pw = envFloat("power_v", (float)MOTOR_POWER_MAX_VOLTAGE);
        ratio = (op > 0.0f) ? (pw / op) : 1.0f;
        if (ratio < 0.0f) ratio = 0.0f;
        if (ratio > 1.0f) ratio = 1.0f;   // a pack above the motor's rating does
                                          // not make it spin faster than rated
    }
    return ratio;
}

static inline float simSagTauMs()
{
    static float tau = -1.0f;
    if (tau < 0.0f) {
        tau = envFloat("sim_sag_tau", (float)SIM_BATT_SAG_TAU_MS);
        if (tau < 0.0f) tau = 0.0f;
    }
    return tau;
}

static inline float simDrvDrop()
{
    static float drop = -1.0f;
    if (drop < 0.0f) {
        drop = envFloat("sim_drv_drop", (float)SIM_DRV_DROP);
        if (drop < 0.0f) drop = 0.0f;
        if (drop > 0.9f) drop = 0.9f;   // a bridge that keeps everything is not a bridge
    }
    return drop;
}

static inline float simDrvR()
{
    static float r = -1.0f;
    if (r < 0.0f) {
        r = envFloat("sim_drv_r", (float)SIM_DRV_R);
        if (r < 0.0f) r = 0.0f;
    }
    return r;
}

static inline float simMotorStallA()
{
    static float a = -1.0f;
    if (a < 0.0f) {
        a = envFloat("sim_stall_a", (float)SIM_MOTOR_STALL_A);
        if (a < 0.0f) a = 0.0f;
    }
    return a;
}

// 0 is a real answer here ("no limiter"), so the sentinel has to be below it.
static inline float simDrvLimitA()
{
    static float a = -1.0f;
    if (a < 0.0f) {
        a = envFloat("sim_ilimit_a", (float)SIM_DRV_ILIMIT_A);
        if (a < 0.0f) a = 0.0f;
    }
    return a;
}

#ifndef SIM_WHEEL_NOISE_RPM
#define SIM_WHEEL_NOISE_RPM 1.0    // +/- peak white noise on the reported RPM
#endif

static inline float simWheelNoiseRpm()
{
    static float noise = -1.0f;
    if (noise < 0.0f)
        noise = envFloat("sim_noise_rpm", (float)SIM_WHEEL_NOISE_RPM);
    return noise;
}

// +/- peak white noise, scaled by amplitude
static inline float simWheelNoise(float amplitude)
{
    return ((float)random(-1000, 1001) / 1000.0) * amplitude;
}

// The variance of what simWheelNoise() produces: uniform on +/-peak, so
// var = peak^2 / 3. A simulated sensor is the one sensor whose noise is known
// exactly -- so it declares this, rather than a constant somebody has to
// remember to update alongside.
static inline constexpr float simCov(float peak) { return peak * peak / 3.0f; }

class SimEncoder : public EncoderInterface
{
private:
    int counts_per_rev_ = -1;
    float duty_ = 0.0;              // commanded duty cycle, -1.0 .. 1.0
    float wheel_rpm_ = 0.0;         // simulated wheel speed
    double ticks_ = 0.0;            // simulated tick accumulator
    unsigned long prev_update_time_ = 0;
    int slot_ = -1;                 // which wheel this is, for the shared pack

    // Every wheel's current demand, as a fraction of its own stall demand. The
    // pack is shared, so the sag one wheel causes is felt by all of them --
    // shared rather than per-instance for exactly that reason.
    //
    // A function-local static, not a static data member: this is a header-only
    // class, the ESP32 core compiles as gnu++11 so there are no inline
    // variables, and a static member would need an out-of-line definition in a
    // .cpp this library does not have. The file already uses this idiom for
    // simRobotMass() and says why -- these are read inside a class used before
    // setup() finishes, where a global's initialisation order is not safe.
    static float *demand()
    {
        static float d[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        return d;
    }

    // The pack's own filter state: how much sag has DEVELOPED, as against how
    // much the present current would cause. Shared, like demand(), because there
    // is one pack.
    static float &sagState()
    {
        static float s = 0.0f;
        return s;
    }
    static unsigned long &sagClock()
    {
        static unsigned long t = 0;
        return t;
    }

    // What the four wheels together are doing to the voltage the motors see, as
    // a scale on the no-load speed. 1.0 with the robot at rest.
    //
    // Three terms, and they are separate because they behave differently:
    //
    //   the pack      lags. Internal resistance drops the voltage at once but the
    //                 chemistry polarises over hundreds of ms, so a held load
    //                 sags deeper than a brief one. This is why a heavy robot
    //                 suffers more than its peak current suggests -- heavy means
    //                 drawing that current for LONGER.
    //   Rds(on)       follows the current instantly, so it uses the present
    //                 demand rather than the filtered one.
    //   the diode/Vce floor  is there at any current at all.
    //
    // Advanced from whichever wheel calls first each cycle: each call moves the
    // filter by the time since the last one, so four wheels advance it by one
    // cycle in total rather than four.
    static float busScale()
    {
        float inst = 0.0;
        for (int i = 0; i < 4; i++)
            inst += demand()[i];
        inst *= 0.25;                       // mean across the four wheels
        if (inst < 0.0) inst = 0.0;
        if (inst > 1.0) inst = 1.0;

        const unsigned long now = micros();
        unsigned long dt = now - sagClock();
        if (sagClock() == 0 || dt > 1000000UL) {
            // first call, or a micros() rollover: seed, do not step
            sagClock() = now;
            sagState() = inst;
        } else if (dt > 0) {
            sagClock() = now;
            const float tau_s = simSagTauMs() / 1000.0f;
            float k = 1.0f;                 // tau 0 -> instantaneous, the old behaviour
            if (tau_s > 0.0f) {
                k = ((float)dt / 1000000.0f) / tau_s;
                if (k > 1.0f) k = 1.0f;
            }
            sagState() += (inst - sagState()) * k;
        }

        float scale = (1.0f - simDrvDrop()) /
                      (1.0f + simBattSag() * sagState() + simDrvR() * inst);
        if (scale < 0.0f) scale = 0.0f;
        return scale;
    }

    // advance the wheel model to now
    void integrate()
    {
        unsigned long current_time = micros();
        unsigned long dt = current_time - prev_update_time_;
        prev_update_time_ = current_time;
        // first call, or a micros() rollover: seed the clock, don't step the model
        if (dt == 0 || dt > 1000000UL) return;
        float dts = (float)dt / 1000000.0;

        // heavier robot, more inertia per wheel, slower response
        float tau = (SIM_WHEEL_TAU_MS / 1000.0) *
                    (simRobotMass() / (float)SIM_WHEEL_REF_MASS);
        if (tau < 0.001) tau = 0.001;

        // The motor's own no-load speed, derated by the voltage it is actually
        // fed. A 12 V motor on a 9 V pack really does top out at 75% of its
        // rated speed, and Kinematics applies the same ratio when it works out
        // what to ask for (max_rpm_ = (power_v / operating_v) * motor_max_rpm *
        // ratio) -- so leaving it out here made the simulated motor faster than
        // the one the kinematics believes in.
        //
        // max_rpm_RATIO is deliberately NOT applied. That is a derating of the
        // COMMAND -- a margin Kinematics keeps so it never asks for the last
        // 15% -- not a property of the motor. The motor's no-load speed is what
        // it is; the controller simply declines to use all of it. Measured on
        // the bench at 3.5 kg: test_acc drives raw PWM, bypasses Kinematics and
        // reaches 135.6 rpm, where the controller would never request beyond
        // 119.
        float no_load_rpm = duty_ * (float)MOTOR_MAX_RPM * simVoltageRatio();
        // below the stall band the motor cannot hold the wheel against friction
        if (fabsf(duty_) < (float)SIM_WHEEL_STALL_DUTY) no_load_rpm = 0.0;

        // The pack sags under the current the four wheels are drawing, and the
        // no-load speed scales with the voltage that survives. Recorded BEFORE
        // the scale is applied, so the demand this wheel reports is the demand
        // it would make at full voltage -- otherwise the sag feeds back on
        // itself and the whole thing converges on nothing.
        const float stall = (float)MOTOR_MAX_RPM * simVoltageRatio();
        // The current this wheel is asking for, as a fraction of its own stall
        // current: for a brushed motor the two are proportional to the same
        // (no-load speed - present speed) the torque uses.
        const float ifrac = stall > 0.0f ? fabsf(no_load_rpm - wheel_rpm_) / stall : 0.0f;

        // What the driver will actually pass, as a fraction of that demand.
        // A limiter caps TORQUE, so it scales the driving term -- it does not
        // change the speed the motor is trying to reach, which is why this is a
        // separate factor rather than another bite out of no_load_rpm.
        float ilim_scale = 1.0f;
        const float limit_a = simDrvLimitA();
        if (limit_a > 0.0f) {
            const float want_a = ifrac * simMotorStallA();
            if (want_a > limit_a)
                ilim_scale = limit_a / want_a;
        }

        if (slot_ >= 0 && slot_ < 4) {
            // A limited driver also draws less from the pack, so it sags less.
            // Reporting the unlimited demand here would have the pack brown out
            // over current the driver is refusing to pass.
            demand()[slot_] = ifrac * ilim_scale;
        }
        no_load_rpm *= busScale();

        // back-EMF: driving torque is proportional to the remaining speed error,
        // and the gearbox returns only part of it
        float accel = ilim_scale * simGearEfficiency() * (no_load_rpm - wheel_rpm_) / tau;
        // viscous friction always opposes motion
        accel -= wheel_rpm_ * (float)SIM_WHEEL_FRICTION;
        // ...and the gear train's constant drag, which does not scale with speed.
        // Signed against motion, and never enough to drive the wheel backwards
        // through zero: that would be a gearbox pushing the robot.
        if (wheel_rpm_ > 0.0f)
            accel -= simCoulombRpm();
        else if (wheel_rpm_ < 0.0f)
            accel += simCoulombRpm();
        // traction and current limit the achievable acceleration
        if (accel > (float)SIM_WHEEL_MAX_ACCEL_RPM) accel = (float)SIM_WHEEL_MAX_ACCEL_RPM;
        if (accel < -(float)SIM_WHEEL_MAX_ACCEL_RPM) accel = -(float)SIM_WHEEL_MAX_ACCEL_RPM;

        const float was = wheel_rpm_;
        wheel_rpm_ += accel * dts;
        // Coulomb drag brakes; it must not become a motor. A wheel that crossed
        // zero in one step with no drive stops at zero instead of reversing.
        if (no_load_rpm == 0.0 && was != 0.0f && (was > 0.0f) != (wheel_rpm_ > 0.0f))
            wheel_rpm_ = 0.0;
        // an unpowered wheel settles rather than creeping forever
        if (no_load_rpm == 0.0 && fabsf(wheel_rpm_) < 0.5) wheel_rpm_ = 0.0;

        // accumulate ticks so read() stays consistent with getRPM()
        ticks_ += (double)wheel_rpm_ / 60.0 * counts_per_rev_ * dts;
    }

public:
    SimEncoder(int pin1, int pin2, int counts_per_rev, bool invert = false)
    {
        // Which wheel this is, in construction order, so the shared pack can be
        // told what each of them is drawing. A fifth encoder gets no slot and
        // simply contributes no sag rather than corrupting someone else's.
        static int next_slot = 0;
        if (next_slot < 4)
            slot_ = next_slot++;

        // The pins are deliberately ignored. Sim wheel mode exists for boards
        // with nothing wired, where the encoder pins are normally left unset
        // (-1); keying off them would leave every simulated wheel at 0 RPM,
        // which is the one case this class is for. Nothing here touches GPIO.
        // Which wheels actually count is the kinematics' decision, not the pin
        // map: Kinematics::getVelocities() zeroes motors 3 and 4 on a
        // differential base regardless of what the encoders report.
        (void)pin1;
        (void)pin2;
        // ...and so is `invert`, for the same reason.
        //
        // On a real robot the right-hand side is mirrored, so the motor driver
        // inverts what it drives and the encoder inverts what it reads. The two
        // cancel: the wheel turns forward and reports forward.
        //
        // In sim mode there is no motor. Pins are -1, so MotorInterface::spin()
        // returns without touching anything, and NOTHING applies the motor half
        // of that pair -- but feed() was still applying the encoder half. Wheel
        // 2 therefore ran backwards whenever wheel 1 ran forwards, and the
        // simulated robot spun on the spot instead of driving.
        //
        // Measured on a bare Pico 2, 2026-09-20: commanded (0.20, 0.00), odom
        // reported vx down to -0.459 m/s and wz to -4.869 rad/s; commanded
        // (0.00, 0.00) it still reported -0.6 m/s and -3.7 rad/s. Nav2 planned
        // a path and the base could never follow it -- the zero-wiring promise,
        // broken by a sign.
        (void)invert;
        counts_per_rev_ = (counts_per_rev > 0) ? counts_per_rev : 1;
    }

    // called by the control loop with the PWM just handed to the motor driver
    void feed(int pwm)
    {
        if (counts_per_rev_ < 0) return;
        integrate();
        // PWM_MAX expands to an unparenthesized expression (`pow(2, PWM_BITS) - 1`),
        // so it has to be wrapped before dividing or the `- 1` escapes the cast and
        // lands outside the division, turning a small duty into nearly full reverse.
        float duty = (float)pwm / (float)(PWM_MAX);
        if (duty > 1.0) duty = 1.0;
        if (duty < -1.0) duty = -1.0;
        duty_ = duty;
    }

    float getRPM()
    {
        if (counts_per_rev_ < 0) return 0.0;
        integrate();
        return wheel_rpm_ + simWheelNoise(simWheelNoiseRpm());
    }

    inline int32_t read()
    {
        if (counts_per_rev_ < 0) return 0;
        integrate();
        return (int32_t)ticks_;
    }

    inline void write(int32_t p)
    {
        if (counts_per_rev_ < 0) return;
        ticks_ = (double)p;
    }
};

#ifndef SIM_IMU_GRAVITY
#define SIM_IMU_GRAVITY 9.81       // specific force reported on Z when level
#endif

#ifndef SIM_IMU_ACCEL_TAU_MS
#define SIM_IMU_ACCEL_TAU_MS 60    // accelerometer band limit (ms)
#endif

// The simulated sensors are sized to a TYPICAL real one, not to whatever looked
// plausible. mcu_env.py carries datasheet variances for the parts this project
// supports (_imu_cov, _mag_cov); the medians of those tables are used here,
// which in each case is the figure of an actual shipping chip rather than an
// average of things nobody makes:
//
//     accel  5.1e-4 (m/s^2)^2   ICM20948      spread 4.7e-5 .. 1.8e-3
//     gyro   3.0e-6 (rad/s)^2   MPU6050/9250  spread 4.4e-7 .. 4.4e-5
//     mag    4.0e-14 T^2        QMC5883L      spread 2.3e-14 .. 9e-14
//
// simWheelNoise() is uniform on +/-peak, so peak = sigma*sqrt(3) and the
// variance the sensor DECLARES (simCov below) is the variance it actually has.
// Getting this wrong breaks the stack in a way that only shows up on hardware:
// too quiet and the EKF learns to trust an IMU nobody sells, too loud and
// tuning that works on the bench is wrong on a robot.
#ifndef SIM_IMU_ACCEL_NOISE
#define SIM_IMU_ACCEL_NOISE 0.03912 // +/- peak accel noise (m/s^2) = typical 5.1e-4 var
#endif

// A real MEMS IMU is not a clean derivative of the truth: it has a fixed bias,
// a bias that wanders slowly with temperature, a scale-factor error, and white
// noise on top. Fusion (madgwick, the EKF) exists to fight exactly that, so a
// perfect simulated IMU would make the whole estimation stack look better than
// it is on hardware.
#ifndef SIM_IMU_GYRO_BIAS
#define SIM_IMU_GYRO_BIAS 0.004f       // fixed gyro bias (rad/s)
#endif

#ifndef SIM_IMU_GYRO_DRIFT
#define SIM_IMU_GYRO_DRIFT 0.0015f     // gyro bias random walk (rad/s per sqrt(s))
#endif

#ifndef SIM_IMU_ACCEL_BIAS
#define SIM_IMU_ACCEL_BIAS 0.03f       // fixed accelerometer bias (m/s^2)
#endif

#ifndef SIM_IMU_ACCEL_DRIFT
#define SIM_IMU_ACCEL_DRIFT 0.01f      // accelerometer bias random walk (m/s^2 per sqrt(s))
#endif

#ifndef SIM_IMU_SCALE_ERROR
#define SIM_IMU_SCALE_ERROR 0.01f      // scale-factor error, fraction of reading
#endif

#ifndef SIM_MAG_FIELD_T
#define SIM_MAG_FIELD_T 50e-6f     // Simulated field strength (Tesla, ~Earth)
#endif

#ifndef SIM_MAG_ROOM_HEADING
#define SIM_MAG_ROOM_HEADING 0.0f  // Heading (rad) of the room's +X axis vs the field
#endif

#ifndef SIM_MAG_NOISE_T
#define SIM_MAG_NOISE_T 3.464e-7f  // +/- peak mag noise (T) = typical 4.0e-14 var
#endif

// Hard-iron offset. A magnetometer mounted on a robot always sits next to
// motors, batteries and steel, which add a fixed vector to every reading and
// pull the heading round with the robot. Simulating it means the magnetometer
// calibration routine has a real offset to discover and remove, instead of
// converging on zero and proving nothing.
#ifndef SIM_MAG_BIAS_X
#define SIM_MAG_BIAS_X 6.0e-6f
#endif
#ifndef SIM_MAG_BIAS_Y
#define SIM_MAG_BIAS_Y -4.0e-6f
#endif
#ifndef SIM_MAG_BIAS_Z
#define SIM_MAG_BIAS_Z 2.5e-6f
#endif

#ifndef SIM_IMU_GYRO_NOISE
#define SIM_IMU_GYRO_NOISE 0.003   // +/- peak gyro noise (rad/s) = typical 3.0e-6 var
#endif

// Derives IMU readings from the simulated body motion, so the accelerometer
// and gyroscope agree with the wheel encoders instead of reading zero.
// Hold a wandering bias inside a plausible envelope.
static inline float clampBias(float v, float limit)
{
    if (limit < 0.0f) limit = -limit;
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

class SimIMUFromWheels
{
private:
    float linear_x_ = 0.0;          // latest body velocities
    float linear_y_ = 0.0;
    float angular_z_ = 0.0;
    float accel_x_ = 0.0;           // smoothed body accelerations
    float accel_y_ = 0.0;
    float heading_ = 0.0;           // simulated yaw, for the magnetometer
    // slowly wandering sensor biases, seeded to a fixed offset and then walked
    // Start calibrated. A real IMU has its static bias measured and subtracted
    // at startup -- IMUInterface::init() calls calibrateGyro(), which averages
    // 40 samples and stores the offset. Sim wheel mode never calls that (there
    // is no chip to talk to), so seeding these with the full bias simulated an
    // IMU that had skipped its own calibration: the gyro read a steady offset
    // forever, the EKF integrated it, and yaw walked away from the wheels.
    float gyro_bias_z_ = 0.0f;
    float accel_bias_x_ = 0.0f;
    float accel_bias_y_ = 0.0f;

public:
    // With no real IMU on the bus, imu.getData() / mag.getData() are never
    // called, and the two messages never get the frame and covariances those
    // calls would have filled in. Set them once here instead: without a
    // frame_id the messages are dropped by tf, and with zero covariance the
    // EKF treats the simulated sensor as exact.
    void initMsgs(sensor_msgs__msg__Imu &imu_msg,
                  sensor_msgs__msg__MagneticField &mag_msg)
    {
        // The same env keys a real IMU uses. Sim mode publishes through this
        // class rather than IMUInterface, so without this the covariance a
        // config sets reached every robot EXCEPT the simulated one -- which is
        // the default here, and the one an EKF is usually tuned against first.
        //
        // The DEFAULT, though, is derived from this class's own noise rather
        // than the generic ACCEL_COV/GYRO_COV/MAG_COV placeholders: a simulated
        // sensor is the one sensor whose noise is known exactly, so restating
        // it in a second constant only creates something to drift. The
        // placeholders said 1e-5 while the accelerometer produced 5.1e-4, and
        // the EKF was told the simulated IMU was 50x quieter than it was.
        float accel_cov[3] = {simCov(SIM_IMU_ACCEL_NOISE), simCov(SIM_IMU_ACCEL_NOISE),
                              simCov(SIM_IMU_ACCEL_NOISE)};
        float gyro_cov[3] = {simCov(SIM_IMU_GYRO_NOISE), simCov(SIM_IMU_GYRO_NOISE),
                             simCov(SIM_IMU_GYRO_NOISE)};
        float ori_cov[3] = ORI_COV;
        float mag_cov[3] = {simCov(SIM_MAG_NOISE_T), simCov(SIM_MAG_NOISE_T),
                            simCov(SIM_MAG_NOISE_T)};
        envFloatVec("accel_cov", accel_cov, 3);
        envFloatVec("gyro_cov", gyro_cov, 3);
        envFloatVec("ori_cov", ori_cov, 3);
        envFloatVec("mag_cov", mag_cov, 3);

        // initMsgs() runs from setup(), so the env is readable and the robot's
        // namespace goes on here. Sim mode is the DEFAULT on a bare module,
        // so this is the path a two-robot bench actually exercises.
        imu_msg.header.frame_id =
            micro_ros_string_utilities_set(imu_msg.header.frame_id, envPrefixed("imu_link"));
        mag_msg.header.frame_id =
            micro_ros_string_utilities_set(mag_msg.header.frame_id, envPrefixed("imu_link"));

        for (int i = 0; i < 3; i++)
        {
            const int d = i * 4;    // 0, 4, 8: the diagonal of a 3x3 row-major
            imu_msg.linear_acceleration_covariance[d] = accel_cov[i];
            imu_msg.angular_velocity_covariance[d] = gyro_cov[i];
            imu_msg.orientation_covariance[d] = ori_cov[i];
            mag_msg.magnetic_field_covariance[d] = mag_cov[i];
        }
    }

    // fed from the body velocities the kinematics derived from the wheels
    void update(float linear_x, float linear_y, float angular_z, float dt)
    {
        if (dt > 0.0)
        {
            // differencing velocity at the loop rate is spiky; a real
            // accelerometer is band limited, so ease into the new value
            float raw_x = (linear_x - linear_x_) / dt;
            float raw_y = (linear_y - linear_y_) / dt;
            float alpha = 1.0 - expf(-dt / (SIM_IMU_ACCEL_TAU_MS / 1000.0));
            accel_x_ += (raw_x - accel_x_) * alpha;
            accel_y_ += (raw_y - accel_y_) * alpha;

            // Random walk: the step scales with sqrt(dt), so the drift rate is
            // independent of how often this happens to be called.
            //
            // Bounded, because a free random walk has no bound and this one had
            // none: the longer the board ran, the further the bias wandered,
            // and it does not come back. Measured after a long session the
            // stationary gyro read 0.01813 rad/s -- 4.5x the nominal bias --
            // and the EKF turned that into 0.87 deg/s of yaw drift, 52 deg in a
            // minute while the robot stood still. Real parts do not do that:
            // bias instability wanders within an envelope. Clamping to the
            // nominal bias keeps the drift a filter has to cope with, without
            // letting uptime decide whether SLAM works.
            const float rw = sqrtf(dt);
            gyro_bias_z_  += simWheelNoise((float)SIM_IMU_GYRO_DRIFT) * rw;
            accel_bias_x_ += simWheelNoise((float)SIM_IMU_ACCEL_DRIFT) * rw;
            accel_bias_y_ += simWheelNoise((float)SIM_IMU_ACCEL_DRIFT) * rw;
            gyro_bias_z_  = clampBias(gyro_bias_z_,  (float)SIM_IMU_GYRO_BIAS);
            accel_bias_x_ = clampBias(accel_bias_x_, (float)SIM_IMU_ACCEL_BIAS);
            accel_bias_y_ = clampBias(accel_bias_y_, (float)SIM_IMU_ACCEL_BIAS);
        }
        linear_x_ = linear_x;
        linear_y_ = linear_y;
        angular_z_ = angular_z;
    }

    // The simulated robot turns, so a magnetometer stuck at a constant vector
    // would disagree with the yaw the wheels report and drag any heading fusion
    // (madgwick, EKF) away from the truth. Rotate a fixed world field into the
    // body frame instead, so the mag agrees with the simulated room: the field
    // points along the room's +X axis, offset by SIM_MAG_ROOM_HEADING.
    void setHeading(float heading) { heading_ = heading; }

    void applyMag(sensor_msgs__msg__MagneticField &mag_msg)
    {
        const float theta = heading_ - (float)SIM_MAG_ROOM_HEADING;
        const float b = (float)SIM_MAG_FIELD_T;
        // The world field points along +Y, i.e. North in the ENU frame ROS uses,
        // because that is the direction imu_filter_madgwick assumes when it
        // derives heading from the magnetometer. Pointing it along +X instead
        // is physically just as valid but leaves the fused yaw a fixed ~90 deg
        // from the wheel odometry's, and the EKF then has two heading sources
        // that disagree by a quarter turn: it splits the difference, drags the
        // pose sideways during a pure rotation, and the SLAM map comes out
        // sheared. Measured before this: /odom translated 3.9 m while the robot
        // only spun in place.
        //
        // Rotating the world vector (0, b) into the body frame by -theta:
        //   x =  b sin(theta)      y =  b cos(theta)
        // then spoiled by the hard-iron offset calibration is meant to find,
        // and white noise.
        mag_msg.magnetic_field.x =
            b * sinf(theta) + (float)SIM_MAG_BIAS_X + simWheelNoise((float)SIM_MAG_NOISE_T);
        mag_msg.magnetic_field.y =
            b * cosf(theta) + (float)SIM_MAG_BIAS_Y + simWheelNoise((float)SIM_MAG_NOISE_T);
        mag_msg.magnetic_field.z =
            (float)SIM_MAG_BIAS_Z + simWheelNoise((float)SIM_MAG_NOISE_T);
    }

    void apply(sensor_msgs__msg__Imu &imu_msg)
    {
        // a real accelerometer measures specific force: body accel, plus the
        // centripetal term while turning, plus gravity held up by the floor
        // true specific force, then spoiled the way a real sensor spoils it:
        // scale error on the signal, a wandering bias, then white noise
        const float k = 1.0f + (float)SIM_IMU_SCALE_ERROR;
        const float ax = accel_x_ - angular_z_ * linear_y_;
        const float ay = accel_y_ + angular_z_ * linear_x_;

        imu_msg.linear_acceleration.x =
            ax * k + accel_bias_x_ + simWheelNoise((float)SIM_IMU_ACCEL_NOISE);
        imu_msg.linear_acceleration.y =
            ay * k + accel_bias_y_ + simWheelNoise((float)SIM_IMU_ACCEL_NOISE);
        imu_msg.linear_acceleration.z =
            (float)SIM_IMU_GRAVITY + simWheelNoise((float)SIM_IMU_ACCEL_NOISE);

        imu_msg.angular_velocity.x = simWheelNoise((float)SIM_IMU_GYRO_NOISE);
        imu_msg.angular_velocity.y = simWheelNoise((float)SIM_IMU_GYRO_NOISE);
        imu_msg.angular_velocity.z =
            angular_z_ * k + gyro_bias_z_ + simWheelNoise((float)SIM_IMU_GYRO_NOISE);
    }
};


// No `#define ENCODER` either: createEncoder() picks SimEncoder or Encoder
// from `sim_wheel` in the env, per board, at boot.

#endif
