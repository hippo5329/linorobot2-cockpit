// Copyright (c) 2026 Linorobot contributors
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
#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "base_ident.h"
#include "mcu_env.h"
#include "ota.h"
#include "syslog.h"

// See base_ident.h. Everything here runs inside one 20 ms control tick: no delay(), no
// loop that waits -- a step is a count of ticks, and its results are kept in the static
// state below until it ends.

namespace {

const unsigned TICK_MS = 20;            // the base's control timer

enum Kind : uint8_t { K_REST, K_DEADZONE, K_PLANT, K_LOOP, K_LIN, K_SPIN };

struct Step {
    Kind kind;
    int8_t dir;          // +1 forward / left, -1 back / right
    float level;         // loop: fraction of top rpm; runs: fraction of full PWM
    uint16_t ticks;
};

// The whole run, out and back at every step. ~95 s.
const Step STEPS[] = {
    {K_DEADZONE, +1, 0.0f, 600}, {K_REST, 0, 0.0f, 25},
    {K_DEADZONE, -1, 0.0f, 600}, {K_REST, 0, 0.0f, 25},
    {K_PLANT, +1, 1.0f, 30},     {K_REST, 0, 0.0f, 35},
    {K_PLANT, -1, 1.0f, 30},     {K_REST, 0, 0.0f, 35},
    {K_LOOP, +1, 0.20f, 90},     {K_REST, 0, 0.0f, 35},
    {K_LOOP, -1, 0.20f, 90},     {K_REST, 0, 0.0f, 35},
    {K_LOOP, +1, 0.55f, 90},     {K_REST, 0, 0.0f, 35},
    {K_LOOP, -1, 0.55f, 90},     {K_REST, 0, 0.0f, 35},
    {K_LOOP, +1, 0.90f, 90},     {K_REST, 0, 0.0f, 35},
    {K_LOOP, -1, 0.90f, 90},     {K_REST, 0, 0.0f, 35},
    {K_LIN, +1, 1.0f, 50},  {K_REST, 0, 0.0f, 50}, {K_LIN, -1, 1.0f, 50},  {K_REST, 0, 0.0f, 50},
    {K_SPIN, +1, 0.5f, 50}, {K_REST, 0, 0.0f, 50}, {K_SPIN, -1, 0.5f, 50}, {K_REST, 0, 0.0f, 50},
    {K_LIN, +1, 0.5f, 50},  {K_REST, 0, 0.0f, 50}, {K_LIN, -1, 0.5f, 50},  {K_REST, 0, 0.0f, 50},
    {K_SPIN, +1, 0.25f, 50},{K_REST, 0, 0.0f, 50}, {K_SPIN, -1, 0.25f, 50},{K_REST, 0, 0.0f, 50},
    {K_LIN, +1, 0.25f, 50}, {K_REST, 0, 0.0f, 50}, {K_LIN, -1, 0.25f, 50}, {K_REST, 0, 0.0f, 50},
};
const unsigned N_STEPS = sizeof(STEPS) / sizeof(STEPS[0]);
const unsigned LOOP_PRE = 15;            // ticks of PID-to-zero before a loop step

bool active = false;
unsigned step = 0, tick = 0;

// per-step results
int dz_found[4];
float plant[4][30];
float lp_peak[4], lp_last[4], lp_err[4];
int lp_cross[4];
unsigned lp_settle[4];
bool lp_below[4];
float run_peak_v, run_peak_w, run_vel[50];

float topRpm(void)
{
    return (float)envU16("max_rpm", MOTOR_MAX_RPM) * envFloat("rpm_ratio", MAX_RPM_RATIO);
}

void finish(int pwm[4], const char *why)
{
    for (int i = 0; i < 4; i++) pwm[i] = 0;
    active = false;
    if (why)
        syslog(LOG_INFO, "IDENT aborted: %s (step %u of %u)", why, step + 1, N_STEPS);
    else
        syslog(LOG_INFO, "IDENT done");
}

void beginStep(void)
{
    tick = 0;
    for (int i = 0; i < 4; i++) {
        dz_found[i] = -1;
        lp_peak[i] = lp_last[i] = lp_err[i] = 0.0f;
        lp_cross[i] = 0;
        lp_settle[i] = 0;
        lp_below[i] = true;
    }
    run_peak_v = run_peak_w = 0.0f;
}

// What a step reports when it ends -- test_acc's lines, word for word.
void report(const Step &s, unsigned motors, int pwm_max, Kinematics *kin)
{
    const bool fwd = s.dir > 0;
    switch (s.kind) {
    case K_DEADZONE:
        for (unsigned i = 0; i < motors; i++)
            syslog(LOG_INFO, "IDENT %s wheel=%u pwm=%d duty=%.3f", fwd ? "deadzone" : "deadzone_rev",
                   i + 1, dz_found[i], dz_found[i] < 0 ? -1.0 : (float)dz_found[i] / (float)pwm_max);
        break;
    case K_PLANT:
        for (unsigned i = 0; i < motors; i++) {
            const float steady = plant[i][29];
            float peak = 0.0f;
            unsigned tau = 0;
            for (unsigned t = 0; t < 30; t++) if (plant[i][t] > peak) peak = plant[i][t];
            for (unsigned t = 0; t < 30; t++) if (plant[i][t] >= steady * 0.632f) { tau = t; break; }
            syslog(LOG_INFO, "IDENT %s wheel=%u steady_rpm=%.1f tau_ms=%u K=%.5f peak_rpm=%.1f",
                   fwd ? "plant" : "plant_rev", i + 1, steady, tau * TICK_MS,
                   pwm_max > 0 ? steady / (float)pwm_max : 0.0f, peak);
        }
        break;
    case K_LOOP: {
        const float sp = s.dir * s.level * topRpm();
        const unsigned n = s.ticks - LOOP_PRE;
        for (unsigned i = 0; i < motors; i++)
            syslog(LOG_INFO, "IDENT %s wheel=%u sp=%.1f overshoot=%.3f crossings=%d settle_ms=%d err=%.2f final=%.1f",
                   fwd ? "loop" : "loop_rev", i + 1, sp, (lp_peak[i] - fabsf(sp)) / fabsf(sp), lp_cross[i],
                   lp_settle[i] >= n ? -1 : (int)(lp_settle[i] * TICK_MS), lp_err[i] / 25.0f, lp_last[i]);
        break;
    }
    case K_LIN:
    case K_SPIN: {
        const bool lin = s.kind == K_LIN;
        const float peak = lin ? run_peak_v : run_peak_w;
        unsigned t90 = 0;
        for (unsigned t = 0; t < 50; t++) if (fabsf(run_vel[t]) >= 0.9f * peak) { t90 = t; break; }
        const float p = s.level * pwm_max;
        syslog(LOG_INFO, "MAX PWM %6.1f %6.1f", p, -p);
        if (lin) syslog(LOG_INFO, "MAX VEL %6.2f %6.2f m/s  %6.2f rad/s", s.dir * run_peak_v, 0.0, 0.0);
        else     syslog(LOG_INFO, "MAX VEL %6.2f %6.2f m/s  %6.2f rad/s", 0.0, 0.0, s.dir * run_peak_w);
        syslog(LOG_INFO, "time to 0.9x max vel %6.2f sec", t90 * TICK_MS / 1000.0);
        (void)kin;
        break;
    }
    default:
        break;
    }
}

}  // namespace

bool baseIdentActive(void) { return active; }

bool baseIdentTick(const float cur[4], int pwm[4], PID *const pids[4], Kinematics *kin,
                   unsigned motors, int pwm_max, bool operator_cmd, bool sim_wheels)
{
    if (!active) {
        if (!hostIdentRequested())
            return false;
        if (sim_wheels) {
            syslog(LOG_INFO, "IDENT refused: the env says these wheels are simulated (sim_wheel=1)");
            return false;
        }
        if (operator_cmd) {
            syslog(LOG_INFO, "IDENT refused: /cmd_vel is driving the robot");
            return false;
        }
        hostStopClear();          // a stop that ended the last one must not end this one
        active = true;
        step = 0;
        beginStep();
        syslog(LOG_INFO, "IDENT gains kp=%.4f ki=%.4f kd=%.4f pwm_max=%d rate_hz=%u",
               envFloat("kp", K_P), envFloat("ki", K_I), envFloat("kd", K_D), pwm_max, 1000 / TICK_MS);
        syslog(LOG_INFO, "IDENT robot base=%d wheels=%u max_rpm=%d ratio=%.3f wheel_d=%.4f",
               (int)kin->base_platform_, motors, (int)envU16("max_rpm", MOTOR_MAX_RPM),
               envFloat("rpm_ratio", MAX_RPM_RATIO), envFloat("wheel_d", WHEEL_DIAMETER));
        syslog(LOG_INFO, "IDENT start: base mode, %u steps (stop: lino-stop, or any /cmd_vel)", N_STEPS);
    }
    if (hostStopRequested()) { finish(pwm, "stop from the host"); return true; }
    if (operator_cmd)        { finish(pwm, "/cmd_vel took over"); return true; }

    const Step &s = STEPS[step];
    for (int i = 0; i < 4; i++) pwm[i] = 0;
    switch (s.kind) {
    case K_REST:
        break;
    case K_DEADZONE: {
        // 1 % of full PWM more every 200 ms; a wheel has found its dead zone when its
        // encoder moves in the second half of that interval (the first carries the step)
        const int level = (int)(tick / 10) * (pwm_max / 100);
        bool all = true;
        for (unsigned i = 0; i < motors; i++) {
            if (dz_found[i] < 0 && (tick % 10) >= 5 && fabsf(cur[i]) > 2.0f) dz_found[i] = level;
            if (dz_found[i] < 0) all = false;
        }
        if (all || level > pwm_max / 2) { tick = s.ticks - 1; break; }   // ends this tick
        for (unsigned i = 0; i < motors; i++) pwm[i] = s.dir * level;
        break;
    }
    case K_PLANT:
        for (unsigned i = 0; i < motors; i++) {
            if (tick > 0) plant[i][tick - 1] = fabsf(cur[i]);   // the rpm of the PREVIOUS tick's command
            pwm[i] = s.dir * pwm_max;
        }
        if (tick == s.ticks - 1)
            for (unsigned i = 0; i < motors; i++) plant[i][29] = fabsf(cur[i]);
        break;
    case K_LOOP: {
        const float sp = s.dir * s.level * topRpm();
        if (tick < LOOP_PRE) {
            for (unsigned i = 0; i < motors; i++) pwm[i] = (int)pids[i]->compute(0.0f, cur[i]);
            break;
        }
        const unsigned t = tick - LOOP_PRE, n = s.ticks - LOOP_PRE;
        for (unsigned i = 0; i < motors; i++) {
            const float rpm = cur[i];
            pwm[i] = (int)pids[i]->compute(sp, rpm);
            if (fabsf(rpm) > lp_peak[i]) lp_peak[i] = fabsf(rpm);
            const bool below = rpm < sp;
            if (t > 0 && below != lp_below[i]) lp_cross[i]++;
            lp_below[i] = below;
            if (fabsf(rpm - sp) > fabsf(sp) * 0.02f) lp_settle[i] = t + 1;
            if (t >= n - 25) lp_err[i] += (sp - rpm);
            lp_last[i] = rpm;
        }
        break;
    }
    case K_LIN:
    case K_SPIN: {
        const int p = (int)(s.level * pwm_max);
        for (unsigned i = 0; i < motors; i++) {
            // a spin: the left wheels (1, 3) one way, the right (2, 4) the other -- left first
            const int side = (s.kind == K_SPIN && (i % 2 == 0)) ? -1 : 1;
            pwm[i] = s.dir * side * p;
        }
        const Kinematics::velocities v = kin->getVelocities(cur[0], cur[1], cur[2], cur[3]);
        const float val = s.kind == K_LIN ? v.linear_x : v.angular_z;
        if (tick < 50) run_vel[tick] = val;
        if (s.kind == K_LIN && fabsf(val) > run_peak_v) run_peak_v = fabsf(val);
        if (s.kind == K_SPIN && fabsf(val) > run_peak_w) run_peak_w = fabsf(val);
        break;
    }
    }
    if (++tick >= s.ticks) {
        report(s, motors, pwm_max, kin);
        if (s.kind == K_LOOP)
            for (unsigned i = 0; i < motors; i++) pids[i]->compute(0.0f, 0.0f);   // unwind: the next step starts calm
        if (++step >= N_STEPS) { finish(pwm, NULL); return true; }
        beginStep();
    }
    return true;
}
