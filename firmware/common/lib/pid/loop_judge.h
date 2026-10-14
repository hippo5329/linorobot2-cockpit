// The closed-loop verdict of one wheel step: test_acc's and the base app's IDENT loop lines.
//
// It judged each 20 ms tick's RPM against a 2 % band. One encoder count per tick is
// 60 / (cpr * 0.02) RPM -- 3.4 RPM on an 887-count wheel, 13 % of a crawl's 26 RPM and 2.9 %
// of 117 RPM -- so every gain set read "settle never, ~35 crossings": the count flickering
// across the setpoint, not the loop (2026-10-14: 0.6/0.8/0.5 and 2.8/1.4/0 printed the same
// numbers, while steps timed by the encoder distance told them apart at once).
//
// So the speed is averaged over WIN ticks (100 ms: the counts over 100 ms, one count = a fifth
// of a tick's step), the band is never finer than that average can resolve, and a crossing is
// counted only when the speed leaves the band on the OTHER side from where it last left it:
// one overshoot that comes back inside is 1, ringing is 2 or more. Five floats a wheel; no
// per-tick trace is kept.
#ifndef LOOP_JUDGE_H
#define LOOP_JUDGE_H

#include <math.h>

struct LoopJudge {
    static const unsigned WIN = 5;          // ticks averaged: 100 ms at the 50 Hz control rate

    float sp = 0.0f, band = 0.0f, peak = 0.0f, last = 0.0f, err_sum = 0.0f;
    float win[WIN] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    unsigned seen = 0, settle_tick = 0, err_n = 0;
    int crossings = 0, side = 0;            // side: -1 short of the band, +1 past it, 0 none yet

    // res_rpm: what the WIN-tick average can resolve, 60 / (cpr * WIN * tick_s); 0 = unknown.
    void begin(float setpoint, float res_rpm)
    {
        *this = LoopJudge();
        sp = setpoint;
        band = fabsf(setpoint) * 0.02f;
        if (res_rpm > band) band = res_rpm;
    }

    // One tick's RPM, at tick t of a step `ticks` long. The steady error is the raw mean over
    // the last 25 ticks (0.5 s): averaging removes the flicker, not the bias.
    void add(unsigned t, unsigned ticks, float rpm)
    {
        last = rpm;
        if (t + 25 >= ticks) { err_sum += sp - rpm; err_n++; }
        win[seen % WIN] = rpm;
        seen++;
        if (seen < WIN) { settle_tick = t + 1; return; }
        float avg = 0.0f;
        for (unsigned k = 0; k < WIN; k++) avg += win[k];
        avg /= (float)WIN;
        if (fabsf(avg) > peak) peak = fabsf(avg);
        const float past = sp >= 0.0f ? avg - sp : sp - avg;     // > 0: past the setpoint
        const int s = past > band ? 1 : (past < -band ? -1 : 0);
        if (s != 0) {
            if (side != 0 && s != side) crossings++;
            side = s;
            settle_tick = t + 1;
        }
    }

    float overshoot() const { return sp != 0.0f ? (peak - fabsf(sp)) / fabsf(sp) : 0.0f; }
    float err() const { return err_n ? err_sum / (float)err_n : 0.0f; }
    // ms, from the step's start; -1 = never inside the band to the end. The average lags the
    // speed by half its window, which is taken back off.
    int settleMs(unsigned ticks, unsigned tick_ms) const
    {
        if (settle_tick >= ticks) return -1;
        const unsigned lag = WIN / 2;
        return (int)((settle_tick > lag ? settle_tick - lag : 0) * tick_ms);
    }
};

// What the average can resolve, RPM: one count over WIN ticks.
inline float loopJudgeResRpm(int cpr, unsigned tick_ms)
{
    if (cpr <= 0 || tick_ms == 0) return 0.0f;
    return 60.0f / ((float)cpr * (float)LoopJudge::WIN * (float)tick_ms * 0.001f);
}

#endif // LOOP_JUDGE_H
