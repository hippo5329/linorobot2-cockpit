"""The IDENT loop verdict (firmware/common/lib/pid/loop_judge.h), on the wheel it was wrong for.

test_acc judged each 20 ms tick's RPM against a 2 % band. On a mecanum robot's 887-count wheels one
count per tick is 3.4 RPM, wider than the band at every setpoint it used, so two gain sets
far apart (0.6/0.8/0.5 and 2.8/1.4/0) both printed "settle never, ~35 crossings, RINGS",
while steps timed by the encoder distance told them apart at once (2026-10-14).

Behavioural: the real header is compiled and fed a quantised wheel, so a judge that goes back
to single ticks fails here, not on the floor.
"""
import ctypes
import math
import os
import shutil
import subprocess

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PID_DIR = os.path.join(REPO_ROOT, "firmware", "common", "lib", "pid")
ACC = os.path.join(REPO_ROOT, "firmware", "src", "tools", "test_acc.cpp")
BASE = os.path.join(REPO_ROOT, "firmware", "src", "base_ident.cpp")

SHIM = """
#include "loop_judge.h"
extern "C" {
float judge_res(int cpr, unsigned tick_ms) { return loopJudgeResRpm(cpr, tick_ms); }
// out: overshoot, crossings, settle_ms, err, band
void judge_run(float sp, float res, const float *rpm, unsigned n, float *out) {
    LoopJudge j;
    j.begin(sp, res);
    for (unsigned t = 0; t < n; t++) j.add(t, n, rpm[t]);
    out[0] = j.overshoot(); out[1] = (float)j.crossings; out[2] = (float)j.settleMs(n, 20);
    out[3] = j.err(); out[4] = j.band;
}
}
"""

CPR = 887
TICKS = 75          # 1.5 s, test_acc's loop step


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no C++ compiler")
    d = tmp_path_factory.mktemp("judge")
    (d / "shim.cpp").write_text(SHIM)
    so = d / "libjudge.so"
    subprocess.run([cxx, "-std=c++11", "-shared", "-fPIC", "-O1", "-I", PID_DIR,
                    str(d / "shim.cpp"), "-o", str(so)], check=True)
    lb = ctypes.CDLL(str(so))
    lb.judge_res.restype = ctypes.c_float
    lb.judge_res.argtypes = [ctypes.c_int, ctypes.c_uint]
    lb.judge_run.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float),
                             ctypes.c_uint, ctypes.POINTER(ctypes.c_float)]
    return lb


def judge(lib, sp, speeds):
    arr = (ctypes.c_float * len(speeds))(*speeds)
    out = (ctypes.c_float * 5)()
    lib.judge_run(sp, lib.judge_res(CPR, 20), arr, len(speeds), out)
    return {"overshoot": out[0], "crossings": int(out[1]), "settle_ms": int(out[2]),
            "err": out[3], "band": out[4]}


def quantised(true_rpm):
    """What getRPM() reads: whole counts per 20 ms tick, the remainder carried."""
    out, acc = [], 0.0
    for v in true_rpm:
        acc += v / 60.0 * CPR * 0.02
        n = math.floor(acc)
        acc -= n
        out.append(n * 60.0 / (CPR * 0.02))
    return out


def first_order(sp, tau=0.04):
    return [sp * (1 - math.exp(-(t + 1) * 0.02 / tau)) for t in range(TICKS)]


def jittered(true_rpm, seed=7, jitter=0.15):
    """getRPM() on a real tick: whole counts over an interval that wanders (+-15 %), read as
    counts over the NOMINAL 20 ms -- the flicker a crawl showed on the robot."""
    import random
    rnd = random.Random(seed)
    out, acc = [], 0.0
    for v in true_rpm:
        dt = 0.02 * (1 + rnd.uniform(-jitter, jitter))
        acc += v / 60.0 * CPR * dt
        n = math.floor(acc)
        acc -= n
        out.append(n * 60.0 / (CPR * dt))
    return out


def test_a_calm_loop_on_a_coarse_encoder_is_calm(lib):
    for sp in (25.9, 71.3, 116.6):
        raw = quantised(first_order(sp))
        assert len(set(round(v, 1) for v in raw[20:])) > 1, "the reading must flicker"
        got = judge(lib, sp, raw)
        assert got["crossings"] == 0, (sp, got)
        assert 0 <= got["settle_ms"] <= 300, (sp, got)
        assert abs(got["err"]) < 0.02 * sp + 0.7, (sp, got)


def test_a_calm_crawl_with_tick_jitter_is_not_ringing(lib):
    """One count of the average is enough: flicker and tick jitter alone never cross it, so a
    crawl that does cross it twice is a limit cycle worth printing as RINGS."""
    for seed in range(5):
        got = judge(lib, 25.9, jittered(first_order(25.9), seed=seed))
        assert got["crossings"] <= 1 and got["settle_ms"] >= 0, (seed, got)


def test_the_band_is_never_finer_than_the_average_can_resolve(lib):
    res = lib.judge_res(CPR, 20)
    assert abs(res - 60.0 / (CPR * 0.1)) < 1e-3
    assert abs(judge(lib, 25.9, quantised(first_order(25.9)))["band"] - res) < 1e-3
    assert abs(judge(lib, 116.6, quantised(first_order(116.6)))["band"] - 0.02 * 116.6) < 1e-3


def _damped(sp, zeta, wn):
    wd = wn * math.sqrt(1 - zeta * zeta)
    return [sp * (1 - math.exp(-zeta * wn * t) * (math.cos(wd * t) + zeta / math.sqrt(1 - zeta * zeta)
                                                    * math.sin(wd * t)))
            for t in (k * 0.02 for k in range(1, TICKS + 1))]


def test_a_ringing_loop_still_rings(lib):
    got = judge(lib, 71.3, quantised(_damped(71.3, 0.08, 12.0)))
    assert got["crossings"] >= 2, got
    assert got["overshoot"] > 0.3, got


def test_one_overshoot_that_settles_is_one_crossing(lib):
    got = judge(lib, 71.3, quantised(_damped(71.3, 0.6, 25.0)))
    assert got["crossings"] == 1, got
    assert got["settle_ms"] >= 0, got


def test_reverse_steps_are_judged_the_same_way(lib):
    raw = [-v for v in quantised(first_order(71.3))]
    got = judge(lib, -71.3, raw)
    assert got["crossings"] == 0 and got["settle_ms"] >= 0, got


def test_both_judges_use_it_and_print_the_band():
    for path in (ACC, BASE):
        src = open(path, encoding="utf-8").read()
        assert '#include "loop_judge.h"' in src, path
        assert "loopJudgeResRpm(encoderCpr(" in src, path
        assert "band=%.2f" in src, path


def test_the_host_reads_the_new_lines_by_their_band():
    import sys
    sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
    import drivetrain_report as dr
    new = {"settle_ms": 300.0, "crossings": 0.0, "overshoot": 0.04, "band": 1.43}
    assert dr.loop_verdict(new) == "calm"
    assert dr.loop_verdict(dict(new, crossings=1.0)) == "overshoots"
    assert dr.loop_verdict(dict(new, crossings=3.0)) == "RINGS"
    assert dr.loop_verdict(dict(new, settle_ms=-1.0)) == "RINGS"
    old = {"settle_ms": 1460.0, "crossings": 35.0, "overshoot": 0.065}
    assert dr.loop_verdict(old) == "RINGS"
