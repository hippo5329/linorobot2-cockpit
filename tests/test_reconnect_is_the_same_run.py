"""A session rebuilt after a blip is the same run; only a new run resets the simulated pose.

Gate 20260926-disp7, GenDrv serial lyrical: the firmware declared the agent lost
on ONE failed 100 ms ping, rebuilt its session twice in five seconds, and each
new session put the simulated pose back at the origin -- from 3 m out, in the
middle of a Nav2 goal. The EKF coasted to -0.98 m/s through the gap, SLAM jumped
2.3 m, and the goal aborted with a plan of 0 poses. 23 of 246 GenDrv serial
bringups since 2026-09-22 had such a mid-run reconnect; this one landed in a goal.
"""
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


MAIN = read("firmware", "src", "main.cpp")


def const_ms(name):
    m = re.search(rf"static const uint32_t {name} = (\d+);", MAIN)
    assert m, f"{name} is gone from main.cpp"
    return int(m.group(1))


def test_one_lost_ping_is_not_a_lost_agent():
    connected = MAIN[MAIN.index("case AGENT_CONNECTED:"):MAIN.index("case AGENT_DISCONNECTED:")]
    assert "state = ok ? AGENT_CONNECTED : AGENT_DISCONNECTED" not in connected
    assert "millis() - ping_fail_since_ms >= AGENT_LOSS_MS" in connected
    assert const_ms("AGENT_LOSS_MS") >= 1000


def test_only_a_new_run_resets_the_simulated_pose():
    create = MAIN[MAIN.index("bool createEntities()\n{"):MAIN.index("bool destroyEntities()\n{")]
    reset = create.index("odometry->reset();")
    guard = create.rindex("if (", 0, reset)
    assert "!had_session || gone_ms >= SIM_POSE_RESET_AFTER_MS" in create[guard:reset]
    assert "had_session = true;" in create[reset:]


def test_the_pipelines_pose_reset_outlasts_the_threshold():
    """The pipeline restarts the bringup to get a new run; if the agent came back
    inside the firmware's threshold, the new session would keep the pose."""
    pipe = read("scripts", "one_click_pipeline.py")
    m = re.search(r"^SIM_POSE_RESET_AFTER_S = ([0-9.]+)$", pipe, re.M)
    assert m and float(m.group(1)) * 1000 == const_ms("SIM_POSE_RESET_AFTER_MS"), \
        "the pipeline and the firmware disagree about what a new run is"
    step = pipe[pipe.index("[4.7/6] [POSE] The robot is at"):]
    wait = step.index("SIM_POSE_RESET_AFTER_S + 1.0 - (time.time() - t_gone)")
    assert wait < step.index('log_tag="bringup2"'), "the wait must come before the relaunch"
