"""slam_toolbox stuck in `inactive` is activated until it is active, and reported as it was.

On a real robot (lyrical, 2026-10-11) slam_toolbox missed its launch file's transition event,
the activation guard found it inactive and made ONE `lifecycle set activate` call, which met
the same discovery race ("Node not found"). SLAM stayed inactive, no map came, and Nav2's
planner_server never configured. The pipeline then reported "slam_toolbox reached:
Configuring -> Activating", because the guard's own "... Activating it." line held the word.
"""
import importlib.util
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _ocp():
    spec = importlib.util.spec_from_file_location("ocp_slam_act", os.path.join(ROOT, "scripts", "one_click_pipeline.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_guard_asks_until_slam_is_active():
    src = open(os.path.join(ROOT, "launchers", "slam.launch.py"), encoding="utf-8").read()
    guard = src[src.index("def activation_guard"):src.index("def generate_launch_description")]
    assert "for j in 1 2 3 4 5" in guard
    assert 'ros2 lifecycle set --no-daemon "$NODE" activate' in guard
    loop = guard[guard.index("for j in 1 2 3 4 5"):]
    assert 'ros2 lifecycle get --no-daemon "$NODE"' in loop, "each call is checked by reading the state back"
    assert "active*)" in loop
    assert "did not activate after 5 calls" in guard


def test_the_guards_own_line_is_not_taken_for_slam_activating(tmp_path):
    log = tmp_path / "slam.log"
    log.write_text(
        "[async_slam_toolbox_node-1] [INFO] [1.0] [slam_toolbox]: Configuring\n"
        "[bash-2] [Linorobot2 Cockpit] slam_toolbox is still inactive -- its launch file missed the"
        " transition event. Activating it.\n"
        "[bash-2] Node not found\n")
    out = _ocp()._slam_complaints(str(log))
    assert "slam_toolbox reached: Configuring" in out
    assert "Activating" not in out.split("reached:")[1].splitlines()[0]
