"""The LD driver must look for a finished revolution faster than the LiDAR makes them.

ldlidar_stl_ros2's loop runs at its `pub_rate` (default 10 Hz) and asks once per pass
whether a revolution is ready; the ask does not wait, and the parser overwrites a
revolution nobody collected. At 10 Hz against a 10 Hz LD19, arrival jitter puts two
revolutions between some looks and one is lost: a real robot's Wi-Fi LD19 published
6.6-8.8 Hz for minutes with every forwarded packet received (2026-10-15), and the motion
guard stopped Nav2 for it. Every LD driver node bringup launches must set `pub_rate`
well above the fastest LD spin (13 Hz).
"""
import ast
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(REPO_ROOT, "launchers", "bringup.launch.py")


def _tree():
    return ast.parse(open(LAUNCH).read())


def _ld_param_dicts(tree):
    """the parameter dicts of every Node(executable="ldlidar_stl_ros2_node")"""
    out = []
    for n in ast.walk(tree):
        if not (isinstance(n, ast.Call) and getattr(n.func, "id", "") == "Node"):
            continue
        kw = {k.arg: k.value for k in n.keywords}
        exe = kw.get("executable")
        if not (isinstance(exe, ast.Constant) and exe.value == "ldlidar_stl_ros2_node"):
            continue
        for d in ast.walk(kw["parameters"]):
            if isinstance(d, ast.Dict):
                out.append({k.value: v for k, v in zip(d.keys, d.values) if isinstance(k, ast.Constant)})
    return out


def test_every_ld_driver_node_sets_a_fast_pub_rate():
    tree = _tree()
    consts = {t.id: n.value.value for n in tree.body if isinstance(n, ast.Assign)
              for t in n.targets if isinstance(t, ast.Name) and isinstance(n.value, ast.Constant)}
    dicts = _ld_param_dicts(tree)
    assert len(dicts) >= 2, "expected the UDP and the serial/topic LD driver nodes"
    for d in dicts:
        assert "pub_rate" in d, f"an LD driver node without pub_rate: {sorted(d)}"
        v = d["pub_rate"]
        rate = consts[v.id] if isinstance(v, ast.Name) else v.value
        assert rate >= 40.0, f"pub_rate {rate} Hz: too close to the LiDAR's own 5-13 Hz"
