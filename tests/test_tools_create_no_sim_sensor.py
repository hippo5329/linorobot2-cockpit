"""The tools run on a real robot, and nothing simulated stands in for a part they cannot find.

User, 2026-10-12: "tools are for real robots, no sim devices should be used", and
"sensors should be auto-detect".

createIMU()/createMAG() hand `base` a simulated part for `sim` or a name they do not know.
The tools went through the same factories: on a mecanum robot with `mag: none`,
test_sensors printed "[+] Magnetometer initialized successfully" and streamed a simulated
field, and with `imu: none` test_acc reported a simulated IMU's acceleration as IMU ACC.
Now toolSelect() sets the env real-only (envSetRealOnly) for a tool -- never for `base`,
which returns before the tool loop -- the factories return NULL for
anything but a real chip's driver, the battery and barometer models stay off, and every
tool asks the bus first (i2cProbeSelect), so `auto` is the chip that answered or none.
test_sensors also prints RANGE none for a sonar that is not fitted (it printed the seeded
max range, 4.84 m, as if clear).
"""
import glob
import os
import re

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB = os.path.join(REPO_ROOT, "firmware", "common", "lib")
TOOLS_DIR = os.path.join(REPO_ROOT, "firmware", "src", "tools")
FACTORY = os.path.join(LIB, "sensor_factory", "sensor_factory.cpp")


def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def _src(name):
    return _read(os.path.join(TOOLS_DIR, name + ".cpp"))


def _func(src, signature):
    m = re.search(re.escape(signature) + r"\s*\{(.*?)\n\}", src, re.S)
    assert m, f"{signature} not found"
    return m.group(1)


def test_tool_select_sets_real_only_before_starting_a_tool():
    body = _func(_src("tools"), "AppMode toolSelect(void)")
    ret = body.index("return TOOLS[i].mode;")
    assert "envSetRealOnly(true);" in body[body.rfind("simulatedFor(", 0, ret):ret]


def test_base_never_turns_real_only():
    """`base` is TOOLS[0], so the loop matches `app=base` too. Reaching envSetRealOnly()
    there made createIMU("sim") return NULL in the robot firmware itself, and every MCU
    faulted seconds into each micro-ROS session (rc-20261012.2: 4 sessions in 45 s, the
    board re-enumerating between them). `base` has to leave toolSelect() before the loop."""
    src = _src("tools")
    table = src[src.index("static const ToolEntry TOOLS[]"):]
    assert re.search(r'\{\s*APP_BASE\s*,\s*"base"', table), "base left the table: re-check this test"
    body = _func(src, "AppMode toolSelect(void)")
    early = re.search(r'if \(!want \|\| !\*want \|\| strcmp\(want, "base"\) == 0\)\s*return APP_BASE;', body)
    assert early, "toolSelect() must return APP_BASE for app=base before the tool loop"
    assert early.start() < body.index("for (size_t i = 0; i < TOOL_COUNT; i++)")
    assert early.start() < body.index("envSetRealOnly(true);")
    # And nothing else turns it on: only toolSelect() sets it, and only for a tool.
    sources = [f for d in ("src", "common") for ext in ("*.c", "*.cpp", "*.h")
               for f in glob.glob(os.path.join(REPO_ROOT, "firmware", d, "**", ext), recursive=True)]
    setters = [f for f in sources if "envSetRealOnly(true)" in _read(f)]
    assert [os.path.basename(f) for f in setters] == ["tools.cpp"]


@pytest.mark.parametrize("kind", ["IMU", "MAG"])
def test_factory_makes_nothing_simulated_when_real_only(kind):
    body = _func(_read(FACTORY), f"{kind}Interface *create{kind}(const char *name)")
    real_only = body[body.index("if (envRealOnly())"):body.index("return NULL;")]
    # The table lookup skips the simulated row, and the fallback is NULL, not new Sim*.
    assert f'strcasecmp({kind}_TABLE[i].name, "sim") != 0' in real_only
    assert "new Sim" not in real_only
    assert body.index("if (envRealOnly())") < body.index(f"return new Sim{kind}();")


@pytest.mark.parametrize("path,key", [("battery/battery.cpp", "sim_battery"), ("env/env.cpp", "sim_env")])
def test_battery_and_barometer_models_stay_off_in_a_tool(path, key):
    src = _read(os.path.join(LIB, path))
    assert re.search(r's_sim = !envRealOnly\(\) && envFlag\("%s"' % key, src), path


@pytest.mark.parametrize("tool", ["test_sensors", "test_acc", "test_motors"])
def test_every_tool_asks_the_bus_before_creating(tool):
    src = _src(tool)
    probe = src.index("i2cProbeSelect(&imu_name, &mag_name);")
    for kind in ("IMU", "MAG"):
        assert probe < src.index(f"createIMU(imu_name)" if kind == "IMU" else "createMAG(mag_name)")


@pytest.mark.parametrize("tool", ["test_sensors", "test_acc", "test_motors"])
@pytest.mark.parametrize("kind", ["IMU", "MAG"])
def test_every_create_is_null_checked(tool, kind):
    src = _src(tool)
    var = kind.lower()
    calls = [m.end() for m in re.finditer(r"create%s\(%s_name\)" % (kind, var), src)]
    assert calls, f"{tool} creates no {kind}"
    for end in calls:
        after = src[end:end + 40]
        assert re.match(r"\)\)|;\s*\n\s*if \(%s\)" % var, after), f"{tool}: create{kind}() result used unchecked"
    assert "sensorFitted(" not in src


@pytest.mark.parametrize("tool", ["test_sensors", "test_acc"])
def test_reads_are_guarded(tool):
    src = _src(tool)
    for m in re.finditer(r"\b(imu|mag)->getData\(\)", src):
        line_start = src.rfind("\n", 0, m.start())
        ctx = src[max(0, line_start - 300):m.start()]
        assert re.search(r"if \(!?%s\b|if \(%s\)|!%s \|\|" % ((m.group(1),) * 3), ctx), \
            f"{tool}: {m.group(0)} at {m.start()} without a null check"


def test_test_sensors_prints_range_none_without_a_sonar():
    src = _src("test_sensors")
    assert "RANGE: none" in src and "rangePresent()" in src


def test_test_acc_says_none_without_an_imu():
    assert 'IMU ACC none (no IMU)' in _src("test_acc")


def test_reference_designs_leave_add_on_sensors_to_the_bus():
    # A design names only what is soldered on its board; everything else is AUTO, so a
    # chip the user adds is found and an empty bus is none. `NONE` would stop the probe.
    for path in glob.glob(os.path.join(REPO_ROOT, "config", "reference", "*_config.yaml")):
        sensors = (yaml.safe_load(_read(path)).get("base_controller") or {}).get("sensors") or {}
        for field in ("imu", "mag", "current", "env"):
            if field in sensors:
                assert str(sensors[field]).upper() != "NONE", f"{os.path.basename(path)}: {field} NONE"
