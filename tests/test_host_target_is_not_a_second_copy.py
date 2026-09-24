"""The host micro-ROS target must stay an instrument, not a reimplementation.

`firmware/host/` compiles the firmware's own `mcu_env.cpp` and `uros_transport.cpp`
natively so that micro-ROS over UDP4 -- the one layer `scripts/sim_base_node.py`
removes -- can be tested without silicon. Its value depends entirely on those
being the SAME sources a board runs. The moment a copy is taken "just to get it
building", the target starts passing for code no robot executes, and it becomes
the most convincing kind of false green: a test named after the thing it stopped
testing. `tests/test_sim_base_node.py` exists to stop exactly that happening to
the Python simulated base; this stops it happening here.

Four invariants, each one a way that could go wrong quietly.
"""
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(REPO_ROOT, "firmware")
HOST = os.path.join(FW, "host")
SHIM = os.path.join(HOST, "shim")
CMAKELISTS = os.path.join(HOST, "probe", "CMakeLists.txt")
PROBE_MAIN = os.path.join(HOST, "probe", "host_probe.cpp")


def test_the_shim_declares_the_target_itself():
    """`LINO_HOST` has to come from Arduino.h, not from a build flag.

    Every per-MCU branch in the firmware -- `mcu_env.cpp`'s loader,
    `uros_transport.cpp`'s socket -- tests this macro, and every one of our
    translation units includes Arduino.h. A `-DLINO_HOST` in the CMakeLists could
    be given to one target and forgotten for the next, which would hand one object
    a different view of the tree than its neighbour: an ODR violation that links
    cleanly and misbehaves at run time. Defining it in the header the branches
    already depend on makes that impossible rather than merely unlikely.
    """
    arduino_h = open(os.path.join(SHIM, "Arduino.h")).read()
    assert re.search(r"^#define\s+LINO_HOST\b", arduino_h, re.M), \
        "firmware/host/shim/Arduino.h must define LINO_HOST"

    cmake = open(CMAKELISTS).read()
    assert "-DLINO_HOST" not in cmake, (
        "LINO_HOST must not also be passed on the command line: two sources for one "
        "macro is how they come to disagree")


def test_every_source_under_test_comes_from_the_firmware_tree():
    """The CMakeLists must reach into `firmware/`, and the files must be there."""
    cmake = open(CMAKELISTS).read()
    body = cmake[cmake.index("add_executable"):cmake.index(")", cmake.index("add_executable"))]
    sources = [s.strip() for s in body.splitlines()[1:] if s.strip()]
    assert sources, "the probe builds no sources"

    from_firmware = [s for s in sources if "${FIRMWARE_ROOT}" in s]
    assert len(from_firmware) >= 2, (
        "the host target exists to exercise the firmware's OWN env and transport; "
        f"only {len(from_firmware)} of its sources come from the firmware tree")

    for src in from_firmware:
        rel = src.replace("${FIRMWARE_ROOT}/", "")
        assert os.path.exists(os.path.join(FW, rel)), \
            f"the probe builds {rel}, which is not in the firmware tree"

    # And the two that carry the claim have to be among them, because the target's
    # whole purpose is that the env and the transport are the board's.
    joined = " ".join(from_firmware)
    for required in ("mcu_env.cpp", "uros_transport.cpp"):
        assert required in joined, (
            f"{required} is no longer compiled from the firmware tree: the host "
            "target would then prove nothing about the firmware")


def test_the_host_tree_holds_no_copy_of_a_firmware_source():
    """A shim may ADD an Arduino surface; it may never re-implement our own code.

    Checked by relationship rather than by a list of forbidden filenames: anything
    under `firmware/host/` whose name matches a real firmware source is a copy,
    whatever it is called today.
    """
    firmware_sources = set()
    for root, _dirs, files in os.walk(os.path.join(FW, "common", "lib")):
        for name in files:
            if name.endswith((".cpp", ".h")):
                firmware_sources.add(name)
    firmware_sources.update(
        n for n in os.listdir(os.path.join(FW, "src")) if n.endswith((".cpp", ".h")))

    # config.h is the one deliberate exception, and the reason is structural: on a
    # board it is the generated header for one robot, and the host target must NOT
    # have one. The shim's is the ABSENCE of a robot, not a copy of one -- which the
    # next test pins down.
    #
    # Nothing else gets an exception, which is why the probe's entry point is
    # host_probe.cpp and not main.cpp: it is the host's own scaffolding rather than
    # a port of firmware/src/main.cpp, and a name that invited that reading would
    # also collide for real the day the host target compiles the base application.
    allowed = {"config.h"}

    copies = []
    for root, _dirs, files in os.walk(HOST):
        for name in files:
            if name in firmware_sources and name not in allowed:
                copies.append(os.path.relpath(os.path.join(root, name), REPO_ROOT))
    assert not copies, (
        "these files under firmware/host/ shadow real firmware sources; the host "
        f"target must compile the originals, not copies: {sorted(copies)}")


def test_the_host_has_no_compiled_in_robot():
    """The shim's config.h supplies fallbacks for a HOST, never a robot's values.

    On a board `config.h` includes the header generated for one robot. The host
    reads its configuration from the env image instead, so anything answered at
    compile time here is a second source of truth for a value the env already
    carries -- and the first thing the pair would do is disagree. That is not
    hypothetical: a design leaked into every image twice already, the second time
    through the generated bare config.

    So the allowlist is the test. Adding a macro here is allowed, but it has to be
    a deliberate act with a justification, not something that arrives with a
    copy-paste.
    """
    text = open(os.path.join(SHIM, "config.h")).read()
    defined = set(re.findall(r"^#define\s+([A-Z0-9_]+)", text, re.M))
    justified = {
        "CONFIG_H",            # the include guard
        "TRANSPORT_DEFAULT",   # udp4: the only transport a host HAS
        "AGENT_IP_DEFAULT",    # loopback: needs no configuration to be reachable
        "AGENT_PORT_DEFAULT",  # 8888: micro_ros_agent's own default
        "SIM_LD19_DEFAULT",   # false: there is no UART to emit LD19 frames on
        "LIDAR_RXD",           # -1: the firmware's own "not wired"
    }
    unjustified = defined - justified
    assert not unjustified, (
        "firmware/host/shim/config.h defines macros that are not justified for a "
        f"host: {sorted(unjustified)}. If the host genuinely needs one, add it to "
        "this test's allowlist with the reason. If it describes a ROBOT, it belongs "
        "in the env image instead.")


def test_the_probe_does_not_configure_the_rmws_own_socket():
    """The tripwire for the exact false positive this target was built after.

    `rmw_uros_options_set_udp_address()` points the rmw's BUILT-IN UDP socket at an
    agent. Call it and the session works -- while `uros_transport.cpp`'s four
    functions sit installed and never called, so a `transport=udp4` regression in
    the firmware passes. The address must come from the env image and the bytes must
    go through the firmware, which is what `--cmake-args RMW_UXRCE_TRANSPORT=custom`
    and this absence together guarantee.

    (The earlier version of this work reported success from a binary that published
    over Fast DDS with no agent running at all. The lesson was not "check harder",
    it was "leave no second path to a green".)
    """
    main = open(PROBE_MAIN).read()
    code = re.sub(r"//[^\n]*", "", main)
    code = re.sub(r"/\*.*?\*/", "", code, flags=re.S)
    assert "rmw_uros_options_set_udp_address" not in code, (
        "main.cpp configures the rmw's own UDP socket: the firmware's transport "
        "functions would then be installed and never called, and the probe would "
        "pass without testing anything of ours")

    # The other half of the same claim: the address has to be read from the env.
    assert "initUrosTransport" in code, \
        "the probe must install the transport through the firmware's own selector"
    assert "initMcuEnv" in code, \
        "the probe must configure itself from the env image, like a board"
