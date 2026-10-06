"""A failed createEntities() returns false, so the state machine tears down and retries.

RCCHECK called rclErrorLoop(), which never returns. An entity request times out after
1 s, and an agent still creating its DDS participant on a loaded computer is that slow:
on the UNO Q (gate rc-20261007.1) node init failed 1.006 s after the session opened and
the Sim MCU sat silent until the leg was stopped. Only RP2's watchdog got a board out of
it, by rebooting; ESP32, the S3 and the Sim MCU stayed stuck until power was cut.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN = open(os.path.join(ROOT, "firmware", "src", "main.cpp")).read()
CREATE = MAIN[MAIN.index("bool createEntities()\n{"):MAIN.index("bool destroyEntities()\n{")]
RCCHECK = MAIN[MAIN.index("#define RCCHECK("):MAIN.index("#endif", MAIN.index("#define RCCHECK("))]


def test_rccheck_returns_false_and_never_loops():
    assert "return false" in RCCHECK
    assert "rclErrorLoop" not in RCCHECK


def test_rccheck_is_used_only_in_create_entities():
    # `return false` is only right inside a bool function the state machine retries
    outside = MAIN.replace(CREATE, "").replace(RCCHECK, "")
    assert not re.search(r"\bRCCHECK\(", outside)


def test_the_state_machine_retries_a_failed_create():
    i = MAIN.index("case AGENT_AVAILABLE:")
    block = MAIN[i:MAIN.index("case AGENT_CONNECTED:", i)]
    assert "(true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT" in block
    assert "destroyEntities();" in block


def test_a_failed_domain_id_frees_the_init_options():
    j = CREATE.index("rcl_init_options_set_domain_id")
    assert "rcl_init_options_fini(&init_options)" in CREATE[j:CREATE.index("RCCHECK(domain_rc)")]
