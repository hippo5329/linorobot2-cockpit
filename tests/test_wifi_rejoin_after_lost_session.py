"""A udp4 board that lost its session rejoins Wi-Fi once, not never and not forever.

2026-10-17, a GenDrv on udp4: the agent session dropped, and for 70 s the board's
LiDAR stream kept reaching the agent's host while not one ping answer came back.
The radio still said WL_CONNECTED, so runWifis() saw nothing to fix, and the
board pinged into the void until the run was torn down.

The rules, read off the source the way the firmware's other invariants are:
  * only a LOST udp4 session arms the rejoin (boot, and serial, never do);
  * any answered ping disarms it;
  * it fires once, AGENT_REJOIN_MS after the loss, past the firmware's own
    5 s udp4 loss verdict and well short of the ~50 s before a gate's next run;
  * wifiRejoin() drops only an association that exists, and logs before it does.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN = os.path.join(ROOT, "firmware", "src", "main.cpp")
WIFIS = os.path.join(ROOT, "firmware", "common", "lib", "wifi", "wifis.cpp")
WIFIS_H = os.path.join(ROOT, "firmware", "common", "lib", "wifi", "wifis.h")


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _case(src, name):
    start = src.index(f"case {name}:")
    end = src.index("break;", src.index("});", start) if name == "WAITING_AGENT" else start)
    return src[start:end]


def test_only_a_lost_udp4_session_arms_the_rejoin():
    src = _read(MAIN)
    arms = [m.start() for m in re.finditer(r"session_lost_ms = millis\(\)", src)]
    assert len(arms) == 1, "one place arms the rejoin"
    lost = _case(src, "AGENT_DISCONNECTED")
    assert "session_lost_ms = millis() | 1" in lost
    assert "if (urosTransportIsUdp())" in lost
    assert "static uint32_t session_lost_ms = 0;" in src, "boot starts disarmed"


def test_an_answered_ping_disarms_and_the_rejoin_fires_once():
    waiting = _case(_read(MAIN), "WAITING_AGENT")
    assert re.search(r"if \(ok\) \{\s*session_lost_ms = 0;", waiting)
    fire = waiting[waiting.index("AGENT_REJOIN_MS"):]
    assert fire.index("session_lost_ms = 0;") < fire.index("wifiRejoin("), \
        "disarmed before it fires, so it fires once"


def test_the_rejoin_waits_past_the_loss_verdict_and_not_into_the_next_run():
    src = _read(MAIN)
    rejoin = int(re.search(r"AGENT_REJOIN_MS\s*=\s*(\d+)", src).group(1))
    loss = int(re.search(r"AGENT_LOSS_UDP_MS\s*=\s*(\d+)", src).group(1))
    assert loss < rejoin <= 20000


def test_wifi_rejoin_drops_only_an_association_that_exists_and_says_so_first():
    src = _read(WIFIS)
    body = src[src.index("void wifiRejoin("):src.index("void runWifis(")]
    assert "WiFi.status() != WL_CONNECTED" in body and "!wifiWanted()" in body
    assert body.index("syslog(") < body.index("WiFi.disconnect()")


def test_a_board_without_a_radio_compiles_the_call_away():
    assert "#define wifiRejoin(why)" in _read(WIFIS_H)
