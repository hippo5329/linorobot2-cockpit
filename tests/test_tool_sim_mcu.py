"""The tool, driven for real: the firmware built for the host (the Sim MCU), its console on
stdin/stdout and its terminal on a loopback TCP port.

The other tool tests read the source. These run the shell -- the boot window, the parser,
a person's `set` against a command's own arguments, the refusals, the stop, `base`, a former
app name's fallback, the challenge login and the terminal's deadman -- because two of the
2026-10-13 review's defects came from fixes that every source test still passed.

They need the host build (scripts/host_firmware.py binary(): the robot image has it at
/opt/hosturos_ws); without it they skip. The robot's wheels are made real in the env
(sim_wheel 0) so the drive commands run: the host's motors drive nothing and its encoders
read 0, which is all a test of the shell needs.
"""
import hashlib
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import host_firmware  # noqa: E402

BINARY = host_firmware.binary()
pytestmark = pytest.mark.skipif(BINARY is None, reason="no Sim MCU host build here")

GENDRV = os.path.join(REPO_ROOT, "config", "reference", "gendrv_config.yaml")


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class SimMcu:
    """The host firmware with one env, its console read on a thread."""

    def __init__(self, **env):
        import mcu_env
        e = host_firmware.env(GENDRV)
        e.update({k: str(v) for k, v in env.items()})
        self.dir = tempfile.mkdtemp(prefix="lino_tool_test_")
        path = os.path.join(self.dir, "env.bin")
        with open(path, "wb") as fh:
            fh.write(mcu_env.encode(e))
        penv = dict(os.environ, LINO_ENV_BIN=path)
        penv.pop("LINO_NEXT_BOOT", None)
        # Started as the backend starts it (host_firmware.shell_command): run bare, the
        # binary cannot load librclc and every test here saw an empty console.
        self.p = subprocess.Popen(["bash", "-c", host_firmware.shell_command(BINARY)],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, env=penv)
        self.out = b""
        self.lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        while True:
            d = self.p.stdout.read1(4096) if hasattr(self.p.stdout, "read1") else self.p.stdout.read(1)
            if not d:
                return
            with self.lock:
                self.out += d

    def text(self) -> str:
        with self.lock:
            return self.out.decode(errors="replace")

    def send(self, line: str, end: bytes = b"\n"):
        self.p.stdin.write(line.encode() + end)
        self.p.stdin.flush()

    def wait(self, needle: str, timeout: float = 10.0, after: int = 0) -> int:
        end = time.time() + timeout
        while time.time() < end:
            i = self.text().find(needle, after)
            if i >= 0:
                return i
            time.sleep(0.05)
        raise AssertionError(f"no {needle!r} within {timeout} s; got:\n{self.text()[-2000:]}")

    def close(self):
        self.p.kill()
        self.p.wait(5)


@pytest.fixture
def sim():
    made = []

    def make(**env):
        s = SimMcu(**env)
        made.append(s)
        return s
    yield make
    for s in made:
        s.close()


def _in_tool(sim, **env):
    s = sim(tool_wait=2000, sim_wheel=0, **env)
    s.wait("[boot] To open the test tool")
    s.send("tool")
    s.wait("tool> ")
    return s


def test_tool_typed_in_the_window_opens_the_shell(sim):
    s = _in_tool(sim)
    assert "[app] tool" in s.text()
    assert "[uros]" not in s.text()


def test_no_input_goes_on_to_micro_ros(sim):
    s = sim(tool_wait=1000)
    s.wait("[boot] To open the test tool")
    s.wait("[uros]", timeout=8)
    assert "[app] tool" not in s.text()


def test_a_persons_set_reaches_acc(sim):
    """2026-10-13: `set ta_ms 500` then `acc` ran 1000 ms phases -- the command cleared it."""
    s = _in_tool(sim)
    s.send("set ta_ms 400")
    s.wait("[tool] ta_ms = 400")
    s.send("acc")
    s.wait("step_ms=400", timeout=15)
    s.send("stop")
    s.wait("[test_acc] STOPPED")


def test_a_commands_own_argument_does_not_carry_over(sim):
    s = _in_tool(sim)
    s.send("motor 1 30 7")
    s.wait("for 7 s")
    s.send("stop")
    at = s.wait("stopped: motor 1 30 7 (you typed stop")
    s.send("motor 1 30")
    s.wait("for 120 s", after=at)
    s.send("stop")


def test_set_refuses_what_is_read_at_boot(sim):
    s = _in_tool(sim)
    s.send("set sim_wheel 0")
    s.wait("sim_wheel is not a test setting")
    s.send("set ota_password x")
    s.wait("ota_password is not a test setting")


def test_a_2wd_refuses_wheels_3_and_4(sim):
    s = _in_tool(sim)
    s.send("motor 3 30")
    s.wait("this robot has 2 wheels")


def test_a_bare_cr_ends_a_line(sim):
    s = _in_tool(sim)
    s.send("status", end=b"\r")
    s.wait("[tool] app=tool, idle")


def test_stop_stops_a_drive_and_says_why(sim):
    s = _in_tool(sim)
    s.send("motors 25")
    s.wait("MOTOR TEST")
    s.send("")                      # an empty line stops too
    s.wait("ok: motors 25 (running)")
    s.wait("stopped: motors 25 (you typed stop -- motors off)")


def test_base_leaves_for_the_robot_even_when_the_env_boots_the_tool(sim):
    s = sim(app="tool")
    s.wait("tool> ")
    s.send("base")
    s.wait("restarting into base")
    s.wait("[uros]", timeout=10)


def test_a_refused_former_name_boots_base(sim):
    s = sim(app="test_motors", sim_wheel=1)
    s.wait("[app] 'test_motors' tests the wheels")
    s.wait("[uros]", timeout=8)


def _login(port, password):
    c = socket.create_connection(("127.0.0.1", port), timeout=3)
    c.settimeout(0.2)
    buf = b""
    end = time.time() + 5
    while b"login: " not in buf and time.time() < end:
        try:
            buf += c.recv(4096)
        except socket.timeout:
            pass
    nonce = [ln.split()[1] for ln in buf.decode().splitlines() if ln.startswith("nonce ")][0]
    c.sendall(hashlib.md5(f"{password}:{nonce}".encode()).hexdigest().encode() + b"\r")
    return c


def _read_until(c, needle, timeout=6.0):
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        try:
            d = c.recv(4096)
            if not d:
                break
            buf += d
            if needle in buf:
                return buf
        except socket.timeout:
            pass
    raise AssertionError(f"no {needle!r}; got {buf[-800:]!r}")


def test_the_terminal_logs_in_by_challenge_and_stops_a_drive_that_goes_quiet(sim):
    port = _free_port()
    s = _in_tool(sim, term_port=port, ota_password="test-pw-1")
    bad = _login(port, "wrong")
    _read_until(bad, b"wrong answer")
    bad.close()
    good = _login(port, "test-pw-1")
    _read_until(good, b"tool> ")
    good.sendall(b"motor 1 30\r")
    _read_until(good, b"MOTOR 1 at 30 %")
    # No keep-alive from here: the board must stop the drive by itself.
    _read_until(good, b"the terminal went quiet", timeout=8)
    good.close()
    s.wait("the terminal went quiet")
