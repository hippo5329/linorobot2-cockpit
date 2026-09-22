"""release_serial_port() must signal the processes that hold the port -- and only them.

In a rootless Docker container a passed-through device node resolves to the
device numbers of /dev/null, so `lsof -t <port>` listed every process with
/dev/null open, this flasher included, and the SIGINT meant for a stale agent
interrupted the flash itself. The holder scan now reads /proc and never
returns our own ancestry.
"""
import importlib.util
import os
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load():
    spec = importlib.util.spec_from_file_location("flash_mcu", os.path.join(ROOT, "scripts", "flash_mcu.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_a_child_holding_the_path_is_found_and_we_are_not():
    fm = _load()
    with tempfile.NamedTemporaryFile(delete=False) as tmp:
        path = tmp.name
    try:
        # Our own open fd on the path must not count.
        with open(path) as _own:
            assert os.getpid() not in fm._holders_of(path)
            assert fm._holders_of(path) == []
        child = subprocess.Popen([sys.executable, "-c",
                                  f"import time; f=open({path!r}); time.sleep(30)"])
        try:
            deadline = time.time() + 5
            holders = []
            while time.time() < deadline:
                holders = fm._holders_of(path)
                if child.pid in holders:
                    break
                time.sleep(0.05)
            assert child.pid in holders, holders
            assert os.getpid() not in holders and os.getppid() not in holders
        finally:
            child.kill(); child.wait()
    finally:
        os.unlink(path)


def test_the_ancestry_walk_includes_us_and_our_parent():
    fm = _load()
    anc = fm._ancestry()
    assert os.getpid() in anc and os.getppid() in anc


def test_release_no_longer_shells_out_to_lsof():
    with open(os.path.join(ROOT, "scripts", "flash_mcu.py")) as fh:
        src = fh.read()
    body = src[src.index("def release_serial_port"):]
    body = body[:body.index("\ndef ", 1)]
    assert "lsof" not in body.replace("Not lsof", "").replace("`lsof -t", ""), "the holder scan reads /proc"
    assert "_holders_of(serial_port)" in body
