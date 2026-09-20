#!/usr/bin/env python3
# ==============================================================================
# robot_stack.py — what the 1-Click pipeline left running, and how to stop it
#
# The pipeline used to tear bringup, SLAM and Nav2 down in its `finally` block
# the moment it finished. That is right for an automated run and wrong for a
# person: they press Start 1-Click to GET a robot, and were handed a robot that
# had just been switched off. Keeping the stack alive means the pipeline exits
# while its children keep running -- so something has to remember them, or the
# Stop buttons have nothing to signal.
#
# The record lives in the shared state directory for the reason cockpit_paths
# already documents: the pipeline runs as container-root and the backend as the
# container user, so anything under $HOME is two different files and neither
# side sees the other.
#
# Process groups, not names. Every launch is started with os.setsid(), so one
# signal to the group reaches `ros2 launch` and everything it spawned -- and a
# pgid cannot match the wrong process the way a name can (AGENTS.md: never
# pkill, never killall).
# ==============================================================================
import json
import os
import signal
import time

try:
    import cockpit_paths
except ImportError:  # running from somewhere that has not put scripts/ on the path
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import cockpit_paths

STACK_FILE = "robot_stack.json"


def _path(state_dir: str = None) -> str:
    return os.path.join(state_dir or cockpit_paths.state_dir(), STACK_FILE)


def load(state_dir: str = None) -> list:
    """Every entry we believe is running, newest first, dead ones dropped."""
    try:
        with open(_path(state_dir)) as fh:
            entries = json.load(fh)
    except (OSError, ValueError):
        return []
    if not isinstance(entries, list):
        return []
    alive = [e for e in entries if isinstance(e, dict) and is_alive(e)]
    if len(alive) != len(entries):
        _write(alive, state_dir)
    return alive


def _write(entries: list, state_dir: str = None) -> None:
    path = _path(state_dir)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(entries, fh, indent=1)
    os.replace(tmp, path)
    try:
        os.chmod(path, 0o666)  # written by root, stopped by the cockpit user
    except OSError:
        pass


def _is_zombie(pid: int) -> bool:
    """A reaped-but-not-collected `ros2 launch` is not a running robot.

    Its parent is the pipeline, which exited on purpose, so nothing collects
    it until init does -- and meanwhile its process GROUP still exists, so
    killpg(pgid, 0) succeeds and the corpse reads as alive. Measured here: all
    three launches sat as `[ros2] <defunct>` after being stopped.
    """
    try:
        with open(f"/proc/{int(pid)}/stat") as fh:
            # ... ) S ... -- the state follows the comm field, which may itself
            # contain spaces or brackets, so split after the last ')'.
            data = fh.read()
        return data[data.rindex(")") + 1:].split()[0] == "Z"
    except (OSError, ValueError, IndexError):
        return False


def is_alive(entry: dict) -> bool:
    pgid = entry.get("pgid")
    if not pgid:
        return False
    pid = entry.get("pid")
    if pid and _is_zombie(pid):
        return False
    try:
        os.killpg(int(pgid), 0)
        return True
    except (ProcessLookupError, ValueError):
        return False
    except PermissionError:
        # It exists and belongs to someone else -- alive as far as we know.
        return True


def record(tag: str, pid: int, pgid: int = None, state_dir: str = None) -> None:
    if pgid is None:
        try:
            pgid = os.getpgid(pid)
        except Exception:
            pgid = pid
    entries = [e for e in load(state_dir) if e.get("tag") != tag]
    entries.insert(0, {"tag": tag, "pid": int(pid), "pgid": int(pgid),
                       "started": time.time()})
    _write(entries, state_dir)


def forget(tag: str = None, state_dir: str = None) -> None:
    if tag is None:
        _write([], state_dir)
        return
    _write([e for e in load(state_dir) if e.get("tag") != tag], state_dir)


def stop(tag: str = None, state_dir: str = None) -> list:
    """Signal the recorded group(s). SIGINT first: `ros2 launch` tears its
    nodes down cleanly on SIGINT and is merely killed by SIGTERM."""
    stopped = []
    for entry in load(state_dir):
        if tag is not None and entry.get("tag") != tag:
            continue
        pgid = int(entry["pgid"])
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            except PermissionError:
                break
            deadline = time.time() + 3.0
            gone = False
            while time.time() < deadline:
                if not is_alive(entry):
                    gone = True
                    break
                time.sleep(0.15)
            if gone:
                break
        stopped.append(entry.get("tag"))
        forget(entry.get("tag"), state_dir)
    return stopped


def describe(state_dir: str = None) -> str:
    entries = load(state_dir)
    if not entries:
        return "nothing from a 1-Click run is still running"
    return ", ".join(f"{e['tag']} (pgid {e['pgid']})" for e in entries)


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(description="What the 1-Click pipeline left running.")
    ap.add_argument("--stop", nargs="?", const="__all__", metavar="TAG",
                    help="stop everything, or one of bringup/slam/nav2")
    a = ap.parse_args()
    if a.stop:
        tag = None if a.stop == "__all__" else a.stop
        stopped = stop(tag)
        print("stopped: " + (", ".join(t for t in stopped if t) or "nothing was running"))
        return 0
    print(describe())
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
