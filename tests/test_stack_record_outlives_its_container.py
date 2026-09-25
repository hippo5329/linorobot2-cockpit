"""robot_stack.json lives in the config volume and outlives the container. After
a restart the PID namespace is new and the recorded pgids are handed out again;
a record must never make stop() signal whoever holds that number now."""
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import robot_stack  # noqa: E402


def _live(tmp_path):
    proc = subprocess.Popen(["sleep", "60"], start_new_session=True)
    robot_stack.record("bringup", proc.pid, state_dir=str(tmp_path))
    return proc


def _rewrite(tmp_path, **change):
    path = tmp_path / robot_stack.STACK_FILE
    entries = json.loads(path.read_text())
    for e in entries:
        e.update(change)
        for k, v in change.items():
            if v is None:
                e.pop(k)
    path.write_text(json.dumps(entries))


def _survives_stop(tmp_path, proc):
    robot_stack.stop(state_dir=str(tmp_path))
    return proc.poll() is None


def test_our_own_record_is_alive(tmp_path):
    proc = _live(tmp_path)
    try:
        assert [e["tag"] for e in robot_stack.load(str(tmp_path))] == ["bringup"]
    finally:
        proc.kill()


def test_a_record_from_another_pid_namespace_is_not_trusted(tmp_path):
    proc = _live(tmp_path)
    try:
        _rewrite(tmp_path, pidns="pid:[1]")
        assert robot_stack.load(str(tmp_path)) == []
        assert _survives_stop(tmp_path, proc)
    finally:
        proc.kill()


def test_a_reused_leader_pid_is_not_the_recorded_process(tmp_path):
    proc = _live(tmp_path)
    try:
        _rewrite(tmp_path, starttime=1)
        assert robot_stack.load(str(tmp_path)) == []
        assert _survives_stop(tmp_path, proc)
    finally:
        proc.kill()


def test_a_legacy_record_without_identity_is_not_trusted(tmp_path):
    proc = _live(tmp_path)
    try:
        _rewrite(tmp_path, pidns=None, starttime=None)
        assert robot_stack.load(str(tmp_path)) == []
        assert _survives_stop(tmp_path, proc)
    finally:
        proc.kill()
