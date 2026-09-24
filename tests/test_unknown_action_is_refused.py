"""An action the server does not know must fail, not run the endpoint's default.

`resolve_command` builds the command from a named server-side action, and an
unknown name used to fall through to the raw-command path -- which, with raw
disabled, means `return default`. So the request succeeded and ran something
else:

  * `/api/agent/exec` defaults to `micro_ros_agent serial --dev <port> -b <baud>`,
    so a misspelled UDP action silently started a SERIAL agent on the config's
    port;
  * `/api/bringup/exec` defaults to a bare `ros2 launch ... bringup.launch.py`,
    so a misspelled namespaced action silently brought up an UNPREFIXED robot.

Both look like the request worked, which is the same failure shape this project
has already paid for twice: a params file the node does not match is not an
error, and a bare frame_id is not an error. Renaming an action in actions.py
would have done this to every caller that had not been updated.
"""
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))

# core.py sys.exit(1)s when FastAPI is absent, and importorskip cannot catch
# SystemExit -- probe first, exactly as tests/test_robot_list_identity.py does.
pytest.importorskip("fastapi", reason="the cockpit backend needs fastapi/uvicorn")
import core  # noqa: E402

from fastapi import HTTPException  # noqa: E402

BRINGUP_DEFAULT = "ros2 launch linorobot2_cockpit bringup.launch.py"


@pytest.fixture(autouse=True)
def raw_exec_off(monkeypatch):
    monkeypatch.delenv("COCKPIT_ALLOW_RAW_EXEC", raising=False)


def test_an_unknown_action_does_not_run_the_default():
    with pytest.raises(HTTPException) as exc:
        core.resolve_command({"action": "slaam", "args": {"distro": "jazzy"}},
                             default=BRINGUP_DEFAULT)
    assert exc.value.status_code == 400
    assert "slaam" in str(exc.value.detail)


def test_a_known_action_still_builds():
    """The guard must not break the path every UI screen uses."""
    assert core.actions.known("bringup"), "the registry lost 'bringup'"
    got = core.resolve_command(
        {"action": "bringup",
         "args": {"launcher": "linorobot2_cockpit/bringup.launch.py",
                  "config_path": "/home/ubuntu/linorobot2-config/pico2_mecanum_config.yaml"}},
        default="")
    assert got and "bringup.launch.py" in got


def test_a_known_action_with_bad_args_is_a_400_not_a_default():
    """A validation failure inside a known action must not become the fallback
    either -- same confusion, one layer down."""
    with pytest.raises(HTTPException) as exc:
        core.resolve_command({"action": "bringup", "args": {}}, default=BRINGUP_DEFAULT)
    assert exc.value.status_code == 400


def test_no_action_at_all_still_falls_back_to_the_default():
    """An endpoint's own fallback is for a request that names nothing, which is
    a different thing from a request that names something wrong."""
    assert core.resolve_command({}, default=BRINGUP_DEFAULT) == BRINGUP_DEFAULT


def test_a_raw_command_is_still_refused_when_raw_is_off():
    with pytest.raises(HTTPException) as exc:
        core.resolve_command({"command": "rm -rf /"}, default=BRINGUP_DEFAULT)
    assert exc.value.status_code == 400
    assert "raw commands are disabled" in str(exc.value.detail)


def test_an_unknown_action_beside_a_raw_command_is_refused_when_raw_is_off():
    """The legacy shape was a human LABEL in `action` plus the real `command`.
    It only ever worked with raw enabled; with raw off it must not become a
    silent default either."""
    with pytest.raises(HTTPException) as exc:
        core.resolve_command({"action": "Start SLAM", "command": "ros2 launch x y"},
                             default=BRINGUP_DEFAULT)
    assert exc.value.status_code == 400


def test_the_legacy_label_plus_command_still_works_with_raw_enabled(monkeypatch):
    monkeypatch.setenv("COCKPIT_ALLOW_RAW_EXEC", "on")
    got = core.resolve_command({"action": "Start SLAM", "command": "ros2 launch x y"},
                               default=BRINGUP_DEFAULT)
    assert got == "ros2 launch x y"


def test_an_expired_prepared_handle_is_refused_not_defaulted():
    with pytest.raises(HTTPException) as exc:
        core.resolve_command({"action": "prepared", "handle": "nope"},
                             default=BRINGUP_DEFAULT)
    assert exc.value.status_code == 400
