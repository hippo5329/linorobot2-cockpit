"""The cockpit and the command it runs must agree on where the workspace is.

The shipped container image puts its built workspace at /opt/lino_ws/setup.bash,
not at <ws>/install/setup.bash. `get_ros_env()` sourced it; the /api/status
"is it built?" check did not, so on the official image the Bringup tab said the
workspace was unbuilt and ran `cd ~/linorobot2_ws && colcon build` -- in a
directory that does not exist. Bringup could not be started from the UI at all.
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import one_click_pipeline as ocp  # noqa: E402


def test_the_container_image_layout_is_first_in_the_list():
    assert ocp.WORKSPACE_SETUPS[0] == "/opt/lino_ws/setup.bash"


def test_every_path_the_shell_chain_sources_is_in_the_list():
    """get_ros_env() builds the chain as shell; this list must not drift.

    Compare the two normalised, because the shell chain spells the checkout
    relatively (`<repo>/../../install/setup.bash`) and writes $HOME rather than
    expanding it, while the Python list is what os.path produces.
    """
    import re
    chain = ocp.get_ros_env("jazzy")
    sourced = {os.path.normpath(m.replace("$HOME", os.path.expanduser("~")))
               for m in re.findall(r"\[ -f (\S*setup\.bash) \]", chain)}
    for path in ocp.WORKSPACE_SETUPS:
        assert os.path.normpath(path) in sourced, (
            f"{path} is in WORKSPACE_SETUPS but get_ros_env() never sources it")


def test_workspace_setup_reports_the_first_that_exists(tmp_path, monkeypatch):
    a = tmp_path / "a" / "setup.bash"
    b = tmp_path / "b" / "setup.bash"
    for p in (a, b):
        p.parent.mkdir(parents=True)
        p.write_text("")
    monkeypatch.setattr(ocp, "WORKSPACE_SETUPS", (str(a), str(b)))
    assert ocp.workspace_setup() == str(a)


def test_no_workspace_is_an_empty_string_not_a_guess(tmp_path, monkeypatch):
    monkeypatch.setattr(ocp, "WORKSPACE_SETUPS", (str(tmp_path / "nope" / "setup.bash"),))
    assert ocp.workspace_setup() == ""
