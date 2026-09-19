"""The config dir's .gitignore keeps this machine's state out of the user's repo."""
import os

import cockpit_paths


def _gitignore(d):
    with open(os.path.join(d, ".gitignore")) as fh:
        return fh.read().splitlines()


def test_seeded_gitignore_hides_machine_state(isolated_config_dir):
    d = cockpit_paths.ensure_config_dir(quiet=True)
    lines = _gitignore(d)
    for entry in ("secrets.yaml", ".cockpit_token", ".active_robot", "generated/"):
        assert entry in lines


def test_old_gitignore_gains_the_new_entries(isolated_config_dir):
    d = cockpit_paths.ensure_config_dir(quiet=True)
    with open(os.path.join(d, ".gitignore"), "w") as fh:
        fh.write("secrets.yaml\n")
    cockpit_paths.ensure_config_dir(quiet=True)
    lines = _gitignore(d)
    assert lines[0] == "secrets.yaml"
    for entry in (".cockpit_token", ".active_robot", "generated/"):
        assert entry in lines
