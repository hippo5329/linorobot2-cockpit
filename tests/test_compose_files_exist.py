"""Every file `actions.py` passes to `-f` must be one that exists.

`docker-compose.yml` has carried that name since the first commit. The action
registry that replaced the browser's raw command strings (`0d789db`, the fix for
the previous review round's headline finding) typed `docker-compose.yaml` in four
places, so `bringup_docker`, `docker_service_up`, `docker_build` and
`docker_down` each named a file that is not in the repo and cannot be generated.
Nothing caught it: the bench path runs `one_click_pipeline.py`, not the compose
actions, and no test asserted the names.

This is the shape the project has already paid for -- a check that names its
inputs rots the moment the inputs are renamed. So assert the relationship rather
than the spelling: whatever `actions.py` names, the repo must be able to provide.
"""
import os
import re
import subprocess

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACTIONS = os.path.join(REPO_ROOT, "web", "backend", "actions.py")

# Files the cockpit writes itself at run time, so they are legitimately absent
# from a fresh clone. Anything else has to be tracked.
GENERATED = {"devices.generated.yaml"}


def _named_compose_files():
    """Every `-f <file>` target in actions.py, in source order."""
    with open(ACTIONS) as fh:
        src = fh.read()
    # Only the literal flags strings, not a comment mentioning a name.
    return [m.group(1) for m in re.finditer(r"-f\s+([A-Za-z0-9_.-]+\.ya?ml)", src)]


def test_actions_names_at_least_one_compose_file():
    """A regex that matches nothing would make every assertion below vacuous."""
    assert _named_compose_files(), "no -f targets found; the regex or the file changed shape"


def test_every_named_compose_file_is_tracked_or_generated():
    tracked = set(subprocess.run(["git", "ls-files"], cwd=REPO_ROOT,
                                 capture_output=True, text=True).stdout.split())
    assert tracked, "git ls-files returned nothing; not a checkout?"
    for name in _named_compose_files():
        if name in GENERATED:
            continue
        assert name in tracked, (
            f"actions.py passes -f {name}, which is neither tracked nor generated. "
            f"The repo has: {sorted(n for n in tracked if 'compose' in n)}")


def test_the_compose_file_on_disk_is_the_one_actions_names():
    """Belt and braces: the tracked check passes on a stale index too."""
    for name in _named_compose_files():
        if name in GENERATED:
            continue
        assert os.path.exists(os.path.join(REPO_ROOT, name)), \
            f"actions.py passes -f {name}, which is not on disk"
