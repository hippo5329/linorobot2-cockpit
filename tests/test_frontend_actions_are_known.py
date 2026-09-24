"""Every action the browser names must exist in the server-side registry.

The browser no longer sends command text; it names an action and the server
builds the command from validated args (`web/backend/actions.py`). That makes the
action name a contract across two languages and eleven files, with nothing but
this test holding the two ends together.

It matters more since an unknown action became a 400 rather than a silent
fallback to the endpoint's default: before, a renamed action made a screen
quietly do the wrong thing (a serial agent instead of UDP, an unprefixed robot
instead of a namespaced one); now it makes that screen fail outright. Either way
the repair is the same and this test is where it should be caught -- in the
suite, not on a robot.

Deliberately paired in both directions. An action in the registry that no screen
names is not an error (the CLI and the pipeline use some of them), so that half
is reported, not asserted.
"""
import glob
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACTIONS_PY = os.path.join(REPO_ROOT, "web", "backend", "actions.py")
FRONTEND = os.path.join(REPO_ROOT, "web", "frontend")

# Not a builder in the registry: "prepared" claims a handle another endpoint
# already built, handled in core.resolve_command before the registry is asked.
NOT_A_BUILDER = {"prepared"}


def _registry():
    src = open(ACTIONS_PY).read()
    block = re.search(r"_ACTIONS:[^{]*\{(.*?)\n\}", src, re.S)
    assert block, "could not find the _ACTIONS registry; actions.py changed shape"
    names = set(re.findall(r"""^\s*['"]([a-z0-9_]+)['"]\s*:""", block.group(1), re.M))
    assert names, "the _ACTIONS registry parsed as empty"
    return names


def _frontend_action_names():
    """`action: "name"` in any frontend script, with its file for the message."""
    found = {}
    for path in sorted(glob.glob(os.path.join(FRONTEND, "*.js"))
                       + glob.glob(os.path.join(FRONTEND, "*.html"))):
        src = open(path, errors="replace").read()
        for m in re.finditer(r"""\baction:\s*['"]([a-z0-9_]+)['"]""", src):
            found.setdefault(m.group(1), set()).add(os.path.basename(path))
    return found


def test_the_scan_found_something():
    """A regex that matches nothing makes the real assertion vacuous -- the
    exact way a check that names its inputs rots."""
    names = _frontend_action_names()
    assert len(names) >= 10, f"only found {sorted(names)}; the call shape changed"


def test_every_frontend_action_is_a_known_server_action():
    registry = _registry()
    unknown = {n: sorted(f) for n, f in _frontend_action_names().items()
               if n not in registry and n not in NOT_A_BUILDER}
    assert not unknown, (
        "the browser names actions the server does not know, so those screens "
        f"now fail with 400: {unknown}. Registry: {sorted(registry)}")


def test_registry_actions_unused_by_the_ui_are_reported_not_failed(capsys):
    """Some actions are driven by the CLI or the pipeline rather than a screen,
    so an unused builder is information, not a defect."""
    unused = sorted(_registry() - set(_frontend_action_names()))
    print(f"registry actions no frontend screen names: {unused or 'none'}")
    assert True
