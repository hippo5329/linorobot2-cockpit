"""The matrix must ship the leg drivers it is about to run.

The legs do not run from the repo. Each one runs a copy inside its cell --
`/home/ubuntu/release_test_rp2.sh`, the GenDrv's two-transport driver and so
on -- and for a long time nothing refreshed those copies. patch_cells.sh
pushes the cockpit's own files; the leg drivers were written by hand, once.

On 2026-09-25 the 2wd slice came back 0 of 10 in six minutes. Every leg died in
argparse on `--mode fake`: the pipeline had been renamed to `--mode sim` days
earlier, every copy in the repo said `sim`, and the cells still held their
Sep 22-23 copies. The firmware under test never got the chance to run, and the
transcript read as thirty red legs.

soak_run2.sh already carries a comment about exactly this -- it re-pushes
soak_leg_inside.sh on every run because a stale copy once cost it 50 rounds --
and it is the same lesson as patched-scripts-must-reach-the-installed-copy. A
leg driver is an argument to the run, not part of the bench's furniture.

So: every remote script path the matrix runs must be one the matrix pushes,
before the first leg, and the push must be verified rather than hoped at.
"""
import os
import re

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(os.path.dirname(ROOT), "linorobot2-cockpit-lab",
                       "skills", "bench-rig", "scripts")

pytestmark = pytest.mark.skipif(
    not os.path.isdir(SCRIPTS), reason="the lab repo is not checked out here")


def _read(name):
    with open(os.path.join(SCRIPTS, name), encoding="utf-8") as fh:
        return fh.read()


def _uncommented(src):
    """Shell without its comment lines.

    The comments quote the very paths the assertions look for, and a test that
    read them would pass on the explanation of the bug instead of the fix.
    """
    return "\n".join(l for l in src.split("\n") if not l.lstrip().startswith("#"))


# A leg driver invoked by absolute path in the cell. run_release_test.sh is
# named relatively (`cd ~ && bash run_release_test.sh`) and is deliberately not
# in scope: it is not copied but GENERATED in the cell, by insert_fw_step_inside.sh
# grafting the PATCHED_COPY steps into it, so pushing the repo's copy over it
# would strip them.
REMOTE_SCRIPT = re.compile(r"/home/ubuntu/[A-Za-z0-9_/.-]*\.sh")


def test_every_leg_driver_the_matrix_runs_is_also_pushed():
    matrix = _uncommented(_read("full_matrix.sh"))
    invoked = set(REMOTE_SCRIPT.findall(matrix))
    invoked |= set(REMOTE_SCRIPT.findall(_uncommented(_read("one_leg.sh"))))
    assert invoked, "found no remote leg driver at all -- the pattern has stopped matching"

    pushed = set(re.findall(r"push_leg_script\s+\S+\s+\S+\s+\S+\s+(\S+)", matrix))
    missing = invoked - pushed
    assert not missing, (
        f"the matrix runs {sorted(missing)} in the cells but never sends it -- "
        "whatever the cell happens to hold is what the gate measures")


def test_the_push_happens_before_the_first_leg():
    """A refresh after the legs start is a refresh of nothing.

    Any host: the pattern keys on the cell argument, so this public file names
    no machine of the bench (AGENTS.md rule 8)."""
    matrix = _uncommented(_read("full_matrix.sh"))
    last_push = max(m.start() for m in re.finditer(r"push_leg_script\s+\S+\s+lino-", matrix))
    start = matrix.index("MATRIX_START")
    assert last_push < start, \
        "a leg driver is pushed after MATRIX_START -- by then a leg may already have run"


def test_the_push_is_verified_not_hoped_at():
    """A copy that failed to land looks exactly like one that was never stale."""
    matrix = _uncommented(_read("full_matrix.sh"))
    body = matrix[matrix.index("push_leg_script () {"):]
    body = body[:body.index("\n}")]
    assert "sha256sum" in body, "push_leg_script does not check what arrived"
    assert re.search(r'REFUSING TO START', body), \
        "push_leg_script reports a mismatch without stopping the run"
