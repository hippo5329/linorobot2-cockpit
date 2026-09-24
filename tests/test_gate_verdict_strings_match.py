"""The gate's verdict sentences are an interface, and one end drifted.

`cut_and_soak.sh` is the script that pulls the trigger: it runs the gate, reads
the verdict, and only then cuts and soaks. It read the verdict by grepping for a
literal sentence -- "GATE: PASS -- all ten, all on the staged firmware" -- which
`gate_check.sh` stopped printing when it learned to distinguish a 10-leg slice
from the 30-leg matrix and started saying "SLICE PASS" instead.

Nothing failed. The grep simply could never match again, so the trigger script
was dead, and a reader watching it refuse would have read that as a red gate
rather than a broken one. This is the same shape as the CI check that listed
frontend filenames and went red for four runs after a rename: a check that names
its inputs rots, and silently, because the naming is the part nobody runs.

So the handshake gets a test. Each sentence a consumer greps for must actually
be produced by the script it greps.

The second assertion is about how much evidence the trigger asks for.
`gate_all.sh` exists because reading 10 of 30 legs as a matrix pass cost a re-cut
on 2026-09-22, and `cut_and_soak.sh` was the one caller still asking
`gate_check.sh` -- one drivetrain -- for a verdict that gates all three.

Lives in the public repo because that is where the suite is; skipped when the lab
is not checked out beside it, like test_no_sim_on_a_real_sensor_run.py.
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


def _grepped_sentences(src):
    """The literals this script greps a gate verdict out of.

    Only `GATE:` patterns: the script greps other things for other reasons, and
    a test that claimed every grep in the file was a verdict would itself be the
    kind of over-broad check that rots.
    """
    out = []
    for pat in re.findall(r'grep\s+-q\s+"([^"]+)"', src):
        lit = pat.lstrip("^")
        if lit.startswith("GATE:"):
            out.append(lit)
    return out


def test_the_trigger_greps_a_sentence_the_gate_actually_prints():
    src = _read("cut_and_soak.sh")
    sentences = _grepped_sentences(src)
    assert sentences, "cut_and_soak.sh no longer greps for a GATE: verdict at all"
    producer = _read("gate_all.sh") + _read("gate_check.sh")
    for lit in sentences:
        assert lit in producer, (
            f"cut_and_soak.sh waits for {lit!r}, which no gate script prints -- "
            "the verdict was reworded and the trigger was left behind")


def test_the_trigger_asks_for_all_thirty_not_one_slice():
    src = _read("cut_and_soak.sh")
    assert "gate_all.sh" in src, \
        "cut_and_soak.sh must take its verdict from gate_all.sh (30 legs)"
    assert "all thirty" in " ".join(_grepped_sentences(src)), \
        "the sentence it waits for is not the thirty-leg one"
    assert not re.search(r'\$HERE/gate_check\.sh', src), \
        "cut_and_soak.sh is judging a single drivetrain slice again -- that is 10 of 30"


def test_release_mode_still_proves_which_firmware_ran():
    """GATE_RELEASE must swap the provenance check, never drop it.

    The staged gate demands every profile name the hand build; the release gate
    demands every profile name the published tag. The tempting third option --
    skip provenance when the cells are unstaged -- would make a green matrix say
    nothing about WHICH firmware it flashed, which is the whole point of the
    section.
    """
    src = _read("gate_check.sh")
    assert "GATE_RELEASE" in src, "gate_check.sh has no release mode"
    assert "releases/download/$REL/" in src, (
        "release mode does not pin the leg's download to the tag being gated")
    assert re.search(r'\[ "\$noprov" = 0 \]', src), \
        "the pass no longer requires every profile to have provenance"


def test_a_verdict_does_not_name_the_firmware_with_a_pair_of_expansions():
    """`${REL:+release $REL}${REL:-the staged firmware}` prints BOTH halves.

    `:-` yields the variable when it is set rather than the default, so with
    GATE_RELEASE set the slice verdict read

        GATE: SLICE PASS (skid_steer) -- ten, all on release rc-20260925.1rc-20260925.1.

    Harmless to a grep and confusing to a person, which is the wrong way round
    for a line whose whole job is to be read. Name the firmware once, in a
    variable, and echo that.
    """
    for name in ("gate_check.sh", "gate_all.sh"):
        for i, line in enumerate(_read(name).split("\n"), 1):
            if not line.lstrip().startswith("echo"):
                continue
            assert not (":+" in line and ":-" in line), (
                f"{name}:{i} picks a name with a :+/:- pair, which expands to both")
