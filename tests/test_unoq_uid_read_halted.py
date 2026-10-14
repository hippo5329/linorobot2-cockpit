"""The UNO Q's UID is read with the core halted (scripts/unoq_swd.py identify).

Read while the core ran, the UID came back different from one flash to the next (mc200,
2026-10-14), and the flash guard compares it to tell this board from another.
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import unoq_swd  # noqa: E402


def test_identify_halts_before_reading_and_resumes(monkeypatch):
    seen = []

    def fake_run(cmds, timeout=120):
        seen.extend(cmds)
        return ("0xe0044000: 30076482 \n"
                "0x0bfa0700: 0018003c 58435005 20393543 \n")
    monkeypatch.setattr(unoq_swd, "run", fake_run)
    dev, uid = unoq_swd.identify()
    assert dev == 0x482 and uid == "20393543584350050018003C"
    reads = [i for i, c in enumerate(seen) if "mdw" in c]
    assert seen.index("halt") < min(reads) < max(reads) < seen.index("resume")
