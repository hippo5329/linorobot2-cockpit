"""Every numeric character reference in the frontend is well formed.

The Tailwind palette pass rewrote colour literals by pattern and read the
emoji reference `&#128193;` (a folder) as the colour #128193, turning seven
file-picker buttons into the text `&#0e7490;`. A decimal reference is digits
only and a hex one starts with `x`; anything else renders as garbage.
"""
import glob
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF = re.compile(r"&#([^;\s]{1,10});")


def test_numeric_character_references_are_decimal_or_hex():
    bad = []
    for path in sorted(glob.glob(os.path.join(ROOT, "web", "frontend", "*.html"))
                       + glob.glob(os.path.join(ROOT, "web", "frontend", "*.js"))):
        text = open(path, encoding="utf-8").read()
        for m in REF.finditer(text):
            body = m.group(1)
            if not (body.isdigit() or re.fullmatch(r"[xX][0-9a-fA-F]+", body)):
                line = text.count("\n", 0, m.start()) + 1
                bad.append(f"{os.path.basename(path)}:{line}: {m.group(0)}")
    assert not bad, "malformed character references: " + ", ".join(bad)
