"""The header must be writable by whoever writes it second.

The cockpit backend runs as the container user; the one-click pipeline runs as
container-root. `open(path, "w")` needs permission on the EXISTING FILE, so
whichever wrote the header last left it unwritable by the other -- and the web
UI surfaced that as a raw Python traceback out of gen_firmware_header.main(),
on every box in the rig:

    ❌ Failed to generate firmware header: Traceback (most recent call last):
      File "/ws/scripts/gen_firmware_header.py", line 957, in main
        with open(args.out, "w") as f:

Renaming a temp file over the target needs permission on the DIRECTORY, which
both users have.
"""
import os
import stat
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import gen_firmware_header  # noqa: E402


def test_replaces_a_file_this_user_cannot_open_for_writing(tmp_path):
    target = tmp_path / "lino_base_config.h"
    target.write_text("// written by the other user\n")
    # Read-only for everyone: open(w) raises, rename does not.
    os.chmod(target, stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH)

    if os.geteuid() == 0:
        pytest.skip("root ignores the mode bits this test depends on")

    with pytest.raises(PermissionError):
        open(target, "w")

    gen_firmware_header.write_text_atomic(str(target), "// regenerated\n")
    assert target.read_text() == "// regenerated\n"


def test_creates_the_directory_and_the_file(tmp_path):
    target = tmp_path / "nested" / "deeper" / "lino_base_config.h"
    gen_firmware_header.write_text_atomic(str(target), "// fresh\n")
    assert target.read_text() == "// fresh\n"


def test_leaves_no_temp_files_behind(tmp_path):
    target = tmp_path / "lino_base_config.h"
    gen_firmware_header.write_text_atomic(str(target), "// one\n")
    gen_firmware_header.write_text_atomic(str(target), "// two\n")
    leftovers = [p.name for p in tmp_path.iterdir() if p.name != target.name]
    assert leftovers == [], f"temp files left in the output directory: {leftovers}"


def test_the_generator_uses_it():
    """main() must not go back to opening args.out directly.

    Checked against the CODE, not the source text: the helper's docstring
    quotes the old call on purpose, and a plain substring search matches that.
    """
    import ast

    src = open(os.path.join(REPO_ROOT, "scripts", "gen_firmware_header.py")).read()
    tree = ast.parse(src)

    # Drop every docstring, then look at what is left.
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Module)):
            body = getattr(node, "body", [])
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                node.body = body[1:]
    code = ast.unparse(tree)

    assert "write_text_atomic(args.out, header_content)" in code
    assert "open(args.out, 'w')" not in code and 'open(args.out, "w")' not in code, (
        "the header is being opened directly again, which fails for whichever "
        "user did not write it last"
    )
