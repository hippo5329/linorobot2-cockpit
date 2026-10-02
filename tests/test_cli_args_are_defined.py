"""Every `args.<name>` a script reads is an argument its parser defines.

a3170ed (2026-09-27) dropped `--flash-attempt-timeout` from one_click_pipeline.py
while flash_firmware() still passed `args.flash_attempt_timeout` to the flasher.
Nothing failed until a board needed flashing: then every 1-Click run that had to
update the firmware died with AttributeError at [3/6] FLASH -- the whole Nav2 gate,
on every board, in its first minute. Argparse cannot catch it (the attribute is
read long after parsing) and no unit test called that path, so the check is
static: parse each script, collect the dests its add_argument calls define, and
require every args.<name> load to be one of them or assigned in the script.
"""
import ast
import glob
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _dest(call):
    for kw in call.keywords:
        if kw.arg == "dest" and isinstance(kw.value, ast.Constant):
            return kw.value.value
    flags = [a.value for a in call.args if isinstance(a, ast.Constant) and isinstance(a.value, str)]
    longs = [f for f in flags if f.startswith("--")]
    name = longs[0] if longs else (flags[0] if flags else None)
    return name.lstrip("-").replace("-", "_") if name else None


def _check(path):
    tree = ast.parse(open(path, encoding="utf-8").read(), filename=path)
    defined, read, assigned = set(), {}, set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and node.func.attr == "add_argument":
            d = _dest(node)
            if d:
                defined.add(d)
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "args":
            if isinstance(node.ctx, ast.Store):
                assigned.add(node.attr)
            else:
                read.setdefault(node.attr, node.lineno)
    return {a: ln for a, ln in read.items() if a not in defined | assigned}


def test_every_args_attribute_a_script_reads_is_defined():
    bad = {}
    for path in sorted(glob.glob(os.path.join(REPO, "scripts", "*.py"))):
        src = open(path, encoding="utf-8").read()
        if "add_argument(" not in src or "parse_args(" not in src:
            continue
        missing = _check(path)
        if missing:
            bad[os.path.basename(path)] = missing
    assert not bad, f"args.<name> read but never defined (name: first line): {bad}"
