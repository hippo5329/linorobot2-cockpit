#!/usr/bin/env python3
"""Write a robot config back without losing its comments.

yaml.safe_dump throws away every comment in the file, and a robot's config carries the
user's reasons: its DDS domain, its LiDAR mask, its BMS cutoff (the TS100's, twice on
2026-10-16 -- drivetrain_report --write, then the web UI's distro switch). Values are
rewritten where they stand, keys the file lacks go under their parent, and the result
must load back equal to the data, or the caller gets None and dumps the whole file.
"""
import re

import yaml

_KV_LINE = re.compile(r'^(?P<indent> *)(?P<key>[A-Za-z0-9_.:/+~-]+):(?P<sp> *)'
                      r'(?P<val>(?:"[^"]*"|\'[^\']*\'|[^#\n])*?)(?P<comment> *#.*)?$')
_ITEM_LINE = re.compile(r'^(?P<indent> *)- (?P<val>(?:"[^"]*"|\'[^\']*\'|[^#\n])*?)(?P<comment> *#.*)?$')
_MISSING = object()


def _at(params, path):
    node = params
    for key in path:
        if not isinstance(node, dict) or key not in node:
            return _MISSING
        node = node[key]
    return node


def _scalar_text(value) -> str:
    return yaml.safe_dump(value).splitlines()[0]


def edit_in_place(text: str, params: dict):
    """`text` with each scalar (and each item of a scalar block list) that differs in
    `params` rewritten where it stands, keys it lacks added under their parent, comments
    and layout untouched -- or None when the result does not load back as `params` (a
    list that changed length, say): then nothing in place can say it."""
    result = _edited(text, params)
    try:
        return result if yaml.safe_load(result) == params else None
    except yaml.YAMLError:
        return None


def _edited(text: str, params: dict) -> str:
    # A line is rewritten only when it holds its whole value: a plain scalar folded
    # over two lines parses short from its first one (gendrv's description).
    try:
        have = yaml.safe_load(text) or {}
    except yaml.YAMLError:
        return text
    out, stack, lst = [], [], None        # lst: [path, indent of its items, index]
    for line in text.splitlines(keepends=True):
        body = line.rstrip("\n")
        item = _ITEM_LINE.match(body)
        if item and lst is not None and not item.group("val").startswith("- "):
            ind = len(item.group("indent"))
            if lst[1] is None:
                lst[1] = ind
            if ind == lst[1]:
                new, was = _at(params, lst[0]), _at(have, lst[0])
                try:
                    old = yaml.safe_load(item.group("val"))
                except yaml.YAMLError:
                    old = _MISSING
                if (isinstance(new, list) and isinstance(was, list) and lst[2] < len(new)
                        and lst[2] < len(was) and was[lst[2]] == old
                        and not isinstance(new[lst[2]], (dict, list))
                        and not isinstance(old, (dict, list)) and new[lst[2]] != old):
                    body = (f"{item.group('indent')}- {_scalar_text(new[lst[2]])}"
                            f"{item.group('comment') or ''}")
                    line = body + ("\n" if line.endswith("\n") else "")
                lst[2] += 1
            out.append(line)
            continue
        kv = _KV_LINE.match(body)
        if kv:
            ind = len(kv.group("indent"))
            while stack and stack[-1][0] >= ind:
                stack.pop()
            path = tuple(k for _, k in stack) + (kv.group("key"),)
            val = (kv.group("val") or "").rstrip()
            if val == "":
                stack.append((ind, kv.group("key")))
                lst = [path, None, 0]
            else:
                lst = None
                new = _at(params, path)
                try:
                    old = yaml.safe_load(val)
                except yaml.YAMLError:
                    old = _MISSING
                if (new is not _MISSING and old == _at(have, path)
                        and not isinstance(new, (dict, list))
                        and not isinstance(old, (dict, list)) and new != old):
                    body = (f"{kv.group('indent')}{kv.group('key')}:{kv.group('sp') or ' '}"
                            f"{_scalar_text(new)}{kv.group('comment') or ''}")
                    line = body + ("\n" if line.endswith("\n") else "")
        out.append(line)
    return _insert_missing("".join(out), params)


def _key_lines(lines) -> dict:
    """path -> (line index, indent) of every mapping key line."""
    found, stack = {}, []
    for i, line in enumerate(lines):
        kv = _KV_LINE.match(line.rstrip("\n"))
        if not kv:
            continue
        ind = len(kv.group("indent"))
        while stack and stack[-1][0] >= ind:
            stack.pop()
        path = tuple(k for _, k in stack) + (kv.group("key"),)
        found[path] = (i, ind)
        if not (kv.group("val") or "").strip():
            stack.append((ind, kv.group("key")))
    return found


def _missing(new, old, path=()):
    """(parent path, key, value) for each key in `new` that `old` lacks, outermost only."""
    for key, value in new.items():
        if key not in old:
            yield path, key, value
        elif isinstance(value, dict) and isinstance(old[key], dict):
            yield from _missing(value, old[key], path + (key,))


def _insert_missing(text: str, params: dict) -> str:
    """Keys `params` has and the file does not (a first step_response), written at the
    end of their parent's block -- the file's comments stay where they were."""
    try:
        have = yaml.safe_load(text) or {}
    except yaml.YAMLError:
        return text
    if not isinstance(have, dict):
        return text
    lines = text.splitlines(keepends=True)
    if lines and not lines[-1].endswith("\n"):
        lines[-1] += "\n"
    # bottom-up, so the line numbers found for the earlier insertions still hold
    places = []
    keys = _key_lines(lines)
    for parent, key, value in _missing(params, have):
        if parent and parent not in keys:
            continue                     # left to the check: the result will not load equal
        if parent:
            start, pind = keys[parent]
            end, ind = start, None
            for j in range(start + 1, len(lines)):
                body = lines[j].rstrip("\n")
                if not body.strip() or body.lstrip().startswith("#"):
                    continue
                lead = len(body) - len(body.lstrip(" "))
                if lead <= pind:
                    break
                end = j
                ind = lead if ind is None else min(ind, lead)
            ind = pind + 2 if ind is None else ind
        else:
            end, ind = len(lines) - 1, 0
        block = yaml.safe_dump({key: value}, sort_keys=False, default_flow_style=False)
        places.append((end, "".join(" " * ind + l + "\n" for l in block.splitlines())))
    for end, block in sorted(places, key=lambda p: -p[0]):
        lines.insert(end + 1, block)
    return "".join(lines)


def write_config(path: str, params: dict) -> bool:
    """Write `params` to `path`, keeping its comments; True when they were kept."""
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    kept = edit_in_place(text, params)
    with open(path, "w", encoding="utf-8") as fh:
        if kept is not None:
            fh.write(kept)
        else:
            yaml.safe_dump(params, fh, sort_keys=False)
    return kept is not None
