"""Every element the scripts bind at load time exists when they run.

The scripts are classic, loaded at the end of <body>, and bind their handlers
with `document.getElementById(id)?.addEventListener(...)` as they execute. Two
modals sat AFTER the script tags, so those lookups returned null and the `?.`
swallowed it: the port-conflict modal's close, Ignore and Release buttons, and
the rootless-Docker guide's, did nothing at all (every-control browser walk,
2026-09-25: "dialog will not close").
"""
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_no_element_with_an_id_follows_the_scripts():
    html = open(os.path.join(REPO_ROOT, "web", "frontend", "index.html")).read()
    after = html[html.index("<script"):]
    late = re.findall(r'<(?!script)[a-z]+[^>]*\sid="([^"]+)"', after)
    assert not late, f"markup after the first <script> is unbound at load: {late}"
