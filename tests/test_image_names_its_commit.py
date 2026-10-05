"""The robot image says which commit it was built from (OCI revision label, /ws/BUILD_COMMIT).

`COPY . /ws` carries no .git. Without this an image could not say which tree it holds, and
a bench run could pair an older image's backend or Sim MCU with newer scripts unnoticed.
The release and CI builds pass LINO_GIT_REV=<github.sha>.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_the_final_stage_labels_and_records_its_commit():
    df = open(os.path.join(ROOT, "docker", "Dockerfile")).read()
    final = df[df.rindex("\nFROM "):]
    assert re.search(r"^ARG LINO_GIT_REV", final, re.M), "the final stage must declare the ARG itself"
    assert "LABEL org.opencontainers.image.revision=$LINO_GIT_REV" in final
    assert "/ws/BUILD_COMMIT" in final


def test_every_image_build_passes_the_commit():
    for wf in ("release.yml", "ci.yml"):
        assert "LINO_GIT_REV=${{ github.sha }}" in open(os.path.join(ROOT, ".github", "workflows", wf)).read(), wf


def test_a_release_image_fetches_its_own_release():
    """VERSION in a tag build is the tag; the tree's `<date>-dev` resolved to the NEWEST release."""
    df = open(os.path.join(ROOT, "docker", "Dockerfile")).read()
    final = df[df.rindex("\nFROM "):]
    assert re.search(r"^ARG LINO_VERSION", final, re.M)
    assert '> /ws/VERSION' in final
    wf = open(os.path.join(ROOT, ".github", "workflows", "release.yml")).read()
    assert "LINO_VERSION=${{ github.ref_type == 'tag' && github.ref_name || '' }}" in wf
