"""The Fast DDS profiles must exist, parse, carry both fixes, and reach every launch path.

Two nav2 failures with no error of their own live behind config/fastdds_service_qos.xml
(its own comment has the mechanisms): the 100 ms service-reply race in rmw_fastrtps, and
Fast DDS 3.x leaving one endpoint unmatched forever after a lost TypeLookup reply, which
cost bt_navigator's activate transition on 8 of 13 lyrical runs on 2026-09-19.

The second fix is only as good as its delivery. Until that day only one_click_pipeline.py
exported FASTDDS_DEFAULT_PROFILES_FILE, so everything the web UI started -- the path a
user actually takes -- ran without any of it. These tests pin the file's content and
every place that has to point at it.
"""
import os
import re
import xml.etree.ElementTree as ET

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROFILE = os.path.join(REPO_ROOT, "config", "fastdds_service_qos.xml")
NS = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}


def _read(*parts):
    with open(os.path.join(REPO_ROOT, *parts)) as fh:
        return fh.read()


def test_profile_parses_and_matches_types_by_name():
    root = ET.parse(PROFILE).getroot()
    participants = root.findall(".//f:participant", NS)
    defaults = [p for p in participants if p.get("is_default_profile") == "true"]
    assert len(defaults) == 1, "exactly one default participant profile"
    props = {p.find("f:name", NS).text: p.find("f:value", NS).text
             for p in defaults[0].findall(".//f:property", NS)}
    assert props.get("fastdds.type_propagation") == "registration_only", (
        "type matching must fall back to the type name, or Fast DDS 3.x can leave one "
        "endpoint pending on a lost TypeLookup reply and nav2 never activates")


def test_profile_keeps_the_service_reply_ceiling():
    root = ET.parse(PROFILE).getroot()
    for kind in ("data_writer", "data_reader"):
        prof = [e for e in root.findall(f".//f:{kind}", NS) if e.get("profile_name") == "service"]
        assert len(prof) == 1, f"{kind} profile named 'service' (rmw_fastrtps looks it up by that name)"
        sec = prof[0].find(".//f:max_blocking_time/f:sec", NS)
        assert sec is not None and int(sec.text) >= 10, f"{kind}: reply-writer ceiling of at least 10 s"


def test_every_launch_path_exports_the_profile():
    var = "FASTDDS_DEFAULT_PROFILES_FILE"
    # The image: container-wide, so the supervisor, rosbridge and the web UI's launches see it.
    assert re.search(rf"^\s*{var}=/ws/config/fastdds_service_qos\.xml", _read("docker", "Dockerfile"), re.M), \
        "docker/Dockerfile must ENV the profile for the whole container"
    assert re.search(rf"^\s*-\s*{var}=/ws/config/fastdds_service_qos\.xml", _read("docker-compose.yml"), re.M), \
        "docker-compose.yml must set the profile in the cockpit service environment"
    # Native runs: both env builders export it (the pipeline always, the runner when unset).
    assert var in _read("scripts", "one_click_pipeline.py")
    runners = _read("web", "backend", "runners.py")
    assert var in runners and "fastdds_service_qos.xml" in runners, \
        "runners.py builds the ROS env for everything the web UI starts; it must export the profile too"
