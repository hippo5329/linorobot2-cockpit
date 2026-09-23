"""RViz on the host's screen, out of the robot image, and what makes it see.

It is the SAME image as the cockpit, and that was measured rather than assumed.
The robot image already carries every rviz LIBRARY as a dependency of the stack
-- rviz_common, rviz_default_plugins, rviz_rendering, rviz_ogre_vendor,
nav2_rviz_plugins -- and Qt, libgl1-mesa-dri, libxkbcommon-x11-0 and
libxcb-cursor0 with them. Adding the rviz2 EXECUTABLE reports one new package,
20164 bytes to download, 122 KB installed, and takes the published lyrical
image from 5.43 to 5.44 GB. A viewer image of its own measured 1.63 GB and two
more tags per distro.

A viewer that comes up empty is the normal failure, and every cause of it is
invisible: wrong ROS_DOMAIN_ID, wrong RMW, no Fast DDS profile, no shared
memory. measure-from-inside-the-stack-env is the rule these tests pin down --
a reader without the stack's profile cannot see a stack that has it, and gets
silence rather than an error.
"""
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMPOSE = os.path.join(ROOT, "docker-compose.yml")
DOCKERFILE = os.path.join(ROOT, "docker", "Dockerfile")
ENTRY = os.path.join(ROOT, "docker", "rviz-entrypoint.sh")


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _service():
    with open(COMPOSE, encoding="utf-8") as fh:
        return yaml.safe_load(fh)["services"]["rviz"]


def test_the_viewer_is_not_started_by_compose_up():
    """`docker compose up` on a robot must not try to open a window."""
    assert _service()["profiles"] == ["rviz"]


def test_it_joins_the_graph_the_way_the_cockpit_does():
    svc = _service()
    cockpit = yaml.safe_load(_read(COMPOSE))["services"]["cockpit"]
    # discovery is multicast, and the costmaps need the shared-memory transport
    assert svc["network_mode"] == "host" == cockpit["network_mode"]
    assert svc["ipc"] == "host" == cockpit["ipc"]
    assert any(str(v).startswith("/dev/shm:/dev/shm") for v in svc["volumes"])


def test_it_reads_the_same_fast_dds_profile_as_the_stack():
    """The one that fails silently. Without it the window is simply empty."""
    svc, cockpit = _service(), yaml.safe_load(_read(COMPOSE))["services"]["cockpit"]
    env = dict(e.split("=", 1) for e in svc["environment"])
    cenv = dict(e.split("=", 1) for e in cockpit["environment"] if "=" in e)
    # the same file at the same path, and it comes from the image both use
    assert env["FASTDDS_DEFAULT_PROFILES_FILE"] == cenv["FASTDDS_DEFAULT_PROFILES_FILE"]
    # and the same domain and RMW, defaulted identically to the cockpit service
    assert env["ROS_DOMAIN_ID"] == cenv["ROS_DOMAIN_ID"]
    assert env["RMW_IMPLEMENTATION"] == cenv["RMW_IMPLEMENTATION"]


def test_it_is_the_same_image_as_the_cockpit():
    """Measured, not assumed -- see the module docstring."""
    svc, cockpit = _service(), yaml.safe_load(_read(COMPOSE))["services"]["cockpit"]
    assert svc["image"] == cockpit["image"]
    assert "build" not in svc, "the viewer must not be a second image to build"
    assert svc["entrypoint"] == ["/ws/docker/rviz-entrypoint.sh"]


def test_the_robot_image_installs_the_executable_it_was_missing():
    df = _read(DOCKERFILE)
    assert "ros-${ROS_DISTRO}-rviz2" in df


def test_the_display_comes_from_the_host_and_the_cookie_with_it():
    svc = _service()
    env = dict(e.split("=", 1) for e in svc["environment"])
    assert env["DISPLAY"] == "${DISPLAY:-:0}"
    assert any("/tmp/.X11-unix:/tmp/.X11-unix" in str(v) for v in svc["volumes"])
    # XAUTHORITY is empty on many desktops; the default has to cover that
    assert any("${XAUTHORITY:-~/.Xauthority}" in str(v) for v in svc["volumes"])
    assert env["XAUTHORITY"] == "/run/xauth"


def test_qt_is_pinned_to_xcb_so_a_wayland_host_still_works():
    """A GNOME Wayland session serves X clients through Xwayland.

    Inheriting XDG_SESSION_TYPE=wayland makes Qt try the wayland plugin and
    exit with "could not load the Qt platform plugin" rather than falling back.
    """
    env = dict(e.split("=", 1) for e in _service()["environment"])
    assert env["QT_QPA_PLATFORM"] == "xcb"
    assert env["LIBGL_ALWAYS_SOFTWARE"] == "1"


def test_nothing_it_saves_lands_on_the_host_filesystem():
    """The constraint that makes this a feature: the cockpit does not write to
    the machine it runs on."""
    svc = _service()
    assert any(str(v).startswith("cockpit-rviz:/rviz") for v in svc["volumes"])
    assert "cockpit-rviz" in yaml.safe_load(_read(COMPOSE))["volumes"]
    # every host-path mount it has is read-only
    host_mounts = [str(v) for v in svc["volumes"]
                   if str(v).startswith("/") or str(v).startswith("./") or str(v).startswith("${")]
    assert all(m.endswith(":ro") or m.startswith("/dev/shm") for m in host_mounts), host_mounts


def test_the_entrypoint_says_what_it_attached_to_before_it_draws():
    e = _read(ENTRY)
    for k in ("DISPLAY=", "ROS_DOMAIN_ID=", "RMW=", "FASTDDS_PROFILES="):
        assert k in e, k
    assert "will not see a stack that uses it" in e


def test_the_shipped_configs_seed_the_volume_without_overwriting_edits():
    e = _read(ENTRY)
    assert '[ -e "$CONFIG_DIR/$(basename "$f")" ] || cp "$f" "$CONFIG_DIR/"' in e
    for name in ("teleop", "slam", "navigation"):
        assert os.path.exists(os.path.join(ROOT, "rviz", f"{name}.rviz")), name


def test_the_release_workflow_publishes_no_extra_viewer_image():
    """One image. A second one would be two more tags per distro to keep in
    step, and four more manifests once arm64 lands."""
    with open(os.path.join(ROOT, ".github", "workflows", "release.yml"), encoding="utf-8") as fh:
        wf = yaml.safe_load(fh)
    names = [m["name"] for m in wf["jobs"]["images"]["strategy"]["matrix"]["include"]]
    assert not [n for n in names if "rviz" in n], names
    assert not os.path.exists(os.path.join(ROOT, "docker", "Dockerfile.rviz"))
