"""Every directory CMakeLists.txt installs must exist in a clean checkout.

rc-20260919 shipped both robot images with a half-built ROS workspace because
`install(DIRECTORY ... maps ...)` named a gitignored directory. In a developer's
tree maps/ exists from an earlier run and the build is green; in CI's clean
checkout it does not, and install(DIRECTORY) on a missing directory is a hard
CMake error. That aborted the install partway through -- before rviz and before
everything ament_package() generates -- and aborted the vendored LiDAR driver
with it, so the published images had no local_setup.bash and no
ldlidar_stl_ros2 node, and bringup could not start on any board or distro.

The build swallowed it (colcon piped into tail) and the guard behind it checked
a file colcon writes even on total failure, so nothing downstream noticed. This
test checks the one condition that actually distinguishes the two trees.
"""
import os
import re
import subprocess

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _installed_directories():
    with open(os.path.join(REPO_ROOT, "CMakeLists.txt")) as fh:
        text = fh.read()
    block = re.search(r"install\(\s*DIRECTORY\s+([^)]*?)\s+DESTINATION", text, re.S)
    assert block, "no install(DIRECTORY ... DESTINATION ...) in CMakeLists.txt"
    return [d for d in block.group(1).split() if not d.startswith("$")]


def test_installed_directories_exist():
    for d in _installed_directories():
        assert os.path.isdir(os.path.join(REPO_ROOT, d)), (
            f"CMakeLists.txt installs '{d}', which does not exist"
        )


def test_installed_directories_are_not_gitignored():
    """The condition that differs between a developer tree and CI's checkout."""
    dirs = _installed_directories()
    proc = subprocess.run(
        ["git", "check-ignore"] + dirs,
        cwd=REPO_ROOT, capture_output=True, text=True,
    )
    # check-ignore exits 0 and prints the paths it WOULD ignore; 1 means none.
    ignored = [line for line in proc.stdout.split("\n") if line.strip()]
    assert not ignored, (
        "CMakeLists.txt installs gitignored directories, which do not exist in a "
        f"clean checkout and make install(DIRECTORY) a hard error: {ignored}"
    )


def test_installed_directories_are_tracked():
    """A directory with no tracked file is absent from a fresh clone."""
    for d in _installed_directories():
        out = subprocess.run(
            ["git", "ls-files", "--", d],
            cwd=REPO_ROOT, capture_output=True, text=True,
        ).stdout.strip()
        assert out, f"CMakeLists.txt installs '{d}', which has no tracked files"
