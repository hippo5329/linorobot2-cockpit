# Stamp the image with where it came from: a build date and the 7-character git
# revision of the tree it was built from.
#
# Why this is not cosmetic. Everything else about a board is now data -- pins,
# transport, credentials and the application all live in the env partition and
# can be rewritten without a compiler (AGENTS.md §10) -- which leaves exactly one
# question the env cannot answer: *which build is on this board?* Without an
# answer the only safe assumption is "not the one I just made", so every run
# reflashes, and reflashing is the one step that can brick an assembled robot.
# The stamp is what lets the Cockpit skip it: the board says `git=6aa607f` in its
# boot banner, the host compares it with its own HEAD, and a matching board is
# left alone.
#
# Three sources, in order:
#   1. `git rev-parse` in the project tree -- a real clone (the host workstation).
#   2. firmware/.git_rev -- a one-line file written by whatever seeded the tree.
#      The build boxes get their workspace as `git archive HEAD`, which carries
#      no .git at all, so without this every board built in a box would report
#      `git=unknown` and the comparison above would never match.
#   3. "unknown" -- an honest answer. It must never be a plausible-looking lie:
#      a wrong revision is worse than no revision, because it reads as a match.
#
# A dirty tree is flagged with a trailing '+': `git=6aa607f+` says the build
# carries edits that revision does not, which is exactly the case where a
# skipped reflash would leave stale firmware on the board.
import datetime
import os
import subprocess

Import("env")  # noqa: F821  (SCons injects this)

project_dir = env["PROJECT_DIR"]          # firmware/
repo_root = os.path.dirname(project_dir)


def _git(*args):
    try:
        out = subprocess.run(["git", "-C", repo_root, *args],
                             capture_output=True, text=True, timeout=10)
        if out.returncode == 0:
            return out.stdout.strip()
    except Exception:
        pass
    return ""


def git_rev():
    rev = _git("rev-parse", "--short=7", "HEAD")
    if rev:
        # --porcelain over the whole repo, not just firmware/: a config change
        # reaches the image through the generated header.
        return rev + ("+" if _git("status", "--porcelain") else "")
    stamp = os.path.join(project_dir, ".git_rev")
    if os.path.isfile(stamp):
        with open(stamp) as fh:
            words = fh.read().split()
        if words:
            return words[0][:8]
    return "unknown"


rev = git_rev()
date = datetime.date.today().isoformat()

# The ROS 2 distro is the one thing about an image the env partition can never
# carry (AGENTS.md §10): board_microros_distro selects the precompiled micro_ros
# library the firmware links against, so it is fixed at link time. That makes it
# exactly the kind of fact the banner exists to report -- and without it the
# banner cannot tell a jazzy image from a lyrical one built at the same
# revision, so `mcu_probe.py` would answer `up_to_date` for a board carrying the
# wrong half of the release matrix. The failure that follows names nothing: the
# agent starts, the board boots, and the two never discover each other.
distro = env.GetProjectOption("board_microros_distro", "") or "unknown"

env.Append(CPPDEFINES=[
    ("FW_GIT_REV", env.StringifyMacro(rev)),
    ("FW_BUILD_DATE", env.StringifyMacro(date)),
    ("FW_ROS_DISTRO", env.StringifyMacro(distro)),
])
print(f"[build_stamp] linorobot2_hardware built={date} git={rev} distro={distro}")
