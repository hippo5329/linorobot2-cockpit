#!/usr/bin/env python3
"""
cockpit_paths.py — where the cockpit keeps things, answered in one place.

The cockpit repo is code and reference material; the user's robots are DATA
and live outside it, in their own git repository:

    ~/linorobot2-config/            (override: COCKPIT_CONFIG_DIR)
        rover_pico2_config.yaml     one robot per file
        secrets.yaml                Wi-Fi / addresses, gitignored there too
        .gitignore

Keeping them apart is what makes the firmware's `git=` stamp mean something:
`firmware/common/build_stamp.py` reads the COCKPIT tree's revision, so editing
or committing a robot config never changes what an image reports, and pulling
a new cockpit never touches a robot.

The repo ships reference robots in `config/reference/`. The first time the
config directory is needed and is empty, they are copied in and the directory
is turned into a git repository, so the user's edits are versioned from the
first save. Nothing here ever writes into `config/reference/`.

Every script, launcher and the supervisor resolve the directory through this
module. Do not build the path anywhere else.
"""
import os
import shutil
import subprocess

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REFERENCE_CONFIG_DIR = os.path.join(REPO_ROOT, "config", "reference")
SECRETS_EXAMPLE_PATH = os.path.join(REPO_ROOT, "config", "secrets.yaml.example")
FASTDDS_PROFILE = os.path.join(REPO_ROOT, "config", "fastdds_service_qos.xml")
DEFAULT_ROBOT = "rover_pico2"

_CONFIG_GITIGNORE = """# Credentials never leave this machine.
secrets.yaml
.cockpit_token
# Which robot the supervisor has open: this machine's state, not the robot's.
.active_robot
# Derived from the configs at every bringup (the URDF); regenerated, never edited.
generated/
*.bak
*.orig
"""

# Lines a directory seeded by an older cockpit is missing. Appended, never
# rewritten: the user's own additions stay.
_GITIGNORE_REQUIRED = (".cockpit_token", ".active_robot", "generated/")


def config_dir() -> str:
    """The user's robot config directory (not created here; see ensure_config_dir)."""
    raw = os.environ.get("COCKPIT_CONFIG_DIR") or "~/linorobot2-config"
    return os.path.abspath(os.path.expanduser(raw))


def generated_dir(directory: str = None) -> str:
    """<config dir>/generated: files derived from a config (the URDF). Gitignored
    there, because they are a function of the config, not a source."""
    d = os.path.join(directory or config_dir(), "generated")
    os.makedirs(d, exist_ok=True)
    return d


def secrets_path() -> str:
    return os.path.join(config_dir(), "secrets.yaml")


def robot_config_files(directory: str = None) -> list:
    """Every `<robot>_config.yaml` in the config directory, sorted."""
    d = directory or config_dir()
    if not os.path.isdir(d):
        return []
    return [os.path.join(d, f) for f in sorted(os.listdir(d))
            if f.endswith(("_config.yaml", "_config.yml")) and not f.startswith("secrets")]


def robot_config_path(robot: str = None, directory: str = None) -> str:
    """`<dir>/<robot>_config.yaml`, or the default robot, or whichever exists.

    Raises SystemExit when `robot` was named and no such file exists: a name
    that silently resolves to another robot's file is how one robot's controller
    ends up bolted onto another robot's wiring.
    """
    d = directory or config_dir()
    if robot:
        for cand in (os.path.join(d, f"{robot}_config.yaml"), os.path.join(d, robot)):
            if os.path.isfile(cand):
                return cand
        raise SystemExit(f"No config for robot '{robot}' (expected {d}/{robot}_config.yaml)")
    preferred = os.path.join(d, f"{DEFAULT_ROBOT}_config.yaml")
    if os.path.isfile(preferred):
        return preferred
    files = robot_config_files(d)
    if files:
        return files[0]
    return preferred


def ensure_config_dir(quiet: bool = False) -> str:
    """Create and seed the config directory if it does not hold a robot yet.

    Seeding copies `config/reference/*.yaml` and `secrets.yaml.example` (as
    `secrets.yaml`) and runs `git init` + an initial commit when git is
    available and the directory is not already inside a repository. Idempotent.
    """
    d = config_dir()
    os.makedirs(d, exist_ok=True)
    seeded = []
    if not robot_config_files(d):
        for name in sorted(os.listdir(REFERENCE_CONFIG_DIR)) if os.path.isdir(REFERENCE_CONFIG_DIR) else []:
            if name.endswith((".yaml", ".yml")):
                shutil.copy2(os.path.join(REFERENCE_CONFIG_DIR, name), os.path.join(d, name))
                seeded.append(name)
    sec = os.path.join(d, "secrets.yaml")
    if not os.path.isfile(sec) and os.path.isfile(SECRETS_EXAMPLE_PATH):
        shutil.copy2(SECRETS_EXAMPLE_PATH, sec)
        try:
            os.chmod(sec, 0o600)
        except OSError:
            pass
        seeded.append("secrets.yaml")
    gi = os.path.join(d, ".gitignore")
    if not os.path.isfile(gi):
        with open(gi, "w") as fh:
            fh.write(_CONFIG_GITIGNORE)
    else:
        # A directory seeded before the access token, the active-robot marker
        # or the generated URDF existed must not commit any of them.
        with open(gi) as fh:
            lines = fh.read().splitlines()
        missing = [x for x in _GITIGNORE_REQUIRED if x not in lines]
        if missing:
            with open(gi, "a") as fh:
                fh.write("".join(f"{x}\n" for x in missing))
    if seeded:
        _git_init(d)
        if not quiet:
            print(f"[cockpit] seeded {d} with {len(seeded)} file(s) from config/reference "
                  f"(set COCKPIT_CONFIG_DIR to use another directory)")
    return d


def _git(d: str, *args, timeout: int = 15):
    """Run one git command in the config dir. Returns None when git is missing."""
    git = shutil.which("git")
    if not git:
        return None
    try:
        return subprocess.run([git, "-C", d, *args], capture_output=True, text=True,
                              timeout=timeout)
    except Exception:
        return None


def git_state(directory: str = None) -> dict:
    """What the config repo looks like: branch, last commit, and what is uncommitted.

    The config directory is the USER's own git repository (seeded and `git init`ed
    by ensure_config_dir), and until now nothing in the cockpit showed that or let
    them record a change from the UI -- so a robot tuned through Config Studio
    accumulated uncommitted edits nobody could see.
    """
    d = directory or config_dir()
    out = {"path": d, "is_repo": False, "git_available": shutil.which("git") is not None,
           "branch": "", "last_commit": "", "changes": [], "dirty": False}
    if not out["git_available"] or not os.path.isdir(d):
        return out
    inside = _git(d, "rev-parse", "--is-inside-work-tree")
    if inside is None or inside.returncode != 0 or inside.stdout.strip() != "true":
        return out
    out["is_repo"] = True
    br = _git(d, "rev-parse", "--abbrev-ref", "HEAD")
    if br is not None and br.returncode == 0:
        out["branch"] = br.stdout.strip()
    last = _git(d, "log", "-1", "--pretty=%h %ad %s", "--date=short")
    if last is not None and last.returncode == 0:
        out["last_commit"] = last.stdout.strip()
    st = _git(d, "status", "--porcelain")
    if st is not None and st.returncode == 0:
        # "XY path" -- keep both, the UI shows the status letters.
        out["changes"] = [ln.rstrip() for ln in st.stdout.splitlines() if ln.strip()]
    out["dirty"] = bool(out["changes"])
    return out


def git_commit(message: str, directory: str = None) -> dict:
    """Stage everything the repo does not ignore and commit it.

    Secrets stay out by .gitignore, not by a rule here: `secrets.yaml`, the access
    token, the active-robot marker and `generated/` are all ignored, so `add -A`
    cannot pick them up.
    """
    d = directory or config_dir()
    state = git_state(d)
    if not state["git_available"]:
        return {"status": "error", "detail": "git is not installed on this machine."}
    if not state["is_repo"]:
        return {"status": "error", "detail": f"{d} is not a git repository."}
    if not state["dirty"]:
        return {"status": "noop", "detail": "Nothing to commit; the config directory is clean.",
                "state": state}
    add = _git(d, "add", "-A")
    if add is None or add.returncode != 0:
        return {"status": "error", "detail": (add.stderr.strip() if add else "git add failed")}
    # The user's own identity when they have one; ours only as the fallback, so a
    # commit is never refused for "Please tell me who you are".
    ident = _git(d, "config", "user.email")
    pre = []
    if ident is None or ident.returncode != 0 or not ident.stdout.strip():
        pre = ["-c", "user.name=linorobot2-cockpit", "-c", "user.email=cockpit@localhost"]
    res = _git(d, *pre, "commit", "-q", "-m", message)
    if res is None or res.returncode != 0:
        return {"status": "error",
                "detail": ((res.stderr or res.stdout).strip() if res else "git commit failed")}
    return {"status": "ok", "detail": message, "state": git_state(d)}


def _git_init(d: str) -> None:
    git = shutil.which("git")
    if not git:
        return
    try:
        inside = subprocess.run([git, "-C", d, "rev-parse", "--is-inside-work-tree"],
                                capture_output=True, text=True, timeout=10)
        if inside.returncode == 0 and inside.stdout.strip() == "true":
            return
        subprocess.run([git, "-C", d, "init", "-q", "-b", "main"], check=True, timeout=10)
        subprocess.run([git, "-C", d, "add", "-A"], check=True, timeout=10)
        subprocess.run([git, "-C", d, "-c", "user.name=linorobot2-cockpit",
                        "-c", "user.email=cockpit@localhost", "commit", "-q",
                        "-m", "Seed robot configs from linorobot2-cockpit config/reference"],
                       check=False, timeout=10)
    except Exception:
        pass


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ensure", action="store_true", help="create and seed the directory if needed")
    a = ap.parse_args()
    print(ensure_config_dir() if a.ensure else config_dir())
