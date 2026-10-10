"""rclpy's logger has `warning`, not `warn`: Lyrical's RcutilsLogger dropped the alias, and
test_nav2_goal.py died on it (`'RcutilsLogger' object has no attribute 'warn'`) the first time
it ran with no /cmd_vel publisher. A warning path runs rarely, so nothing else catches it."""
import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
WARN = re.compile(r"(get_logger\(\)|\blogger)\.warn\(")


def test_no_ros_logger_calls_warn():
    hits = []
    for d in ("scripts", "launchers", "backend", "ros2_ws"):
        for p in (REPO_ROOT / d).rglob("*.py") if (REPO_ROOT / d).is_dir() else ():
            for i, line in enumerate(p.read_text(errors="replace").splitlines(), 1):
                if WARN.search(line):
                    hits.append(f"{p.relative_to(REPO_ROOT)}:{i}: {line.strip()}")
    assert not hits, "use .warning(), not .warn():\n" + "\n".join(hits)
