"""The UNO Q build: micro-ROS waits for Zephyr's generated headers.

rcutils is compiled in an external project against the POSIX headers Zephyr provides,
which include the GENERATED zephyr/syscall_list.h. Nothing ordered the two, so a loaded
build ran them side by side and failed "zephyr/syscall_list.h: No such file or directory"
(the rc-20261006.2 campaign's on-board lyrical build, 2026-10-06).
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATCH = os.path.join(ROOT, "firmware", "zephyr", "unoq", "patches", "micro_ros_zephyr_module-zephyr44-unoq.patch")


def test_libmicroros_depends_on_the_generated_headers():
    added = [ln[1:].strip() for ln in open(PATCH) if ln.startswith("+") and not ln.startswith("+++")]
    assert "add_dependencies(libmicroros_project zephyr_generated_headers ${SYSCALL_LIST_H_TARGET})" in added
