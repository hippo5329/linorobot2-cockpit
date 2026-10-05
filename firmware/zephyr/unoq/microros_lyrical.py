#!/usr/bin/env python3
"""Prepare the micro-ROS Zephyr module's sources for a Lyrical build.

    microros_lyrical.py <module>/modules/libmicroros [<this script's path in the build>]

micro_ros_zephyr_module has branches up to kilted and none for lyrical, while the
board's agent must be the one in the lyrical image: micro-ROS is fixed at link time
and a jazzy client never holds a session with a lyrical agent. The PlatformIO
builds get lyrical from micro_ros_platformio (repositories.py / library_builder.py);
this applies the same choices to the Zephyr module's libmicroros.mk:

  * every repository on its `lyrical` branch, except rclc (`rolling`: ros2/rclc has
    no lyrical branch) and micro-CDR / Micro-XRCE-DDS-Client (`ros2`, as for jazzy);
  * the packages micro_ros_platformio ignores for lyrical get a COLCON_IGNORE;
  * colcon.meta adds RCUTILS_NO_PROCESS_SUPPORT (Zephyr has no fork);
  * the source patches micro_ros_platformio's patch_mcu_sources() applies, run after
    the clone and before colcon (this script, called again by the makefile with
    --patch-sources <src>).

Idempotent: a second run changes nothing.
"""
import os
import re
import sys

IGNORE = ["test_tracetools", "rcl_logging_spdlog", "rcl_logging_implementation",
          "rcl_yaml_param_parser", "rclc_examples", "lttngpy", "rmw_security_common",
          "rosidl_buffer_py", "test_rmw_implementation", "rosidl_buffer_backend_registry"]
BRANCH = {"rclc": "rolling"}           # repo dir -> branch where it is not `lyrical`
MARK = "# lino: lyrical sources"


def _rw(path, fn):
    if not os.path.exists(path):
        return False
    with open(path) as f:
        old = f.read()
    new = fn(old)
    if new != old:
        with open(path, "w") as f:
            f.write(new)
        return True
    return False


def patch_makefile(libmicroros, me=None):
    """`me`: this script's path as the makefile will see it (the build container's)."""
    mk = os.path.join(libmicroros, "libmicroros.mk")
    me = me or os.path.abspath(__file__)

    def edit(s):
        if MARK in s:
            return s

        def branch(m):
            d = m.group(2)
            return f"git clone -b {BRANCH.get(d, 'lyrical')} {m.group(1)} src/{d};"
        s = re.sub(r"git clone -b jazzy (\S+) src/(\S+);", branch, s)
        # After the source clone's last `touch`, before colcon: ignore list and patches.
        dev = "touch src/ament_cmake_ros/rmw_test_fixture/COLCON_IGNORE;"
        i = s.index(dev) + len(dev)
        s = s[:i] + f" \\\n\tpython3 {me} --patch-dev $(COMPONENT_PATH)/micro_ros_dev/src;" + s[i:]
        last = s.rindex("touch src/rcl_interfaces/test_msgs/COLCON_IGNORE;")
        end = last + len("touch src/rcl_interfaces/test_msgs/COLCON_IGNORE;")
        hook = f" \\\n\tpython3 {me} --patch-sources $(UROS_DIR)/src; {MARK}"
        return s[:end] + hook + s[end:]
    meta = _rw(os.path.join(libmicroros, "colcon.meta"), _meta)
    return _rw(mk, edit) or meta


def _meta(c):
    """Lyrical's rcutils compiles process.c (fork/execvp) unless told not to. newlib
    declares fork as a stub, which is why the PlatformIO builds never needed this;
    Zephyr's POSIX layer has no fork at all."""
    if "RCUTILS_NO_PROCESS_SUPPORT" in c:
        return c
    return c.replace('"-DRCUTILS_NO_THREAD_SUPPORT=ON",',
                     '"-DRCUTILS_NO_THREAD_SUPPORT=ON",\n                "-DRCUTILS_NO_PROCESS_SUPPORT=ON",', 1)


AMENT_TARGET_DEPENDENCIES = """
# Compatibility ament_target_dependencies macro for micro-ROS
macro(ament_target_dependencies target)
  cmake_parse_arguments(_ARG "SYSTEM;INTERFACE;PUBLIC;PRIVATE" "" "" ${ARGN})
  set(_dependencies ${_ARG_UNPARSED_ARGUMENTS})
  foreach(_dep ${_dependencies})
    find_package(${_dep} QUIET)
    if(TARGET ${_dep})
      if(_ARG_INTERFACE)
        target_link_libraries(${target} INTERFACE ${_dep})
      elseif(_ARG_PUBLIC)
        target_link_libraries(${target} PUBLIC ${_dep})
      else()
        target_link_libraries(${target} PRIVATE ${_dep})
      endif()
    elseif(TARGET ${_dep}::${_dep})
      if(_ARG_INTERFACE)
        target_link_libraries(${target} INTERFACE ${_dep}::${_dep})
      elseif(_ARG_PUBLIC)
        target_link_libraries(${target} PUBLIC ${_dep}::${_dep})
      else()
        target_link_libraries(${target} PRIVATE ${_dep}::${_dep})
      endif()
    endif()
    if(${_dep}_INCLUDE_DIRS)
      if(_ARG_INTERFACE)
        target_include_directories(${target} INTERFACE ${${_dep}_INCLUDE_DIRS})
      elseif(_ARG_PUBLIC)
        target_include_directories(${target} PUBLIC ${${_dep}_INCLUDE_DIRS})
      else()
        target_include_directories(${target} PRIVATE ${${_dep}_INCLUDE_DIRS})
      endif()
    endif()
    if(${_dep}_LIBRARIES)
      if(_ARG_INTERFACE)
        target_link_libraries(${target} INTERFACE ${${_dep}_LIBRARIES})
      elseif(_ARG_PUBLIC)
        target_link_libraries(${target} PUBLIC ${${_dep}_LIBRARIES})
      else()
        target_link_libraries(${target} PRIVATE ${${_dep}_LIBRARIES})
      endif()
    endif()
  endforeach()
endmacro()
"""


def patch_dev(src):
    """micro_ros_platformio's patch_dev_sources(), for the host-side ament tools."""
    changed = []
    for p in ("rmw_test_fixture", "rmw_test_fixture_implementation", "domain_coordinator"):
        d = os.path.join(src, "ament_cmake_ros", p)
        if os.path.isdir(d) and not os.path.exists(os.path.join(d, "COLCON_IGNORE")):
            open(os.path.join(d, "COLCON_IGNORE"), "w").close()
            changed.append(f"ignore {p}")
    if _rw(os.path.join(src, "ament_cmake_ros", "ament_cmake_ros_core", "cmake", "ament_ros_defaults.cmake"),
           lambda c: c.replace("cxx_std_20", "cxx_std_17").replace("c_std_17", "c_std_11")):
        changed.append("ament_ros_defaults C++17/C11")
    if _rw(os.path.join(src, "ament_cmake", "ament_cmake_core", "cmake", "core", "all.cmake"),
           lambda c: c if "macro(ament_target_dependencies" in c else c + "\n" + AMENT_TARGET_DEPENDENCIES):
        changed.append("ament_target_dependencies macro")
    return changed


def patch_sources(src):
    changed = []
    for root, dirs, files in os.walk(src):
        if "package.xml" in files:
            name = re.search(r"<name>\s*([^<\s]+)", open(os.path.join(root, "package.xml")).read())
            if name and name.group(1) in IGNORE:
                open(os.path.join(root, "COLCON_IGNORE"), "a").close()
                changed.append(f"ignore {name.group(1)}")
            dirs[:] = []

    # 1. rcutils base64.c: no pthread_once without thread support.
    def base64(bc):
        if "RCUTILS_NO_THREAD_SUPPORT" in bc:
            return bc
        return bc.replace(
            "#ifdef _WIN32\nstatic INIT_ONCE base64_map_initialization_once = INIT_ONCE_STATIC_INIT;",
            "#if defined(RCUTILS_NO_THREAD_SUPPORT)\nstatic bool base64_map_initialized = false;\n"
            "#elif defined(_WIN32)\nstatic INIT_ONCE base64_map_initialization_once = INIT_ONCE_STATIC_INIT;"
        ).replace(
            "#else\n  pthread_once(&base64_map_initialization_once, initialize_base64_map);\n#endif",
            "#elif defined(RCUTILS_NO_THREAD_SUPPORT)\n  if (!base64_map_initialized) { initialize_base64_map(); "
            "base64_map_initialized = true; }\n#else\n  pthread_once(&base64_map_initialization_once, "
            "initialize_base64_map);\n#endif")
    if _rw(os.path.join(src, "rcutils", "src", "base64.c"), base64):
        changed.append("rcutils base64.c")

    # 2. rosidl_buffer: C++17 without ament_ros_cxx_standard, and no exceptions.
    for b in ("rosidl", "rosidl_core"):
        def cmake(s):
            out = []
            for line in s.splitlines(True):
                if "target_link_libraries" in line and ("ament_ros_cxx_standard" in line
                                                       or "PRIVATE )" in line or "PRIVATE  )" in line):
                    out.append("target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)\n")
                else:
                    out.append(line)
            return "".join(out)
        if _rw(os.path.join(src, b, "rosidl_buffer", "CMakeLists.txt"), cmake):
            changed.append(f"{b} rosidl_buffer CMakeLists")

        def hpp(h):
            h = h.replace("if constexpr (std::is_same_v<Allocator, std::allocator<T>>)",
                          "if (std::is_same<Allocator, std::allocator<T>>::value)")
            h = h.replace("std::is_same_v<Allocator, std::allocator<T>>",
                          "std::is_same<Allocator, std::allocator<T>>::value")
            if "__EXCEPTIONS" not in h:
                h = h.replace(
                    '      throw std::invalid_argument("Buffer implementation must not be null");',
                    '#if __EXCEPTIONS || defined(__cpp_exceptions)\n'
                    '      throw std::invalid_argument("Buffer implementation must not be null");\n#endif')
                h = h.replace(
                    "  void throw_if_not_cpu_backend() const\n  {\n    if (!cpu_impl_) {",
                    "  void throw_if_not_cpu_backend() const\n  {\n#if __EXCEPTIONS || defined(__cpp_exceptions)\n"
                    "    if (!cpu_impl_) {")
                h = h.replace(
                    '". Use to_vector() for explicit conversion to CPU.");\n    }\n  }',
                    '". Use to_vector() for explicit conversion to CPU.");\n    }\n#endif\n  }')
            return h
        if _rw(os.path.join(src, b, "rosidl_buffer", "include", "rosidl_buffer", "buffer.hpp"), hpp):
            changed.append(f"{b} rosidl_buffer buffer.hpp")

    # 3. rclc_lifecycle: lyrical's rcl_lifecycle_state_machine_init takes a clock.
    def lifecycle(c):
        if "lifecycle_clock" in c:
            return c
        t = ("  rcl_ret_t rcl_ret = rcl_lifecycle_state_machine_init(\n    state_machine,\n    node,\n"
             "    ROSIDL_GET_MSG_TYPE_SUPPORT(lifecycle_msgs, msg, TransitionEvent),")
        if t not in c:
            return c
        r = ("  static rcl_clock_t lifecycle_clock;\n  static bool lifecycle_clock_initialized = false;\n"
             "  if (!lifecycle_clock_initialized) {\n    rcl_ros_clock_init(&lifecycle_clock, allocator);\n"
             "    lifecycle_clock_initialized = true;\n  }\n\n"
             "  rcl_ret_t rcl_ret = rcl_lifecycle_state_machine_init(\n    state_machine,\n    node,\n"
             "    &lifecycle_clock,\n    ROSIDL_GET_MSG_TYPE_SUPPORT(lifecycle_msgs, msg, TransitionEvent),")
        return "#include <rcl/time.h>\n" + c.replace(t, r)
    if _rw(os.path.join(src, "rclc", "rclc_lifecycle", "src", "rclc_lifecycle", "rclc_lifecycle.c"), lifecycle):
        changed.append("rclc_lifecycle clock")

    # 4. rcutils fault_injection.c: a braced atomic initializer is valid everywhere.
    def fault(c):
        t = "#if defined(_WIN32) && !defined(__MINGW64__)"
        if t not in c:
            return c
        start = c.index(t)
        stale = "// The initializer must match the definition of _Atomic in"
        if stale in c[:start]:
            start = c.index(stale)
        end = c.index("#endif", c.index("= -1;", start)) + len("#endif")
        return c[:start] + "static atomic_int_least64_t g_rcutils_fault_injection_count = {-1};" + c[end:]
    if _rw(os.path.join(src, "rcutils", "src", "testing", "fault_injection.c"), fault):
        changed.append("rcutils fault_injection.c")
    return changed


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--patch-dev":
        for c in patch_dev(sys.argv[2]):
            print(f"lyrical dev: {c}")
    elif len(sys.argv) == 3 and sys.argv[1] == "--patch-sources":
        for c in patch_sources(sys.argv[2]):
            print(f"lyrical: {c}")
    elif len(sys.argv) in (2, 3):
        done = patch_makefile(*sys.argv[1:3])
        print("lyrical: libmicroros.mk " + ("patched" if done else "already patched"))
    else:
        sys.exit(__doc__)
