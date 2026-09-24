cmake_minimum_required(VERSION 3.8)
project(linorobot2_cockpit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# maps/ is deliberately NOT installed: it is gitignored runtime output, so it
# does not exist in a clean checkout, and install(DIRECTORY) on a missing
# directory is a hard CMake error -- which is how every published image came to
# ship a half-installed overlay (no local_setup.bash, no ldlidar_stl_ros2).
# Nothing reads maps from share/ either; the container mounts a volume on
# /ws/maps and the code resolves it from REPO_ROOT.
install(
  DIRECTORY launchers config scripts web rviz
  DESTINATION share/${PROJECT_NAME}
)

# The firmware headers a HOST script reads.
#
# scripts/gen_bare_config.py parses the simulated world's defaults out of
# sim_wheel.h and sim_ld19.h rather than restating them, and it runs at
# bringup from the INSTALLED copy under share/ -- where there was no firmware
# tree at all, so it found nothing and generated a config with the whole
# simulation block missing. That is not a crash, and that is the problem: the
# firmware then falls back to its compiled defaults, silently, and a config
# somebody reads no longer describes the robot they are running.
#
# Only the headers. The firmware itself travels as a built image, and installing
# the sources would put a second copy of the tree in every container.
install(
  DIRECTORY firmware/common/lib/encoder firmware/common/lib/lidar
  DESTINATION share/${PROJECT_NAME}/firmware/common/lib
  FILES_MATCHING PATTERN "*.h"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
