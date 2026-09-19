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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
