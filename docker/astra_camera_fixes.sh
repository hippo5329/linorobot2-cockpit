#!/bin/bash
# Makes Orbbec's ros2_astra_camera (last changed 2023-11, written for Humble) build on
# Jazzy and Lyrical -- the changes of its open upstream PR #20, "fix compatibility with
# ROS2 Jazzy and Kilted". cv_bridge, image_geometry and message_filters dropped their .h
# headers for .hpp, and on Lyrical tf2's LinearMath did too (tf2_ros's .hpp names exist on
# both, so those move as well); rclcpp renamed OnParametersSetCallbackType to
# OnSetParametersCallbackType; constants.h uses uint32_t without <cstdint>. Fails when a
# pattern it replaces is still there afterwards, so an upstream move shows up as a broken
# build, not as a driver built from half-patched sources.
#   astra_camera_fixes.sh <ros2_astra_camera checkout>
set -euo pipefail
pkg="$1/astra_camera"
files=$(grep -rlE 'cv_bridge/cv_bridge\.h>|image_geometry/pinhole_camera_model\.h>|message_filters/[a-z_/]+\.h>|tf2/LinearMath/[A-Za-z0-9]+\.h>|tf2_ros/[a-z_]+\.h>|OnParametersSetCallbackType' \
        "$pkg/include" "$pkg/src" || true)
for f in $files; do
  sed -i -E \
    -e 's#<cv_bridge/cv_bridge\.h>#<cv_bridge/cv_bridge.hpp>#' \
    -e 's#<image_geometry/pinhole_camera_model\.h>#<image_geometry/pinhole_camera_model.hpp>#' \
    -e 's#<message_filters/([a-z_/]+)\.h>#<message_filters/\1.hpp>#' \
    -e 's#<tf2/LinearMath/([A-Za-z0-9]+)\.h>#<tf2/LinearMath/\1.hpp>#' \
    -e 's#<tf2_ros/([a-z_]+)\.h>#<tf2_ros/\1.hpp>#' \
    -e 's#OnParametersSetCallbackType#OnSetParametersCallbackType#g' \
    "$f"
done
grep -q '#include <cstdint>' "$pkg/include/astra_camera/constants.h" \
  || sed -i 's|^#pragma once$|#pragma once\n#include <cstdint>|' "$pkg/include/astra_camera/constants.h"
# message_filters::Subscriber::subscribe takes an rmw_qos_profile_t on Jazzy and only an
# rclcpp::QoS after it (Lyrical) -- no one call compiles on both, so past Jazzy the profile is
# wrapped, the same QoS. ROS_DISTRO is the ROS base image's.
xyzrgb="$pkg/src/point_cloud_proc/point_cloud_xyzrgb.cpp"
if [ "${ROS_DISTRO:-jazzy}" != jazzy ]; then
  sed -i -E 's#(sub_info_\.subscribe\(node_, "[^"]+", )([a-z_]+_qos_profile_)\);#\1rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(\2), \2));#' "$xyzrgb"
  if grep -nE 'sub_info_\.subscribe\(node_, "[^"]+", [a-z_]+_qos_profile_\);' "$xyzrgb"; then
    echo "[astra_camera] the subscribe above still passes an rmw profile" >&2; exit 1
  fi
fi
# Lyrical's ament_cmake no longer has ament_target_dependencies (its package keeps only a
# helper). Where the command is missing, link each dependency's exported targets -- with the
# plain target_link_libraries signature the file's own calls use, as CMake will not mix them.
if ! grep -q 'lino: ament_target_dependencies shim' "$pkg/CMakeLists.txt"; then
  shim=$(mktemp)
  cat > "$shim" <<'CMAKE'
if(NOT COMMAND ament_target_dependencies)  # lino: ament_target_dependencies shim (Lyrical)
  function(ament_target_dependencies target)
    foreach(dep IN LISTS ARGN)
      if(DEFINED ${dep}_TARGETS)
        target_link_libraries(${target} ${${dep}_TARGETS})
      else()
        target_include_directories(${target} PUBLIC ${${dep}_INCLUDE_DIRS})
        target_link_libraries(${target} ${${dep}_LIBRARIES})
      endif()
    endforeach()
  endfunction()
endif()
CMAKE
  awk -v shim="$shim" '{print} !done && /^find_package\(ament_cmake REQUIRED\)/ {while ((getline l < shim) > 0) print l; done=1}' \
    "$pkg/CMakeLists.txt" > "$pkg/CMakeLists.txt.new"
  mv "$pkg/CMakeLists.txt.new" "$pkg/CMakeLists.txt"; rm -f "$shim"
  grep -q 'lino: ament_target_dependencies shim' "$pkg/CMakeLists.txt"
fi
if grep -rnE 'cv_bridge/cv_bridge\.h>|image_geometry/pinhole_camera_model\.h>|message_filters/[a-z_/]+\.h>|tf2/LinearMath/[A-Za-z0-9]+\.h>|tf2_ros/[a-z_]+\.h>|OnParametersSetCallbackType' \
     "$pkg/include" "$pkg/src"; then
  echo "[astra_camera] the lines above were not fixed" >&2; exit 1
fi
grep -q '#include <cstdint>' "$pkg/include/astra_camera/constants.h"
echo "[astra_camera] Jazzy/Lyrical fixes applied to $(echo $files | wc -w) files"
