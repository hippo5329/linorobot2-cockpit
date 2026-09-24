cmake_minimum_required(VERSION 3.16)
project(lino_host_probe C)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(rmw_microxrcedds REQUIRED)
find_package(microxrcedds_client REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(lino_host_probe main.c)
ament_target_dependencies(lino_host_probe rcl rclc rmw_microxrcedds microxrcedds_client std_msgs)
install(TARGETS lino_host_probe DESTINATION lib/${PROJECT_NAME})
ament_package()
