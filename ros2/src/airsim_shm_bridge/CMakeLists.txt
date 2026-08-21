cmake_minimum_required(VERSION 3.12)
project(airsim_shm_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(shm_bridge
  src/shm_bridge_node.cpp
  src/shm_segment.cpp)
target_include_directories(shm_bridge PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(shm_bridge rclcpp sensor_msgs)

# ⚠ The verifier is C++ for the same reason the bridge is: a Python subscriber cannot keep up with
# 1080p at rate, and its drops would produce FALSE FAILURES on the stereo-pairing check, which works
# by intersecting timestamp sets across two topics.
add_executable(shm_verify
  src/verify_node.cpp
  src/shm_segment.cpp)
target_include_directories(shm_verify PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(shm_verify rclcpp sensor_msgs)

install(TARGETS shm_bridge shm_verify DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
