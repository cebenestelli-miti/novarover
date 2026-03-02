cmake_minimum_required(VERSION 3.8)
project(mower_obstacles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mower_msgs REQUIRED)

add_executable(ultrasonic_guard_node src/ultrasonic_guard_node.cpp)
ament_target_dependencies(ultrasonic_guard_node rclcpp std_msgs mower_msgs)

install(TARGETS ultrasonic_guard_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
