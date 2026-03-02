cmake_minimum_required(VERSION 3.8)
project(mower_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mower_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(mission_manager_node src/mission_manager_node.cpp)
ament_target_dependencies(mission_manager_node rclcpp mower_msgs std_srvs)

add_executable(waypoint_follower_node src/waypoint_follower_node.cpp)
ament_target_dependencies(waypoint_follower_node rclcpp mower_msgs std_srvs geometry_msgs nav_msgs)

install(TARGETS mission_manager_node waypoint_follower_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
