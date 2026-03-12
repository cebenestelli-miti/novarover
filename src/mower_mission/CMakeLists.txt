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
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(mission_loader SHARED src/mission_loader.cpp)
ament_target_dependencies(mission_loader rclcpp)
target_include_directories(mission_loader PUBLIC include)

add_executable(mission_manager_node src/mission_manager_node.cpp)
target_link_libraries(mission_manager_node mission_loader)
ament_target_dependencies(mission_manager_node rclcpp mower_msgs std_srvs nav_msgs)

add_executable(waypoint_follower_node src/waypoint_follower_node.cpp)
target_link_libraries(waypoint_follower_node mission_loader)
target_include_directories(waypoint_follower_node PRIVATE include)
ament_target_dependencies(waypoint_follower_node
  rclcpp
  mower_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS mission_loader mission_manager_node waypoint_follower_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include
  USE_SOURCE_PERMISSIONS)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  USE_SOURCE_PERMISSIONS)
install(DIRECTORY config/missions/
  DESTINATION share/${PROJECT_NAME}/missions
  USE_SOURCE_PERMISSIONS)

ament_package()
