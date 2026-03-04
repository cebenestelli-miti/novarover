cmake_minimum_required(VERSION 3.8)
project(mower_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install share resources for Gazebo and launch
install(DIRECTORY launch config models worlds
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
install(PROGRAMS scripts/sim_helpers_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
